/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "etnaviv_drv.h"
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"

/*
 * Cmdstream submission:
 */

#define BO_INVALID_FLAGS ~(ETNA_SUBMIT_BO_READ | ETNA_SUBMIT_BO_WRITE)
/* make sure these don't conflict w/ MSM_SUBMIT_BO_x */
#define BO_VALID    0x8000
#define BO_LOCKED   0x4000
#define BO_PINNED   0x2000

static inline void __user *to_user_ptr(u64 address)
{
	return (void __user *)(uintptr_t)address;
}

static struct etnaviv_gem_submit *submit_create(struct drm_device *dev,
		struct etnaviv_gpu *gpu, int nr)
{
	struct etnaviv_gem_submit *submit;
	int sz = sizeof(*submit) + (nr * sizeof(submit->bos[0]));

	submit = kmalloc(sz, GFP_TEMPORARY | __GFP_NOWARN | __GFP_NORETRY);
	if (submit) {
		submit->dev = dev;
		submit->gpu = gpu;

		/* initially, until copy_from_user() and bo lookup succeeds: */
		submit->nr_bos = 0;
		submit->nr_cmds = 0;

		INIT_LIST_HEAD(&submit->bo_list);
		ww_acquire_init(&submit->ticket, &reservation_ww_class);
	}

	return submit;
}

static int submit_lookup_objects(struct etnaviv_gem_submit *submit,
		struct drm_etnaviv_gem_submit *args, struct drm_file *file)
{
	unsigned i;
	int ret = 0;

	spin_lock(&file->table_lock);

	for (i = 0; i < args->nr_bos; i++) {
		struct drm_etnaviv_gem_submit_bo submit_bo;
		struct drm_gem_object *obj;
		struct etnaviv_gem_object *etnaviv_obj;
		void __user *userptr =
			to_user_ptr(args->bos + (i * sizeof(submit_bo)));

		ret = copy_from_user(&submit_bo, userptr, sizeof(submit_bo));
		if (ret) {
			ret = -EFAULT;
			goto out_unlock;
		}

		if (submit_bo.flags & BO_INVALID_FLAGS) {
			DRM_ERROR("invalid flags: %x\n", submit_bo.flags);
			ret = -EINVAL;
			goto out_unlock;
		}

		submit->bos[i].flags = submit_bo.flags;
		/* in validate_objects() we figure out if this is true: */
		submit->bos[i].iova  = submit_bo.presumed;

		/* normally use drm_gem_object_lookup(), but for bulk lookup
		 * all under single table_lock just hit object_idr directly:
		 */
		obj = idr_find(&file->object_idr, submit_bo.handle);
		if (!obj) {
			DRM_ERROR("invalid handle %u at index %u\n", submit_bo.handle, i);
			ret = -EINVAL;
			goto out_unlock;
		}

		etnaviv_obj = to_etnaviv_bo(obj);

		if (!list_empty(&etnaviv_obj->submit_entry)) {
			DRM_ERROR("handle %u at index %u already on submit list\n",
					submit_bo.handle, i);
			ret = -EINVAL;
			goto out_unlock;
		}

		drm_gem_object_reference(obj);

		submit->bos[i].obj = etnaviv_obj;

		list_add_tail(&etnaviv_obj->submit_entry, &submit->bo_list);
	}

out_unlock:
	submit->nr_bos = i;
	spin_unlock(&file->table_lock);

	return ret;
}

static void submit_unlock_unpin_bo(struct etnaviv_gem_submit *submit, int i)
{
	struct etnaviv_gem_object *etnaviv_obj = submit->bos[i].obj;

	if (submit->bos[i].flags & BO_PINNED)
		etnaviv_gem_put_iova(&etnaviv_obj->base);

	if (submit->bos[i].flags & BO_LOCKED)
		ww_mutex_unlock(&etnaviv_obj->resv->lock);

	if (!(submit->bos[i].flags & BO_VALID))
		submit->bos[i].iova = 0;

	submit->bos[i].flags &= ~(BO_LOCKED | BO_PINNED);
}

/* This is where we make sure all the bo's are reserved and pin'd: */
static int submit_validate_objects(struct etnaviv_gem_submit *submit)
{
	int contended, slow_locked = -1, i, ret = 0;

retry:
	submit->valid = true;

	for (i = 0; i < submit->nr_bos; i++) {
		struct etnaviv_gem_object *etnaviv_obj = submit->bos[i].obj;
		uint32_t iova;

		if (slow_locked == i)
			slow_locked = -1;

		contended = i;

		if (!(submit->bos[i].flags & BO_LOCKED)) {
			ret = ww_mutex_lock_interruptible(&etnaviv_obj->resv->lock,
					&submit->ticket);
			if (ret)
				goto fail;
			submit->bos[i].flags |= BO_LOCKED;
		}


		/* if locking succeeded, pin bo: */
		ret = etnaviv_gem_get_iova_locked(submit->gpu, &etnaviv_obj->base, &iova);

		/* this would break the logic in the fail path.. there is no
		 * reason for this to happen, but just to be on the safe side
		 * let's notice if this starts happening in the future:
		 */
		WARN_ON(ret == -EDEADLK);

		if (ret)
			goto fail;

		submit->bos[i].flags |= BO_PINNED;

		if (iova == submit->bos[i].iova) {
			submit->bos[i].flags |= BO_VALID;
		} else {
			submit->bos[i].iova = iova;
			submit->bos[i].flags &= ~BO_VALID;
			submit->valid = false;
		}
	}

	ww_acquire_done(&submit->ticket);

	return 0;

fail:
	for (; i >= 0; i--)
		submit_unlock_unpin_bo(submit, i);

	if (slow_locked > 0)
		submit_unlock_unpin_bo(submit, slow_locked);

	if (ret == -EDEADLK) {
		struct etnaviv_gem_object *etnaviv_obj = submit->bos[contended].obj;
		/* we lost out in a seqno race, lock and retry.. */
		ret = ww_mutex_lock_slow_interruptible(&etnaviv_obj->resv->lock,
				&submit->ticket);
		if (!ret) {
			submit->bos[contended].flags |= BO_LOCKED;
			slow_locked = contended;
			goto retry;
		}
	}

	return ret;
}

static int submit_bo(struct etnaviv_gem_submit *submit, uint32_t idx,
		struct etnaviv_gem_object **obj, uint32_t *iova, bool *valid)
{
	if (idx >= submit->nr_bos) {
		DRM_ERROR("invalid buffer index: %u (out of %u)\n",
				idx, submit->nr_bos);
		return -EINVAL;
	}

	if (obj)
		*obj = submit->bos[idx].obj;
	if (iova)
		*iova = submit->bos[idx].iova;
	if (valid)
		*valid = !!(submit->bos[idx].flags & BO_VALID);

	return 0;
}

/* process the reloc's and patch up the cmdstream as needed: */
static int submit_reloc(struct etnaviv_gem_submit *submit, struct etnaviv_gem_object *obj,
		uint32_t offset, uint32_t nr_relocs, uint64_t relocs)
{
	uint32_t i, last_offset = 0;
	uint32_t *ptr = obj->vaddr;
	int ret;

	if (offset % 4) {
		DRM_ERROR("non-aligned cmdstream buffer: %u\n", offset);
		return -EINVAL;
	}

	for (i = 0; i < nr_relocs; i++) {
		struct drm_etnaviv_gem_submit_reloc submit_reloc;
		void __user *userptr =
			to_user_ptr(relocs + (i * sizeof(submit_reloc)));
		uint32_t iova, off;
		bool valid;

		ret = copy_from_user(&submit_reloc, userptr, sizeof(submit_reloc));
		if (ret)
			return -EFAULT;

		if (submit_reloc.submit_offset % 4) {
			DRM_ERROR("non-aligned reloc offset: %u\n",
					submit_reloc.submit_offset);
			return -EINVAL;
		}

		/* offset in dwords: */
		off = submit_reloc.submit_offset / 4;

		if ((off >= (obj->base.size / 4)) ||
				(off < last_offset)) {
			DRM_ERROR("invalid offset %u at reloc %u\n", off, i);
			return -EINVAL;
		}

		ret = submit_bo(submit, submit_reloc.reloc_idx, NULL, &iova, &valid);
		if (ret)
			return ret;

		if (valid)
			continue;

		iova += submit_reloc.reloc_offset;

		if (submit_reloc.shift < 0)
			iova >>= -submit_reloc.shift;
		else
			iova <<= submit_reloc.shift;

		ptr[off] = iova | submit_reloc.or;

		last_offset = off;
	}

	return 0;
}

static void submit_cleanup(struct etnaviv_gem_submit *submit, bool fail)
{
	unsigned i;

	for (i = 0; i < submit->nr_bos; i++) {
		struct etnaviv_gem_object *etnaviv_obj = submit->bos[i].obj;
		submit_unlock_unpin_bo(submit, i);
		list_del_init(&etnaviv_obj->submit_entry);
		drm_gem_object_unreference(&etnaviv_obj->base);
	}

	ww_acquire_fini(&submit->ticket);
	kfree(submit);
}

int etnaviv_ioctl_gem_submit(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct drm_etnaviv_gem_submit *args = data;
	struct etnaviv_file_private *ctx = file->driver_priv;
	struct etnaviv_gem_submit *submit;
	struct etnaviv_gpu *gpu;
	unsigned i;
	int ret;

	if (args->pipe >= ETNA_MAX_PIPES)
		return -EINVAL;

	gpu = priv->gpu[args->pipe];
	if (!gpu)
		return -ENXIO;

	if (args->nr_cmds > MAX_CMDS)
		return -EINVAL;

	mutex_lock(&dev->struct_mutex);

	submit = submit_create(dev, gpu, args->nr_bos);
	if (!submit) {
		ret = -ENOMEM;
		goto out;
	}

	ret = submit_lookup_objects(submit, args, file);
	if (ret)
		goto out;

	ret = submit_validate_objects(submit);
	if (ret)
		goto out;

	for (i = 0; i < args->nr_cmds; i++) {
		struct drm_etnaviv_gem_submit_cmd submit_cmd;
		void __user *userptr =
			to_user_ptr(args->cmds + (i * sizeof(submit_cmd)));
		struct etnaviv_gem_object *etnaviv_obj;

		ret = copy_from_user(&submit_cmd, userptr, sizeof(submit_cmd));
		if (ret) {
			ret = -EFAULT;
			goto out;
		}

		ret = submit_bo(submit, submit_cmd.submit_idx,
				&etnaviv_obj, NULL, NULL);
		if (ret)
			goto out;

		if (!(etnaviv_obj->flags & ETNA_BO_CMDSTREAM)) {
			DRM_ERROR("cmdstream bo has flag ETNA_BO_CMDSTREAM not set\n");
			ret = -EINVAL;
			goto out;
		}

		if (submit_cmd.size % 4) {
			DRM_ERROR("non-aligned cmdstream buffer size: %u\n",
					submit_cmd.size);
			ret = -EINVAL;
			goto out;
		}

		if ((submit_cmd.size + submit_cmd.submit_offset) >=
				etnaviv_obj->base.size) {
			DRM_ERROR("invalid cmdstream size: %u\n", submit_cmd.size);
			ret = -EINVAL;
			goto out;
		}

		submit->cmd[i].type = submit_cmd.type;
		submit->cmd[i].size = submit_cmd.size / 4;
		submit->cmd[i].obj = etnaviv_obj;

		if (submit->valid)
			continue;

		ret = submit_reloc(submit, etnaviv_obj, submit_cmd.submit_offset,
				submit_cmd.nr_relocs, submit_cmd.relocs);
		if (ret)
			goto out;
	}

	submit->nr_cmds = i;

	ret = etnaviv_gpu_submit(gpu, submit, ctx);

	args->fence = submit->fence;

out:
	if (submit)
		submit_cleanup(submit, !!ret);
	mutex_unlock(&dev->struct_mutex);
	return ret;
}
