/*
 * Copyright (C) 2015 Etnaviv Project
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

#include <linux/spinlock.h>
#include <linux/shmem_fs.h>

#include "etnaviv_drv.h"
#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"
#include "etnaviv_mmu.h"

static void etnaviv_gem_scatter_map(struct etnaviv_gem_object *etnaviv_obj)
{
	struct drm_device *dev = etnaviv_obj->base.dev;
	struct sg_table *sgt = etnaviv_obj->sgt;

	/*
	 * For non-cached buffers, ensure the new pages are clean
	 * because display controller, GPU, etc. are not coherent.
	 */
	if (etnaviv_obj->flags & (ETNA_BO_WC|ETNA_BO_CACHED)) {
		dma_map_sg(dev->dev, sgt->sgl, sgt->nents, DMA_BIDIRECTIONAL);
		dma_unmap_sg(dev->dev, sgt->sgl, sgt->nents, DMA_BIDIRECTIONAL);
	} else {
		struct scatterlist *sg;
		unsigned int i;

		for_each_sg(sgt->sgl, sg, sgt->nents, i) {
			sg_dma_address(sg) = sg_phys(sg);
#ifdef CONFIG_NEED_SG_DMA_LENGTH
			sg_dma_len(sg) = sg->length;
#endif
		}
	}
}

static void etnaviv_gem_scatterlist_unmap(struct etnaviv_gem_object *etnaviv_obj)
{
	struct drm_device *dev = etnaviv_obj->base.dev;
	struct sg_table *sgt = etnaviv_obj->sgt;

	/*
	 * For non-cached buffers, ensure the new pages are clean
	 * because display controller, GPU, etc. are not coherent:
	 *
	 * WARNING: The DMA API does not support concurrent CPU
	 * and device access to the memory area.  With BIDIRECTIONAL,
	 * we will clean the cache lines which overlap the region,
	 * and invalidate all cache lines (partially) contained in
	 * the region.
	 *
	 * If you have dirty data in the overlapping cache lines,
	 * that will corrupt the GPU-written data.  If you have
	 * written into the remainder of the region, this can
	 * discard those writes.
	 */
	if (etnaviv_obj->flags & (ETNA_BO_WC|ETNA_BO_CACHED)) {
		dma_map_sg(dev->dev, sgt->sgl, sgt->nents, DMA_BIDIRECTIONAL);
		dma_unmap_sg(dev->dev, sgt->sgl, sgt->nents, DMA_BIDIRECTIONAL);
	}
}

/* called with dev->struct_mutex held */
static int etnaviv_gem_shmem_get_pages(struct etnaviv_gem_object *etnaviv_obj)
{
	struct drm_device *dev = etnaviv_obj->base.dev;
	struct page **p = drm_gem_get_pages(&etnaviv_obj->base);

	if (IS_ERR(p)) {
		dev_err(dev->dev, "could not get pages: %ld\n", PTR_ERR(p));
		return PTR_ERR(p);
	}

	etnaviv_obj->pages = p;

	return 0;
}

static void put_pages(struct etnaviv_gem_object *etnaviv_obj)
{
	if (etnaviv_obj->sgt) {
		etnaviv_gem_scatterlist_unmap(etnaviv_obj);
		sg_free_table(etnaviv_obj->sgt);
		kfree(etnaviv_obj->sgt);
		etnaviv_obj->sgt = NULL;
	}
	if (etnaviv_obj->pages) {
		drm_gem_put_pages(&etnaviv_obj->base, etnaviv_obj->pages,
				  true, false);

		etnaviv_obj->pages = NULL;
	}
}

struct page **etnaviv_gem_get_pages(struct etnaviv_gem_object *etnaviv_obj)
{
	int ret;

	if (!etnaviv_obj->pages) {
		ret = etnaviv_obj->ops->get_pages(etnaviv_obj);
		if (ret < 0)
			return ERR_PTR(ret);
	}

	if (!etnaviv_obj->sgt) {
		struct drm_device *dev = etnaviv_obj->base.dev;
		int npages = etnaviv_obj->base.size >> PAGE_SHIFT;
		struct sg_table *sgt;

		sgt = drm_prime_pages_to_sg(etnaviv_obj->pages, npages);
		if (IS_ERR(sgt)) {
			dev_err(dev->dev, "failed to allocate sgt: %ld\n",
				PTR_ERR(sgt));
			return ERR_CAST(sgt);
		}

		etnaviv_obj->sgt = sgt;

		etnaviv_gem_scatter_map(etnaviv_obj);
	}

	return etnaviv_obj->pages;
}

void etnaviv_gem_put_pages(struct etnaviv_gem_object *etnaviv_obj)
{
	/* when we start tracking the pin count, then do something here */
}

static int etnaviv_gem_mmap_cmd(struct drm_gem_object *obj,
	struct vm_area_struct *vma)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	int ret;

	/*
	 * Clear the VM_PFNMAP flag that was set by drm_gem_mmap(), and set the
	 * vm_pgoff (used as a fake buffer offset by DRM) to 0 as we want to map
	 * the whole buffer.
	 */
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;

	ret = dma_mmap_coherent(obj->dev->dev, vma,
				etnaviv_obj->vaddr, etnaviv_obj->paddr,
				vma->vm_end - vma->vm_start);

	return ret;
}

static int etnaviv_gem_mmap_obj(struct drm_gem_object *obj,
		struct vm_area_struct *vma)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	pgprot_t vm_page_prot;

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;

	vm_page_prot = vm_get_page_prot(vma->vm_flags);

	if (etnaviv_obj->flags & ETNA_BO_WC) {
		vma->vm_page_prot = pgprot_writecombine(vm_page_prot);
	} else if (etnaviv_obj->flags & ETNA_BO_UNCACHED) {
		vma->vm_page_prot = pgprot_noncached(vm_page_prot);
	} else {
		/*
		 * Shunt off cached objs to shmem file so they have their own
		 * address_space (so unmap_mapping_range does what we want,
		 * in particular in the case of mmap'd dmabufs)
		 */
		fput(vma->vm_file);
		get_file(obj->filp);
		vma->vm_pgoff = 0;
		vma->vm_file  = obj->filp;

		vma->vm_page_prot = vm_page_prot;
	}

	return 0;
}

int etnaviv_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct etnaviv_gem_object *obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret) {
		DBG("mmap failed: %d", ret);
		return ret;
	}

	obj = to_etnaviv_bo(vma->vm_private_data);
	if (obj->flags & ETNA_BO_CMDSTREAM)
		ret = etnaviv_gem_mmap_cmd(vma->vm_private_data, vma);
	else
		ret = etnaviv_gem_mmap_obj(vma->vm_private_data, vma);

	return ret;
}

int etnaviv_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct drm_device *dev = obj->dev;
	struct page **pages, *page;
	pgoff_t pgoff;
	int ret;

	/*
	 * Make sure we don't parallel update on a fault, nor move or remove
	 * something from beneath our feet.  Note that vm_insert_page() is
	 * specifically coded to take care of this, so we don't have to.
	 */
	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		goto out;

	/* make sure we have pages attached now */
	pages = etnaviv_gem_get_pages(to_etnaviv_bo(obj));
	mutex_unlock(&dev->struct_mutex);

	if (IS_ERR(pages)) {
		ret = PTR_ERR(pages);
		goto out;
	}

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = ((unsigned long)vmf->virtual_address -
			vma->vm_start) >> PAGE_SHIFT;

	page = pages[pgoff];

	VERB("Inserting %p pfn %lx, pa %lx", vmf->virtual_address,
	     page_to_pfn(page), page_to_pfn(page) << PAGE_SHIFT);

	ret = vm_insert_page(vma, (unsigned long)vmf->virtual_address, page);

out:
	switch (ret) {
	case -EAGAIN:
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
	case -EBUSY:
		/*
		 * EBUSY is ok: this just means that another thread
		 * already did the job.
		 */
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

/* get mmap offset - must be called under struct_mutex */
int etnaviv_gem_mmap_offset(struct drm_gem_object *obj, u64 *offset)
{
	int ret;

	/* Make it mmapable */
	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		dev_err(obj->dev->dev, "could not allocate mmap offset\n");
	else
		*offset = drm_vma_node_offset_addr(&obj->vma_node);

	return ret;
}

/* should be called under struct_mutex.. although it can be called
 * from atomic context without struct_mutex to acquire an extra
 * iova ref if you know one is already held.
 *
 * That means when I do eventually need to add support for unpinning
 * the refcnt counter needs to be atomic_t.
 */
int etnaviv_gem_get_iova_locked(struct etnaviv_gpu *gpu,
	struct drm_gem_object *obj, u32 *iova)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	struct etnaviv_vram_mapping *mapping;
	int ret = 0;

	if (etnaviv_obj->flags & ETNA_BO_CMDSTREAM) {
		*iova = etnaviv_obj->paddr;
		return 0;
	}

	mapping = etnaviv_gem_get_vram_mapping(etnaviv_obj, gpu->mmu);
	if (!mapping) {
		struct page **pages = etnaviv_gem_get_pages(etnaviv_obj);
		if (IS_ERR(pages))
			return PTR_ERR(pages);
		ret = etnaviv_iommu_map_gem(gpu->mmu, etnaviv_obj,
				gpu->memory_base, &mapping);
	}

	if (!ret)
		*iova = mapping->iova;

	return ret;
}

int etnaviv_gem_get_iova(struct etnaviv_gpu *gpu, struct drm_gem_object *obj,
	int id, u32 *iova)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	struct etnaviv_vram_mapping *mapping =
			etnaviv_gem_get_vram_mapping(etnaviv_obj, gpu->mmu);
	int ret;

	/* this is safe right now because we don't unmap until the
	 * bo is deleted:
	 */
	if (mapping) {
		*iova = mapping->iova;
		return 0;
	}

	mutex_lock(&obj->dev->struct_mutex);
	ret = etnaviv_gem_get_iova_locked(gpu, obj, iova);
	mutex_unlock(&obj->dev->struct_mutex);

	return ret;
}

void etnaviv_gem_put_iova(struct drm_gem_object *obj)
{
	/*
	 * XXX TODO ..
	 * NOTE: probably don't need a _locked() version.. we wouldn't
	 * normally unmap here, but instead just mark that it could be
	 * unmapped (if the iova refcnt drops to zero), but then later
	 * if another _get_iova_locked() fails we can start unmapping
	 * things that are no longer needed..
	 */
}

void *etnaviv_gem_vaddr_locked(struct drm_gem_object *obj)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);

	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));

	if (!etnaviv_obj->vaddr) {
		struct page **pages = etnaviv_gem_get_pages(etnaviv_obj);

		if (IS_ERR(pages))
			return ERR_CAST(pages);

		etnaviv_obj->vaddr = vmap(pages, obj->size >> PAGE_SHIFT,
				VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	}

	return etnaviv_obj->vaddr;
}

void *etnaviv_gem_vaddr(struct drm_gem_object *obj)
{
	void *ret;

	mutex_lock(&obj->dev->struct_mutex);
	ret = etnaviv_gem_vaddr_locked(obj);
	mutex_unlock(&obj->dev->struct_mutex);

	return ret;
}

dma_addr_t etnaviv_gem_paddr_locked(struct drm_gem_object *obj)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);

	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));

	return etnaviv_obj->paddr;
}

void etnaviv_gem_move_to_active(struct drm_gem_object *obj,
	struct etnaviv_gpu *gpu, u32 access, u32 fence)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);

	etnaviv_obj->gpu = gpu;

	if (access & ETNA_SUBMIT_BO_READ)
		etnaviv_obj->read_fence = fence;
	if (access & ETNA_SUBMIT_BO_WRITE)
		etnaviv_obj->write_fence = fence;

	etnaviv_obj->access |= access;

	list_del_init(&etnaviv_obj->mm_list);
	list_add_tail(&etnaviv_obj->mm_list, &gpu->active_list);
}

void etnaviv_gem_move_to_inactive(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	etnaviv_obj->gpu = NULL;
	etnaviv_obj->read_fence = 0;
	etnaviv_obj->write_fence = 0;
	etnaviv_obj->access = 0;
	list_del_init(&etnaviv_obj->mm_list);
	list_add_tail(&etnaviv_obj->mm_list, &priv->inactive_list);
}

int etnaviv_gem_cpu_prep(struct drm_gem_object *obj, u32 op,
		struct timespec *timeout)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	int ret = 0;

	if (is_active(etnaviv_obj)) {
		struct etnaviv_gpu *gpu = etnaviv_obj->gpu;
		u32 fence = 0;

		if (op & ETNA_PREP_READ)
			fence = etnaviv_obj->write_fence;
		if (op & ETNA_PREP_WRITE)
			fence = max(fence, etnaviv_obj->read_fence);
		if (op & ETNA_PREP_NOSYNC)
			timeout = NULL;

		ret = etnaviv_gpu_wait_fence_interruptible(gpu, fence, timeout);
	}

	/* TODO cache maintenance */

	return ret;
}

int etnaviv_gem_cpu_fini(struct drm_gem_object *obj)
{
	/* TODO cache maintenance */
	return 0;
}

int etnaviv_gem_wait_bo(struct etnaviv_gpu *gpu, struct drm_gem_object *obj,
	struct timespec *timeout)
{
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);

	return etnaviv_gpu_wait_obj_inactive(gpu, etnaviv_obj, timeout);
}

#ifdef CONFIG_DEBUG_FS
static void etnaviv_gem_describe(struct drm_gem_object *obj, struct seq_file *m)
{
	struct drm_device *dev = obj->dev;
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	unsigned long off = drm_vma_node_start(&obj->vma_node);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	seq_printf(m, "%08x: %c(r=%u,w=%u) %2d (%2d) %08lx %p %zd\n",
			etnaviv_obj->flags, is_active(etnaviv_obj) ? 'A' : 'I',
			etnaviv_obj->read_fence, etnaviv_obj->write_fence,
			obj->name, obj->refcount.refcount.counter,
			off, etnaviv_obj->vaddr, obj->size);
}

void etnaviv_gem_describe_objects(struct list_head *list, struct seq_file *m)
{
	struct etnaviv_gem_object *etnaviv_obj;
	int count = 0;
	size_t size = 0;

	list_for_each_entry(etnaviv_obj, list, mm_list) {
		struct drm_gem_object *obj = &etnaviv_obj->base;

		seq_puts(m, "   ");
		etnaviv_gem_describe(obj, m);
		count++;
		size += obj->size;
	}

	seq_printf(m, "Total %d objects, %zu bytes\n", count, size);
}
#endif

static void etnaviv_gem_cmd_release(struct etnaviv_gem_object *etnaviv_obj)
{
	dma_free_coherent(etnaviv_obj->base.dev->dev, etnaviv_obj->base.size,
		etnaviv_obj->vaddr, etnaviv_obj->paddr);
}

static const struct etnaviv_gem_ops etnaviv_gem_cmd_ops = {
	.release = etnaviv_gem_cmd_release,
};

static void etnaviv_gem_shmem_release(struct etnaviv_gem_object *etnaviv_obj)
{
	if (etnaviv_obj->vaddr)
		vunmap(etnaviv_obj->vaddr);
	put_pages(etnaviv_obj);
}

static const struct etnaviv_gem_ops etnaviv_gem_shmem_ops = {
	.get_pages = etnaviv_gem_shmem_get_pages,
	.release = etnaviv_gem_shmem_release,
};

void etnaviv_gem_free_object(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	struct etnaviv_vram_mapping *mapping, *tmp;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	/* object should not be on active list: */
	WARN_ON(is_active(etnaviv_obj));

	list_del(&etnaviv_obj->mm_list);

	list_for_each_entry_safe(mapping, tmp, &etnaviv_obj->vram_list,
				 obj_node)
		etnaviv_iommu_unmap_gem(mapping);

	drm_gem_free_mmap_offset(obj);
	etnaviv_obj->ops->release(etnaviv_obj);
	reservation_object_fini(&etnaviv_obj->_resv);
	drm_gem_object_release(obj);

	kfree(etnaviv_obj);
}

int etnaviv_gem_obj_add(struct drm_device *dev, struct drm_gem_object *obj)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_gem_object *etnaviv_obj = to_etnaviv_bo(obj);
	int ret;

	ret = mutex_lock_killable(&dev->struct_mutex);
	if (ret)
		return ret;

	list_add_tail(&etnaviv_obj->mm_list, &priv->inactive_list);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int etnaviv_gem_new_impl(struct drm_device *dev,
		u32 size, u32 flags,
		struct drm_gem_object **obj)
{
	struct etnaviv_gem_object *etnaviv_obj;
	unsigned sz = sizeof(*etnaviv_obj);
	bool valid = true;

	/* validate flags */
	if (flags & ETNA_BO_CMDSTREAM) {
		if ((flags & ETNA_BO_CACHE_MASK) != 0)
			valid = false;
	} else {
		switch (flags & ETNA_BO_CACHE_MASK) {
		case ETNA_BO_UNCACHED:
		case ETNA_BO_CACHED:
		case ETNA_BO_WC:
			break;
		default:
			valid = false;
		}
	}

	if (!valid) {
		dev_err(dev->dev, "invalid cache flag: %x (cmd: %d)\n",
				(flags & ETNA_BO_CACHE_MASK),
				(flags & ETNA_BO_CMDSTREAM));
		return -EINVAL;
	}

	etnaviv_obj = kzalloc(sz, GFP_KERNEL);
	if (!etnaviv_obj)
		return -ENOMEM;

	if (flags & ETNA_BO_CMDSTREAM) {
		etnaviv_obj->vaddr = dma_alloc_coherent(dev->dev, size,
				&etnaviv_obj->paddr, GFP_KERNEL);

		if (!etnaviv_obj->vaddr) {
			kfree(etnaviv_obj);
			return -ENOMEM;
		}
	}

	etnaviv_obj->flags = flags;

	etnaviv_obj->resv = &etnaviv_obj->_resv;
	reservation_object_init(&etnaviv_obj->_resv);

	INIT_LIST_HEAD(&etnaviv_obj->submit_entry);
	INIT_LIST_HEAD(&etnaviv_obj->mm_list);
	INIT_LIST_HEAD(&etnaviv_obj->vram_list);

	*obj = &etnaviv_obj->base;

	return 0;
}

static struct drm_gem_object *__etnaviv_gem_new(struct drm_device *dev,
		u32 size, u32 flags)
{
	struct drm_gem_object *obj = NULL;
	int ret;

	size = PAGE_ALIGN(size);

	ret = etnaviv_gem_new_impl(dev, size, flags, &obj);
	if (ret)
		goto fail;

	ret = 0;
	if (flags & ETNA_BO_CMDSTREAM) {
		to_etnaviv_bo(obj)->ops = &etnaviv_gem_cmd_ops;
		drm_gem_private_object_init(dev, obj, size);
	} else {
		to_etnaviv_bo(obj)->ops = &etnaviv_gem_shmem_ops;
		ret = drm_gem_object_init(dev, obj, size);
	}

	if (ret)
		goto fail;

	return obj;

fail:
	if (obj)
		drm_gem_object_unreference_unlocked(obj);

	return ERR_PTR(ret);
}

/* convenience method to construct a GEM buffer object, and userspace handle */
int etnaviv_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		u32 size, u32 flags, u32 *handle)
{
	struct drm_gem_object *obj;
	int ret;

	obj = __etnaviv_gem_new(dev, size, flags);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	ret = etnaviv_gem_obj_add(dev, obj);
	if (ret < 0) {
		drm_gem_object_unreference_unlocked(obj);
		return ret;
	}

	ret = drm_gem_handle_create(file, obj, handle);

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

struct drm_gem_object *etnaviv_gem_new(struct drm_device *dev,
		u32 size, u32 flags)
{
	struct drm_gem_object *obj;
	int ret;

	obj = __etnaviv_gem_new(dev, size, flags);
	if (IS_ERR(obj))
		return obj;

	ret = etnaviv_gem_obj_add(dev, obj);
	if (ret < 0) {
		drm_gem_object_unreference_unlocked(obj);
		return ERR_PTR(ret);
	}

	return obj;
}

int etnaviv_gem_new_private(struct drm_device *dev, size_t size, u32 flags,
	struct etnaviv_gem_object **res)
{
	struct drm_gem_object *obj;
	int ret;

	ret = etnaviv_gem_new_impl(dev, size, flags, &obj);
	if (ret)
		return ret;

	drm_gem_private_object_init(dev, obj, size);

	*res = to_etnaviv_bo(obj);

	return 0;
}

struct etnaviv_vram_mapping *
etnaviv_gem_get_vram_mapping(struct etnaviv_gem_object *obj,
			     struct etnaviv_iommu *mmu)
{
	struct etnaviv_vram_mapping *mapping;

	list_for_each_entry(mapping, &obj->vram_list, obj_node) {
		if (mapping->mmu == mmu)
			return mapping;
	}

	return NULL;
}

struct get_pages_work {
	struct work_struct work;
	struct mm_struct *mm;
	struct task_struct *task;
	struct etnaviv_gem_object *etnaviv_obj;
};

static struct page **etnaviv_gem_userptr_do_get_pages(
	struct etnaviv_gem_object *etnaviv_obj, struct mm_struct *mm, struct task_struct *task)
{
	int ret, pinned, npages = etnaviv_obj->base.size >> PAGE_SHIFT;
	struct page **pvec;
	uintptr_t ptr;

	pvec = drm_malloc_ab(npages, sizeof(struct page *));
	if (!pvec)
		return ERR_PTR(-ENOMEM);

	pinned = 0;
	ptr = etnaviv_obj->userptr.ptr;

	down_read(&mm->mmap_sem);
	while (pinned < npages) {
		ret = get_user_pages(task, mm, ptr, npages - pinned,
				     !etnaviv_obj->userptr.ro, 0,
				     pvec + pinned, NULL);
		if (ret < 0)
			break;

		ptr += ret * PAGE_SIZE;
		pinned += ret;
	}
	up_read(&mm->mmap_sem);

	if (ret < 0) {
		release_pages(pvec, pinned, 0);
		drm_free_large(pvec);
		return ERR_PTR(ret);
	}

	return pvec;
}

static void __etnaviv_gem_userptr_get_pages(struct work_struct *_work)
{
	struct get_pages_work *work = container_of(_work, typeof(*work), work);
	struct etnaviv_gem_object *etnaviv_obj = work->etnaviv_obj;
	struct drm_device *dev = etnaviv_obj->base.dev;
	struct page **pvec;

	pvec = etnaviv_gem_userptr_do_get_pages(etnaviv_obj, work->mm, work->task);

	mutex_lock(&dev->struct_mutex);
	if (IS_ERR(pvec)) {
		etnaviv_obj->userptr.work = ERR_CAST(pvec);
	} else {
		etnaviv_obj->userptr.work = NULL;
		etnaviv_obj->pages = pvec;
	}

	drm_gem_object_unreference(&etnaviv_obj->base);
	mutex_unlock(&dev->struct_mutex);

	mmput(work->mm);
	put_task_struct(work->task);
	kfree(work);
}

static int etnaviv_gem_userptr_get_pages(struct etnaviv_gem_object *etnaviv_obj)
{
	struct page **pvec = NULL;
	struct get_pages_work *work;
	struct mm_struct *mm;
	int ret, pinned, npages = etnaviv_obj->base.size >> PAGE_SHIFT;

	if (etnaviv_obj->userptr.work) {
		if (IS_ERR(etnaviv_obj->userptr.work)) {
			ret = PTR_ERR(etnaviv_obj->userptr.work);
			etnaviv_obj->userptr.work = NULL;
		} else {
			ret = -EAGAIN;
		}
		return ret;
	}

	mm = get_task_mm(etnaviv_obj->userptr.task);
	pinned = 0;
	if (mm == current->mm) {
		pvec = drm_malloc_ab(npages, sizeof(struct page *));
		if (!pvec) {
			mmput(mm);
			return -ENOMEM;
		}

		pinned = __get_user_pages_fast(etnaviv_obj->userptr.ptr, npages,
					       !etnaviv_obj->userptr.ro, pvec);
		if (pinned < 0) {
			drm_free_large(pvec);
			mmput(mm);
			return pinned;
		}

		if (pinned == npages) {
			etnaviv_obj->pages = pvec;
			mmput(mm);
			return 0;
		}
	}

	release_pages(pvec, pinned, 0);
	drm_free_large(pvec);

	work = kmalloc(sizeof(*work), GFP_KERNEL);
	if (!work) {
		mmput(mm);
		return -ENOMEM;
	}

	get_task_struct(current);
	drm_gem_object_reference(&etnaviv_obj->base);

	work->mm = mm;
	work->task = current;
	work->etnaviv_obj = etnaviv_obj;

	etnaviv_obj->userptr.work = &work->work;
	INIT_WORK(&work->work, __etnaviv_gem_userptr_get_pages);

	etnaviv_queue_work(etnaviv_obj->base.dev, &work->work);

	return -EAGAIN;
}

static void etnaviv_gem_userptr_release(struct etnaviv_gem_object *etnaviv_obj)
{
	if (etnaviv_obj->sgt) {
		etnaviv_gem_scatterlist_unmap(etnaviv_obj);
		sg_free_table(etnaviv_obj->sgt);
		kfree(etnaviv_obj->sgt);
	}
	if (etnaviv_obj->pages) {
		int npages = etnaviv_obj->base.size >> PAGE_SHIFT;

		release_pages(etnaviv_obj->pages, npages, 0);
		drm_free_large(etnaviv_obj->pages);
	}
	put_task_struct(etnaviv_obj->userptr.task);
}

static const struct etnaviv_gem_ops etnaviv_gem_userptr_ops = {
	.get_pages = etnaviv_gem_userptr_get_pages,
	.release = etnaviv_gem_userptr_release,
};

int etnaviv_gem_new_userptr(struct drm_device *dev, struct drm_file *file,
	uintptr_t ptr, u32 size, u32 flags, u32 *handle)
{
	struct etnaviv_gem_object *etnaviv_obj;
	int ret;

	ret = etnaviv_gem_new_private(dev, size, ETNA_BO_CACHED, &etnaviv_obj);
	if (ret)
		return ret;

	etnaviv_obj->ops = &etnaviv_gem_userptr_ops;
	etnaviv_obj->userptr.ptr = ptr;
	etnaviv_obj->userptr.task = current;
	etnaviv_obj->userptr.ro = !(flags & ETNA_USERPTR_WRITE);
	get_task_struct(current);

	ret = etnaviv_gem_obj_add(dev, &etnaviv_obj->base);
	if (ret) {
		drm_gem_object_unreference_unlocked(&etnaviv_obj->base);
		return ret;
	}

	ret = drm_gem_handle_create(file, &etnaviv_obj->base, handle);

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(&etnaviv_obj->base);

	return ret;
}
