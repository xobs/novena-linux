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

#ifndef __ETNAVIV_GEM_H__
#define __ETNAVIV_GEM_H__

#include <linux/reservation.h>
#include "etnaviv_drv.h"

struct etnaviv_gem_ops;
struct etnaviv_gem_object;

struct etnaviv_gem_userptr {
	uintptr_t ptr;
	struct task_struct *task;
	struct work_struct *work;
	bool ro;
};

struct etnaviv_vram_mapping {
	struct list_head obj_node;
	struct list_head scan_node;
	struct etnaviv_gem_object *object;
	struct etnaviv_iommu *mmu;
	struct drm_mm_node vram_node;
	u32 iova;
};

struct etnaviv_gem_object {
	struct drm_gem_object base;
	const struct etnaviv_gem_ops *ops;

	u32 flags;

	/* And object is either:
	 *  inactive - on priv->inactive_list
	 *  active   - on one one of the gpu's active_list..  well, at
	 *     least for now we don't have (I don't think) hw sync between
	 *     2d and 3d one devices which have both, meaning we need to
	 *     block on submit if a bo is already on other ring
	 *
	 */
	struct list_head mm_list;
	struct etnaviv_gpu *gpu;     /* non-null if active */
	u32 access;
	u32 read_fence, write_fence;

	/* Transiently in the process of submit ioctl, objects associated
	 * with the submit are on submit->bo_list.. this only lasts for
	 * the duration of the ioctl, so one bo can never be on multiple
	 * submit lists.
	 */
	struct list_head submit_entry;

	struct page **pages;
	struct sg_table *sgt;
	void *vaddr;

	/* for ETNA_BO_CMDSTREAM */
	dma_addr_t paddr;

	/* normally (resv == &_resv) except for imported bo's */
	struct reservation_object *resv;
	struct reservation_object _resv;

	struct list_head vram_list;

	/* for buffer manipulation during submit */
	bool is_ring_buffer;
	u32 offset;

	struct etnaviv_gem_userptr userptr;
};

static inline
struct etnaviv_gem_object *to_etnaviv_bo(struct drm_gem_object *obj)
{
	return container_of(obj, struct etnaviv_gem_object, base);
}

struct etnaviv_gem_ops {
	int (*get_pages)(struct etnaviv_gem_object *);
	void (*release)(struct etnaviv_gem_object *);
};

static inline bool is_active(struct etnaviv_gem_object *etnaviv_obj)
{
	return etnaviv_obj->gpu != NULL;
}

#define MAX_CMDS 4

/* Created per submit-ioctl, to track bo's and cmdstream bufs, etc,
 * associated with the cmdstream submission for synchronization (and
 * make it easier to unwind when things go wrong, etc).  This only
 * lasts for the duration of the submit-ioctl.
 */
struct etnaviv_gem_submit {
	struct drm_device *dev;
	struct etnaviv_gpu *gpu;
	u32 exec_state;
	struct list_head bo_list;
	struct ww_acquire_ctx ticket;
	u32 fence;
	unsigned int nr_cmds;
	unsigned int nr_bos;
	struct {
		u32 type;
		u32 offset; /* in dwords */
		u32 size;  /* in dwords */
		struct etnaviv_gem_object *obj;
	} cmd[MAX_CMDS];
	struct {
		u32 flags;
		struct etnaviv_gem_object *obj;
		u32 iova;
	} bos[0];
};

int etnaviv_gem_wait_bo(struct etnaviv_gpu *gpu, struct drm_gem_object *obj,
	struct timespec *timeout);
struct etnaviv_vram_mapping *
etnaviv_gem_get_vram_mapping(struct etnaviv_gem_object *obj,
			     struct etnaviv_iommu *mmu);
int etnaviv_gem_new_private(struct drm_device *dev, size_t size, u32 flags,
	struct etnaviv_gem_object **res);
int etnaviv_gem_obj_add(struct drm_device *dev, struct drm_gem_object *obj);
struct page **etnaviv_gem_get_pages(struct etnaviv_gem_object *obj);
void etnaviv_gem_put_pages(struct etnaviv_gem_object *obj);

#endif /* __ETNAVIV_GEM_H__ */
