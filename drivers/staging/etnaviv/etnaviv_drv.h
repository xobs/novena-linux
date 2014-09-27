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

#ifndef __ETNAVIV_DRV_H__
#define __ETNAVIV_DRV_H__

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/iommu.h>
#include <linux/types.h>
#include <linux/sizes.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/etnaviv_drm.h>

struct etnaviv_gpu;
struct etnaviv_mmu;
struct etnaviv_gem_submit;

struct etnaviv_file_private {
	/* currently we don't do anything useful with this.. but when
	 * per-context address spaces are supported we'd keep track of
	 * the context's page-tables here.
	 */
	int dummy;
};

struct etnaviv_drm_private {
	struct etnaviv_gpu *gpu[ETNA_MAX_PIPES];
	struct etnaviv_file_private *lastctx;

	uint32_t next_fence, completed_fence;
	wait_queue_head_t fence_event;

	/* list of GEM objects: */
	struct list_head inactive_list;

	struct workqueue_struct *wq;

	/* registered MMUs: */
	struct etnaviv_iommu *mmu;
};

void etnaviv_register_mmu(struct drm_device *dev, struct etnaviv_iommu *mmu);

int etnaviv_wait_fence_interruptable(struct drm_device *dev, uint32_t pipe,
		uint32_t fence,	struct timespec *timeout);
void etnaviv_update_fence(struct drm_device *dev, uint32_t fence);

int etnaviv_ioctl_gem_submit(struct drm_device *dev, void *data,
		struct drm_file *file);

int etnaviv_gem_mmap(struct file *filp, struct vm_area_struct *vma);
int etnaviv_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
uint64_t msm_gem_mmap_offset(struct drm_gem_object *obj);
int etnaviv_gem_get_iova_locked(struct etnaviv_gpu *gpu, struct drm_gem_object *obj,
		uint32_t *iova);
int etnaviv_gem_get_iova(struct etnaviv_gpu *gpu, struct drm_gem_object *obj, int id, uint32_t *iova);
struct page **etnaviv_gem_get_pages(struct drm_gem_object *obj);
void msm_gem_put_pages(struct drm_gem_object *obj);
void etnaviv_gem_put_iova(struct drm_gem_object *obj);
int msm_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		struct drm_mode_create_dumb *args);
int msm_gem_dumb_map_offset(struct drm_file *file, struct drm_device *dev,
		uint32_t handle, uint64_t *offset);
struct sg_table *msm_gem_prime_get_sg_table(struct drm_gem_object *obj);
void *msm_gem_prime_vmap(struct drm_gem_object *obj);
void msm_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);
struct drm_gem_object *msm_gem_prime_import_sg_table(struct drm_device *dev,
		size_t size, struct sg_table *sg);
int msm_gem_prime_pin(struct drm_gem_object *obj);
void msm_gem_prime_unpin(struct drm_gem_object *obj);
void *etnaviv_gem_vaddr_locked(struct drm_gem_object *obj);
void *msm_gem_vaddr(struct drm_gem_object *obj);
dma_addr_t etnaviv_gem_paddr_locked(struct drm_gem_object *obj);
void etnaviv_gem_move_to_active(struct drm_gem_object *obj,
		struct etnaviv_gpu *gpu, bool write, uint32_t fence);
void etnaviv_gem_move_to_inactive(struct drm_gem_object *obj);
int etnaviv_gem_cpu_prep(struct drm_gem_object *obj, uint32_t op,
		struct timespec *timeout);
int etnaviv_gem_cpu_fini(struct drm_gem_object *obj);
void etnaviv_gem_free_object(struct drm_gem_object *obj);
int etnaviv_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		uint32_t size, uint32_t flags, uint32_t *handle);
struct drm_gem_object *etnaviv_gem_new(struct drm_device *dev,
		uint32_t size, uint32_t flags);
struct drm_gem_object *msm_gem_import(struct drm_device *dev,
		uint32_t size, struct sg_table *sgt);
u32 etnaviv_buffer_init(struct etnaviv_gpu *gpu);
void etnaviv_buffer_queue(struct etnaviv_gpu *gpu, unsigned int event, struct etnaviv_gem_submit *submit);

#ifdef CONFIG_DEBUG_FS
void msm_gem_describe(struct drm_gem_object *obj, struct seq_file *m);
void msm_gem_describe_objects(struct list_head *list, struct seq_file *m);
void msm_framebuffer_describe(struct drm_framebuffer *fb, struct seq_file *m);
#endif

void __iomem *etnaviv_ioremap(struct platform_device *pdev, const char *name,
		const char *dbgname);
void etnaviv_writel(u32 data, void __iomem *addr);
u32 etnaviv_readl(const void __iomem *addr);

#define DBG(fmt, ...) DRM_DEBUG(fmt"\n", ##__VA_ARGS__)
#define VERB(fmt, ...) if (0) DRM_DEBUG(fmt"\n", ##__VA_ARGS__)

static inline bool fence_completed(struct drm_device *dev, uint32_t fence)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	return priv->completed_fence >= fence;
}

static inline int align_pitch(int width, int bpp)
{
	int bytespp = (bpp + 7) / 8;
	/* adreno needs pitch aligned to 32 pixels: */
	return bytespp * ALIGN(width, 32);
}

#endif /* __ETNAVIV_DRV_H__ */
