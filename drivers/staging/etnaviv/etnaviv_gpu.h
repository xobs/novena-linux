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

#ifndef __ETNAVIV_GPU_H__
#define __ETNAVIV_GPU_H__

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include "etnaviv_drv.h"

struct etnaviv_gem_submit;

struct etnaviv_chip_identity {
	/* Chip model. */
	uint32_t model;

	/* Revision value.*/
	uint32_t revision;

	/* Supported feature fields. */
	uint32_t features;

	/* Supported minor feature fields. */
	uint32_t minor_features0;

	/* Supported minor feature 1 fields. */
	uint32_t minor_features1;

	/* Supported minor feature 2 fields. */
	uint32_t minor_features2;

	/* Supported minor feature 3 fields. */
	uint32_t minor_features3;

	/* Number of streams supported. */
	uint32_t stream_count;

	/* Total number of temporary registers per thread. */
	uint32_t register_max;

	/* Maximum number of threads. */
	uint32_t thread_count;

	/* Number of shader cores. */
	uint32_t shader_core_count;

	/* Size of the vertex cache. */
	uint32_t vertex_cache_size;

	/* Number of entries in the vertex output buffer. */
	uint32_t vertex_output_buffer_size;

	/* Number of pixel pipes. */
	uint32_t pixel_pipes;

	/* Number of instructions. */
	uint32_t instruction_count;

	/* Number of constants. */
	uint32_t num_constants;

	/* Buffer size */
	uint32_t buffer_size;
};

struct etnaviv_gpu {
	const char *name;
	struct drm_device *dev;
	struct etnaviv_chip_identity identity;
	int pipe;

	/* 'ring'-buffer: */
	struct drm_gem_object *buffer;

	/* event management: */
	bool event_used[30];
	uint32_t event_to_fence[30];
	struct completion event_free;
	struct spinlock event_spinlock;

	/* list of GEM active objects: */
	struct list_head active_list;

	uint32_t submitted_fence;
	uint32_t retired_fence;

	/* worker for handling active-list retiring: */
	struct work_struct retire_work;

	void __iomem *mmio;
	int irq;

	struct etnaviv_iommu *mmu;

	/* memory manager for GPU address area */
	struct drm_mm mm;

	/* Power Control: */
#if 0
	struct regulator *gpu_reg, *gpu_cx;
#endif
	struct clk *clk_bus;
	struct clk *clk_core;
	struct clk *clk_shader;

	/* Hang Detction: */
#define DRM_MSM_HANGCHECK_PERIOD 500 /* in ms */
#define DRM_MSM_HANGCHECK_JIFFIES msecs_to_jiffies(DRM_MSM_HANGCHECK_PERIOD)
	struct timer_list hangcheck_timer;
	uint32_t hangcheck_fence;
	struct work_struct recover_work;
};

static inline void gpu_write(struct etnaviv_gpu *gpu, u32 reg, u32 data)
{
	etnaviv_writel(data, gpu->mmio + reg);
}

static inline u32 gpu_read(struct etnaviv_gpu *gpu, u32 reg)
{
	return etnaviv_readl(gpu->mmio + reg);
}

int etnaviv_gpu_get_param(struct etnaviv_gpu *gpu, uint32_t param, uint64_t *value);

int etnaviv_gpu_init(struct etnaviv_gpu *gpu);
int etnaviv_gpu_pm_suspend(struct etnaviv_gpu *gpu);
int etnaviv_gpu_pm_resume(struct etnaviv_gpu *gpu);

#ifdef CONFIG_DEBUG_FS
void etnaviv_gpu_debugfs(struct etnaviv_gpu *gpu, struct seq_file *m);
#endif

void etnaviv_gpu_retire(struct etnaviv_gpu *gpu);
int etnaviv_gpu_submit(struct etnaviv_gpu *gpu, struct etnaviv_gem_submit *submit,
		struct etnaviv_file_private *ctx);

extern struct platform_driver etnaviv_gpu_driver;

#endif /* __ETNAVIV_GPU_H__ */
