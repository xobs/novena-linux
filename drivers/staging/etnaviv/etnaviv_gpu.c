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

#include <linux/component.h>
#include <linux/of_device.h>
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"
#include "etnaviv_iommu.h"
#include "etnaviv_iommu_v2.h"
#include "common.xml.h"
#include "state.xml.h"
#include "state_hi.xml.h"
#include "cmdstream.xml.h"

static const struct platform_device_id gpu_ids[] = {
	{ .name = "etnaviv-gpu,2d" },
	{ },
};

/*
 * Driver functions:
 */

int etnaviv_gpu_get_param(struct etnaviv_gpu *gpu, u32 param, u64 *value)
{
	switch (param) {
	case ETNAVIV_PARAM_GPU_MODEL:
		*value = gpu->identity.model;
		break;

	case ETNAVIV_PARAM_GPU_REVISION:
		*value = gpu->identity.revision;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_0:
		*value = gpu->identity.features;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_1:
		*value = gpu->identity.minor_features0;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_2:
		*value = gpu->identity.minor_features1;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_3:
		*value = gpu->identity.minor_features2;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_4:
		*value = gpu->identity.minor_features3;
		break;

	case ETNAVIV_PARAM_GPU_STREAM_COUNT:
		*value = gpu->identity.stream_count;
		break;

	case ETNAVIV_PARAM_GPU_REGISTER_MAX:
		*value = gpu->identity.register_max;
		break;

	case ETNAVIV_PARAM_GPU_THREAD_COUNT:
		*value = gpu->identity.thread_count;
		break;

	case ETNAVIV_PARAM_GPU_VERTEX_CACHE_SIZE:
		*value = gpu->identity.vertex_cache_size;
		break;

	case ETNAVIV_PARAM_GPU_SHADER_CORE_COUNT:
		*value = gpu->identity.shader_core_count;
		break;

	case ETNAVIV_PARAM_GPU_PIXEL_PIPES:
		*value = gpu->identity.pixel_pipes;
		break;

	case ETNAVIV_PARAM_GPU_VERTEX_OUTPUT_BUFFER_SIZE:
		*value = gpu->identity.vertex_output_buffer_size;
		break;

	case ETNAVIV_PARAM_GPU_BUFFER_SIZE:
		*value = gpu->identity.buffer_size;
		break;

	case ETNAVIV_PARAM_GPU_INSTRUCTION_COUNT:
		*value = gpu->identity.instruction_count;
		break;

	case ETNAVIV_PARAM_GPU_NUM_CONSTANTS:
		*value = gpu->identity.num_constants;
		break;

	default:
		DBG("%s: invalid param: %u", dev_name(gpu->dev), param);
		return -EINVAL;
	}

	return 0;
}

static void etnaviv_hw_specs(struct etnaviv_gpu *gpu)
{
	if (gpu->identity.minor_features0 &
	    chipMinorFeatures0_MORE_MINOR_FEATURES) {
		u32 specs[2];

		specs[0] = gpu_read(gpu, VIVS_HI_CHIP_SPECS);
		specs[1] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_2);

		gpu->identity.stream_count =
			(specs[0] & VIVS_HI_CHIP_SPECS_STREAM_COUNT__MASK)
				>> VIVS_HI_CHIP_SPECS_STREAM_COUNT__SHIFT;
		gpu->identity.register_max =
			(specs[0] & VIVS_HI_CHIP_SPECS_REGISTER_MAX__MASK)
				>> VIVS_HI_CHIP_SPECS_REGISTER_MAX__SHIFT;
		gpu->identity.thread_count =
			(specs[0] & VIVS_HI_CHIP_SPECS_THREAD_COUNT__MASK)
				>> VIVS_HI_CHIP_SPECS_THREAD_COUNT__SHIFT;
		gpu->identity.vertex_cache_size =
			(specs[0] & VIVS_HI_CHIP_SPECS_VERTEX_CACHE_SIZE__MASK)
				>> VIVS_HI_CHIP_SPECS_VERTEX_CACHE_SIZE__SHIFT;
		gpu->identity.shader_core_count =
			(specs[0] & VIVS_HI_CHIP_SPECS_SHADER_CORE_COUNT__MASK)
				>> VIVS_HI_CHIP_SPECS_SHADER_CORE_COUNT__SHIFT;
		gpu->identity.pixel_pipes =
			(specs[0] & VIVS_HI_CHIP_SPECS_PIXEL_PIPES__MASK)
				>> VIVS_HI_CHIP_SPECS_PIXEL_PIPES__SHIFT;
		gpu->identity.vertex_output_buffer_size =
			(specs[0] & VIVS_HI_CHIP_SPECS_VERTEX_OUTPUT_BUFFER_SIZE__MASK)
				>> VIVS_HI_CHIP_SPECS_VERTEX_OUTPUT_BUFFER_SIZE__SHIFT;

		gpu->identity.buffer_size =
			(specs[1] & VIVS_HI_CHIP_SPECS_2_BUFFER_SIZE__MASK)
				>> VIVS_HI_CHIP_SPECS_2_BUFFER_SIZE__SHIFT;
		gpu->identity.instruction_count =
			(specs[1] & VIVS_HI_CHIP_SPECS_2_INSTRUCTION_COUNT__MASK)
				>> VIVS_HI_CHIP_SPECS_2_INSTRUCTION_COUNT__SHIFT;
		gpu->identity.num_constants =
			(specs[1] & VIVS_HI_CHIP_SPECS_2_NUM_CONSTANTS__MASK)
				>> VIVS_HI_CHIP_SPECS_2_NUM_CONSTANTS__SHIFT;

		gpu->identity.register_max = 1 << gpu->identity.register_max;
		gpu->identity.thread_count = 1 << gpu->identity.thread_count;
		gpu->identity.vertex_output_buffer_size =
			1 << gpu->identity.vertex_output_buffer_size;
	} else {
		dev_err(gpu->dev, "TODO: determine GPU specs based on model\n");
	}

	switch (gpu->identity.instruction_count) {
	case 0:
		if ((gpu->identity.model == 0x2000 &&
		     gpu->identity.revision == 0x5108) ||
		    gpu->identity.model == 0x880)
			gpu->identity.instruction_count = 512;
		else
			gpu->identity.instruction_count = 256;
		break;

	case 1:
		gpu->identity.instruction_count = 1024;
		break;

	case 2:
		gpu->identity.instruction_count = 2048;
		break;

	default:
		gpu->identity.instruction_count = 256;
		break;
	}
}

static void etnaviv_hw_identify(struct etnaviv_gpu *gpu)
{
	u32 chipIdentity;

	chipIdentity = gpu_read(gpu, VIVS_HI_CHIP_IDENTITY);

	/* Special case for older graphic cores. */
	if (VIVS_HI_CHIP_IDENTITY_FAMILY(chipIdentity) ==  0x01) {
		gpu->identity.model    = 0x500; /* gc500 */
		gpu->identity.revision = VIVS_HI_CHIP_IDENTITY_REVISION(chipIdentity);
	} else {

		gpu->identity.model = gpu_read(gpu, VIVS_HI_CHIP_MODEL);
		gpu->identity.revision = gpu_read(gpu, VIVS_HI_CHIP_REV);

		/*
		 * !!!! HACK ALERT !!!!
		 * Because people change device IDs without letting software
		 * know about it - here is the hack to make it all look the
		 * same.  Only for GC400 family.
		 */
		if ((gpu->identity.model & 0xff00) == 0x0400 &&
		    gpu->identity.model != 0x0420) {
			gpu->identity.model = gpu->identity.model & 0x0400;
		}

		/* Another special case */
		if (gpu->identity.model == 0x300 &&
		    gpu->identity.revision == 0x2201) {
			u32 chipDate = gpu_read(gpu, VIVS_HI_CHIP_DATE);
			u32 chipTime = gpu_read(gpu, VIVS_HI_CHIP_TIME);

			if (chipDate == 0x20080814 && chipTime == 0x12051100) {
				/*
				 * This IP has an ECO; put the correct
				 * revision in it.
				 */
				gpu->identity.revision = 0x1051;
			}
		}
	}

	dev_info(gpu->dev, "model: GC%x, revision: %x\n",
		 gpu->identity.model, gpu->identity.revision);

	gpu->identity.features = gpu_read(gpu, VIVS_HI_CHIP_FEATURE);

	/* Disable fast clear on GC700. */
	if (gpu->identity.model == 0x700)
		gpu->identity.features &= ~chipFeatures_FAST_CLEAR;

	if ((gpu->identity.model == 0x500 && gpu->identity.revision < 2) ||
	    (gpu->identity.model == 0x300 && gpu->identity.revision < 0x2000)) {

		/*
		 * GC500 rev 1.x and GC300 rev < 2.0 doesn't have these
		 * registers.
		 */
		gpu->identity.minor_features0 = 0;
		gpu->identity.minor_features1 = 0;
		gpu->identity.minor_features2 = 0;
		gpu->identity.minor_features3 = 0;
	} else
		gpu->identity.minor_features0 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_0);

	if (gpu->identity.minor_features0 &
	    chipMinorFeatures0_MORE_MINOR_FEATURES) {
		gpu->identity.minor_features1 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_1);
		gpu->identity.minor_features2 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_2);
		gpu->identity.minor_features3 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_3);
	}

	/* GC600 idle register reports zero bits where modules aren't present */
	if (gpu->identity.model == chipModel_GC600) {
		gpu->idle_mask = VIVS_HI_IDLE_STATE_TX |
				 VIVS_HI_IDLE_STATE_RA |
				 VIVS_HI_IDLE_STATE_SE |
				 VIVS_HI_IDLE_STATE_PA |
				 VIVS_HI_IDLE_STATE_SH |
				 VIVS_HI_IDLE_STATE_PE |
				 VIVS_HI_IDLE_STATE_DE |
				 VIVS_HI_IDLE_STATE_FE;
	} else {
		gpu->idle_mask = ~VIVS_HI_IDLE_STATE_AXI_LP;
	}

	etnaviv_hw_specs(gpu);
}

static void etnaviv_gpu_load_clock(struct etnaviv_gpu *gpu, u32 clock)
{
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, clock |
		  VIVS_HI_CLOCK_CONTROL_FSCALE_CMD_LOAD);
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, clock);
}

static int etnaviv_hw_reset(struct etnaviv_gpu *gpu)
{
	u32 control, idle;
	unsigned long timeout;
	bool failed = true;

	/* TODO
	 *
	 * - clock gating
	 * - puls eater
	 * - what about VG?
	 */

	/* We hope that the GPU resets in under one second */
	timeout = jiffies + msecs_to_jiffies(1000);

	while (time_is_after_jiffies(timeout)) {
		control = VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS |
			  VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(0x40);

		/* enable clock */
		etnaviv_gpu_load_clock(gpu, control);

		/* Wait for stable clock.  Vivante's code waited for 1ms */
		usleep_range(1000, 10000);

		/* isolate the GPU. */
		control |= VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* set soft reset. */
		control |= VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* wait for reset. */
		msleep(1);

		/* reset soft reset bit. */
		control &= ~VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* reset GPU isolation. */
		control &= ~VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* read idle register. */
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		/* try reseting again if FE it not idle */
		if ((idle & VIVS_HI_IDLE_STATE_FE) == 0) {
			dev_dbg(gpu->dev, "FE is not idle\n");
			continue;
		}

		/* read reset register. */
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		/* is the GPU idle? */
		if (((control & VIVS_HI_CLOCK_CONTROL_IDLE_3D) == 0) ||
		    ((control & VIVS_HI_CLOCK_CONTROL_IDLE_2D) == 0)) {
			dev_dbg(gpu->dev, "GPU is not idle\n");
			continue;
		}

		failed = false;
		break;
	}

	if (failed) {
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		dev_err(gpu->dev, "GPU failed to reset: FE %sidle, 3D %sidle, 2D %sidle\n",
			idle & VIVS_HI_IDLE_STATE_FE ? "" : "not ",
			control & VIVS_HI_CLOCK_CONTROL_IDLE_3D ? "" : "not ",
			control & VIVS_HI_CLOCK_CONTROL_IDLE_2D ? "" : "not ");

		return -EBUSY;
	}

	/* We rely on the GPU running, so program the clock */
	control = VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS |
		  VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(0x40);

	/* enable clock */
	etnaviv_gpu_load_clock(gpu, control);

	return 0;
}

static void etnaviv_gpu_hw_init(struct etnaviv_gpu *gpu)
{
	u32 words; /* 32 bit words */

	if (gpu->identity.model == chipModel_GC320 &&
	    gpu_read(gpu, VIVS_HI_CHIP_TIME) != 0x2062400 &&
	    (gpu->identity.revision == 0x5007 ||
	     gpu->identity.revision == 0x5220)) {
		u32 mc_memory_debug;

		mc_memory_debug = gpu_read(gpu, VIVS_MC_DEBUG_MEMORY) & ~0xff;

		if (gpu->identity.revision == 0x5007)
			mc_memory_debug |= 0x0c;
		else
			mc_memory_debug |= 0x08;

		gpu_write(gpu, VIVS_MC_DEBUG_MEMORY, mc_memory_debug);
	}

	/*
	 * Update GPU AXI cache atttribute to "cacheable, no allocate".
	 * This is necessary to prevent the iMX6 SoC locking up.
	 */
	gpu_write(gpu, VIVS_HI_AXI_CONFIG,
		  VIVS_HI_AXI_CONFIG_AWCACHE(2) |
		  VIVS_HI_AXI_CONFIG_ARCACHE(2));

	/* GC2000 rev 5108 needs a special bus config */
	if (gpu->identity.model == 0x2000 && gpu->identity.revision == 0x5108) {
		u32 bus_config = gpu_read(gpu, VIVS_MC_BUS_CONFIG);
		bus_config &= ~(VIVS_MC_BUS_CONFIG_FE_BUS_CONFIG__MASK |
				VIVS_MC_BUS_CONFIG_TX_BUS_CONFIG__MASK);
		bus_config |= VIVS_MC_BUS_CONFIG_FE_BUS_CONFIG(1) |
			      VIVS_MC_BUS_CONFIG_TX_BUS_CONFIG(0);
		gpu_write(gpu, VIVS_MC_BUS_CONFIG, bus_config);
	}

	/* set base addresses */
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_RA, gpu->memory_base);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_FE, gpu->memory_base);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_TX, gpu->memory_base);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_PEZ, gpu->memory_base);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_PE, gpu->memory_base);

	/* setup the MMU page table pointers */
	etnaviv_iommu_domain_restore(gpu, gpu->mmu->domain);

	/* Start command processor */
	words = etnaviv_buffer_init(gpu);

	/* convert number of 32 bit words to number of 64 bit words */
	words = ALIGN(words, 2) / 2;

	gpu_write(gpu, VIVS_HI_INTR_ENBL, ~0U);
	gpu_write(gpu, VIVS_FE_COMMAND_ADDRESS,
		  etnaviv_gem_paddr_locked(gpu->buffer) - gpu->memory_base);
	gpu_write(gpu, VIVS_FE_COMMAND_CONTROL,
		  VIVS_FE_COMMAND_CONTROL_ENABLE |
		  VIVS_FE_COMMAND_CONTROL_PREFETCH(words));
}

int etnaviv_gpu_init(struct etnaviv_gpu *gpu)
{
	int ret, i;
	struct iommu_domain *iommu;
	enum etnaviv_iommu_version version;
	bool mmuv2;

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0)
		return ret;

	etnaviv_hw_identify(gpu);

	if (gpu->identity.model == 0) {
		dev_err(gpu->dev, "Unknown GPU model\n");
		pm_runtime_put_autosuspend(gpu->dev);
		return -ENXIO;
	}

	ret = etnaviv_hw_reset(gpu);
	if (ret)
		goto fail;

	/* Setup IOMMU.. eventually we will (I think) do this once per context
	 * and have separate page tables per context.  For now, to keep things
	 * simple and to get something working, just use a single address space:
	 */
	mmuv2 = gpu->identity.minor_features1 & chipMinorFeatures1_MMU_VERSION;
	dev_dbg(gpu->dev, "mmuv2: %d\n", mmuv2);

	if (!mmuv2) {
		iommu = etnaviv_iommu_domain_alloc(gpu);
		version = ETNAVIV_IOMMU_V1;
	} else {
		iommu = etnaviv_iommu_v2_domain_alloc(gpu);
		version = ETNAVIV_IOMMU_V2;
	}

	if (!iommu) {
		ret = -ENOMEM;
		goto fail;
	}

	/* TODO: we will leak here memory - fix it! */

	gpu->mmu = etnaviv_iommu_new(gpu->dev, iommu, version);
	if (!gpu->mmu) {
		ret = -ENOMEM;
		goto fail;
	}

	/* Create buffer: */
	gpu->buffer = etnaviv_gem_new(gpu->drm, PAGE_SIZE, ETNA_BO_CMDSTREAM);
	if (IS_ERR(gpu->buffer)) {
		ret = PTR_ERR(gpu->buffer);
		gpu->buffer = NULL;
		dev_err(gpu->dev, "could not create buffer: %d\n", ret);
		goto fail;
	}

	/* Setup event management */
	spin_lock_init(&gpu->event_spinlock);
	init_completion(&gpu->event_free);
	for (i = 0; i < ARRAY_SIZE(gpu->event); i++) {
		gpu->event[i].used = false;
		complete(&gpu->event_free);
	}

	/* Now program the hardware */
	mutex_lock(&gpu->drm->struct_mutex);
	etnaviv_gpu_hw_init(gpu);
	mutex_unlock(&gpu->drm->struct_mutex);

	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);

	return 0;

fail:
	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
struct dma_debug {
	u32 address[2];
	u32 state[2];
};

static void verify_dma(struct etnaviv_gpu *gpu, struct dma_debug *debug)
{
	u32 i;

	debug->address[0] = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	debug->state[0]   = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);

	for (i = 0; i < 500; i++) {
		debug->address[1] = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		debug->state[1]   = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);

		if (debug->address[0] != debug->address[1])
			break;

		if (debug->state[0] != debug->state[1])
			break;
	}
}

int etnaviv_gpu_debugfs(struct etnaviv_gpu *gpu, struct seq_file *m)
{
	struct dma_debug debug;
	u32 dma_lo, dma_hi, axi, idle;
	int ret;

	seq_printf(m, "%s Status:\n", dev_name(gpu->dev));

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0)
		return ret;

	ret = mutex_lock_interruptible(&gpu->drm->struct_mutex);
	if (ret < 0)
		goto err_rpm;

	dma_lo = gpu_read(gpu, VIVS_FE_DMA_LOW);
	dma_hi = gpu_read(gpu, VIVS_FE_DMA_HIGH);
	axi = gpu_read(gpu, VIVS_HI_AXI_STATUS);
	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

	verify_dma(gpu, &debug);

	seq_puts(m, "\tfeatures\n");
	seq_printf(m, "\t minor_features0: 0x%08x\n",
		   gpu->identity.minor_features0);
	seq_printf(m, "\t minor_features1: 0x%08x\n",
		   gpu->identity.minor_features1);
	seq_printf(m, "\t minor_features2: 0x%08x\n",
		   gpu->identity.minor_features2);
	seq_printf(m, "\t minor_features3: 0x%08x\n",
		   gpu->identity.minor_features3);

	seq_puts(m, "\tspecs\n");
	seq_printf(m, "\t stream_count:  %d\n",
			gpu->identity.stream_count);
	seq_printf(m, "\t register_max: %d\n",
			gpu->identity.register_max);
	seq_printf(m, "\t thread_count: %d\n",
			gpu->identity.thread_count);
	seq_printf(m, "\t vertex_cache_size: %d\n",
			gpu->identity.vertex_cache_size);
	seq_printf(m, "\t shader_core_count: %d\n",
			gpu->identity.shader_core_count);
	seq_printf(m, "\t pixel_pipes: %d\n",
			gpu->identity.pixel_pipes);
	seq_printf(m, "\t vertex_output_buffer_size: %d\n",
			gpu->identity.vertex_output_buffer_size);
	seq_printf(m, "\t buffer_size: %d\n",
			gpu->identity.buffer_size);
	seq_printf(m, "\t instruction_count: %d\n",
			gpu->identity.instruction_count);
	seq_printf(m, "\t num_constants: %d\n",
			gpu->identity.num_constants);

	seq_printf(m, "\taxi: 0x%08x\n", axi);
	seq_printf(m, "\tidle: 0x%08x\n", idle);
	idle |= ~gpu->idle_mask & ~VIVS_HI_IDLE_STATE_AXI_LP;
	if ((idle & VIVS_HI_IDLE_STATE_FE) == 0)
		seq_puts(m, "\t FE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_DE) == 0)
		seq_puts(m, "\t DE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PE) == 0)
		seq_puts(m, "\t PE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SH) == 0)
		seq_puts(m, "\t SH is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PA) == 0)
		seq_puts(m, "\t PA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SE) == 0)
		seq_puts(m, "\t SE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_RA) == 0)
		seq_puts(m, "\t RA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TX) == 0)
		seq_puts(m, "\t TX is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_VG) == 0)
		seq_puts(m, "\t VG is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_IM) == 0)
		seq_puts(m, "\t IM is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_FP) == 0)
		seq_puts(m, "\t FP is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TS) == 0)
		seq_puts(m, "\t TS is not idle\n");
	if (idle & VIVS_HI_IDLE_STATE_AXI_LP)
		seq_puts(m, "\t AXI low power mode\n");

	if (gpu->identity.features & chipFeatures_DEBUG_MODE) {
		u32 read0 = gpu_read(gpu, VIVS_MC_DEBUG_READ0);
		u32 read1 = gpu_read(gpu, VIVS_MC_DEBUG_READ1);
		u32 write = gpu_read(gpu, VIVS_MC_DEBUG_WRITE);

		seq_puts(m, "\tMC\n");
		seq_printf(m, "\t read0: 0x%08x\n", read0);
		seq_printf(m, "\t read1: 0x%08x\n", read1);
		seq_printf(m, "\t write: 0x%08x\n", write);
	}

	seq_puts(m, "\tDMA ");

	if (debug.address[0] == debug.address[1] &&
	    debug.state[0] == debug.state[1]) {
		seq_puts(m, "seems to be stuck\n");
	} else if (debug.address[0] == debug.address[1]) {
		seq_puts(m, "adress is constant\n");
	} else {
		seq_puts(m, "is runing\n");
	}

	seq_printf(m, "\t address 0: 0x%08x\n", debug.address[0]);
	seq_printf(m, "\t address 1: 0x%08x\n", debug.address[1]);
	seq_printf(m, "\t state 0: 0x%08x\n", debug.state[0]);
	seq_printf(m, "\t state 1: 0x%08x\n", debug.state[1]);
	seq_printf(m, "\t last fetch 64 bit word: 0x%08x 0x%08x\n",
		   dma_lo, dma_hi);

	ret = 0;

	mutex_unlock(&gpu->drm->struct_mutex);

err_rpm:
	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);

	return ret;
}
#endif

/*
 * Power Management:
 */
static int enable_clk(struct etnaviv_gpu *gpu)
{
	if (gpu->clk_core)
		clk_prepare_enable(gpu->clk_core);
	if (gpu->clk_shader)
		clk_prepare_enable(gpu->clk_shader);

	return 0;
}

static int disable_clk(struct etnaviv_gpu *gpu)
{
	if (gpu->clk_core)
		clk_disable_unprepare(gpu->clk_core);
	if (gpu->clk_shader)
		clk_disable_unprepare(gpu->clk_shader);

	return 0;
}

static int enable_axi(struct etnaviv_gpu *gpu)
{
	if (gpu->clk_bus)
		clk_prepare_enable(gpu->clk_bus);

	return 0;
}

static int disable_axi(struct etnaviv_gpu *gpu)
{
	if (gpu->clk_bus)
		clk_disable_unprepare(gpu->clk_bus);

	return 0;
}

/*
 * Hangcheck detection for locked gpu:
 */
static void recover_worker(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       recover_work);
	struct drm_device *dev = gpu->drm;
	u32 busy;

	dev_err(gpu->dev, "hangcheck recover!\n");

	mutex_lock(&dev->struct_mutex);
	busy = ~gpu_read(gpu, VIVS_HI_IDLE_STATE) & gpu->idle_mask;

	/*
	 * If the only unit which is busy is the front end, we have likely
	 * completed all operations.  We've just missed an interrupt.
	 */
	if (busy == VIVS_HI_IDLE_STATE_FE) {
		unsigned long flags;
		unsigned int i;

		dev_err(gpu->dev, "missed interrupt?\n");

		spin_lock_irqsave(&gpu->event_spinlock, flags);
		for (i = 0; i < ARRAY_SIZE(gpu->event); i++) {
			if (!gpu->event[i].used)
				continue;
			dev_err(gpu->dev, "probable lost event %u, fence %u\n",
				i, gpu->event[i].fence);
			if (fence_after(gpu->event[i].fence,
					gpu->completed_fence))
				gpu->completed_fence = gpu->event[i].fence;
			gpu->event[i].used = false;
			complete(&gpu->event_free);
		}
		spin_unlock_irqrestore(&gpu->event_spinlock, flags);
	}

	/* TODO gpu->funcs->recover(gpu); */
	mutex_unlock(&dev->struct_mutex);

	/* Retire the buffer objects in a work */
	etnaviv_queue_work(gpu->drm, &gpu->retire_work);
}

static void hangcheck_timer_reset(struct etnaviv_gpu *gpu)
{
	DBG("%s", dev_name(gpu->dev));
	mod_timer(&gpu->hangcheck_timer,
		  round_jiffies_up(jiffies + DRM_ETNAVIV_HANGCHECK_JIFFIES));
}

static void hangcheck_handler(unsigned long data)
{
	struct etnaviv_gpu *gpu = (struct etnaviv_gpu *)data;
	u32 fence = gpu->completed_fence;
	bool progress = false;

	if (fence != gpu->hangcheck_fence) {
		gpu->hangcheck_fence = fence;
		progress = true;
	}

	if (!progress) {
		u32 dma_addr = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		int change = dma_addr - gpu->hangcheck_dma_addr;

		if (change < 0 || change > 16) {
			gpu->hangcheck_dma_addr = dma_addr;
			progress = true;
		}
	}

	if (!progress && fence_after(gpu->submitted_fence, fence)) {
		dev_err(gpu->dev, "hangcheck detected gpu lockup!\n");
		dev_err(gpu->dev, "     completed fence: %u\n", fence);
		dev_err(gpu->dev, "     submitted fence: %u\n",
			gpu->submitted_fence);
		etnaviv_queue_work(gpu->drm, &gpu->recover_work);
	}

	/* if still more pending work, reset the hangcheck timer: */
	if (fence_after(gpu->submitted_fence, gpu->hangcheck_fence))
		hangcheck_timer_reset(gpu);
}

static void hangcheck_disable(struct etnaviv_gpu *gpu)
{
	del_timer_sync(&gpu->hangcheck_timer);
	cancel_work_sync(&gpu->recover_work);
}

/*
 * event management:
 */

static unsigned int event_alloc(struct etnaviv_gpu *gpu)
{
	unsigned long ret, flags;
	unsigned int i, event = ~0U;

	ret = wait_for_completion_timeout(&gpu->event_free,
					  msecs_to_jiffies(10 * 10000));
	if (!ret)
		dev_err(gpu->dev, "wait_for_completion_timeout failed");

	spin_lock_irqsave(&gpu->event_spinlock, flags);

	/* find first free event */
	for (i = 0; i < ARRAY_SIZE(gpu->event); i++) {
		if (gpu->event[i].used == false) {
			gpu->event[i].used = true;
			event = i;
			break;
		}
	}

	spin_unlock_irqrestore(&gpu->event_spinlock, flags);

	return event;
}

static void event_free(struct etnaviv_gpu *gpu, unsigned int event)
{
	unsigned long flags;

	spin_lock_irqsave(&gpu->event_spinlock, flags);

	if (gpu->event[event].used == false) {
		dev_warn(gpu->dev, "event %u is already marked as free",
			 event);
		spin_unlock_irqrestore(&gpu->event_spinlock, flags);
	} else {
		gpu->event[event].used = false;
		spin_unlock_irqrestore(&gpu->event_spinlock, flags);

		complete(&gpu->event_free);
	}
}

/*
 * Cmdstream submission/retirement:
 */

static void retire_worker(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       retire_work);
	struct drm_device *dev = gpu->drm;
	u32 fence = gpu->completed_fence;

	mutex_lock(&dev->struct_mutex);

	while (!list_empty(&gpu->active_list)) {
		struct etnaviv_gem_object *obj;

		obj = list_first_entry(&gpu->active_list,
				struct etnaviv_gem_object, mm_list);

		if ((!(obj->access & ETNA_SUBMIT_BO_READ) ||
		     fence_after_eq(fence, obj->read_fence)) &&
		    (!(obj->access & ETNA_SUBMIT_BO_WRITE) ||
		     fence_after_eq(fence, obj->write_fence))) {
			/* move to inactive: */
			etnaviv_gem_move_to_inactive(&obj->base);
			etnaviv_gem_put_iova(&obj->base);
			drm_gem_object_unreference(&obj->base);
		} else {
			break;
		}
	}

	gpu->retired_fence = fence;

	mutex_unlock(&dev->struct_mutex);

	wake_up_all(&gpu->fence_event);
}

static unsigned long etnaviv_timeout_to_jiffies(struct timespec *timeout)
{
	unsigned long timeout_jiffies = timespec_to_jiffies(timeout);
	unsigned long start_jiffies = jiffies;
	unsigned long remaining_jiffies;

	if (time_after(start_jiffies, timeout_jiffies))
		remaining_jiffies = 0;
	else
		remaining_jiffies = timeout_jiffies - start_jiffies;

	return remaining_jiffies;
}

int etnaviv_gpu_wait_fence_interruptible(struct etnaviv_gpu *gpu,
	u32 fence, struct timespec *timeout)
{
	int ret;

	if (fence_after(fence, gpu->submitted_fence)) {
		DRM_ERROR("waiting on invalid fence: %u (of %u)\n",
				fence, gpu->submitted_fence);
		return -EINVAL;
	}

	if (!timeout) {
		/* No timeout was requested: just test for completion */
		ret = fence_completed(gpu, fence) ? 0 : -EBUSY;
	} else {
		unsigned long remaining = etnaviv_timeout_to_jiffies(timeout);

		ret = wait_event_interruptible_timeout(gpu->fence_event,
						fence_completed(gpu, fence),
						remaining);
		if (ret == 0) {
			DBG("timeout waiting for fence: %u (retired: %u completed: %u)",
				fence, gpu->retired_fence,
				gpu->completed_fence);
			ret = -ETIMEDOUT;
		} else if (ret != -ERESTARTSYS) {
			ret = 0;
		}
	}

	return ret;
}

/*
 * Wait for an object to become inactive.  This, on it's own, is not race
 * free: the object is moved by the retire worker off the active list, and
 * then the iova is put.  Moreover, the object could be re-submitted just
 * after we notice that it's become inactive.
 *
 * Although the retirement happens under the struct_mutex, we don't want
 * to hold that lock in this function.  Instead, the caller is responsible
 * for ensuring that the retire worker has finished (which will happen, eg,
 * when we unreference the object, an action which takes the struct_mutex.)
 */
int etnaviv_gpu_wait_obj_inactive(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *etnaviv_obj, struct timespec *timeout)
{
	unsigned long remaining;
	long ret;

	if (!timeout)
		return !is_active(etnaviv_obj) ? 0 : -EBUSY;

	remaining = etnaviv_timeout_to_jiffies(timeout);

	ret = wait_event_interruptible_timeout(gpu->fence_event,
					       !is_active(etnaviv_obj),
					       remaining);
	if (ret > 0)
		return 0;
	else if (ret == -ERESTARTSYS)
		return -ERESTARTSYS;
	else
		return -ETIMEDOUT;
}

int etnaviv_gpu_pm_get_sync(struct etnaviv_gpu *gpu)
{
	return pm_runtime_get_sync(gpu->dev);
}

void etnaviv_gpu_pm_put(struct etnaviv_gpu *gpu)
{
	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);
}

/* add bo's to gpu's ring, and kick gpu: */
int etnaviv_gpu_submit(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_submit *submit, struct etnaviv_file_private *ctx)
{
	struct drm_device *dev = gpu->drm;
	struct etnaviv_drm_private *priv = dev->dev_private;
	unsigned int event, i;
	int ret;

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0)
		return ret;

	/*
	 * TODO
	 *
	 * - flush
	 * - data endian
	 * - prefetch
	 *
	 */

	event = event_alloc(gpu);
	if (unlikely(event == ~0U)) {
		DRM_ERROR("no free event\n");
		pm_runtime_put_autosuspend(gpu->dev);
		return -EBUSY;
	}

	submit->fence = ++priv->next_fence;

	gpu->submitted_fence = submit->fence;

	if (gpu->lastctx != ctx) {
		gpu->mmu->need_flush = true;
		gpu->switch_context = true;
		gpu->lastctx = ctx;
	}

	etnaviv_buffer_queue(gpu, event, submit);

	for (i = 0; i < submit->nr_bos; i++) {
		struct etnaviv_gem_object *etnaviv_obj = submit->bos[i].obj;

		/* can't happen yet.. but when we add 2d support we'll have
		 * to deal w/ cross-ring synchronization:
		 */
		WARN_ON(is_active(etnaviv_obj) && (etnaviv_obj->gpu != gpu));

		if (!is_active(etnaviv_obj)) {
			u32 iova;

			/* ring takes a reference to the bo and iova: */
			drm_gem_object_reference(&etnaviv_obj->base);
			etnaviv_gem_get_iova_locked(gpu, &etnaviv_obj->base,
						    &iova);
		}

		if (submit->bos[i].flags & (ETNA_SUBMIT_BO_READ |
					    ETNA_SUBMIT_BO_WRITE))
			etnaviv_gem_move_to_active(&etnaviv_obj->base, gpu,
						   submit->bos[i].flags,
						   submit->fence);
	}
	hangcheck_timer_reset(gpu);

	return 0;
}

/*
 * Init/Cleanup:
 */
static irqreturn_t irq_handler(int irq, void *data)
{
	struct etnaviv_gpu *gpu = data;
	irqreturn_t ret = IRQ_NONE;

	u32 intr = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);

	if (intr != 0) {
		int event;

		pm_runtime_mark_last_busy(gpu->dev);

		dev_dbg(gpu->dev, "intr 0x%08x\n", intr);

		if (intr & VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR) {
			dev_err(gpu->dev, "AXI bus error\n");
			intr &= ~VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR;
		}

		while ((event = ffs(intr)) != 0) {
			event -= 1;

			intr &= ~(1 << event);

			dev_dbg(gpu->dev, "event %u\n", event);
			/*
			 * Events can be processed out of order.  Eg,
			 * - allocate and queue event 0
			 * - allocate event 1
			 * - event 0 completes, we process it
			 * - allocate and queue event 0
			 * - event 1 and event 0 complete
			 * we can end up processing event 0 first, then 1.
			 */
			if (fence_after(gpu->event[event].fence,
					gpu->completed_fence))
				gpu->completed_fence = gpu->event[event].fence;
			event_free(gpu, event);

			/*
			 * We need to balance the runtime PM count caused by
			 * each submission.  Upon submission, we increment
			 * the runtime PM counter, and allocate one event.
			 * So here, we put the runtime PM count for each
			 * completed event.
			 */
			pm_runtime_put_autosuspend(gpu->dev);
		}

		/* Retire the buffer objects in a work */
		etnaviv_queue_work(gpu->drm, &gpu->retire_work);

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int etnaviv_gpu_clk_enable(struct etnaviv_gpu *gpu)
{
	int ret;

	ret = enable_clk(gpu);
	if (ret)
		return ret;

	ret = enable_axi(gpu);
	if (ret) {
		disable_clk(gpu);
		return ret;
	}

	return 0;
}

static int etnaviv_gpu_clk_disable(struct etnaviv_gpu *gpu)
{
	int ret;

	ret = disable_axi(gpu);
	if (ret)
		return ret;

	ret = disable_clk(gpu);
	if (ret)
		return ret;

	return 0;
}

static int etnaviv_gpu_hw_suspend(struct etnaviv_gpu *gpu)
{
	if (gpu->buffer) {
		unsigned long timeout;

		/* Replace the last WAIT with END */
		etnaviv_buffer_end(gpu);

		/*
		 * We know that only the FE is busy here, this should
		 * happen quickly (as the WAIT is only 200 cycles).  If
		 * we fail, just warn and continue.
		 */
		timeout = jiffies + msecs_to_jiffies(100);
		do {
			u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

			if ((idle & gpu->idle_mask) == gpu->idle_mask)
				break;

			if (time_is_before_jiffies(timeout)) {
				dev_warn(gpu->dev,
					 "timed out waiting for idle: idle=0x%x\n",
					 idle);
				break;
			}

			udelay(5);
		} while (1);
	}

	return etnaviv_gpu_clk_disable(gpu);
}

static int etnaviv_gpu_hw_resume(struct etnaviv_gpu *gpu)
{
	struct drm_device *drm = gpu->drm;
	u32 clock;
	int ret;

	ret = mutex_lock_killable(&drm->struct_mutex);
	if (ret)
		return ret;

	clock = VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS |
		VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(0x40);

	etnaviv_gpu_load_clock(gpu, clock);
	etnaviv_gpu_hw_init(gpu);

	gpu->switch_context = true;

	mutex_unlock(&drm->struct_mutex);

	return 0;
}

static int etnaviv_gpu_bind(struct device *dev, struct device *master,
	void *data)
{
	struct drm_device *drm = data;
	struct etnaviv_drm_private *priv = drm->dev_private;
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	int ret;

#ifdef CONFIG_PM
	ret = pm_runtime_get_sync(gpu->dev);
#else
	ret = etnaviv_gpu_clk_enable(gpu);
#endif
	if (ret < 0)
		return ret;

	gpu->drm = drm;

	INIT_LIST_HEAD(&gpu->active_list);
	INIT_WORK(&gpu->retire_work, retire_worker);
	INIT_WORK(&gpu->recover_work, recover_worker);
	init_waitqueue_head(&gpu->fence_event);

	setup_timer(&gpu->hangcheck_timer, hangcheck_handler,
			(unsigned long)gpu);

	priv->gpu[priv->num_gpus++] = gpu;

	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);

	return 0;
}

static void etnaviv_gpu_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);

	DBG("%s", dev_name(gpu->dev));

	hangcheck_disable(gpu);

	WARN_ON(!list_empty(&gpu->active_list));

#ifdef CONFIG_PM
	pm_runtime_get_sync(gpu->dev);
	pm_runtime_put_sync_suspend(gpu->dev);
#else
	etnaviv_gpu_hw_suspend(gpu);
#endif

	if (gpu->buffer) {
		drm_gem_object_unreference_unlocked(gpu->buffer);
		gpu->buffer = NULL;
	}

	if (gpu->mmu) {
		etnaviv_iommu_destroy(gpu->mmu);
		gpu->mmu = NULL;
	}

	gpu->drm = NULL;
}

static const struct component_ops gpu_ops = {
	.bind = etnaviv_gpu_bind,
	.unbind = etnaviv_gpu_unbind,
};

static const struct of_device_id etnaviv_gpu_match[] = {
	{
		.compatible = "vivante,gc"
	},
	{ /* sentinel */ }
};

static int etnaviv_gpu_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct etnaviv_gpu *gpu;
	int err = 0;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->dev = &pdev->dev;

	/*
	 * Set the GPU base address to the start of physical memory.  This
	 * ensures that if we have up to 2GB, the v1 MMU can address the
	 * highest memory.  This is important as command buffers may be
	 * allocated outside of this limit.
	 */
	gpu->memory_base = PHYS_OFFSET;

	/* Map registers: */
	gpu->mmio = etnaviv_ioremap(pdev, NULL, dev_name(gpu->dev));
	if (IS_ERR(gpu->mmio))
		return PTR_ERR(gpu->mmio);

	/* Get Interrupt: */
	gpu->irq = platform_get_irq(pdev, 0);
	if (gpu->irq < 0) {
		err = gpu->irq;
		dev_err(dev, "failed to get irq: %d\n", err);
		goto fail;
	}

	err = devm_request_irq(&pdev->dev, gpu->irq, irq_handler, 0,
			       dev_name(gpu->dev), gpu);
	if (err) {
		dev_err(dev, "failed to request IRQ%u: %d\n", gpu->irq, err);
		goto fail;
	}

	/* Get Clocks: */
	gpu->clk_bus = devm_clk_get(&pdev->dev, "bus");
	DBG("clk_bus: %p", gpu->clk_bus);
	if (IS_ERR(gpu->clk_bus))
		gpu->clk_bus = NULL;

	gpu->clk_core = devm_clk_get(&pdev->dev, "core");
	DBG("clk_core: %p", gpu->clk_core);
	if (IS_ERR(gpu->clk_core))
		gpu->clk_core = NULL;

	gpu->clk_shader = devm_clk_get(&pdev->dev, "shader");
	DBG("clk_shader: %p", gpu->clk_shader);
	if (IS_ERR(gpu->clk_shader))
		gpu->clk_shader = NULL;

	/* TODO: figure out max mapped size */
	dev_set_drvdata(dev, gpu);

	/*
	 * We treat the device as initially suspended.  The runtime PM
	 * autosuspend delay is rather arbitary: no measurements have
	 * yet been performed to determine an appropriate value.
	 */
	pm_runtime_use_autosuspend(gpu->dev);
	pm_runtime_set_autosuspend_delay(gpu->dev, 200);
	pm_runtime_enable(gpu->dev);

	err = component_add(&pdev->dev, &gpu_ops);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register component: %d\n", err);
		goto fail;
	}

	return 0;

fail:
	return err;
}

static int etnaviv_gpu_platform_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &gpu_ops);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int etnaviv_gpu_rpm_suspend(struct device *dev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	u32 idle, mask;

	/* If we have outstanding fences, we're not idle */
	if (gpu->completed_fence != gpu->submitted_fence)
		return -EBUSY;

	/* Check whether the hardware (except FE) is idle */
	mask = gpu->idle_mask & ~VIVS_HI_IDLE_STATE_FE;
	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE) & mask;
	if (idle != mask)
		return -EBUSY;

	return etnaviv_gpu_hw_suspend(gpu);
}

static int etnaviv_gpu_rpm_resume(struct device *dev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	int ret;

	/* We must never runtime-PM resume holding struct_mutex */
	if (gpu->drm && WARN_ON_ONCE(mutex_is_locked(&gpu->drm->struct_mutex)))
		return -EDEADLK;

	ret = etnaviv_gpu_clk_enable(gpu);
	if (ret)
		return ret;

	/* Re-initialise the basic hardware state */
	if (gpu->drm && gpu->buffer) {
		ret = etnaviv_gpu_hw_resume(gpu);
		if (ret) {
			etnaviv_gpu_clk_disable(gpu);
			return ret;
		}
	}

	return 0;
}
#endif

static const struct dev_pm_ops etnaviv_gpu_pm_ops = {
	SET_RUNTIME_PM_OPS(etnaviv_gpu_rpm_suspend, etnaviv_gpu_rpm_resume,
			   NULL)
};

struct platform_driver etnaviv_gpu_driver = {
	.driver = {
		.name = "etnaviv-gpu",
		.owner = THIS_MODULE,
		.pm = &etnaviv_gpu_pm_ops,
		.of_match_table = etnaviv_gpu_match,
	},
	.probe = etnaviv_gpu_platform_probe,
	.remove = etnaviv_gpu_platform_remove,
	.id_table = gpu_ids,
};
