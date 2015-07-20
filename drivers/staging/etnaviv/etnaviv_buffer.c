/*
 * Copyright (C) 2014 Etnaviv Project
 * Author: Christian Gmeiner <christian.gmeiner@gmail.com>
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

#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"

#include "common.xml.h"
#include "state.xml.h"
#include "cmdstream.xml.h"

/*
 * Command Buffer helper:
 */


static inline void OUT(struct etnaviv_gem_object *buffer, u32 data)
{
	u32 *vaddr = (u32 *)buffer->vaddr;

	BUG_ON(buffer->offset >= buffer->base.size / sizeof(*vaddr));

	vaddr[buffer->offset++] = data;
}

static inline void CMD_LOAD_STATE(struct etnaviv_gem_object *buffer,
	u32 reg, u32 value)
{
	u32 index = reg >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR;

	buffer->offset = ALIGN(buffer->offset, 2);

	/* write a register via cmd stream */
	OUT(buffer, VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
		    VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
		    VIV_FE_LOAD_STATE_HEADER_OFFSET(index));
	OUT(buffer, value);
}

static inline void CMD_END(struct etnaviv_gem_object *buffer)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_END_HEADER_OP_END);
}

static inline void CMD_WAIT(struct etnaviv_gem_object *buffer)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_WAIT_HEADER_OP_WAIT | 200);
}

static inline void CMD_LINK(struct etnaviv_gem_object *buffer,
	u16 prefetch, u32 address)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_LINK_HEADER_OP_LINK |
		    VIV_FE_LINK_HEADER_PREFETCH(prefetch));
	OUT(buffer, address);
}

static inline void CMD_STALL(struct etnaviv_gem_object *buffer,
	u32 from, u32 to)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_STALL_HEADER_OP_STALL);
	OUT(buffer, VIV_FE_STALL_TOKEN_FROM(from) | VIV_FE_STALL_TOKEN_TO(to));
}

static void etnaviv_cmd_select_pipe(struct etnaviv_gem_object *buffer, u8 pipe)
{
	u32 flush;
	u32 stall;

	if (pipe == ETNA_PIPE_2D)
		flush = VIVS_GL_FLUSH_CACHE_DEPTH | VIVS_GL_FLUSH_CACHE_COLOR;
	else
		flush = VIVS_GL_FLUSH_CACHE_TEXTURE;

	stall = VIVS_GL_SEMAPHORE_TOKEN_FROM(SYNC_RECIPIENT_FE) |
		VIVS_GL_SEMAPHORE_TOKEN_TO(SYNC_RECIPIENT_PE);

	CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE, flush);
	CMD_LOAD_STATE(buffer, VIVS_GL_SEMAPHORE_TOKEN, stall);

	CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);

	CMD_LOAD_STATE(buffer, VIVS_GL_PIPE_SELECT,
		       VIVS_GL_PIPE_SELECT_PIPE(pipe));
}

static u32 gpu_va(struct etnaviv_gpu *gpu, struct etnaviv_gem_object *obj)
{
	return obj->paddr - gpu->memory_base;
}

static void etnaviv_buffer_dump(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *obj, u32 off, u32 len)
{
	u32 size = obj->base.size;
	u32 *ptr = obj->vaddr + off;

	dev_info(gpu->dev, "virt %p phys 0x%08x free 0x%08x\n",
			ptr, gpu_va(gpu, obj) + off, size - len * 4 - off);

	print_hex_dump(KERN_INFO, "cmd ", DUMP_PREFIX_OFFSET, 16, 4,
			ptr, len * 4, 0);
}

u32 etnaviv_buffer_init(struct etnaviv_gpu *gpu)
{
	struct etnaviv_gem_object *buffer = to_etnaviv_bo(gpu->buffer);

	/* initialize buffer */
	buffer->offset = 0;
	buffer->is_ring_buffer = true;

	CMD_WAIT(buffer);
	CMD_LINK(buffer, 2, gpu_va(gpu, buffer) + ((buffer->offset - 1) * 4));

	return buffer->offset;
}

void etnaviv_buffer_end(struct etnaviv_gpu *gpu)
{
	struct etnaviv_gem_object *buffer = to_etnaviv_bo(gpu->buffer);

	/* Replace the last WAIT with an END */
	buffer->offset -= 4;

	CMD_END(buffer);
	mb();
}

void etnaviv_buffer_queue(struct etnaviv_gpu *gpu, unsigned int event,
	struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gem_object *buffer = to_etnaviv_bo(gpu->buffer);
	struct etnaviv_gem_object *cmd;
	u32 *lw = buffer->vaddr + ((buffer->offset - 4) * 4);
	u32 back, link_target, link_size, reserve_size, extra_size = 0;
	u32 i;

	if (drm_debug & DRM_UT_DRIVER)
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);

	/*
	 * If we need to flush the MMU prior to submitting this buffer, we
	 * will need to append a mmu flush load state, followed by a new
	 * link to this buffer - a total of four additional words.
	 */
	if (gpu->mmu->need_flush || gpu->switch_context) {
		/* link command */
		extra_size += 2;
		/* flush command */
		if (gpu->mmu->need_flush)
			extra_size += 2;
		/* pipe switch commands */
		if (gpu->switch_context)
			extra_size += 8;
	}

	reserve_size = 6 + extra_size;

	/*
	 * if we are going to completely overflow the buffer, we need to wrap.
	 */
	if (buffer->offset + reserve_size >
	    buffer->base.size / sizeof(u32))
		buffer->offset = 0;

	/* save offset back into main buffer */
	back = buffer->offset + reserve_size - 6;
	link_target = gpu_va(gpu, buffer) + buffer->offset * 4;
	link_size = 6;

	/* Skip over any extra instructions */
	link_target += extra_size * sizeof(u32);

	/* update offset for every cmd stream */
	for (i = submit->nr_cmds; i--; ) {
		cmd = submit->cmd[i].obj;

		cmd->offset = submit->cmd[i].offset + submit->cmd[i].size;

		if (drm_debug & DRM_UT_DRIVER)
			pr_info("stream link from buffer %u to 0x%08x @ 0x%08x %p\n",
				i, link_target,
				gpu_va(gpu, cmd) + cmd->offset * 4,
				cmd->vaddr + cmd->offset * 4);

		/* jump back from last cmd to main buffer */
		CMD_LINK(cmd, link_size, link_target);

		/* update the size */
		submit->cmd[i].size = cmd->offset - submit->cmd[i].offset;

		link_target = gpu_va(gpu, cmd) + submit->cmd[i].offset * 4;
		link_size = submit->cmd[i].size * 2;
	}

	if (drm_debug & DRM_UT_DRIVER) {
		for (i = 0; i < submit->nr_cmds; i++) {
			struct etnaviv_gem_object *obj = submit->cmd[i].obj;

			etnaviv_buffer_dump(gpu, obj, submit->cmd[i].offset,
					submit->cmd[i].size);
		}

		pr_info("link op: %p\n", lw);
		pr_info("link addr: %p\n", lw + 1);
		pr_info("addr: 0x%08x\n", link_target);
		pr_info("back: 0x%08x\n", gpu_va(gpu, buffer) + (back * 4));
		pr_info("event: %d\n", event);
	}

	if (gpu->mmu->need_flush || gpu->switch_context) {
		u32 new_target = gpu_va(gpu, buffer) + buffer->offset *
					sizeof(u32);

		if (gpu->mmu->need_flush) {
			/* Add the MMU flush */
			CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_MMU,
				       VIVS_GL_FLUSH_MMU_FLUSH_FEMMU |
				       VIVS_GL_FLUSH_MMU_FLUSH_UNK1 |
				       VIVS_GL_FLUSH_MMU_FLUSH_UNK2 |
				       VIVS_GL_FLUSH_MMU_FLUSH_PEMMU |
				       VIVS_GL_FLUSH_MMU_FLUSH_UNK4);

			gpu->mmu->need_flush = false;
		}

		if (gpu->switch_context) {
			etnaviv_cmd_select_pipe(buffer, submit->exec_state);
			gpu->switch_context = false;
		}

		/* And the link to the first buffer */
		CMD_LINK(buffer, link_size, link_target);

		/* Update the link target to point to above instructions */
		link_target = new_target;
		link_size = extra_size;
	}

	/* Save the event and buffer position of the new event trigger */
	gpu->event[event].fence = submit->fence;

	/* trigger event */
	CMD_LOAD_STATE(buffer, VIVS_GL_EVENT, VIVS_GL_EVENT_EVENT_ID(event) |
		       VIVS_GL_EVENT_FROM_PE);

	/* append WAIT/LINK to main buffer */
	CMD_WAIT(buffer);
	CMD_LINK(buffer, 2, gpu_va(gpu, buffer) + ((buffer->offset - 1) * 4));

	/* Change WAIT into a LINK command; write the address first. */
	*(lw + 1) = link_target;
	mb();
	*(lw) = VIV_FE_LINK_HEADER_OP_LINK |
		VIV_FE_LINK_HEADER_PREFETCH(link_size);
	mb();

	if (drm_debug & DRM_UT_DRIVER)
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);
}
