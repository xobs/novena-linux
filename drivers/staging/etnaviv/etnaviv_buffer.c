/*
 * Copyright (C) 2014 2014 Etnaviv Project
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

#include "common.xml.h"
#include "state.xml.h"
#include "cmdstream.xml.h"

/*
 * Command Buffer helper:
 */


static inline void OUT(struct etnaviv_gem_object *buffer, uint32_t data)
{
	u32 *vaddr = (u32 *)buffer->vaddr;
	BUG_ON(buffer->offset * sizeof(*vaddr) >= buffer->base.size);

	vaddr[buffer->offset++] = data;
}

static inline void CMD_LOAD_STATE(struct etnaviv_gem_object *buffer, u32 reg, u32 value)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	/* write a register via cmd stream */
	OUT(buffer, VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			VIV_FE_LOAD_STATE_HEADER_OFFSET(reg >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR));
	OUT(buffer, value);
}

static inline void CMD_LOAD_STATES(struct etnaviv_gem_object *buffer, u32 reg, u16 count, u32 *values)
{
	u16 i;
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | VIV_FE_LOAD_STATE_HEADER_COUNT(count) |
			VIV_FE_LOAD_STATE_HEADER_OFFSET(reg >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR));

	for (i = 0; i < count; i++)
		OUT(buffer, values[i]);
}

static inline void CMD_END(struct etnaviv_gem_object *buffer)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_END_HEADER_OP_END);
}

static inline void CMD_NOP(struct etnaviv_gem_object *buffer)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_NOP_HEADER_OP_NOP);
}

static inline void CMD_WAIT(struct etnaviv_gem_object *buffer)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_WAIT_HEADER_OP_WAIT | 200);
}

static inline void CMD_LINK(struct etnaviv_gem_object *buffer, u16 prefetch, u32 address)
{
	buffer->offset = ALIGN(buffer->offset, 2);

	OUT(buffer, VIV_FE_LINK_HEADER_OP_LINK | VIV_FE_LINK_HEADER_PREFETCH(prefetch));
	OUT(buffer, address);
}

static inline void CMD_STALL(struct etnaviv_gem_object *buffer, u32 from, u32 to)
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

	CMD_LOAD_STATE(buffer, VIVS_GL_PIPE_SELECT, VIVS_GL_PIPE_SELECT_PIPE(pipe));
}

static void etnaviv_buffer_dump(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *obj, u32 len)
{
	u32 size = obj->base.size;
	u32 *ptr = obj->vaddr;

	dev_info(gpu->dev->dev, "virt %p phys 0x%08x free 0x%08x\n",
			obj->vaddr, obj->paddr, size - len * 4);

	print_hex_dump(KERN_INFO, "cmd ", DUMP_PREFIX_OFFSET, 16, 4,
			ptr, len * 4, 0);
}

u32 etnaviv_buffer_init(struct etnaviv_gpu *gpu)
{
	struct etnaviv_gem_object *buffer = to_etnaviv_bo(gpu->buffer);

	/* initialize buffer */
	buffer->offset = 0;

	etnaviv_cmd_select_pipe(buffer, gpu->pipe);

	CMD_WAIT(buffer);
	CMD_LINK(buffer, 2, buffer->paddr + ((buffer->offset - 1) * 4));

	return buffer->offset;
}

void etnaviv_buffer_queue(struct etnaviv_gpu *gpu, unsigned int event, struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gem_object *buffer = to_etnaviv_bo(gpu->buffer);
	struct etnaviv_gem_object *cmd;
	u32 *lw = buffer->vaddr + ((buffer->offset - 4) * 4);
	u32 back;
	u32 i;

	etnaviv_buffer_dump(gpu, buffer, 0x50);

	/* save offset back into main buffer */
	back = buffer->offset;

	/* trigger event */
	CMD_LOAD_STATE(buffer, VIVS_GL_EVENT, VIVS_GL_EVENT_EVENT_ID(event) | VIVS_GL_EVENT_FROM_PE);

	/* append WAIT/LINK to main buffer */
	CMD_WAIT(buffer);
	CMD_LINK(buffer, 2, buffer->paddr + ((buffer->offset - 1) * 4));

	/* update offset for every cmd stream */
	for (i = 0; i < submit->nr_cmds; i++)
		submit->cmd[i].obj->offset = submit->cmd[i].size;

	/* TODO: inter-connect all cmd buffers */

	/* jump back from last cmd to main buffer */
	cmd = submit->cmd[submit->nr_cmds - 1].obj;
	CMD_LINK(cmd, 4, buffer->paddr + (back * 4));

	printk(KERN_ERR "stream link @ 0x%08x\n", cmd->paddr + ((cmd->offset - 1) * 4));
	printk(KERN_ERR "stream link @ %p\n", cmd->vaddr + ((cmd->offset - 1) * 4));

	for (i = 0; i < submit->nr_cmds; i++) {
		struct etnaviv_gem_object *obj = submit->cmd[i].obj;

		/* TODO: remove later */
		if (unlikely(drm_debug & DRM_UT_CORE))
			etnaviv_buffer_dump(gpu, obj, submit->cmd[i].size);
	}

	/* change ll to NOP */
	printk(KERN_ERR "link op: %p\n", lw);
	printk(KERN_ERR "link addr: %p\n", lw + 1);
	printk(KERN_ERR "addr: 0x%08x\n", submit->cmd[0].obj->paddr);
	printk(KERN_ERR "back: 0x%08x\n", buffer->paddr + (back * 4));
	printk(KERN_ERR "event: %d\n", event);

	/* Change WAIT into a LINK command; write the address first. */
	i = VIV_FE_LINK_HEADER_OP_LINK | VIV_FE_LINK_HEADER_PREFETCH(submit->cmd[0].size * 2);
	*(lw + 1) = submit->cmd[0].obj->paddr;
	mb();
	*(lw)= i;
	mb();

	etnaviv_buffer_dump(gpu, buffer, 0x50);
}
