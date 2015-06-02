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

#include <linux/kernel.h>

#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"

#include "cmdstream.xml.h"

#define EXTRACT(val, field) (((val) & field##__MASK) >> field##__SHIFT)

static bool etnaviv_validate_load_state(struct etnaviv_gpu *gpu, u32 *buf,
	unsigned int state, unsigned int num)
{
	return true;
	if (0x1200 - state < num * 4)
		return false;
	if (0x1228 - state < num * 4)
		return false;
	if (0x1238 - state < num * 4)
		return false;
	if (0x1284 - state < num * 4)
		return false;
	if (0x128c - state < num * 4)
		return false;
	if (0x1304 - state < num * 4)
		return false;
	if (0x1310 - state < num * 4)
		return false;
	if (0x1318 - state < num * 4)
		return false;
	if (0x1280c - state < num * 4 + 0x0c)
		return false;
	if (0x128ac - state < num * 4 + 0x0c)
		return false;
	if (0x128cc - state < num * 4 + 0x0c)
		return false;
	if (0x1297c - state < num * 4 + 0x0c)
		return false;
	return true;
}

bool etnaviv_cmd_validate_one(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *obj, unsigned int offset, unsigned int size)
{
	u32 *start = obj->vaddr + offset * 4;
	u32 *buf = start;
	u32 *end = buf + size;

	while (buf < end) {
		u32 cmd = *buf;
		unsigned int len, n, off;
		unsigned int op = cmd >> 27;

		switch (op) {
		case FE_OPCODE_LOAD_STATE:
			n = EXTRACT(cmd, VIV_FE_LOAD_STATE_HEADER_COUNT);
			len = 1 + n;
			if (buf + len > end)
				break;

			off = EXTRACT(cmd, VIV_FE_LOAD_STATE_HEADER_OFFSET);
			if (!etnaviv_validate_load_state(gpu, buf + 1,
							 off * 4, n)) {
				dev_warn(gpu->dev, "%s: load state covers restricted state (0x%x-0x%x) at offset %tu\n",
					 __func__, off * 4, (off + n) * 4, buf - start);
				return false;
			}
			break;

		case FE_OPCODE_DRAW_2D:
			n = EXTRACT(cmd, VIV_FE_DRAW_2D_HEADER_COUNT);
			if (n == 0)
				n = 256;
			len = 2 + n * 2;
			break;

		case FE_OPCODE_DRAW_PRIMITIVES:
			len = 4;
			break;

		case FE_OPCODE_DRAW_INDEXED_PRIMITIVES:
			len = 6;
			break;

		case FE_OPCODE_NOP:
		case FE_OPCODE_STALL:
			len = 2;
			break;

		default:
			dev_err(gpu->dev, "%s: op %u not permitted at offset %tu\n",
				__func__, op, buf - start);
			return false;
		}

		buf += ALIGN(len, 2);
	}

	if (buf > end) {
		dev_err(gpu->dev, "%s: commands overflow end of buffer: %tu > %u\n",
			__func__, buf - start, size);
		return false;
	}

	return true;
}
