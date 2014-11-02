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

#ifndef __ETNAVIV_MMU_H__
#define __ETNAVIV_MMU_H__

#include <linux/iommu.h>

struct etnaviv_iommu {
	struct drm_device *dev;
	struct iommu_domain *domain;
	bool need_flush;
};

int etnaviv_iommu_attach(struct etnaviv_iommu *iommu, const char **names,
	int cnt);
int etnaviv_iommu_map(struct etnaviv_iommu *iommu, uint32_t iova,
	struct sg_table *sgt, unsigned len, int prot);
int etnaviv_iommu_unmap(struct etnaviv_iommu *iommu, uint32_t iova,
	struct sg_table *sgt, unsigned len);
void etnaviv_iommu_destroy(struct etnaviv_iommu *iommu);

struct etnaviv_iommu *etnaviv_iommu_new(struct drm_device *dev,
	struct iommu_domain *domain);

#endif /* __ETNAVIV_MMU_H__ */
