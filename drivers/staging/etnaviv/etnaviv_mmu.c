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

#include "etnaviv_drv.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"

static int etnaviv_fault_handler(struct iommu_domain *iommu, struct device *dev,
		unsigned long iova, int flags, void *arg)
{
	DBG("*** fault: iova=%08lx, flags=%d", iova, flags);
	return 0;
}

int etnaviv_iommu_map(struct etnaviv_iommu *iommu, u32 iova,
		struct sg_table *sgt, unsigned len, int prot)
{
	struct iommu_domain *domain = iommu->domain;
	struct scatterlist *sg;
	unsigned int da = iova;
	unsigned int i, j;
	int ret;

	if (!domain || !sgt)
		return -EINVAL;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		u32 pa = sg_dma_address(sg) - sg->offset;
		size_t bytes = sg_dma_len(sg) + sg->offset;

		VERB("map[%d]: %08x %08x(%zx)", i, iova, pa, bytes);

		ret = iommu_map(domain, da, pa, bytes, prot);
		if (ret)
			goto fail;

		da += bytes;
	}

	return 0;

fail:
	da = iova;

	for_each_sg(sgt->sgl, sg, i, j) {
		size_t bytes = sg_dma_len(sg) + sg->offset;

		iommu_unmap(domain, da, bytes);
		da += bytes;
	}
	return ret;
}

int etnaviv_iommu_unmap(struct etnaviv_iommu *iommu, u32 iova,
		struct sg_table *sgt, unsigned len)
{
	struct iommu_domain *domain = iommu->domain;
	struct scatterlist *sg;
	unsigned int da = iova;
	int i;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		size_t bytes = sg_dma_len(sg) + sg->offset;
		size_t unmapped;

		unmapped = iommu_unmap(domain, da, bytes);
		if (unmapped < bytes)
			return unmapped;

		VERB("unmap[%d]: %08x(%zx)", i, iova, bytes);

		BUG_ON(!PAGE_ALIGNED(bytes));

		da += bytes;
	}

	return 0;
}

int etnaviv_iommu_map_gem(struct etnaviv_iommu *mmu,
	struct etnaviv_gem_object *etnaviv_obj, u32 memory_base,
	struct etnaviv_vram_mapping **out_mapping)
{
	struct etnaviv_drm_private *priv = etnaviv_obj->base.dev->dev_private;
	struct sg_table *sgt = etnaviv_obj->sgt;
	struct etnaviv_vram_mapping *mapping, *free = NULL;
	struct drm_mm_node *node;
	int ret;

	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;

	INIT_LIST_HEAD(&mapping->scan_node);
	mapping->object = etnaviv_obj;
	mapping->mmu = mmu;

	/* v1 MMU can optimize single entry (contiguous) scatterlists */
	if (sgt->nents == 1 && !(etnaviv_obj->flags & ETNA_BO_FORCE_MMU)) {
		u32 iova;

		iova = sg_dma_address(sgt->sgl) - memory_base;
		if (iova < 0x80000000 - sg_dma_len(sgt->sgl)) {
			mapping->iova = iova;
			list_add_tail(&mapping->obj_node,
				      &etnaviv_obj->vram_list);
			if (out_mapping)
				*out_mapping = mapping;
			return 0;
		}
	}

	node = &mapping->vram_node;
	while (1) {
		struct etnaviv_gem_object *o;
		struct etnaviv_vram_mapping *m, *n;
		struct list_head list;
		bool found;

		ret = drm_mm_insert_node_in_range(&mmu->mm, node,
			etnaviv_obj->base.size, 0, mmu->last_iova, ~0UL,
			DRM_MM_SEARCH_DEFAULT);

		if (ret != -ENOSPC)
			break;

		/*
		 * If we did not search from the start of the MMU region,
		 * try again in case there are free slots.
		 */
		if (mmu->last_iova) {
			mmu->last_iova = 0;
			mmu->need_flush = true;
			continue;
		}

		/* Try to retire some entries */
		drm_mm_init_scan(&mmu->mm, etnaviv_obj->base.size, 0, 0);

		found = 0;
		INIT_LIST_HEAD(&list);
		list_for_each_entry(o, &priv->inactive_list, mm_list) {
			free = etnaviv_gem_get_vram_mapping(o, mmu);
			if (!free)
				continue;

			/*
			 * If this vram node has not been used, skip this.
			 */
			if (!free->vram_node.mm)
				continue;

			/*
			 * If it's on the submit list, then it is part of
			 * a submission, and we want to keep its entry.
			 */
			if (!list_empty(&o->submit_entry))
				continue;

			list_add(&free->scan_node, &list);
			if (drm_mm_scan_add_block(&free->vram_node)) {
				found = true;
				break;
			}
		}

		if (!found) {
			/* Nothing found, clean up and fail */
			list_for_each_entry_safe(m, n, &list, scan_node)
				BUG_ON(drm_mm_scan_remove_block(&m->vram_node));
			break;
		}

		/*
		 * drm_mm does not allow any other operations while
		 * scanning, so we have to remove all blocks first.
		 * If drm_mm_scan_remove_block() returns false, we
		 * can leave the block pinned.
		 */
		list_for_each_entry_safe(m, n, &list, scan_node)
			if (!drm_mm_scan_remove_block(&m->vram_node))
				list_del_init(&m->scan_node);

		list_for_each_entry_safe(m, n, &list, scan_node) {
			list_del_init(&m->scan_node);
			etnaviv_iommu_unmap_gem(m);
		}

		/*
		 * We removed enough mappings so that the new allocation will
		 * succeed.  Ensure that the MMU will be flushed and retry
		 * the allocation one more time.
		 */
		mmu->need_flush = true;
	}

	if (ret < 0) {
		kfree(mapping);
		return ret;
	}

	mmu->last_iova = node->start + etnaviv_obj->base.size;
	mapping->iova = node->start;
	ret = etnaviv_iommu_map(mmu, node->start, sgt, etnaviv_obj->base.size,
				IOMMU_READ | IOMMU_WRITE);

	if (ret < 0) {
		drm_mm_remove_node(node);
		kfree(mapping);
		return ret;
	}

	list_add_tail(&mapping->obj_node, &etnaviv_obj->vram_list);
	if (out_mapping)
		*out_mapping = mapping;

	return ret;
}

void etnaviv_iommu_unmap_gem(struct etnaviv_vram_mapping *mapping)
{
	struct etnaviv_iommu *mmu;
	struct etnaviv_gem_object *etnaviv_obj;

	if (!mapping)
		return;

	mmu = mapping->mmu;

	/* If the vram node is on the mm, unmap and remove the node */
	if (mapping->vram_node.mm == &mmu->mm) {
		etnaviv_obj = mapping->object;
		etnaviv_iommu_unmap(mmu, mapping->vram_node.start,
				    etnaviv_obj->sgt, etnaviv_obj->base.size);
		drm_mm_remove_node(&mapping->vram_node);
	}

	list_del(&mapping->obj_node);
	kfree(mapping);
}

void etnaviv_iommu_destroy(struct etnaviv_iommu *mmu)
{
	drm_mm_takedown(&mmu->mm);
	iommu_domain_free(mmu->domain);
	kfree(mmu);
}

struct etnaviv_iommu *etnaviv_iommu_new(struct device *dev,
	struct iommu_domain *domain, enum etnaviv_iommu_version version)
{
	struct etnaviv_iommu *mmu;

	mmu = kzalloc(sizeof(*mmu), GFP_KERNEL);
	if (!mmu)
		return ERR_PTR(-ENOMEM);

	mmu->domain = domain;
	mmu->dev = dev;
	mmu->version = version;

	drm_mm_init(&mmu->mm, domain->geometry.aperture_start,
		    domain->geometry.aperture_end -
		      domain->geometry.aperture_start + 1);

	iommu_set_fault_handler(domain, etnaviv_fault_handler, dev);

	return mmu;
}
