// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/device.h>

#include "ipu3.h"
#include "ipu3-css-pool.h"
#include "ipu3-dmamap.h"

int ipu3_css_dma_buffer_resize(struct imgu_device *imgu,
			       struct ipu3_css_map *map, size_t size)
{
	if (map->size < size && map->vaddr) {
		dev_warn(&imgu->pci_dev->dev, "dma buf resized from %zu to %zu",
			 map->size, size);

		ipu3_dmamap_free(imgu, map);
		if (!ipu3_dmamap_alloc(imgu, map, size))
			return -ENOMEM;
	}

	return 0;
}

void ipu3_css_pool_cleanup(struct imgu_device *imgu, struct ipu3_css_pool *pool)
{
	unsigned int i;

	for (i = 0; i < IPU3_CSS_POOL_SIZE; i++)
		ipu3_dmamap_free(imgu, &pool->entry[i].param);
}

int ipu3_css_pool_init(struct imgu_device *imgu, struct ipu3_css_pool *pool,
		       size_t size)
{
	unsigned int i;

	for (i = 0; i < IPU3_CSS_POOL_SIZE; i++) {
		/*
		 * entry[i].framenum is initialized to INT_MIN so that
		 * ipu3_css_pool_check() can treat it as usesable slot.
		 */
		pool->entry[i].framenum = INT_MIN;

		if (size == 0) {
			pool->entry[i].param.vaddr = NULL;
			continue;
		}

		if (!ipu3_dmamap_alloc(imgu, &pool->entry[i].param, size))
			goto fail;
	}

	pool->last = IPU3_CSS_POOL_SIZE;

	return 0;

fail:
	ipu3_css_pool_cleanup(imgu, pool);
	return -ENOMEM;
}

/*
 * Allocate a new parameter from pool at frame number `framenum'.
 * Release the oldest entry in the pool to make space for the new entry.
 */
void ipu3_css_pool_get(struct ipu3_css_pool *pool, long framenum)
{
	/* Get the oldest entry */
	int n = (pool->last + 1) % IPU3_CSS_POOL_SIZE;

	pool->entry[n].framenum = framenum;
	pool->last = n;
}

/*
 * Undo, for all practical purposes, the effect of pool_get().
 */
void ipu3_css_pool_put(struct ipu3_css_pool *pool)
{
	pool->entry[pool->last].framenum = INT_MIN;
	pool->last = (pool->last + IPU3_CSS_POOL_SIZE - 1) % IPU3_CSS_POOL_SIZE;
}

/*
 * Return the nth entry from last, if that entry has no frame stored,
 * return a null map instead to indicate frame not available for the entry.
 */
const struct ipu3_css_map *
ipu3_css_pool_last(struct ipu3_css_pool *pool, unsigned int n)
{
	static const struct ipu3_css_map null_map = { 0 };
	int i = (pool->last + IPU3_CSS_POOL_SIZE - n) % IPU3_CSS_POOL_SIZE;

	WARN_ON(n >= IPU3_CSS_POOL_SIZE);

	if (pool->entry[i].framenum < 0)
		return &null_map;

	return &pool->entry[i].param;
}
