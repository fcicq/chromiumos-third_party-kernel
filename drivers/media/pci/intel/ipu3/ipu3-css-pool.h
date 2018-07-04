/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018 Intel Corporation */

#ifndef __IPU3_UTIL_H
#define __IPU3_UTIL_H

struct device;

#define IPU3_CSS_POOL_SIZE		4

struct ipu3_css_map {
	size_t size;
	void *vaddr;
	dma_addr_t daddr;
	struct vm_struct *vma;
};

struct ipu3_css_pool {
	struct {
		struct ipu3_css_map param;
		long framenum;
	} entry[IPU3_CSS_POOL_SIZE];
	unsigned int last; /* Latest entry */
};

int ipu3_css_dma_buffer_resize(struct device *dev, struct ipu3_css_map *map,
			       size_t size);
void ipu3_css_pool_cleanup(struct device *dev, struct ipu3_css_pool *pool);
int ipu3_css_pool_init(struct device *dev, struct ipu3_css_pool *pool,
		       size_t size);
int ipu3_css_pool_get(struct ipu3_css_pool *pool, long framenum);
void ipu3_css_pool_put(struct ipu3_css_pool *pool);
const struct ipu3_css_map *ipu3_css_pool_last(struct ipu3_css_pool *pool,
					      unsigned int last);

#endif
