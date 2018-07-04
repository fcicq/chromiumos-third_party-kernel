/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018 Intel Corporation */
/* Copyright (C) 2018 Google, Inc. */

#ifndef __IPU3_DMAMAP_H
#define __IPU3_DMAMAP_H

struct imgu_device;
struct scatterlist;

void *ipu3_dmamap_alloc(struct device *dev, struct ipu3_css_map *map,
			size_t len);
void ipu3_dmamap_free(struct device *dev, struct ipu3_css_map *map);

int ipu3_dmamap_map_sg(struct device *dev, struct scatterlist *sglist,
		       int nents, struct ipu3_css_map *map);
void ipu3_dmamap_unmap(struct device *dev, struct ipu3_css_map *map);

int ipu3_dmamap_init(struct device *dev);
void ipu3_dmamap_exit(struct device *dev);

#endif
