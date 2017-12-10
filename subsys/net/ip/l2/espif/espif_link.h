/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

u32_t espif_link_read_u32(struct device *dev, u32_t addr, int *err);

int espif_link_read_dma(struct device *dev, int addr,
			struct net_buf_simple *sink, int len, int *err);

int espif_link_write_u32(struct device *dev, int addr, u32_t value,
				int *err);

int espif_link_write_dma(struct device *dev, int addr,
			 struct net_buf_simple *write, int *err);

bool espif_link_synced(struct device *dev);
bool espif_link_drop_sync(struct device *dev);

void espif_link_flush(struct device *dev);
