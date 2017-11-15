/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_DMA_LEVEL
#define SYS_LOG_DOMAIN "dma"

#include <board.h>
#include <device.h>
#include <dma.h>
#include <errno.h>
#include <init.h>
#include <logging/sys_log.h>
#include <stdio.h>
#include <string.h>

#define DMA_SAM0_NUM_CHANNELS 12

typedef void (*dma_callback)(struct device *dev, u32_t channel, int error_code);

struct dma_sam0_device {
	__aligned(16) DmacDescriptor descriptors[DMA_SAM0_NUM_CHANNELS];
	__aligned(16) DmacDescriptor descriptors_wb[DMA_SAM0_NUM_CHANNELS];
	dma_callback callbacks[DMA_SAM0_NUM_CHANNELS];
};

static struct device DEVICE_NAME_GET(dma_sam0_0);

static void dma_sam0_channel_isr(struct device *dev,
				 struct dma_sam0_device *data, int channel)
{
	u8_t ints = DMAC->CHINTFLAG.reg & DMAC->CHINTENSET.reg;
	dma_callback callback = data->callbacks[channel];

	DMAC->CHINTFLAG.reg = ints;

	if (callback != NULL) {
		if ((ints & DMAC_CHINTFLAG_TCMPL) != 0) {
			callback(dev, channel, 0);
		}
	}
}

static void dma_sam0_isr(void *arg)
{
	struct device *dev = arg;
	struct dma_sam0_device *data = dev->driver_data;
	int channel;
	u32_t intstatus = DMAC->INTSTATUS.reg;

	for (channel = 0; intstatus != 0; channel++, intstatus >>= 1) {
		if ((intstatus & 1) != 0) {
			dma_sam0_channel_isr(dev, data, channel);
		}
	}
}

int dma_sam0_config(struct device *dev, u32_t channel,
		    struct dma_config *config)
{
	struct dma_sam0_device *data = dev->driver_data;
	DmacDescriptor *desc = &data->descriptors[channel];
	struct dma_block_config *block = config->head_block;
	DMAC_BTCTRL_Type btctrl = {.reg = 0};
	int key;

	SYS_LOG_DBG("%p:%u", dev, channel);

	key = irq_lock();
	DMAC->CHID.reg = channel;

	if (DMAC->CHCTRLB.bit.TRIGSRC != config->dma_slot) {
		DMAC->CHCTRLA.reg = 0;
		DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
		while (DMAC->CHCTRLA.bit.SWRST) {
		}
		DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT |
				    DMAC_CHCTRLB_TRIGSRC(config->dma_slot);
	}

	irq_unlock(key);

	desc->BTCNT.reg = block->block_size;
	desc->DESCADDR.reg = 0;

	if (block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		desc->SRCADDR.reg = block->source_address + block->block_size;
		btctrl.bit.SRCINC = 1;
	} else {
		desc->SRCADDR.reg = block->source_address;
	}
	if (block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		desc->DSTADDR.reg = block->dest_address + block->block_size;
		btctrl.bit.DSTINC = 1;
	} else {
		desc->DSTADDR.reg = block->dest_address;
	}

	btctrl.bit.VALID = 1;
	desc->BTCTRL = btctrl;

	data->callbacks[channel] = config->dma_callback;
	/* Callback is always invoked */
	DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;

	return 0;
}

int dma_sam0_start(struct device *dev, u32_t channel)
{
	int key;

	SYS_LOG_DBG("%p:%u", dev, channel);

	key = irq_lock();
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;

	irq_unlock(key);

	return 0;
}

int dma_sam0_stop(struct device *dev, u32_t channel)
{
	int key = irq_lock();

	SYS_LOG_DBG("%p:%u", dev, channel);
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.reg = 0;

	irq_unlock(key);

	return 0;
}

static int dma_sam0_init(struct device *dev)
{
	struct dma_sam0_device *data = dev->driver_data;

	SYS_LOG_DBG("%p", dev);

	/* Enable clocks. */
	PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

	/* Disable and reset. */
	DMAC->CTRL.bit.DMAENABLE = 0;
	DMAC->CTRL.bit.SWRST = 1;

	DMAC->BASEADDR.reg = (uintptr_t)&data->descriptors;
	DMAC->WRBADDR.reg = (uintptr_t)&data->descriptors_wb;

	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0x0F);

	IRQ_CONNECT(DMAC_IRQn, CONFIG_DMA_0_IRQ_PRI, dma_sam0_isr,
		    DEVICE_GET(dma_sam0_0), 0);
	irq_enable(DMAC_IRQn);

	return 0;
}

static const struct dma_driver_api dma_sam0_api = {
	.config = dma_sam0_config,
	.start = dma_sam0_start,
	.stop = dma_sam0_stop,
};

static struct dma_sam0_device dma_sam0_device_0;

DEVICE_AND_API_INIT(dma_sam0_0, CONFIG_DMA_0_NAME, &dma_sam0_init,
		    &dma_sam0_device_0, NULL, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &dma_sam0_api);
