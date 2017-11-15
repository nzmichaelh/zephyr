/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_DMA_LEVEL

#include <board.h>
#include <device.h>
#include <dma.h>
#include <errno.h>
#include <init.h>
#include <logging/sys_log.h>
#include <stdio.h>
#include <string.h>

#define DMA_SAMD_NUM_CHANNELS 12

typedef void (*dma_callback)(struct device *dev, u32_t channel, int error_code);

struct dma_samd_device {
	DmacDescriptor descriptors[DMA_SAMD_NUM_CHANNELS];
	dma_callback callbacks[DMA_SAMD_NUM_CHANNELS];
};

static struct device DEVICE_NAME_GET(dma_samd_0);

static void dma_samd_channel_isr(struct device *dev,
				 struct dma_samd_device *data, int channel)
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

static void dma_samd_isr(void *arg)
{
	struct device *dev = arg;
	struct dma_samd_device *data = dev->driver_data;
	int channel;

	u32_t intstatus = DMAC->INTSTATUS.reg;

	for (channel = 0; intstatus != 0; channel++, intstatus >>= 1) {
		if ((intstatus & 1) != 0) {
			dma_samd_channel_isr(dev, data, channel);
		}
	}
}

int dma_samd_config(struct device *dev, u32_t channel,
		    struct dma_config *config)
{
	struct dma_samd_device *data = dev->driver_data;

	DmacDescriptor *desc = &data->descriptors[channel];
	struct dma_block_config *block = config->head_block;
	DMAC_BTCTRL_Type btctrl = { .reg = 0 };
	int key;

	desc->SRCADDR.reg = block->source_address;
	desc->DSTADDR.reg = block->dest_address;
	desc->BTCNT.reg = block->block_size;
	desc->DESCADDR.reg = 0;

	if (block->source_addr_adj == 1) {
		btctrl.bit.SRCINC = 1;
	}
	if (block->dest_addr_adj == 1) {
		btctrl.bit.DSTINC = 1;
	}
	btctrl.bit.VALID = 1;
	desc->BTCTRL = btctrl;

	data->callbacks[channel] = config->dma_callback;
	/* Callback is always invoked */
	DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;

	key = irq_lock();

	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
	DMAC->CHCTRLB.reg =
	    DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(config->dma_slot);
	irq_unlock(key);

	return 0;
}

int dma_samd_start(struct device *dev, u32_t channel)
{
	int key = irq_lock();

	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;

	irq_unlock(key);

	return 0;
}

int dma_samd_stop(struct device *dev, u32_t channel)
{
	int key = irq_lock();

	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.reg = 0;

	irq_unlock(key);

	return 0;
}

static int dma_samd_init(struct device *dev)
{
	struct dma_samd_device *data = dev->driver_data;

	/* Enable clocks. */
	PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

	/* Disable and reset. */
	DMAC->CTRL.bit.DMAENABLE = 0;
	DMAC->CTRL.bit.SWRST = 1;

	DMAC->BASEADDR.reg = (uintptr_t)&data->descriptors;
	DMAC->WRBADDR.reg = (uintptr_t)&data->descriptors;

	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0x0F);

	IRQ_CONNECT(DMAC_IRQn, CONFIG_DMA_0_IRQ_PRI, dma_samd_isr,
		    DEVICE_GET(dma_samd_0), 0);
	irq_enable(DMAC_IRQn);

	return 0;
}

static const struct dma_driver_api dma_samd_api = {
	.config = dma_samd_config,
	.start = dma_samd_start,
	.stop = dma_samd_stop,
};

static struct dma_samd_device dma_samd_device_0;

DEVICE_AND_API_INIT(dma_samd_0, CONFIG_DMA_0_NAME, &dma_samd_init,
		    &dma_samd_device_0, NULL, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &dma_samd_api);
