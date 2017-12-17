/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include "spi_context.h"
#include <device.h>
#include <errno.h>
#include <init.h>
#include <misc/__assert.h>
#include <soc.h>
#include <spi.h>

/* Device constant configuration parameters */
struct spi_sam0_config {
	SercomSpi *regs;
	u32_t ctrla;
	u32_t pm_apbcmask;
	u16_t gclk_clkctrl_id;
	struct soc_gpio_pin pin_miso;
	struct soc_gpio_pin pin_mosi;
	struct soc_gpio_pin pin_sck;
};

/* Device run time data */
struct spi_sam0_data {
	struct spi_context ctx;
};

static void wait_synchronization(SercomSpi *regs)
{
	while ((regs->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_MASK) != 0) {
	}
}

static int spi_sam0_configure(struct spi_config *config)
{
	const struct spi_sam0_config *cfg = config->dev->config->config_info;
	SercomSpi *regs = cfg->regs;
	SERCOM_SPI_CTRLA_Type ctrla = {.reg = 0};
	SERCOM_SPI_CTRLB_Type ctrlb = {.reg = 0};
	int div;

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		/* Slave mode is not implemented. */
		return -ENOTSUP;
	}

	ctrla.bit.MODE = SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val;

	if ((config->operation & SPI_TRANSFER_LSB) != 0) {
		ctrla.bit.DORD = 1;
	}

	if ((config->operation & SPI_MODE_CPOL) != 0) {
		ctrla.bit.CPOL = 1;
	}

	if ((config->operation & SPI_MODE_CPHA) != 0) {
		ctrla.bit.CPHA = 1;
	}

	/* MOSI on PAD2, SCK on PAD3 */
	ctrla.bit.DOPO = 1;

	if ((config->operation & SPI_MODE_LOOP) != 0) {
		/* Put MISO on the same pin as MOSI. */
		ctrla.bit.DIPO = 2;
	} else {
		ctrla.bit.DIPO = 0;
	}

	ctrla.bit.ENABLE = 1;
	ctrlb.bit.RXEN = 1;

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		return -ENOTSUP;
	}

	/* 8 bits per transfer */
	ctrlb.bit.CHSIZE = 0;

	div = (SOC_ATMEL_SAM0_GCLK0_FREQ_HZ / 2 / config->frequency) - 1;
	div = max(0, min(UINT8_MAX, div));

	/* Update the configuration if it has changed. */
	if (regs->CTRLA.reg != ctrla.reg || regs->CTRLB.reg != ctrlb.reg ||
	    regs->BAUD.reg != div) {
		regs->CTRLA.bit.ENABLE = 0;
		wait_synchronization(regs);

		regs->CTRLB = ctrlb;
		wait_synchronization(regs);
		regs->BAUD.reg = div;
		wait_synchronization(regs);
		regs->CTRLA = ctrla;
		wait_synchronization(regs);
	}

	return 0;
}

static bool spi_sam0_transfer_ongoing(struct spi_sam0_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static void spi_sam0_shift_master(SercomSpi *regs, struct spi_sam0_data *data)
{
	u8_t tx;
	u8_t rx;

	if (spi_context_tx_on(&data->ctx)) {
		tx = *(u8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0;
	}

	while (!regs->INTFLAG.bit.DRE) {
	}

	regs->DATA.reg = tx;
	spi_context_update_tx(&data->ctx, 1, 1);

	while (!regs->INTFLAG.bit.RXC) {
	}

	rx = regs->DATA.reg;

	if (spi_context_rx_on(&data->ctx)) {
		*data->ctx.rx_buf = rx;
		spi_context_update_rx(&data->ctx, 1, 1);
	}
}

static int spi_sam0_transceive(struct spi_config *config,
			       const struct spi_buf *tx_bufs, size_t tx_count,
			       struct spi_buf *rx_bufs, size_t rx_count)
{
	const struct spi_sam0_config *cfg = config->dev->config->config_info;
	struct spi_sam0_data *data = config->dev->driver_data;
	SercomSpi *regs = cfg->regs;
	int err;

	spi_context_lock(&data->ctx, false, NULL);

	err = spi_sam0_configure(config);

	if (err != 0) {
		spi_context_release(&data->ctx, err);
		goto done;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, tx_count, rx_bufs,
				  rx_count, 1);

	spi_context_cs_control(&data->ctx, true);

	do {
		spi_sam0_shift_master(regs, data);
	} while (spi_sam0_transfer_ongoing(data));

	spi_context_cs_control(&data->ctx, false);

done:
	spi_context_release(&data->ctx, err);
	return err;
}

static int spi_sam0_release(struct spi_config *config)
{
	/* Locking is not implemented so release always succeeds. */
	return 0;
}

static int spi_sam0_init(struct device *dev)
{
	const struct spi_sam0_config *cfg = dev->config->config_info;
	SercomSpi *regs = cfg->regs;

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg =
		cfg->gclk_clkctrl_id | GCLK_CLKCTRL_GEN_GCLK0
		| GCLK_CLKCTRL_CLKEN;

	/* Enable SERCOM clock in PM */
	PM->APBCMASK.reg |= cfg->pm_apbcmask;

	/* Connect pins to the peripheral */
	soc_gpio_configure(&cfg->pin_mosi);
	soc_gpio_configure(&cfg->pin_miso);
	soc_gpio_configure(&cfg->pin_sck);

	/* Disable all SPI interrupts */
	regs->INTENCLR.reg = SERCOM_SPI_INTENCLR_MASK;
	wait_synchronization(regs);

	/* The device will be configured and enabled when transceive
	 * is called.
	 */

	return 0;
}

static const struct spi_driver_api spi_sam0_driver_api = {
	.transceive = spi_sam0_transceive, .release = spi_sam0_release,
};

static const struct spi_sam0_config spi_sam0_config_4 = {
	.regs = &SERCOM4->SPI,
	.pm_apbcmask = PM_APBCMASK_SERCOM4,
	.gclk_clkctrl_id = GCLK_CLKCTRL_ID_SERCOM4_CORE,
	.ctrla = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1),
	.pin_miso = {0, 12, PORT_PMUX_PMUXE_D_Val},
	.pin_mosi = {1, 10, PORT_PMUX_PMUXE_D_Val},
	.pin_sck = {1, 11, PORT_PMUX_PMUXE_D_Val},
};

static struct spi_sam0_data spi_sam0_dev_data_4 = {
	SPI_CONTEXT_INIT_LOCK(spi_sam0_dev_data_4, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_sam0_dev_data_4, ctx),
};

DEVICE_AND_API_INIT(spi_sam0_0, CONFIG_SPI_0_NAME, &spi_sam0_init,
		    &spi_sam0_dev_data_4, &spi_sam0_config_4, POST_KERNEL,
		    CONFIG_SPI_INIT_PRIORITY, &spi_sam0_driver_api);
