/*
 * Copyright (c) 2020 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Driver for the SAMD51 QSPI Quad Serial Peripheral Interface controller in
 * SPI mode.
 */
#define DT_DRV_COMPAT atmel_sam0_qspi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_sam0_qspi);

#include "spi_context.h"
#include <errno.h>
#include <device.h>
#include <drivers/spi.h>
#include <soc.h>

/* Device constant configuration parameters */
struct spi_sam0_qspi_config {
	Qspi *regs;

	/* Clocks */
	volatile u32_t *mclk;
	u32_t mclk_mask;
	volatile u32_t *mclk_qspi_2x;
	u32_t mclk_qspi_2x_mask;
	volatile u32_t *mclk_qspi_apb;
	u32_t mclk_qspi_apb_mask;
};

/* Device run time data */
struct spi_sam0_qspi_data {
	struct spi_context ctx;
};

static int spi_sam0_qspi_configure(struct device *dev,
				   const struct spi_config *config)
{
	const struct spi_sam0_qspi_config *cfg = dev->config_info;
	struct spi_sam0_qspi_data *data = dev->driver_data;
	Qspi *regs = cfg->regs;
	QSPI_CTRLA_Type ctrla = { .reg = 0 };
	QSPI_CTRLB_Type ctrlb = { .reg = 0 };
	QSPI_BAUD_Type baud = { .reg = 0 };
	int div;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		/* Slave mode is not implemented. */
		return -ENOTSUP;
	}

	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		/* LSB mode is not supported. */
		return -ENOTSUP;
	}

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		baud.bit.CPOL = 1;
	}

	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		baud.bit.CPHA = 1;
	}

	if ((config->operation & SPI_MODE_LOOP) != 0U) {
		ctrlb.bit.LOOPEN = 1;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		/* Only eight bit works are implemented. */
		return -ENOTSUP;
	}

	/* 8 bits per transfer */
	ctrlb.bit.DATALEN = 0;

	/* Use the requested or next highest possible frequency */
	div = (SOC_ATMEL_SAM0_MCK_FREQ_HZ / config->frequency) - 1;
	div = MAX(0, MIN(UINT8_MAX, div));
	baud.bit.BAUD = div;

	ctrlb.bit.DLYCS = 10;
	ctrlb.bit.DLYBCT = 10;
	baud.bit.DLYBS = 10;

	/* Update the configuration only if it has changed */
	if (regs->CTRLA.reg != ctrla.reg || regs->CTRLB.reg != ctrlb.reg ||
	    regs->BAUD.reg != baud.reg) {
		regs->CTRLA.bit.ENABLE = 0;

		regs->CTRLB = ctrlb;
		regs->BAUD = baud;
		regs->CTRLA = ctrla;

		regs->CTRLA.bit.ENABLE = 1;
	}

	data->ctx.config = config;
	spi_context_cs_configure(&data->ctx);

	return 0;
}

static bool spi_sam0_qspi_transfer_ongoing(struct spi_sam0_qspi_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static void spi_sam0_qspi_shift_master(Qspi *regs,
				       struct spi_sam0_qspi_data *data)
{
	u8_t tx;
	u8_t rx;

	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(u8_t *)(data->ctx.tx_buf);
	} else {
		tx = 0U;
	}
	spi_context_update_tx(&data->ctx, 1, 1);

	while (!regs->INTFLAG.bit.DRE) {
	}

	regs->TXDATA.reg = tx;

	while (!regs->INTFLAG.bit.RXC) {
	}

	rx = regs->RXDATA.reg;

	if (spi_context_rx_buf_on(&data->ctx)) {
		*data->ctx.rx_buf = rx;
	}
	spi_context_update_rx(&data->ctx, 1, 1);
}

static int spi_sam0_qspi_transceive(struct device *dev,
				    const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const struct spi_sam0_qspi_config *cfg = dev->config_info;
	struct spi_sam0_qspi_data *data = dev->driver_data;
	Qspi *regs = cfg->regs;
	int err;

	spi_context_lock(&data->ctx, false, NULL);

	err = spi_sam0_qspi_configure(dev, config);
	if (err != 0) {
		goto done;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	do {
		spi_sam0_qspi_shift_master(regs, data);
	} while (spi_sam0_qspi_transfer_ongoing(data));

done:
	spi_context_release(&data->ctx, err);
	return err;
}

static int spi_sam0_qspi_release(struct device *dev,
				 const struct spi_config *config)
{
	struct spi_sam0_qspi_data *data = dev->driver_data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_sam0_qspi_init(struct device *dev)
{
	const struct spi_sam0_qspi_config *cfg = dev->config_info;
	struct spi_sam0_qspi_data *data = dev->driver_data;
	Qspi *regs = cfg->regs;

	/* Enable the QSPI clock in MCLK */
	*cfg->mclk |= cfg->mclk_mask;
	*cfg->mclk_qspi_2x |= cfg->mclk_qspi_2x_mask;
	*cfg->mclk_qspi_apb |= cfg->mclk_qspi_apb_mask;

	regs->CTRLA.bit.SWRST = 1;

	/* Disable all QSPI interrupts */
	regs->INTENCLR.reg = QSPI_INTENCLR_MASK;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_sam0_qspi_driver_api = {
	.transceive = spi_sam0_qspi_transceive,
	.release = spi_sam0_qspi_release,
};

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define _MCLK_MASK_DT_INT_REG_ADDR(name)				       \
	(volatile uint32_t *)(DT_REG_ADDR(DT_INST_PHANDLE_BY_NAME(0, clocks,   \
								  name)) +     \
			      DT_INST_CLOCKS_CELL_BY_NAME(0, name, offset))

static const struct spi_sam0_qspi_config spi_sam0_qspi_config_0 = {
	.regs = (Qspi *)DT_INST_REG_ADDR(0),
	.mclk = _MCLK_MASK_DT_INT_REG_ADDR(mclk),
	.mclk_mask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(0, mclk, bit)),
	.mclk_qspi_2x = _MCLK_MASK_DT_INT_REG_ADDR(mclk_qspi_2x),
	.mclk_qspi_2x_mask =
		BIT(DT_INST_CLOCKS_CELL_BY_NAME(0, mclk_qspi_2x, bit)),
	.mclk_qspi_apb = _MCLK_MASK_DT_INT_REG_ADDR(mclk_qspi_apb),
	.mclk_qspi_apb_mask =
		BIT(DT_INST_CLOCKS_CELL_BY_NAME(0, mclk_qspi_apb, bit)),
};

static struct spi_sam0_qspi_data spi_sam0_qspi_dev_data_0 = {
	SPI_CONTEXT_INIT_LOCK(spi_sam0_qspi_dev_data_0, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_sam0_qspi_dev_data_0, ctx),
};

DEVICE_AND_API_INIT(spi_sam0_qspi, DT_INST_LABEL(0), &spi_sam0_qspi_init,
		    &spi_sam0_qspi_dev_data_0, &spi_sam0_qspi_config_0,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &spi_sam0_qspi_driver_api);
#endif
