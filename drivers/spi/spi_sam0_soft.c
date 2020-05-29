/*
 * Copyright (c) 202 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT atmel_sam0_soft_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_sam0_soft);

#include "spi_context.h"
#include <errno.h>
#include <device.h>
#include <drivers/spi.h>
#include <soc.h>

#define SCLK  2
#define MOSI  3
#define MISO  8

#define DEV_CFG(dev) \
	((const struct spi_sam0_soft_config *const)(dev)->config_info)

/* Device constant configuration parameters */
struct spi_sam0_soft_config {
	PortGroup *regs;
};

/* Device run time data */
struct spi_sam0_soft_data {
	struct spi_context ctx;
};

static bool spi_sam0_soft_transfer_ongoing(struct spi_sam0_soft_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static void spi_sam0_soft_shift(PortGroup *regs, struct spi_sam0_soft_data *data)
{
	u8_t tx = 0;
	u8_t rx = 0;

	if (spi_context_tx_buf_on(&data->ctx)) {
		tx = *(u8_t *)(data->ctx.tx_buf);
        }
	spi_context_update_tx(&data->ctx, 1, 1);

        for (int i = 0; i < 8; i++) {
                if (tx & 0x80) {
                        regs->OUTCLR.reg = BIT(SCLK);
                        regs->OUTSET.reg = BIT(MOSI);
                } else {
                        regs->OUTCLR.reg = BIT(MOSI) | BIT(SCLK);
                }
                tx <<= 1;
                regs->OUTSET.reg = BIT(SCLK);
                if (regs->IN.reg & BIT(MISO)) {
                        rx |= 1;
                }
                rx <<= 1;
        }

	if (spi_context_rx_buf_on(&data->ctx)) {
		*data->ctx.rx_buf = rx;
	}
	spi_context_update_rx(&data->ctx, 1, 1);
}

static int spi_sam0_soft_transceive(struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	const struct spi_sam0_soft_config *cfg = DEV_CFG(dev);
	struct spi_sam0_soft_data *data = dev->driver_data;
	PortGroup *regs = cfg->regs;
        int err =0;

	spi_context_lock(&data->ctx, false, NULL);

        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

        do {
                spi_sam0_soft_shift(regs, data);
        } while (spi_sam0_soft_transfer_ongoing(data));

	spi_context_release(&data->ctx, err);

        return 0;
}

static int spi_sam0_soft_init(struct device *dev)
{
	const struct spi_sam0_soft_config *config = DEV_CFG(dev);
	struct spi_sam0_soft_data *data = dev->driver_data;
	PortGroup *regs = config->regs;

        regs->DIRSET.reg = BIT(SCLK) | BIT(MOSI);
        regs->OUTCLR.reg = BIT(MOSI) | BIT(SCLK);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_sam0_soft_driver_api = {
	.transceive = spi_sam0_soft_transceive,
};

#define SPI_SAM0_SOFT_DEFINE_CONFIG(n)					\
static const struct spi_sam0_soft_config spi_sam0_soft_config_##n = {		\
	.regs = (PortGroup *)DT_REG_ADDR(DT_NODELABEL(portb)), \
        }

#define SPI_SAM0_SOFT_DEVICE_INIT(n)						\
	SPI_SAM0_SOFT_DEFINE_CONFIG(n);					\
	static struct spi_sam0_soft_data spi_sam0_soft_dev_data_##n = { \
		SPI_CONTEXT_INIT_LOCK(spi_sam0_soft_dev_data_##n, ctx),	\
		SPI_CONTEXT_INIT_SYNC(spi_sam0_soft_dev_data_##n, ctx),	\
};                                                                      \
	DEVICE_AND_API_INIT(spi_sam0_soft_##n,				\
			    DT_INST_LABEL(n),				\
			    &spi_sam0_soft_init, &spi_sam0_soft_dev_data_##n,	\
			    &spi_sam0_soft_config_##n, POST_KERNEL,		\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_sam0_soft_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SAM0_SOFT_DEVICE_INIT)
