/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT wch_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_wch);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <hal_ch32fun.h>

#define SPI_CTLR1_BR_POS 3
#define DMA_WCH_MAX_CHAN 11

struct dma_wch_chan_regs {
	volatile uint32_t CFGR;
	volatile uint32_t CNTR;
	volatile uint32_t PADDR;
	volatile uint32_t MADDR;
	volatile uint32_t reserved1;
};

struct dma_wch_regs {
	DMA_TypeDef base;
	struct dma_wch_chan_regs channels[DMA_WCH_MAX_CHAN];
	DMA_TypeDef ext;
};

struct spi_wch_config {
	SPI_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
	const struct device *clk_dev;
	uint8_t clock_id;
#if defined(CONFIG_SPI_WCH_DMA)
	const struct device *dma_dev;
	struct dma_wch_regs *dma_regs;
	uint8_t tx_channel;
	uint8_t rx_channel;
#endif
};

struct spi_wch_data {
	struct spi_context ctx;
};

static uint8_t spi_wch_get_br(uint32_t target_clock_ratio)
{
	uint8_t prescaler;
	int prescaler_val = 2;

	for (prescaler = 0; prescaler < 7; prescaler++) {
		if (prescaler_val > target_clock_ratio) {
			break;
		}
		prescaler_val *= 2;
	}

	return prescaler;
}

static int spi_wch_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;
	SPI_TypeDef *regs = cfg->regs;
	int err;
	uint32_t clock_rate;
	clock_control_subsys_t clk_sys;
	int8_t prescaler;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if ((config->operation & SPI_HALF_DUPLEX) != 0U) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_MODE_LOOP) != 0U) {
		LOG_ERR("Loop mode not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Frame size != 8 bits not supported");
		return -ENOTSUP;
	}

	regs->CTLR1 = 0;
	regs->CTLR2 = 0;
	regs->STATR = 0;

	if (spi_cs_is_gpio(config)) {
		/* When using soft NSS, SSI must be set high */
		regs->CTLR1 |= SPI_CTLR1_SSM | SPI_CTLR1_SSI;
	} else {
		regs->CTLR2 |= SPI_CTLR2_SSOE;
	}

	regs->CTLR1 |= SPI_CTLR1_MSTR;

	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		regs->CTLR1 |= SPI_CTLR1_LSBFIRST;
	}

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		regs->CTLR1 |= SPI_CTLR1_CPOL;
	}

	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		regs->CTLR1 |= SPI_CTLR1_CPHA;
	}

	clk_sys = (clock_control_subsys_t)(uintptr_t)cfg->clock_id;
	err = clock_control_get_rate(cfg->clk_dev, clk_sys, &clock_rate);
	if (err != 0) {
		return err;
	}

	/* Approximate clock rate given ratios available */
	prescaler = spi_wch_get_br(clock_rate / config->frequency);
#if CONFIG_SPI_LOG_LEVEL >= LOG_LEVEL_INF
	uint32_t j = 2;

	for (int i = 0; i < prescaler; i++) {
		j = j * 2;
	}
	LOG_INF("Selected divider %d, value %d, results in %d frequency", j, prescaler,
		clock_rate / j);
#endif
	regs->CTLR1 |= prescaler << SPI_CTLR1_BR_POS;
	regs->CTLR2 |= SPI_CTLR2_TXDMAEN | SPI_CTLR2_RXDMAEN;

	data->ctx.config = config;

	return 0;
}

static int spi_wch_dma_tx(const struct spi_wch_config *cfg, const uint8_t *tx, size_t len)
{
	SPI_TypeDef *regs = cfg->regs;
	struct dma_status stat;
	int err;

	if (len == 0) {
		return 0;
	}

#if 1
	cfg->dma_regs->channels[cfg->tx_channel].CFGR &= ~DMA_CFGR1_EN;
	cfg->dma_regs->channels[cfg->tx_channel].MADDR = (uint32_t)tx;
	cfg->dma_regs->channels[cfg->tx_channel].CNTR = len;
	cfg->dma_regs->channels[cfg->tx_channel].CFGR |= DMA_CFGR1_EN;

	while ((cfg->dma_regs->base.INTFR & (DMA_TCIF1 << (cfg->tx_channel * 4))) == 0) {
	}

	cfg->dma_regs->channels[cfg->tx_channel].CFGR &= ~DMA_CFGR1_EN;
	cfg->dma_regs->base.INTFCR = DMA_CTCIF1 << (cfg->tx_channel * 4);

#else
	err = dma_reload(cfg->dma_dev, cfg->tx_channel, (uint32_t)tx, (uint32_t)&regs->DATAR, len);
	if (err < 0) {
		return err;
	}

	err = dma_start(cfg->dma_dev, cfg->tx_channel);
	if (err < 0) {
		return err;
	}

	do {
		err = dma_get_status(cfg->dma_dev, cfg->tx_channel, &stat);
		if (err < 0) {
			return err;
		}
	} while (stat.busy);
#endif

	while ((regs->STATR & SPI_STATR_BSY) != 0U) {
	}

	(void)regs->DATAR;

	return 0;
}

/* Fast path that transmits a buf */
static void spi_wch_tx(const struct spi_wch_config *cfg, const uint8_t *tx, size_t len)
{
	SPI_TypeDef *regs = cfg->regs;
	uint8_t ch;

	spi_wch_dma_tx(cfg, tx, len);
	return;
#if 0
	/*
	 * Unrolling increases the throughput from 2.17 MiB/s to 2.75 MiB/s, i.e. 96 % of
	 * theoretical.
	 */
	for (; len >= 2; len -= 2) {
		ch = tx[0];
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = ch;
		/* See the comment below about overlapping. */
		compiler_barrier();
		ch = tx[1];
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = ch;
		compiler_barrier();
		tx += 2;
	}
	if (len != 0) {
		ch = tx[0];
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = ch;
	}

	while ((regs->STATR & SPI_STATR_BSY) != 0U) {
	}
	/* Drop the unused receive data, if any. */
	(void)regs->DATAR;
#endif
}

static int spi_wch_dma_rx(const struct spi_wch_config *cfg, uint8_t *rx, size_t len)
{
	SPI_TypeDef *regs = cfg->regs;
	int err;

	if (len == 0) {
		return 0;
	}
	uint32_t start = k_cycle_get_32();
	int l1 = len;

#if 1
	cfg->dma_regs->channels[cfg->rx_channel].CFGR &= ~DMA_CFGR1_EN;
	cfg->dma_regs->channels[cfg->rx_channel].MADDR = (uint32_t)rx;
	cfg->dma_regs->channels[cfg->rx_channel].CNTR = len;

	regs->DATAR = 0;
	len--;
	cfg->dma_regs->channels[cfg->rx_channel].CFGR |= DMA_CFGR1_EN;

#if 1
	for (; len >= 2; len -= 2) {
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = 0;
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = 0;
	}
	if (len != 0) {
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = 0;
	}
#else
	for (; len > 0; len--) {
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = 0;
	}
#endif

	cfg->dma_regs->channels[cfg->rx_channel].CFGR &= ~DMA_CFGR1_EN;
	cfg->dma_regs->base.INTFCR = DMA_CTCIF1 << (cfg->rx_channel * 4);
	while ((regs->STATR & SPI_STATR_BSY) != 0U) {
	}

	int32_t took = k_cycle_get_32() - start;
	printf("%d in %d - %d KiB/s\n", l1, took,
	       l1 * (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1024) / took);
	return 0;
#else
	err = dma_reload(cfg->dma_dev, cfg->rx_channel, (uint32_t)&regs->DATAR, (uint32_t)rx, len);
	if (err < 0) {
		return err;
	}

	regs->DATAR = 0;
	len--;

	err = dma_start(cfg->dma_dev, cfg->rx_channel);
	if (err < 0) {
		return err;
	}

	for (; len != 0; len--) {
		while ((regs->STATR & SPI_STATR_TXE) == 0U) {
		}
		regs->DATAR = 0;
	}
	while ((regs->STATR & SPI_STATR_BSY) != 0U) {
	}

	return dma_stop(cfg->dma_dev, cfg->rx_channel);
#endif
}

/* Fast path that reads into a buf */
static void spi_wch_rx(const struct spi_wch_config *cfg, uint8_t *rx, size_t len)
{
	SPI_TypeDef *regs = cfg->regs;
	uint8_t ch;

	if (len <= 0) {
		return;
	}
	if (len > 3) {
		spi_wch_dma_rx(cfg, rx, len);
		return;
	}

	regs->DATAR = 0;
	for (len--; len > 0; len--) {
		while ((regs->STATR & SPI_STATR_RXNE) == 0U) {
		}
		ch = regs->DATAR;
		regs->DATAR = 0;
		/*
		 * Ensure DATAR is written so the next transmit happens concurrently with
		 * the store and branch.
		 */
		compiler_barrier();
		*rx++ = ch;
	}
	while ((regs->STATR & SPI_STATR_RXNE) == 0U) {
	}
	*rx = regs->DATAR;
}

/* Fast path that writes and reads bufs of the same length */
static void spi_wch_txrx(const struct spi_wch_config *cfg, const struct spi_buf *tx_buf,
			 const struct spi_buf *rx_buf)
{
	SPI_TypeDef *regs = cfg->regs;
	const uint8_t *tx = tx_buf->buf;
	const uint8_t *txend = tx + tx_buf->len;
	uint8_t *rx = rx_buf->buf;
	uint8_t *rxend = rx + rx_buf->len;

	while (tx != txend && rx != rxend) {
		regs->DATAR = *tx++;
		while ((regs->STATR & SPI_STATR_RXNE) == 0U) {
		}
		*rx++ = regs->DATAR;
	}
	spi_wch_rx(cfg, rx, rxend - rx);
	spi_wch_tx(cfg, tx, rxend - tx);
}

static int spi_wch_transceive(const struct device *dev, const struct spi_config *config,
			      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;
	SPI_TypeDef *regs = cfg->regs;
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;
	int err;

	if (tx_bufs != NULL) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	err = spi_wch_configure(dev, config);
	if (err != 0) {
		goto done;
	}

	spi_context_cs_control(&data->ctx, true);

	/* Start SPI *AFTER* setting CS */
	regs->CTLR1 |= SPI_CTLR1_SPE;

	while (tx_count != 0 && rx_count != 0) {
		if (tx->buf == NULL) {
			spi_wch_rx(cfg, rx->buf, rx->len);
		} else if (rx->buf == NULL) {
			spi_wch_tx(cfg, tx->buf, tx->len);
		} else {
			spi_wch_txrx(cfg, tx, rx);
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	for (; tx_count != 0; tx_count--) {
		spi_wch_tx(cfg, tx->buf, tx->len);
		tx++;
	}

	for (; rx_count != 0; rx_count--) {
		spi_wch_rx(cfg, rx->buf, rx->len);
		rx++;
	}

done:
	regs->CTLR1 &= ~(SPI_CTLR1_SPE);
	spi_context_cs_control(&data->ctx, false);
	spi_context_release(&data->ctx, err);
	return err;
}

static int spi_wch_transceive_sync(const struct device *dev, const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs)
{
	return spi_wch_transceive(dev, config, tx_bufs, rx_bufs);
}

static int spi_wch_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_wch_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_wch_init_dma(const struct device *dev)
{
	const struct spi_wch_config *cfg = dev->config;
	SPI_TypeDef *regs = cfg->regs;
	int err;

	struct dma_block_config tx_block = {
		.dest_address = (uint32_t)&regs->DATAR,
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
	};
	struct dma_config tx_config = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = 1,
		.dest_data_size = 2,
		.block_count = 1,
		.head_block = &tx_block,
	};
	struct dma_block_config rx_block = {
		.source_address = (uint32_t)&regs->DATAR,
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	};
	struct dma_config rx_config = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_data_size = 2,
		.dest_data_size = 1,
		.block_count = 1,
		.head_block = &rx_block,
	};

	err = dma_config(cfg->dma_dev, cfg->tx_channel, &tx_config);
	if (err < 0) {
		return err;
	}

	return dma_config(cfg->dma_dev, cfg->rx_channel, &rx_config);
}

static int spi_wch_init(const struct device *dev)
{
	int err;
	const struct spi_wch_config *cfg = dev->config;
	struct spi_wch_data *data = dev->data;
	clock_control_subsys_t clk_sys;

	clk_sys = (clock_control_subsys_t)(uintptr_t)cfg->clock_id;

	err = clock_control_on(cfg->clk_dev, clk_sys);
	if (err < 0) {
		return err;
	}

	err = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	err = spi_wch_init_dma(dev);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_wch_driver_api) = {
	.transceive = spi_wch_transceive_sync,
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_wch_release,
};

#define SPI_WCH_DMA_INIT(n)                                                                        \
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                                \
	.dma_regs = (struct dma_wch_regs *)DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),          \
	.tx_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                                   \
	.rx_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),

#ifdef CONFIG_SPI_WCH_DMA
#define SPI_WCH_USE_DMA(n) DT_INST_DMAS_HAS_NAME(n, tx)
#else
#define SPI_WCH_USE_DMA(n) 0
#endif

#define SPI_WCH_DEVICE_INIT(n)                                                                     \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct spi_wch_config spi_wch_config_##n = {                                  \
		.regs = (SPI_TypeDef *)DT_INST_REG_ADDR(n),                                        \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.clock_id = DT_INST_CLOCKS_CELL(n, id),                                            \
		COND_CODE_1(SPI_WCH_USE_DMA(n), (SPI_WCH_DMA_INIT(n)), ()) };                        \
	static struct spi_wch_data spi_wch_dev_data_##n = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_wch_dev_data_##n, ctx),                                  \
		SPI_CONTEXT_INIT_SYNC(spi_wch_dev_data_##n, ctx),                                  \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_wch_init, NULL, &spi_wch_dev_data_##n,                    \
				  &spi_wch_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,      \
				  &spi_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_WCH_DEVICE_INIT)
