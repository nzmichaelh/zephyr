/*
 * Copyright (c) 2024 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_adc

#include <soc.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(adc_ch32v00x, CONFIG_ADC_LOG_LEVEL);

#include <ch32_adc.h>

struct adc_ch32v00x_data {
};

struct adc_ch32v00x_config {
	ADC_TypeDef *regs;
	const struct device *clock_dev;
	uint8_t clock_id;
};

static int adc_ch32v00x_channel_setup(const struct device *dev,
				      const struct adc_channel_cfg *channel_cfg)
{
	if (channel_cfg->gain != ADC_GAIN_1) {
		return -EINVAL;
	}
	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		return -EINVAL;
	}
	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		return -EINVAL;
	}
	if (channel_cfg->differential) {
		return -EINVAL;
	}
	if (channel_cfg->channel_id >= 10) {
		return -EINVAL;
	}

	return 0;
}

static int adc_ch32v00x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_ch32v00x_config *config = dev->config;
	ADC_TypeDef *regs = config->regs;
	uint32_t channels = sequence->channels;
	int rsqr = 2;
	int sequence_id = 0;
	int total_channels = 0;
	int i;
	uint16_t *samples = sequence->buffer;

	if (sequence->options != NULL) {
		return -ENOTSUP;
	}
	if (sequence->resolution != 10) {
		return -EINVAL;
	}
	if (sequence->oversampling != 0) {
		return -ENOTSUP;
	}
	if (sequence->channels >= (1 << 10)) {
		return -EINVAL;
	}

	if (sequence->calibrate) {
		regs->CTLR2 |= ADC_RSTCAL;
		while ((regs->CTLR2 & ADC_RSTCAL) != 0) {
		}
		regs->CTLR2 |= ADC_CAL;
		while ((ADC1->CTLR2 & ADC_CAL) != 0) {
		}
	}

	regs->RSQR1 = 0;
	regs->RSQR2 = 0;
	regs->RSQR3 = 0;

	for (i = 0; channels != 0; i++, channels >>= 1) {
		if ((channels & 1) != 0) {
			total_channels++;
			(&regs->RSQR1)[rsqr] |= i << sequence_id;
			/* The channel IDs are packed 5 bits at a time into RSQR3 down to RSQR1.
			 */
			sequence_id += ADC_SQ2_0;
			if (sequence_id >= 32) {
				sequence_id = 0;
				rsqr--;
			}
		}
	}
	if (total_channels == 0) {
		return 0;
	}
	if (sequence->buffer_size < total_channels * sizeof(*samples)) {
		return -ENOMEM;
	}

	/* Set the number of channels to read. Note that '0' means 'one channel'. */
	regs->RSQR1 |= (total_channels - 1) * ADC_L_0;
	regs->CTLR2 |= ADC_SWSTART;
	for (i = 0; i < total_channels; i++) {
		while ((regs->STATR & ADC_EOC) == 0) {
		}
		*samples++ = regs->RDATAR;
	}

	return 0;
}

static int adc_ch32v00x_init(const struct device *dev)
{
	const struct adc_ch32v00x_config *config = dev->config;
	ADC_TypeDef *regs = config->regs;

	clock_control_on(config->clock_dev, (clock_control_subsys_t *)(uintptr_t)config->clock_id);

	regs->CTLR2 = ADC_ADON | ADC_EXTSEL;

	return 0;
}

#define ADC_CH32V00X_DEVICE(n)                                                                     \
	static const struct adc_driver_api adc_ch32v00x_api_##n = {                                \
		.channel_setup = adc_ch32v00x_channel_setup,                                       \
		.read = adc_ch32v00x_read,                                                         \
		.ref_internal = DT_INST_PROP(n, vref_mv),                                          \
	};                                                                                         \
                                                                                                   \
	static const struct adc_ch32v00x_config adc_ch32v00x_config_##n = {                        \
		.regs = (ADC_TypeDef *)DT_INST_REG_ADDR(n),                                        \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id = DT_INST_CLOCKS_CELL(n, id),                                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, adc_ch32v00x_init, NULL, NULL, &adc_ch32v00x_config_##n,          \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_ch32v00x_api_##n);

DT_INST_FOREACH_STATUS_OKAY(ADC_CH32V00X_DEVICE)
