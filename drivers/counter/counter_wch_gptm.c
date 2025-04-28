/*
 * Copyright (c) 2025 Michael Hope <michaelh@juju.nz>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_gptm

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <ch32fun.h>

#if defined(CONFIG_COUNTER_WCH_GPTM_ALARMS)
#define COUNTER_WCH_GPTM_CHANNELS DT_INST_PROP(0, channels)
#else
#define COUNTER_WCH_GPTM_CHANNELS 0
#endif

struct counter_wch_gptm_channel_data {
	counter_alarm_callback_t callback;
	void *user;
};

struct counter_wch_gptm_data {
	counter_top_callback_t top_callback;
	void *top_user;
	struct counter_wch_gptm_channel_data channels[COUNTER_WCH_GPTM_CHANNELS];
};

struct counter_wch_gptm_config {
	struct counter_config_info base;
	TIM_TypeDef *regs;
	const struct device *clock_dev;
	uint8_t clock_id;
	uint16_t prescaler;
};

int counter_wch_gptm_start(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	regs->CTLR1 |= TIM_CEN;

	return 0;
}

int counter_wch_gptm_stop(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	regs->CTLR1 &= ~TIM_CEN;

	return 0;
}

int counter_wch_gptm_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	*ticks = regs->CNT;

	return 0;
}

static int counter_wch_gptm_set_alarm(const struct device *dev, uint8_t channel_id,
				      const struct counter_alarm_cfg *alarm_config)
{
	const struct counter_wch_gptm_config *config = dev->config;
	struct counter_wch_gptm_data *data = dev->data;
	TIM_TypeDef *regs = config->regs;
	struct counter_wch_gptm_channel_data *channel;
	uint16_t top;
	uint16_t target;
	unsigned int key;
	int err = 0;

	if (!IS_ENABLED(CONFIG_COUNTER_WCH_GPTM_ALARMS)) {
		/* Note that the rest of the function is compiled but optimised out */
		return -ENOTSUP;
	}

	if ((alarm_config->flags & ~COUNTER_ALARM_CFG_ABSOLUTE) != 0) {
		return -ENOTSUP;
	}

	if (channel_id >= ARRAY_SIZE(data->channels)) {
		return -EINVAL;
	}

	channel = &data->channels[channel_id];
	top = regs->ATRLR;

	if (alarm_config->ticks > top) {
		return -EINVAL;
	}

	key = irq_lock();

	if (channel->callback != NULL) {
		err = -EBUSY;
		goto unlock;
	}

	channel->callback = alarm_config->callback;
	channel->user = alarm_config->user_data;

	if ((alarm_config->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0) {
		target = alarm_config->ticks;
	} else {
		target = regs->CNT + alarm_config->ticks;
		if (target > top) {
			target -= top;
		}
	}

	BUILD_ASSERT(&regs->CH1CVR + 1 == &regs->CH2CVR);
	(&regs->CH1CVR)[channel_id] = target;
	regs->DMAINTENR |= TIM_CC1IE << channel_id;

unlock:
	irq_unlock(key);

	return err;
}

int counter_wch_gptm_cancel_alarm(const struct device *dev, uint8_t channel_id)
{
	const struct counter_wch_gptm_config *config = dev->config;
	struct counter_wch_gptm_data *data = dev->data;
	TIM_TypeDef *regs = config->regs;
	struct counter_wch_gptm_channel_data *channel;
	unsigned int key;

	if (!IS_ENABLED(CONFIG_COUNTER_WCH_GPTM_ALARMS)) {
		return -ENOTSUP;
	}

	if (channel_id >= ARRAY_SIZE(data->channels)) {
		return -EINVAL;
	}

	channel = &data->channels[channel_id];

	key = irq_lock();
	channel->callback = NULL;
	regs->DMAINTENR &= ~(TIM_CC1IE << channel_id);
	irq_unlock(key);

	return 0;
}

int counter_wch_gptm_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	const struct counter_wch_gptm_config *config = dev->config;
	struct counter_wch_gptm_data *data = dev->data;
	TIM_TypeDef *regs = config->regs;
	int err = 0;

	if ((cfg->flags & ~(COUNTER_TOP_CFG_DONT_RESET | COUNTER_TOP_CFG_RESET_WHEN_LATE)) != 0) {
		return -ENOTSUP;
	}

	if (cfg->callback == NULL) {
		/* Disable the interrupt before updating the callback */
		regs->DMAINTENR &= ~TIM_UIE;
	}

	regs->ATRLR = cfg->ticks;
	data->top_callback = cfg->callback;
	data->top_user = cfg->user_data;

	if ((cfg->flags & COUNTER_TOP_CFG_DONT_RESET) == 0) {
		regs->CNT = 0;
	} else if (regs->CNT >= cfg->ticks) {
		err = -ETIME;
		if ((cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) != 0) {
			regs->CNT = 0;
		}
	}

	if (cfg->callback != NULL) {
		regs->DMAINTENR |= TIM_UIE;
	}

	return err;
}

uint32_t counter_wch_gptm_get_pending_int(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	return (regs->INTFR & TIM_UIF) != 0;
}

uint32_t counter_wch_gptm_get_top_value(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	return regs->ATRLR;
}

uint32_t counter_wch_gptm_get_freq(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clock_id;
	uint32_t clock_rate;

	clock_control_get_rate(config->clock_dev, clock_sys, &clock_rate);

	return clock_rate / (config->prescaler + 1);
}

static void counter_wch_gptm_isr(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	struct counter_wch_gptm_data *data = dev->data;
	TIM_TypeDef *regs = config->regs;

	if ((regs->INTFR & TIM_UIF) != 0) {
		regs->INTFR = TIM_UIF;
		if (data->top_callback != NULL) {
			data->top_callback(dev, data->top_user);
		}
	}
}

static int counter_wch_gptm_init(const struct device *dev)
{
	const struct counter_wch_gptm_config *config = dev->config;
	TIM_TypeDef *regs = config->regs;

	clock_control_on(config->clock_dev, (clock_control_subsys_t *)(uintptr_t)config->clock_id);

	/* Disable and configure the counter */
	regs->CTLR1 = TIM_ARPE & ~TIM_CEN;
	regs->PSC = config->prescaler;

	return 0;
}

static DEVICE_API(counter, counter_wch_gptm_api) = {
	.start = counter_wch_gptm_start,
	.stop = counter_wch_gptm_stop,
	.get_value = counter_wch_gptm_get_value,
	.set_alarm = counter_wch_gptm_set_alarm,
	.cancel_alarm = counter_wch_gptm_cancel_alarm,
	.set_top_value = counter_wch_gptm_set_top_value,
	.get_pending_int = counter_wch_gptm_get_pending_int,
	.get_top_value = counter_wch_gptm_get_top_value,
	.get_freq = counter_wch_gptm_get_freq,
};

#define COUNTER_WCH_GPTM_INIT(idx)                                                                 \
	static int counter_wch_gptm_init_##idx(const struct device *dev)                           \
	{                                                                                          \
		int err = counter_wch_gptm_init(dev);                                              \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, global, irq),                                 \
			    DT_INST_IRQ_BY_NAME(idx, global, priority), counter_wch_gptm_isr,      \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, global, irq));                                 \
                                                                                                   \
		return err;                                                                        \
	}                                                                                          \
                                                                                                   \
	static struct counter_wch_gptm_data counter_wch_gptm_##idx##_data;                         \
                                                                                                   \
	static const struct counter_wch_gptm_config counter_wch_gptm_##idx##_config = {            \
		.base =                                                                            \
			{                                                                          \
				.max_top_value = UINT16_MAX,                                       \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = DT_INST_PROP(idx, channels),                           \
			},                                                                         \
		.regs = (TIM_TypeDef *)DT_INST_REG_ADDR(idx),                                      \
		.prescaler = DT_INST_PROP(idx, prescaler),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),                              \
		.clock_id = DT_INST_CLOCKS_CELL(idx, id),                                          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, counter_wch_gptm_init_##idx, NULL,                              \
			      &counter_wch_gptm_##idx##_data, &counter_wch_gptm_##idx##_config,    \
			      PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY, &counter_wch_gptm_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_WCH_GPTM_INIT);
