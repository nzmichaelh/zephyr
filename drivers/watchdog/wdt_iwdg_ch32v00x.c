/*
 * Copyright (c) 2024 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_iwdg

#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <errno.h>

#include <ch32_iwdg.h>

static int iwdg_ch32v00x_setup(const struct device *dev, uint8_t options)
{
	if (options != 0) {
		return -ENOTSUP;
	}

	IWDG->CTLR = CTLR_KEY_Enable;

	return 0;
}

static int iwdg_ch32v00x_disable(const struct device *dev)
{
	return -EPERM;
}

static int iwdg_ch32v00x_install_timeout(const struct device *dev,
					 const struct wdt_timeout_cfg *config)
{
	/* The IWDT is driven by the 128 kHz LSI oscillator with at least a /4 prescaler. */
	int prescaler = 0;
	uint32_t reload = config->window.max * (128 / 4);

	if (config->callback != NULL) {
		return -ENOTSUP;
	}
	if (config->window.min != 0) {
		return -ENOTSUP;
	}
	if ((config->flags & WDT_FLAG_RESET_MASK) != WDT_FLAG_RESET_CPU_CORE) {
		return -ENOTSUP;
	}

	for (; reload > IWDG_RL && prescaler < IWDG_PR;) {
		prescaler++;
		reload /= 2;
	}
	/* The highest prescaler is effectively invalid. Use that to detect a too long timeout. */
	if (prescaler == IWDG_PR) {
		return -EINVAL;
	}

	/* Wait for the watchdog to be idle, unlock it, update, and wait for idle. */
	while ((IWDG->STATR & (IWDG_RVU | IWDG_PVU)) != 0) {
	}

	IWDG->CTLR = IWDG_WriteAccess_Enable;
	IWDG->PSCR = prescaler;
	IWDG->RLDR = reload;

	while ((IWDG->STATR & (IWDG_RVU | IWDG_PVU)) != 0) {
	}

	return 0;
}

static int iwdg_ch32v00x_feed(const struct device *dev, int channel_id)
{
	IWDG->CTLR = CTLR_KEY_Reload;

	return 0;
}

static const struct wdt_driver_api iwdg_ch32v00x_api = {
	.setup = iwdg_ch32v00x_setup,
	.disable = iwdg_ch32v00x_disable,
	.install_timeout = iwdg_ch32v00x_install_timeout,
	.feed = iwdg_ch32v00x_feed,
};

static int iwdg_ch32v00x_init(const struct device *dev)
{
	return 0;
}

DEVICE_DT_INST_DEFINE(0, iwdg_ch32v00x_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &iwdg_ch32v00x_api);
