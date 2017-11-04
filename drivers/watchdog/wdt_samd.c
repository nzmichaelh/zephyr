/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <watchdog.h>

/* Device constant configuration parameters */
struct wdt_samd_dev_cfg {
	Wdt *regs;
};

#define DEV_CFG(dev)                                                           \
	((const struct wdt_samd_dev_cfg *const)(dev)->config->config_info)

static void wait_synchronization(Wdt *const wdt)
{
	while (wdt->STATUS.bit.SYNCBUSY) {
	}
}

static void wdt_samd_enable(struct device *dev)
{
	Wdt *const wdt = DEV_CFG(dev)->regs;
	wdt->CTRL.reg = WDT_CTRL_ENABLE;
	wait_synchronization(wdt);
}

static void wdt_samd_disable(struct device *dev)
{
	Wdt *const wdt = DEV_CFG(dev)->regs;
	wdt->CTRL.reg = 0;
	wait_synchronization(wdt);
}

static int wdt_samd_set_config(struct device *dev, struct wdt_config *config)
{
	return -ENOTSUP;
}

static void wdt_samd_get_config(struct device *dev, struct wdt_config *config)
{
}

static void wdt_samd_reload(struct device *dev)
{
	Wdt *const wdt = DEV_CFG(dev)->regs;
	wdt->CLEAR.bit.CLEAR = WDT_CLEAR_CLEAR_KEY_Val;
}

static const struct wdt_driver_api wdt_samd_api = {
	.enable = wdt_samd_enable,
	.disable = wdt_samd_disable,
	.get_config = wdt_samd_get_config,
	.set_config = wdt_samd_set_config,
	.reload = wdt_samd_reload,
};

static int wdt_samd_init(struct device *dev)
{
	Wdt *const wdt = DEV_CFG(dev)->regs;

	/* Enable APB clock */
	PM->APBAMASK.bit.WDT_ = 1;

	/* Connect to GCLK2 (~1 kHz) */
	GCLK->CLKCTRL.reg =
	    GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;

	wdt_samd_disable(dev);

	/* Pick the longest timeout */
	wdt->CONFIG.reg = WDT_CONFIG_PER_16K;
	wait_synchronization(wdt);

#ifndef CONFIG_WDT_SAM_DISABLE_AT_BOOT
	wdt_samd_enable(dev);
	wdt_samd_reload(dev);
#endif
	return 0;
}

static const struct wdt_samd_dev_cfg wdt_samd_config = {
	.regs = (Wdt *)ATMEL_SAMD_WATCHDOG_40001000_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(wdt_samd, ATMEL_SAMD_WATCHDOG_40001000_LABEL,
		    wdt_samd_init, NULL, &wdt_samd_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_samd_api);
