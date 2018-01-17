/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gpio.h>
#include <soc.h>

#include "gpio_utils.h"

struct gpio_sam0_config {
	PortGroup *regs;
};

struct gpio_sam0_data {
	sys_slist_t cb;
	u32_t cb_pins;
};

DEVICE_DECLARE(gpio_sam0_0);

#define DEV_CFG(dev)							     \
	((const struct gpio_sam0_config *const)(dev)->config->config_info)

/* Wait for the external interrupt controller to synchronise */
static void gpio_sam0_sync_eic(void)
{
	while (EIC->STATUS.bit.SYNCBUSY) {
	}
}

static int gpio_sam0_config(struct device *dev, int access_op, u32_t pin,
			    int flags)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	PortGroup *regs = config->regs;
	u32_t mask = 1 << pin;
	bool is_out = (flags & GPIO_DIR_MASK) == GPIO_DIR_OUT;
	int pud = flags & GPIO_PUD_MASK;
	u32_t wrcfg = 0;

	/* Builds the configuration into wrcfg and writes it in one go */

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	/* Direction */
	if (is_out) {
		regs->DIRSET.bit.DIRSET = mask;
	} else {
		regs->DIRCLR.bit.DIRCLR = mask;
	}

	/* Pull up / pull down */
	if (is_out && pud != GPIO_PUD_NORMAL) {
		return -ENOTSUP;
	}

	switch (pud) {
	case GPIO_PUD_NORMAL:
		break;
	case GPIO_PUD_PULL_UP:
		wrcfg |= PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_PULLEN | mask;
		regs->OUTSET.reg = mask;
		break;
	case GPIO_PUD_PULL_DOWN:
		wrcfg |= PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_PULLEN | mask;
		regs->OUTCLR.reg = mask;
		break;
	default:
		return -ENOTSUP;
	}

	/* External interrupts */
	if ((flags & GPIO_INT) != 0) {
		u32_t config;
		u32_t cmask;
		int idx = pin / 8;
		int nibble = pin % 8;
		bool edge = (flags & GPIO_INT_EDGE) != 0;
		bool high = (flags & GPIO_INT_ACTIVE_HIGH) != 0;

		wrcfg |= PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_PMUXEN |
			 PORT_WRCONFIG_WRPMUX |
			 PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_A_Val) | mask;

		if ((flags & GPIO_INT_DOUBLE_EDGE) != 0) {
			config = EIC_CONFIG_SENSE0_BOTH;
		} else if (edge) {
			config = high ? EIC_CONFIG_SENSE0_RISE
				      : EIC_CONFIG_SENSE0_FALL;
		} else {
			config = high ? EIC_CONFIG_SENSE0_HIGH
				      : EIC_CONFIG_SENSE0_LOW;
		}

		if ((flags & GPIO_INT_DEBOUNCE) != 0) {
			config |= EIC_CONFIG_FILTEN0;
		}

		/* The config is 4 bits long and is packed in at 8
		 * pins per 32 bit word.
		 */
		config <<= (EIC_CONFIG_SENSE1_Pos * nibble);
		cmask = ~(0x0F << (EIC_CONFIG_SENSE1_Pos * nibble));

		/* Disable the EIC so it can be updated */
		EIC->CTRL.bit.ENABLE = 0;
		gpio_sam0_sync_eic();

		/* Commit the configuration */
		EIC->CONFIG[idx].reg =
			(EIC->CONFIG[idx].reg & cmask) | config;

		/* Clear any pending interrupts and unmask */
		EIC->INTFLAG.reg = mask;
		EIC->INTENSET.reg = mask;

		/* And re-enable the unit */
		EIC->CTRL.bit.ENABLE = 1;
		gpio_sam0_sync_eic();
	} else {
		EIC->INTENCLR.reg = mask;
	}

	/* Write the now-built pin configuration */
	regs->WRCONFIG.reg = wrcfg;

	if ((flags & GPIO_POL_MASK) != GPIO_POL_NORMAL) {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_sam0_write(struct device *dev, int access_op, u32_t pin,
			   u32_t value)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	u32_t mask = 1 << pin;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		/* TODO(mlhx): support GPIO_ACCESS_BY_PORT */
		return -ENOTSUP;
	}

	if (value != 0) {
		config->regs->OUTSET.bit.OUTSET = mask;
	} else {
		config->regs->OUTCLR.bit.OUTCLR = mask;
	}

	return 0;
}

static int gpio_sam0_read(struct device *dev, int access_op, u32_t pin,
			  u32_t *value)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	u32_t bits;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		/* TODO(mlhx): support GPIO_ACCESS_BY_PORT */
		return -ENOTSUP;
	}

	bits = config->regs->IN.bit.IN;
	*value = (bits >> pin) & 1;

	return 0;
}

static int gpio_sam0_manage_callback(struct device *dev,
				     struct gpio_callback *callback, bool set)
{
	struct gpio_sam0_data *data = dev->driver_data;

	_gpio_manage_callback(&data->cb, callback, set);

	return 0;
}

static int gpio_sam0_enable_callback(struct device *dev, int access_op,
				     u32_t pin)
{
	struct gpio_sam0_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins |= 1 << pin;

	return 0;
}

static int gpio_sam0_disable_callback(struct device *dev, int access_op,
				      u32_t pin)
{
	struct gpio_sam0_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins &= ~(1 << pin);

	return 0;
}

static u32_t gpio_sam0_get_pending_int(struct device *dev) { return 0; }

static void gpio_sam0_isr(struct device *dev)
{
	struct gpio_sam0_data *data = dev->driver_data;
	u32_t flags = EIC->INTFLAG.reg;

	/* Acknowledge all interrupts */
	EIC->INTFLAG.reg = flags;

	flags &= data->cb_pins;
	if (flags != 0) {
		_gpio_fire_callbacks(&data->cb, dev, flags);
	}
}

int gpio_sam0_init(struct device *dev)
{
	/* Enable the EIC clock in PM */
	PM->APBAMASK.bit.EIC_ = 1;

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN_GCLK0 |
			    GCLK_CLKCTRL_CLKEN;

	/* Reset the EIC */
	EIC->CTRL.reg = EIC_CTRL_SWRST;
	gpio_sam0_sync_eic();

	/* Route and enable the interrupt.  This is safe as nothing
	 * happens until an individual channel is unmasked.
	 */
	IRQ_CONNECT(EIC_IRQn, 0, gpio_sam0_isr, DEVICE_GET(gpio_sam0_0), 0);
	irq_enable(EIC_IRQn);

	return 0;
}

static const struct gpio_driver_api gpio_sam0_api = {
	.config = gpio_sam0_config,
	.write = gpio_sam0_write,
	.read = gpio_sam0_read,
	.manage_callback = gpio_sam0_manage_callback,
	.enable_callback = gpio_sam0_enable_callback,
	.disable_callback = gpio_sam0_disable_callback,
	.get_pending_int = gpio_sam0_get_pending_int,
};

/* Port A */
#ifdef CONFIG_GPIO_SAM0_PORTA_BASE_ADDRESS

static const struct gpio_sam0_config gpio_sam0_config_0 = {
	.regs = (PortGroup *)CONFIG_GPIO_SAM0_PORTA_BASE_ADDRESS,
};

static struct gpio_sam0_data gpio_sam0_data_0;

DEVICE_AND_API_INIT(gpio_sam0_0, CONFIG_GPIO_SAM0_PORTA_LABEL, gpio_sam0_init,
		    &gpio_sam0_data_0, &gpio_sam0_config_0, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam0_api);
#endif

/* Port B */
#ifdef CONFIG_GPIO_SAM0_PORTB_BASE_ADDRESS

static const struct gpio_sam0_config gpio_sam0_config_1 = {
	.regs = (PortGroup *)CONFIG_GPIO_SAM0_PORTB_BASE_ADDRESS,
};

static struct gpio_sam0_data gpio_sam0_data_1;

DEVICE_AND_API_INIT(gpio_sam0_1, CONFIG_GPIO_SAM0_PORTB_LABEL, gpio_sam0_init,
		    &gpio_sam0_data_1, &gpio_sam0_config_1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam0_api);
#endif
