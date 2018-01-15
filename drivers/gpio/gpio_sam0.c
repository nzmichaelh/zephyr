/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gpio_utils.h"
#include <device.h>
#include <errno.h>
#include <gpio.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

struct gpio_sam0_config {
	PortGroup *regs;
};

struct gpio_sam0_data {
	sys_slist_t cb;
	u32_t cb_pins;
};

#define DEV_CFG(dev)                                                         \
	((const struct gpio_sam0_config *const)(dev)->config->config_info)

static int gpio_sam0_config(struct device *dev, int access_op, u32_t pin,
			    int flags)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	PortGroup *regs = config->regs;
	u32_t mask = 1 << pin;
	bool is_out = (flags & GPIO_DIR_MASK) == GPIO_DIR_OUT;
	int pud = flags & GPIO_PUD_MASK;
	u32_t wrcfg = 0;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	if (is_out) {
		regs->DIRSET.bit.DIRSET = mask;
	} else {
		regs->DIRCLR.bit.DIRCLR = mask;
	}

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

	if ((flags & GPIO_INT) != 0) {
		wrcfg |= PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_PMUXEN |
			 PORT_WRCONFIG_WRPMUX |
			 PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_A_Val) | mask;
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

int gpio_sam0_manage_callback(struct device *port,
			      struct gpio_callback *callback, bool set)
{
	struct gpio_sam0_data *data = port->driver_data;

	_gpio_manage_callback(&data->cb, callback, set);

	return 0;
}

int gpio_sam0_enable_callback(struct device *port, int access_op, u32_t pin)
{
	struct gpio_sam0_data *data = port->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins |= 1 << pin;

	return 0;
}

int gpio_sam0_disable_callback(struct device *port, int access_op, u32_t pin)
{
	return -ENOTSUP;
}

u32_t gpio_sam0_get_pending_int(struct device *dev) { return 0; }

static const struct gpio_driver_api gpio_sam0_api = {
	.config = gpio_sam0_config,
	.write = gpio_sam0_write,
	.read = gpio_sam0_read,
	.manage_callback = gpio_sam0_manage_callback,
	.enable_callback = gpio_sam0_enable_callback,
	.disable_callback = gpio_sam0_disable_callback,
	.get_pending_int = gpio_sam0_get_pending_int,
};

int gpio_sam0_init(struct device *dev) { return 0; }

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
