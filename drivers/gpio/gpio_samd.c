/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>

#include <device.h>
#include <init.h>

#include <soc.h>

#include <gpio.h>

struct gpio_samd_config {
	PortGroup *regs;
	u32_t mask;
};

#define DEV_CFG(dev)							       \
	((const struct gpio_samd_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct gpio_samd_dev_data * const)(dev)->driver_data)

static int gpio_samd_config(struct device *dev, int access_op, u32_t pin,
			    int flags)
{
	const struct gpio_samd_config *config = DEV_CFG(dev);
	u32_t mask = 1 << pin;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	if ((config->mask & mask) == 0) {
		return -EINVAL; /* Pin not in our validity mask */
	}

	if (flags & GPIO_DIR_OUT) {
		config->regs->DIRSET.bit.DIRSET = mask;
		flags &= ~GPIO_DIR_OUT;
	} else {
		config->regs->DIRCLR.bit.DIRCLR = mask;
	}

	if (flags != 0) {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_samd_write(struct device *dev, int access_op, u32_t pin,
			   u32_t value)
{
	const struct gpio_samd_config *config = DEV_CFG(dev);
	u32_t mask = 1 << pin;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		/* TODO(mlhx): support GPIO_ACCESS_BY_PORT */
		return -ENOTSUP;
	}

	if ((config->mask & mask) == 0) {
		/* Pin not in our validity mask */
		return -EINVAL;
	}

	if (value != 0) {
		config->regs->OUTSET.bit.OUTSET = mask;
	} else {
		config->regs->OUTCLR.bit.OUTCLR = mask;
	}

	return 0;
}

static int gpio_samd_read(struct device *dev, int access_op, u32_t pin,
			  u32_t *value)
{
	const struct gpio_samd_config *config = DEV_CFG(dev);
	u32_t mask = 1 << pin;
	u32_t bits;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		/* TODO(mlhx): support GPIO_ACCESS_BY_PORT */
		return -ENOTSUP;
	}

	if ((config->mask & mask) == 0) {
		/* Pin not in our validity mask */
		return -EINVAL;
	}

	bits = config->regs->IN.bit.IN;
	*value = (bits >> pin) & 1;

	return 0;
}

static const struct gpio_driver_api gpio_samd_api = {
	.config = gpio_samd_config,
	.write = gpio_samd_write,
	.read = gpio_samd_read,
};

int gpio_samd_init(struct device *dev) { return 0; }

/* Port A */
#ifdef ATMEL_SAMD_GPIO_41004400_BASE_ADDRESS

static const struct gpio_samd_config gpio_samd_config_0 = {
	.regs = (PortGroup *)ATMEL_SAMD_GPIO_41004400_BASE_ADDRESS,
	/* TODO(nzmichaelh): implement or delete. */
	.mask = ~0,
};

DEVICE_AND_API_INIT(gpio_samd_0, ATMEL_SAMD_GPIO_41004400_LABEL, gpio_samd_init,
		    NULL, &gpio_samd_config_0, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_samd_api);
#endif

#ifdef ATMEL_SAMD_GPIO_41004480_BASE_ADDRESS

static const struct gpio_samd_config gpio_samd_config_1 = {
	.regs = (PortGroup *)ATMEL_SAMD_GPIO_41004480_BASE_ADDRESS,
	/* TODO(nzmichaelh): implement or delete. */
	.mask = ~0,
};

DEVICE_AND_API_INIT(gpio_samd_1, ATMEL_SAMD_GPIO_41004480_LABEL, gpio_samd_init,
		    NULL, &gpio_samd_config_1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_samd_api);
#endif
