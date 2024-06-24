/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_ch32v00x);

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <soc.h>
#include "i2c-priv.h"

#include <ch32v00x.h>

#define DT_DRV_COMPAT wch_i2c

struct i2c_ch32v00x_config {
	I2C_TypeDef *regs;
	const struct pinctrl_dev_config *pin_config;
	const struct device *clock_dev;
	uint8_t clock_id;
	uint32_t bus_frequency;
};

static int i2c_ch32v00x_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_ch32v00x_config *config = dev->config;
	I2C_TypeDef *regs = config->regs;
	uint32_t clock_rate;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clock_id;
	int err;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0) {
		return -ENOTSUP;
	}
	if ((dev_config & I2C_ADDR_10_BITS) != 0) {
		return -ENOTSUP;
	}

	err = clock_control_get_rate(config->clock_dev, clock_sys, &clock_rate);
	if (err != 0) {
		return err;
	}

	/* TODO: michaelh@: understand what FREQ means and does. */
	regs->CTLR2 = 4;

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		regs->CKCFGR = clock_rate / (2 * 100000);
		break;
	case I2C_SPEED_FAST:
		/* TODO: michaelh@: check if the divisor is 2 or 3. */
		regs->CKCFGR = I2C_CKCFGR_FS | (clock_rate / (3 * 400000));
		break;
	case I2C_SPEED_FAST_PLUS:
		regs->CKCFGR = I2C_CKCFGR_FS | (clock_rate / (3 * 1000000));
		break;
	default:
		return -ENOTSUP;
	}

	regs->CTLR1 = I2C_CTLR1_PE;

	return 0;
}

/* Waits for `regs->STAR1` has all of `mask` set. Returns an error on timeout. */
static int i2c_ch32v00x_await(I2C_TypeDef *regs, uint32_t mask)
{
	int i;
	for (i = 0; i < 10000000; ++i) {
		if ((regs->STAR1 & mask) == mask) {
			return 0;
		}
	}
	return -ETIMEDOUT;
}

/* If the device is idle, then starts the transaction and sends `address`. `address` must include
 * the RW bit. */
static int i2c_ch32v00x_set_address(I2C_TypeDef *regs, uint16_t address, bool restart, bool ack)
{
	int err;

	if (!restart && (regs->STAR2 & I2C_STAR2_BUSY) != 0) {
		return 0;
	}

	regs->CTLR1 = (regs->CTLR1 & ~I2C_CTLR1_ACK) | (ack ? I2C_CTLR1_ACK : 0) | I2C_CTLR1_START;

	err = i2c_ch32v00x_await(regs, I2C_STAR1_SB);
	if (err != 0) {
		return err;
	}
	regs->DATAR = address;
	err = i2c_ch32v00x_await(regs, I2C_STAR1_ADDR);
	if (err != 0) {
		return err;
	}
	/* STAR2 and STAR1 act as a pair, so read STAR2 to complete the checks. */
	(void)regs->STAR2;

	return 0;
}

/* Attempts to read from `address` into the buffer in `msg`. */
static int i2c_ch32v00x_read_msg(I2C_TypeDef *regs, struct i2c_msg *msg, uint16_t address)
{
	int err;

	err = i2c_ch32v00x_set_address(regs, (address << 1) | 1,
				       (msg->flags & I2C_MSG_RESTART) != 0,
				       /*ack=*/msg->len > 1);
	if (err != 0) {
		return err;
	}

	for (uint32_t i = 0; i < msg->len; i++) {
		if (i == msg->len - 1) {
			/* This is the last byte so NACK */
			regs->CTLR1 &= ~I2C_CTLR1_ACK;
		}
		err = i2c_ch32v00x_await(regs, I2C_STAR1_RXNE);
		if (err != 0) {
			return err;
		}
		msg->buf[i] = regs->DATAR;
	}

	return 0;
}

/* Attemps to write the message in `msg` to address `address`. */
static int i2c_ch32v00x_write_msg(I2C_TypeDef *regs, struct i2c_msg *const msg, uint16_t address)
{
	uint32_t i;
	int err;

	err = i2c_ch32v00x_set_address(regs, address << 1, (msg->flags & I2C_MSG_RESTART) != 0,
				       /*ack=*/false);
	if (err != 0) {
		return err;
	}

	for (i = 0; i < msg->len; i++) {
		err = i2c_ch32v00x_await(regs, I2C_STAR1_TXE);
		if (err != 0) {
			return err;
		}
		regs->DATAR = msg->buf[i];
	}

	/* Wait for the data to finish sending. */
	err = i2c_ch32v00x_await(regs, I2C_STAR1_BTF);
	if (err != 0) {
		return err;
	}

	return 0;
}

/* Attemps to read and write based on the message vector `msgs`. */
static int i2c_ch32v00x_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				 uint16_t addr)
{
	const struct i2c_ch32v00x_config *config = dev->config;
	I2C_TypeDef *regs = config->regs;
	struct i2c_msg *msg;
	int err;

	for (int i = 0; i < num_msgs; i++) {
		msg = msgs + i;
		if (msg->flags & I2C_MSG_READ) {
			err = i2c_ch32v00x_read_msg(regs, msg, addr);
			if (err != 0) {
				return err;
			}
		} else {
			err = i2c_ch32v00x_write_msg(regs, msg, addr);
			if (err != 0) {
				return err;
			}
		}
		if ((msg->flags & I2C_MSG_STOP) != 0) {
			regs->CTLR1 |= I2C_CTLR1_STOP;
			while ((regs->STAR2 & I2C_STAR2_BUSY) != 0) {
			}
		}
	}

	return 0;
}

static int i2c_ch32v00x_init(const struct device *dev)
{
	const struct i2c_ch32v00x_config *config = dev->config;
	int err;

	clock_control_on(config->clock_dev, (clock_control_subsys_t *)(uintptr_t)config->clock_id);

	err = pinctrl_apply_state(config->pin_config, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	return i2c_ch32v00x_configure(dev, I2C_MODE_CONTROLLER |
						   i2c_map_dt_bitrate(config->bus_frequency));
}

static const struct i2c_driver_api i2c_ch32v00x_api = {
	.configure = i2c_ch32v00x_configure,
	.transfer = i2c_ch32v00x_transfer,
};

#define I2C_CH32V00X_INIT(idx)                                                                     \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	static const struct i2c_ch32v00x_config i2c_ch32v00x_##idx##_config = {                    \
		.regs = (I2C_TypeDef *)DT_INST_REG_ADDR(idx),                                      \
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                 \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),                              \
		.clock_id = DT_INST_CLOCKS_CELL(idx, id),                                          \
		.bus_frequency = DT_INST_PROP(idx, clock_frequency),                               \
	};                                                                                         \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(idx, i2c_ch32v00x_init, NULL, NULL,                              \
				  &i2c_ch32v00x_##idx##_config, POST_KERNEL,                       \
				  CONFIG_I2C_INIT_PRIORITY, &i2c_ch32v00x_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_CH32V00X_INIT)
