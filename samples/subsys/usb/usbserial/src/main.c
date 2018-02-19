/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL SYS_LOG_LEVEL_DEBUG
#include <logging/sys_log.h>

#include <kernel.h>
#include <led_strip.h>
#include <misc/reboot.h>
#include <uart.h>

#include "buf.h"

enum {
	FLAG_CONNECTED = 1,
	FLAG_OUT = 2,
	FLAG_IN = 4,
	FLAG_ACTIVITY = 8,
};

struct app {
	volatile u8_t flags;
	u8_t blink;

	struct device *cdc;
	struct device *uart;
	struct device *led;

	/* Transmit to the serial port */
	struct buf outbuf;
	/* Received from the serial port */
	struct buf inbuf;

	struct k_work in_work;
	struct k_delayed_work control_work;

	u32_t baud;
	bool ining;
	bool zlp;
	bool reset;
};

static struct app app;

static void reset(void)
{
#define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
#define DBL_TAP_MAGIC 0xf01669ef
	extern u32_t __kernel_ram_end;

	(&__kernel_ram_end)[-1] = DBL_TAP_MAGIC;
	sys_reboot(0);
}

static void check_control(void)
{
	u32_t dtr = 0;
	u32_t baud = 0;
	int err;

	if (app.reset) {
		reset();
	}

	err = uart_line_ctrl_get(app.cdc, LINE_CTRL_BAUD_RATE, &baud);
	if (err == 0) {
		if (baud == 1200) {
			app.reset = true;
		} else if (baud != app.baud) {
			SYS_LOG_INF("%p changing baud rate to %d", app.uart,
				    baud);
			err = uart_line_ctrl_set(app.uart,
						 LINE_CTRL_BAUD_RATE, baud);
			if (err != 0) {
				SYS_LOG_ERR("got err=%d while setting baud "
					    "rate to %d",
					    err, baud);
			}
			app.baud = baud;
		}
	}

	err = uart_line_ctrl_get(app.cdc, LINE_CTRL_DTR, &dtr);
	if (err == 0) {
		if (dtr) {
			app.flags |= FLAG_CONNECTED;
		} else {
			app.flags &= ~FLAG_CONNECTED;
		}
	}
}

static void in_handler(struct k_work *work)
{
	int avail = 0;
	u8_t *p = buf_head(&app.inbuf, &avail);

	if (avail == 0) {
		if (app.zlp) {
			/* Write the trailing zero length packet */
			int wrote = uart_fifo_fill(app.cdc, NULL, 0);

			if (wrote == 0) {
				app.zlp = false;
			}
		}
	} else {
		int wrote = uart_fifo_fill(app.cdc, p, avail);

		if (wrote > 0) {
			buf_discard(&app.inbuf, wrote);
		}
		app.zlp = true;
	}
	if (app.zlp) {
		k_work_submit(work);
	}
}

static void cdc_rx(struct device *dev)
{
	bool any = false;

	for (;;) {
		int free;
		u8_t *p = buf_tail(&app.outbuf, &free);
		int got = uart_fifo_read(dev, p, free);

		if (got <= 0) {
			break;
		}
		buf_add(&app.outbuf, got);
		any = true;
		app.flags |= FLAG_OUT;
		if (got != free) {
			break;
		}
	}
	if (any) {
		/* Wake up the transmitter */
		uart_irq_tx_enable(app.uart);
	}
}

static void cdc_cb(struct device *dev)
{
	uart_irq_update(dev);

	if (uart_irq_rx_ready(dev)) {
		cdc_rx(dev);
	}
	if (uart_irq_tx_ready(dev)) {
		k_work_submit(&app.in_work);
	}
}

static void uart_rx(struct device *dev)
{
	bool any = false;

	for (;;) {
		int free;
		u8_t *p = buf_tail(&app.inbuf, &free);
		int got = uart_fifo_read(dev, p, free);

		if (got <= 0) {
			break;
		}
		buf_add(&app.inbuf, got);
		any = true;
		app.flags |= FLAG_IN;
#if 0
		if (got != free) {
			break;
		}
#endif
	}
	if (any) {
		/* Wake up the transmitter */
		k_work_submit(&app.in_work);
	}
}

static void uart_tx(struct device *dev)
{
	for (;;) {
		int avail;
		int wrote;
		u8_t *p = buf_head(&app.outbuf, &avail);

		if (avail == 0) {
			uart_irq_tx_disable(dev);
			break;
		}
		wrote = uart_fifo_fill(dev, p, avail);
		if (wrote <= 0) {
			break;
		}
		buf_discard(&app.outbuf, wrote);
	}
}

static void uart_cb(struct device *dev)
{
	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uart_rx(dev);
		}
		if (uart_irq_tx_ready(dev)) {
			uart_tx(dev);
		}
	}
}

static void set_leds(u8_t flags)
{
	struct led_rgb pixel = {
		.r = (flags & FLAG_OUT) ? 0xFF : 0,
		.g = (flags & FLAG_IN) ? 0xFF : 0,
		.b = (flags & FLAG_CONNECTED) ? 0xFF : 0,
	};

	led_strip_update_rgb(app.led, &pixel, 1);
}

static void blink(void)
{
	u8_t flags = app.flags;
	u8_t on = flags & app.blink;

	set_leds(on);
	app.blink ^= FLAG_OUT | FLAG_IN | FLAG_ACTIVITY;

	app.flags = flags & ~(on & (FLAG_OUT | FLAG_IN | FLAG_ACTIVITY));
}

void main(void)
{
	app.cdc = device_get_binding(CONFIG_CDC_ACM_PORT_NAME);
	app.uart = device_get_binding(CONFIG_UART_SAM0_SERCOM2_LABEL);
	app.led = device_get_binding(CONFIG_APA102_STRIP_NAME);
	app.blink = FLAG_IN | FLAG_ACTIVITY;

	k_work_init(&app.in_work, in_handler);

	uart_irq_callback_set(app.uart, uart_cb);
	uart_irq_rx_enable(app.uart);

	uart_irq_callback_set(app.cdc, cdc_cb);
	uart_irq_rx_enable(app.cdc);

	SYS_LOG_INF("started");

	for (;;) {
		check_control();
		blink();
		k_sleep(100);
	}
}
