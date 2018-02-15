/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <led_strip.h>
#include <board.h>
#include <gpio.h>
#include <shell/shell.h>
#include <misc/reboot.h>
#include <uart.h>

#define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
#define DBL_TAP_MAGIC 0xf01669ef
extern u32_t __kernel_ram_end;

enum {
	FLAG_CONNECTED = 1,
	FLAG_RX_ERROR = 2,
	FLAG_RX = 4,
	FLAG_TX = 8,
};

static volatile u8_t flags_;

struct buf {
	u8_t data[500];
	u16_t head;
	u16_t tail;
};

static u8_t *buf_tail(struct buf *buf, int *free)
{
	if (buf->tail >= buf->head) {
		*free = sizeof(buf->data) - buf->tail;
	} else {
		*free = buf->head - buf->tail;
	}
	return &buf->data[buf->tail];
}

static void buf_add(struct buf *buf, int count)
{
	int tail = buf->tail + count;

	if (tail >= sizeof(buf->data)) {
		tail -= sizeof(buf->data);
	}
	buf->tail = tail;
}

static u8_t *buf_head(struct buf *buf, int *avail)
{
	if (buf->head <= buf->tail) {
		*avail = buf->tail - buf->head;
	} else {
		*avail = sizeof(buf->data) - buf->head;
	}
	return &buf->data[buf->head];
}

static void buf_discard(struct buf *buf, int count)
{
	int head = buf->head + count;

	if (head >= sizeof(buf->data)) {
		head -= sizeof(buf->data);
	}
	buf->head = head;
}

static struct buf rx_buf;

static void do_reset(struct k_work *work)
{
	(&__kernel_ram_end)[-1] = DBL_TAP_MAGIC;
	sys_reboot(0);
}

static struct k_delayed_work reset_work;

void sys_reset_to_bootloader(void)
{
	k_delayed_work_submit(&reset_work, 100);
}

static void on_uart(struct device *dev)
{
	int avail;
	u8_t *p;

	while (uart_irq_rx_ready(dev)) {
		int got;
		int free;

		p = buf_tail(&rx_buf, &free);
		got = uart_fifo_read(dev, p, free);

		flags_ |= FLAG_CONNECTED;

		if (got < 0) {
			printk("err=%d\n", got);
		} else if (got > 0) {
			flags_ |= FLAG_RX;
			buf_add(&rx_buf, got);
		}
		if (got < free) {
			break;
		}
	}
	p = buf_head(&rx_buf, &avail);
	if (avail > 0) {
		/* Try to write */
		int wrote = uart_fifo_fill(dev, p, avail);

		if (wrote > 0) {
			flags_ |= FLAG_TX;
			buf_discard(&rx_buf, wrote);
		}
	}
}

void main(void)
{
	struct device *strip = device_get_binding(CONFIG_APA102_STRIP_NAME);
	struct device *led = device_get_binding(LED0_GPIO_PORT);
	struct device *cdc = device_get_binding(CONFIG_CDC_ACM_PORT_NAME);
	u8_t cycle = FLAG_RX | FLAG_TX;
	u8_t mask = FLAG_RX;

	gpio_pin_configure(led, LED0_GPIO_PIN, GPIO_DIR_OUT);
	k_delayed_work_init(&reset_work, do_reset);

	uart_irq_callback_set(cdc, on_uart);
	uart_irq_rx_enable(cdc);
	uart_irq_tx_enable(cdc);

	for (;;) {
		struct led_rgb rgb = { 0 };
		u8_t flags = flags_;

		if ((flags & mask & FLAG_RX) != 0) {
			rgb.g = 255;
		}
		if ((flags & mask & FLAG_TX) != 0) {
			rgb.r = 255;
		}
		if ((flags & FLAG_CONNECTED) != 0) {
			rgb.b = 128;
		}
		flags_ ^= (flags & mask);
		led_strip_update_rgb(strip, &rgb, 1);
		mask ^= cycle;

		k_sleep(100);
	}
}
