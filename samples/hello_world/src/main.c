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
	u8_t data[70];
	u8_t head;
	u8_t tail;
};

static struct buf bufs[30];
static volatile int rx_buf;
static int tx_buf;

static int cmd_reboot(int argc, char *argv[])
{
	(&__kernel_ram_end)[-1] = DBL_TAP_MAGIC;
	sys_reboot(0);
	return 0;
}

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

static void set_led(struct device *dev, int h)
{
	struct led_rgb rgb;

	hsv2rgb(h, 255, 255, &rgb);
	led_strip_update_rgb(dev, &rgb, 1);
}

static void on_uart(struct device *dev)
{
	int rx = rx_buf;

	while (uart_irq_rx_ready(dev)) {
		struct buf *b = &bufs[rx];
		int free = sizeof(b->data) - b->head;
		int got = uart_fifo_read(dev, b->data, free);

		flags_ |= FLAG_CONNECTED;

		if (got > 0) {
			flags_ |= FLAG_RX;
			b->head += got;
			if (b->head >= sizeof(b->data)) {
				if (++rx >= ARRAY_SIZE(bufs)) {
					rx = 0;
				}
				bufs[rx].head = 0;
			}
		}
		if (got < 0) {
			printk("err=%d\n", got);
		}
		if (got < sizeof(free)) {
			break;
		}
	}
	if (bufs[rx].head != 0) {
		if (++rx >= ARRAY_SIZE(bufs)) {
			rx = 0;
		}
		bufs[rx].head = 0;
	}
	rx_buf = rx;
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

	for (;;) {
		bool any = false;
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

		while (rx_buf != tx_buf) {
			struct buf *b = &bufs[tx_buf];

			any = true;
			printk("rx_buf=%d got=%d\n", tx_buf, b->head);

			if (++tx_buf >= ARRAY_SIZE(bufs)) {
				tx_buf = 0;
			}
		}
		if (any) {
			printk("\n");
		}
		k_sleep(100);
	}
}
