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

#define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
#define DBL_TAP_MAGIC 0xf01669ef
extern u32_t __kernel_ram_end;

static int cmd_reboot(int argc, char *argv[])
{
	(&__kernel_ram_end)[-1] = DBL_TAP_MAGIC;
	sys_reboot(0);
	return 0;
}

static struct shell_cmd commands[] = {
	{ "reboot", cmd_reboot, "Reboot"},
	{ NULL, NULL, NULL }
};

static void hsv2rgb(int h, int s, int v, struct led_rgb *rgb)
{
	int b2 = ((255 - s) * v) / 256;
	int vb = v - b2;
	int hm = h % 60;
	int r;
	int g;
	int b;

	switch (h/60) {
	case 0:
		r = v;
		g = vb * h / 60 + b2;
		b = b2;
		break;
	case 1:
		r = vb * (60 - hm) / 60 + b2;
		g = v;
		b = b2;
		break;
	case 2:
		r = b2;
		g = v;
		b = vb * hm / 60 + b2;
		break;
	case 3:
		r = b2;
		g = vb * (60 - hm) / 60 + b2;
		b = v;
		break;
	case 4:
		r = vb * hm / 60 + b2;
		g = b2;
		b = v;
		break;
	case 5:
	default:
		r = v;
		g = b2;
		b = vb * (60 - hm) / 60 + b2;
		break;
	}

	rgb->r = r;
	rgb->g = g;
	rgb->b = b;
}

static void set_led(struct device *dev, int h)
{
	struct led_rgb rgb;

	hsv2rgb(h, 255, 255, &rgb);
	led_strip_update_rgb(dev, &rgb, 1);
}

void main(void)
{
	struct device *strip = device_get_binding(CONFIG_APA102_STRIP_NAME);
	struct device *led = device_get_binding(LED0_GPIO_PORT);
	int speed = 1;
	int dir = 1;
	int h = 0;

	SHELL_REGISTER("dbg", commands);

	gpio_pin_configure(led, LED0_GPIO_PIN, GPIO_DIR_OUT);

	for (;;) {
		set_led(strip, h);
		k_sleep(20);

		h += speed;
		if (h >= 360) {
			h -= 360;
		}
		if (h < 0) {
			h += 360;
		}
		speed += dir;
		if (speed >= 19) {
			dir = -dir;
			gpio_pin_write(led, LED0_GPIO_PIN, 1);
		} else if (speed <= -19) {
			dir = -dir;
			gpio_pin_write(led, LED0_GPIO_PIN, 0);
		}
	}
}
