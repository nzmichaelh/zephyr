/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>

static int board_pinmux_init(struct device *dev)
{
	struct device *muxa = device_get_binding(CONFIG_PINMUX_SAM0_A_LABEL);

	ARG_UNUSED(dev);

#if CONFIG_UART_SAM0_SERCOM0_BASE_ADDRESS
	/* SERCOM0 on RX=PA10/pad 2, TX=PA08/pad 0 */
	pinmux_pin_set(muxa, 10, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 8, PINMUX_FUNC_C);
#endif

#if CONFIG_UART_SAM0_SERCOM1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
