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
	struct device *muxb = device_get_binding(CONFIG_PINMUX_SAM0_B_LABEL);

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

#if CONFIG_SPI_SAM0_SERCOM1_BASE_ADDRESS
	/* SPI1 on MOSI=PA16/pad 0, MISO on PA18/pad 2, SCK on PA17/pad 1 */
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 18, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_C);
#endif

#if CONFIG_SPI_SAM0_SERCOM4_BASE_ADDRESS
	/* SPI4 on MOSI=PB12/pad 0, MISO on PB14/pad 2, SCK on PB13/pad 1 */
	pinmux_pin_set(muxb, 12, PINMUX_FUNC_C);
	pinmux_pin_set(muxb, 14, PINMUX_FUNC_C);
	pinmux_pin_set(muxb, 13, PINMUX_FUNC_C);
#endif

#if CONFIG_SPI_SAM0_SERCOM0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
