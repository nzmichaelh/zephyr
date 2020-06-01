/*
 * Copyright (c) 2020 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int board_pinmux_init(struct device *dev)
{
	struct device *muxa =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));
	struct device *muxb =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_b)));

	ARG_UNUSED(dev);

#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_uart) && CONFIG_UART_SAM0)
	/* SERCOM3 on RX=PA16/pad 1, TX=PA17/pad 0 */
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_D);
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_spi) && CONFIG_SPI_SAM0)
	/* SPI SERCOM1 on MISO=PB23/pad 3, MOSI=PA0/pad 0, SCK=PA1/pad 1 */
	pinmux_pin_set(muxa, 0, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 1, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 23, PINMUX_FUNC_C);
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(tcc1), atmel_sam0_tcc_pwm, okay) && CONFIG_PWM_SAM0
        /* TCC1/WO[2] on PA18 (D7) */
	pinmux_pin_set(muxa, 18, PINMUX_FUNC_F);
        /* TCC1/WO[3] on PA19 (D9) */
	pinmux_pin_set(muxa, 19, PINMUX_FUNC_F);
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
