/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INC_BOARD_H
#define __INC_BOARD_H

#include <soc.h>

/* LED1 (blue) on PA19 */
#define LED0_GPIO_PORT	CONFIG_GPIO_SAM0_PORTA_LABEL
#define LED0_GPIO_PIN	19

/* LED2 (red) on PA20 */
#define LED1_GPIO_PORT	CONFIG_GPIO_SAM0_PORTA_LABEL
#define LED1_GPIO_PIN	20

/* LED3 (green) on PA21 */
#define LED2_GPIO_PORT	CONFIG_GPIO_SAM0_PORTA_LABEL
#define LED2_GPIO_PIN	21

/* LED4 (yellow) on PB16 */
#define LED3_GPIO_PORT	CONFIG_GPIO_SAM0_PORTB_LABEL
#define LED3_GPIO_PIN	16

/* LED5 (orange) on PB17 */
#define LED4_GPIO_PORT	CONFIG_GPIO_SAM0_PORTB_LABEL
#define LED4_GPIO_PIN	17

#endif /* __INC_BOARD_H */
