/*
 * Copyright (c) 2016 RnDity Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Interrupt numbers for STM32F3 family processors.
 *
 * Based on reference manual:
 *   STM32F303xB/C/D/E, STM32F303x6/8, STM32F328x8, STM32F358xC,
 *   STM32F398xE advanced ARM(r)-based MCUs
 *
 * Chapter 14.1.3: Interrupt and exception vectors
 */


#ifndef _STM32F3_SOC_IRQ_H_
#define _STM32F3_SOC_IRQ_H_

/* FIXME: Remove when use of enum line number in IRQ_CONNECT is
 * made possible by GH-2657.
 * soc_irq.h, once it is possible, should be removed. */

#define STM32F3_IRQ_WWDG		0
#define STM32F3_IRQ_PVD			1
#define STM32F3_IRQ_TAMPER		2
#define STM32F3_IRQ_RTC			3
#define STM32F3_IRQ_FLASH		4
#define STM32F3_IRQ_RCC			5
#define STM32F3_IRQ_EXTI0		6
#define STM32F3_IRQ_EXTI1		7
#define STM32F3_IRQ_EXTI2_TS		8
#define STM32F3_IRQ_EXTI3		9
#define STM32F3_IRQ_EXTI4		10
#define STM32F3_IRQ_DMA1_CH1		11
#define STM32F3_IRQ_DMA1_CH2		12
#define STM32F3_IRQ_DMA1_CH3		13
#define STM32F3_IRQ_DMA1_CH4		14
#define STM32F3_IRQ_DMA1_CH5		15
#define STM32F3_IRQ_DMA1_CH6		16
#define STM32F3_IRQ_DMA1_CH7		17
#define STM32F3_IRQ_ADC1_2		18
#define STM32F3_IRQ_USB_HP_CAN_TX	19
#define STM32F3_IRQ_USB_LP_CAN_RX0	20
#define STM32F3_IRQ_CAN_RX1		21
#define STM32F3_IRQ_CAN_SCE		22
#define STM32F3_IRQ_EXTI9_5		23
#define STM32F3_IRQ_TIM15		24
#define STM32F3_IRQ_TIM1_BRK		STM32F3_IRQ_TIM15
#define STM32F3_IRQ_TIM16		25
#define STM32F3_IRQ_TIM1_UP		STM32F3_IRQ_TIM16
#define STM32F3_IRQ_TIM17		26
#define STM32F3_IRQ_TIM1_TRG_COM	STM32F3_IRQ_TIM17
#define STM32F3_IRQ_TIM18		27
#define STM32F3_IRQ_DAC2_URR		STM32F3_IRQ_TIM18
#define STM32F3_IRQ_TIM2		28
#define STM32F3_IRQ_TIM3		29
#define STM32F3_IRQ_TIM4		30
#define STM32F3_IRQ_I2C1_EV		31
#define STM32F3_IRQ_I2C1_ER		32
#define STM32F3_IRQ_I2C2_EV		33
#define STM32F3_IRQ_I2C2_ER		34
#define STM32F3_IRQ_SPI1		35
#define STM32F3_IRQ_SPI2		36
#define STM32F3_IRQ_USART1		37
#define STM32F3_IRQ_USART2		38
#define STM32F3_IRQ_USART3		39
#define STM32F3_IRQ_EXTI15_10		40
#define STM32F3_IRQ_RTC_ALARM		41
#define STM32F3_IRQ_USB_WAKEUP		42
#define STM32F3_IRQ_TIM12		43
#define STM32F3_IRQ_TIM8_BRK		STM32F3_IRQ_TIM12
#define STM32F3_IRQ_TIM13		44
#define STM32F3_IRQ_TIM8_UP		STM32F3_IRQ_TIM13
#define STM32F3_IRQ_TIM14		45
#define STM32F3_IRQ_TIM8_TRG_COM	STM32F3_IRQ_TIM14
#define STM32F3_IRQ_TIM8_CC		46
#define STM32F3_IRQ_ADC3		47
#define STM32F3_IRQ_FMC			48
/* reserved */
/* reserved */
#define STM32F3_IRQ_TIM5		50
#define STM32F3_IRQ_SPI3		51
#define STM32F3_IRQ_UART4		52
#define STM32F3_IRQ_UART5		53
#define STM32F3_IRQ_TIM6		54
#define STM32F3_IRQ_DAC1_URR		STM32F3_IRQ_TIM6
#define STM32F3_IRQ_TIM7		55
#define STM32F3_IRQ_DMA2_CH1		56
#define STM32F3_IRQ_DMA2_CH2		57
#define STM32F3_IRQ_DMA2_CH3		58
#define STM32F3_IRQ_DMA2_CH4		59
#define STM32F3_IRQ_DMA2_CH5		60
#define STM32F3_IRQ_ADC4		61
/* reserved */
/* reserved */
#define STM32F3_IRQ_COMP_1_2_3		64
#define STM32F3_IRQ_COMP_2		STM32F3_IRQ_COMP_1_2_3
#define STM32F3_IRQ_COMP_1_2		STM32F3_IRQ_COMP_1_2_3
#define STM32F3_IRQ_COMP_4_5_6		65
#define STM32F3_IRQ_COMP_4_6		STM32F3_IRQ_COMP_4_5_6
#define STM32F3_IRQ_COMP_7		66
#define STM32F3_IRQ_OTG_FS		67
#define STM32F3_IRQ_I2C3_EV		72
#define STM32F3_IRQ_I2C3_EV_EXTI27	STM32F3_IRQ_I2C3_EV
#define STM32F3_IRQ_I2C3_ER		73
#define STM32F3_IRQ_USB_HP		74
#define STM32F3_IRQ_USB_LP		75
#define STM32F3_IRQ_USB_WAKEUP_RMP	76
#define STM32F3_IRQ_TIM20_BRK		77
#define STM32F3_IRQ_TIM19		78
#define STM32F3_IRQ_TIM20_UP		STM32F3_IRQ_TIM19
#define STM32F3_IRQ_TIM20_TRG_COM	79
#define STM32F3_IRQ_TIM20_CC		80
#define STM32F3_IRQ_FPU			81

#endif	/* _STM32F3_SOC_IRQ_H_ */
