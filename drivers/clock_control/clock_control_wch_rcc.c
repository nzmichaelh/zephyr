/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_rcc

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util_macro.h>

#include <hal_ch32fun.h>

/*
 * The CH32H41X drives a dedicated SYSPLL (sourced from HSE or HSI, up to
 * 480 MHz) that needs a different bring-up sequence and clock-enable register
 * names than the other WCH parts. Select that path from the RCC clock-source
 * compatible in devicetree rather than a SoC Kconfig, so the driver follows its
 * devicetree description instead of changing behaviour per SoC.
 */
#define WCH_RCC_SRC_IS_H41X_PLL DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32h41x_pll_clock)

/*
 * The CH32H41X RCC keeps the CH32V20x/30x clock-enable register ordering (AHB,
 * then APB2, then APB1) but names them HBPCENR / HB2PCENR / HB1PCENR. Alias the
 * first clock-enable register so the shared offset arithmetic below keeps working.
 */
#if WCH_RCC_SRC_IS_H41X_PLL
#define WCH_RCC_PCENR_BASE HBPCENR
#else
#define WCH_RCC_PCENR_BASE AHBPCENR
#endif

#define WCH_RCC_CLOCK_ID_OFFSET(id) (((id) >> 5) & 0xFF)
#define WCH_RCC_CLOCK_ID_BIT(id)    ((id) & 0x1F)
/* See also CH32V00X_PB1_PCENR_OFFSET, CH32V20X_V30X_APB1_PCENR_OFFSET */
#define WCH_RCC_CLOCK_ID_HB         0
#define WCH_RCC_CLOCK_ID_PB2        1
#define WCH_RCC_CLOCK_ID_PB1        2

#define WCH_RCC_PLLMUL_VAL(mul)     (((mul) << 0x12) & RCC_PLLMULL)
#define WCH_RCC_HB_PRESCALER        DT_INST_PROP_OR(0, hb_prescaler, 1)
#define WCH_RCC_PB1_PRESCALER       DT_INST_PROP_OR(0, pb1_prescaler, 1)
#define WCH_RCC_PB2_PRESCALER       DT_INST_PROP_OR(0, pb2_prescaler, 1)
#define WCH_RCC_ADC_PRESCALER       DT_INST_PROP_OR(0, adc_prescaler, 2)
#define WCH_RCC_F_PRESCALER         DT_INST_PROP_OR(0, f_prescaler, 1)
#define WCH_RCC_SYSCLK (DT_PROP(DT_NODELABEL(cpu0), clock_frequency) * WCH_RCC_HB_PRESCALER)

#if WCH_RCC_SRC_IS_H41X_PLL
/*
 * Bound the clock-readiness spins by hardware cycle count (the mcycle CSR)
 * instead of a bare loop counter, whose duration would vary with compiler
 * codegen. Bring-up runs on the ~25 MHz HSI, so ~500000 cycles (~20 ms) sits
 * comfortably above crystal/PLL start-up while staying a deterministic ceiling.
 */
#define WCH_H41X_CLK_TIMEOUT_CYCLES 500000U
#endif

#if DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v00x_pll_clock) ||                          \
	DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v20x_30x_pll_clock)
#define WCH_RCC_SRC_IS_PLL 1
#if DT_NODE_HAS_COMPAT(DT_CLOCKS_CTLR(DT_INST_CLOCKS_CTLR(0)), wch_ch32v00x_hse_clock)
#define WCH_RCC_PLL_SRC_IS_HSE 1
#elif DT_NODE_HAS_COMPAT(DT_CLOCKS_CTLR(DT_INST_CLOCKS_CTLR(0)), wch_ch32v00x_hsi_clock)
#define WCH_RCC_PLL_SRC_IS_HSI 1
#endif
#elif DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v00x_hse_clock)
#define WCH_RCC_SRC_IS_HSE 1
#elif DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR(0), wch_ch32v00x_hsi_clock)
#define WCH_RCC_SRC_IS_HSI 1
#endif

#if defined(CONFIG_DT_HAS_WCH_CH32V20X_30X_PLL_CLOCK_ENABLED)
#if defined(CONFIG_SOC_CH32V305) || defined(CONFIG_SOC_CH32V307) || defined(CONFIG_SOC_CH32V317)
/* TODO: Entry 13 is 6.5x (fractional multiple currently unsupported without
 * changes to RCC config datatype)
 */
static const uint8_t pllmul_lut[] = {18, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 15, 16};
#else
static const uint8_t pllmul_lut[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18};
#endif
#endif

struct clock_control_wch_rcc_config {
	RCC_TypeDef *regs;
	uint8_t mul;
};

static int clock_control_wch_rcc_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_wch_rcc_config *config = dev->config;
	RCC_TypeDef *regs = config->regs;
	uint8_t id = (uintptr_t)sys;
	uint32_t reg = (uint32_t)(&regs->WCH_RCC_PCENR_BASE + WCH_RCC_CLOCK_ID_OFFSET(id));
	uint32_t val = sys_read32(reg);

	val |= BIT(WCH_RCC_CLOCK_ID_BIT(id));
	sys_write32(val, reg);

	return 0;
}

static int clock_control_wch_rcc_get_rate(const struct device *dev, clock_control_subsys_t sys,
					  uint32_t *rate)
{
	const struct clock_control_wch_rcc_config *config = dev->config;
	RCC_TypeDef *regs = config->regs;
	uint32_t cfgr0 = regs->CFGR0;
	uint32_t hclk = WCH_RCC_SYSCLK;

#if defined(RCC_FPRE)
	/* The H417 always use HPRE=/1 */
	switch (cfgr0 & RCC_FPRE) {
	case RCC_FPRE_DIV4:
		hclk /= 4;
		break;
	case RCC_FPRE_DIV2:
		hclk /= 2;
		break;
	case RCC_FPRE_DIV1:
	default:
		break;
	}
#endif

	if ((cfgr0 & RCC_HPRE_3) != 0) {
		/* The range 0b1000 divides by a power of 2, where 0b1000 is /2, 0b1001 is /4, etc.
		 */
		hclk /= 2 << ((cfgr0 & (RCC_HPRE_0 | RCC_HPRE_1 | RCC_HPRE_2)) / RCC_HPRE_0);
	} else {
#if defined(RCC_HPRE_DIV3)
		/* The range 0b0nnn divides by n + 1, where 0b0000 is /1, 0b001 is /2, etc. */
		hclk /= ((cfgr0 & (RCC_HPRE_0 | RCC_HPRE_1 | RCC_HPRE_2)) / RCC_HPRE_0) + 1;
#else
		/* 0b0xx is /1 */
#endif
	}

	switch (WCH_RCC_CLOCK_ID_OFFSET((uint32_t)sys)) {
	case WCH_RCC_CLOCK_ID_PB1:
#if defined(RCC_PPRE1)
		if ((cfgr0 & RCC_PPRE1) < RCC_PPRE1_DIV2) {
			/* 0b0xx is /1 */
			*rate = hclk;
			return 0;
		} else {
			/* 0b100 is /2, 0b101 is /4... */
			*rate = hclk >> ((cfgr0 & (RCC_PPRE1_0 | RCC_PPRE1_1)) / RCC_PPRE1_0 + 1);
			return 0;
		}
#else
		*rate = hclk;
		return 0;
#endif
	case WCH_RCC_CLOCK_ID_PB2:
#if defined(RCC_PPRE2)
		if ((cfgr0 & RCC_PPRE2) < RCC_PPRE2_DIV2) {
			/* 0xx is /1 */
			*rate = hclk;
			return 0;
		} else {
			/* 0b100 is /2, 0b101 is /4... */
			*rate = hclk >> ((cfgr0 & (RCC_PPRE2_0 | RCC_PPRE2_1)) / RCC_PPRE2_0 + 1);
			return 0;
		}
#else
		*rate = hclk;
		return 0;
#endif
	case WCH_RCC_CLOCK_ID_HB:
		*rate = hclk;
		return 0;
	default:
		return -EINVAL;
	}
}

/* Returns the register value for the given HB prescale factor */
static uint32_t clock_control_wch_rcc_hpre_cfg(uint32_t prescaler)
{
#if defined(RCC_HPRE_DIV3)
	/* CH32V00X */
	if (prescaler <= 7) {
		/* 1, 2, 3...7 */
		return (prescaler - 1) * RCC_HPRE_0;
	} else {
		/* 8, 16, 32...256 */
		return RCC_HPRE_3 | (LOG2(prescaler) - 1) * RCC_HPRE_0;
	}
#else
	/* CH32V20x/30x */
	if (prescaler == 1) {
		return RCC_HPRE_DIV1;
	} else if (prescaler <= 16) {
		/* 2, 4...16 */
		return RCC_HPRE3 | (LOG2(prescaler) - 1) * RCC_HPRE_0;
	} else {
		/* 64, 128...512. Note that there is no 32 */
		return RCC_HPRE3 | (LOG2(prescaler) - 2) * RCC_HPRE_0;
	}
#endif
}

#if defined(RCC_PPRE1)

/* Returns the register value for the given PB1 prescale factor */
static uint32_t clock_control_wch_rcc_ppre1_cfg(uint32_t prescaler)
{
	if (prescaler <= 1) {
		return RCC_PPRE1_DIV1;
	} else {
		return RCC_PPRE1_2 | (LOG2(prescaler) - 1) * RCC_PPRE1_0;
	}
}

/* Returns the register value for the given PB2 prescale factor */
static uint32_t clock_control_wch_rcc_ppre2_cfg(uint32_t prescaler)
{
	if (prescaler <= 1) {
#if defined(RCC_PPRE2_DIV0)
		/* The H41x HAL calls this 'DIV0' */
		return RCC_PPRE2_DIV0;
#else
		return RCC_PPRE2_DIV1;
#endif
	} else {
		return RCC_PPRE2_2 | (LOG2(prescaler) - 1) * RCC_PPRE2_0;
	}
}

#endif

#if defined(RCC_FPRE)

static uint32_t clock_control_wch_rcc_fpre_cfg(uint32_t prescaler)
{
	return LOG2(prescaler) * RCC_FPRE_0;
}

#endif

static void clock_control_wch_rcc_setup_flash(void)
{
#if defined(FLASH_ACTLR_LATENCY)
	uint32_t latency;

#if defined(CONFIG_SOC_CH32V003)
	if (WCH_RCC_SYSCLK <= 24000000) {
		latency = FLASH_ACTLR_LATENCY_0;
	} else {
		latency = FLASH_ACTLR_LATENCY_1;
	}
#elif defined(CONFIG_SOC_CH32V006)
	if (WCH_RCC_SYSCLK <= 15000000) {
		latency = FLASH_ACTLR_LATENCY_0;
	} else if (WCH_RCC_SYSCLK <= 24000000) {
		latency = FLASH_ACTLR_LATENCY_1;
	} else {
		latency = FLASH_ACTLR_LATENCY_2;
	}
#elif defined(CONFIG_SOC_CH32V103)
	if (WCH_RCC_SYSCLK <= 24000000) {
		latency = FLASH_ACTLR_LATENCY_0;
	} else if (WCH_RCC_SYSCLK <= 48000000) {
		latency = FLASH_ACTLR_LATENCY_1;
	} else {
		latency = FLASH_ACTLR_LATENCY_2;
	}
#else
#error Unrecognised SOC family
#endif
	FLASH->ACTLR = (FLASH->ACTLR & ~FLASH_ACTLR_LATENCY) | latency;
#endif
}

#if WCH_RCC_SRC_IS_H41X_PLL
static bool clock_control_wch_wait_ready(volatile uint32_t *reg, uint32_t mask, uint32_t value)
{
	uint32_t start = csr_read(mcycle);

	while ((*reg & mask) != value) {
		if ((uint32_t)(csr_read(mcycle) - start) > WCH_H41X_CLK_TIMEOUT_CYCLES) {
			return false;
		}
	}

	return true;
}

static void clock_control_wch_h41x_init_480m(void)
{
	volatile uint32_t *sys_cfgr0 = (volatile uint32_t *)SYS_CFGR0_BASE;
	uint32_t pll_src = RCC_USBHSPLLSRC_HSI;
	uint32_t flash_actlr;

	/* Match WCH's 480 MHz SYSCLK / 480 MHz V5F / 120 MHz HCLK setup. */
	*sys_cfgr0 = (*sys_cfgr0 & ~(0x7U << 4)) | (0x5U << 4);

	RCC->CTLR |= RCC_HSION;
	(void)clock_control_wch_wait_ready(&RCC->CTLR, RCC_HSIRDY, RCC_HSIRDY);

	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_HSI;
	(void)clock_control_wch_wait_ready(&RCC->CFGR0, RCC_SWS, RCC_SWS_HSI);

	RCC->PLLCFGR &= ~RCC_SYSPLL_GATE;
	RCC->CTLR &= ~RCC_USBHS_PLLON;

	RCC->CTLR |= RCC_HSEON;
	if (clock_control_wch_wait_ready(&RCC->CTLR, RCC_HSERDY, RCC_HSERDY)) {
		pll_src = RCC_USBHSPLLSRC_HSE;
	}

	RCC->PLLCFGR2 &= ~RCC_USBHSPLL_REFSEL;
	RCC->PLLCFGR2 = (RCC->PLLCFGR2 & ~RCC_USBHSPLLSRC) | pll_src;
	(void)clock_control_wch_wait_ready(&RCC->PLLCFGR2, RCC_USBHSPLLSRC, pll_src);

	RCC->CTLR |= RCC_USBHS_PLLON;
	(void)clock_control_wch_wait_ready(&RCC->CTLR, RCC_USBHS_PLLRDY, RCC_USBHS_PLLRDY);

	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_SYSPLL_SEL) | RCC_SYSPLL_USBHS;
	(void)clock_control_wch_wait_ready(&RCC->PLLCFGR, RCC_SYSPLL_SEL, RCC_SYSPLL_USBHS);

	/* Note that the H41x does not have PB1 */
	RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_HPRE | RCC_FPRE | RCC_PPRE2)) |
		     clock_control_wch_rcc_hpre_cfg(WCH_RCC_HB_PRESCALER) |
		     clock_control_wch_rcc_ppre2_cfg(WCH_RCC_PB2_PRESCALER) |
		     clock_control_wch_rcc_fpre_cfg(WCH_RCC_F_PRESCALER);

	flash_actlr = FLASH->ACTLR;
	flash_actlr &= ~0x3U;
	flash_actlr |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
	FLASH->ACTLR = flash_actlr;

	RCC->PLLCFGR |= RCC_SYSPLL_GATE;
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
	(void)clock_control_wch_wait_ready(&RCC->CFGR0, RCC_SWS, RCC_SWS_PLL);
}
#endif

static DEVICE_API(clock_control, clock_control_wch_rcc_api) = {
	.on = clock_control_wch_rcc_on,
	.get_rate = clock_control_wch_rcc_get_rate,
};

static int clock_control_wch_rcc_init(const struct device *dev)
{
	clock_control_wch_rcc_setup_flash();

#if WCH_RCC_SRC_IS_H41X_PLL
	clock_control_wch_h41x_init_480m();

	RCC->RSTSCKR |= RCC_LSION;
	while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {
	}

	RCC->CTLR |= RCC_CSSON;
	RCC->INTR = RCC_CSSC | RCC_PLLRDYC | RCC_HSERDYC | RCC_LSIRDYC;

	return 0;
#endif

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_PLL_CLOCK_ENABLED) ||
	    IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V20X_30X_PLL_CLOCK_ENABLED)) {
		/* Disable the PLL before potentially changing the input clocks. */
		RCC->CTLR &= ~RCC_PLLON;
	}

	/* Always enable the LSI. */
	RCC->RSTSCKR |= RCC_LSION;
	while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {
	}

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_HSI_CLOCK_ENABLED)) {
		RCC->CTLR |= RCC_HSION;
		while ((RCC->CTLR & RCC_HSIRDY) == 0) {
		}
	}

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_HSE_CLOCK_ENABLED)) {
		RCC->CTLR |= RCC_HSEON;
		while ((RCC->CTLR & RCC_HSERDY) == 0) {
		}
	}

	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V00X_PLL_CLOCK_ENABLED)) {
		if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSE)) {
			RCC->CFGR0 |= RCC_PLLSRC;
		} else if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSI)) {
			RCC->CFGR0 &= ~RCC_PLLSRC;
		}
		RCC->CTLR |= RCC_PLLON;
		while ((RCC->CTLR & RCC_PLLRDY) == 0) {
		}
	}
	if (IS_ENABLED(CONFIG_DT_HAS_WCH_CH32V20X_30X_PLL_CLOCK_ENABLED)) {
		if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSE)) {
			RCC->CFGR0 |= RCC_PLLSRC;
		} else if (IS_ENABLED(WCH_RCC_PLL_SRC_IS_HSI)) {
			RCC->CFGR0 &= ~RCC_PLLSRC;
		}
#if defined(RCC_PLLMULL)
		const struct clock_control_wch_rcc_config *config = dev->config;
		uint8_t pllmul = 0x0; /* Default Reset Value */

		for (size_t i = 0; i < ARRAY_SIZE(pllmul_lut); i++) {
			if (pllmul_lut[i] == config->mul) {
				pllmul = i;
			}
		}
		RCC->CFGR0 &= ~RCC_PLLMULL;
		RCC->CFGR0 |= WCH_RCC_PLLMUL_VAL(pllmul);
#endif
		RCC->CTLR |= RCC_PLLON;
		while ((RCC->CTLR & RCC_PLLRDY) == 0) {
		}
	}

	if (IS_ENABLED(WCH_RCC_SRC_IS_HSI)) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_HSI;
	} else if (IS_ENABLED(WCH_RCC_SRC_IS_HSE)) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_HSE;
	} else if (IS_ENABLED(WCH_RCC_SRC_IS_PLL)) {
		RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
	}
	RCC->CTLR |= RCC_CSSON;

	/* Clear the interrupt flags. */
	RCC->INTR = RCC_CSSC | RCC_PLLRDYC | RCC_HSERDYC | RCC_LSIRDYC;
	RCC->CFGR0 =
		(RCC->CFGR0 & ~RCC_HPRE) | clock_control_wch_rcc_hpre_cfg(WCH_RCC_HB_PRESCALER);

#if defined(RCC_PPRE1)
	RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_PPRE1 | RCC_PPRE2)) |
		     clock_control_wch_rcc_ppre1_cfg(WCH_RCC_PB1_PRESCALER) |
		     clock_control_wch_rcc_ppre2_cfg(WCH_RCC_PB2_PRESCALER);
#endif

	return 0;
}

#define CLOCK_CONTROL_WCH_RCC_INIT(idx)                                                            \
	static const struct clock_control_wch_rcc_config clock_control_wch_rcc_##idx##_config = {  \
		.regs = (RCC_TypeDef *)DT_INST_REG_ADDR(idx),                                      \
		.mul = DT_PROP_OR(DT_INST_CLOCKS_CTLR(idx), mul, 1),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, clock_control_wch_rcc_init, NULL, NULL,                         \
			      &clock_control_wch_rcc_##idx##_config, PRE_KERNEL_1,                 \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_wch_rcc_api);

/* There is only ever one RCC */
CLOCK_CONTROL_WCH_RCC_INIT(0)
