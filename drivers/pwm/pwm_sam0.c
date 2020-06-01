/*
 * Copyright (c) 2020 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PWM driver using the SAM0 Timer/Counter (TCC) in Normal PWM (NPWM) mode.
 * Currently supports the SAMD5x series.  Adding support for the SAMD2x series
 * should be straight forward.
 */

#define DT_DRV_COMPAT atmel_sam0_tcc_pwm

#include <device.h>
#include <errno.h>
#include <drivers/pwm.h>
#include <soc.h>

/* Static configuration */
struct pwm_sam0_config {
	Tcc *regs;
	u8_t channels;
	u8_t counter_size;
	u16_t prescaler;
	u32_t freq;

	volatile u32_t *mclk;
	u32_t mclk_mask;
	u16_t gclk_id;
};

#define DEV_CFG(dev) ((const struct pwm_sam0_config *const)(dev)->config_info)

/* Wait for the peripheral to finish all commands */
static void wait_synchronization(Tcc *regs)
{
	while (regs->SYNCBUSY.reg != 0) {
	}
}

static int pwm_sam0_get_cycles_per_sec(struct device *dev, u32_t ch,
				       u64_t *cycles)
{
	const struct pwm_sam0_config *const cfg = DEV_CFG(dev);

	if (ch >= cfg->channels) {
		return -EINVAL;
	}
	*cycles = cfg->freq;

	return 0;
}

static int pwm_sam0_pin_set(struct device *dev, u32_t ch, u32_t period_cycles,
			    u32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_sam0_config *const cfg = DEV_CFG(dev);
	Tcc *regs = cfg->regs;
	u32_t top = 1 << cfg->counter_size;
	u32_t invert_mask = 1 << ch;
	bool invert = ((flags & PWM_POLARITY_INVERTED) != 0);
	bool inverted = ((regs->DRVCTRL.vec.INVEN & invert_mask) != 0);

	if (ch >= cfg->channels) {
		return -EINVAL;
	}
	if (period_cycles >= top || pulse_cycles >= top) {
		return -EINVAL;
	}

	/*
         * Update the buffered width and period.  These will be automatically
         * loaded on the next cycle.
        */
	regs->CCBUF[ch].reg = TCC_CCBUF_CCBUF(pulse_cycles);
	regs->PERBUF.reg = TCC_PERBUF_PERBUF(period_cycles);

	if (invert != inverted) {
		regs->CTRLA.bit.ENABLE = 0;
		wait_synchronization(regs);

		regs->DRVCTRL.vec.INVEN ^= invert_mask;
		regs->CTRLA.bit.ENABLE = 1;
		wait_synchronization(regs);
	}

	return 0;
}

static int pwm_sam0_init(struct device *dev)
{
	const struct pwm_sam0_config *const cfg = DEV_CFG(dev);
	Tcc *regs = cfg->regs;

	/* Enable the clocks */
	GCLK->PCHCTRL[cfg->gclk_id].reg =
		GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
	*cfg->mclk |= cfg->mclk_mask;

	regs->CTRLA.bit.SWRST = 1;
	wait_synchronization(regs);

	regs->CTRLA.reg = cfg->prescaler;
	regs->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
	regs->PER.reg = TCC_PERBUF_PERBUF(1);

	regs->CTRLA.bit.ENABLE = 1;
	wait_synchronization(regs);

	return 0;
}

static const struct pwm_driver_api pwm_sam0_driver_api = {
	.pin_set = pwm_sam0_pin_set,
	.get_cycles_per_sec = pwm_sam0_get_cycles_per_sec,
};

#define PWM_SAM0_INIT(inst)                                                    \
	static const struct pwm_sam0_config pwm_sam0_config_##inst = {         \
		.regs = (Tcc *)DT_INST_REG_ADDR(inst),                         \
		.channels = DT_INST_PROP(inst, channels),                      \
		.counter_size = DT_INST_PROP(inst, counter_size),              \
		.prescaler = UTIL_CAT(TCC_CTRLA_PRESCALER_DIV,                 \
				      DT_INST_PROP(inst, prescaler)),          \
		.freq = SOC_ATMEL_SAM0_GCLK0_FREQ_HZ /                         \
			DT_INST_PROP(inst, prescaler),                         \
		.mclk = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(inst),  \
		.mclk_mask =                                                   \
			BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, bit)),     \
		.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, periph_ch), \
	};                                                                     \
                                                                               \
	DEVICE_AND_API_INIT(pwm_sam0_##inst, DT_INST_LABEL(inst),              \
			    &pwm_sam0_init, NULL, &pwm_sam0_config_##inst,     \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,   \
			    &pwm_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SAM0_INIT)
