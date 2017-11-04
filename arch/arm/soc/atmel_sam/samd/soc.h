#ifndef _ATMEL_SAMD_SOC_H_
#define _ATMEL_SAMD_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT
#define DONT_USE_PREDEFINED_CORE_HANDLERS
#define DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS

#include <zephyr/types.h>

#if defined CONFIG_SOC_PART_NUMBER_SAMD21G18A
#include <samd21g18a.h>
#else
#error Library does not support the specified device.
#endif

struct soc_gpio_pin {
	u8_t group;
	u8_t pin;
	u8_t mux;
};

void soc_gpio_configure(const struct soc_gpio_pin *pin);

#endif /* _ASMLANGUAGE */

/** Processor Clock (HCLK) Frequency */
#define SOC_ATMEL_SAM_HCLK_FREQ_HZ CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
/** Master Clock (MCK) Frequency */
#define SOC_ATMEL_SAMD_MCK_FREQ_HZ SOC_ATMEL_SAM_HCLK_FREQ_HZ
#define SOC_ATMEL_SAMD_XOSC32K_FREQ_HZ 32768
#define SOC_ATMEL_SAMD_OSC8M_FREQ_HZ 8000000
#define SOC_ATMEL_SAMD_GCLK0_FREQ_HZ SOC_ATMEL_SAMD_MCK_FREQ_HZ
#define SOC_ATMEL_SAMD_GCLK1_FREQ_HZ SOC_ATMEL_SAMD_XOSC32K_FREQ_HZ
#define SOC_ATMEL_SAMD_GCLK3_FREQ_HZ SOC_ATMEL_SAMD_OSC8M_FREQ_HZ
#define SOC_ATMEL_SAMD_APBA_FREQ_HZ SOC_ATMEL_SAMD_MCK_FREQ_HZ
#define SOC_ATMEL_SAMD_APBB_FREQ_HZ SOC_ATMEL_SAMD_MCK_FREQ_HZ
#define SOC_ATMEL_SAMD_APBC_FREQ_HZ SOC_ATMEL_SAMD_MCK_FREQ_HZ

#endif /* _ATMEL_SAMD_SOC_H_ */
