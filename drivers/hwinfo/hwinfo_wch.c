/*
 * Copyright (c) 2025 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <string.h>

#include <hal_ch32fun.h>

#define DT_DRV_COMPAT wch_esig

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	const ESIG_TypeDef *regs = (ESIG_TypeDef *)DT_INST_REG_ADDR(0);

	length = MIN(length, 12);
	memcpy(buffer, (void *)&regs->UNIID1, length);

	return length;
}
