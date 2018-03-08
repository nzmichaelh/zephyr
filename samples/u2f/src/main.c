/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL 4
#define SYS_LOG_DOMAIN "main"
#include <logging/sys_log.h>

#include <zephyr.h>

void hid_init(void);

void main(void)
{
	SYS_LOG_DBG("Starting application");

	prng_init();
	hid_init();

	for (;;) {
		k_sleep(K_SECONDS(2));
	}
}
