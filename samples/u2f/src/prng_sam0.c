/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL 4
#define SYS_LOG_DOMAIN "prng"
#include <logging/sys_log.h>

#include <arch/arm/cortex_m/cmsis.h>
#include <misc/byteorder.h>
#include <net/buf.h>
#include <string.h>
#include <zephyr.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/hmac_prng.h>

#define SERIAL_0 0x0080A00C
#define SERIAL_1 0x0080A040
#define SERIAL_2 0x0080A044
#define SERIAL_3 0x0080A048

struct prng_data {
	struct tc_hmac_prng_struct prng;
	u32_t entropy[17];
	u8_t at;
};

static struct prng_data data;

static void prng_add(u32_t ch)
{
	data.entropy[data.at++] = ch;
	if (data.at >= ARRAY_SIZE(data.entropy)) {
		data.at = 0;
	}
}

void prng_feed(void)
{
	prng_add(SysTick->VAL);
	prng_add(k_uptime_get_32());
}

int default_CSPRNG(u8_t *buf, unsigned int len)
{
	int err;

	for (;;) {
		err = tc_hmac_prng_generate(buf, len, &data.prng);
		SYS_LOG_DBG("err=%d", err);

		switch (err) {
		case TC_CRYPTO_SUCCESS:
			return err;
		case TC_HMAC_PRNG_RESEED_REQ:
			prng_feed();
			dump_hex("entropy", data.entropy,
				 sizeof(data.entropy));
			err = tc_hmac_prng_reseed(
				&data.prng, (u8_t *)data.entropy,
				sizeof(data.entropy), NULL, 0);
			if (err != TC_CRYPTO_SUCCESS) {
				return err;
			}
			break;
		default:
			return TC_CRYPTO_FAIL;
		}
	}
}

int prng_init(void)
{
	u32_t personalization[] = {
		*(u32_t *)SERIAL_0, *(u32_t *)SERIAL_1, *(u32_t *)SERIAL_2,
		*(u32_t *)SERIAL_3, (u32_t)prng_init,
	};
	int err;

	err = tc_hmac_prng_init(&data.prng, (u8_t *)personalization,
				sizeof(personalization));
	if (err != TC_CRYPTO_SUCCESS) {
		return -EIO;
	}
	return 0;
}
