/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL 4
#define SYS_LOG_DOMAIN "prng"
#include <logging/sys_log.h>

#include <misc/byteorder.h>
#include <net/buf.h>
#include <string.h>
#include <zephyr.h>

#define SERIAL_0 0x0080A00C
#define SERIAL_1 0x0080A040
#define SERIAL_2 0x0080A044
#define SERIAL_3 0x0080A048

struct prng_data {
	u32_t entropy[64 / 4];
	u8_t at;
};

static struct prng_data data;

void prng_init(void)
{
	data.entropy[data.at++] = *(u32_t *)SERIAL_0;
	data.entropy[data.at++] = *(u32_t *)SERIAL_1;
	data.entropy[data.at++] = *(u32_t *)SERIAL_2;
	data.entropy[data.at++] = *(u32_t *)SERIAL_3;
	data.entropy[data.at++] = (u32_t)prng_init;
}

void prng_feed(void)
{
	data.entropy[data.at++] ^= k_uptime_get_32();
	if (data.at >= ARRAY_SIZE(data.entropy)) {
		data.at = 0;
	}
}

int default_CSPRNG(u8_t *dest, unsigned int size)
{
	dump_hex("entropy", data.entropy, sizeof(data.entropy));

	for (; size >= sizeof(data.entropy); size -= sizeof(data.entropy)) {
		memcpy(dest, data.entropy, sizeof(data.entropy));
		dest += sizeof(data.entropy);
	}
	memcpy(dest, data.entropy, size);

	return 1;
}
