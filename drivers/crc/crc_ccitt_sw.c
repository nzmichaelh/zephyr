/*
 * Copyright (c) Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <crc16.h>

u16_t crc_ccitt(u16_t seed, const u8_t *src, size_t len)
{
	for (; len > 0; len--) {
		u8_t e, f;

		e = seed ^ *src++;
		f = e ^ (e << 4);
		seed = (seed >> 8) ^ (f << 8) ^ (f << 3) ^ (f >> 4);
	}

	return seed;
}
