/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <crc7.h>

u8_t crc7(u8_t seed, const u8_t *src, size_t len)
{
	while (len--) {
		u8_t ch = *src++;
		u8_t d0 = ch & 1;

		u8_t e = seed ^ (ch >> 1);

		u8_t e345 = (e >> 3) & 7;
		u8_t e01 = e & 3;
		u8_t e6 = (e >> 6) & 1;
		u8_t e2 = (e >> 2) & 1;

		u8_t t = e345 ^ (e01 << 1) ^ d0;

		u8_t t1 = t ^ (t << 3);
		u8_t t2 = e6 ^ (e2 << 3) ^ ((e6 ^ e2) << 6);
		u8_t t3 = e345 << 4;

		seed = t1 ^ t2 ^ t3;
	}

	return seed;
}
