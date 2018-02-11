/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CRC7_H_
#define __CRC7_H_

#include <zephyr/types.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute the CRC-7 checksum of a buffer.
 *
 * See JESD84-A441.  Used by the MMC protocol.  Uses 0x09 as the
 * polynomial with no reflection.
 *
 * @param seed Value to seed the CRC with
 * @param src Input bytes for the computation
 * @param len Length of the input in bytes
 *
 * @return The computed CRC7 value
 */
u8_t crc7(u8_t seed, const u8_t *src, size_t len);

#ifdef __cplusplus
}
#endif

#endif
