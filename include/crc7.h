/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief CRC-7 computation function
 */

#ifndef __CRC7_H
#define __CRC7_H

#include <zephyr/types.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @defgroup checksum Checksum
 */

/**
 * @defgroup crc7 CRC 7
 * @ingroup checksum
 * @{
 */

/**
 * @brief Compute the CRC-7 checksum of a buffer.
 *
 * See JESD84-A441 section 10.2.  Uses 0x09 as the polynomial with no
 * reflection.
 *
 * To calculate the CRC across non-contigious blocks use the return
 * value from block N-1 as the seed for block N.
 *
 * Checksums in this family include:
 *
 * * MMC/SD: seed=0, right justified, verify by computing and
 *   comparing the CRCs.
 * * WILC1000: seed=0, left justified, residual=0.
 *
 * @param seed Value to seed the CRC with
 * @param src Input bytes for the computation
 * @param len Length of the input in bytes
 *
 * @return The computed CRC16 value
 */
u8_t crc7(u8_t seed, const u8_t *src, size_t len);

/**
 * @}
 */
#endif
