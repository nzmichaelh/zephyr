/*
 * Copyright (c) 2024 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

/* Exports */
GTEXT(__start)

/* Imports */
GTEXT(__initialize)

SECTION_FUNC(vectors, ivt)
	.option norvc
	j       __start
	.rept (32)
	.word   _isr_wrapper
	.endr

SECTION_FUNC(vectors, __start)
	/* Jump to __initialize */
	tail __initialize
