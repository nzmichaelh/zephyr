/*
 * Copyright (c) 2024 Michael Hope
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

#ifndef CONFIG_VECTOR_TABLE_SIZE
#error "VECTOR_TABLE_SIZE must be defined"
#endif

/* Exports */
GTEXT(__start)

/* Imports */
GTEXT(__initialize)

SECTION_FUNC(vectors, ivt)
	.option norvc
	lui     a0, %hi(__start)
	jr      %lo(__start)(a0)

SECTION_FUNC(vectors, __start)
	la	a0, _vec_trampoline
	la      a1, _isr_wrapper
	sw	a1, (a0)
	or	a0, a0, 2
	csrw	mtvec, a0
	tail	__initialize

SECTION_VAR(vec, _vec_trampoline)
	.word 0
