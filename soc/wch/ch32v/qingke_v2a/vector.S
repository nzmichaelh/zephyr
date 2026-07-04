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
	li	a0, 1
	csrw	0x804, a0
	la	a0, _vec_trampoline
	la      a1, _isr_wrapper
	sw	a1, (a0)
	or	a0, a0, 2
	csrw	mtvec, a0

#if 0
	li sp, 0x20000000 + 1024

	li t0, 1
	li t1, 2
	li t2, 3
	li a5, 5
	ecall
#endif

	tail	__initialize

SECTION_VAR(vec, _vec_trampoline)
	.word 0
