/*
 * Copyright (C) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_ISR_STACKING_H_
#define SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_ISR_STACKING_H_

#include <zephyr/arch/riscv/irq.h>

#if !defined(_ASMLANGUAGE)

#ifdef CONFIG_EXCEPTION_DEBUG
#define ESF_CSF _callee_saved_t *csf;
#else
#define ESF_CSF
#endif

/* Note that the processor manual lists these as x1, x5, x6, x7, x10, x11, x12, x13, x14, and x15
 * which maps to ra, t0, t1, t2, a0, a1, a2, a3, a4, a5. */

/* Test dump shows

sp = 0x200003d0

0x200003d0 <z_idle_stacks+128>: 0x20000458      0x08000048      0x00000005      0x40013800
0x200003e0 <z_idle_stacks+144>: 0x00050000      0x20000398      0x0000004c      0x20000002
0x200003f0 <z_idle_stacks+160>: 0x00000003      0x00000002      0x00000001      0x00000d80
0x20000400 <z_idle_stacks+176>: 0x00000001      0x200003c0      0x00000008      0x08001f36

where t0=1, t1=2, t2=3, a5=5

RISC-V always keeps a 16 byte alignment, hence the 48 bytes.

*/
#define SOC_ISR_STACKING_ESF_DECLARE                                                               \
	struct arch_esf {                                                                          \
		unsigned long mcause;                                                              \
		unsigned long mstatus; /* machine status register */                               \
		unsigned long s0;      /* callee-saved s0 */                                       \
		ESF_CSF;                                                                           \
                                                                                                   \
		unsigned long pad2;                                                                \
		unsigned long mepc; /* machine exception program counter */                        \
		unsigned long a5;   /* function argument */                                        \
		unsigned long a4;   /* function argument */                                        \
		unsigned long a3;   /* function argument */                                        \
		unsigned long a2;   /* function argument */                                        \
		unsigned long a1;   /* function argument */                                        \
		unsigned long a0;   /* function argument/return value */                           \
                                                                                                   \
		unsigned long t2; /* Caller-saved temporary register */                            \
		unsigned long t1; /* Caller-saved temporary register */                            \
		unsigned long t0; /* Caller-saved temporary register */                            \
		unsigned long ra; /* return address */                                             \
	} __aligned(16);

#if 0

#define SOC_ISR_STACKING_ESF_DECLARE                                                               \
	struct arch_esf {                                                                          \
		unsigned long mepc;    /* machine exception program counter */                     \
		unsigned long mstatus; /* machine status register */                               \
                                                                                                   \
		unsigned long s0; /* callee-saved s0 */                                            \
		ESF_CSF;                                                                           \
                                                                                                   \
		unsigned long pad2;                                                                \
		unsigned long ra; /* return address */                                             \
		unsigned long a5; /* function argument */                                          \
		unsigned long a4; /* function argument */                                          \
		unsigned long a3; /* function argument */                                          \
		unsigned long a2; /* function argument */                                          \
		unsigned long a1; /* function argument */                                          \
		unsigned long a0; /* function argument/return value */                             \
                                                                                                   \
		unsigned long t2; /* Caller-saved temporary register */                            \
		unsigned long t1; /* Caller-saved temporary register */                            \
		unsigned long t0; /* Caller-saved temporary register */                            \
		unsigned long pad0;                                                                \
	} __aligned(16);
#endif

#define SOC_ISR_STACKING_ESR_INIT stack_init->mcause = 0;

#else /* _ASMLANGUAGE */

#define SOC_ISR_SW_STACKING addi sp, sp, -16;

#define SOC_ISR_SW_UNSTACKING addi sp, sp, 16

#endif /* _ASMLANGUAGE */

#endif /* SOC_RISCV_NORDIC_NRF_COMMON_VPR_SOC_ISR_STACKING_H_ */
