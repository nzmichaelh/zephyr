/*
 * Copyright (c) 2024 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_qingkev2_debug

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/winstream.h>

struct qingkeyv2_debug_regs {
	uint32_t unused;
	uint32_t data0;
	uint32_t data1;
};

struct k_spinlock trace_lock;

#if defined(CONFIG_STDOUT_CONSOLE)
extern void __stdout_hook_install(int (*hook)(int));
#endif

#if defined(CONFIG_PRINTK)
extern void __printk_hook_install(int (*fn)(int));
#endif

#define TX_TIMEOUT 160000
#define TX_FULL    0x80
#define TX_VALID   0x04

static int qingkev2_debug_console_putc(int ch)
{
	/*
	 `minichlink` encodes the incoming and outgoing characters into `data0`. The least
	 significant byte is used for control and length, while the upper bytes are used for up
	 to three bytes of data. The `TX_FULL` bit is used to pass ownership back and forth
	 between the host and device.
	 */
	volatile struct qingkeyv2_debug_regs *regs =
		(volatile struct qingkeyv2_debug_regs *)DT_INST_REG_ADDR(0);
	int timeout = TX_TIMEOUT;

	while ((regs->data0 & TX_FULL) != 0) {
		if (--timeout == 0) {
			return 0;
		}
	}
	regs->data0 = (ch << 8) | 1 | TX_VALID | TX_FULL;
	return 1;
}

static int qingkev2_debug_console_init(void)
{
#if defined(CONFIG_STDOUT_CONSOLE)
	__stdout_hook_install(qingkev2_debug_console_putc);
#endif
#if defined(CONFIG_PRINTK)
	__printk_hook_install(qingkev2_debug_console_putc);
#endif
	return 0;
}

SYS_INIT(qingkev2_debug_console_init, PRE_KERNEL_1, CONFIG_CONSOLE_INIT_PRIORITY);
