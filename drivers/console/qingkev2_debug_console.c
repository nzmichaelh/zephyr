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

#if defined(CONFIG_QINGKEV2_DEBUG_CONSOLE_BUFFER)
struct qingkeyv2_data {
	uint32_t buffer;
	uint8_t count;
};

struct qingkeyv2_data qingkeyv2_data_0;
#endif

static int qingkev2_debug_console_write(uint32_t buffer, int count)
{
	volatile struct qingkeyv2_debug_regs *regs =
		(volatile struct qingkeyv2_debug_regs *)DT_INST_REG_ADDR(0);
	int timeout = TX_TIMEOUT;

	uint32_t tx = (buffer << 8) | count | TX_VALID | TX_FULL;
	while ((regs->data0 & TX_FULL) != 0) {
		if (--timeout == 0) {
			return 0;
		}
	}
	regs->data0 = tx;
	return 0;
}

static int qingkev2_debug_console_putc(int ch)
{
	/*
	 `minichlink` encodes the incoming and outgoing characters into `data0`. The least
	 significant byte is used for control and length, while the upper bytes are used for up
	 to three bytes of data. The `TX_FULL` bit is used to pass ownership back and forth
	 between the host and device.
	 */
	int err;

#if defined(CONFIG_QINGKEV2_DEBUG_CONSOLE_BUFFER)
	volatile struct qingkeyv2_debug_regs *regs =
		(volatile struct qingkeyv2_debug_regs *)DT_INST_REG_ADDR(0);
	struct qingkeyv2_data *data = &qingkeyv2_data_0;

	if (data->count < 3) {
		data->buffer |= ch << (8 * data->count);
		data->count++;
	}

	if (data->count == 3 || ch == '\n' || (regs->data0 & TX_FULL) == 0) {
		err = qingkev2_debug_console_write(data->buffer, data->count);
		data->count = 0;
		data->buffer = 0;
	}
#else
	err = qingkev2_debug_console_write(ch, /*count=*/1);
#endif
	return (err == 0) ? 1 : err;
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
