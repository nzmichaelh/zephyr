/*
 * Copyright (c) 2024 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_qingkev2_debug

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/winstream.h>

#define TX_TIMEOUT   1000000
#define TX_FULL      BIT(7)
#define TX_VALID     BIT(2)
#define TX_SIZE_MASK 0x03

/*
 * Tracks if the host has been detected. Used to prevent spinning when the console is enabled but
 * the host is not connected.
 */
enum qingkev2_debug_state {
	QINGKEV2_DEBUG_STATE_INITIAL,
	/* The first buffer has been written to the host. */
	QINGKEV2_DEBUG_STATE_FIRST,
	/* The first buffer was acknowledged by the host. */
	QINGKEV2_DEBUG_STATE_ESTABLISHED,
	/* Timeout while trying to send the second buffer to the host. */
	QINGKEV2_DEBUG_STATE_MISSING,
};

struct qingkeyv2_debug_regs {
	uint32_t unused;
	uint32_t data0;
	uint32_t data1;
};

struct qingkeyv2_debug_data {
	/* Encoded text to be written to the host. */
	uint32_t buffer;
	enum qingkev2_debug_state state;
};

static struct qingkeyv2_debug_data qingkeyv2_debug_data_0;

#if defined(CONFIG_STDOUT_CONSOLE)
extern void __stdout_hook_install(int (*hook)(int));
#endif

#if defined(CONFIG_PRINTK)
extern void __printk_hook_install(int (*fn)(int));
#endif

static int qingkev2_debug_console_putc(int ch)
{
	/*
	 * `minichlink` encodes the incoming and outgoing characters into `data0`. The least
	 * significant byte is used for control and length, while the upper bytes are used for
	 * up to three characters. The `TX_FULL` bit is used to pass ownership back and forth
	 * between the host and device.
	 */
	volatile struct qingkeyv2_debug_regs *regs =
		(volatile struct qingkeyv2_debug_regs *)DT_INST_REG_ADDR(0);
	struct qingkeyv2_debug_data *data = &qingkeyv2_debug_data_0;
	int count = data->buffer & TX_SIZE_MASK;
	int timeout;

	data->buffer |= (ch << (++count * 8));
	data->buffer++;

	/*
	 * Try to send if the buffer is full, or the character is a space or a control character, or
	 * the host is ready.
	 */
	if (count == 3 || ch <= ' ' || (regs->data0 & TX_FULL) == 0) {
		if (data->state != QINGKEV2_DEBUG_STATE_MISSING) {
			for (timeout = 0; timeout != TX_TIMEOUT && (regs->data0 & TX_FULL) != 0;
			     timeout++) {
			}
		}
		if ((regs->data0 & TX_FULL) == 0) {
			regs->data0 = data->buffer | TX_FULL | TX_VALID;
			if (data->state < QINGKEV2_DEBUG_STATE_ESTABLISHED) {
				/* Transitions from INITIAL -> FIRST -> ESTABLISHED */
				data->state++;
			} else if (data->state == QINGKEV2_DEBUG_STATE_MISSING) {
				/* The host has caught up. */
				data->state = QINGKEV2_DEBUG_STATE_ESTABLISHED;
			}
		} else {
			if (data->state == QINGKEV2_DEBUG_STATE_FIRST) {
				data->state = QINGKEV2_DEBUG_STATE_MISSING;
			}
		}
		data->buffer = 0;
	}

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
