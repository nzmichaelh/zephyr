/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <init.h>
#include <misc/__assert.h>
#include <soc.h>
#include <uart.h>

/* Device constant configuration parameters */
struct uart_sam0_dev_cfg {
	SercomUsart *regs;
	u32_t baudrate;
	u32_t ctrla;
	u32_t pm_apbcmask;
	u16_t gclk_clkctrl_id;
	struct soc_gpio_pin pin_rx;
	struct soc_gpio_pin pin_tx;
};

/* Device run time data */
struct uart_sam0_dev_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_t cb;
#endif
};

#define DEV_CFG(dev)							       \
	((const struct uart_sam0_dev_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct uart_sam0_dev_data * const)(dev)->driver_data)

static void uart_sam0_isr(void *arg);

static void wait_synchronization(SercomUsart *const usart)
{
	while ((usart->SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_MASK) != 0) {
	}
}

static int uart_sam0_set_baudrate(SercomUsart *const usart, u32_t baudrate,
				  u32_t clk_freq_hz)
{
	u64_t tmp;
	u16_t baud;

	tmp = (u64_t)baudrate << 20;
	tmp = (tmp + (clk_freq_hz >> 1)) / clk_freq_hz;

	/* Verify that the calculated result is within range */
	if (tmp < 1 || tmp > UINT16_MAX) {
		return -ERANGE;
	}

	baud = 65536 - (u16_t)tmp;
	usart->BAUD.reg = baud;
	wait_synchronization(usart);

	return 0;
}

static int uart_sam0_init(struct device *dev)
{
	int retval;
	const struct uart_sam0_dev_cfg *const cfg = DEV_CFG(dev);
	SercomUsart *const usart = cfg->regs;

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg =
	    cfg->gclk_clkctrl_id | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;

	/* Enable SERCOM clock in PM */
	PM->APBCMASK.reg |= cfg->pm_apbcmask;

	/* Connect pins to the peripheral */
	soc_gpio_configure(&cfg->pin_rx);
	soc_gpio_configure(&cfg->pin_tx);

	/* Disable all USART interrupts */
	usart->INTENCLR.reg = SERCOM_USART_INTENCLR_MASK;
	wait_synchronization(usart);

	/* 8 bits of data, no parity, 1 stop bit in normal mode */
	usart->CTRLA.reg =
	    cfg->ctrla |
	    /* Internal clock */
	    SERCOM_USART_CTRLA_MODE_USART_INT_CLK
	    /* 16x oversampling with arithmetic baud rate generation */
	    | SERCOM_USART_CTRLA_SAMPR(0) | SERCOM_USART_CTRLA_FORM(0) |
	    SERCOM_USART_CTRLA_CPOL | SERCOM_USART_CTRLA_DORD;
	wait_synchronization(usart);

	/* Enable receiver and transmitter */
	usart->CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(0) |
			   SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;
	wait_synchronization(usart);

	retval = uart_sam0_set_baudrate(usart, cfg->baudrate,
					SOC_ATMEL_SAM0_GCLK0_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}

	usart->CTRLA.bit.ENABLE = 1;
	wait_synchronization(usart);
	return 0;
}

static int uart_sam0_poll_in(struct device *dev, unsigned char *c)
{
	SercomUsart *const usart = DEV_CFG(dev)->regs;

	if (!usart->INTFLAG.bit.RXC) {
		return -EBUSY;
	}

	*c = (unsigned char)usart->DATA.reg;
	return 0;
}

static unsigned char uart_sam0_poll_out(struct device *dev, unsigned char c)
{
	SercomUsart *const usart = DEV_CFG(dev)->regs;

	while (!usart->INTFLAG.bit.DRE) {
	}

	/* send a character */
	usart->DATA.reg = c;
	return c;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

static void uart_sam0_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_sam0_dev_data *const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev);
	}
}

static void uart_sam0_irq_tx_disable(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	regs->INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

static void uart_sam0_irq_rx_enable(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	regs->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
}

static void uart_sam0_irq_rx_disable(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	regs->INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
}

static int uart_sam0_irq_rx_ready(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	return regs->INTFLAG.bit.RXC != 0;
}

static int uart_sam0_fifo_read(struct device *dev, u8_t *rx_data,
			       const int size)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	if (regs->INTFLAG.bit.RXC) {
		u8_t ch = regs->DATA.reg;

		if (size >= 1) {
			*rx_data = ch;
			return 1;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static int uart_sam0_irq_is_pending(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	return (regs->INTENSET.reg & regs->INTFLAG.reg) != 0;
}

static int uart_sam0_irq_update(struct device *dev) { return 1; }

static void uart_sam0_irq_callback_set(struct device *dev,
				       uart_irq_callback_t cb)
{
	struct uart_sam0_dev_data *const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
}
#endif

static const struct uart_driver_api uart_sam0_driver_api = {
	.poll_in = uart_sam0_poll_in,
	.poll_out = uart_sam0_poll_out,
#if CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_read = uart_sam0_fifo_read,
	.irq_tx_disable = uart_sam0_irq_tx_disable,
	.irq_rx_enable = uart_sam0_irq_rx_enable,
	.irq_rx_disable = uart_sam0_irq_rx_disable,
	.irq_rx_ready = uart_sam0_irq_rx_ready,
	.irq_is_pending = uart_sam0_irq_is_pending,
	.irq_update = uart_sam0_irq_update,
	.irq_callback_set = uart_sam0_irq_callback_set,
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SAM0_INIT_DEFN(addr, n)					       \
	static int uart_sam0_init_##n(struct device *dev)		       \
	{								       \
		IRQ_CONNECT(CONFIG_UART_SAM0_SERCOM##n##_IRQ,		       \
			    CONFIG_UART_SAM0_SERCOM##n##_IRQ_PRIORITY,	       \
			    uart_sam0_isr, DEVICE_GET(uart_sam0_##n), 0);      \
		irq_enable(CONFIG_UART_SAM0_SERCOM##n##_IRQ);		       \
		return uart_sam0_init(dev);				       \
	}
#else
#define UART_SAM0_INIT_DEFN(n)						       \
	static int uart_sam0_init_##n(struct device *dev)		       \
	{								       \
		return uart_sam0_init(dev);				       \
	}
#endif

#define UART_SAM0_CONFIG_INIT(n)					       \
	.regs = (SercomUsart *)CONFIG_UART_SAM0_SERCOM##n##_BASE_ADDRESS,      \
	.baudrate = CONFIG_UART_SAM0_SERCOM##n##_CURRENT_SPEED,		       \
	.pm_apbcmask = PM_APBCMASK_SERCOM##n,				       \
	.gclk_clkctrl_id = GCLK_CLKCTRL_ID_SERCOM##n##_CORE,		       \
	.ctrla = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1)

#define UART_SAM0_DEVICE_INIT(n)					       \
	static int uart_sam0_init_##n(struct device *dev);		       \
									       \
	static struct uart_sam0_dev_data uart_sam0_data_##n = {};	       \
									       \
	DEVICE_AND_API_INIT(uart_sam0_##n, CONFIG_UART_SAM0_SERCOM##n##_LABEL, \
			    uart_sam0_init_##n, &uart_sam0_data_##n,	       \
			    &uart_sam0_config_##n, PRE_KERNEL_1,	       \
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		       \
			    &uart_sam0_driver_api);			       \
	UART_SAM0_INIT_DEFN(addr, n)

/* SERCOM0 on RX=PA11, TX=PA10 */
#if CONFIG_UART_SAM0_SERCOM0_BASE_ADDRESS

static const struct uart_sam0_dev_cfg uart_sam0_config_0 = {
	UART_SAM0_CONFIG_INIT(0),
	.pin_rx = {0, 11, PORT_PMUX_PMUXE_C_Val},
	.pin_tx = {0, 10, PORT_PMUX_PMUXE_C_Val},
};

UART_SAM0_DEVICE_INIT(0)
#endif

/* SERCOM1 on RX=PA19, TX=PA18 */
#if CONFIG_UART_SAM0_SERCOM1_BASE_ADDRESS

static const struct uart_sam0_dev_cfg uart_sam0_config_1 = {
	UART_SAM0_CONFIG_INIT(1),
	.pin_rx = {0, 19, PORT_PMUX_PMUXE_C_Val},
	.pin_tx = {0, 18, PORT_PMUX_PMUXE_C_Val},
};

UART_SAM0_DEVICE_INIT(1)
#endif

/* SERCOM2 on RX=PA15, TX=PA14 */
#if CONFIG_UART_SAM0_SERCOM2_BASE_ADDRESS

static const struct uart_sam0_dev_cfg uart_sam0_config_2 = {
	UART_SAM0_CONFIG_INIT(2),
	.pin_rx = {0, 15, PORT_PMUX_PMUXE_C_Val},
	.pin_tx = {0, 14, PORT_PMUX_PMUXE_C_Val},
};

UART_SAM0_DEVICE_INIT(2)
#endif

/* SERCOM3 on RX=PA21, TX=PA20 */
#if CONFIG_UART_SAM0_SERCOM3_BASE_ADDRESS

static const struct uart_sam0_dev_cfg uart_sam0_config_3 = {
	UART_SAM0_CONFIG_INIT(3),
	.pin_rx = {0, 21, PORT_PMUX_PMUXE_D_Val},
	.pin_tx = {0, 20, PORT_PMUX_PMUXE_D_Val},
};

UART_SAM0_DEVICE_INIT(3)
#endif

/* SERCOM4 on RX=PB11, TX=PB10 */
#if CONFIG_UART_SAM0_SERCOM4_BASE_ADDRESS

static const struct uart_sam0_dev_cfg uart_sam0_config_4 = {
	UART_SAM0_CONFIG_INIT(4),
	.pin_rx = {0, 21, PORT_PMUX_PMUXE_C_Val},
	.pin_tx = {0, 20, PORT_PMUX_PMUXE_C_Val},
};

UART_SAM0_DEVICE_INIT(4)
#endif

/* SERCOM5 on RX=PB23, TX=PB22 */
#if CONFIG_UART_SAM0_SERCOM5_BASE_ADDRESS

static const struct uart_sam0_dev_cfg uart_sam0_config_5 = {
	UART_SAM0_CONFIG_INIT(5),
	.pin_rx = {1, 23, PORT_PMUX_PMUXE_D_Val},
	.pin_tx = {1, 22, PORT_PMUX_PMUXE_D_Val},
};

UART_SAM0_DEVICE_INIT(5)
#endif
