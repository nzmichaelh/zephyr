/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <init.h>
#include <misc/__assert.h>
#include <soc.h>
#include <uart.h>

/* Device constant configuration parameters */
struct uart_samd_dev_cfg {
	SercomUsart *regs;
	struct soc_gpio_pin pin_rx;
	struct soc_gpio_pin pin_tx;
	u32_t baudrate;
	u32_t ctrla;
	u32_t pm_apbcmask;
	u16_t gclk_clkctrl_id;
};

/* Device run time data */
struct uart_samd_dev_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_t cb;
#endif
};

#define DEV_CFG(dev)							       \
	((const struct uart_samd_dev_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct uart_samd_dev_data * const)(dev)->driver_data)

static void uart_samd_isr(void *arg);

static void wait_synchronization(SercomUsart *const usart)
{
	while ((usart->SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_MASK) != 0) {
	}
}

static int uart_samd_set_baudrate(SercomUsart *const usart, u32_t baudrate,
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

static int uart_samd_init(struct device *dev)
{
	int retval;
	const struct uart_samd_dev_cfg *const cfg = DEV_CFG(dev);
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

	retval = uart_samd_set_baudrate(usart, cfg->baudrate,
					SOC_ATMEL_SAMD_GCLK0_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}

	usart->CTRLA.bit.ENABLE = 1;
	wait_synchronization(usart);
	return 0;
}

static int uart_samd_poll_in(struct device *dev, unsigned char *c)
{
	SercomUsart *const usart = DEV_CFG(dev)->regs;

	if (!usart->INTFLAG.bit.RXC) {
		return -EBUSY;
	}

	*c = (unsigned char)usart->DATA.reg;
	return 0;
}

static unsigned char uart_samd_poll_out(struct device *dev, unsigned char c)
{
	SercomUsart *const usart = DEV_CFG(dev)->regs;

	while (!usart->INTFLAG.bit.DRE) {
	}

	/* send a character */
	usart->DATA.reg = c;
	return c;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

static void uart_samd_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_samd_dev_data *const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev);
	}
}

static void uart_samd_irq_tx_disable(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	regs->INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

static void uart_samd_irq_rx_enable(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	regs->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
}

static void uart_samd_irq_rx_disable(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	regs->INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
}

static int uart_samd_irq_rx_ready(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	return regs->INTFLAG.bit.RXC != 0;
}

static int uart_samd_fifo_read(struct device *dev, u8_t *rx_data,
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

static int uart_samd_irq_is_pending(struct device *dev)
{
	SercomUsart *const regs = DEV_CFG(dev)->regs;

	return (regs->INTENSET.reg & regs->INTFLAG.reg) != 0;
}

static int uart_samd_irq_update(struct device *dev) { return 1; }

static void uart_samd_irq_callback_set(struct device *dev,
				       uart_irq_callback_t cb)
{
	struct uart_samd_dev_data *const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
}
#endif

static const struct uart_driver_api uart_samd_driver_api = {
	.poll_in = uart_samd_poll_in,
	.poll_out = uart_samd_poll_out,
#if CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_read = uart_samd_fifo_read,
	.irq_tx_disable = uart_samd_irq_tx_disable,
	.irq_rx_enable = uart_samd_irq_rx_enable,
	.irq_rx_disable = uart_samd_irq_rx_disable,
	.irq_rx_ready = uart_samd_irq_rx_ready,
	.irq_is_pending = uart_samd_irq_is_pending,
	.irq_update = uart_samd_irq_update,
	.irq_callback_set = uart_samd_irq_callback_set,
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SAMD_INIT_DEFN(addr, n)					       \
	static int uart_samd_init_##n(struct device *dev)		       \
	{								       \
		uart_samd_init(dev);					       \
		IRQ_CONNECT(ATMEL_SAMD_USART_##addr##_IRQ_0,		       \
			    ATMEL_SAMD_USART_##addr##_IRQ_0_PRIORITY,	       \
			    uart_samd_isr, DEVICE_GET(uart_samd_##n), 0);      \
		irq_enable(ATMEL_SAMD_USART_##addr##_IRQ_0);		       \
		return 0;						       \
	}
#else
#define UART_SAMD_INIT_DEFN(addr, n)					       \
	static int uart_samd_init_##n(struct device *dev)		       \
	{								       \
		return uart_samd_init(dev);				       \
	}
#endif

#define UART_SAMD_DEVICE_INIT(addr, n)					       \
	static int uart_samd_init_##n(struct device *dev);		       \
									       \
	static struct uart_samd_dev_data uart_samd_data_##n = {};	       \
									       \
	DEVICE_AND_API_INIT(uart_samd_##n, ATMEL_SAMD_USART_##addr##_LABEL,    \
			    uart_samd_init_##n, &uart_samd_data_##n,	       \
			    &uart_samd_config_##n, PRE_KERNEL_1,	       \
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		       \
			    &uart_samd_driver_api);			       \
	UART_SAMD_INIT_DEFN(addr, n)

/* SERCOM0 */
#ifdef ATMEL_SAMD_USART_42000800_BASE_ADDRESS

static const struct uart_samd_dev_cfg uart_samd_config_0 = {
	.regs = (SercomUsart *)ATMEL_SAMD_USART_42000800_BASE_ADDRESS,
	.pin_rx = {0, 11, PORT_PMUX_PMUXE_C_Val},
	.pin_tx = {0, 10, PORT_PMUX_PMUXE_C_Val},
	.baudrate = ATMEL_SAMD_USART_42000800_CURRENT_SPEED,
	.pm_apbcmask = PM_APBCMASK_SERCOM0,
	.gclk_clkctrl_id = GCLK_CLKCTRL_ID_SERCOM0_CORE,
	.ctrla = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1),
};

UART_SAMD_DEVICE_INIT(42000800, 0)
#endif

/* SERCOM5 */
#ifdef ATMEL_SAMD_USART_42001C00_BASE_ADDRESS

static const struct uart_samd_dev_cfg uart_samd_config_5 = {
	.regs = (SercomUsart *)ATMEL_SAMD_USART_42001C00_BASE_ADDRESS,
	.pin_rx = {1, 23, PORT_PMUX_PMUXE_D_Val},
	.pin_tx = {1, 22, PORT_PMUX_PMUXE_D_Val},
	.baudrate = ATMEL_SAMD_USART_42000800_CURRENT_SPEED,
	.pm_apbcmask = PM_APBCMASK_SERCOM5,
	.gclk_clkctrl_id = GCLK_CLKCTRL_ID_SERCOM5_CORE,
	.ctrla = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1),
	.baudrate = ATMEL_SAMD_USART_42001C00_CURRENT_SPEED,
};

UART_SAMD_DEVICE_INIT(42001C00, 5)
#endif
