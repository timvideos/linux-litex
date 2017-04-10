/*
 * Copyright (C) Hasjim Williams 2017
 * Authors:  Hasjim Williams <@>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Inspired by stm32-usart.c from STMicroelectronics (c)
 */

#if defined(CONFIG_SERIAL_STM32_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-direction.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/spinlock.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/tty.h>

#include "uart-litex.h"

static void litex_stop_tx(struct uart_port *port);
static void litex_transmit_chars(struct uart_port *port);

static inline struct litex_port *to_litex_port(struct uart_port *port)
{
	return container_of(port, struct litex_port, port);
}

static void litex_set_bits(struct uart_port *port, u32 reg, u32 bits)
{
#if 0
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val |= bits;
	writel_relaxed(val, port->membase + reg);
#endif
}

static void litex_clr_bits(struct uart_port *port, u32 reg, u32 bits)
{
#if 0
	u32 val;

	val = readl_relaxed(port->membase + reg);
	val &= ~bits;
	writel_relaxed(val, port->membase + reg);
#endif
}

static int litex_pending_rx(struct uart_port *port, u32 *sr, int *last_res,
			    bool threaded)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	enum dma_status status;
	struct dma_tx_state state;

	*sr = readl_relaxed(port->membase + ofs->isr);

	if (threaded && litex_port->rx_ch) {
		status = dmaengine_tx_status(litex_port->rx_ch,
					     litex_port->rx_ch->cookie,
					     &state);
		if ((status == DMA_IN_PROGRESS) &&
		    (*last_res != state.residue))
			return 1;
		else
			return 0;
	} else if (*sr & UART_SR_RXNE) {
		return 1;
	}
	return 0;
#endif
}

static unsigned long
litex_get_char(struct uart_port *port, u32 *sr, int *last_res)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	unsigned long c;

	if (litex_port->rx_ch) {
		c = litex_port->rx_buf[RX_BUF_L - (*last_res)--];
		if ((*last_res) == 0)
			*last_res = RX_BUF_L;
		return c;
	} else {
		return readl_relaxed(port->membase + ofs->rdr);
	}
#endif
}

static void litex_receive_chars(struct uart_port *port, bool threaded)
{
#if 0
	struct tty_port *tport = &port->state->port;
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	unsigned long c;
	u32 sr;
	char flag;
	static int last_res = RX_BUF_L;

	if (port->irq_wake)
		pm_wakeup_event(tport->tty->dev, 0);

	while (litex_pending_rx(port, &sr, &last_res, threaded)) {
		sr |= UART_SR_DUMMY_RX;
		c = litex_get_char(port, &sr, &last_res);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (sr & UART_SR_ERR_MASK) {
			if (sr & UART_SR_LBD) {
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (sr & UART_SR_ORE) {
				if (ofs->icr != UNDEF_REG)
					writel_relaxed(UART_ICR_ORECF,
						       port->membase +
						       ofs->icr);
				port->icount.overrun++;
			} else if (sr & UART_SR_PE) {
				port->icount.parity++;
			} else if (sr & UART_SR_FE) {
				port->icount.frame++;
			}

			sr &= port->read_status_mask;

			if (sr & UART_SR_LBD)
				flag = TTY_BREAK;
			else if (sr & UART_SR_PE)
				flag = TTY_PARITY;
			else if (sr & UART_SR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, c))
			continue;
		uart_insert_char(port, sr, UART_SR_ORE, c, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(tport);
	spin_lock(&port->lock);
#endif
}

static void litex_tx_dma_complete(void *arg)
{
#if 0
	struct uart_port *port = arg;
	struct litex_port *stm32port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &stm32port->info->ofs;
	unsigned int isr;
	int ret;

	ret = readl_relaxed_poll_timeout_atomic(port->membase + ofs->isr,
						isr,
						(isr & UART_SR_TC),
						10, 100000);

	if (ret)
		dev_err(port->dev, "terminal count not set\n");

	if (ofs->icr == UNDEF_REG)
		litex_clr_bits(port, ofs->isr, UART_SR_TC);
	else
		litex_set_bits(port, ofs->icr, UART_CR_TC);

	litex_clr_bits(port, ofs->cr3, UART_CR3_DMAT);
	stm32port->tx_dma_busy = false;

	/* Let's see if we have pending data to send */
	litex_transmit_chars(port);
#endif
}

static void litex_transmit_chars_pio(struct uart_port *port)
{
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int isr;
	int ret;

#if 0
	if (litex_port->tx_dma_busy) {
		litex_clr_bits(port, ofs->cr3, UART_CR3_DMAT);
		litex_port->tx_dma_busy = false;
	}
#endif

#if 0
	ret = readl_relaxed_poll_timeout_atomic(port->membase + ofs->isr,
						isr,
						(isr & UART_SR_TXE),
						10, 100);

	if (ret)
		dev_err(port->dev, "tx empty not set\n");
#endif

	// litex_set_bits(port, ofs->cr1, UART_CR1_TXEIE);

	writel_relaxed(xmit->buf[xmit->tail], port->membase);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	port->icount.tx++;
}

static void litex_transmit_chars_dma(struct uart_port *port)
{
#if 0
	struct litex_port *stm32port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &stm32port->info->ofs;
	struct circ_buf *xmit = &port->state->xmit;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_cookie_t cookie;
	unsigned int count, i;

	if (stm32port->tx_dma_busy)
		return;

	stm32port->tx_dma_busy = true;

	count = uart_circ_chars_pending(xmit);

	if (count > TX_BUF_L)
		count = TX_BUF_L;

	if (xmit->tail < xmit->head) {
		memcpy(&stm32port->tx_buf[0], &xmit->buf[xmit->tail], count);
	} else {
		size_t one = UART_XMIT_SIZE - xmit->tail;
		size_t two;

		if (one > count)
			one = count;
		two = count - one;

		memcpy(&stm32port->tx_buf[0], &xmit->buf[xmit->tail], one);
		if (two)
			memcpy(&stm32port->tx_buf[one], &xmit->buf[0], two);
	}

	desc = dmaengine_prep_slave_single(stm32port->tx_ch,
					   stm32port->tx_dma_buf,
					   count,
					   DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT);

	if (!desc) {
		for (i = count; i > 0; i--)
			litex_transmit_chars_pio(port);
		return;
	}

	desc->callback = litex_tx_dma_complete;
	desc->callback_param = port;

	/* Push current DMA TX transaction in the pending queue */
	cookie = dmaengine_submit(desc);

	/* Issue pending DMA TX requests */
	dma_async_issue_pending(stm32port->tx_ch);

	litex_clr_bits(port, ofs->isr, UART_SR_TC);
	litex_set_bits(port, ofs->cr3, UART_CR3_DMAT);

	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	port->icount.tx += count;
#endif
}

static void litex_transmit_chars(struct uart_port *port)
{
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	struct circ_buf *xmit = &port->state->xmit;

	if (port->x_char) {
		// xon/xoff char
#if 0
		if (litex_port->tx_dma_busy)
			litex_clr_bits(port, ofs->cr3, UART_CR3_DMAT);
#endif
		writel_relaxed(port->x_char, port->membase);
		port->x_char = 0;
		port->icount.tx++;
#if 0
		if (litex_port->tx_dma_busy)
			litex_set_bits(port, ofs->cr3, UART_CR3_DMAT);
#endif
		return;
	}

	if (uart_tx_stopped(port)) {
		litex_stop_tx(port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		litex_stop_tx(port);
		return;
	}

#if 0
	if (litex_port->tx_ch)
		litex_transmit_chars_dma(port);
	else
#endif
		litex_transmit_chars_pio(port);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		litex_stop_tx(port);
}

static irqreturn_t litex_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	u32 sr;

	spin_lock(&port->lock);

#if 0
	sr = readl_relaxed(port->membase + ofs->isr);

	if ((sr & UART_SR_RXNE) && !(litex_port->rx_ch))
		litex_receive_chars(port, false);
#endif

#if 0
	if ((sr & UART_SR_TXE) && !(litex_port->tx_ch))
#else
	if (1)
#endif
		litex_transmit_chars(port);

	spin_unlock(&port->lock);

#if 0
	if (litex_port->rx_ch)
		return IRQ_WAKE_THREAD;
	else
#endif
		return IRQ_HANDLED;
}

static irqreturn_t litex_threaded_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	struct litex_port *litex_port = to_litex_port(port);

	spin_lock(&port->lock);

#if 0
	if (litex_port->rx_ch)
		litex_receive_chars(port, true);
#endif

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static unsigned int litex_tx_empty(struct uart_port *port)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;

	return readl_relaxed(port->membase + ofs->isr) & UART_SR_TXE;
#endif
}

static void litex_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;

	if ((mctrl & TIOCM_RTS) && (port->status & UPSTAT_AUTORTS))
		litex_set_bits(port, ofs->cr3, UART_CR3_RTSE);
	else
		litex_clr_bits(port, ofs->cr3, UART_CR3_RTSE);
#endif
}

static unsigned int litex_get_mctrl(struct uart_port *port)
{
#if 0
	/* This routine is used to get signals of: DCD, DSR, RI, and CTS */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
#endif
}

/* Transmit stop */
static void litex_stop_tx(struct uart_port *port)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;

	litex_clr_bits(port, ofs->cr1, UART_CR1_TXEIE);
#endif
}

/* There are probably characters waiting to be transmitted. */
static void litex_start_tx(struct uart_port *port)
{
#if 0
	struct circ_buf *xmit = &port->state->xmit;

	if (uart_circ_empty(xmit))
		return;

	litex_transmit_chars(port);
#endif
}

/* Throttle the remote when input buffer is about to overflow. */
static void litex_throttle(struct uart_port *port)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	litex_clr_bits(port, ofs->cr1, UART_CR1_RXNEIE);
	spin_unlock_irqrestore(&port->lock, flags);
#endif
}

/* Unthrottle the remote, the input buffer can now accept data. */
static void litex_unthrottle(struct uart_port *port)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	litex_set_bits(port, ofs->cr1, UART_CR1_RXNEIE);
	spin_unlock_irqrestore(&port->lock, flags);
#endif
}

/* Receive stop */
static void litex_stop_rx(struct uart_port *port)
{
#if 0
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;

	litex_clr_bits(port, ofs->cr1, UART_CR1_RXNEIE);
#endif
}

/* Handle breaks - ignored by us */
static void litex_break_ctl(struct uart_port *port, int break_state)
{
}

static int litex_startup(struct uart_port *port)
{
	struct litex_port *litex_port = to_litex_port(port);
	// struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	const char *name = to_platform_device(port->dev)->name;
	u32 val;
	int ret;

	ret = request_threaded_irq(port->irq, litex_interrupt,
				   litex_threaded_interrupt,
				   IRQF_NO_SUSPEND, name, port);
	if (ret)
		return ret;

#if 0
	val = UART_CR1_RXNEIE | UART_CR1_TE | UART_CR1_RE;
	litex_set_bits(port, ofs->cr1, val);
#endif

	return 0;
}

static void litex_shutdown(struct uart_port *port)
{
	struct litex_port *litex_port = to_litex_port(port);
#if 0
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	struct litex_uart_config *cfg = &litex_port->info->cfg;
#endif
	u32 val;

#if 0
	val = UART_CR1_TXEIE | UART_CR1_RXNEIE | UART_CR1_TE | UART_CR1_RE;
	val |= BIT(cfg->uart_enable_bit);
	litex_clr_bits(port, ofs->cr1, val);

	free_irq(port->irq, port);
#endif
}

static void litex_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	struct litex_port *litex_port = to_litex_port(port);
#if 0
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
	struct litex_uart_config *cfg = &litex_port->info->cfg;
#endif
	unsigned int baud;
	u32 usartdiv, mantissa, fraction, oversampling;
	tcflag_t cflag = termios->c_cflag;
	u32 cr1, cr2, cr3;
	unsigned long flags;

#if 0
	if (!litex_port->hw_flow_control)
		cflag &= ~CRTSCTS;

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 8);

	spin_lock_irqsave(&port->lock, flags);

	/* Stop serial port and reset value */
	writel_relaxed(0, port->membase + ofs->cr1);

	cr1 = UART_CR1_TE | UART_CR1_RE | UART_CR1_RXNEIE;
	cr1 |= BIT(cfg->uart_enable_bit);
	cr2 = 0;
	cr3 = 0;

	if (cflag & CSTOPB)
		cr2 |= UART_CR2_STOP_2B;

	if (cflag & PARENB) {
		cr1 |= UART_CR1_PCE;
		if ((cflag & CSIZE) == CS8) {
			if (cfg->has_7bits_data)
				cr1 |= UART_CR1_M0;
			else
				cr1 |= UART_CR1_M;
		}
	}

	if (cflag & PARODD)
		cr1 |= UART_CR1_PS;

	port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	if (cflag & CRTSCTS) {
		port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
		cr3 |= UART_CR3_CTSE;
	}

	usartdiv = DIV_ROUND_CLOSEST(port->uartclk, baud);

	/*
	 * The UART supports 16 or 8 times oversampling.
	 * By default we prefer 16 times oversampling, so that the receiver
	 * has a better tolerance to clock deviations.
	 * 8 times oversampling is only used to achieve higher speeds.
	 */
	if (usartdiv < 16) {
		oversampling = 8;
		litex_set_bits(port, ofs->cr1, UART_CR1_OVER8);
	} else {
		oversampling = 16;
		litex_clr_bits(port, ofs->cr1, UART_CR1_OVER8);
	}

	mantissa = (usartdiv / oversampling) << UART_BRR_DIV_M_SHIFT;
	fraction = usartdiv % oversampling;
	writel_relaxed(mantissa | fraction, port->membase + ofs->brr);

	uart_update_timeout(port, cflag, baud);

	port->read_status_mask = UART_SR_ORE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_SR_PE | UART_SR_FE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= UART_SR_LBD;

	/* Characters to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask = UART_SR_PE | UART_SR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_SR_LBD;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_SR_ORE;
	}

	/* Ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_SR_DUMMY_RX;

	if (litex_port->rx_ch)
		cr3 |= UART_CR3_DMAR;

	writel_relaxed(cr3, port->membase + ofs->cr3);
	writel_relaxed(cr2, port->membase + ofs->cr2);
	writel_relaxed(cr1, port->membase + ofs->cr1);
#endif

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *litex_type(struct uart_port *port)
{
	return (port->type == PORT_LITEX) ? DRIVER_NAME : NULL;
}

/* serial core request to release uart iomem */
static void litex_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *res_mem;
	unsigned int res_size;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res_mem))
		return;
	res_size = resource_size(res_mem);

	release_mem_region(port->mapbase, res_size);
}

/* serial core request to claim uart iomem */
static int litex_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *res_mem;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res_mem))
		return -EINVAL;

	if (!request_mem_region(port->mapbase, resource_size(res_mem),
				"uart-litex_mem"))
		return -EBUSY;

	port->membase = devm_ioremap_nocache(port->dev, port->mapbase,
						resource_size(res_mem));
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, resource_size(res_mem));
		return -ENOMEM;
	}

	return 0;
}

static void litex_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		if (litex_request_port(port))
			return;
		port->type = PORT_LITEX;
	}
}

static int
litex_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* No user changeable parameters */
	return -EINVAL;
}

static void litex_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
#if 0
	struct litex_port *stm32port = container_of(port,
			struct litex_port, port);
	struct litex_uart_offsets *ofs = &stm32port->info->ofs;
	struct litex_uart_config *cfg = &stm32port->info->cfg;
	unsigned long flags = 0;

	switch (state) {
	case UART_PM_STATE_ON:
		clk_prepare_enable(stm32port->clk);
		break;
	case UART_PM_STATE_OFF:
		spin_lock_irqsave(&port->lock, flags);
		litex_clr_bits(port, ofs->cr1, BIT(cfg->uart_enable_bit));
		spin_unlock_irqrestore(&port->lock, flags);
		clk_disable_unprepare(stm32port->clk);
		break;
	}
#endif
}

static const struct uart_ops litex_uart_ops = {
	.tx_empty	= litex_tx_empty,
	.set_mctrl	= litex_set_mctrl,
	.get_mctrl	= litex_get_mctrl,
	.stop_tx	= litex_stop_tx,
	.start_tx	= litex_start_tx,
	.throttle	= litex_throttle,
	.unthrottle	= litex_unthrottle,
	.stop_rx	= litex_stop_rx,
	.break_ctl	= litex_break_ctl,
	.startup	= litex_startup,
	.shutdown	= litex_shutdown,
	.set_termios	= litex_set_termios,
	.pm		= litex_pm,
	.type		= litex_type,
	.release_port	= litex_release_port,
	.request_port	= litex_request_port,
	.config_port	= litex_config_port,
	.verify_port	= litex_verify_port,
};

static int litex_init_port(struct litex_port *stm32port,
			  struct platform_device *pdev)
{
	struct uart_port *port = &stm32port->port;
	struct resource *res;
	int ret;

	port->iotype	= UPIO_MEM;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &litex_uart_ops;
	port->dev	= &pdev->dev;
	port->irq	= platform_get_irq(pdev, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);
	port->mapbase = res->start;

	spin_lock_init(&port->lock);

#if 0
	stm32port->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(stm32port->clk))
		return PTR_ERR(stm32port->clk);

	/* Ensure that clk rate is correct by enabling the clk */
	ret = clk_prepare_enable(stm32port->clk);
	if (ret)
		return ret;

	stm32port->port.uartclk = clk_get_rate(stm32port->clk);
	if (!stm32port->port.uartclk)
		ret = -EINVAL;
#endif

	return ret;
}

static struct litex_port *litex_of_get_litex_port(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int id;

	if (!np)
		return NULL;

	id = of_alias_get_id(np, "serial");
	if (id < 0)
		id = 0;

	if (WARN_ON(id >= LITEX_MAX_PORTS))
		return NULL;

#if 0
	litex_ports[id].hw_flow_control = of_property_read_bool(np,
							"st,hw-flow-ctrl");
#endif
	litex_ports[id].port.line = id;
	return &litex_ports[id];
}

#ifdef CONFIG_OF
static const struct of_device_id litex_match[] = {
	{ .compatible = "litex,uart-litex", .data = &litex_info},
	{},
};

MODULE_DEVICE_TABLE(of, litex_match);
#endif

static int litex_of_dma_rx_probe(struct litex_port *stm32port,
				 struct platform_device *pdev)
{
#if 0
	struct litex_uart_offsets *ofs = &stm32port->info->ofs;
	struct uart_port *port = &stm32port->port;
	struct device *dev = &pdev->dev;
	struct dma_slave_config config;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_cookie_t cookie;
	int ret;

	/* Request DMA RX channel */
	stm32port->rx_ch = dma_request_slave_channel(dev, "rx");
	if (!stm32port->rx_ch) {
		dev_info(dev, "rx dma alloc failed\n");
		return -ENODEV;
	}
	stm32port->rx_buf = dma_alloc_coherent(&pdev->dev, RX_BUF_L,
						 &stm32port->rx_dma_buf,
						 GFP_KERNEL);
	if (!stm32port->rx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	/* Configure DMA channel */
	memset(&config, 0, sizeof(config));
	config.src_addr = port->mapbase + ofs->rdr;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	ret = dmaengine_slave_config(stm32port->rx_ch, &config);
	if (ret < 0) {
		dev_err(dev, "rx dma channel config failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	/* Prepare a DMA cyclic transaction */
	desc = dmaengine_prep_dma_cyclic(stm32port->rx_ch,
					 stm32port->rx_dma_buf,
					 RX_BUF_L, RX_BUF_P, DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "rx dma prep cyclic failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	/* No callback as dma buffer is drained on usart interrupt */
	desc->callback = NULL;
	desc->callback_param = NULL;

	/* Push current DMA transaction in the pending queue */
	cookie = dmaengine_submit(desc);

	/* Issue pending DMA requests */
	dma_async_issue_pending(stm32port->rx_ch);

	return 0;

config_err:
	dma_free_coherent(&pdev->dev,
			  RX_BUF_L, stm32port->rx_buf,
			  stm32port->rx_dma_buf);

alloc_err:
	dma_release_channel(stm32port->rx_ch);
	stm32port->rx_ch = NULL;

	return ret;
#endif
}

static int litex_of_dma_tx_probe(struct litex_port *stm32port,
				 struct platform_device *pdev)
{
#if 0
	struct litex_uart_offsets *ofs = &stm32port->info->ofs;
	struct uart_port *port = &stm32port->port;
	struct device *dev = &pdev->dev;
	struct dma_slave_config config;
	int ret;

	stm32port->tx_dma_busy = false;

	/* Request DMA TX channel */
	stm32port->tx_ch = dma_request_slave_channel(dev, "tx");
	if (!stm32port->tx_ch) {
		dev_info(dev, "tx dma alloc failed\n");
		return -ENODEV;
	}
	stm32port->tx_buf = dma_alloc_coherent(&pdev->dev, TX_BUF_L,
						 &stm32port->tx_dma_buf,
						 GFP_KERNEL);
	if (!stm32port->tx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	/* Configure DMA channel */
	memset(&config, 0, sizeof(config));
	config.dst_addr = port->mapbase + ofs->tdr;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	ret = dmaengine_slave_config(stm32port->tx_ch, &config);
	if (ret < 0) {
		dev_err(dev, "tx dma channel config failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	return 0;

config_err:
	dma_free_coherent(&pdev->dev,
			  TX_BUF_L, stm32port->tx_buf,
			  stm32port->tx_dma_buf);

alloc_err:
	dma_release_channel(stm32port->tx_ch);
	stm32port->tx_ch = NULL;

	return ret;
#endif
}

static int litex_serial_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct litex_port *litexport;
	int ret;

	litexport = litex_of_get_litex_port(pdev);
	if (!litexport)
		return -ENODEV;

	match = of_match_device(litex_match, &pdev->dev);
	if (match && match->data)
		litexport->info = (struct litex_uart_info *)match->data;
	else
		return -EINVAL;

	ret = litex_init_port(litexport, pdev);
	if (ret)
		return ret;

	ret = uart_add_one_port(&litex_uart_driver, &litexport->port);
	if (ret)
		return ret;


	platform_set_drvdata(pdev, &litexport->port);

	return 0;
}

static int litex_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct litex_port *litex_port = to_litex_port(port);
#if 0
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;

	litex_clr_bits(port, ofs->cr3, UART_CR3_DMAR);

	if (litex_port->rx_ch)
		dma_release_channel(litex_port->rx_ch);

	if (litex_port->rx_dma_buf)
		dma_free_coherent(&pdev->dev,
				  RX_BUF_L, litex_port->rx_buf,
				  litex_port->rx_dma_buf);

	litex_clr_bits(port, ofs->cr3, UART_CR3_DMAT);

	if (litex_port->tx_ch)
		dma_release_channel(litex_port->tx_ch);

	if (litex_port->tx_dma_buf)
		dma_free_coherent(&pdev->dev,
				  TX_BUF_L, litex_port->tx_buf,
				  litex_port->tx_dma_buf);

	clk_disable_unprepare(litex_port->clk);
#endif

	return uart_remove_one_port(&litex_uart_driver, port);
}


#ifdef CONFIG_SERIAL_UART_LITEX_CONSOLE
static void litex_console_putchar(struct uart_port *port, int ch)
{
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;

#if 0 // fingers crossed
	while (!(readl_relaxed(port->membase + ofs->isr) & UART_SR_TXE))
		cpu_relax();
#endif

#if 1 // fingers crossed
	writel_relaxed(ch, port->membase);
#endif
}

static void litex_console_write(struct console *co, const char *s, unsigned cnt)
{
	struct uart_port *port = &litex_ports[co->index].port;
	struct litex_port *litex_port = to_litex_port(port);
	struct litex_uart_offsets *ofs = &litex_port->info->ofs;
#if 0
	struct litex_uart_config *cfg = &litex_port->info->cfg;
#endif
	unsigned long flags;
	u32 old_cr1, new_cr1;
	int locked = 1;

	local_irq_save(flags);
	if (port->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&port->lock);
	else
		spin_lock(&port->lock);

#if 0
	/* Save and disable interrupts, enable the transmitter */
	old_cr1 = readl_relaxed(port->membase + ofs->cr1);
	new_cr1 = old_cr1 & ~UART_CR1_IE_MASK;
	new_cr1 |=  UART_CR1_TE | BIT(cfg->uart_enable_bit);
	writel_relaxed(new_cr1, port->membase + ofs->cr1);

	uart_console_write(port, s, cnt, litex_console_putchar);

	/* Restore interrupt state */
	writel_relaxed(old_cr1, port->membase + ofs->cr1);
#endif

	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);
}

static int litex_console_setup(struct console *co, char *options)
{
	struct litex_port *litexport;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= LITEX_MAX_PORTS)
		return -ENODEV;

	litexport = &litex_ports[co->index];

#if 1 // fingers crossed
	static char __litex_console_setup[20] = "litex_console_setup\n";
	uint8_t i;
	for (i = 0; i<sizeof(__litex_console_setup); i++) {
		writel_relaxed(__litex_console_setup[i], litexport->port.membase /* 0xe0001000 */);
	}
#endif

	/*
	 * This driver does not support early console initialization,
	 * so we only expect this to be called during the uart port
	 * registration when the driver gets probed and the port
	 * should be mapped at that point.
	 */
	if (litexport->port.mapbase == 0 || litexport->port.membase == NULL)
		return -ENXIO;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&litexport->port, co, baud, parity, bits, flow);
}

static void early_litex_putc(struct uart_port *port, int c)
{
	/*
	 * Limit how many times we'll spin waiting for TX FIFO status.
	 * This will prevent lockups if the base address is incorrectly
	 * set, or any other issue on the UARTLITE.
	 * This limit is pretty arbitrary, unless we are at about 10 baud
	 * we'll never timeout on a working UART.
	 */

	unsigned retries = 1000000;
	/* read tx status bit - 0x4 offset */
	while (--retries && (readl(port->membase + 4)))
		;

	/* Only attempt the iowrite if we didn't timeout */
	/* write to TXRX_FIFO - 0x0 offset */
	if (retries)
		writel(c & 0xff, port->membase);
}

static void early_litex_write(struct console *console,
				 const char *s, unsigned n)
{
	struct earlycon_device *device = console->data;
	uart_console_write(&device->port, s, n, early_litex_putc);
}

static int __init early_litex_setup(struct earlycon_device *device,
				       const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = early_litex_write;
	return 0;
}
EARLYCON_DECLARE(litex, early_litex_setup);
OF_EARLYCON_DECLARE(litex_a, "litex", early_litex_setup);

static struct console litex_console = {
	.name		= LITEX_SERIAL_NAME,
	.device		= uart_console_device,
	.write		= litex_console_write,
	.setup		= litex_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &litex_uart_driver,
};

#define LITEX_SERIAL_CONSOLE (&litex_console)

#else
#define LITEX_SERIAL_CONSOLE NULL
#endif /* CONFIG_SERIAL_LITEX_CONSOLE */

static struct uart_driver litex_uart_driver = {
	.driver_name	= DRIVER_NAME,
	.dev_name	= LITEX_SERIAL_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		= LITEX_MAX_PORTS,
	.cons		= LITEX_SERIAL_CONSOLE,
};

static struct platform_driver litex_serial_driver = {
	.probe		= litex_serial_probe,
	.remove		= litex_serial_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(litex_match),
	},
};

static int __init uart_init(void)
{
#if 0 // fingers crossed - BROKEN (no 'port' declared here)
	static char* __uart_init = "uart_init\n";
	uint8_t i;
	for (i = 0; i<sizeof(__uart_init); i++) {
		writel_relaxed(__uart_init[i], port->membase /* 0xe0001000 */);
	}
#endif

	static char banner[] __initdata = "Lite-X UART driver initialized";
	int ret;

	pr_info("%s\n", banner);

	ret = uart_register_driver(&litex_uart_driver);
	if (ret)
		return ret;

	// pr_info("uart-litex %d\n", __LINE__);

	ret = platform_driver_register(&litex_serial_driver);
	if (ret)
		uart_unregister_driver(&litex_uart_driver);

	// pr_info("uart-litex %d\n", __LINE__);

	return ret;
}

static void __exit uart_exit(void)
{
	platform_driver_unregister(&litex_serial_driver);
	uart_unregister_driver(&litex_uart_driver);
}

module_init(uart_init);
module_exit(uart_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("Enjoy-Digital Lite-X serial port driver");
MODULE_LICENSE("GPL v2");
