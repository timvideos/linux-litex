/*
 * uart-litex.c: Serial driver for Lite-X serial controller
 *
 * Copyright (C) Hasjim Williams 2017
 * Authors:  Hasjim Williams <@>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#define LITEX_UART_NAME		"ttyLX"
#define LITEX_NR_UARTS		16

#define LITEX_UART_RX		0x00
#define LITEX_UART_TX		0x00
#define LITEX_UART_TXFULL	0x04
#define LITEX_UART_RXEMPTY	0x08
#define LITEX_UART_EV_STATUS	0x0c
#define LITEX_UART_EV_PENDING	0x10
#define LITEX_UART_EV_ENABLE	0x14
#define LITEX_UART_REGION		16

#define LITEX_EV_TX	1
#define LITEX_EV_RX	2

#if 0 // delete
#define LITEX_STATUS_RXVALID	0x01
#define LITEX_STATUS_RXFULL	0x02
#define LITEX_STATUS_TXEMPTY	0x04
#define LITEX_STATUS_TXFULL	0x08
#define LITEX_STATUS_IE		0x10
#define LITEX_STATUS_OVERRUN	0x20
#define LITEX_STATUS_FRAME	0x40
#define LITEX_STATUS_PARITY	0x80
#endif

#if 0 // delete
#define LITEX_CONTROL_RST_TX	0x01
#define LITEX_CONTROL_RST_RX	0x02
#define LITEX_CONTROL_IE	0x10
#endif


struct litex_reg_ops {
	u32 (*in)(void __iomem *addr);
	void (*out)(u32 val, void __iomem *addr);
};

static u32 litex_inbe32(void __iomem *addr)
{
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	return ioread32be(addr);
}

static void litex_outbe32(u32 val, void __iomem *addr)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	iowrite32be(val, addr);
}

static const struct litex_reg_ops litex_be = {
	.in = litex_inbe32,
	.out = litex_outbe32,
};

static u32 litex_inle32(void __iomem *addr)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	return ioread32(addr);
}

static void litex_outle32(u32 val, void __iomem *addr)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	iowrite32(val, addr);
}

static const struct litex_reg_ops litex_le = {
	.in = litex_inle32,
	.out = litex_outle32,
};

static inline u32 uart_in32(u32 offset, struct uart_port *port)
{
	const struct litex_reg_ops *reg_ops = port->private_data;
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	return reg_ops->in(port->membase + offset);
}

static inline void uart_out32(u32 val, u32 offset, struct uart_port *port)
{
	const struct litex_reg_ops *reg_ops = port->private_data;
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	reg_ops->out(val, port->membase + offset);
}

static struct uart_port litex_uart_ports[LITEX_NR_UARTS];

/* ---------------------------------------------------------------------
 * Core UART driver operations
 */

static int litex_uart_receive(struct uart_port *port, int stat)
{
	struct tty_port *tport = &port->state->port;
	unsigned char ch = 0;
	char flag = TTY_NORMAL;
	//printk(KERN_INFO "%s %s %x\n", __FILE__, __func__, stat);

#if 0 // TODO
	if ((stat & (LITEX_STATUS_RXVALID | LITEX_STATUS_OVERRUN
		     | LITEX_STATUS_FRAME)) == 0)
		return 0;
#endif
	if ((stat & 1) == 1) {
		return 0;
	}

	/* RXEMPTY */
	if ((stat & 1) == 0) {
		port->icount.rx++;
		ch = uart_in32(LITEX_UART_RX, port);

#if 0
		if (stat & LITEX_STATUS_PARITY)
			port->icount.parity++;
#endif
	}

#if 0 // TODO
	if (stat & LITEX_STATUS_OVERRUN)
		port->icount.overrun++;

	if (stat & LITEX_STATUS_FRAME)
		port->icount.frame++;


	/* drop byte with parity error if IGNPAR specificed */
	if (stat & port->ignore_status_mask & LITEX_STATUS_PARITY)
		stat &= ~LITEX_STATUS_RXVALID;
#endif

	stat &= port->read_status_mask;

#if 0 // error TODO
	if (stat & LITEX_STATUS_PARITY)
		flag = TTY_PARITY;
#endif

	stat &= ~port->ignore_status_mask;

	tty_insert_flip_char(tport, ch, flag);
#if 0 // error TODO
	if (stat & LITEX_STATUS_RXVALID)

	if (stat & LITEX_STATUS_FRAME)
		tty_insert_flip_char(tport, 0, TTY_FRAME);

	if (stat & LITEX_STATUS_OVERRUN)
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
#endif

	return 1;
}

static int litex_uart_transmit(struct uart_port *port, int stat)
{
	struct circ_buf *xmit  = &port->state->xmit;

	//printk(KERN_INFO "%s %s %s\n", __FILE__, __func__, stat);
#if 0 // TXFULL
	if (stat & LITEX_STATUS_TXFULL)
		return 0;
#endif
	if (stat & 1) {
		return 0;
	}

	if (port->x_char) {
		uart_out32(port->x_char, LITEX_UART_TX, port);
		port->x_char = 0;
		port->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return 0;

	uart_out32(xmit->buf[xmit->tail], LITEX_UART_TX, port);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
	port->icount.tx++;

	/* wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return 1;
}

static irqreturn_t litex_uart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	int stat, busy, n, r = 0;
	unsigned long flags;

	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	// Clear the pending IRQs
	r = uart_in32(LITEX_UART_EV_PENDING, port);
	//printk(KERN_INFO "%s clearing pending EVs %x\n", __FILE__, r);
	uart_out32(r, LITEX_UART_EV_PENDING, port);

	busy = 0;

	do {
		//spin_lock_irqsave(&port->lock, flags);
		stat = uart_in32(LITEX_UART_RXEMPTY, port);
		busy = litex_uart_receive(port, stat);
		//spin_unlock_irqrestore(&port->lock, flags);
		n++;
	} while (busy);

	busy = 0;

	// tx
	do {
		//spin_lock_irqsave(&port->lock, flags);
		stat = uart_in32(LITEX_UART_TXFULL, port);
		busy = litex_uart_transmit(port, stat);
		//spin_unlock_irqrestore(&port->lock, flags);
		n++;
	} while (busy);

	/* work done? */
	if (n > 1) {
		tty_flip_buffer_push(&port->state->port);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static unsigned int litex_uart_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret;

	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	spin_lock_irqsave(&port->lock, flags);
#if 0 // TODO error
	ret = uart_in32(LITEX_STATUS, port);
#else
	ret = uart_in32(LITEX_UART_TXFULL, port);
#endif
	spin_unlock_irqrestore(&port->lock, flags);

	return 0; // TIOCSER_TEMT; // TODO - ret & LITEX_STATUS_TXEMPTY ? TIOCSER_TEMT : 0;
	// return (ret & 1) ? TIOCSER_TEMT : 0;
}

static unsigned int litex_uart_get_mctrl(struct uart_port *port)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void litex_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	/* N/A */
}

static void litex_uart_stop_tx(struct uart_port *port)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	/* N/A */
}

static void litex_uart_start_tx(struct uart_port *port)
{
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
#if 0 // TODO error
	litex_uart_transmit(port, uart_in32(LITEX_STATUS, port));
#else
	// litex_uart_transmit(port, uart_in32(LITEX_UART_TXFULL, port));
	litex_uart_transmit(port, 0);
#endif
}

static void litex_uart_stop_rx(struct uart_port *port)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	/* don't forward any more data (like !CREAD) */
#if 0 // TODO
	port->ignore_status_mask = LITEX_STATUS_RXVALID | LITEX_STATUS_PARITY
		| LITEX_STATUS_FRAME | LITEX_STATUS_OVERRUN;
#endif
}

static void litex_uart_break_ctl(struct uart_port *port, int ctl)
{
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	/* N/A */
}

static int litex_uart_startup(struct uart_port *port)
{
	int ret;
	unsigned int r;
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	printk(KERN_INFO "%s requested irq %d\n", __FILE__, port->irq);

#if 1 // TODO enable IRQ and ISR
	ret = request_irq(port->irq, litex_uart_isr, IRQF_TRIGGER_LOW /* IRQF_SHARED */ /* IRQF_TRIGGER_RISING */,
			  "litex_uart", port);
	printk(KERN_INFO "%s ret %d\n", __FILE__, ret);
	if (ret) {
		// printk(KERN_INFO "%s ret %d\n", __FILE__, ret);
		if (ret == -22) {
			printk(KERN_INFO "%s %d -EINVAL\n", __FILE__, __LINE__);
		}
		return ret;
	}
#endif

	r = uart_in32(LITEX_UART_TXFULL, port);
	printk(KERN_INFO "%s TXFULL %x\n", __FILE__, r);
	r = uart_in32(LITEX_UART_RXEMPTY, port);
	printk(KERN_INFO "%s RXEMPTY %x\n", __FILE__, r);
	r = uart_in32(LITEX_UART_EV_STATUS, port);
	printk(KERN_INFO "%s EV_STATUS %x\n", __FILE__, r);
	r = uart_in32(LITEX_UART_EV_PENDING, port);
	if (r != 0) {
		printk(KERN_INFO "%s clearing pending EVs\n", __FILE__);
		uart_out32(r, LITEX_UART_EV_PENDING, port);
	}

	printk(KERN_INFO "%s EV_PENDING %x\n", __FILE__, r);
	r = uart_in32(LITEX_UART_EV_ENABLE, port);
	printk(KERN_INFO "%s EV_ENABLE %x\n", __FILE__, r);
	if (r == 0) {
		printk(KERN_INFO "%s enabling EV\n", __FILE__);
		uart_out32(0x3, LITEX_UART_EV_ENABLE, port);
	}

#if 0 // TODO error
	uart_out32(LITEX_CONTROL_RST_RX | LITEX_CONTROL_RST_TX,
		LITEX_CONTROL, port);
	uart_out32(LITEX_CONTROL_IE, LITEX_CONTROL, port);
#endif

	return 0;
}

static void litex_uart_shutdown(struct uart_port *port)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
#if 0 // TODO error
	uart_out32(0, LITEX_CONTROL, port);
	uart_in32(LITEX_CONTROL, port); /* dummy */
#endif
	free_irq(port->irq, port);
}

static void litex_uart_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud;

	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	spin_lock_irqsave(&port->lock, flags);

#if 0
	port->read_status_mask = LITEX_STATUS_RXVALID | LITEX_STATUS_OVERRUN
		| LITEX_STATUS_TXFULL;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |=
			LITEX_STATUS_PARITY | LITEX_STATUS_FRAME;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= LITEX_STATUS_PARITY
			| LITEX_STATUS_FRAME | LITEX_STATUS_OVERRUN;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |=
			LITEX_STATUS_RXVALID | LITEX_STATUS_PARITY
			| LITEX_STATUS_FRAME | LITEX_STATUS_OVERRUN;
#endif

	/* update timeout */
	baud = uart_get_baud_rate(port, termios, old, 0, 460800);
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *litex_uart_type(struct uart_port *port)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	return port->type == PORT_UART_LITEX ? "litex_uart" : NULL;
}

static void litex_uart_release_port(struct uart_port *port)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	release_mem_region(port->mapbase, LITEX_UART_REGION);
	iounmap(port->membase);
	port->membase = NULL;
}

static int litex_uart_request_port(struct uart_port *port)
{
	int ret;
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	pr_info("litex console: port=%p; port->mapbase=%llx\n",
		 port, (unsigned long long) port->mapbase);

	if (!request_mem_region(port->mapbase, LITEX_UART_REGION, "litex_uart")) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, LITEX_UART_REGION);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, LITEX_UART_REGION);
		return -EBUSY;
	}

	port->private_data = (void *)&litex_be;

#if 0 // TODO error
	ret = uart_in32(LITEX_CONTROL, port);
	uart_out32(LITEX_CONTROL_RST_TX, LITEX_CONTROL, port);
	ret = uart_in32(LITEX_STATUS, port);
	/* Endianess detection */
	if ((ret & LITEX_STATUS_TXEMPTY) != LITEX_STATUS_TXEMPTY)
		port->private_data = (void *)&litex_le;
#endif

	return 0;
}

static void litex_uart_config_port(struct uart_port *port, int flags)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	if (!litex_uart_request_port(port))
		port->type = PORT_UART_LITEX;
}

static int litex_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

#ifdef CONFIG_CONSOLE_POLL
static int litex_uart_get_poll_char(struct uart_port *port)
{
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	return NO_POLL_CHAR; // hack - delete me

#if 0 // TODO RX
	if (!(uart_in32(LITEX_STATUS, port) & LITEX_STATUS_RXVALID))
		return NO_POLL_CHAR;
#endif

	return uart_in32(LITEX_UART_RX, port);
}

static void litex_uart_put_poll_char(struct uart_port *port, unsigned char ch)
{
	// printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

#if 0 // TODO TXFULL
	while (uart_in32(LITEX_STATUS, port) & LITEX_STATUS_TXFULL)
		cpu_relax();
#else
	// while (uart_in32(LITEX_UART_TXFULL, port) & 1)
		// cpu_relax();

	// msleep(10);
#endif

	/* write char to device */
	uart_out32(ch, LITEX_UART_TX, port);
}
#endif

static const struct uart_ops litex_uart_ops = {
	.tx_empty	= litex_uart_tx_empty,
	.set_mctrl	= litex_uart_set_mctrl,
	.get_mctrl	= litex_uart_get_mctrl,
	.stop_tx	= litex_uart_stop_tx,
	.start_tx	= litex_uart_start_tx,
	.stop_rx	= litex_uart_stop_rx,
	.break_ctl	= litex_uart_break_ctl,
	.startup	= litex_uart_startup,
	.shutdown	= litex_uart_shutdown,
	.set_termios	= litex_uart_set_termios,
	.type		= litex_uart_type,
	.release_port	= litex_uart_release_port,
	.request_port	= litex_uart_request_port,
	.config_port	= litex_uart_config_port,
	.verify_port	= litex_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= litex_uart_get_poll_char,
	.poll_put_char	= litex_uart_put_poll_char,
#endif
};

/* ---------------------------------------------------------------------
 * Console driver operations
 */

#ifdef CONFIG_SERIAL_UART_LITEX_CONSOLE
static void litex_uart_console_wait_tx(struct uart_port *port)
{
	u8 val;
	unsigned long timeout;

	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	/*
	 * Spin waiting for TX fifo to have space available.
	 * When using the Microblaze Debug Module this can take up to 1s
	 */
	timeout = jiffies + msecs_to_jiffies(1000);
#if 0 // TODO TXFULL
	while (1) {
		val = uart_in32(LITEX_STATUS, port);
		if ((val & LITEX_STATUS_TXFULL) == 0)
			break;
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				 "timeout waiting for TX buffer empty\n");
			break;
		}
		cpu_relax();
	}
#else
	while (1) {
		val = uart_in32(LITEX_UART_TXFULL, port);
		if ((val & 1) == 0)
			break;
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				 "timeout waiting for TX buffer empty\n");
			break;
		}
		cpu_relax();
	}

	// msleep(1);
#endif
}

static void litex_uart_console_putchar(struct uart_port *port, int ch)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
#if 0 // TODO TXFULL
	litex_uart_console_wait_tx(port);
#endif
	uart_out32(ch, LITEX_UART_TX, port);
}

static void litex_uart_console_write(struct console *co, const char *s,
				unsigned int count)
{
	struct uart_port *port = &litex_uart_ports[co->index];
	unsigned long flags;
	unsigned int ier;
	int locked = 1;

	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	if (oops_in_progress) {
		locked = spin_trylock_irqsave(&port->lock, flags);
	} else
		spin_lock_irqsave(&port->lock, flags);

	/* save and disable interrupt */
#if 0 // TODO interrupt
	ier = uart_in32(LITEX_STATUS, port) & LITEX_STATUS_IE;
	uart_out32(0, LITEX_CONTROL, port);
#endif

	uart_console_write(port, s, count, litex_uart_console_putchar);

	litex_uart_console_wait_tx(port);

	/* restore interrupt state */
#if 0 // TODO interrupt
	if (ier)
		uart_out32(LITEX_CONTROL_IE, LITEX_CONTROL, port);
#endif

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

static int litex_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	if (co->index < 0 || co->index >= LITEX_NR_UARTS)
		return -EINVAL;

	port = &litex_uart_ports[co->index];

	/* Has the device been initialized yet? */
	if (!port->mapbase) {
		pr_info("console on ttyLX%i not present\n", co->index);
		return -ENODEV;
	}

	/* not initialized yet? */
	if (!port->membase) {
		if (litex_uart_request_port(port))
			return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver litex_uart_driver;

static struct console litex_uart_console = {
	.name	= LITEX_UART_NAME,
	.write	= litex_uart_console_write,
	.device	= uart_console_device,
	.setup	= litex_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1, /* Specified on the cmdline (e.g. console=ttyLX0 ) */
	.data	= &litex_uart_driver,
};

static int __init litex_uart_console_init(void)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	register_console(&litex_uart_console);
	return 0;
}

console_initcall(litex_uart_console_init);

static void early_litex_uart_putc(struct uart_port *port, int c)
{
	/*
	 * Limit how many times we'll spin waiting for TX FIFO status.
	 * This will prevent lockups if the base address is incorrectly
	 * set, or any other issue on the UART_LITEX.
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
		writeb(c & 0xff, port->membase);
}

static void early_litex_uart_write(struct console *console,
				 const char *s, unsigned n)
{
	struct earlycon_device *device = console->data;
	uart_console_write(&device->port, s, n, early_litex_uart_putc);
}

static int __init early_litex_uart_setup(struct earlycon_device *device,
				       const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = early_litex_uart_write;
	return 0;
}
EARLYCON_DECLARE(litex_uart, early_litex_uart_setup);
OF_EARLYCON_DECLARE(litex_uart_a, "litex_uart", early_litex_uart_setup);

#endif /* CONFIG_SERIAL_UART_LITEX_CONSOLE */

static struct uart_driver litex_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "litex_uart",
	.dev_name	= LITEX_UART_NAME,
	.major		= 204,
	.minor		= 187,
	.nr		= LITEX_NR_UARTS,
#ifdef CONFIG_SERIAL_UART_LITEX_CONSOLE
	.cons		= &litex_uart_console,
#else
#error "Blah!"
#endif
};

/* ---------------------------------------------------------------------
 * Port assignment functions (mapping devices to uart_port structures)
 */

/** litex_uart_assign: register a litex device with the driver
 *
 * @dev: pointer to device structure
 * @id: requested id number.  Pass -1 for automatic port assignment
 * @base: base address of litex registers
 * @irq: irq number for litex
 *
 * Returns: 0 on success, <0 otherwise
 */
static int litex_uart_assign(struct device *dev, int id, u32 base, int irq)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	struct uart_port *port;
	int rc;

	/* if id = -1; then scan for a free id and use that */
	if (id < 0) {
		for (id = 0; id < LITEX_NR_UARTS; id++)
			if (litex_uart_ports[id].mapbase == 0)
				break;
	}
	if (id < 0 || id >= LITEX_NR_UARTS) {
		dev_err(dev, "%s%i too large\n", LITEX_UART_NAME, id);
		return -EINVAL;
	}

	if ((litex_uart_ports[id].mapbase) && (litex_uart_ports[id].mapbase != base)) {
		dev_err(dev, "cannot assign to %s%i; it is already in use\n",
			LITEX_UART_NAME, id);
		return -EBUSY;
	}

	port = &litex_uart_ports[id];

	spin_lock_init(&port->lock);
	port->fifosize = 16;
	port->regshift = 2;
	port->iotype = UPIO_MEM;
	port->iobase = 1; /* mark port in use */
	port->mapbase = base;
	port->membase = NULL;
	port->ops = &litex_uart_ops;
	port->irq = irq;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = dev;
	port->type = PORT_UNKNOWN;
	port->line = id;

	dev_set_drvdata(dev, port);

	/* Register the port */
	rc = uart_add_one_port(&litex_uart_driver, port);
	if (rc) {
		dev_err(dev, "uart_add_one_port() failed; err=%i\n", rc);
		port->mapbase = 0;
		dev_set_drvdata(dev, NULL);
		return rc;
	}

	return 0;
}

/** litex_uart_release: register a litex device with the driver
 *
 * @dev: pointer to device structure
 */
static int litex_uart_release(struct device *dev)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	struct uart_port *port = dev_get_drvdata(dev);
	int rc = 0;

	if (port) {
		rc = uart_remove_one_port(&litex_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}

	return rc;
}

/* ---------------------------------------------------------------------
 * Platform bus binding
 */

#if defined(CONFIG_OF)
/* Match table for of_platform binding */
static const struct of_device_id litex_uart_of_match[] = {
	{ .compatible = "litex,litex_uart" /* , .data = &litex_uart_info */ },
	{}
};
MODULE_DEVICE_TABLE(of, litex_uart_of_match);
#endif /* CONFIG_OF */

static int litex_uart_probe(struct platform_device *pdev)
{
	struct resource *res;
	int irq;
	int id = pdev->id;
#ifdef CONFIG_OF
	const __be32 *prop;

	prop = of_get_property(pdev->dev.of_node, "port-number", NULL);
	if (prop)
		id = be32_to_cpup(prop);
#endif
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	printk_once(KERN_INFO "%s %s %d\n", __FILE__, __func__, __LINE__);

	// irq 0 is valid on openrisc
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENXIO;

	printk_once(KERN_INFO "%s %s %d\n", __FILE__, __func__, __LINE__);

	return litex_uart_assign(&pdev->dev, id, res->start, irq);
}

static int litex_uart_remove(struct platform_device *pdev)
{
	return litex_uart_release(&pdev->dev);
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:litex");

static struct platform_driver litex_uart_platform_driver = {
	.probe = litex_uart_probe,
	.remove = litex_uart_remove,
	.driver = {
		.name  = "litex_uart",
		.of_match_table = of_match_ptr(litex_uart_of_match),
	},
};

/* ---------------------------------------------------------------------
 * Module setup/teardown
 */

static int __init litex_uart_init(void)
{
	int ret;
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);

	pr_info("litex: calling uart_register_driver()\n");
	ret = uart_register_driver(&litex_uart_driver);
	if (ret)
		goto err_uart;

	pr_info("litex: calling platform_driver_register()\n");
	ret = platform_driver_register(&litex_uart_platform_driver);
	if (ret)
		goto err_plat;

	return 0;

err_plat:
	uart_unregister_driver(&litex_uart_driver);
err_uart:
	pr_err("registering litex driver failed: err=%i", ret);
	return ret;
}

static void __exit litex_uart_exit(void)
{
	printk_once(KERN_INFO "%s %s\n", __FILE__, __func__);
	platform_driver_unregister(&litex_uart_platform_driver);
	uart_unregister_driver(&litex_uart_driver);
}

module_init(litex_uart_init);
module_exit(litex_uart_exit);

MODULE_AUTHOR("Hasjim Williams <>");
MODULE_DESCRIPTION("Lite-X UART serial driver");
MODULE_LICENSE("GPL");
