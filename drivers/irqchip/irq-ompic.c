/*
 * Open Multi-Processor Interrupt Controller driver
 *
 * Copyright (C) 2014 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/delay.h>

#include "irqchip.h"

#define OMPIC_IPI_BASE			0x0
#define OMPIC_IPI_CTRL(cpu)		(OMPIC_IPI_BASE + 0x0 + (cpu)*8)
#define OMPIC_IPI_STAT(cpu)		(OMPIC_IPI_BASE + 0x4 + (cpu)*8)

#define OMPIC_IPI_CTRL_IRQ_ACK		(1 << 31)
#define OMPIC_IPI_CTRL_IRQ_GEN		(1 << 30)
#define OMPIC_IPI_CTRL_DST(cpu)		(((cpu) & 0x3fff) << 16)

#define OMPIC_IPI_STAT_IRQ_PENDING	(1 << 30)

#define OMPIC_IPI_DATA(x)		((x) & 0xffff)

DEFINE_RAW_SPINLOCK(ompic_ipi_lock);

static void __iomem *ompic_base;

static inline u32 ompic_readreg(void __iomem *base, loff_t offset)
{
	return ioread32be(base + offset);
}

static void ompic_writereg(void __iomem *base, loff_t offset, u32 data)
{
	iowrite32be(data, base + offset);
}

#ifdef CONFIG_SMP
void ompic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	unsigned int dst_cpu;
	unsigned int src_cpu = smp_processor_id();
	int retry = 10000;
	unsigned long flags;

	raw_spin_lock_irqsave(&ompic_ipi_lock, flags);
	for_each_cpu(dst_cpu, mask) {
		while (--retry &&
		       ompic_readreg(ompic_base, OMPIC_IPI_STAT(dst_cpu)) &
		       OMPIC_IPI_STAT_IRQ_PENDING) {
			udelay(1);
		}

		ompic_writereg(ompic_base, OMPIC_IPI_CTRL(src_cpu),
			       OMPIC_IPI_CTRL_IRQ_GEN |
			       OMPIC_IPI_CTRL_DST(dst_cpu) |
			       OMPIC_IPI_DATA(irq));

		if (!retry)
			pr_crit("OMPIC softirq timed out");
	}
	raw_spin_unlock_irqrestore(&ompic_ipi_lock, flags);
}
#endif

irqreturn_t ompic_ipi_handler(int irq, void *dev_id)
{
	u32 status;
	unsigned int cpu = smp_processor_id();

	status = ompic_readreg(ompic_base, OMPIC_IPI_STAT(cpu));
	ompic_writereg(ompic_base, OMPIC_IPI_CTRL(cpu), OMPIC_IPI_CTRL_IRQ_ACK);

	handle_IPI(OMPIC_IPI_DATA(status));

	return IRQ_HANDLED;
}

static struct irqaction ompi_ipi_irqaction = {
	.handler =      ompic_ipi_handler,
	.flags =        IRQF_PERCPU,
	.name =         "ompic_ipi",
};

#ifdef CONFIG_OF
int __init ompic_of_init(struct device_node *node, struct device_node *parent)
{
	int irq;

	if (WARN_ON(!node))
		return -ENODEV;

	ompic_base = of_iomap(node, 0);

	irq = irq_of_parse_and_map(node, 0);
	setup_irq(irq, &ompi_ipi_irqaction);

#ifdef CONFIG_SMP
	set_smp_cross_call(ompic_raise_softirq);
#endif

	return 0;
}
IRQCHIP_DECLARE(ompic, "ompic", ompic_of_init);
#endif
