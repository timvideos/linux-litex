/*
 * Copyright (C) 2014 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 *
 * Loosely based on arm64 and arc implementations
 * Copyright (C) 2013 ARM Ltd.
 * Copyright (C) 2004, 2007-2010, 2011-2012 Synopsys, Inc. (www.synopsys.com)
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/cpu.h>
#include <linux/sched.h>

volatile unsigned long secondary_release = -1;
struct thread_info *secondary_thread_info;

static phys_addr_t cpu_release_addr[NR_CPUS];
static DEFINE_SPINLOCK(boot_lock);

static int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	secondary_release = cpu;

	/*
	 * now the secondary core is starting up let it run its
         * calibrations, then wait for it to finish
         */
	spin_unlock(&boot_lock);

	return 0;
}

void __init smp_prepare_boot_cpu(void)
{
}

void __init smp_init_cpus(void)
{
	int i;

	for (i = 0; i < NR_CPUS; i++)
		set_cpu_possible(i, true);
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);
}

void __init smp_cpus_done(unsigned int max_cpus)
{

}

static DECLARE_COMPLETION(cpu_running);

int __cpu_up(unsigned int cpu, struct task_struct *idle)
{
	int ret;

	secondary_thread_info = task_thread_info(idle);

	ret = boot_secondary(cpu, idle);
	if (ret == 0) {
		wait_for_completion_timeout(&cpu_running,
					    msecs_to_jiffies(1000));
		if (!cpu_online(cpu))
			ret = -EIO;
	}

	return ret;
}

asmlinkage void secondary_start_kernel(void)
{
	struct mm_struct *mm = &init_mm;
	unsigned int cpu = smp_processor_id();
	/*
	 * All kernel threads share the same mm context; grab a
	 * reference and switch to it.
	 */
	atomic_inc(&mm->mm_count);
	current->active_mm = mm;
	cpumask_set_cpu(cpu, mm_cpumask(mm));

	printk("CPU%u: Booted secondary processor\n", cpu);

	notify_cpu_starting(cpu);

	/*
	 * OK, now it's safe to let the boot CPU continue
	 */
	set_cpu_online(cpu, true);
	complete(&cpu_running);

	local_irq_enable();

	/*
	 * OK, it's off to the idle thread for us
	 */
	cpu_startup_entry(CPUHP_ONLINE);
}

void smp_send_reschedule(int cpu)
{
	BUG(); /* SJK TODO */
}

void smp_send_stop(void)
{
	BUG(); /* SJK TODO */
}

void arch_send_call_function_single_ipi(int cpu)
{
	BUG(); /* SJK TODO */
}

void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
	BUG(); /* SJK TODO */
}
