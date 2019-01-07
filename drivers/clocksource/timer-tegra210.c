// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/percpu.h>
#include <linux/syscore_ops.h>

static u32 tegra210_timer_freq;
static void __iomem *tegra210_timer_reg_base;
static u32 usec_config;

#define TIMER_PTV		0x0
#define TIMER_PTV_EN		BIT(31)
#define TIMER_PTV_PER		BIT(30)
#define TIMER_PCR		0x4
#define TIMER_PCR_INTR_CLR	BIT(30)
#define TIMERUS_CNTR_1US	0x10
#define TIMERUS_USEC_CFG	0x14

#define TIMER10_OFFSET		0x90

#define TIMER_FOR_CPU(cpu) (TIMER10_OFFSET + (cpu) * 8)

struct tegra210_clockevent {
	struct clock_event_device evt;
	char name[20];
	void __iomem *reg_base;
};
#define to_tegra_cevt(p) (container_of(p, struct tegra210_clockevent, evt))

static struct tegra210_clockevent __percpu *tegra210_evt;

static int tegra210_timer_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	struct tegra210_clockevent *tevt;

	tevt = to_tegra_cevt(evt);
	writel(TIMER_PTV_EN |
	       ((cycles > 1) ? (cycles - 1) : 0), /* n+1 scheme */
	       tevt->reg_base + TIMER_PTV);

	return 0;
}

static inline void timer_shutdown(struct tegra210_clockevent *tevt)
{
	writel(0, tevt->reg_base + TIMER_PTV);
}

static int tegra210_timer_shutdown(struct clock_event_device *evt)
{
	struct tegra210_clockevent *tevt;

	tevt = to_tegra_cevt(evt);
	timer_shutdown(tevt);

	return 0;
}

static int tegra210_timer_set_periodic(struct clock_event_device *evt)
{
	struct tegra210_clockevent *tevt;

	tevt = to_tegra_cevt(evt);
	writel(TIMER_PTV_EN | TIMER_PTV_PER | ((tegra210_timer_freq / HZ) - 1),
	       tevt->reg_base + TIMER_PTV);

	return 0;
}

static irqreturn_t tegra210_timer_isr(int irq, void *dev_id)
{
	struct tegra210_clockevent *tevt;

	tevt = dev_id;
	writel(TIMER_PCR_INTR_CLR, tevt->reg_base + TIMER_PCR);
	tevt->evt.event_handler(&tevt->evt);

	return IRQ_HANDLED;
}

static int tegra210_timer_setup(unsigned int cpu)
{
	struct tegra210_clockevent *tevt = per_cpu_ptr(tegra210_evt, cpu);

	irq_force_affinity(tevt->evt.irq, cpumask_of(cpu));
	enable_irq(tevt->evt.irq);

	clockevents_config_and_register(&tevt->evt, tegra210_timer_freq,
					1, /* min */
					0x1fffffff); /* 29 bits */

	return 0;
}

static int tegra210_timer_stop(unsigned int cpu)
{
	struct tegra210_clockevent *tevt = per_cpu_ptr(tegra210_evt, cpu);

	tevt->evt.set_state_shutdown(&tevt->evt);
	disable_irq_nosync(tevt->evt.irq);

	return 0;
}

static int tegra_timer_suspend(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		void __iomem *reg_base = tegra210_timer_reg_base +
					 TIMER_FOR_CPU(cpu);
		writel(TIMER_PCR_INTR_CLR, reg_base + TIMER_PCR);
	}

	return 0;
}

static void tegra_timer_resume(void)
{
	writel(usec_config, tegra210_timer_reg_base + TIMERUS_USEC_CFG);
}

static struct syscore_ops tegra_timer_syscore_ops = {
	.suspend = tegra_timer_suspend,
	.resume = tegra_timer_resume,
};

static int __init tegra210_timer_init(struct device_node *np)
{
	int cpu, ret;
	struct tegra210_clockevent *tevt;
	struct clk *clk;

	tegra210_evt = alloc_percpu(struct tegra210_clockevent);
	if (!tegra210_evt)
		return -ENOMEM;

	tegra210_timer_reg_base = of_iomap(np, 0);
	if (!tegra210_timer_reg_base)
		return -ENXIO;

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk))
		return -EINVAL;

	clk_prepare_enable(clk);
	tegra210_timer_freq = clk_get_rate(clk);

	for_each_possible_cpu(cpu) {
		tevt = per_cpu_ptr(tegra210_evt, cpu);
		tevt->reg_base = tegra210_timer_reg_base + TIMER_FOR_CPU(cpu);
		tevt->evt.irq = irq_of_parse_and_map(np, cpu);
		if (!tevt->evt.irq) {
			pr_err("%s: can't map IRQ for CPU%d\n",
			       __func__, cpu);
			return -EINVAL;
		}

		snprintf(tevt->name, ARRAY_SIZE(tevt->name),
			 "tegra210_timer%d", cpu);
		tevt->evt.name = tevt->name;
		tevt->evt.cpumask = cpumask_of(cpu);
		tevt->evt.set_next_event = tegra210_timer_set_next_event;
		tevt->evt.set_state_shutdown = tegra210_timer_shutdown;
		tevt->evt.set_state_periodic = tegra210_timer_set_periodic;
		tevt->evt.set_state_oneshot = tegra210_timer_shutdown;
		tevt->evt.tick_resume = tegra210_timer_shutdown;
		tevt->evt.features = CLOCK_EVT_FEAT_PERIODIC |
			CLOCK_EVT_FEAT_ONESHOT;
		tevt->evt.rating = 460;

		irq_set_status_flags(tevt->evt.irq, IRQ_NOAUTOEN);
		ret = request_irq(tevt->evt.irq, tegra210_timer_isr,
				  IRQF_TIMER | IRQF_NOBALANCING,
				  tevt->name, tevt);
		if (ret) {
			pr_err("%s: cannot setup irq %d for CPU%d\n",
				__func__, tevt->evt.irq, cpu);
			return -EINVAL;
		}
	}

	/*
	 * Configure microsecond timers to have 1MHz clock
	 * Config register is 0xqqww, where qq is "dividend", ww is "divisor"
	 * Uses n+1 scheme
	 */
	switch (tegra210_timer_freq) {
	case 12000000:
		usec_config = 0x000b; /* (11+1)/(0+1) */
		break;
	case 12800000:
		usec_config = 0x043f; /* (63+1)/(4+1) */
		break;
	case 13000000:
		usec_config = 0x000c; /* (12+1)/(0+1) */
		break;
	case 16800000:
		usec_config = 0x0453; /* (83+1)/(4+1) */
		break;
	case 19200000:
		usec_config = 0x045f; /* (95+1)/(4+1) */
		break;
	case 26000000:
		usec_config = 0x0019; /* (25+1)/(0+1) */
		break;
	case 38400000:
		usec_config = 0x04bf; /* (191+1)/(4+1) */
		break;
	case 48000000:
		usec_config = 0x002f; /* (47+1)/(0+1) */
		break;
	default:
		return -EINVAL;
	}

	writel(usec_config, tegra210_timer_reg_base + TIMERUS_USEC_CFG);

	cpuhp_setup_state(CPUHP_AP_TEGRA_TIMER_STARTING,
			  "AP_TEGRA_TIMER_STARTING", tegra210_timer_setup,
			  tegra210_timer_stop);

	register_syscore_ops(&tegra_timer_syscore_ops);

	return 0;
}

TIMER_OF_DECLARE(tegra210_timer, "nvidia,tegra210-timer", tegra210_timer_init);
