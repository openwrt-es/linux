// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCM6345 style external interrupt controller driver
 *
 * Copyright (C) 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (C) 2014 Jonas Gorski <jonas.gorski@gmail.com>
 */

#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define MAX_IRQS		4

#define EXTIRQ_CFG_SENSE	0
#define EXTIRQ_CFG_STAT		1
#define EXTIRQ_CFG_CLEAR	2
#define EXTIRQ_CFG_MASK		3
#define EXTIRQ_CFG_BOTHEDGE	4
#define EXTIRQ_CFG_LEVELSENSE	5

struct intc_data {
	struct irq_chip chip;
	struct irq_domain *domain;
	raw_spinlock_t lock;

	int parent_irq[MAX_IRQS];
	void __iomem *reg;
	int shift;
	unsigned int toggle_clear_on_ack:1;
};

static void bcm6345_ext_intc_irq_handle(struct irq_desc *desc)
{
	struct intc_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int irq = irq_desc_get_irq(desc);
	unsigned int idx;

	chained_irq_enter(chip, desc);

	for (idx = 0; idx < MAX_IRQS; idx++) {
		if (data->parent_irq[idx] != irq)
			continue;

		generic_handle_irq(irq_find_mapping(data->domain, idx));
	}

	chained_irq_exit(chip, desc);
}

static void bcm6345_ext_intc_irq_ack(struct irq_data *data)
{
	struct intc_data *priv = data->domain->host_data;
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	u32 reg;

	raw_spin_lock(&priv->lock);
	reg = __raw_readl(priv->reg);
	__raw_writel(reg | (1 << (hwirq + EXTIRQ_CFG_CLEAR * priv->shift)),
		     priv->reg);
	if (priv->toggle_clear_on_ack)
		__raw_writel(reg, priv->reg);
	raw_spin_unlock(&priv->lock);
}

static void bcm6345_ext_intc_irq_mask(struct irq_data *data)
{
	struct intc_data *priv = data->domain->host_data;
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	u32 reg;

	raw_spin_lock(&priv->lock);
	reg = __raw_readl(priv->reg);
	reg &= ~(1 << (hwirq + EXTIRQ_CFG_MASK * priv->shift));
	__raw_writel(reg, priv->reg);
	raw_spin_unlock(&priv->lock);
}

static void bcm6345_ext_intc_irq_unmask(struct irq_data *data)
{
	struct intc_data *priv = data->domain->host_data;
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	u32 reg;

	raw_spin_lock(&priv->lock);
	reg = __raw_readl(priv->reg);
	reg |= 1 << (hwirq + EXTIRQ_CFG_MASK * priv->shift);
	__raw_writel(reg, priv->reg);
	raw_spin_unlock(&priv->lock);
}

static int bcm6345_ext_intc_set_type(struct irq_data *data,
				     unsigned int flow_type)
{
	struct intc_data *priv = data->domain->host_data;
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	bool levelsense = 0, sense = 0, bothedge = 0;
	u32 reg;

	flow_type &= IRQ_TYPE_SENSE_MASK;

	if (flow_type == IRQ_TYPE_NONE)
		flow_type = IRQ_TYPE_LEVEL_LOW;

	switch (flow_type) {
	case IRQ_TYPE_EDGE_BOTH:
		bothedge = 1;
		break;

	case IRQ_TYPE_EDGE_RISING:
		sense = 1;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		levelsense = 1;
		sense = 1;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		levelsense = 1;
		break;

	default:
		pr_err("bogus flow type combination given!\n");
		return -EINVAL;
	}

	raw_spin_lock(&priv->lock);
	reg = __raw_readl(priv->reg);

	if (levelsense)
		reg |= 1 << (hwirq + EXTIRQ_CFG_LEVELSENSE * priv->shift);
	else
		reg &= ~(1 << (hwirq + EXTIRQ_CFG_LEVELSENSE * priv->shift));
	if (sense)
		reg |= 1 << (hwirq + EXTIRQ_CFG_SENSE * priv->shift);
	else
		reg &= ~(1 << (hwirq + EXTIRQ_CFG_SENSE * priv->shift));
	if (bothedge)
		reg |= 1 << (hwirq + EXTIRQ_CFG_BOTHEDGE * priv->shift);
	else
		reg &= ~(1 << (hwirq + EXTIRQ_CFG_BOTHEDGE * priv->shift));

	__raw_writel(reg, priv->reg);
	raw_spin_unlock(&priv->lock);

	irqd_set_trigger_type(data, flow_type);
	if (flow_type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		irq_set_handler_locked(data, handle_level_irq);
	else
		irq_set_handler_locked(data, handle_edge_irq);

	return 0;
}

static int bcm6345_ext_intc_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	struct intc_data *priv = d->host_data;

	irq_set_chip_and_handler(irq, &priv->chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops bcm6345_ext_domain_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = bcm6345_ext_intc_map,
};

static int __init bcm6345_ext_intc_init(struct device_node *node,
					int num_irqs, int *irqs,
					void __iomem *reg, int shift,
					bool toggle_clear_on_ack)
{
	struct intc_data *data;
	unsigned int i;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	raw_spin_lock_init(&data->lock);

	for (i = 0; i < num_irqs; i++) {
		data->parent_irq[i] = irqs[i];

		irq_set_handler_data(irqs[i], data);
		irq_set_chained_handler(irqs[i], bcm6345_ext_intc_irq_handle);
	}

	data->reg = reg;
	data->shift = shift;
	data->toggle_clear_on_ack = toggle_clear_on_ack;

	data->chip.name = "bcm6345-ext-intc";
	data->chip.irq_ack = bcm6345_ext_intc_irq_ack;
	data->chip.irq_mask = bcm6345_ext_intc_irq_mask;
	data->chip.irq_unmask = bcm6345_ext_intc_irq_unmask;
	data->chip.irq_set_type = bcm6345_ext_intc_set_type;

	data->domain = irq_domain_add_simple(node, num_irqs, 0,
					     &bcm6345_ext_domain_ops, data);
	if (!data->domain) {
		kfree(data);
		return -ENOMEM;
	}

	return 0;
}

static int __init bcm6345_ext_intc_of_init(struct device_node *node,
					   struct device_node *parent)
{
	int num_irqs, ret = -EINVAL;
	unsigned i;
	void __iomem *base;
	int irqs[MAX_IRQS] = { 0 };
	u32 shift;
	bool toggle_clear_on_ack = false;

	num_irqs = of_irq_count(node);

	if (!num_irqs || num_irqs > MAX_IRQS)
		return -EINVAL;

	if (of_property_read_u32(node, "brcm,field-width", &shift))
		shift = 4;

	/* On BCM6318 setting CLEAR seems to continuously mask interrupts */
	if (of_device_is_compatible(node, "brcm,bcm6318-ext-intc"))
		toggle_clear_on_ack = true;

	for (i = 0; i < num_irqs; i++) {
		irqs[i] = irq_of_parse_and_map(node, i);
		if (!irqs[i])
			return -ENOMEM;
	}

	base = of_iomap(node, 0);
	if (!base)
		return -ENXIO;

	ret = bcm6345_ext_intc_init(node, num_irqs, irqs, base, shift,
				    toggle_clear_on_ack);
	if (!ret)
		return 0;

	iounmap(base);

	for (i = 0; i < num_irqs; i++)
		irq_dispose_mapping(irqs[i]);

	return ret;
}

IRQCHIP_DECLARE(bcm6318_ext_intc, "brcm,bcm6318-ext-intc",
		bcm6345_ext_intc_of_init);
IRQCHIP_DECLARE(bcm6345_ext_intc, "brcm,bcm6345-ext-intc",
		bcm6345_ext_intc_of_init);
