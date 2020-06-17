// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6328 PCIe Controller Driver
 *
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (C) 2015 Jonas Gorski <jonas.gorski@gmail.com>
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "../pci.h"

#define SERDES_PCIE_EN			BIT(0)
#define SERDES_PCIE_EXD_EN		BIT(15)

#define PCIE_BUS_BRIDGE			0
#define PCIE_BUS_DEVICE			1

#define PCIE_CONFIG2_REG		0x408
#define CONFIG2_BAR1_SIZE_EN		1
#define CONFIG2_BAR1_SIZE_MASK		0xf

#define PCIE_IDVAL3_REG			0x43c
#define IDVAL3_CLASS_CODE_MASK		0xffffff
#define IDVAL3_SUBCLASS_SHIFT		8
#define IDVAL3_CLASS_SHIFT		16

#define PCIE_DLSTATUS_REG		0x1048
#define DLSTATUS_PHYLINKUP		BIT(13)

#define PCIE_BRIDGE_OPT1_REG		0x2820
#define OPT1_RD_BE_OPT_EN		BIT(7)
#define OPT1_RD_REPLY_BE_FIX_EN		BIT(9)
#define OPT1_PCIE_BRIDGE_HOLE_DET_EN	BIT(11)
#define OPT1_L1_INT_STATUS_MASK_POL	BIT(12)

#define PCIE_BRIDGE_OPT2_REG		0x2824
#define OPT2_UBUS_UR_DECODE_DIS		BIT(2)
#define OPT2_TX_CREDIT_CHK_EN		BIT(4)
#define OPT2_CFG_TYPE1_BD_SEL		BIT(7)
#define OPT2_CFG_TYPE1_BUS_NO_SHIFT	16
#define OPT2_CFG_TYPE1_BUS_NO_MASK	(0xff << OPT2_CFG_TYPE1_BUS_NO_SHIFT)

#define PCIE_BRIDGE_BAR0_BASEMASK_REG	0x2828
#define BASEMASK_REMAP_EN		BIT(0)
#define BASEMASK_SWAP_EN		BIT(1)
#define BASEMASK_MASK_SHIFT		4
#define BASEMASK_MASK_MASK		(0xfff << BASEMASK_MASK_SHIFT)
#define BASEMASK_BASE_SHIFT		20
#define BASEMASK_BASE_MASK		(0xfff << BASEMASK_BASE_SHIFT)

#define PCIE_BRIDGE_BAR0_REBASE_ADDR_REG 0x282c
#define REBASE_ADDR_BASE_SHIFT		20
#define REBASE_ADDR_BASE_MASK		(0xfff << REBASE_ADDR_BASE_SHIFT)

#define PCIE_BRIDGE_RC_INT_MASK_REG	0x2854
#define PCIE_RC_INT_A			BIT(0)
#define PCIE_RC_INT_B			BIT(1)
#define PCIE_RC_INT_C			BIT(2)
#define PCIE_RC_INT_D			BIT(3)

#define PCIE_DEVICE_OFFSET		0x8000

struct bcm6328_pcie {
	void __iomem *base;
	struct regmap *serdes;
	struct clk *clk;
	struct reset_control *reset;
	struct reset_control *reset_ext;
	struct reset_control *reset_core;
	struct reset_control *reset_hard;
};

static struct bcm6328_pcie bcm6328_pcie;

/*
 * swizzle 32bits data to return only the needed part
 */
static int postprocess_read(u32 data, int where, unsigned int size)
{
	u32 ret = 0;

	switch (size) {
	case 1:
		ret = (data >> ((where & 3) << 3)) & 0xff;
		break;
	case 2:
		ret = (data >> ((where & 3) << 3)) & 0xffff;
		break;
	case 4:
		ret = data;
		break;
	}

	return ret;
}

static int preprocess_write(u32 orig_data, u32 val, int where,
			    unsigned int size)
{
	u32 ret = 0;

	switch (size) {
	case 1:
		ret = (orig_data & ~(0xff << ((where & 3) << 3))) |
		      (val << ((where & 3) << 3));
		break;
	case 2:
		ret = (orig_data & ~(0xffff << ((where & 3) << 3))) |
		      (val << ((where & 3) << 3));
		break;
	case 4:
		ret = val;
		break;
	}

	return ret;
}

static int bcm6328_pcie_can_access(struct pci_bus *bus, int devfn)
{
	struct bcm6328_pcie *priv = &bcm6328_pcie;

	switch (bus->number) {
	case PCIE_BUS_BRIDGE:
		return PCI_SLOT(devfn) == 0;
	case PCIE_BUS_DEVICE:
		if (PCI_SLOT(devfn) == 0)
			return __raw_readl(priv->base + PCIE_DLSTATUS_REG)
			       & DLSTATUS_PHYLINKUP;
		/* fall through */
	default:
		return false;
	}
}

static int bcm6328_pcie_read(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 *val)
{
	struct bcm6328_pcie *priv = &bcm6328_pcie;
	u32 data;
	u32 reg = where & ~3;

	if (!bcm6328_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(priv->base + reg);
	*val = postprocess_read(data, where, size);

	return PCIBIOS_SUCCESSFUL;
}

static int bcm6328_pcie_write(struct pci_bus *bus, unsigned int devfn,
			      int where, int size, u32 val)
{
	struct bcm6328_pcie *priv = &bcm6328_pcie;
	u32 data;
	u32 reg = where & ~3;

	if (!bcm6328_pcie_can_access(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == PCIE_BUS_DEVICE)
		reg += PCIE_DEVICE_OFFSET;

	data = __raw_readl(priv->base + reg);
	data = preprocess_write(data, val, where, size);
	__raw_writel(data, priv->base + reg);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops bcm6328_pcie_ops = {
	.read = bcm6328_pcie_read,
	.write = bcm6328_pcie_write,
};

static struct resource bcm6328_pcie_io_resource;
static struct resource bcm6328_pcie_mem_resource;
static struct resource bcm6328_pcie_busn_resource;

static struct pci_controller bcm6328_pcie_controller = {
	.pci_ops = &bcm6328_pcie_ops,
	.io_resource = &bcm6328_pcie_io_resource,
	.mem_resource = &bcm6328_pcie_mem_resource,
	.busn_resource = &bcm6328_pcie_busn_resource,
};

static void bcm6328_pcie_reset(struct bcm6328_pcie *priv)
{
	regmap_write_bits(priv->serdes, 0,
			  SERDES_PCIE_EN | SERDES_PCIE_EXD_EN,
			  SERDES_PCIE_EN | SERDES_PCIE_EXD_EN);

	reset_control_assert(priv->reset);
	reset_control_assert(priv->reset_core);
	reset_control_assert(priv->reset_ext);
	if (priv->reset_hard) {
		reset_control_assert(priv->reset_hard);
		mdelay(10);
		reset_control_deassert(priv->reset_hard);
	}
	mdelay(10);

	reset_control_deassert(priv->reset_core);
	reset_control_deassert(priv->reset);
	mdelay(10);

	reset_control_deassert(priv->reset_ext);
	mdelay(200);
}

static void bcm6328_pcie_setup(struct bcm6328_pcie *priv)
{
	u32 val;

	val = __raw_readl(priv->base + PCIE_BRIDGE_OPT1_REG);
	val |= OPT1_RD_BE_OPT_EN;
	val |= OPT1_RD_REPLY_BE_FIX_EN;
	val |= OPT1_PCIE_BRIDGE_HOLE_DET_EN;
	val |= OPT1_L1_INT_STATUS_MASK_POL;
	__raw_writel(val, priv->base + PCIE_BRIDGE_OPT1_REG);

	val = __raw_readl(priv->base + PCIE_BRIDGE_RC_INT_MASK_REG);
	val |= PCIE_RC_INT_A;
	val |= PCIE_RC_INT_B;
	val |= PCIE_RC_INT_C;
	val |= PCIE_RC_INT_D;
	__raw_writel(val, priv->base + PCIE_BRIDGE_RC_INT_MASK_REG);

	val = __raw_readl(priv->base + PCIE_BRIDGE_OPT2_REG);
	/* enable credit checking and error checking */
	val |= OPT2_TX_CREDIT_CHK_EN;
	val |= OPT2_UBUS_UR_DECODE_DIS;
	/* set device bus/func for the pcie device */
	val |= (PCIE_BUS_DEVICE << OPT2_CFG_TYPE1_BUS_NO_SHIFT);
	val |= OPT2_CFG_TYPE1_BD_SEL;
	__raw_writel(val, priv->base + PCIE_BRIDGE_OPT2_REG);

	/* setup class code as bridge */
	val = __raw_readl(priv->base + PCIE_IDVAL3_REG);
	val &= ~IDVAL3_CLASS_CODE_MASK;
	val |= (PCI_CLASS_BRIDGE_PCI << IDVAL3_SUBCLASS_SHIFT);
	__raw_writel(val, priv->base + PCIE_IDVAL3_REG);

	/* disable bar1 size */
	val = __raw_readl(priv->base + PCIE_CONFIG2_REG);
	val &= ~CONFIG2_BAR1_SIZE_MASK;
	__raw_writel(val, priv->base + PCIE_CONFIG2_REG);

	/* set bar0 to little endian */
	val = (bcm6328_pcie_mem_resource.start >> 20)
	      << BASEMASK_BASE_SHIFT;
	val |= (bcm6328_pcie_mem_resource.end >> 20) << BASEMASK_MASK_SHIFT;
	val |= BASEMASK_REMAP_EN;
	__raw_writel(val, priv->base + PCIE_BRIDGE_BAR0_BASEMASK_REG);

	val = (bcm6328_pcie_mem_resource.start >> 20)
	      << REBASE_ADDR_BASE_SHIFT;
	__raw_writel(val, priv->base + PCIE_BRIDGE_BAR0_REBASE_ADDR_REG);
}

static int bcm6328_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm6328_pcie *priv = &bcm6328_pcie;
	int ret;

	of_pci_check_probe_only();

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->serdes = syscon_regmap_lookup_by_phandle(np, "brcm,serdes");
	if (IS_ERR(priv->serdes))
		return PTR_ERR(priv->serdes);

	priv->reset = devm_reset_control_get_exclusive(dev, "pcie");
	if (IS_ERR(priv->reset))
		return PTR_ERR(priv->reset);

	priv->reset_ext = devm_reset_control_get_exclusive(dev, "pcie-ext");
	if (IS_ERR(priv->reset_ext))
		return PTR_ERR(priv->reset_ext);

	priv->reset_core = devm_reset_control_get_exclusive(dev, "pcie-core");
	if (IS_ERR(priv->reset_core))
		return PTR_ERR(priv->reset_core);

	priv->reset_hard = devm_reset_control_get_optional_exclusive(dev,
		"pcie-hard");
	if (IS_ERR(priv->reset_hard))
		return PTR_ERR(priv->reset_hard);

	priv->clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "could not enable clock: %d\n", ret);
		return ret;
	}

	pci_load_of_ranges(&bcm6328_pcie_controller, np);
	if (!bcm6328_pcie_mem_resource.start)
		return -EINVAL;

	of_pci_parse_bus_range(np, &bcm6328_pcie_busn_resource);

	bcm6328_pcie_reset(priv);
	bcm6328_pcie_setup(priv);

	register_pci_controller(&bcm6328_pcie_controller);

	return 0;
}

static const struct of_device_id bcm6328_pcie_of_match[] = {
	{ .compatible = "brcm,bcm6328-pcie", },
	{ .compatible = "brcm,bcm6362-pcie", },
	{ .compatible = "brcm,bcm63268-pcie", },
	{ /* sentinel */ },
};

static struct platform_driver bcm6328_pcie_driver = {
	.driver	= {
		.name = "bcm6328-pcie",
		.of_match_table = bcm6328_pcie_of_match,
	},
	.probe = bcm6328_pcie_probe,
};
builtin_platform_driver(bcm6328_pcie_driver);
