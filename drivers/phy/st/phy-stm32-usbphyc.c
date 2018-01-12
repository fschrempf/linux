/*
 * STMicroelectronics STM32 USB PHY Controller driver
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com> for STMicroelectronics.
 *
 * License terms: GPL V2.0.
 *
 * stm32-usbphyc driver is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * stm32-usbphyc driver is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * stm32-usbphyc driver. If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>

#define STM32_USBPHYC_PLL	0x0
#define STM32_USBPHYC_MISC	0x8
#define STM32_USBPHYC_VERSION	0x3F4

/* STM32_USBPHYC_PLL bit fields */
#define PLLNDIV			GENMASK(6, 0)
#define PLLFRACIN		GENMASK(25, 10)
#define PLLEN			BIT(26)
#define PLLSTRB			BIT(27)
#define PLLSTRBYP		BIT(28)
#define PLLFRACCTL		BIT(29)
#define PLLDITHEN0		BIT(30)
#define PLLDITHEN1		BIT(31)

/* STM32_USBPHYC_MISC bit fields */
#define SWITHOST		BIT(0)

/* STM32_USBPHYC_VERSION bit fields */
#define MINREV			GENMASK(3, 0)
#define MAJREV			GENMASK(7, 4)

#define MAX_PHYS		2

#define PLL_LOCK_TIME_US	100
#define PLL_PWR_DOWN_TIME_US	5
#define PLL_FVCO		2880 /* in MHz */
#define PLL_INFF_MIN_RATE	19200000 /* in Hz */
#define PLL_INFF_MAX_RATE	38400000 /* in Hz */

struct pll_params {
	u8 ndiv;
	u16 frac;
};

struct stm32_usbphyc {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	struct regulator *vdd;
	struct stm32_usbphyc_phy {
		struct phy *phy;
		int index;
		bool init;
	} phys[MAX_PHYS];
};

static inline struct stm32_usbphyc *
to_usbphyc(struct stm32_usbphyc_phy *usbphyc_phy)
{
	return container_of(usbphyc_phy, struct stm32_usbphyc,
			    phys[usbphyc_phy->index]);
}

static inline void stm32_usbphyc_set_bits(void __iomem *reg, u32 bits)
{
	writel_relaxed(readl_relaxed(reg) | bits, reg);
}

static inline void stm32_usbphyc_clr_bits(void __iomem *reg, u32 bits)
{
	writel_relaxed(readl_relaxed(reg) & ~bits, reg);
}

void stm32_usbphyc_get_pll_params(u32 clk_rate, struct pll_params *pll_params)
{
	unsigned long long fvco, ndiv, frac;

	/*    _
	 *   | FVCO = INFF*2*(NDIV + FRACT/2^16) when DITHER_DISABLE[1] = 1
	 *   | FVCO = 2880MHz
	 *  <
	 *   | NDIV = integer part of input bits to set the LDF
	 *   |_FRACT = fractional part of input bits to set the LDF
	 *  =>	PLLNDIV = integer part of (FVCO / (INFF*2))
	 *  =>	PLLFRACIN = fractional part of(FVCO / INFF*2) * 2^16
	 * <=>  PLLFRACIN = ((FVCO / (INFF*2)) - PLLNDIV) * 2^16
	 */
	fvco = (unsigned long long)PLL_FVCO * 1000000; /* In Hz */

	ndiv = fvco;
	do_div(ndiv, (clk_rate * 2));
	pll_params->ndiv = (u8)ndiv;

	frac = fvco * (1 << 16);
	do_div(frac, (clk_rate * 2));
	frac = frac - (ndiv * (1 << 16));
	pll_params->frac = (u16)frac;
}

static int stm32_usbphyc_pll_init(struct stm32_usbphyc *usbphyc)
{
	struct pll_params pll_params;
	u32 clk_rate = clk_get_rate(usbphyc->clk);
	u32 ndiv, frac;
	u32 usbphyc_pll;

	if ((clk_rate < PLL_INFF_MIN_RATE) || (clk_rate > PLL_INFF_MAX_RATE)) {
		dev_err(usbphyc->dev, "input clk freq (%dHz) out of range\n",
			clk_rate);
		return -EINVAL;
	}

	stm32_usbphyc_get_pll_params(clk_rate, &pll_params);
	ndiv = FIELD_PREP(PLLNDIV, pll_params.ndiv);
	frac = FIELD_PREP(PLLFRACIN, pll_params.frac);

	usbphyc_pll = PLLDITHEN1 | PLLDITHEN0 | PLLSTRBYP | ndiv;

	if (pll_params.frac)
		usbphyc_pll |= PLLFRACCTL | frac;

	writel_relaxed(usbphyc_pll, usbphyc->base + STM32_USBPHYC_PLL);

	dev_dbg(usbphyc->dev, "input clk freq=%dHz, ndiv=%lu, frac=%lu\n",
		clk_rate, FIELD_GET(PLLNDIV, usbphyc_pll),
		FIELD_GET(PLLFRACIN, usbphyc_pll));

	return 0;
}

static bool stm32_usbphyc_is_init(struct stm32_usbphyc *usbphyc)
{
	int i;

	for (i = 0; i < MAX_PHYS; i++) {
		if (usbphyc->phys[i].init)
			return true;
	}

	return false;
}

static int stm32_usbphyc_phy_init(struct phy *phy)
{
	struct stm32_usbphyc_phy *usbphyc_phy = phy_get_drvdata(phy);
	struct stm32_usbphyc *usbphyc = to_usbphyc(usbphyc_phy);
	void __iomem *pll_reg = usbphyc->base + STM32_USBPHYC_PLL;
	bool pllen = !!(readl_relaxed(pll_reg) & PLLEN);
	int ret;

	/* Check if one phy port has already configured the pll */
	if (pllen && stm32_usbphyc_is_init(usbphyc))
		goto initialized;

	if (pllen) {
		stm32_usbphyc_clr_bits(pll_reg, PLLEN);
		udelay(PLL_PWR_DOWN_TIME_US);
	}

	ret = stm32_usbphyc_pll_init(usbphyc);
	if (ret)
		return ret;

	stm32_usbphyc_set_bits(pll_reg, PLLEN);

	udelay(PLL_LOCK_TIME_US);

	if (!(readl_relaxed(pll_reg) & PLLEN)) {
		dev_err(usbphyc->dev, "PLLEN not set\n");
		return -EIO;
	}

initialized:
	usbphyc_phy->init = true;

	return 0;
}

static int stm32_usbphyc_phy_exit(struct phy *phy)
{
	struct stm32_usbphyc_phy *usbphyc_phy = phy_get_drvdata(phy);
	struct stm32_usbphyc *usbphyc = to_usbphyc(usbphyc_phy);
	void __iomem *pll_reg = usbphyc->base + STM32_USBPHYC_PLL;

	usbphyc_phy->init = false;

	/* Check if other phy port requires pllen */
	if (stm32_usbphyc_is_init(usbphyc))
		return 0;

	stm32_usbphyc_clr_bits(pll_reg, PLLEN);

	udelay(PLL_PWR_DOWN_TIME_US);

	if (!!(readl_relaxed(pll_reg) & PLLEN)) {
		dev_err(usbphyc->dev, "PLL not reset\n");
		return -EIO;
	}

	return 0;
}

static int stm32_usbphyc_phy_power_on(struct phy *phy)
{
	struct stm32_usbphyc_phy *usbphyc_phy = phy_get_drvdata(phy);
	struct stm32_usbphyc *usbphyc = to_usbphyc(usbphyc_phy);

	return regulator_enable(usbphyc->vdd);
}

static int stm32_usbphyc_phy_power_off(struct phy *phy)
{
	struct stm32_usbphyc_phy *usbphyc_phy = phy_get_drvdata(phy);
	struct stm32_usbphyc *usbphyc = to_usbphyc(usbphyc_phy);

	return regulator_disable(usbphyc->vdd);
}

static const struct phy_ops stm32_usbphyc_phy_ops = {
	.init = stm32_usbphyc_phy_init,
	.exit = stm32_usbphyc_phy_exit,
	.power_on = stm32_usbphyc_phy_power_on,
	.power_off = stm32_usbphyc_phy_power_off,
	.owner = THIS_MODULE,
};

static struct phy *stm32_usbphyc_of_xlate(struct device *dev,
					  struct of_phandle_args *args)
{
	struct stm32_usbphyc *usbphyc = dev_get_drvdata(dev);

	if (args->args[0] >= MAX_PHYS)
		return ERR_PTR(-ENODEV);

	return usbphyc->phys[args->args[0]].phy;
}

static int stm32_usbphyc_probe(struct platform_device *pdev)
{
	struct stm32_usbphyc *usbphyc;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct phy_provider *phy_provider;
	bool switch_to_host;
	u32 version;
	int i, ret;

	usbphyc = devm_kzalloc(dev, sizeof(*usbphyc), GFP_KERNEL);
	if (!usbphyc)
		return -ENOMEM;
	usbphyc->dev = dev;
	dev_set_drvdata(dev, usbphyc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	usbphyc->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(usbphyc->base))
		return PTR_ERR(usbphyc->base);

	usbphyc->clk = devm_clk_get(dev, 0);
	if (IS_ERR(usbphyc->clk)) {
		ret = PTR_ERR(usbphyc->clk);
		dev_err(dev, "clk get failed: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(usbphyc->clk);
	if (ret) {
		dev_err(dev, "clk enable failed: %d\n", ret);
		return ret;
	}

	usbphyc->rst = devm_reset_control_get(dev, 0);
	if (!IS_ERR(usbphyc->rst)) {
		reset_control_assert(usbphyc->rst);
		udelay(2);
		reset_control_deassert(usbphyc->rst);
	}

	usbphyc->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(usbphyc->vdd)) {
		ret = PTR_ERR(usbphyc->vdd);
		if (ret == -EPROBE_DEFER)
			goto clk_disable;

		dev_err(dev, "vdd_usb regulator get failed: %d\n", ret);
		goto clk_disable;
	}

	for (i = 0; i < MAX_PHYS; i++) {
		struct stm32_usbphyc_phy *usbphyc_phy = usbphyc->phys + i;

		usbphyc_phy->phy = devm_phy_create(dev, NULL,
						   &stm32_usbphyc_phy_ops);
		if (IS_ERR(usbphyc_phy->phy)) {
			ret = PTR_ERR(usbphyc_phy->phy);
			dev_err(dev, "failed to create phy%d: %d\n", i, ret);
			goto clk_disable;
		}

		usbphyc_phy->index = i;
		usbphyc_phy->init = false;
		phy_set_bus_width(usbphyc_phy->phy, 8);
		phy_set_drvdata(usbphyc_phy->phy, usbphyc_phy);
	}

	phy_provider = devm_of_phy_provider_register(dev,
						     stm32_usbphyc_of_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		dev_err(dev, "failed to register phy provider: %d\n", ret);
		goto clk_disable;
	}

	/* Check if second port has to be used for host controller */
	switch_to_host = of_property_read_bool(np, "st,port2-switch-to-host");
	if (switch_to_host)
		stm32_usbphyc_set_bits(usbphyc->base + STM32_USBPHYC_MISC,
				       SWITHOST);

	version = readl_relaxed(usbphyc->base + STM32_USBPHYC_VERSION);
	dev_info(dev, "registered rev:%lu.%lu\n",
		 FIELD_GET(MAJREV, version), FIELD_GET(MINREV, version));

	return 0;

clk_disable:
	clk_disable_unprepare(usbphyc->clk);

	return ret;
}

static int stm32_usbphyc_remove(struct platform_device *pdev)
{
	struct stm32_usbphyc *usbphyc = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(usbphyc->clk);

	return 0;
}

static const struct of_device_id stm32_usbphyc_of_match[] = {
	{ .compatible = "st,stm32mp1-usbphyc", },
	{ },
};
MODULE_DEVICE_TABLE(of, stm32_usbphyc_of_match);

static struct platform_driver stm32_usbphyc_driver = {
	.probe = stm32_usbphyc_probe,
	.remove = stm32_usbphyc_remove,
	.driver = {
		.of_match_table = stm32_usbphyc_of_match,
		.name = "stm32-usbphyc",
	}
};
module_platform_driver(stm32_usbphyc_driver);

MODULE_DESCRIPTION("STMicroelectronics STM32 USBPHYC driver");
MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@st.com>");
MODULE_LICENSE("GPL v2");
