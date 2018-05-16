/*
 * dwmac-stm32.c - DWMAC Specific Glue layer for STM32 MCU
 *
 * Copyright (C) Alexandre Torgue 2015
 * Author:  Alexandre Torgue <alexandre.torgue@gmail.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/stmmac.h>
#include <linux/pm_wakeirq.h>
#include <linux/micrel_phy.h>

#include "stmmac_platform.h"

#define MII_PHY_SEL_MASK	GENMASK(23, 21)
#define MII_PHY_CLK_SEL_MASK	GENMASK(20, 16)


struct stm32_dwmac {
	struct clk *clk_ethmac_k;
	struct clk *clk_tx;
	struct clk *clk_rx;
	struct clk *clk_ck;
	struct clk *clk_ethstp;
	struct clk *syscfg_clk;
	int irq_pwr_wakeup;
	u32 mode_reg;		/* MAC glue-logic mode register */
	struct regmap *regmap;
	u32 speed;
	bool provide_phy_clk;
};

static int stm32_dwmac_init(struct plat_stmmacenet_data *plat_dat)
{
	struct stm32_dwmac *dwmac = plat_dat->bsp_priv;
	u32 reg = dwmac->mode_reg;
	u32 val;
	u32 val2;
	int ret;

	if (plat_dat->interface == PHY_INTERFACE_MODE_MII) {
		pr_debug("init for MII\n");
		val = 0;
		val2 = 0x10;  // SYSCFG->PMCR (bit (20): ETH_SELMII)
	} else if (plat_dat->interface == PHY_INTERFACE_MODE_RMII) {
		pr_debug("init for RMII\n");
		val = 4;
		if (dwmac->provide_phy_clk){
			val2 = 2; /* enable internal RMII clock (ETH_REF_CLK_SEL) */
			pr_debug("ETH_REF_CLK_SEL enabled\n");
		}
		else {
			val2 = 0;
		}
	} else if (plat_dat->interface == PHY_INTERFACE_MODE_GMII) {
		pr_debug("init for GMII\n");
		val = 0;
		val2 = 1;
	} else if (plat_dat->interface == PHY_INTERFACE_MODE_RGMII) {
		pr_debug("init for RGMII\n");
		val = 1;
		val2 = 0;
	} else {
		pr_err("NO interface defined!\n");
		return -EINVAL;
	}

	ret = regmap_update_bits(dwmac->regmap, reg,
				 MII_PHY_SEL_MASK, (val << 21));
	if (ret)
		return ret;

	ret = regmap_update_bits(dwmac->regmap, reg,
				 MII_PHY_CLK_SEL_MASK, (val2 << 16));
	if (ret)
		return ret;

	ret = clk_prepare_enable(dwmac->clk_ethmac_k);
	if (ret)
		return ret;

	ret = clk_prepare_enable(dwmac->clk_tx);
	if (ret) {
		clk_disable_unprepare(dwmac->clk_ethmac_k);
		return ret;
	}

	ret = clk_prepare_enable(dwmac->clk_rx);
	if (ret) {
		clk_disable_unprepare(dwmac->clk_tx);
		clk_disable_unprepare(dwmac->clk_ethmac_k);
		return ret;
	}

	ret = clk_prepare_enable(dwmac->clk_ck);
	if (ret) {
		clk_disable_unprepare(dwmac->clk_rx);
		clk_disable_unprepare(dwmac->clk_tx);
		clk_disable_unprepare(dwmac->clk_ethmac_k);
		return ret;
	}

	ret = clk_prepare_enable(dwmac->syscfg_clk);
	if (ret) {
		clk_disable_unprepare(dwmac->clk_ck);
		clk_disable_unprepare(dwmac->clk_rx);
		clk_disable_unprepare(dwmac->clk_tx);
		clk_disable_unprepare(dwmac->clk_ethmac_k);
		return ret;
	}

	ret = clk_prepare_enable(dwmac->clk_ethstp);
	if (ret) {
		clk_disable_unprepare(dwmac->syscfg_clk);
		clk_disable_unprepare(dwmac->clk_ck);
		clk_disable_unprepare(dwmac->clk_rx);
		clk_disable_unprepare(dwmac->clk_tx);
		clk_disable_unprepare(dwmac->clk_ethmac_k);
	}

	return ret;
}

static void stm32_dwmac_clk_disable(struct stm32_dwmac *dwmac)
{
	clk_disable_unprepare(dwmac->clk_ethmac_k);
	clk_disable_unprepare(dwmac->clk_tx);
//	clk_disable_unprepare(dwmac->clk_rx);
	clk_disable_unprepare(dwmac->clk_ck);
	clk_disable_unprepare(dwmac->clk_ethstp);
	clk_disable_unprepare(dwmac->syscfg_clk);
}

static int stm32_dwmac_parse_data(struct stm32_dwmac *dwmac,
				  struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct platform_device *pdev = to_platform_device(dev);

	int err;

	/*  Get ETHMAC_K clocks */
	dwmac->clk_ethmac_k = devm_clk_get(dev, "ethmac_k");
	if (IS_ERR(dwmac->clk_ethmac_k)) {
		dev_err(dev, "No ethmac_k clock provided...\n");
		return PTR_ERR(dwmac->clk_ethmac_k);
	}

	/*  Get TX/RX clocks */
	dwmac->clk_tx = devm_clk_get(dev, "mac-clk-tx");
	if (IS_ERR(dwmac->clk_tx)) {
		dev_err(dev, "No tx clock provided...\n");
		return PTR_ERR(dwmac->clk_tx);
	}
	dwmac->clk_rx = devm_clk_get(dev, "mac-clk-rx");
	if (IS_ERR(dwmac->clk_rx)) {
		dev_err(dev, "No rx clock provided...\n");
		return PTR_ERR(dwmac->clk_rx);
	}
	/*  to update and change naming */
	dwmac->clk_ck = devm_clk_get(dev, "mac-clk-ck");
	if (IS_ERR(dwmac->clk_ck)) {
		dev_err(dev, "No ck clock provided...\n");
		return PTR_ERR(dwmac->clk_ck);
	}

	/*  to update and change naming */
	dwmac->clk_ethstp = devm_clk_get(dev, "ethstp");
	if (IS_ERR(dwmac->clk_ethstp)) {
		dev_err(dev, "No ck clock provided...\n");
		return PTR_ERR(dwmac->clk_ethstp);
	}

	/*  to update and change naming */
	dwmac->syscfg_clk = devm_clk_get(dev, "syscfg-clk");
	if (IS_ERR(dwmac->syscfg_clk)) {
		dev_err(dev, "No ck clock provided...\n");
		return PTR_ERR(dwmac->syscfg_clk);
	}

	/* Get mode register */
	dwmac->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscon");
	if (IS_ERR(dwmac->regmap))
		return PTR_ERR(dwmac->regmap);

	err = of_property_read_u32_index(np, "st,syscon", 1, &dwmac->mode_reg);
	if (err) {
		dev_err(dev, "Can't get sysconfig mode offset (%d)\n", err);
		return err;
	}

	/* Get IRQ information early to have an ability to ask for deferred
	 * probe if needed before we went too far with resource allocation.
	 */
	dwmac->irq_pwr_wakeup = platform_get_irq_byname(pdev,
							"stm32_pwr_wakeup");
	if (dwmac->irq_pwr_wakeup < 0) {
		if (dwmac->irq_pwr_wakeup != -EPROBE_DEFER)
			dev_err(dev,
				"STM32 PWR WAKEUP IRQ configuration not found\n");

		return dwmac->irq_pwr_wakeup;
	}
	
	dwmac->provide_phy_clk = of_property_read_bool(np, "ex,provide-phy-clk");
	if (dwmac->provide_phy_clk){
		dev_info(dev, "Provide 50MHz clock for phy\n");
	}

	return err;
}

/* For stmexceet boards, that use 50MHz clock from SOC to supply PHY */
static int stm32_dwmac_ksz8081_phy_fixup(struct phy_device *phydev)
{
	int val;

	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		printk(KERN_INFO "Configure phy for 50MHz clock\n");
		// tell the PHY that a 50MHz clock is used
		val = phy_read(phydev, 0x1F);
		val |= 0x0080;
		phy_write(phydev, 0x1F, val);
	}

	return 0;
}

static void stm32_dwmac_fixup_phy(struct stm32_dwmac *dwmac)
{
	if (dwmac->provide_phy_clk){
		phy_register_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK, stm32_dwmac_ksz8081_phy_fixup);
	}
}


static void stm32_dwmac_remove_fixup_phy(struct stm32_dwmac *dwmac)
{
	if (dwmac->provide_phy_clk){
		phy_unregister_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK);
	}
}


static int stm32_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct stm32_dwmac *dwmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	ret = stm32_dwmac_parse_data(dwmac, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to parse OF data\n");
		goto err_remove_config_dt;
	}

	stm32_dwmac_fixup_phy(dwmac);

	plat_dat->bsp_priv = dwmac;

	ret = stm32_dwmac_init(plat_dat);
	if (ret)
		goto err_remove_config_dt;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_clk_disable;

	return 0;

err_clk_disable:
	stm32_dwmac_clk_disable(dwmac);
err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	ret = dev_pm_set_dedicated_wake_irq(&pdev->dev, dwmac->irq_pwr_wakeup);

	return ret;
}

static int stm32_dwmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	stm32_dwmac_remove_fixup_phy(priv->plat->bsp_priv);
	int ret = stmmac_dvr_remove(&pdev->dev);

	stm32_dwmac_clk_disable(priv->plat->bsp_priv);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int stm32_dwmac_suspend(struct device *dev)
{
//	struct net_device *ndev = dev_get_drvdata(dev);
//	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret;

	ret = stmmac_suspend(dev);
//	stm32_dwmac_clk_disable(priv->plat->bsp_priv);

	return ret;
}

static int stm32_dwmac_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret;

	ret = stm32_dwmac_init(priv->plat);
	if (ret)
		return ret;

	ret = stmmac_resume(dev);

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(stm32_dwmac_pm_ops,
	stm32_dwmac_suspend, stm32_dwmac_resume);

static const struct of_device_id stm32_dwmac_match[] = {
	{ .compatible = "st,stm32-dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, stm32_dwmac_match);

static struct platform_driver stm32_dwmac_driver = {
	.probe  = stm32_dwmac_probe,
	.remove = stm32_dwmac_remove,
	.driver = {
		.name           = "stm32-dwmac",
		.pm		= &stm32_dwmac_pm_ops,
		.of_match_table = stm32_dwmac_match,
	},
};
module_platform_driver(stm32_dwmac_driver);

MODULE_AUTHOR("Alexandre Torgue <alexandre.torgue@gmail.com>");
MODULE_DESCRIPTION("STMicroelectronics MCU DWMAC Specific Glue layer");
MODULE_LICENSE("GPL v2");
