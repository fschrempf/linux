/*
  *
  * Copyright (C) STMicroelectronics SA 2017
  * Author(s): Olivier Bideau <olivier.bideau@st.com> for STMicroelectronics.
  *
  * License terms: GPL V2.0.
  *
  * This program is free software; you can redistribute it and/or modify it
  * under the terms of the GNU General Public License version 2 as published by
  * the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
  * details.
  */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mfd/stm32-rcc-pwr.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/syscore_ops.h>

#define STM32_SVC_RCC	0x82001000
#define STM32_WRITE	0x1

#define SMC(class, op, address, val)\
	({\
	struct arm_smccc_res res;\
	arm_smccc_smc(class, op, address, val,\
			0, 0, 0, 0, &res);\
	})

/* This table lists the IPs for which CSLEEP is enabled */
static const u32 lp_table[] = {
	0x00000000, 0xB04, /* APB1 */
	0x00000000, 0xB0C, /* APB2 */
	0x00000800, 0xB14, /* APB3 */
	0x00000000, 0x304, /* APB4 */
	0x00000000, 0xB1C, /* AHB2 */
	0x00000000, 0xB24, /* AHB3 */
	0x00000000, 0xB2C, /* AHB4 */
	0x00000000, 0x31C, /* AHB6 */
	0x00000000, 0xB34, /* AXIM */
	0x00000000, 0xB3C, /* MLAHB */
};

struct stm32mp1_clk_dump {
	struct clk *clk;
	struct clk *clkp;
	bool state;
};

static const char * const clk_table[] = {
	"sdmmc1_k",
	"sdmmc2_k",
	"sdmmc3_k",
	"fmc_k",
	"qspi_k",
	"ethmac_k",
	"rng1_k",
	"rng2_k",
	"gpu_k",
	"usbphy_k",
	"stgen_k",
	"spdif_k",
	"spi1_k",
	"spi2_k",
	"spi3_k",
	"spi4_k",
	"spi5_k",
	"spi6_k",
	"cec_k",
	"i2c1_k",
	"i2c2_k",
	"i2c3_k",
	"i2c4_k",
	"i2c5_k",
	"i2c6_k",
	"lptim1_k",
	"lptim2_k",
	"lptim3_k",
	"lptim4_k",
	"lptim5_k",
	"usart1_k",
	"usart2_k",
	"usart3_k",
	"uart4_k",
	"uart5_k",
	"uart6_k",
	"uart7_k",
	"uart8_k",
	"dfsdm_k",
	"fdcan_k",
	"sai1_k",
	"sai2_k",
	"sai3_k",
	"sai4_k",
	"adc12_k",
	"dsi_k",
	"adfsdm_k",
};

#define MAX_CLK ARRAY_SIZE(clk_table)

struct stm32mp1_clk_dump saved_tree[MAX_CLK];

static const char * const bus_clk_table[] = {
	"tim2",
	"tim3",
	"tim4",
	"tim5",
	"tim6",
	"tim7",
	"tim12",
	"tim13",
	"tim14",
	"spi2",
	"spi3",
	"spdif",
	"dac12",
	"mdio",
	"tim1",
	"tim8",
	"tim15",
	"tim16",
	"tim17",
	"syscfg",
	"vref",
	"tmpsens",
	"pmbctrl",
	"hdp",
	"iwdg2",
	"usbphy",
	"stgenro",
	"rtcapb",
	"tzc",
	"tzpc",
	"bsec",
	"stgen",
	"dma1",
	"dma2",
	"dmamux",
	"usbo",
	"dcmi",
	"cryp2",
	"hash2",
	"rng2",
	"crc2",
	"hsem",
	"mbox",
	"gpioa",
	"gpiob",
	"gpioc",
	"gpiod",
	"gpioe",
	"gpiof",
	"gpiog",
	"gpioh",
	"gpioi",
	"gpioj",
	"gpiok",
	"gpioz",
	"cryp1",
	"rng1",
	"hash1",
	"bkpsram",
	"mdma",
	"ethck",
	"ethtx",
	"ethrx",
	"ethmac",
	"crc1",
	"usbh",
};

#define MAX_BUS_CLK ARRAY_SIZE(bus_clk_table)

struct stm32mp1_clk_dump bus_saved_tree[MAX_BUS_CLK];

static void __iomem *base;

static irqreturn_t stm32mp1_rcc_wakeup_handler(int irq, void *sdata)
{
	pr_info("RCC wakeup interrupt received\n");

	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_CIFR, RCC_WAKUP);

	return IRQ_HANDLED;
}

static irqreturn_t stm32mp1_rcc_irq_handler(int irq, void *sdata)
{
	pr_info("RCC generic interrupt received\n");

	/* clear interrupt flag */
	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_CIFR, RCC_IRQ_FLAGS_MASK);

	return IRQ_HANDLED;
}

int stm32mp1_setup_rcc_cst(bool enable)
{
	if (enable)
		SMC(STM32_SVC_RCC, STM32_WRITE, RCC_SREQSETR, RCC_STOP_MASK);
	else
		SMC(STM32_SVC_RCC, STM32_WRITE, RCC_SREQCLRR, RCC_STOP_MASK);

	return 0;
}

static int stm32mp1_clk_suspend(void)
{
	struct clk *clk1, *clk2;
	int i;

	for (i = 0; i < MAX_CLK; i++) {
		clk1 = __clk_lookup(clk_table[i]);
		clk2 = clk_get_parent(clk1);
		saved_tree[i].clk = clk1;
		saved_tree[i].clkp = clk2;
		if (__clk_is_enabled(clk1) && __clk_get_enable_count(clk1))
			saved_tree[i].state = 1;
	}

	for (i = 0; i < MAX_BUS_CLK; i++) {
		clk1 = __clk_lookup(bus_clk_table[i]);

		bus_saved_tree[i].clk = clk1;

		if (__clk_is_enabled(clk1) && __clk_get_enable_count(clk1))
			bus_saved_tree[i].state = 1;
	}

	return 0;
}

static void stm32mp1_clk_resume(void)
{
	struct clk *clk1, *clk2;
	int i;

	for (i = 0; i < MAX_CLK; i++) {
		clk1 = saved_tree[i].clk;
		clk_prepare_enable(clk1);

		clk2 = saved_tree[i].clkp;
		clk_set_parent(clk1, clk2);
	}

	for (i = 0; i < MAX_BUS_CLK; i++)	{
		clk1 = bus_saved_tree[i].clk;

		if (bus_saved_tree[i].state)
			clk_prepare_enable(clk1);
	}
}

static struct syscore_ops stm32mp1_clk_ops = {
	.suspend	= stm32mp1_clk_suspend,
	.resume		= stm32mp1_clk_resume,
};

static int stm32mp1_rcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq;
	int ret;
	int i;

	struct device_node *np;

	np = of_get_parent(dev->of_node); /* parent should be syscon node */
	if (!np)
		return -ENODEV;

	/* register generic irq */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to get stm32mp1 RCC generic IRQ\n");
		return irq;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, stm32mp1_rcc_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev->driver->name, pdev);

	if (ret) {
		dev_err(dev, "failed to register generic IRQ %d\n", irq);
		return ret;
	}

	/* register wakeup irq */
	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(dev, "failed to get stm32mp1 RCC wakeup IRQ\n");
		return irq;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, stm32mp1_rcc_wakeup_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev->driver->name, pdev);

	if (ret) {
		dev_err(dev, "failed to register wakeup IRQ %d\n", irq);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret) {
		dev_err(dev, "failed to enable RCC IRQ as wakeup %d\n", irq);
		return ret;
	}

	base = of_iomap(np, 0);
	if (IS_ERR(base)) {
		pr_err("%s: unable to map resource", np->name);
		return PTR_ERR(base);
	}

	/* Configure LPEN static table */
	for (i = 0; i < ARRAY_SIZE(lp_table); i += 2)
		writel_relaxed(lp_table[i], base + lp_table[i + 1]);

	/* cleanup RCC flags */
	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_CIFR, RCC_IRQ_FLAGS_MASK);

	stm32mp1_setup_rcc_cst(DISABLE);

	register_syscore_ops(&stm32mp1_clk_ops);

	dev_info(dev, "stm32mp1 RCC driver initialized\n");

	return ret;
}

static const struct of_device_id stm32mp1_rcc_match[] = {
	{ .compatible = "st,stm32mp1-rcc-pwr" },
	{},
};
MODULE_DEVICE_TABLE(of, stm32mp1_rcc_match);

static struct platform_driver stm32mp1_rcc_driver = {
	.probe = stm32mp1_rcc_probe,
	.driver = {
		.name = "stm32mp1-rcc",
		.owner = THIS_MODULE,
		.of_match_table = stm32mp1_rcc_match,
	},
};

static int __init stm32mp1_rcc_driver_init(void)
{
	return platform_driver_register(&stm32mp1_rcc_driver);
}
subsys_initcall(stm32mp1_rcc_driver_init);
