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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/stm32_pm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

#define STM32_SVC_PWR	0x82001001
#define STM32_WRITE	0x1
#define PWR_CR2		0x50001008
#define PWR_MPUPCR	0x50001010
#define PWR_WKUPCR	0x50001020
#define PWR_WKUPENR	0x50001028

#define SMC(class, op, address, val)		\
{						\
	struct arm_smccc_res res;		\
	arm_smccc_smc(class, op, address, val,	\
		      0, 0, 0, 0, &res);	\
}						\

static void __iomem *base;

static DEFINE_SPINLOCK(pwr_lock);

static irqreturn_t stm32_pwr_irq_handler(int irq, void *sdata)
{
	u32 val;

	val = readl_relaxed(base + WKUPFR);

	/* clear interrupt flags in WKUPCR */
	SMC(STM32_SVC_PWR, STM32_WRITE, PWR_WKUPCR, val | WUP_FLAGS_MASK);

	return IRQ_HANDLED;
}

static irqreturn_t stm32_hold_irq_handler(int irq, void *sdata)
{
	u32 val;

	/* CPU is awake, clear the hold flag */
	val = readl_relaxed(base + MPUCR);

	SMC(STM32_SVC_PWR, STM32_WRITE, PWR_MPUPCR, val | CSSF);

	return IRQ_HANDLED;
}

/* The next two functions configure the wkup pins */
int stm32_setup_pwr_irq(int pin_id, bool enable)
{
	u32 val;
	u8 pin;
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);

	pin_id = pin_id - 1;
	pin = 1 << pin_id;

	val = readl_relaxed(base + WKUPENR);

	if (enable)
		val |= pin;
	else
		val &= ~(pin);

	SMC(STM32_SVC_PWR, STM32_WRITE, PWR_WKUPENR, val);

	spin_unlock_irqrestore(&pwr_lock, flags);

	return 0;
}

int stm32_configure_wkup_pin(int pin_id, int pupd, int polarity)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);

	val = readl_relaxed(base + WKUPCR);

	/* adjust offset */
	pin_id = pin_id - 1;

	/* pupd */
	val &= ~(WKUPUPD_MASK << (WKUPUPD1_SHIFT + 2 * pin_id));
	val |= (pupd << (WKUPUPD1_SHIFT + 2 * pin_id));

	/* polarity */
	val &= ~(WKUPP_MASK << (WKUPP1_SHIFT + pin_id));
	val |= (polarity << (WKUPP1_SHIFT + pin_id));

	writel_relaxed(val, base + WKUPCR);

	/* Clear flags */
	val |= WUP_FLAGS_MASK;
	SMC(STM32_SVC_PWR, STM32_WRITE, PWR_WKUPCR, val);

	spin_unlock_irqrestore(&pwr_lock, flags);

	return 0;
}

int stm32_setup_ret_reg(bool enable)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);

	val = readl_relaxed(base + CR2);

	if (enable)
		val |= RREN;
	else
		val &= ~(RREN);

	SMC(STM32_SVC_PWR, STM32_WRITE, PWR_CR2, val);

	spin_unlock_irqrestore(&pwr_lock, flags);

	return 0;
}

int stm32_setup_backup_reg(bool enable)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&pwr_lock, flags);

	val = readl_relaxed(base + CR2);

	if (enable)
		val |= BREN;
	else
		val &= ~(BREN);

	SMC(STM32_SVC_PWR, STM32_WRITE, PWR_CR2, val);

	spin_unlock_irqrestore(&pwr_lock, flags);

	return 0;
}

static int stm32_pwr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq, irq2;
	int ret;

	/* register irq */
	irq = platform_get_irq(pdev, 0);

	if (irq < 0) {
		dev_err(dev, "failed to get PWR IRQ\n");
		return irq;
	}

	ret = devm_request_threaded_irq(dev, irq,
					NULL, stm32_pwr_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev->driver->name, pdev);

	if (ret) {
		dev_err(dev, "failed to register PWR IRQ %d\n", irq);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret) {
		dev_err(dev, "failed to enable PWR IRQ as wakeup %d\n", irq);
		return ret;
	}

	irq2 = platform_get_irq(pdev, 1);

	if (irq2 < 0) {
		dev_err(dev, "failed to get HOLD IRQ\n");
		return irq2;
	}

	ret = devm_request_threaded_irq(dev, irq2,
					NULL, stm32_hold_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev->driver->name, pdev);

	if (ret) {
		dev_err(dev, "failed to register HOLD IRQ %d\n", irq2);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	stm32_configure_wkup_pin(WKUP1, NO_PULL, ENABLE);
	stm32_setup_pwr_irq(WKUP1, ENABLE);

	/* Keep retention and backup ram content in standby */
	stm32_setup_ret_reg(ENABLE);
	stm32_setup_backup_reg(ENABLE);

	dev_info(dev, "stm32 PWR driver initialized\n");

	return 0;
}

static const struct of_device_id stm32_pwr_match[] = {
	{ .compatible = "st,stm32mp1-pwr" },
	{},
};

static struct platform_driver stm32_pwr_driver = {
	.probe = stm32_pwr_probe,
	.driver = {
		.name = "stm32-pwr",
		.owner = THIS_MODULE,
		.of_match_table = stm32_pwr_match,
	},
};

static int __init stm32_pwr_init(void)
{
	return platform_driver_register(&stm32_pwr_driver);
}
arch_initcall(stm32_pwr_init);
