/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) STMicroelectronics 2017 - All Rights Reserved
 * Author: Olivier Bideau <olivier.bideau@st.com> for STMicroelectronics.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/stm32mp1-clks.h>

#include "clk-stm32-core.h"
#include "clk-stm32mp1.h"

struct stm32_pll_obj {
	void __iomem *reg;
	/* lock pll enable/disable registers */
	spinlock_t *lock;
	struct clk_hw hw;
};

struct timer_cker {
	void __iomem *apbdiv;
	void __iomem *timpre;
	struct clk_hw hw;
	/* lock the kernel output divider register */
	spinlock_t *lock;
};


#define to_pll(_hw) container_of(_hw, struct stm32_pll_obj, hw)

static int __pll_is_enabled(struct clk_hw *hw)
{
	struct stm32_pll_obj *clk_elem = to_pll(hw);

	return readl_relaxed(clk_elem->reg) & BIT(0);
}

static int pll_enable(struct clk_hw *hw)
{
	struct stm32_pll_obj *clk_elem = to_pll(hw);
	u32 reg;
	unsigned long flags = 0;

	spin_lock_irqsave(clk_elem->lock, flags);

	if (__pll_is_enabled(hw))
		goto unlock;

	reg = readl_relaxed(clk_elem->reg);
	reg |= (ODF_MASK << ODF_SHIFT);
	writel_relaxed(reg, clk_elem->reg);

	reg = readl_relaxed(clk_elem->reg);
	reg |= 0x1;
	writel_relaxed(reg, clk_elem->reg);

	/* Wait for lock */
	while ((readl_relaxed(clk_elem->reg) & BIT(1)) == 0)
		;

unlock:
	spin_unlock_irqrestore(clk_elem->lock, flags);

	return 0;
}

static void pll_disable(struct clk_hw *hw)
{
	struct stm32_pll_obj *clk_elem = to_pll(hw);
	u32 reg;
	unsigned long flags = 0;

	spin_lock_irqsave(clk_elem->lock, flags);

	reg = readl_relaxed(clk_elem->reg);
	reg &= ~0x1;
	writel_relaxed(reg, clk_elem->reg);

	reg &= ~(ODF_MASK << ODF_SHIFT);
	writel_relaxed(reg, clk_elem->reg);

	spin_unlock_irqrestore(clk_elem->lock, flags);
}

static u32 pll_frac_val(struct clk_hw *hw)
{
	struct stm32_pll_obj *clk_elem = to_pll(hw);
	u32 reg, frac = 0;

	reg = readl_relaxed(clk_elem->reg + FRAC_OFFSET);
	if (reg & BIT(16))
		frac = (reg >> FRAC_SHIFT) & FRAC_MASK;

	return frac;
}

static unsigned long pll_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	struct stm32_pll_obj *clk_elem = to_pll(hw);
	u32 reg;
	u32 frac, divm, divn;
	u64 rate, rate_frac = 0;

	reg = readl_relaxed(clk_elem->reg + 4);

	divm = ((reg >> DIVM_SHIFT) & DIVM_MASK) + 1;
	divn = ((reg >> DIVN_SHIFT) & DIVN_MASK) + 1;
	rate = (u64)parent_rate * divn;

	do_div(rate, divm);

	frac = pll_frac_val(hw);
	if (frac) {
		rate_frac = (u64)parent_rate * (u64)frac;
		do_div(rate_frac, (divm * 8192));
	}

	return rate + rate_frac;
}

static int pll_is_enabled(struct clk_hw *hw)
{
	struct stm32_pll_obj *clk_elem = to_pll(hw);
	unsigned long flags = 0;
	int ret;

	spin_lock_irqsave(clk_elem->lock, flags);
	ret = __pll_is_enabled(hw);
	spin_unlock_irqrestore(clk_elem->lock, flags);

	return ret;
}

static const struct clk_ops pll_ops = {
	.enable = pll_enable,
	.disable = pll_disable,
	.recalc_rate = pll_recalc_rate,
	.is_enabled = pll_is_enabled,
};

static struct clk_hw *clk_register_pll(struct device *dev, const char *name,
				       const char *parent_name,
				       void __iomem *reg,
				       unsigned long flags,
				       spinlock_t *lock)
{
	struct stm32_pll_obj *element;
	struct clk_init_data init;
	struct clk_hw *hw;
	int err;

	element = kzalloc(sizeof(*element), GFP_KERNEL);
	if (!element)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &pll_ops;
	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	element->hw.init = &init;
	element->reg = reg;
	element->lock = lock;

	hw = &element->hw;
	err = clk_hw_register(dev, hw);

	if (err) {
		kfree(element);
		return ERR_PTR(err);
	}

	return hw;
}

#define to_timer_cker(_hw) container_of(_hw, struct timer_cker, hw)

#define APB_DIV_MASK 0x07
#define TIM_PRE_MASK 0x01

static unsigned long __bestmult(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct timer_cker *tim_ker = to_timer_cker(hw);
	u32 prescaler;
	unsigned int mult = 0;

	prescaler = readl_relaxed(tim_ker->apbdiv) & APB_DIV_MASK;
	if (prescaler < 2)
		return 1;

	mult = 2;

	if (rate / parent_rate >= 4)
		mult = 4;

	return mult;
}

static long timer_ker_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *parent_rate)
{
	unsigned long factor = __bestmult(hw, rate, *parent_rate);

	return *parent_rate * factor;
}

static int timer_ker_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct timer_cker *tim_ker = to_timer_cker(hw);
	unsigned long flags = 0;
	unsigned long factor = __bestmult(hw, rate, parent_rate);
	int ret = 0;

	spin_lock_irqsave(tim_ker->lock, flags);

	switch (factor) {
	case 1:
		break;
	case 2:
		writel_relaxed(0, tim_ker->timpre);
		break;
	case 4:
		writel_relaxed(1, tim_ker->timpre);
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(tim_ker->lock, flags);

	return ret;
}

static unsigned long timer_ker_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct timer_cker *tim_ker = to_timer_cker(hw);
	u32 prescaler, timpre;
	u32 mul;

	prescaler = readl_relaxed(tim_ker->apbdiv) & APB_DIV_MASK;

	timpre = readl_relaxed(tim_ker->timpre) & TIM_PRE_MASK;

	if (!prescaler)
		return parent_rate;

	mul = (timpre + 1) * 2;

	return parent_rate * mul;
}

static const struct clk_ops timer_ker_ops = {
	.recalc_rate	= timer_ker_recalc_rate,
	.round_rate	= timer_ker_round_rate,
	.set_rate	= timer_ker_set_rate,

};

static struct clk_hw *clk_register_cktim(struct device *dev, const char *name,
					 const char *parent_name,
					 unsigned long flags,
					 void __iomem *apbdiv,
					 void __iomem *timpre,
					 spinlock_t *lock)
{
	struct timer_cker *tim_ker;
	struct clk_init_data init;
	struct clk_hw *hw;
	int err;

	tim_ker = kzalloc(sizeof(*tim_ker), GFP_KERNEL);
	if (!tim_ker)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &timer_ker_ops;
	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	tim_ker->hw.init = &init;
	tim_ker->lock = lock;
	tim_ker->apbdiv = apbdiv;
	tim_ker->timpre = timpre;

	hw = &tim_ker->hw;
	err = clk_hw_register(dev, hw);

	if (err) {
		kfree(tim_ker);
		return ERR_PTR(err);
	}

	return hw;
}

struct clk_hw *_clk_register_pll(struct device *dev,
				 struct clk_hw_onecell_data *clk_data,
				 void __iomem *base, spinlock_t *lock,
				 const struct clock_config *cfg)
{
	struct stm32_pll_cfg *stm_pll_cfg = cfg->cfg;
	struct clk_hw *hw;

	hw = clk_register_pll(dev, cfg->name, cfg->parent_name,
			      base + stm_pll_cfg->offset, cfg->flags, lock);
	if (!IS_ERR(hw))
		if (clk_hw_is_enabled(hw))
			clk_prepare_enable(hw->clk);
	return hw;
}

struct clk_hw *_clk_register_cktim(struct device *dev,
				   struct clk_hw_onecell_data *clk_data,
				   void __iomem *base, spinlock_t *lock,
				   const struct clock_config *cfg)
{
	struct stm32_cktim_cfg	*cktim_cfg = cfg->cfg;

	return clk_register_cktim(dev, cfg->name, cfg->parent_name, cfg->flags,
				  cktim_cfg->offset_apbdiv + base,
				  cktim_cfg->offset_timpre + base, lock);
}

static struct stm32_clock_match_data stm32mp1_data = {
	.cfg		= stm32mp1_clock_cfg,
	.num		= ARRAY_SIZE(stm32mp1_clock_cfg),
	.maxbinding	= STM32MP1_LAST_CLK,
};

static const struct of_device_id stm32mp1_match_data[] = {
	{
		.compatible = "st,stm32mp1-rcc-clk",
		.data = &stm32mp1_data,
	},
	{ }
};

static void stm32mp1_rcc_init(struct device_node *np)
{
	return stm32_rcc_init(np, stm32mp1_match_data);
}

CLK_OF_DECLARE(stm32mp1_rcc, "st,stm32mp1-rcc-clk", stm32mp1_rcc_init);
