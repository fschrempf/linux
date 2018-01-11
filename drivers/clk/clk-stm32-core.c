/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) STMicroelectronics 2017 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com> for STMicroelectronics.
 */

#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include "clk-stm32-core.h"

static DEFINE_SPINLOCK(rlock);

int stm32_gate_clk_is_enabled(struct clk_hw *hw)
{
	struct stm32_gate *gate = to_stm32_gate_clk(hw);

	return !!(((readl_relaxed(gate->reg_set) >> gate->bit_idx) & 0x1));
}

int stm32_gate_clk_enable(struct clk_hw *hw)
{
	struct stm32_gate *gate = to_stm32_gate_clk(hw);
	unsigned long flags = 0;
	u32 val;

	if (stm32_gate_clk_is_enabled(hw))
		return 0;

	spin_lock_irqsave(gate->lock, flags);

	/* Enable the gate */
	val = readl_relaxed(gate->reg_set);
	val |= BIT(gate->bit_idx);

	writel_relaxed(val, gate->reg_set);

	spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

void stm32_gate_clk_disable(struct clk_hw *hw)
{
	struct stm32_gate *gate = to_stm32_gate_clk(hw);
	unsigned long flags = 0;
	u32 val;

	if (!stm32_gate_clk_is_enabled(hw))
		return;

	spin_lock_irqsave(gate->lock, flags);

	val = readl_relaxed(gate->reg_clr);

	if (gate->reg_set == gate->reg_clr)
		val &= ~BIT(gate->bit_idx);
	else
		val = BIT(gate->bit_idx);

	writel_relaxed(val, gate->reg_clr);

	spin_unlock_irqrestore(gate->lock, flags);
}

const struct clk_ops stm32_gate_clk_ops = {
	.enable		= stm32_gate_clk_enable,
	.disable	= stm32_gate_clk_disable,
	.is_enabled	= stm32_gate_clk_is_enabled,
};

struct clk_hw *clk_register_stm32_gate(struct device *dev,
				       const char *name,
				       const char *parent_name,
				       void __iomem *base,
				       const struct stm32_gate_cfg *gate_cfg,
				       unsigned long flags, spinlock_t *lock)
{
	struct clk_init_data init = { NULL };
	struct stm32_gate *gate;
	int ret;
	struct clk_hw *hw;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &stm32_gate_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	gate->reg_set = gate_cfg->offset_set + base;
	gate->reg_clr = gate_cfg->offset_clr + base;
	gate->bit_idx = gate_cfg->bit_idx;

	gate->lock = lock;

	gate->hw.init = &init;
	hw = &gate->hw;

	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}
	return hw;
}

struct clk_mux *_get_cmux(void __iomem *reg, u8 shift, u8 width,
			  u32 flags, spinlock_t *lock)
{
	struct clk_mux *mux;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	mux->reg = reg;
	mux->shift = shift;
	mux->mask = (1 << width) - 1;
	mux->flags = flags;
	mux->lock = lock;

	return mux;
}

struct clk_divider *_get_cdiv(void __iomem *reg, u8 shift, u8 width,
			      const struct clk_div_table *table, u32 flags,
			      spinlock_t *lock)
{
	struct clk_divider *div;

	div = kzalloc(sizeof(*div), GFP_KERNEL);

	if (!div)
		return ERR_PTR(-ENOMEM);

	div->reg = reg;
	div->shift = shift;
	div->width = width;
	div->flags = flags;
	div->lock = lock;
	div->table = table;

	return div;
}

struct stm32_gate *_get_stm32_cgate(void __iomem *reg_set,
				    void __iomem *reg_clr,
				    u8 bit_idx, u32 flags,
				    spinlock_t *lock)
{
	struct stm32_gate *gate;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	gate->reg_set = reg_set;
	gate->reg_clr = reg_clr;
	gate->bit_idx = bit_idx;
	gate->lock = lock;

	return gate;
}

struct clk_hw *
clk_stm32_register_composite(struct device *dev,
			     const char *name, const char * const *parent_names,
			     int num_parents, void __iomem *base,
			     const struct stm32_composite_cfg *cfg,
			     unsigned long flags, spinlock_t *lock)
{
	struct clk_mux *mux = NULL;
	struct clk_divider *div = NULL;
	struct stm32_gate *gate = NULL;
	const struct clk_ops *mux_ops, *div_ops, *gate_ops;
	struct clk_hw *hw;
	struct clk_hw *mux_hw;
	struct clk_hw *div_hw;
	struct clk_hw *gate_hw;

	mux_hw = NULL;
	div_hw = NULL;
	gate_hw = NULL;
	mux_ops = NULL;
	div_ops = NULL;
	gate_ops = NULL;

	if (cfg->mux_c && cfg->mux) {
		mux = _get_cmux(base + cfg->mux->offset,
				cfg->mux->shift,
				cfg->mux->width,
				cfg->mux_c->flags, lock);

		if (!IS_ERR(mux)) {
			mux_hw = &mux->hw;
			mux_ops = cfg->mux_c->ops ?
				  cfg->mux_c->ops : &clk_mux_ops;
		}
	}

	if (cfg->div_c && cfg->div) {
		div = _get_cdiv(base + cfg->div->offset,
				cfg->div->shift,
				cfg->div->width,
				cfg->div->table,
				cfg->div_c->flags, lock);

		if (!IS_ERR(div)) {
			div_hw = &div->hw;
			div_ops = cfg->div_c->ops ?
				  cfg->div_c->ops : &clk_divider_ops;
		}
	}

	if (cfg->gate_c && cfg->gate) {
		gate = _get_stm32_cgate(base + cfg->gate->offset_set,
					base + cfg->gate->offset_clr,
					cfg->gate->bit_idx,
					cfg->gate_c->flags, lock);

		if (!IS_ERR(gate)) {
			gate_hw = &gate->hw;
			gate_ops = cfg->gate_c->ops ?
				   cfg->gate_c->ops : &stm32_gate_clk_ops;
		}
	}

	hw = clk_hw_register_composite(dev, name, parent_names, num_parents,
				       mux_hw, mux_ops, div_hw, div_ops,
				       gate_hw, gate_ops, flags);

	return hw;
}

struct clk_hw *
_clk_hw_register_fixed_factor(struct device *dev,
			      struct clk_hw_onecell_data *clk_data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg)
{
	struct fixed_factor_cfg *ff_cfg = cfg->cfg;

	return clk_hw_register_fixed_factor(dev, cfg->name, cfg->parent_name,
					    cfg->flags, ff_cfg->mult,
					    ff_cfg->div);
}

struct clk_hw *
_clk_hw_register_gate(struct device *dev,
		      struct clk_hw_onecell_data *clk_data,
		      void __iomem *base, spinlock_t *lock,
		      const struct clock_config *cfg)
{
	struct gate_cfg *gate_cfg = cfg->cfg;

	return clk_hw_register_gate(dev,
				    cfg->name,
				    cfg->parent_name,
				    cfg->flags,
				    gate_cfg->reg_off + base,
				    gate_cfg->bit_idx,
				    gate_cfg->gate_flags,
				    lock);
}

struct clk_hw *
_clk_hw_register_divider_table(struct device *dev,
			       struct clk_hw_onecell_data *clk_data,
			       void __iomem *base, spinlock_t *lock,
			       const struct clock_config *cfg)
{
	struct divider_cfg *div_cfg = cfg->cfg;

	return clk_hw_register_divider_table(dev,
					     cfg->name,
					     cfg->parent_name,
					     cfg->flags,
					     div_cfg->reg_off + base,
					     div_cfg->shift,
					     div_cfg->width,
					     div_cfg->div_flags,
					     div_cfg->table,
					     lock);
}

struct clk_hw *
_clk_hw_register_mux(struct device *dev,
		     struct clk_hw_onecell_data *clk_data,
		     void __iomem *base, spinlock_t *lock,
		     const struct clock_config *cfg)
{
	struct mux_cfg *mux_cfg = cfg->cfg;

	return clk_hw_register_mux(dev,	cfg->name, cfg->parent_names,
				   cfg->num_parents, cfg->flags,
				   mux_cfg->reg_off + base, mux_cfg->shift,
				   mux_cfg->width, mux_cfg->mux_flags, lock);
}

struct clk_hw *
_clk_register_stm32_gate(struct device *dev,
			 struct clk_hw_onecell_data *clk_data,
			 void __iomem *base, spinlock_t *lock,
			 const struct clock_config *cfg)
{
	return clk_register_stm32_gate(dev, cfg->name, cfg->parent_name, base,
				       cfg->cfg, cfg->flags, lock);
}

struct clk_hw *
_clk_stm32_register_composite(struct device *dev,
			      struct clk_hw_onecell_data *clk_data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg)
{
	return clk_stm32_register_composite(dev, cfg->name, cfg->parent_names,
					    cfg->num_parents, base, cfg->cfg,
					    cfg->flags,	lock);
}

int register_hw_clk(struct device *dev, struct clk_hw_onecell_data *clk_data,
		    void __iomem *base, spinlock_t *lock,
		    const struct clock_config *cfg)
{
	static struct clk_hw **hws;
	struct clk_hw *hw = ERR_PTR(-ENOENT);

	hws = clk_data->hws;

	if (cfg->func)
		hw = (*cfg->func)(dev, clk_data, base, lock, cfg);

	if (IS_ERR(hw)) {
		pr_err("Unable to register %s\n", cfg->name);
		return  PTR_ERR(hw);
	}

	if (cfg->id != NO_ID)
		hws[cfg->id] = hw;

	return 0;
}

void stm32_rcc_init(struct device_node *np,
		    const struct of_device_id *match_data)
{
	struct clk_hw_onecell_data *clk_data;
	struct device_node *parent;
	static struct clk_hw **hws;
	static void __iomem *base;
	const struct of_device_id *match;
	const struct stm32_clock_match_data *data;
	const struct clock_config *clock_cfg;
	int n;
	int num_clk;
	int max_binding;

	match = of_match_node(match_data, np);
	if (WARN_ON(!match))
		return;

	data = match->data;

	clock_cfg = data->cfg;
	num_clk = data->num;
	max_binding =  data->maxbinding;

	clk_data = kzalloc(sizeof(*clk_data) +
				  sizeof(*clk_data->hws) * max_binding,
				  GFP_KERNEL);
	if (!clk_data)
		return;

	clk_data->num = max_binding;

	hws = clk_data->hws;

	for (n = 0; n < max_binding; n++)
		hws[n] = ERR_PTR(-ENOENT);

	parent = of_get_parent(np);
	if (!parent) {
		pr_err("%s: parent should be syscon node\n", __func__);
		return;
	}

	base = of_iomap(parent, 0);
	if (!base) {
		pr_err("%s: unable to map resource", parent->name);
		goto err_free_data;
	}

	for (n = 0; n < num_clk; n++) {
		if (register_hw_clk(NULL, clk_data, base, &rlock,
				    &clock_cfg[n])) {
			pr_err("%s: can't register  %s\n", __func__,
			       clock_cfg[n].name);
			goto err_io;
		}
	}

	of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_data);

	return;

err_io:
	iounmap(base);
err_free_data:
	kfree(clk_data);
}

/* Security management */

/* Write Secure TODO create new API file */
#define STM32MP1_SVC_RCC 0x82001000
#define STM32MP1_WRITE 0x1
u32 stm32_clk_writel_secure(u32 value, void *reg)
{
	struct arm_smccc_res res;
	u32 address;

	address = page_to_phys(vmalloc_to_page(reg)) + offset_in_page(reg);

	arm_smccc_smc(STM32MP1_SVC_RCC, STM32MP1_WRITE, address, value, 0, 0, 0,
		      0, &res);

	if (res.a0)
		pr_warn("%s: Failed to write in secure mode at 0x%x (err = %ld)\n"
				, __func__
				, address
				, res.a0);

	return res.a0;
}

int stm32_sgate_clk_enable(struct clk_hw *hw)
{
	struct stm32_gate *gate = to_stm32_gate_clk(hw);
	unsigned long flags = 0;
	u32 val;

	if (stm32_gate_clk_is_enabled(hw))
		return 0;

	spin_lock_irqsave(gate->lock, flags);

	/* Enable the gate */
	val = readl_relaxed(gate->reg_set);
	val |= BIT(gate->bit_idx);

	stm32_clk_writel_secure(val, gate->reg_set);

	spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

void stm32_sgate_clk_disable(struct clk_hw *hw)
{
	struct stm32_gate *gate = to_stm32_gate_clk(hw);
	unsigned long flags = 0;
	u32 val;

	if (!stm32_gate_clk_is_enabled(hw))
		return;

	spin_lock_irqsave(gate->lock, flags);

	val = readl_relaxed(gate->reg_clr);

	if (gate->reg_set == gate->reg_clr)
		val &= ~BIT(gate->bit_idx);
	else
		val = BIT(gate->bit_idx);

	stm32_clk_writel_secure(val, gate->reg_clr);

	spin_unlock_irqrestore(gate->lock, flags);
}

const struct clk_ops stm32_gate_s_clk_ops = {
	.enable		= stm32_sgate_clk_enable,
	.disable	= stm32_sgate_clk_disable,
	.is_enabled	= stm32_gate_clk_is_enabled,
};

struct clk_hw *clk_register_stm32_sgate(struct device *dev,
					const char *name,
					const char *parent_name,
					void __iomem *base,
					const struct stm32_gate_cfg *gate_cfg,
					unsigned long flags, spinlock_t *lock)
{
	struct clk_init_data init = { NULL };
	struct stm32_gate *gate;
	int ret;
	struct clk_hw *hw;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &stm32_gate_s_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	gate->reg_set = gate_cfg->offset_set + base;
	gate->reg_clr = gate_cfg->offset_clr + base;
	gate->bit_idx = gate_cfg->bit_idx;

	gate->lock = lock;

	gate->hw.init = &init;
	hw = &gate->hw;

	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}
	return hw;
}

static u8  clk_mux_get_parent_secure(struct clk_hw *hw)
{
	return clk_mux_ops.get_parent(hw);
}

static int clk_mux_set_parent_secure(struct clk_hw *hw, u8 index)
{
	struct clk_mux *mux = to_clk_mux(hw);
	u32 val;
	unsigned long flags = 0;

	if (mux->table) {
		index = mux->table[index];
	} else {
		if (mux->flags & CLK_MUX_INDEX_BIT)
			index = 1 << index;

		if (mux->flags & CLK_MUX_INDEX_ONE)
			index++;
	}

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);
	else
		__acquire(mux->lock);

	val = clk_readl(mux->reg);
	val &= ~(mux->mask << mux->shift);
	val |= index << mux->shift;

	stm32_clk_writel_secure(val, mux->reg);

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);
	else
		__release(mux->lock);

	return 0;
}

const struct clk_ops clk_mux_s_ops = {
	.get_parent = clk_mux_get_parent_secure,
	.set_parent = clk_mux_set_parent_secure,
	.determine_rate = __clk_mux_determine_rate,
};

struct clk_div_secure {
	struct clk_divider div;
	u8 secure;
};

#define to_clk_div_secure(_hw) container_of(_hw, struct clk_div_secure, div)

static unsigned long clk_divider_recalc_rate_secure(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	return clk_divider_ops.recalc_rate(hw, parent_rate);
}

static long clk_divider_round_rate_secure(struct clk_hw *hw, unsigned long rate,
					  unsigned long *prate)
{
	return clk_divider_ops.round_rate(hw, rate, prate);
}

#define div_mask(width) ((1 << (width)) - 1)

static int clk_divider_set_rate_secure(struct clk_hw *hw, unsigned long rate,
				       unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int value;
	unsigned long flags = 0;
	u32 val;

	value = divider_get_val(rate, parent_rate, divider->table,
				divider->width, divider->flags);

	if (value < 0)
		return value;

	if (divider->lock)
		spin_lock_irqsave(divider->lock, flags);
	else
		__acquire(divider->lock);

	if (divider->flags & CLK_DIVIDER_HIWORD_MASK) {
		val = div_mask(divider->width) << (divider->shift + 16);
	} else {
		val = clk_readl(divider->reg);
		val &= ~(div_mask(divider->width) << divider->shift);
	}
	val |= (u32)value << divider->shift;

	stm32_clk_writel_secure(val, divider->reg);

	if (divider->lock)
		spin_unlock_irqrestore(divider->lock, flags);
	else
		__release(divider->lock);

	return 0;
}

const struct clk_ops clk_divider_secure_ops = {
	.recalc_rate = clk_divider_recalc_rate_secure,
	.round_rate = clk_divider_round_rate_secure,
	.set_rate = clk_divider_set_rate_secure,
};

static struct clk_hw *_register_divider_sec(struct device *dev,
					    const char *name,
					    const char *parent_name,
					    unsigned long flags,
					    void __iomem *reg,
					    u8 shift, u8 width,
					    u8 clk_divider_flags,
					    const struct clk_div_table *table,
					    spinlock_t *lock)
{
	struct clk_divider *div;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	if (clk_divider_flags & CLK_DIVIDER_HIWORD_MASK) {
		if (width + shift > 16) {
			pr_warn("divider value exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the divider */
	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	if (clk_divider_flags & CLK_DIVIDER_READ_ONLY)
		init.ops = &clk_divider_ro_ops;
	else
		init.ops = &clk_divider_secure_ops;

	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_divider assignments */
	div->reg = reg;
	div->shift = shift;
	div->width = width;
	div->flags = clk_divider_flags;
	div->lock = lock;
	div->hw.init = &init;
	div->table = table;

	/* register the clock */
	hw = &div->hw;

	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(div);
		hw = ERR_PTR(ret);
	}

	return hw;
}

struct clk *
clk_register_divider_secure(struct device *dev, const char *name,
			    const char *parent_name, unsigned long flags,
			    void __iomem *reg, u8 shift, u8 width,
			    u8 clk_divider_flags, spinlock_t *lock)
{
	struct clk_hw *hw;

	hw =  _register_divider_sec(dev, name, parent_name, flags, reg, shift,
				    width, clk_divider_flags, NULL, lock);
	if (IS_ERR(hw))
		return ERR_CAST(hw);

	return hw->clk;
}

struct clk_hw *
clk_stm32_register_composite_s(struct device *dev,
			       const char *name,
			       const char * const *parent_names,
			       int num_parents,
			       void __iomem *base,
			       const struct stm32_composite_cfg *cfg,
			       unsigned long flags,
			       spinlock_t *lock)
{
	struct clk_mux *mux = NULL;
	struct clk_divider *div = NULL;
	struct stm32_gate *gate = NULL;
	const struct clk_ops *mux_ops, *div_ops, *gate_ops;
	struct clk_hw *hw;
	struct clk_hw *mux_hw;
	struct clk_hw *div_hw;
	struct clk_hw *gate_hw;

	mux_ops = NULL;
	div_ops = NULL;
	gate_ops = NULL;

	mux_hw = NULL;
	div_hw = NULL;
	gate_hw = NULL;

	if (cfg->mux_c && cfg->mux) {
		mux = _get_cmux(base + cfg->mux->offset,
				cfg->mux->shift,
				cfg->mux->width,
				cfg->mux_c->flags, lock);

		if (!IS_ERR(mux)) {
			mux_hw = &mux->hw;
			mux_ops = cfg->mux_c->ops ?
				  cfg->mux_c->ops : &clk_mux_s_ops;
		}
	}

	if (cfg->div_c && cfg->div) {
		div = _get_cdiv(base + cfg->div->offset,
				cfg->div->shift,
				cfg->div->width,
				cfg->div->table,
				cfg->div_c->flags, lock);

		if (!IS_ERR(div)) {
			div_hw = &div->hw;
			div_ops = cfg->div_c->ops ?
				  cfg->div_c->ops : &clk_divider_secure_ops;
		}
	}

	if (cfg->gate_c && cfg->gate) {
		gate = _get_stm32_cgate(base + cfg->gate->offset_set,
					base + cfg->gate->offset_clr,
					cfg->gate->bit_idx,
					cfg->gate_c->flags, lock);

		if (!IS_ERR(gate)) {
			gate_hw = &gate->hw;
			gate_ops = cfg->gate_c->ops ?
				   cfg->gate_c->ops : &stm32_gate_s_clk_ops;
		}
	}

	hw = clk_hw_register_composite(dev, name, parent_names, num_parents,
				       mux_hw, mux_ops,
				       div_hw, div_ops,
				       gate_hw, gate_ops,
				       flags);

	return hw;
}

int _is_soc_secured(void __iomem *base)
{
	u32 val;

	val = readl_relaxed(base);

	return val;
}

struct clk_hw *
__clk_hw_register_divider_table(struct device *dev,
				struct clk_hw_onecell_data *clk_data,
				void __iomem *base, spinlock_t *lock,
				const struct clock_config *cfg)
{
	struct divider_cfg *div_cfg = cfg->cfg;

	if (!_is_soc_secured(base))
		return clk_hw_register_divider_table(dev, cfg->name,
						     cfg->parent_name,
						     cfg->flags,
						     div_cfg->reg_off + base,
						     div_cfg->shift,
						     div_cfg->width,
						     div_cfg->div_flags,
						     div_cfg->table,
						     lock);
	else
		return _register_divider_sec(dev, cfg->name,
					     cfg->parent_name,
					     cfg->flags,
					     div_cfg->reg_off + base,
					     div_cfg->shift,
					     div_cfg->width,
					     div_cfg->div_flags,
					     div_cfg->table,
					     lock);
}

struct clk_hw *
__clk_register_stm32_gate(struct device *dev,
			  struct clk_hw_onecell_data *clk_data,
			  void __iomem *base, spinlock_t *lock,
			  const struct clock_config *cfg)
{
	if (!_is_soc_secured(base))
		return clk_register_stm32_gate(dev, cfg->name,
					       cfg->parent_name, base,
					       cfg->cfg, cfg->flags, lock);
	else
		return clk_register_stm32_sgate(dev, cfg->name,
						cfg->parent_name, base,
						cfg->cfg, cfg->flags, lock);
}

struct clk_hw *
__clk_stm32_register_composite(struct device *dev,
			       struct clk_hw_onecell_data *clk_data,
			       void __iomem *base,
			       spinlock_t *lock,
			       const struct clock_config *cfg)
{
	if (!_is_soc_secured(base))
		return clk_stm32_register_composite(dev, cfg->name,
						    cfg->parent_names,
						    cfg->num_parents, base,
						    cfg->cfg, cfg->flags,
						    lock);
	else
		return clk_stm32_register_composite_s(dev, cfg->name,
						      cfg->parent_names,
						      cfg->num_parents, base,
						      cfg->cfg,	cfg->flags,
						      lock);
}
