// SPDX-License-Identifier: GPL-2.0+
/*
 * OWL SoC's Pinctrl driver
 *
 * Copyright (c) 2014 Actions Semi Inc.
 * Author: David Liu <liuwei@actions-semi.com>
 *
 * Copyright (c) 2018 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "../core.h"
#include "../pinctrl-utils.h"
#include "pinctrl-owl.h"

/**
 * struct owl_pinctrl - pinctrl state of the device
 * @dev: device handle
 * @pctrldev: pinctrl handle
 * @lock: spinlock to protect registers
 * @soc: reference to soc_data
 * @base: pinctrl register base address
 */
struct owl_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctrldev;
	raw_spinlock_t lock;
	struct clk *clk;
	const struct owl_pinctrl_soc_data *soc;
	void __iomem *base;
};

static void owl_update_bits(void __iomem *base, u32 mask, u32 val)
{
	u32 reg_val;

	reg_val = readl_relaxed(base);

	reg_val = (reg_val & ~mask) | (val & mask);

	writel_relaxed(reg_val, base);
}

static u32 owl_read_field(struct owl_pinctrl *pctrl, u32 reg,
				u32 bit, u32 width)
{
	u32 tmp, mask;

	tmp = readl_relaxed(pctrl->base + reg);
	mask = (1 << width) - 1;

	return (tmp >> bit) & mask;
}

static void owl_write_field(struct owl_pinctrl *pctrl, u32 reg, u32 arg,
				u32 bit, u32 width)
{
	u32 mask;

	mask = (1 << width) - 1;
	mask = mask << bit;

	owl_update_bits(pctrl->base + reg, mask, (arg << bit));
}

static int owl_get_groups_count(struct pinctrl_dev *pctrldev)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	return pctrl->soc->ngroups;
}

static const char *owl_get_group_name(struct pinctrl_dev *pctrldev,
				unsigned int group)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	return pctrl->soc->groups[group].name;
}

static int owl_get_group_pins(struct pinctrl_dev *pctrldev,
				unsigned int group,
				const unsigned int **pins,
				unsigned int *num_pins)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	*pins = pctrl->soc->groups[group].pads;
	*num_pins = pctrl->soc->groups[group].npads;

	return 0;
}

static void owl_pin_dbg_show(struct pinctrl_dev *pctrldev,
				struct seq_file *s,
				unsigned int offset)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	seq_printf(s, "%s", dev_name(pctrl->dev));
}

static struct pinctrl_ops owl_pinctrl_ops = {
	.get_groups_count = owl_get_groups_count,
	.get_group_name = owl_get_group_name,
	.get_group_pins = owl_get_group_pins,
	.pin_dbg_show = owl_pin_dbg_show,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
};

static int owl_get_funcs_count(struct pinctrl_dev *pctrldev)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	return pctrl->soc->nfunctions;
}

static const char *owl_get_func_name(struct pinctrl_dev *pctrldev,
				unsigned int function)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	return pctrl->soc->functions[function].name;
}

static int owl_get_func_groups(struct pinctrl_dev *pctrldev,
				unsigned int function,
				const char * const **groups,
				unsigned int * const num_groups)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);

	*groups = pctrl->soc->functions[function].groups;
	*num_groups = pctrl->soc->functions[function].ngroups;

	return 0;
}

static inline int get_group_mfp_mask_val(const struct owl_pingroup *g,
				int function,
				u32 *mask,
				u32 *val)
{
	int id;
	u32 option_num;
	u32 option_mask;

	for (id = 0; id < g->nfuncs; id++) {
		if (g->funcs[id] == function)
			break;
	}
	if (WARN_ON(id == g->nfuncs))
		return -EINVAL;

	option_num = (1 << g->mfpctl_width);
	if (id > option_num)
		id -= option_num;

	option_mask = option_num - 1;
	*mask = (option_mask  << g->mfpctl_shift);
	*val = (id << g->mfpctl_shift);

	return 0;
}

static int owl_set_mux(struct pinctrl_dev *pctrldev,
				unsigned int function,
				unsigned int group)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);
	const struct owl_pingroup *g;
	unsigned long flags;
	u32 val, mask;

	g = &pctrl->soc->groups[group];

	if (get_group_mfp_mask_val(g, function, &mask, &val))
		return -EINVAL;

	raw_spin_lock_irqsave(&pctrl->lock, flags);

	owl_update_bits(pctrl->base + g->mfpctl_reg, mask, val);

	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	return 0;
}

static struct pinmux_ops owl_pinmux_ops = {
	.get_functions_count = owl_get_funcs_count,
	.get_function_name = owl_get_func_name,
	.get_function_groups = owl_get_func_groups,
	.set_mux = owl_set_mux,
};

static int owl_pad_pinconf_reg(const struct owl_padinfo *info,
				unsigned int param,
				u32 *reg,
				u32 *bit,
				u32 *width)
{
	switch (param) {
	case PIN_CONFIG_BIAS_BUS_HOLD:
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!info->pullctl)
			return -EINVAL;
		*reg = info->pullctl->reg;
		*bit = info->pullctl->shift;
		*width = info->pullctl->width;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		if (!info->st)
			return -EINVAL;
		*reg = info->st->reg;
		*bit = info->st->shift;
		*width = info->st->width;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int owl_pad_pinconf_arg2val(const struct owl_padinfo *info,
				unsigned int param,
				u32 *arg)
{
	switch (param) {
	case PIN_CONFIG_BIAS_BUS_HOLD:
		*arg = OWL_PINCONF_PULL_HOLD;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		*arg = OWL_PINCONF_PULL_HIZ;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		*arg = OWL_PINCONF_PULL_DOWN;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		*arg = OWL_PINCONF_PULL_UP;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		*arg = (*arg >= 1 ? 1 : 0);
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int owl_pad_pinconf_val2arg(const struct owl_padinfo *padinfo,
				unsigned int param,
				u32 *arg)
{
	switch (param) {
	case PIN_CONFIG_BIAS_BUS_HOLD:
		*arg = *arg == OWL_PINCONF_PULL_HOLD;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		*arg = *arg == OWL_PINCONF_PULL_HIZ;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		*arg = *arg == OWL_PINCONF_PULL_DOWN;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		*arg = *arg == OWL_PINCONF_PULL_UP;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		*arg = *arg == 1;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int owl_pin_config_get(struct pinctrl_dev *pctrldev,
				unsigned int pin,
				unsigned long *config)
{
	int ret = 0;
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);
	const struct owl_padinfo *info;
	unsigned int param = pinconf_to_config_param(*config);
	u32 reg, bit, width, arg;

	info = &pctrl->soc->padinfo[pin];

	ret = owl_pad_pinconf_reg(info, param, &reg, &bit, &width);
	if (ret)
		return ret;

	arg = owl_read_field(pctrl, reg, bit, width);

	ret = owl_pad_pinconf_val2arg(info, param, &arg);
	if (ret)
		return ret;

	*config = pinconf_to_config_packed(param, arg);

	return ret;
}

static int owl_pin_config_set(struct pinctrl_dev *pctrldev,
				unsigned int pin,
				unsigned long *configs,
				unsigned int num_configs)
{
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);
	const struct owl_padinfo *info;
	unsigned long flags;
	unsigned int param;
	u32 reg, bit, width, arg;
	int ret, i;

	info = &pctrl->soc->padinfo[pin];

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		ret = owl_pad_pinconf_reg(info, param, &reg, &bit, &width);
		if (ret)
			return ret;

		ret = owl_pad_pinconf_arg2val(info, param, &arg);
		if (ret)
			return ret;

		raw_spin_lock_irqsave(&pctrl->lock, flags);

		owl_write_field(pctrl, reg, arg, bit, width);

		raw_spin_unlock_irqrestore(&pctrl->lock, flags);
	}

	return ret;
}

static int owl_group_pinconf_reg(const struct owl_pingroup *g,
				unsigned int param,
				u32 *reg,
				u32 *bit,
				u32 *width)
{
	switch (param) {
	case PIN_CONFIG_DRIVE_STRENGTH:
		if (g->drv_reg < 0)
			return -EINVAL;
		*reg = g->drv_reg;
		*bit = g->drv_shift;
		*width = g->drv_width;
		break;
	case PIN_CONFIG_SLEW_RATE:
		if (g->sr_reg < 0)
			return -EINVAL;
		*reg = g->sr_reg;
		*bit = g->sr_shift;
		*width = g->sr_width;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int owl_group_pinconf_arg2val(const struct owl_pingroup *g,
				unsigned int param,
				u32 *arg)
{
	switch (param) {
	case PIN_CONFIG_DRIVE_STRENGTH:
		switch (*arg) {
		case 2:
			*arg = OWL_PINCONF_DRV_2MA;
			break;
		case 4:
			*arg = OWL_PINCONF_DRV_4MA;
			break;
		case 8:
			*arg = OWL_PINCONF_DRV_8MA;
			break;
		case 12:
			*arg = OWL_PINCONF_DRV_12MA;
			break;
		default:
			return -EINVAL;
		}
		break;
	case PIN_CONFIG_SLEW_RATE:
		if (*arg)
			*arg = OWL_PINCONF_SLEW_FAST;
		else
			*arg = OWL_PINCONF_SLEW_SLOW;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int owl_group_pinconf_val2arg(const struct owl_pingroup *g,
				unsigned int param,
				u32 *arg)
{
	switch (param) {
	case PIN_CONFIG_DRIVE_STRENGTH:
		switch (*arg) {
		case OWL_PINCONF_DRV_2MA:
			*arg = 2;
			break;
		case OWL_PINCONF_DRV_4MA:
			*arg = 4;
			break;
		case OWL_PINCONF_DRV_8MA:
			*arg = 8;
			break;
		case OWL_PINCONF_DRV_12MA:
			*arg = 12;
			break;
		default:
			return -EINVAL;
		}
		break;
	case PIN_CONFIG_SLEW_RATE:
		if (*arg)
			*arg = 1;
		else
			*arg = 0;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int owl_group_config_get(struct pinctrl_dev *pctrldev,
				unsigned int group,
				unsigned long *config)
{
	const struct owl_pingroup *g;
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);
	unsigned int param = pinconf_to_config_param(*config);
	u32 reg, bit, width, arg;
	int ret;

	g = &pctrl->soc->groups[group];

	ret = owl_group_pinconf_reg(g, param, &reg, &bit, &width);
	if (ret)
		return ret;

	arg = owl_read_field(pctrl, reg, bit, width);

	ret = owl_group_pinconf_val2arg(g, param, &arg);
	if (ret)
		return ret;

	*config = pinconf_to_config_packed(param, arg);

	return ret;

}

static int owl_group_config_set(struct pinctrl_dev *pctrldev,
				unsigned int group,
				unsigned long *configs,
				unsigned int num_configs)
{
	const struct owl_pingroup *g;
	struct owl_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctrldev);
	unsigned long flags;
	unsigned int param;
	u32 reg, bit, width, arg;
	int ret, i;

	g = &pctrl->soc->groups[group];

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		ret = owl_group_pinconf_reg(g, param, &reg, &bit, &width);
		if (ret)
			return ret;

		ret = owl_group_pinconf_arg2val(g, param, &arg);
		if (ret)
			return ret;

		/* Update register */
		raw_spin_lock_irqsave(&pctrl->lock, flags);

		owl_write_field(pctrl, reg, arg, bit, width);

		raw_spin_unlock_irqrestore(&pctrl->lock, flags);
	}

	return 0;
}

static const struct pinconf_ops owl_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = owl_pin_config_get,
	.pin_config_set = owl_pin_config_set,
	.pin_config_group_get = owl_group_config_get,
	.pin_config_group_set = owl_group_config_set,
};

static struct pinctrl_desc owl_pinctrl_desc = {
	.pctlops = &owl_pinctrl_ops,
	.pmxops = &owl_pinmux_ops,
	.confops = &owl_pinconf_ops,
	.owner = THIS_MODULE,
};

int owl_pinctrl_probe(struct platform_device *pdev,
				struct owl_pinctrl_soc_data *soc_data)
{
	struct resource *res;
	struct owl_pinctrl *pctrl;
	int ret;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pctrl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctrl->base))
		return PTR_ERR(pctrl->base);

	/* enable GPIO/MFP clock */
	pctrl->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pctrl->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return PTR_ERR(pctrl->clk);
	}

	ret = clk_prepare_enable(pctrl->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable failed\n");
		return ret;
	}

	raw_spin_lock_init(&pctrl->lock);

	owl_pinctrl_desc.name = dev_name(&pdev->dev);
	owl_pinctrl_desc.pins = soc_data->pins;
	owl_pinctrl_desc.npins = soc_data->npins;

	pctrl->soc = soc_data;
	pctrl->dev = &pdev->dev;

	pctrl->pctrldev = devm_pinctrl_register(&pdev->dev,
					&owl_pinctrl_desc, pctrl);
	if (IS_ERR(pctrl->pctrldev)) {
		dev_err(&pdev->dev, "could not register Actions OWL pinmux driver\n");
		return PTR_ERR(pctrl->pctrldev);
	}

	platform_set_drvdata(pdev, pctrl);

	return 0;
}
