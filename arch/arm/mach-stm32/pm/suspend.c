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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/syscon.h>

#include "suspend.h"

struct stm32_hw_state_desc *stm32_hw_state;

/*
 * Currently three low power suspend modes
 * are supported: freeze, standby and mem.
 * freeze support is in-build in kernel and no
 * support from platform power code is required.
 * standby and mem require platform code support.
 * They require explicit callback support.
 * It is to be noted that standby is mapped to (C)STOP
 * and mem is mapped to SoC STANDBY.
 */

#define PM_SHUTDOWN	(PM_SUSPEND_MAX + 1)
#define PM_STM32_MAX	(PM_SHUTDOWN + 1)

struct stm32_soc_mode stm32mp1_soc_modes[MAX_SOC_MODE] = {
	[STOP] = {
		.desc = "STOP",
		.setup = stm32mp1_cstop_setup,
		.enter = stm32mp1_cstop_enter,
	},
	[STANDBY] = {
		.desc = "STANDBY",
		.setup = stm32mp1_cstandby_setup,
		.enter = stm32mp1_cstandby_enter,
	},
	[SHUTDOWN] = {
		.desc = "SHUTDOWN",
		.setup = stm32mp1_shutdown_setup,
	},
};

struct stm32_hw_state_desc stm32mp1_hw_states[PM_STM32_MAX] = {
	[PM_SUSPEND_STANDBY] = {
		.valid = true,
		.stm32_soc_modes = stm32mp1_soc_modes,
		.soc_state = STOP,
		.ddr_state = DDR_SR,
		.reg_state = PM_SUSPEND_STANDBY,
	},
	[PM_SUSPEND_MEM] = {
		.valid = true,
		.stm32_soc_modes = stm32mp1_soc_modes,
		.soc_state = STANDBY,
		.ddr_state = DDR_SR,
		.reg_state = PM_SUSPEND_MEM,
	},
	[PM_SHUTDOWN] = {
		.valid = true,
		.stm32_soc_modes = stm32mp1_soc_modes,
		.soc_state = SHUTDOWN,
		.ddr_state = DDR_OFF,
		.reg_state = PM_SUSPEND_MEM,
	},
};

static const struct of_device_id of_machine_table[] = {
	{
		.compatible = "st,stm32mp157",
		.data = (void *)&stm32mp1_hw_states,
	},
	{ /* end of list */ },
};

int stm32mp1_shutdown_setup(struct stm32_hw_state_desc *state,
			    struct regmap *regmap)
{
	/* Erase return address in backup register */

	return 0;
}

static int stm32_suspend_valid(suspend_state_t state)
{
	return stm32_hw_state[state].valid;
}

static int stm32_suspend_begin(suspend_state_t state)
{
	int ret;

	if (!stm32_hw_state[state].valid)
		return -EINVAL;

	ret = regulator_suspend_prepare(stm32_hw_state[state].reg_state);
	if (ret) {
		pr_err("Failed to prepare regulators for suspend (%d)\n", ret);
		return ret;
	}
	return 0;
}

static int stm32_suspend_enter(suspend_state_t state)
{
	int soc_state;
	struct stm32_soc_mode *soc_mode;
	int (*enter)(struct stm32_hw_state_desc *state);

	if (!stm32_hw_state[state].valid)
		return -EINVAL;

	soc_mode =  stm32_hw_state[state].stm32_soc_modes;

	pr_info("stm32mp1 pm: entering suspend:%s\n", soc_mode->desc);

	soc_state = stm32_hw_state[state].soc_state;

	enter = soc_mode[soc_state].enter;

	if (!enter) {
		pr_err("stm32mp1 pm: state enter() is NULL\n");
		return -EFAULT;
	}

	return enter(&stm32_hw_state[state]);
}

static const struct of_device_id
	*of_match_machine(const struct of_device_id *matches)
{
	const struct of_device_id *match;
	struct device_node *np;

	np = of_find_matching_node_and_match(NULL, matches, &match);
	if (np && of_device_is_available(np))
		return match;

	return NULL;
}

static const char *const ddr_states[PM_SUSPEND_MAX + 1] = {
	[PM_SUSPEND_STANDBY]	= "standby_state",
	[PM_SUSPEND_MEM]	= "suspend_to_ram_state",
};

static const struct platform_suspend_ops stm32_suspend_ops = {
	.valid		= stm32_suspend_valid,
	.begin		= stm32_suspend_begin,
	.enter		= stm32_suspend_enter,
};

void stm32mp1_do_shutdown(void)
{
	/* Request M4 to enter cstop and put system in standby */

	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_RSTCLRR, RCC_RESET_MASK);
	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_CIFR, CIFR_CLR_MASK);
	SMC(STM32_SVC_PWR, STM32_SET_BITS, PWR_MPUCR, CSSF | PDDS);
	SMC(STM32_SVC_RCC, STM32_SET_BITS, RCC_SREQSETR, STPREQ);

	stm32mp1_suspend_exec(0);
}

static int __init stm32_suspend_setup(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	int i, ret = 0;
	u32 tdllrst_cycles;
	u32 tdlllock_cycles;
	int timing_reg;
	struct regmap *rcc_regmap;
	struct clk *clk;
	u32 rate_khz;

	match = of_match_machine(of_machine_table);
	if (!match) {
		pr_err("%s: match data not found\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	np = of_find_compatible_node(NULL, NULL, "st,stm32-pwr");
	if (!np) {
		pr_err("%s: failed to find pwr DT node\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	stm32_hw_state = (struct stm32_hw_state_desc *)match->data;

	clk = of_clk_get_by_name(np, "phyclk");
	if (IS_ERR(clk)) {
		pr_err("stm32mp1: could not find phyclk clock\n");
		return PTR_ERR(clk);
	}

	rate_khz = clk_get_rate(clk) / 1000;

	/* 50 ns for TDLLRST */
	tdllrst_cycles = DIV_ROUND_UP(50 * rate_khz, 1000000);

	/* 5.12 µS for TDLLLOCK */
	tdlllock_cycles = DIV_ROUND_UP(512 * rate_khz, 100000);

	/* Format timing register */
	timing_reg = (tdllrst_cycles | (tdlllock_cycles << TDLL_SHIFT) |
		     (TITMRST_MASK << TITMRST_SHIFT));

	rcc_regmap = syscon_regmap_lookup_by_phandle(np, "st,sysrcc");
	if (IS_ERR(rcc_regmap)) {
		pr_err("stm32mp1: STANDBY: could not find rcc regmap\n");
		return PTR_ERR(rcc_regmap);
	}

	for (i = 0; i < PM_STM32_MAX; i++) {
		struct device_node *suspend_np;
		struct stm32_hw_state_desc *hw_state;
		int (*setup)(struct stm32_hw_state_desc *, struct regmap *);

		hw_state = &stm32_hw_state[i];

		if (!hw_state->valid)
			continue;

		suspend_np = of_get_child_by_name(np, ddr_states[i]);

		if (suspend_np) {
			of_property_read_u32(suspend_np, "ddr_type",
					     &hw_state->ddr_type);
			of_property_read_u32(suspend_np, "ddr_mode",
					     &hw_state->ddr_state);
			of_property_read_u32(suspend_np, "regulator_mode",
					     &hw_state->reg_state);
			of_property_read_u32(suspend_np, "soc_power_mode",
					     &hw_state->soc_state);
		}

		hw_state->ddr_timing = timing_reg;

		setup  = hw_state->stm32_soc_modes[hw_state->soc_state].setup;

		ret = setup(hw_state, rcc_regmap);

		if (ret)
			continue;
	}

	suspend_set_ops(&stm32_suspend_ops);

	pm_power_off = stm32mp1_do_shutdown;

	return ret;
err:
	pr_err("stm32mp1 pm: Suspend support could not be registered\n");
	return ret;
}

late_initcall(stm32_suspend_setup);
