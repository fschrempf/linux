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

#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "suspend.h"

static struct regmap *rcc_regmap;

struct cstandby_private_data {
	u8 ddr_state;
	u32 ddr_timing;
	u8 ddr_type;
};

struct cstandby_private_data cstandby_data;

int stm32mp1_secure_standby(unsigned long data)
{
	struct stm32mp1_suspend_data *lpdata =
		(struct stm32mp1_suspend_data *)data;

	SMC(STM32_SVC_STANDBY, lpdata->ddr_type, lpdata->ddr_timing, 0);

	return 0;
}

int stm32mp1_cstandby_enter(struct stm32_hw_state_desc *state)
{
	unsigned long flag;
	unsigned int val;
	struct cstandby_private_data *cstandbydata =
		((struct cstandby_private_data *)state->state_private_data);

	pr_info("%s\n", __func__);

	local_irq_save(flag);

	if (!state)
		return -EINVAL;

	if (cstandbydata->ddr_state != DDR_OFF &&
	    cstandbydata->ddr_state != DDR_SR)
		return -EINVAL;

	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_RSTCLRR, RCC_RESET_MASK);
	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_CIFR, CIFR_CLR_MASK);
	SMC(STM32_SVC_PWR, STM32_SET_BITS, PWR_MPUCR, CSSF | PDDS | CSTDBYDIS);
	SMC(STM32_SVC_RCC, STM32_SET_BITS, RCC_SREQSETR, STPREQ);

	pr_info("stm32mp1 CSTANDBY: CPU Going to be Frozen\n");

	flush_cache_all();

	/* Call the suspend framework API */
	cpu_suspend((long)(&state->suspend_data), stm32mp1_secure_standby);

	regmap_read(rcc_regmap, RCC_RSTSR, &val);

	if (!(val & STDBY_FLAG))
		pr_info("stm32mp1: Out from CSTOP\n");
	else
		pr_info("stm32mp1: Out from STANDBY\n");

	SMC(STM32_SVC_RCC, STM32_SET_BITS, RCC_SREQCLRR, STPREQ);

	/* First check whether we are running on the correct cpu */
	if (smp_processor_id())
		pr_err("stm32mp1 CSTANDBY: Error: Running on the wrong CPU\n");

	/* Resume the outer cache */
	outer_resume();

	local_irq_restore(flag);

	return 0;
}

int stm32mp1_cstandby_setup(struct stm32_hw_state_desc *state,
			    struct regmap *regmap)
{
	rcc_regmap = regmap;
	cstandby_data.ddr_state = state->ddr_state;
	state->state_private_data = &cstandby_data;
	state->suspend_data.power_off = 0;
	state->suspend_data.ddr_type = state->ddr_type;
	state->suspend_data.ddr_timing = state->ddr_timing;

	return 0;
}
