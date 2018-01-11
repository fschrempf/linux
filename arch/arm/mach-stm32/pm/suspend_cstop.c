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
#include "suspend.h"

struct cstop_private_data {
	u8 ddr_state;
	u32 ddr_timing;
	u8 ddr_type;
};

struct cstop_private_data cstop_data;

int stm32mp1_cstop_enter(struct stm32_hw_state_desc *state)
{
	unsigned long flag;
	struct cstop_private_data *cstopdata =
		((struct cstop_private_data *)state->state_private_data);

	pr_info("%s\n", __func__);

	local_irq_save(flag);

	if (!state)
		return -EINVAL;

	if (cstopdata->ddr_state == DDR_OFF)
		return -EINVAL;

	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_RSTCLRR, RCC_RESET_MASK);
	SMC(STM32_SVC_RCC, STM32_WRITE, RCC_CIFR, CIFR_CLR_MASK);
	SMC(STM32_SVC_PWR, STM32_SET_BITS, PWR_MPUCR, CSSF | CSTDBYDIS);
	SMC(STM32_SVC_PWR, STM32_CLR_BITS, PWR_MPUCR, PDDS);
	SMC(STM32_SVC_RCC, STM32_SET_BITS, RCC_SREQSETR, STPREQ);

	pr_info("stm32mp1 pm CSTOP: CPU Going to be Frozen\n");

	flush_cache_all();

	/*
	 * Call the suspend API
	 * Once a wakeup is received, execution is started just after this call
	 */
	SMC(STM32_SVC_CSTOP, cstopdata->ddr_type, cstopdata->ddr_timing, 0);

	pr_info("Out from CSTOP\n");

	/* Disable CSTOP */
	SMC(STM32_SVC_RCC, STM32_SET_BITS, RCC_SREQCLRR, STPREQ);
	SMC(STM32_SVC_PWR, STM32_CLR_BITS, PWR_MPUCR, CSTDBYDIS);

	pr_info("stm32mp1 pm: System woken up from CSTOP\n");

	local_irq_restore(flag);

	return 0;
}

int stm32mp1_cstop_setup(struct stm32_hw_state_desc *state,
			 struct regmap *regmap)
{
	cstop_data.ddr_state = state->ddr_state;
	cstop_data.ddr_timing = state->ddr_timing;
	cstop_data.ddr_type = state->ddr_type;
	state->state_private_data = &cstop_data;
	state->suspend_data.power_off = 0;

	return 0;
}
