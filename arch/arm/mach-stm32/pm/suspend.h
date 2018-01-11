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

#ifndef __SUSPEND_H__
#define __SUSPEND_H__

#include <linux/arm-smccc.h>
#include <linux/suspend.h>

#define STM32MP1_SUSPEND_DESC_LEN 32
#define MAX_SOC_LOW_POWER_STATES 2

#define STM32_SVC_RCC		0x82001000
#define STM32_SVC_PWR		0x82001001
#define STM32_SVC_SR_MODE	0x82001001
#define STM32_SVC_CSTOP		0x82001005
#define STM32_SVC_STANDBY	0x82001006
#define STM32_WRITE		0x1
#define STM32_SET_BITS		0x2
#define STM32_CLR_BITS		0x3
#define STM32_SR_MODE_SSR	0x0
#define STM32_SR_MODE_ASR	0x1
#define STM32_SR_MODE_HSR	0x2
#define PWR_MPUCR		0x50001010
#define RCC_SREQSETR		0x50000104
#define RCC_SREQCLRR		0x50000108
#define RCC_RSTCLRR		0x50000400
#define RCC_CIFR		0x50000418
#define RCC_RESET_MASK		0x3DF
#define CIFR_CLR_MASK		0x110F1F
#define PDDS			BIT(0)
#define CSTDBYDIS		BIT(3)
#define CSSF			BIT(9)
#define STDBY_FLAG		BIT(11)
#define STPREQ			GENMASK(1, 0)
#define RCC_RSTSR		0x408
#define TDLL_SHIFT		0x6
#define TITMRST_MASK		0x8
#define TITMRST_SHIFT		18

#define SMC(serv, op, address, val)		\
{						\
	struct arm_smccc_res res;		\
	arm_smccc_smc(serv, op, address, val,	\
		      0, 0, 0, 0, &res);	\
}						\

/*  DDR possible states */
enum ddr_low_power_state {
	DDR_SR,
	DDR_OFF
};

enum soc_mode {
	STOP,
	STANDBY,
	LPSTOP,
	SHUTDOWN,
	MAX_SOC_MODE
};

/*
 * struct suspend_data is used by low power
 * entry code.
 */
struct stm32mp1_suspend_data {
	unsigned int power_off;
	u32 ddr_timing;
	u8 ddr_type;
};

struct stm32_soc_mode;

/*
 * struct stm32_hw_state_desc is the structure representing the
 * low power states.
 */
struct stm32_hw_state_desc {
	bool valid;
	enum soc_mode soc_state;
	enum ddr_low_power_state ddr_state;
	int ddr_timing;
	int ddr_type;
	suspend_state_t reg_state;
	struct stm32mp1_suspend_data suspend_data;
	struct stm32_soc_mode *stm32_soc_modes;
	void *state_private_data;
	struct regmap *rcc_regmap;
};

struct stm32_soc_mode {
	char desc[STM32MP1_SUSPEND_DESC_LEN];
	int (*setup)(struct stm32_hw_state_desc *, struct regmap *);
	int (*enter)(struct stm32_hw_state_desc *);
};

/* declaration of the entry functions for CSTOP mode */
int stm32mp1_cstop_setup(struct stm32_hw_state_desc *state,
			 struct regmap *regmap);
int stm32mp1_cstop_enter(struct stm32_hw_state_desc *state);

/* declaration of the entry functions for CSTANDBY mode */
int stm32mp1_cstandby_setup(struct stm32_hw_state_desc *state,
			    struct regmap *regmap);
int stm32mp1_cstandby_enter(struct stm32_hw_state_desc *state);

/* declaration of the entry functions for SHUTDOWN mode */
int stm32mp1_shutdown_setup(struct stm32_hw_state_desc *state,
			    struct regmap *regmap);
int stm32mp1_shutdown_enter(struct stm32_hw_state_desc *state);

int stm32mp1_shutdown(void);

/* specific call declarations */
int stm32mp1_suspend_exec(unsigned long data);

int stm32mp1_suspend_on_eram(void);
extern unsigned long stm32mp1_suspend_on_eram_sz;

/* Function executed after a wakeup on primary core with DDR preserved */
void stm32mp1_defrost_kernel(void);
#endif
