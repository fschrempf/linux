/*
 * Copyright (C) STMicroelectronics SA 2017
 * Author: Ludovic Barre <ludovic.barre@st.com>
 * License terms: GNU General Public License (GPL), version 2
 */
#include <asm/mach/arch.h>
#include <linux/of_platform.h>

static const char *const stm32mp_compat[] __initconst = {
	"st,stm32mp157",
	NULL
};

DT_MACHINE_START(STM32MPDT, "STM32 MP (Device Tree Support)")
	.dt_compat = stm32mp_compat,
MACHINE_END
