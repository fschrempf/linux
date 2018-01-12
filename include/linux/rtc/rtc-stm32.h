/*
 * Copyright (C) STMicroelectronics SA 2017
 * Author:  Amelie Delaunay <amelie.delaunay@st.com>
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __STM32_RTC_H
#define __STM32_RTC_H

#include <linux/regmap.h>
#include <linux/rtc.h>

/* STM32_RTC_TR bit fields  */
#define STM32_RTC_TR_SEC_SHIFT		0
#define STM32_RTC_TR_SEC		GENMASK(6, 0)
#define STM32_RTC_TR_MIN_SHIFT		8
#define STM32_RTC_TR_MIN		GENMASK(14, 8)
#define STM32_RTC_TR_HOUR_SHIFT		16
#define STM32_RTC_TR_HOUR		GENMASK(21, 16)

/* STM32_RTC_DR bit fields */
#define STM32_RTC_DR_DATE_SHIFT		0
#define STM32_RTC_DR_DATE		GENMASK(5, 0)
#define STM32_RTC_DR_MONTH_SHIFT	8
#define STM32_RTC_DR_MONTH		GENMASK(12, 8)
#define STM32_RTC_DR_WDAY_SHIFT		13
#define STM32_RTC_DR_WDAY		GENMASK(15, 13)
#define STM32_RTC_DR_YEAR_SHIFT		16
#define STM32_RTC_DR_YEAR		GENMASK(23, 16)

/* STM32_RTC_CR bit fields */
#define STM32_RTC_CR_FMT		BIT(6)
#define STM32_RTC_CR_ALRAE		BIT(8)
#define STM32_RTC_CR_ALRAIE		BIT(12)

/* STM32_RTC_ISR/STM32_RTC_ICSR bit fields */
#define STM32_RTC_ISR_ALRAWF		BIT(0)
#define STM32_RTC_ISR_INITS		BIT(4)
#define STM32_RTC_ISR_RSF		BIT(5)
#define STM32_RTC_ISR_INITF		BIT(6)
#define STM32_RTC_ISR_INIT		BIT(7)
#define STM32_RTC_ISR_ALRAF		BIT(8)

/* STM32_RTC_SR/_SCR bit fields */
#define STM32_RTC_SR_ALRA		BIT(0)

/* STM32_RTC_PRER bit fields */
#define STM32_RTC_PRER_PRED_S_SHIFT	0
#define STM32_RTC_PRER_PRED_S		GENMASK(14, 0)
#define STM32_RTC_PRER_PRED_A_SHIFT	16
#define STM32_RTC_PRER_PRED_A		GENMASK(22, 16)

/* STM32_RTC_ALRMAR and STM32_RTC_ALRMBR bit fields */
#define STM32_RTC_ALRMXR_SEC_SHIFT	0
#define STM32_RTC_ALRMXR_SEC		GENMASK(6, 0)
#define STM32_RTC_ALRMXR_SEC_MASK	BIT(7)
#define STM32_RTC_ALRMXR_MIN_SHIFT	8
#define STM32_RTC_ALRMXR_MIN		GENMASK(14, 8)
#define STM32_RTC_ALRMXR_MIN_MASK	BIT(15)
#define STM32_RTC_ALRMXR_HOUR_SHIFT	16
#define STM32_RTC_ALRMXR_HOUR		GENMASK(21, 16)
#define STM32_RTC_ALRMXR_PM		BIT(22)
#define STM32_RTC_ALRMXR_HOUR_MASK	BIT(23)
#define STM32_RTC_ALRMXR_DATE_SHIFT	24
#define STM32_RTC_ALRMXR_DATE		GENMASK(29, 24)
#define STM32_RTC_ALRMXR_WDSEL		BIT(30)
#define STM32_RTC_ALRMXR_WDAY_SHIFT	24
#define STM32_RTC_ALRMXR_WDAY		GENMASK(27, 24)
#define STM32_RTC_ALRMXR_DATE_MASK	BIT(31)

/* STM32_RTC_VERR bit fields */
#define STM32_RTC_VERR_MINREV_SHIFT	0
#define STM32_RTC_VERR_MINREV		GENMASK(3, 0)
#define STM32_RTC_VERR_MAJREV_SHIFT	4
#define STM32_RTC_VERR_MAJREV		GENMASK(7, 4)

/* STM32_RTC_WPR key constants */
#define RTC_WPR_1ST_KEY			0xCA
#define RTC_WPR_2ND_KEY			0x53
#define RTC_WPR_WRONG_KEY		0xFF

struct stm32_rtc;

struct stm32_rtc_registers {
	u8 tr;
	u8 dr;
	u8 cr;
	u8 isr;
	u8 prer;
	u8 alrmar;
	u8 wpr;
	u8 sr;
	u8 scr;
	u16 hwcfgr;
	u16 verr;
};

struct stm32_rtc_events {
	u32 alra;
};

struct stm32_rtc_data {
	const struct stm32_rtc_registers regs;
	const struct stm32_rtc_events events;
	void (*clear_events)(struct stm32_rtc *, unsigned int flags);
	bool has_pclk;
};

struct stm32_rtc {
	struct rtc_device *rtc_dev;
	void __iomem *base;
	struct regmap *dbp;
	unsigned int dbp_reg;
	unsigned int dbp_mask;
	struct clk *pclk;
	struct clk *rtc_ck;
	const struct stm32_rtc_data *data;
	int irq_alarm;
	int wakeirq_alarm;
};

#define UNDEF_REG ~0

static inline bool stm32_rtc_reg_is_defined(unsigned int offset)
{
	if ((offset & UNDEF_REG) == offset)
		return true;
	else
		return false;
}
#endif /* __STM32_RTC_H */
