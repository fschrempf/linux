/*
 * This file is part of stpmu1 pmic driver
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author: Philippe Peurichard <philippe.peurichard@st.com>,
 * Pascal Paillet <p.paillet@st.com> for STMicroelectronics.
 *
 * License type: GPLv2
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_MFD_STPMU1_H
#define __LINUX_MFD_STPMU1_H

#define STPMU1_PMIC_NUM_IRQ_REGS	4

#define TURN_ON_REG		0x1
#define TURN_OFF_REG		0x2
#define ICC_LDO_TURN_OFF_REG	0x3
#define ICC_BUCK_TURN_OFF_REG	0x4
#define RESET_STATUS_REG	0x5
#define VERSION_STATUS_REG	0x6

#define MAIN_CONTROL_REG	0x10
#define PADS_PULL_REG		0x11
#define BUCK_PULL_DOWN_REG	0x12
#define LDO14_PULL_DOWN_REG	0x13
#define LDO56_PULL_DOWN_REG	0x14
#define VIN_CONTROL_REG		0x15
#define PONKEY_TURNOFF_REG	0x16
#define MASK_RANK_BUCK_REG	0x17
#define MASK_RESET_BUCK_REG	0x18
#define MASK_RANK_LDO_REG	0x19
#define MASK_RESET_LDO_REG	0x1A
#define WATCHDOG_CONTROL_REG	0x1B
#define WATCHDOG_TIMER_REG	0x1C
#define BUCK_ICC_TURNOFF_REG	0x1D
#define LDO_ICC_TURNOFF_REG     0x1E
#define BUCK_APM_CONTROL_REG    0x1F

#define BUCK1_CONTROL_REG	0x20
#define BUCK2_CONTROL_REG	0x21
#define BUCK3_CONTROL_REG	0x22
#define BUCK4_CONTROL_REG	0x23
#define VREF_DDR_CONTROL_REG	0x24
#define LDO1_CONTROL_REG	0x25
#define LDO2_CONTROL_REG	0x26
#define LDO3_CONTROL_REG	0x27
#define LDO4_CONTROL_REG	0x28
#define LDO5_CONTROL_REG	0x29
#define LDO6_CONTROL_REG	0x2A

#define BUCK1_PWRCTRL_REG	0x30
#define BUCK2_PWRCTRL_REG	0x31
#define BUCK3_PWRCTRL_REG	0x32
#define BUCK4_PWRCTRL_REG	0x33
#define VREF_DDR_PWRCTRL_REG	0x34
#define LDO1_PWRCTRL_REG	0x35
#define LDO2_PWRCTRL_REG	0x36
#define LDO3_PWRCTRL_REG	0x37
#define LDO4_PWRCTRL_REG	0x38
#define LDO5_PWRCTRL_REG	0x39
#define LDO6_PWRCTRL_REG	0x3A
#define FREQUENCY_SPREADING_REG 0x3B

#define USB_CONTROL_REG		0x40

#define ITLATCH1_REG		0x50
#define ITLATCH2_REG		0x51
#define ITLATCH3_REG		0x52
#define ITLATCH4_REG		0x53

#define ITSETLATCH1_REG		0x60
#define ITSETLATCH2_REG		0x61
#define ITSETLATCH3_REG		0x62
#define ITSETLATCH4_REG		0x63

#define ITCLEARLATCH1_REG	0x70
#define ITCLEARLATCH2_REG	0x71
#define ITCLEARLATCH3_REG	0x72
#define ITCLEARLATCH4_REG	0x73

#define ITMASK1_REG		0x80
#define ITMASK2_REG		0x81
#define ITMASK3_REG		0x82
#define ITMASK4_REG		0x83

#define ITSETMASK1_REG		0x90
#define ITSETMASK2_REG		0x91
#define ITSETMASK3_REG		0x92
#define ITSETMASK4_REG		0x93

#define ITCLEARMASK1_REG	0xA0
#define ITCLEARMASK2_REG	0xA1
#define ITCLEARMASK3_REG	0xA2
#define ITCLEARMASK4_REG	0xA3

#define ITSOURCE1_REG		0xB0
#define ITSOURCE2_REG		0xB1
#define ITSOURCE3_REG		0xB2
#define ITSOURCE4_REG		0xB3

#define NVM_SECTOR3_0		0xF8
#define NVM_SECTOR3_1		0xF9
#define NVM_SECTOR3_2		0xFA
#define NVM_SECTOR3_3		0xFB
#define NVM_SECTOR3_4		0xFC
#define NVM_SECTOR3_5		0xFD
#define NVM_SECTOR3_6		0xFE
#define NVM_SECTOR3_7		0xFF

#define PMIC_MAX_REGISTER_ADDRESS NVM_SECTOR3_7

#define TURN_OFF_REG_ICC_EVENT	0x08

#define LDO_VOLTAGE_MASK		GENMASK(6, 2)
#define BUCK_VOLTAGE_MASK		GENMASK(7, 2)
#define LDO_BUCK_VOLTAGE_SHIFT		2

#define LDO_ENABLE_MASK			BIT(0)
#define BUCK_ENABLE_MASK		BIT(0)

#define BUCK_HPLP_ENABLE_MASK		BIT(1)
#define BUCK_HPLP_SHIFT			1

#define BUCK_LDO_LOW_POWER_ENABLE_MASK  BIT(0)

#define BUCK_PULL_DOWN_REGISTER_MASK	GENMASK(7, 0)
#define BUCK_MASK_RANK_REGISTER_MASK	GENMASK(3, 0)
#define BUCK_MASK_RESET_REGISTER_MASK	GENMASK(3, 0)
#define LDO1234_PULL_DOWN_REGISTER_MASK	GENMASK(7, 0)
#define LDO56_PULL_DOWN_REGISTER_MASK	GENMASK(5, 0)
#define LDO_MASK_RANK_REGISTER_MASK	GENMASK(5, 0)
#define LDO_MASK_RESET_REGISTER_MASK	GENMASK(5, 0)

#define BUCK1_PULL_DOWN_REG		BUCK_PULL_DOWN_REG
#define BUCK1_PULL_DOWN_MASK		BIT(0)
#define BUCK2_PULL_DOWN_REG		BUCK_PULL_DOWN_REG
#define BUCK2_PULL_DOWN_MASK		BIT(2)
#define BUCK3_PULL_DOWN_REG		BUCK_PULL_DOWN_REG
#define BUCK3_PULL_DOWN_MASK		BIT(4)
#define BUCK4_PULL_DOWN_REG		BUCK_PULL_DOWN_REG
#define BUCK4_PULL_DOWN_MASK		BIT(6)

#define LDO1_PULL_DOWN_REG		LDO14_PULL_DOWN_REG
#define LDO1_PULL_DOWN_MASK		BIT(0)
#define LDO2_PULL_DOWN_REG		LDO14_PULL_DOWN_REG
#define LDO2_PULL_DOWN_MASK		BIT(2)
#define LDO3_PULL_DOWN_REG		LDO14_PULL_DOWN_REG
#define LDO3_PULL_DOWN_MASK		BIT(4)
#define LDO4_PULL_DOWN_REG		LDO14_PULL_DOWN_REG
#define LDO4_PULL_DOWN_MASK		BIT(6)
#define LDO5_PULL_DOWN_REG		LDO56_PULL_DOWN_REG
#define LDO5_PULL_DOWN_MASK		BIT(0)
#define LDO6_PULL_DOWN_REG		LDO56_PULL_DOWN_REG
#define LDO6_PULL_DOWN_MASK		BIT(2)
#define VREF_DDR_PULL_DOWN_REG		LDO56_PULL_DOWN_REG
#define VREF_DDR_PULL_DOWN_MASK		BIT(4)

#define BUCK_ICC_TURNOFF_REGISTER_MASK	GENMASK(6, 0)
#define LDO_ICC_TURNOFF_REGISTER_MASK	GENMASK(5, 0)

#define LDO_BYPASS_MASK			BIT(7)

/* Main PMIC Control Register
 * MAIN_CONTROL_REG
 * Address : 0x10
 */
#define ICC_EVENT_ENABLED		BIT(4)
#define PWRCTRL_POLARITY_HIGH		BIT(3)
#define PWRCTRL_PIN_VALID		BIT(2)
#define RESTART_REQUEST_ENABLED		BIT(1)
#define SOFTWARE_SWITCH_OFF_ENABLED	BIT(0)

/* Main PMIC PADS Control Register
 * PADS_PULL_REG
 * Address : 0x11
 */
#define WAKEUP_DETECTOR_DISABLED	BIT(4)
#define PWRCTRL_PD_ACTIVE		BIT(3)
#define PWRCTRL_PU_ACTIVE		BIT(2)
#define WAKEUP_PD_ACTIVE		BIT(1)
#define PONKEY_PU_ACTIVE		BIT(0)

/* Main PMIC VINLOW Control Register
 * VIN_CONTROL_REGC DMSC
 * Address : 0x15
 */
#define SWIN_DETECTOR_ENABLED		BIT(7)
#define SWOUT_DETECTOR_ENABLED		BIT(6)
#define VINLOW_ENABLED			BIT(0)
#define VINLOW_CTRL_REG_MASK		GENMASK(7, 0)

/* USB Control Register
 * Address : 0x40
 */
#define BOOST_OVP_DISABLED		BIT(7)
#define VBUS_OTG_DETECTION_DISABLED	BIT(6)
#define SW_OUT_DISCHARGE		BIT(5)
#define VBUS_OTG_DISCHARGE		BIT(4)
#define OCP_LIMIT_HIGH			BIT(3)
#define SWIN_SWOUT_ENABLED		BIT(2)
#define USBSW_OTG_SWITCH_ENABLED	BIT(1)
#define BOOST_ENABLED			BIT(0)

/* PONKEY_TURNOFF_REG
 * Address : 0x16
 */
#define PONKEY_PWR_OFF			BIT(7)
#define PONKEY_CC_FLAG_CLEAR		BIT(6)
#define PONKEY_TURNOFF_TIMER_MASK	GENMASK(3, 0)
#define PONKEY_TURNOFF_MASK		GENMASK(7, 0)
/* IRQ definitions */
enum {
	/* Interrupt Register 1 (0x50 for latch) */
	IT_SWOUT_R,
	IT_SWOUT_F,
	IT_VBUS_OTG_R,
	IT_VBUS_OTG_F,
	IT_WAKEUP_R,
	IT_WAKEUP_F,
	IT_PONKEY_R,
	IT_PONKEY_F,

	/* Interrupt Register 2 (0x51 for latch) */
	IT_OVP_BOOST,
	IT_OCP_BOOST,
	IT_OCP_SWOUT,
	IT_OCP_OTG,
	IT_CURLIM_BUCK4,
	IT_CURLIM_BUCK3,
	IT_CURLIM_BUCK2,
	IT_CURLIM_BUCK1,

	/* Interrupt Register 3 (0x52 for latch) */
	IT_SHORT_SWOUT,
	IT_SHORT_SWOTG,
	IT_CURLIM_LDO6,
	IT_CURLIM_LDO5,
	IT_CURLIM_LDO4,
	IT_CURLIM_LDO3,
	IT_CURLIM_LDO2,
	IT_CURLIM_LDO1,

	/* Interrupt Register 3 (0x52 for latch) */
	IT_SWIN_R,
	IT_SWIN_F,
	IT_RESERVED_1,
	IT_RESERVED_2,
	IT_VINLOW_R,
	IT_VINLOW_F,
	IT_TWARN_R,
	IT_TWARN_F,

	IRQ_NR,
};

/* IRQ masks */
/* Interrupt Mask for Register 1 (0x50 for latch) */
#define IT_SWOUT_R_MASK		BIT(7)
#define IT_SWOUT_F_MASK		BIT(6)
#define IT_VBUS_OTG_R_MASK	BIT(5)
#define IT_VBUS_OTG_F_MASK	BIT(4)
#define IT_WAKEUP_R_MASK	BIT(3)
#define IT_WAKEUP_F_MASK	BIT(2)
#define IT_PONKEY_R_MASK	BIT(1)
#define IT_PONKEY_F_MASK	BIT(0)

/* Interrupt Mask for Register 2 (0x51 for latch) */
#define IT_OVP_BOOST_MASK	BIT(7)
#define IT_OCP_BOOST_MASK	BIT(6)
#define IT_OCP_SWOUT_MASK	BIT(5)
#define IT_OCP_OTG_MASK		BIT(4)
#define IT_CURLIM_BUCK4_MASK	BIT(3)
#define IT_CURLIM_BUCK3_MASK	BIT(2)
#define IT_CURLIM_BUCK2_MASK	BIT(1)
#define IT_CURLIM_BUCK1_MASK	BIT(0)

/* Interrupt Mask for Register 3 (0x52 for latch) */
#define IT_SHORT_SWOUT_MASK     BIT(7)
#define IT_SHORT_SWOTG_MASK     BIT(6)
#define IT_CURLIM_LDO6_MASK	BIT(5)
#define IT_CURLIM_LDO5_MASK	BIT(4)
#define IT_CURLIM_LDO4_MASK	BIT(3)
#define IT_CURLIM_LDO3_MASK	BIT(2)
#define IT_CURLIM_LDO2_MASK	BIT(1)
#define IT_CURLIM_LDO1_MASK	BIT(0)

/* Interrupt Mask for Register 4 (0x53 for latch) */
#define IT_SWIN_R_MASK		BIT(7)
#define IT_SWIN_F_MASK		BIT(6)
/*  Reserved 1 */
/*  Reserved 2 */
#define IT_VINLOW_R_MASK	BIT(3)
#define IT_VINLOW_F_MASK	BIT(2)
#define IT_TWARN_R_MASK		BIT(1)
#define IT_TWARN_F_MASK		BIT(0)

/*
 * struct stpmu1_dev - stpmu1 master device for sub-drivers
 * @dev: master device of the chip (can be used to access platform data)
 * @i2c: i2c client private data for regulator
 * @np: device DT node pointer
 * @irq_base: base IRQ numbers
 * @irq: generic IRQ number
 * @regmap_irq_chip_data: irq chip data
 */
struct stpmu1_dev {
	struct device *dev;
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct device_node *np;
	unsigned int irq_base;
	int irq;
	struct regmap_irq_chip_data *irq_data;
};

#endif /*  __LINUX_MFD_STPMU1_H */
