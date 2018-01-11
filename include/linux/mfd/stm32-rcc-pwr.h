/*
 * Copyright (C) STMicroelectronics SA 2017
 * Author: Olivier Bideau <olivier.bideau@st.com>
 *
 * License terms: GPL V2.0.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define RCC_CIER		0x414
#define RCC_CIFR		0x418
#define RCC_BOOTCR		0x100
#define RCC_SREQSETR		0x104
#define RCC_SREQCLRR		0x108
#define RCC_GCR			0x10C
#define RCC_PWRLPDLYCR		0x41C
#define RCC_RSTSCLRR		0x400
#define RCC_RSTSR		0x408
#define RCC_DDRITFCR		0x0D8
#define CR1			0x0
#define CSR1			0x4
#define CR2			0x8
#define CR3			0xC
#define MPUCR			0x10
#define MCUCR			0x14
#define WKUPCR			0x20
#define WKUPFR			0x24
#define WKUPENR			0x28
#define WKUPEPR			0x2C
#define RCC_LSIRDYIE		BIT(0)
#define RCC_LSERDYIE		BIT(1)
#define RCC_HSIRDYIE		BIT(2)
#define RCC_HSERDYIE		BIT(3)
#define RCC_CSIRDYIE		BIT(4)
#define PLL1_DYIE		BIT(8)
#define PLL2_DYIE		BIT(9)
#define PLL3_DYIE		BIT(10)
#define PLL4_DYIE		BIT(11)
#define LSECSSIE		BIT(16)
#define RCC_WAKUP		BIT(20)
#define RCC_STOP_MASK		(BIT(0) | BIT(1))
#define MCU_BEN			BIT(0)
#define MPU_BEN			BIT(1)
#define BOOT_MCU		BIT(0)
#define PWRDELAY_MASK		GENMASK(21, 0)
#define SKIP_MCU_DELAY		BIT(24)
#define PORRSTF			BIT(0)
#define BORRSTF			BIT(1)
#define PADRSTF			BIT(2)
#define HCSSRSTF		BIT(3)
#define VCORERSTF		BIT(4)
#define MPSYSRSTF		BIT(6)
#define MCSYSRSTF		BIT(7)
#define IWDG1RSTF		BIT(8)
#define IWDG2RSTF		BIT(9)
#define AXIDCGEN		BIT(8)
#define DDRCAPBLPEN		BIT(7)
#define DDRPHYCLPEN		BIT(5)
#define DDRPHYCAPBLPEN		BIT(10)
#define DBP			BIT(8)
#define LPDS			BIT(0)
#define LPCFG			BIT(1)
#define LVDS			BIT(2)
#define MONEN			BIT(4)
#define AVDEN			BIT(16)
#define PVDEN			BIT(4)
#define BREN			BIT(0)
#define RREN			BIT(1)
#define BRRDY			BIT(16)
#define RRRDY			BIT(17)
#define PDDS			BIT(0)
#define CSTBYDIS		BIT(3)
#define HOLDMCUF		BIT(4)
#define STOPF			BIT(5)
#define SBF			BIT(6)
#define SBF_MPU			BIT(7)
#define CSSF			BIT(9)
#define HOLDMCU			BIT(10)
#define HOLMPUEN		BIT(14)
#define STANDBYWFIL2		BIT(15)
#define HOLDMPUF		BIT(4)
#define DEEPSLEEP		BIT(15)
#define REG11			BIT(30)
#define REG18			BIT(28)
#define USB33			BIT(24)
#define DDRSRDIS		BIT(11)
#define DDRSREN			BIT(10)
#define WKUP1			BIT(0)
#define WKUP2			BIT(1)
#define WKUP3			BIT(2)
#define WKUP4			BIT(3)
#define WKUP5			BIT(4)
#define WKUP6			BIT(5)
#define WKUPP1			BIT(8)
#define WKUPP2			BIT(9)
#define WKUPP3			BIT(10)
#define WKUPP4			BIT(11)
#define WKUPP5			BIT(12)
#define WKUPP6			BIT(13)
#define WKUPUPD_MASK		0x3
#define WKUPUPD1_SHIFT		16
#define WKUPUPD2_SHIFT		18
#define WKUPUPD3_SHIFT		20
#define WKUPUPD4_SHIFT		22
#define WKUPUPD5_SHIFT		24
#define WKUPUPD6_SHIFT		26
#define	VOLT0			0
#define	VOLT1			1
#define	VOLT2			2
#define	VOLT3			3
#define	VOLT4			4
#define	VOLT5			5
#define	VOLT6			6
#define	VOLT7			7
#define RCC_IRQ_FLAGS_MASK	0x110F1F
#define RCC_BOOT_FLAGS_MASK	0x3DF
#define WUP_FLAGS_MASK		0x3F
#define PWR_PLS_SHIFT		5
#define PWR_PLS_MASK		0x7
#define PWR_ALS_SHIFT		17
#define PWR_ALS_MASK		0x3

enum {
	DISABLE,
	ENABLE,
};
