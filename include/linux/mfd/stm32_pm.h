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

#define CR2			0x8
#define MPUCR			0x10
#define MCUCR			0x14
#define WKUPCR			0x20
#define WKUPFR			0x24
#define WKUPENR			0x28
#define WKUPEPR			0x2C
#define CSSF			BIT(9)
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
#define WKUPUPD_MASK		GENMASK(1, 0)
#define WKUPUPD1_SHIFT		16
#define WKUPP_MASK		1
#define WKUPP1_SHIFT		8
#define WUP_FLAGS_MASK		GENMASK(5, 0)
#define NO_PULL			0
#define PULL_UP			1
#define PULL_DOWN		2
#define BREN			BIT(0)
#define RREN			BIT(1)

enum {
	DISABLE,
	ENABLE,
};

int stm32_setup_pwr_irq(int pin_id, bool enable);
int stm32_configure_wkup_pin(int pin_id, int pupd, int polarity);
