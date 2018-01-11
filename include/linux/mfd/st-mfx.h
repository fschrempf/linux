/*
 * STMicroelectronics Multi-Function eXpander (ST-MFX)
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com> for STMicroelectronics.
 *
 * License terms: GPL V2.0.
 *
 * st-mfx is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * st-mfx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * st-mfx. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MFD_ST_MFX_H
#define __MFD_ST_MFX_H

enum mfx_block {
	MFX_BLOCK_GPIO		= BIT(0),
	MFX_BLOCK_TS		= BIT(1),
	MFX_BLOCK_IDD		= BIT(2),
	MFX_BLOCK_ALTGPIO	= BIT(3),
};

/*
 * 8 events can activate the MFX_IRQ_OUT signal,
 * but for the moment, only GPIO event is used
 */
#define MFX_NR_IRQ_SRC	1

/**
 * struct mfx - MFX MFD structure
 * @blocks: mask of mfx_block to be enabled
 * @num_gpio: number of gpios
 */
struct mfx {
	u32 blocks;
	u32 num_gpio;
};

int mfx_reg_write(struct mfx *mfx, u8 reg, u8 data);
int mfx_reg_read(struct mfx *mfx, u8 reg);
int mfx_block_read(struct mfx *mfx, u8 reg, u8 length, u8 *values);
int mfx_block_write(struct mfx *mfx, u8 reg, u8 length, const u8 *values);
int mfx_set_bits(struct mfx *mfx, u8 reg, u8 mask, u8 val);
int mfx_enable(struct mfx *mfx, unsigned int blocks);
int mfx_disable(struct mfx *mfx, unsigned int blocks);

/* General */
#define MFX_REG_CHIP_ID			0x00 /* R */
#define MFX_REG_FW_VERSION_MSB		0x01 /* R */
#define MFX_REG_FW_VERSION_LSB		0x02 /* R */
#define MFX_REG_SYS_CTRL		0x40 /* RW */
/* IRQ output management */
#define MFX_REG_IRQ_OUT_PIN		0x41 /* RW */
#define MFX_REG_IRQ_SRC_EN		0x42 /* RW */
#define MFX_REG_IRQ_PENDING		0x08 /* R */
#define MFX_REG_IRQ_ACK			0x44 /* RW */
/* GPIOs expander */
/* GPIO_STATE1 0x10, GPIO_STATE2 0x11, GPIO_STATE3 0x12 */
#define MFX_REG_GPIO_STATE		0x10 /* R */
/* GPIO_DIR1 0x60, GPIO_DIR2 0x61, GPIO_DIR3 0x63 */
#define MFX_REG_GPIO_DIR		0x60 /* RW */
/* GPIO_TYPE1 0x64, GPIO_TYPE2 0x65, GPIO_TYPE3 0x66 */
#define MFX_REG_GPIO_TYPE		0x64 /* RW */
/* GPIO_PUPD1 0x68, GPIO_PUPD2 0x69, GPIO_PUPD3 0x6A */
#define MFX_REG_GPIO_PUPD		0x68 /* RW */
/* GPO_SET1 0x6C, GPO_SET2 0x6D, GPO_SET3 0x6E */
#define MFX_REG_GPO_SET			0x6C /* RW */
/* GPO_CLR1 0x70, GPO_CLR2 0x71, GPO_CLR3 0x72 */
#define MFX_REG_GPO_CLR			0x70 /* RW */
/* IRQ_GPI_SRC1 0x48, IRQ_GPI_SRC2 0x49, IRQ_GPI_SRC3 0x4A */
#define MFX_REG_IRQ_GPI_SRC		0x48 /* RW */
/* IRQ_GPI_EVT1 0x4C, IRQ_GPI_EVT2 0x4D, IRQ_GPI_EVT3 0x4E */
#define MFX_REG_IRQ_GPI_EVT		0x4C /* RW */
/* IRQ_GPI_TYPE1 0x50, IRQ_GPI_TYPE2 0x51, IRQ_GPI_TYPE3 0x52 */
#define MFX_REG_IRQ_GPI_TYPE		0x50 /* RW */
/* IRQ_GPI_PENDING1 0x0C, IRQ_GPI_PENDING2 0x0D, IRQ_GPI_PENDING3 0x0E*/
#define MFX_REG_IRQ_GPI_PENDING		0x0C /* R */
/* IRQ_GPI_ACK1 0x54, IRQ_GPI_ACK2 0x55, IRQ_GPI_ACK3 0x56 */
#define MFX_REG_IRQ_GPI_ACK		0x54 /* RW */

/* MFX_REG_CHIP_ID bitfields */
#define MFX_REG_CHIP_ID_MASK		GENMASK(7, 0)

/* MFX_REG_SYS_CTRL bitfields */
#define MFX_REG_SYS_CTRL_GPIO_EN	BIT(0)
#define MFX_REG_SYS_CTRL_TS_EN		BIT(1)
#define MFX_REG_SYS_CTRL_IDD_EN		BIT(2)
#define MFX_REG_SYS_CTRL_ALTGPIO_EN	BIT(3)
#define MFX_REG_SYS_CTRL_STANDBY	BIT(6)
#define MFX_REG_SYS_CTRL_SWRST		BIT(7)

/* MFX_REG_IRQ_OUT_PIN bitfields */
#define MFX_REG_IRQ_OUT_PIN_TYPE	BIT(0) /* 0-OD 1-PP */
#define MFX_REG_IRQ_OUT_PIN_POL		BIT(1) /* 0-active LOW 1-active HIGH */

/* MFX_REG_IRQ_SRC_EN bitfields */
#define MFX_REG_IRQ_SRC_EN_GPIO		BIT(0)

/* MFX_REG_IRQ_PENDING bitfields */
#define MFX_REG_IRQ_PENDING_GPIO	BIT(0)

#endif /* __MFD_ST_MFX_H */

