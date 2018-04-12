// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 exceet electronics GmbH
 *
 * Author: Frieder Schrempf <frieder.schrempf@exceet.de>
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_TOSHIBA		0x98

static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_X4_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(true, 0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
		SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
		SPINAND_PROG_LOAD(false, 0, NULL, 0));

static int tc58cvg2s0h_ooblayout_ecc(struct mtd_info *mtd, int section,
				     struct mtd_oob_region *region)
{
	if (section > 7)
		return -ERANGE;

	region->offset = 16 * section + 128;
	region->length = 16;

	return 0;
}

static int tc58cvg2s0h_ooblayout_free(struct mtd_info *mtd, int section,
				      struct mtd_oob_region *region)
{
	if (section > 7)
		return -ERANGE;

	region->offset = 16 * section;
	region->length = 16;

	return 0;
}

static const struct mtd_ooblayout_ops tc58cvg2s0h_ooblayout = {
	.ecc = tc58cvg2s0h_ooblayout_ecc,
	.free = tc58cvg2s0h_ooblayout_free,
};

static int tc58cvg2s0h_get_eccsr(struct spinand_device *spinand, u8 *eccsr)
{
	u8 i, temp = 0;
	struct spi_mem_op op = SPINAND_GET_FEATURE_OP(0x40, &temp);
	int ret;
	
	for(i = 0; i < 4; i++) {
		op.addr.val += 0x10;

		ret = spi_mem_exec_op(spinand->spimem, &op);
		if(ret)
			return ret;

		*eccsr += (temp & 0x0F) + (temp >> 4);
	}

	return 0;
}

static int tc58cvg2s0h_ecc_get_status(struct spinand_device *spinand,
				      u8 status)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	u8 eccsr;

	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	case STATUS_ECC_HAS_BITFLIPS:
		/*
		 * Let's try to retrieve the real maximum number of bitflips
		 * in order to avoid forcing the wear-leveling layer to move
		 * data around if it's not necessary.
		 */
		if (tc58cvg2s0h_get_eccsr(spinand, &eccsr))
			return nand->eccreq.strength;

		eccsr >>= 4;

		if (WARN_ON(eccsr > nand->eccreq.strength || !eccsr))
			return nand->eccreq.strength;

		return eccsr;

	default:
		break;
	}

	return -EINVAL;
}

static const struct spinand_info toshiba_spinand_table[] = {
	SPINAND_INFO("TC58CVG2S0H", 0xCD,
		     NAND_MEMORG(1, 4069, 256, 64, 2048, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     SPINAND_HAS_QE_BIT,
		     SPINAND_ECCINFO(&tc58cvg2s0h_ooblayout,
				      tc58cvg2s0h_ecc_get_status),
		     ),
};

static int toshiba_spinand_detect(struct spinand_device *spinand)
{
	u8 *id = spinand->id.data;
	int ret;

	/*
	 * Toshiba SPI NAND read ID needs a dummy byte, so the first byte in
	 * raw_id is garbage.
	 */
	if (id[1] != SPINAND_MFR_TOSHIBA)
		return 0;

	ret = spinand_match_and_init(spinand, toshiba_spinand_table,
				     ARRAY_SIZE(toshiba_spinand_table),
				     id[2]);
	if (ret)
		return ret;

	return 1;
}

static int toshiba_spinand_init(struct spinand_device *spinand)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	const u8 val = 8 << 4;
	struct spi_mem_op op = SPINAND_SET_FEATURE_OP(0x10, &val);

	/* set bitflip threshold to 8 */
	return spi_mem_exec_op(spinand->spimem, &op);
}

static const struct spinand_manufacturer_ops toshiba_spinand_manuf_ops = {
	.detect = toshiba_spinand_detect,
	.init = toshiba_spinand_init,
};

const struct spinand_manufacturer toshiba_spinand_manufacturer = {
	.id = SPINAND_MFR_TOSHIBA,
	.name = "Toshiba",
	.ops = &toshiba_spinand_manuf_ops,
};
