/*
 * stm32_fmc.c
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Christophe Kerello <christophe.kerello@st.com>
 *
 * License terms: GPL V2.0.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* Bad block marker length */
#define FMC_BBM_LEN			2

/* BCHDSRx registers length */
#define FMC_BCHDSRS_LEN			20

/* HECCR length */
#define FMC_HECCR_LEN			4

/* Max requests done for a 8k nand page size */
#define FMC_MAX_SG_COUNT		16

/* Command delay */
#define FMC_RB_DELAY_US			30

/* Bank offsets */
#define FMC_DATA_SECTION		0x00000
#define FMC_COMMAND_SECTION		0x10000
#define FMC_ADDRESS_SECTION		0x20000

/* Timings */
#define FMC_THIZ			1
#define FMC_TIO				8000
#define FMC_TSYNC			3000
#define FMC_PCR_TIMING_MASK		0xf
#define FMC_PMEM_PATT_TIMING_MASK	0xff

/* FMC Controller Registers */
#define FMC_BCR1			0x0
#define FMC_PCR(bank)			(0x0020 * ((bank) + 1))
#define FMC_SR(bank)			((0x0020 * ((bank) + 1)) + 0x04)
#define FMC_PMEM(bank)			((0x0020 * ((bank) + 1)) + 0x08)
#define FMC_PATT(bank)			((0x0020 * ((bank) + 1)) + 0x0c)
#define FMC_HECCR(bank)			((0x0020 * ((bank) + 1)) + 0x14)
#define FMC_CSQCR			0x0200
#define FMC_CSQCFGR1			0x0204
#define FMC_CSQCFGR2			0x0208
#define FMC_CSQCFGR3			0x020c
#define FMC_CSQAR1			0x0210
#define FMC_CSQAR2			0x0214
#define FMC_CSQIER			0x0220
#define FMC_CSQISR			0x0224
#define FMC_CSQICR			0x0228
#define FMC_CSQEMSR			0x0230
#define FMC_BCHIER			0x0250
#define FMC_BCHISR			0x0254
#define FMC_BCHICR			0x0258
#define FMC_BCHPBR1			0x0260
#define FMC_BCHPBR2			0x0264
#define FMC_BCHPBR3			0x0268
#define FMC_BCHPBR4			0x026c
#define FMC_BCHDSR0			0x027c
#define FMC_BCHDSR1			0x0280
#define FMC_BCHDSR2			0x0284
#define FMC_BCHDSR3			0x0288
#define FMC_BCHDSR4			0x028c

/* Register: FMC_BCR1 */
#define FMC_BCR1_FMCEN			BIT(31)

/* Register: FMC_PCR */
#define FMC_PCR_PWAITEN			BIT(1)
#define FMC_PCR_PBKEN			BIT(2)
#define FMC_PCR_PWID_MASK		GENMASK(5, 4)
#define FMC_PCR_PWID(x)			(((x) & 0x3) << 4)
#define FMC_PCR_PWID_BUSWIDTH_8		0
#define FMC_PCR_PWID_BUSWIDTH_16	1
#define FMC_PCR_ECCEN			BIT(6)
#define FMC_PCR_ECCALG			BIT(8)
#define FMC_PCR_TCLR_MASK		GENMASK(12, 9)
#define FMC_PCR_TCLR(x)			(((x) & 0xf) << 9)
#define FMC_PCR_TAR_MASK		GENMASK(16, 13)
#define FMC_PCR_TAR(x)			(((x) & 0xf) << 13)
#define FMC_PCR_ECCSS_MASK		GENMASK(19, 17)
#define FMC_PCR_ECCSS(x)		(((x) & 0x7) << 17)
#define FMC_PCR_ECCSS_512		1
#define FMC_PCR_BCHECC			BIT(24)
#define FMC_PCR_WEN			BIT(25)

/* Register: FMC_SR */
#define FMC_SR_NWRF			BIT(6)

/* Register: FMC_PMEM */
#define FMC_PMEM_MEMSET(x)		(((x) & 0xff) << 0)
#define FMC_PMEM_MEMWAIT(x)		(((x) & 0xff) << 8)
#define FMC_PMEM_MEMHOLD(x)		(((x) & 0xff) << 16)
#define FMC_PMEM_MEMHIZ(x)		(((x) & 0xff) << 24)

/* Register: FMC_PATT */
#define FMC_PATT_ATTSET(x)		(((x) & 0xff) << 0)
#define FMC_PATT_ATTWAIT(x)		(((x) & 0xff) << 8)
#define FMC_PATT_ATTHOLD(x)		(((x) & 0xff) << 16)
#define FMC_PATT_ATTHIZ(x)		(((x) & 0xff) << 24)

/* Register: FMC_CSQCR */
#define FMC_CSQCR_CSQSTART		BIT(0)

/* Register: FMC_CSQCFGR1 */
#define FMC_CSQCFGR1_CMD2EN		BIT(1)
#define FMC_CSQCFGR1_DMADEN		BIT(2)
#define FMC_CSQCFGR1_ACYNBR(x)		(((x) & 0x7) << 4)
#define FMC_CSQCFGR1_CMD1(x)		(((x) & 0xff) << 8)
#define FMC_CSQCFGR1_CMD2(x)		(((x) & 0xff) << 16)
#define FMC_CSQCFGR1_CMD1T		BIT(24)
#define FMC_CSQCFGR1_CMD2T		BIT(25)

/* Register: FMC_CSQCFGR2 */
#define FMC_CSQCFGR2_SQSDTEN		BIT(0)
#define FMC_CSQCFGR2_RCMD2EN		BIT(1)
#define FMC_CSQCFGR2_DMASEN		BIT(2)
#define FMC_CSQCFGR2_RCMD1(x)		(((x) & 0xff) << 8)
#define FMC_CSQCFGR2_RCMD2(x)		(((x) & 0xff) << 16)
#define FMC_CSQCFGR2_RCMD1T		BIT(24)
#define FMC_CSQCFGR2_RCMD2T		BIT(25)

/* Register: FMC_CSQCFGR3 */
#define FMC_CSQCFGR3_SNBR(x)		(((x) & 0x1f) << 8)
#define FMC_CSQCFGR3_AC1T		BIT(16)
#define FMC_CSQCFGR3_AC2T		BIT(17)
#define FMC_CSQCFGR3_AC3T		BIT(18)
#define FMC_CSQCFGR3_AC4T		BIT(19)
#define FMC_CSQCFGR3_AC5T		BIT(20)
#define FMC_CSQCFGR3_SDT		BIT(21)
#define FMC_CSQCFGR3_RAC1T		BIT(22)
#define FMC_CSQCFGR3_RAC2T		BIT(23)

/* Register: FMC_CSQCAR1 */
#define FMC_CSQCAR1_ADDC1(x)		(((x) & 0xff) << 0)
#define FMC_CSQCAR1_ADDC2(x)		(((x) & 0xff) << 8)
#define FMC_CSQCAR1_ADDC3(x)		(((x) & 0xff) << 16)
#define FMC_CSQCAR1_ADDC4(x)		(((x) & 0xff) << 24)

/* Register: FMC_CSQCAR2 */
#define FMC_CSQCAR2_ADDC5(x)		(((x) & 0xff) << 0)
#define FMC_CSQCAR2_SAO(x)		(((x) & 0xffff) << 16)

/* Register: FMC_CSQIER */
#define FMC_CSQIER_TCIE			BIT(0)

/* Register: FMC_CSQICR */
#define FMC_CSQICR_CTCF			BIT(0)
#define FMC_CSQICR_CSCF			BIT(1)
#define FMC_CSQICR_CSEF			BIT(2)
#define FMC_CSQICR_CSUEF		BIT(3)
#define FMC_CSQICR_CCMDTCF		BIT(4)

/* Register: FMC_CSQEMSR */
#define FMC_CSQEMSR_SEM			GENMASK(15, 0)

/* Register: FMC_BCHIER */
#define FMC_BCHIER_DERIE		BIT(1)
#define FMC_BCHIER_EPBRIE		BIT(4)

/* Register: FMC_BCHICR */
#define FMC_BCHICR_CDUEF		BIT(0)
#define FMC_BCHICR_CDERF		BIT(1)
#define FMC_BCHICR_CDEFF		BIT(2)
#define FMC_BCHICR_CDSRF		BIT(3)
#define FMC_BCHICR_CEPBRF		BIT(4)

/* Register: FMC_BCHDSR0 */
#define FMC_BCHDSR0_DUE			BIT(0)
#define FMC_BCHDSR0_DEF			BIT(1)
#define FMC_BCHDSR0_DEN_MASK		GENMASK(7, 4)
#define FMC_BCHDSR0_DEN_SHIFT		4

/* Register: FMC_BCHDSR1 */
#define FMC_BCHDSR1_EBP1_MASK		GENMASK(12, 0)
#define FMC_BCHDSR1_EBP2_MASK		GENMASK(28, 16)
#define FMC_BCHDSR1_EBP2_SHIFT		16

/* Register: FMC_BCHDSR2 */
#define FMC_BCHDSR2_EBP3_MASK		GENMASK(12, 0)
#define FMC_BCHDSR2_EBP4_MASK		GENMASK(28, 16)
#define FMC_BCHDSR2_EBP4_SHIFT		16

/* Register: FMC_BCHDSR3 */
#define FMC_BCHDSR3_EBP5_MASK		GENMASK(12, 0)
#define FMC_BCHDSR3_EBP6_MASK		GENMASK(28, 16)
#define FMC_BCHDSR3_EBP6_SHIFT		16

/* Register: FMC_BCHDSR4 */
#define FMC_BCHDSR4_EBP7_MASK		GENMASK(12, 0)
#define FMC_BCHDSR4_EBP8_MASK		GENMASK(28, 16)
#define FMC_BCHDSR4_EBP8_SHIFT		16

enum stm32_ecc {
	FMC_ECC_HAM = 1,
	FMC_ECC_BCH4 = 4,
	FMC_ECC_BCH8 = 8
};

enum stm32_irq_state {
	FMC_IRQ_UNKNOWN = 0,
	FMC_IRQ_BCH,
	FMC_IRQ_SEQ
};

struct stm32_fmc_timings {
	u8 tclr;
	u8 tar;
	u8 thiz;
	u8 twait;
	u8 thold_mem;
	u8 tset_mem;
	u8 thold_att;
	u8 tset_att;
};

struct stm32_fmc {
	struct nand_chip chip;
	struct device *dev;
	void __iomem *io_base;
	void __iomem *common_base;
	void __iomem *attrib_base;
	struct clk *clk;

	struct dma_chan *dma_data_ch;
	struct dma_chan *dma_ecc_ch;
	struct sg_table dma_data_sg;
	struct sg_table dma_ecc_sg;
	u8 *ecc_buf;
	int dma_ecc_len;

	struct completion complete;
	struct completion dma_data_complete;
	struct completion dma_ecc_complete;

	struct stm32_fmc_timings timings;
	u8 irq_state;
	unsigned int bank;
};

/* Enable irq sources in case of the sequencer is used */
static inline void stm32_fmc_enable_seq_irq(struct stm32_fmc *fmc)
{
	u32 csqier = readl_relaxed(fmc->io_base + FMC_CSQIER);

	csqier |= FMC_CSQIER_TCIE;

	fmc->irq_state = FMC_IRQ_SEQ;

	writel_relaxed(csqier, fmc->io_base + FMC_CSQIER);
}

/* Disable irq sources in case of the sequencer is used */
static inline void stm32_fmc_disable_seq_irq(struct stm32_fmc *fmc)
{
	u32 csqier = readl_relaxed(fmc->io_base + FMC_CSQIER);

	csqier &= ~FMC_CSQIER_TCIE;

	writel_relaxed(csqier, fmc->io_base + FMC_CSQIER);

	fmc->irq_state = FMC_IRQ_UNKNOWN;
}

/* Clear irq sources in case of the sequencer is used */
static inline void stm32_fmc_clear_seq_irq(struct stm32_fmc *fmc)
{
	u32 csqicr = FMC_CSQICR_CTCF;

	csqicr |= FMC_CSQICR_CSCF;
	csqicr |= FMC_CSQICR_CSEF;
	csqicr |= FMC_CSQICR_CSUEF;
	csqicr |= FMC_CSQICR_CCMDTCF;

	writel_relaxed(csqicr, fmc->io_base + FMC_CSQICR);
}

/* Enable irq sources in case of bch is used */
static inline void stm32_fmc_enable_bch_irq(struct stm32_fmc *fmc, int mode)
{
	u32 bchier = readl_relaxed(fmc->io_base + FMC_BCHIER);

	if (mode == NAND_ECC_WRITE)
		bchier |= FMC_BCHIER_EPBRIE;
	else
		bchier |= FMC_BCHIER_DERIE;

	fmc->irq_state = FMC_IRQ_BCH;

	writel_relaxed(bchier, fmc->io_base + FMC_BCHIER);
}

/* Disable irq sources in case of bch is used */
static inline void stm32_fmc_disable_bch_irq(struct stm32_fmc *fmc)
{
	u32 bchier = readl_relaxed(fmc->io_base + FMC_BCHIER);

	bchier &= ~FMC_BCHIER_DERIE;
	bchier &= ~FMC_BCHIER_EPBRIE;

	writel_relaxed(bchier, fmc->io_base + FMC_BCHIER);

	fmc->irq_state = FMC_IRQ_UNKNOWN;
}

/* Clear irq sources in case of bch is used */
static inline void stm32_fmc_clear_bch_irq(struct stm32_fmc *fmc)
{
	u32 bchicr = FMC_BCHICR_CDUEF;

	bchicr |= FMC_BCHICR_CDERF;
	bchicr |= FMC_BCHICR_CDEFF;
	bchicr |= FMC_BCHICR_CDSRF;
	bchicr |= FMC_BCHICR_CEPBRF;

	writel_relaxed(bchicr, fmc->io_base + FMC_BCHICR);
}

/* Send command and address cycles */
static void stm32_fmc_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	void __iomem *base = fmc->attrib_base;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE) {
		writeb_relaxed(cmd, base + FMC_COMMAND_SECTION);
		return;
	}

	writeb_relaxed(cmd, base + FMC_ADDRESS_SECTION);
}

/*
 * Enable ecc logic and reset syndrome/parity bits previously calculated
 * Syndrome/parity bits is cleared by setting the ECCEN bit to 0
 */
static void stm32_fmc_hwctl(struct mtd_info *mtd, int mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	u32 pcr = readl_relaxed(fmc->io_base + FMC_PCR(fmc->bank));

	pcr &= ~FMC_PCR_ECCEN;
	writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));

	if (chip->ecc.strength != FMC_ECC_HAM) {
		if (mode == NAND_ECC_WRITE)
			pcr |= FMC_PCR_WEN;
		else
			pcr &= ~FMC_PCR_WEN;
		writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));

		reinit_completion(&fmc->complete);
		stm32_fmc_clear_bch_irq(fmc);
		stm32_fmc_enable_bch_irq(fmc, mode);
	}

	pcr |= FMC_PCR_ECCEN;
	writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));
}

/*
 * ECC HAMMING calculation
 * ECC is 3 bytes for 512 bytes of data (supports error correction up to
 * max of 1-bit)
 */
static inline void stm32_fmc_ham_decode(const u32 ecc_sta, u8 *ecc)
{
	ecc[0] = ecc_sta;
	ecc[1] = ecc_sta >> 8;
	ecc[2] = ecc_sta >> 16;
	ecc[3] = 0x0;
}

static int stm32_fmc_ham_calculate(struct mtd_info *mtd, const uint8_t *data,
				   uint8_t *ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	u32 sr, heccr;
	int ret;

	ret = readl_relaxed_poll_timeout(fmc->io_base + FMC_SR(fmc->bank),
					 sr, sr & FMC_SR_NWRF, 10, 1000);
	if (ret) {
		dev_err(fmc->dev, "ham timeout\n");
		return ret;
	}

	heccr = readl_relaxed(fmc->io_base + FMC_HECCR(fmc->bank));

	stm32_fmc_ham_decode(heccr, ecc);

	return 0;
}

static int stm32_fmc_ham_correct(struct mtd_info *mtd, uint8_t *dat,
				 uint8_t *read_ecc, uint8_t *calc_ecc)
{
	u8 bit_position = 0, b0, b1, b2;
	u32 byte_addr = 0, b;
	u32 i, shifting = 1;

	/* Indicate which bit and byte is faulty (if any) */
	b0 = read_ecc[0] ^ calc_ecc[0];
	b1 = read_ecc[1] ^ calc_ecc[1];
	b2 = read_ecc[2] ^ calc_ecc[2];
	b = b0 | (b1 << 8) | (b2 << 16);

	/* No errors */
	if (likely(!b))
		return 0;

	/* Calculate bit position */
	for (i = 0; i < 3; i++) {
		switch (b % 4) {
		case 2:
			bit_position += shifting;
		case 1:
			break;
		default:
			return -EBADMSG;
		}
		shifting <<= 1;
		b >>= 2;
	}

	/* Calculate byte position */
	shifting = 1;
	for (i = 0; i < 9; i++) {
		switch (b % 4) {
		case 2:
			byte_addr += shifting;
		case 1:
			break;
		default:
			return -EBADMSG;
		}
		shifting <<= 1;
		b >>= 2;
	}

	/* Flip the bit */
	dat[byte_addr] ^= (1 << bit_position);

	return 1;
}

/*
 * ECC BCH calculation and correction
 * ECC is 7/13 bytes for 512 bytes of data (supports error correction up to
 * max of 4-bit/8-bit)
 */
static int stm32_fmc_bch_calculate(struct mtd_info *mtd, const uint8_t *data,
				   uint8_t *ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	u32 bchpbr;

	/* Wait that the BCH encoder parity is available */
	if (!wait_for_completion_timeout(&fmc->complete,
					 msecs_to_jiffies(1000))) {
		dev_err(fmc->dev, "bch timeout\n");
		stm32_fmc_disable_bch_irq(fmc);
		return -ETIMEDOUT;
	}

	/* Read parity bits (write) or syndrome (read) */
	bchpbr = readl_relaxed(fmc->io_base + FMC_BCHPBR1);
	ecc[0] = bchpbr;
	ecc[1] = bchpbr >> 8;
	ecc[2] = bchpbr >> 16;
	ecc[3] = bchpbr >> 24;

	bchpbr = readl_relaxed(fmc->io_base + FMC_BCHPBR2);
	ecc[4] = bchpbr;
	ecc[5] = bchpbr >> 8;
	ecc[6] = bchpbr >> 16;
	ecc[7] = 0x0;

	if (chip->ecc.strength == FMC_ECC_BCH8) {
		ecc[7] = bchpbr >> 24;

		bchpbr = readl_relaxed(fmc->io_base + FMC_BCHPBR3);
		ecc[8] = bchpbr;
		ecc[9] = bchpbr >> 8;
		ecc[10] = bchpbr >> 16;
		ecc[11] = bchpbr >> 24;

		bchpbr = readl_relaxed(fmc->io_base + FMC_BCHPBR4);
		ecc[12] = bchpbr;
		ecc[13] = 0x0;
	}

	return 0;
}

/* BCH algorithm correction */
static int stm32_fmc_bch_correct_data(struct nand_chip *chip, u8 *dat,
				      u8 *read_ecc, u32 *ecc_sta)
{
	u32 bchdsr0 = ecc_sta[0];
	u32 bchdsr1 = ecc_sta[1];
	u32 bchdsr2 = ecc_sta[2];
	u32 bchdsr3 = ecc_sta[3];
	u32 bchdsr4 = ecc_sta[4];
	u16 pos[8];
	int i, den, eccsize = chip->ecc.size;

	/* No errors found */
	if (likely(!(bchdsr0 & FMC_BCHDSR0_DEF)))
		return 0;

	/* Too many errors detected */
	if (unlikely(bchdsr0 & FMC_BCHDSR0_DUE))
		return -EBADMSG;

	pos[0] = bchdsr1 & FMC_BCHDSR1_EBP1_MASK;
	pos[1] = (bchdsr1 & FMC_BCHDSR1_EBP2_MASK) >> FMC_BCHDSR1_EBP2_SHIFT;
	pos[2] = bchdsr2 & FMC_BCHDSR2_EBP3_MASK;
	pos[3] = (bchdsr2 & FMC_BCHDSR2_EBP4_MASK) >> FMC_BCHDSR2_EBP4_SHIFT;
	pos[4] = bchdsr3 & FMC_BCHDSR3_EBP5_MASK;
	pos[5] = (bchdsr3 & FMC_BCHDSR3_EBP6_MASK) >> FMC_BCHDSR3_EBP6_SHIFT;
	pos[6] = bchdsr4 & FMC_BCHDSR4_EBP7_MASK;
	pos[7] = (bchdsr4 & FMC_BCHDSR4_EBP8_MASK) >> FMC_BCHDSR4_EBP8_SHIFT;

	den = (bchdsr0 & FMC_BCHDSR0_DEN_MASK) >> FMC_BCHDSR0_DEN_SHIFT;
	for (i = 0; i < den; i++) {
		if (pos[i] < eccsize * 8)
			change_bit(pos[i], (unsigned long *)dat);
	}

	return den;
}

static int stm32_fmc_bch_correct(struct mtd_info *mtd, uint8_t *dat,
				 uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	u32 ecc_sta[5];

	/* Wait that the BCH syndrome is available */
	if (!wait_for_completion_timeout(&fmc->complete,
					 msecs_to_jiffies(1000))) {
		dev_err(fmc->dev, "bch timeout\n");
		stm32_fmc_disable_bch_irq(fmc);
		return -ETIMEDOUT;
	}

	ecc_sta[0] = readl_relaxed(fmc->io_base + FMC_BCHDSR0);
	ecc_sta[1] = readl_relaxed(fmc->io_base + FMC_BCHDSR1);
	ecc_sta[2] = readl_relaxed(fmc->io_base + FMC_BCHDSR2);
	ecc_sta[3] = readl_relaxed(fmc->io_base + FMC_BCHDSR3);
	ecc_sta[4] = readl_relaxed(fmc->io_base + FMC_BCHDSR4);

	return stm32_fmc_bch_correct_data(chip, dat, read_ecc, &ecc_sta[0]);
}

static int stm32_fmc_read_page(struct mtd_info *mtd,
			       struct nand_chip *chip, uint8_t *buf,
			       int oob_required, int page)
{
	int i, s, stat, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	int eccstrength = chip->ecc.strength;
	u8 *p = buf;
	u8 *ecc_calc = chip->buffers->ecccalc;
	u8 *ecc_code = chip->buffers->ecccode;
	unsigned int max_bitflips = 0;

	for (i = mtd->writesize + FMC_BBM_LEN, s = 0; s < eccsteps;
	     s++, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_READ);

		/* Read the nand page sector (512 bytes) */
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, s * eccsize, -1);
		chip->read_buf(mtd, p, eccsize);

		/* Read the corresponding ecc bytes */
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, i, -1);
		chip->read_buf(mtd, ecc_code, eccbytes);

		/* Correct the data */
		stat = chip->ecc.correct(mtd, p, ecc_code, ecc_calc);
		if (stat == -EBADMSG)
			/* Check for empty pages with bitflips */
			stat = nand_check_erased_ecc_chunk(p, eccsize,
							   ecc_code, eccbytes,
							   NULL, 0,
							   eccstrength);

		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}

	/* Read oob */
	if (oob_required) {
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	}

	return max_bitflips;
}

/* Sequencer read/write configuration */
static void stm32_fmc_rw_page_init(struct stm32_fmc *fmc, int page,
				   int raw, bool write_data)
{
	struct nand_chip *chip = &fmc->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 csqcfgr1, csqcfgr2, csqcfgr3;
	u32 csqar1, csqar2;
	u32 ecc_offset = mtd->writesize + FMC_BBM_LEN;
	u32 pcr = readl_relaxed(fmc->io_base + FMC_PCR(fmc->bank));

	if (write_data)
		pcr |= FMC_PCR_WEN;
	else
		pcr &= ~FMC_PCR_WEN;
	writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));

	if (chip->ecc.strength != FMC_ECC_HAM)
		stm32_fmc_clear_bch_irq(fmc);

	/*
	 * - Set Program Page/Page Read command
	 * - Enable DMA request data
	 * - Set timings
	 */
	csqcfgr1 = FMC_CSQCFGR1_DMADEN | FMC_CSQCFGR1_CMD1T;
	if (write_data)
		csqcfgr1 |= FMC_CSQCFGR1_CMD1(NAND_CMD_SEQIN);
	else
		csqcfgr1 |= FMC_CSQCFGR1_CMD1(NAND_CMD_READ0) |
			    FMC_CSQCFGR1_CMD2EN |
			    FMC_CSQCFGR1_CMD2(NAND_CMD_READSTART) |
			    FMC_CSQCFGR1_CMD2T;

	/*
	 * - Set Random Data Input/Random Data Read command
	 * - Enable the sequencer to access the Spare data area
	 * - Enable  DMA request status decoding for read
	 * - Set timings
	 */
	csqcfgr2 = FMC_CSQCFGR2_RCMD1T;
	if (write_data)
		csqcfgr2 |= FMC_CSQCFGR2_RCMD1(NAND_CMD_RNDIN);
	else
		csqcfgr2 |= FMC_CSQCFGR2_RCMD1(NAND_CMD_RNDOUT) |
			    FMC_CSQCFGR2_RCMD2EN |
			    FMC_CSQCFGR2_RCMD2(NAND_CMD_RNDOUTSTART) |
			    FMC_CSQCFGR2_RCMD2T;
	if (!raw) {
		csqcfgr2 |= write_data ? 0 : FMC_CSQCFGR2_DMASEN;
		csqcfgr2 |= FMC_CSQCFGR2_SQSDTEN;
	}

	/*
	 * - Set the number of sectors to be written
	 * - Set timings
	 */
	csqcfgr3 = FMC_CSQCFGR3_SNBR(chip->ecc.steps - 1);
	if (write_data) {
		csqcfgr3 |= FMC_CSQCFGR3_RAC2T;
		if (chip->chipsize > SZ_128M)
			csqcfgr3 |= FMC_CSQCFGR3_AC5T;
		else
			csqcfgr3 |= FMC_CSQCFGR3_AC4T;
	}

	/*
	 * Set the fourth first address cycles
	 * Byte 1 and byte 2 => column, we start at 0x0
	 * Byte 3 and byte 4 => page
	 */
	csqar1 = FMC_CSQCAR1_ADDC3(page);
	csqar1 |= FMC_CSQCAR1_ADDC4(page >> 8);

	/*
	 * - Set ecc byte offset in the spare area
	 * - Calculate the number of address cycles to be issued
	 * - Set byte 5 of address cycle if needed
	 */
	if (chip->options & NAND_BUSWIDTH_16)
		csqar2 = FMC_CSQCAR2_SAO(ecc_offset >> 1);
	else
		csqar2 = FMC_CSQCAR2_SAO(ecc_offset);
	if (chip->chipsize > SZ_128M) {
		csqcfgr1 |= FMC_CSQCFGR1_ACYNBR(5);
		csqar2 |= FMC_CSQCAR2_ADDC5(page >> 16);
	} else {
		csqcfgr1 |= FMC_CSQCFGR1_ACYNBR(4);
	}

	writel_relaxed(csqcfgr1, fmc->io_base + FMC_CSQCFGR1);
	writel_relaxed(csqcfgr2, fmc->io_base + FMC_CSQCFGR2);
	writel_relaxed(csqcfgr3, fmc->io_base + FMC_CSQCFGR3);
	writel_relaxed(csqar1, fmc->io_base + FMC_CSQAR1);
	writel_relaxed(csqar2, fmc->io_base + FMC_CSQAR2);
}

static void stm32_fmc_dma_callback(void *arg)
{
	struct completion *dma_completion = arg;

	complete(dma_completion);
}

/* Read/write data from/to a page */
static int stm32_fmc_xfer(struct stm32_fmc *fmc, const u8 *buf,
			  int raw, bool write_data)
{
	struct nand_chip *chip = &fmc->chip;
	struct dma_async_tx_descriptor *desc_data, *desc_ecc;
	struct scatterlist *sg;
	enum dma_data_direction dma_data_dir = DMA_FROM_DEVICE;
	enum dma_transfer_direction dma_transfer_dir = DMA_DEV_TO_MEM;
	u32 csqcr = readl_relaxed(fmc->io_base + FMC_CSQCR);
	int eccsteps = chip->ecc.steps;
	int eccsize = chip->ecc.size;
	const u8 *p = buf;
	int s, ret;

	/* Configure DMA data */
	if (write_data) {
		dma_data_dir = DMA_TO_DEVICE;
		dma_transfer_dir = DMA_MEM_TO_DEV;
	}

	for_each_sg(fmc->dma_data_sg.sgl, sg, eccsteps, s) {
		sg_set_buf(sg, p, eccsize);
		p += eccsize;
	}

	ret = dma_map_sg(fmc->dev, fmc->dma_data_sg.sgl,
			 eccsteps, dma_data_dir);
	if (ret < 0)
		return ret;

	desc_data = dmaengine_prep_slave_sg(fmc->dma_data_ch,
					    fmc->dma_data_sg.sgl,
					    eccsteps, dma_transfer_dir,
					    DMA_PREP_INTERRUPT);
	if (!desc_data) {
		ret = -ENOMEM;
		goto err_unmap_data;
	}

	reinit_completion(&fmc->dma_data_complete);
	reinit_completion(&fmc->complete);
	desc_data->callback = stm32_fmc_dma_callback;
	desc_data->callback_param = &fmc->dma_data_complete;
	ret = dma_submit_error(dmaengine_submit(desc_data));
	if (ret)
		goto err_unmap_data;

	dma_async_issue_pending(fmc->dma_data_ch);

	if (!write_data && !raw) {
		/* Configure DMA ecc status */
		p = fmc->ecc_buf;
		for_each_sg(fmc->dma_ecc_sg.sgl, sg, eccsteps, s) {
			sg_set_buf(sg, p, fmc->dma_ecc_len);
			p += fmc->dma_ecc_len;
		}

		ret = dma_map_sg(fmc->dev, fmc->dma_ecc_sg.sgl,
				 eccsteps, dma_data_dir);
		if (ret < 0)
			goto err_unmap_data;

		desc_ecc = dmaengine_prep_slave_sg(fmc->dma_ecc_ch,
						   fmc->dma_ecc_sg.sgl,
						   eccsteps, dma_transfer_dir,
						   DMA_PREP_INTERRUPT);
		if (!desc_ecc) {
			ret = -ENOMEM;
			goto err_unmap_ecc;
		}

		reinit_completion(&fmc->dma_ecc_complete);
		desc_ecc->callback = stm32_fmc_dma_callback;
		desc_ecc->callback_param = &fmc->dma_ecc_complete;
		ret = dma_submit_error(dmaengine_submit(desc_ecc));
		if (ret)
			goto err_unmap_ecc;

		dma_async_issue_pending(fmc->dma_ecc_ch);
	}

	stm32_fmc_clear_seq_irq(fmc);
	stm32_fmc_enable_seq_irq(fmc);

	/* Start the transfer */
	csqcr |= FMC_CSQCR_CSQSTART;
	writel_relaxed(csqcr, fmc->io_base + FMC_CSQCR);

	/* Wait end of sequencer transfer */
	if (!wait_for_completion_timeout(&fmc->complete,
					 msecs_to_jiffies(1000))) {
		dev_err(fmc->dev, "seq timeout\n");
		stm32_fmc_disable_seq_irq(fmc);
		dmaengine_terminate_all(fmc->dma_data_ch);
		if (!write_data && !raw)
			dmaengine_terminate_all(fmc->dma_ecc_ch);
		ret = -ETIMEDOUT;
		goto err_unmap_ecc;
	}

	/* Wait DMA data transfer completion */
	if (!wait_for_completion_timeout(&fmc->dma_data_complete,
					 msecs_to_jiffies(100))) {
		dev_err(fmc->dev, "data DMA timeout\n");
		dmaengine_terminate_all(fmc->dma_data_ch);
		ret = -ETIMEDOUT;
	}

	/* Wait DMA ecc transfer completion */
	if (!write_data && !raw) {
		if (!wait_for_completion_timeout(&fmc->dma_ecc_complete,
						 msecs_to_jiffies(100))) {
			dev_err(fmc->dev, "ecc DMA timeout\n");
			dmaengine_terminate_all(fmc->dma_ecc_ch);
			ret = -ETIMEDOUT;
		}
	}

err_unmap_ecc:
	if (!write_data && !raw)
		dma_unmap_sg(fmc->dev, fmc->dma_ecc_sg.sgl,
			     eccsteps, dma_data_dir);

err_unmap_data:
	dma_unmap_sg(fmc->dev, fmc->dma_data_sg.sgl, eccsteps, dma_data_dir);

	return ret;
}

static int stm32_fmc_sequencer_write(struct mtd_info *mtd,
				     struct nand_chip *chip,
				     const u8 *buf, int oob_required,
				     int page, int raw)
{
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	int ret;

	/* Configure the sequencer */
	stm32_fmc_rw_page_init(fmc, page, raw, true);

	/* Write the page */
	ret = stm32_fmc_xfer(fmc, buf, raw, true);
	if (ret)
		return ret;

	/* Write oob */
	if (oob_required) {
		chip->cmdfunc(mtd, NAND_CMD_RNDIN, mtd->writesize, -1);
		chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	}

	/* Send command to program the page */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	ret = chip->waitfunc(mtd, chip);

	return ret & NAND_STATUS_FAIL ? -EIO : 0;
}

static int stm32_fmc_sequencer_write_page(struct mtd_info *mtd,
					  struct nand_chip *chip,
					  const uint8_t *buf,
					  int oob_required,
					  int page)
{
	return stm32_fmc_sequencer_write(mtd, chip, buf,
					 oob_required, page, false);
}

static int stm32_fmc_sequencer_write_page_raw(struct mtd_info *mtd,
					      struct nand_chip *chip,
					      const uint8_t *buf,
					      int oob_required,
					      int page)
{
	return stm32_fmc_sequencer_write(mtd, chip, buf,
					 oob_required, page, true);
}

/*
 * Get a status indicating which sectors have errors
 * Only available when the sequencer is used (BCH only)
 */
static inline u16 stm32_fmc_get_mapping_status(struct stm32_fmc *fmc)
{
	u32 csqemsr = readl_relaxed(fmc->io_base + FMC_CSQEMSR);

	return csqemsr & FMC_CSQEMSR_SEM;
}

static int stm32_fmc_sequencer_read_page(struct mtd_info *mtd,
					 struct nand_chip *chip,
					 uint8_t *buf,
					 int oob_required,
					 int page)
{
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	int i, s, ret, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	int eccstrength = chip->ecc.strength;
	u8 *p = buf;
	u8 *ecc_calc = chip->buffers->ecccalc;
	u8 *ecc_code = chip->buffers->ecccode;
	u32 *ecc_sta = (u32 *)fmc->ecc_buf;
	u16 sta_map = 0xFFFF;
	unsigned int max_bitflips = 0;

	/* Configure the sequencer */
	stm32_fmc_rw_page_init(fmc, page, 0, false);

	/* Read the page */
	ret = stm32_fmc_xfer(fmc, buf, 0, false);
	if (ret)
		return ret;

	/* In case of BCH is used, get errors mapping */
	if (eccstrength != FMC_ECC_HAM)
		sta_map = stm32_fmc_get_mapping_status(fmc);

	/* Read oob */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* Check if errors happen for BCH algorithm */
	if (likely(!sta_map))
		return 0;

	ret = mtd_ooblayout_get_eccbytes(mtd, ecc_code, chip->oob_poi, 0,
					 chip->ecc.total);
	if (ret)
		return ret;

	/* Correct data */
	for (i = 0, s = 0; s < eccsteps; s++, i += eccbytes, p += eccsize) {
		int stat = 0;

		if (eccstrength == FMC_ECC_HAM) {
			/* Ecc_sta = FMC_HECCR */
			stm32_fmc_ham_decode(*ecc_sta, &ecc_calc[i]);
			stat = stm32_fmc_ham_correct(mtd, p, &ecc_code[i],
						     &ecc_calc[i]);
			ecc_sta++;
		} else {
			/*
			 * Ecc_sta[0] = FMC_BCHDSR0
			 * Ecc_sta[1] = FMC_BCHDSR1
			 * Ecc_sta[2] = FMC_BCHDSR2
			 * Ecc_sta[3] = FMC_BCHDSR3
			 * Ecc_sta[4] = FMC_BCHDSR4
			 */
			if (sta_map & BIT(s))
				stat = stm32_fmc_bch_correct_data(chip, p,
								  &ecc_code[i],
								  ecc_sta);
			ecc_sta += 5;
		}

		if (stat == -EBADMSG)
			/* Check for empty pages with bitflips */
			stat = nand_check_erased_ecc_chunk(p, eccsize,
							   ecc_code, eccbytes,
							   NULL, 0,
							   eccstrength);

		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}

	return max_bitflips;
}

static int stm32_fmc_sequencer_read_page_raw(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     uint8_t *buf,
					     int oob_required,
					     int page)
{
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	int ret;

	/* Configure the sequencer */
	stm32_fmc_rw_page_init(fmc, page, 1, false);

	/* Read the page */
	ret = stm32_fmc_xfer(fmc, buf, 1, false);
	if (ret)
		return ret;

	/* Read oob */
	if (oob_required) {
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize, -1);
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	}

	return 0;
}

static irqreturn_t stm32_fmc_irq(int irq, void *dev_id)
{
	struct stm32_fmc *fmc = (struct stm32_fmc *)dev_id;

	if (fmc->irq_state == FMC_IRQ_SEQ)
		/* Sequencer is used */
		stm32_fmc_disable_seq_irq(fmc);
	else if (fmc->irq_state == FMC_IRQ_BCH)
		/* BCH is used */
		stm32_fmc_disable_bch_irq(fmc);

	complete(&fmc->complete);

	return IRQ_HANDLED;
}

/* Timings configuration */
static void stm32_fmc_timings_init(struct stm32_fmc *fmc)
{
	struct stm32_fmc_timings *timings = &fmc->timings;
	u32 pcr = readl_relaxed(fmc->io_base + FMC_PCR(fmc->bank));
	u32 pmem, patt;

	/* Set tclr/tar timings */
	pcr &= ~FMC_PCR_TCLR_MASK;
	pcr |= FMC_PCR_TCLR(timings->tclr);
	pcr &= ~FMC_PCR_TAR_MASK;
	pcr |= FMC_PCR_TAR(timings->tar);

	/* Set tset/twait/thold/thiz timings in common bank */
	pmem = FMC_PMEM_MEMSET(timings->tset_mem);
	pmem |= FMC_PMEM_MEMWAIT(timings->twait);
	pmem |= FMC_PMEM_MEMHOLD(timings->thold_mem);
	pmem |= FMC_PMEM_MEMHIZ(timings->thiz);

	/* Set tset/twait/thold/thiz timings in attribut bank */
	patt = FMC_PATT_ATTSET(timings->tset_att);
	patt |= FMC_PATT_ATTWAIT(timings->twait);
	patt |= FMC_PATT_ATTHOLD(timings->thold_att);
	patt |= FMC_PATT_ATTHIZ(timings->thiz);

	writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));
	writel_relaxed(pmem, fmc->io_base + FMC_PMEM(fmc->bank));
	writel_relaxed(patt, fmc->io_base + FMC_PATT(fmc->bank));
}

/* Controller initialization */
static void stm32_fmc_init(struct stm32_fmc *fmc)
{
	u32 pcr = readl_relaxed(fmc->io_base + FMC_PCR(fmc->bank));
	u32 bcr1 = readl_relaxed(fmc->io_base + FMC_BCR1);

	/* Enable wait feature and nand flash memory bank */
	pcr |= FMC_PCR_PWAITEN;
	pcr |= FMC_PCR_PBKEN;

	/* Set buswidth to 8 bits mode for identification */
	pcr &= ~FMC_PCR_PWID_MASK;

	/* Enable FMC controller */
	bcr1 |= FMC_BCR1_FMCEN;

	writel_relaxed(bcr1, fmc->io_base + FMC_BCR1);
	writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));
}

/* Controller configuration */
static void stm32_fmc_setup(struct stm32_fmc *fmc)
{
	struct nand_chip *chip = &fmc->chip;
	u32 pcr = readl_relaxed(fmc->io_base + FMC_PCR(fmc->bank));

	/* Configure in HAMMING by default */
	if (chip->ecc.strength == FMC_ECC_BCH8) {
		pcr |= FMC_PCR_ECCALG;
		pcr |= FMC_PCR_BCHECC;
	} else if (chip->ecc.strength == FMC_ECC_BCH4) {
		pcr |= FMC_PCR_ECCALG;
	}

	/* Set buswidth */
	if (chip->options & NAND_BUSWIDTH_16)
		pcr |= FMC_PCR_PWID(FMC_PCR_PWID_BUSWIDTH_16);

	/* Set ecc sector size */
	pcr &= ~FMC_PCR_ECCSS_MASK;
	pcr |= FMC_PCR_ECCSS(FMC_PCR_ECCSS_512);

	writel_relaxed(pcr, fmc->io_base + FMC_PCR(fmc->bank));
}

/* Controller timings */
static int stm32_fmc_calc_timings(struct stm32_fmc *fmc,
				  const struct nand_sdr_timings *sdrt,
				  struct stm32_fmc_timings *tims)
{
	unsigned long hclk = clk_get_rate(fmc->clk);
	unsigned long hclkp = NSEC_PER_SEC / (hclk / 1000);
	int tar, tclr, thiz, twait, tset_mem, tset_att, thold_mem, thold_att;

	tar = hclkp;
	if (tar < sdrt->tAR_min)
		tar = sdrt->tAR_min;
	tims->tar = DIV_ROUND_UP(tar, hclkp) - 1;
	if (tims->tar > FMC_PCR_TIMING_MASK)
		tims->tar = FMC_PCR_TIMING_MASK;

	tclr = hclkp;
	if (tclr < sdrt->tCLR_min)
		tclr = sdrt->tCLR_min;
	tims->tclr = DIV_ROUND_UP(tclr, hclkp) - 1;
	if (tims->tclr > FMC_PCR_TIMING_MASK)
		tims->tclr = FMC_PCR_TIMING_MASK;

	tims->thiz = FMC_THIZ;
	thiz = (tims->thiz + 1) * hclkp;

	/*
	 * tWAIT > tRP
	 * tWAIT > tWP
	 * tWAIT > tREA + tIO
	 */
	twait = hclkp;
	if (twait < sdrt->tRP_min)
		twait = sdrt->tRP_min;
	if (twait < sdrt->tWP_min)
		twait = sdrt->tWP_min;
	if (twait < sdrt->tREA_max + FMC_TIO)
		twait = sdrt->tREA_max + FMC_TIO;
	tims->twait = DIV_ROUND_UP(twait, hclkp) - 1;
	if (tims->twait > FMC_PMEM_PATT_TIMING_MASK)
		tims->twait = FMC_PMEM_PATT_TIMING_MASK;

	/*
	 * tSETUP_MEM > tCS - tWAIT
	 * tSETUP_MEM > tALS - tWAIT
	 * tSETUP_MEM > tDS - (tWAIT - tHIZ)
	 */
	tset_mem = hclkp;
	if ((sdrt->tCS_min > twait) && (tset_mem < sdrt->tCS_min - twait))
		tset_mem = sdrt->tCS_min - twait;
	if ((sdrt->tALS_min > twait) && (tset_mem < sdrt->tALS_min - twait))
		tset_mem = sdrt->tALS_min - twait;
	if ((twait > thiz) && (sdrt->tDS_min > twait - thiz) &&
	    (tset_mem < sdrt->tDS_min - (twait - thiz)))
		tset_mem = sdrt->tDS_min - (twait - thiz);
	tims->tset_mem = DIV_ROUND_UP(tset_mem, hclkp) - 1;
	if (tims->tset_mem > FMC_PMEM_PATT_TIMING_MASK)
		tims->tset_mem = FMC_PMEM_PATT_TIMING_MASK;

	/*
	 * tHOLD_MEM > tCH
	 * tHOLD_MEM > tREH - tSETUP_MEM
	 * tHOLD_MEM > max(tRC, tWC) - (tSETUP_MEM + tWAIT)
	 */
	thold_mem = hclkp;
	if (thold_mem < sdrt->tCH_min)
		thold_mem = sdrt->tCH_min;
	if ((sdrt->tREH_min > tset_mem) &&
	    (thold_mem < sdrt->tREH_min - tset_mem))
		thold_mem = sdrt->tREH_min - tset_mem;
	if ((sdrt->tRC_min > tset_mem + twait) &&
	    (thold_mem < sdrt->tRC_min - (tset_mem + twait)))
		thold_mem = sdrt->tRC_min - (tset_mem + twait);
	if ((sdrt->tWC_min > tset_mem + twait) &&
	    (thold_mem < sdrt->tWC_min - (tset_mem + twait)))
		thold_mem = sdrt->tWC_min - (tset_mem + twait);
	tims->thold_mem = DIV_ROUND_UP(thold_mem, hclkp) - 1;
	if (tims->thold_mem > FMC_PMEM_PATT_TIMING_MASK)
		tims->thold_mem = FMC_PMEM_PATT_TIMING_MASK;

	/*
	 * tSETUP_ATT > tCS - tWAIT
	 * tSETUP_ATT > tCLS - tWAIT
	 * tSETUP_ATT > tALS - tWAIT
	 * tSETUP_ATT > tRHW - tHOLD_MEM
	 * tSETUP_ATT > tDS - (tWAIT - tHIZ)
	 */
	tset_att = hclkp;
	if ((sdrt->tCS_min > twait) && (tset_att < sdrt->tCS_min - twait))
		tset_att = sdrt->tCS_min - twait;
	if ((sdrt->tCLS_min > twait) && (tset_att < sdrt->tCLS_min - twait))
		tset_att = sdrt->tCLS_min - twait;
	if ((sdrt->tALS_min > twait) && (tset_att < sdrt->tALS_min - twait))
		tset_att = sdrt->tALS_min - twait;
	if ((sdrt->tRHW_min > thold_mem) &&
	    (tset_att < sdrt->tRHW_min - thold_mem))
		tset_att = sdrt->tRHW_min - thold_mem;
	if ((twait > thiz) && (sdrt->tDS_min > twait - thiz) &&
	    (tset_att < sdrt->tDS_min - (twait - thiz)))
		tset_att = sdrt->tDS_min - (twait - thiz);
	tims->tset_att = DIV_ROUND_UP(tset_att, hclkp) - 1;
	if (tims->tset_att > FMC_PMEM_PATT_TIMING_MASK)
		tims->tset_att = FMC_PMEM_PATT_TIMING_MASK;

	/*
	 * tHOLD_ATT > tALH
	 * tHOLD_ATT > tCH
	 * tHOLD_ATT > tCLH
	 * tHOLD_ATT > tCOH
	 * tHOLD_ATT > tDH
	 * tHOLD_ATT > tWB + tIO + tSYNC - tSETUP_MEM
	 * tHOLD_ATT > tADL - tSETUP_MEM
	 * tHOLD_ATT > tWH - tSETUP_MEM
	 * tHOLD_ATT > tWHR - tSETUP_MEM
	 * tHOLD_ATT > tRC - (tSETUP_ATT + tWAIT)
	 * tHOLD_ATT > tWC - (tSETUP_ATT + tWAIT)
	 */
	thold_att = hclkp;
	if (thold_att < sdrt->tALH_min)
		thold_att = sdrt->tALH_min;
	if (thold_att < sdrt->tCH_min)
		thold_att = sdrt->tCH_min;
	if (thold_att < sdrt->tCLH_min)
		thold_att = sdrt->tCLH_min;
	if (thold_att < sdrt->tCOH_min)
		thold_att = sdrt->tCOH_min;
	if (thold_att < sdrt->tDH_min)
		thold_att = sdrt->tDH_min;
	if ((sdrt->tWB_max + FMC_TIO + FMC_TSYNC > tset_mem) &&
	    (thold_att < sdrt->tWB_max + FMC_TIO + FMC_TSYNC - tset_mem))
		thold_att = sdrt->tWB_max + FMC_TIO + FMC_TSYNC - tset_mem;
	if ((sdrt->tADL_min > tset_mem) &&
	    (thold_att < sdrt->tADL_min - tset_mem))
		thold_att = sdrt->tADL_min - tset_mem;
	if ((sdrt->tWH_min > tset_mem) &&
	    (thold_att < sdrt->tWH_min - tset_mem))
		thold_att = sdrt->tWH_min - tset_mem;
	if ((sdrt->tWHR_min > tset_mem) &&
	    (thold_att < sdrt->tWHR_min - tset_mem))
		thold_att = sdrt->tWHR_min - tset_mem;
	if ((sdrt->tRC_min > tset_att + twait) &&
	    (thold_att < sdrt->tRC_min - (tset_att + twait)))
		thold_att = sdrt->tRC_min - (tset_att + twait);
	if ((sdrt->tWC_min > tset_att + twait) &&
	    (thold_att < sdrt->tWC_min - (tset_att + twait)))
		thold_att = sdrt->tWC_min - (tset_att + twait);
	tims->thold_att = DIV_ROUND_UP(thold_att, hclkp) - 1;
	if (tims->thold_att > FMC_PMEM_PATT_TIMING_MASK)
		tims->thold_att = FMC_PMEM_PATT_TIMING_MASK;

	return 0;
}

static int stm32_fmc_setup_interface(struct mtd_info *mtd, int chipnr,
				     const struct nand_data_interface *conf)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct stm32_fmc *fmc = nand_get_controller_data(chip);
	struct stm32_fmc_timings tims;
	const struct nand_sdr_timings *sdrt;
	int ret;

	sdrt = nand_get_sdr_timings(conf);
	if (IS_ERR(sdrt))
		return PTR_ERR(sdrt);

	ret = stm32_fmc_calc_timings(fmc, sdrt, &tims);
	if (ret)
		return ret;

	if (chipnr == NAND_DATA_IFACE_CHECK_ONLY)
		return 0;

	/* Save and apply timings */
	memcpy(&fmc->timings, &tims, sizeof(tims));
	stm32_fmc_timings_init(fmc);

	return 0;
}

/* DMA configuration */
static int stm32_fmc_nand_dma_setup(struct stm32_fmc *fmc, u8 nb_sect,
				    phys_addr_t data_phys_base,
				    phys_addr_t io_phys_base)
{
	struct nand_chip *chip = &fmc->chip;
	struct dma_slave_config dma_cfg;
	int ret;

	/*
	 * Data DMA is found => sequencer mode will be used
	 * Else manual mode is used
	 */
	fmc->dma_data_ch = dma_request_slave_channel(fmc->dev, "rxtx");
	if (fmc->dma_data_ch) {
		memset(&dma_cfg, 0, sizeof(dma_cfg));
		dma_cfg.src_addr = data_phys_base;
		dma_cfg.dst_addr = data_phys_base;
		dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_cfg.src_maxburst = 32;
		dma_cfg.dst_maxburst = 32;

		ret = dmaengine_slave_config(fmc->dma_data_ch, &dma_cfg);
		if (ret) {
			dev_err(fmc->dev, "data DMA engine slave config failed\n");
			return ret;
		}

		ret = sg_alloc_table(&fmc->dma_data_sg, nb_sect, GFP_KERNEL);
		if (ret)
			return ret;

		fmc->dma_ecc_ch = dma_request_slave_channel(fmc->dev, "ecc");
		if (fmc->dma_ecc_ch) {
			/*
			 * HAMMING: we read HECCR register
			 * BCH4/BCH8: we read BCHDSRSx registers
			 */
			memset(&dma_cfg, 0, sizeof(dma_cfg));
			dma_cfg.src_addr = io_phys_base;
			dma_cfg.src_addr += chip->ecc.strength == FMC_ECC_HAM ?
					    FMC_HECCR(fmc->bank) : FMC_BCHDSR0;
			dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

			ret = dmaengine_slave_config(fmc->dma_ecc_ch,
						     &dma_cfg);
			if (ret) {
				dev_err(fmc->dev, "ecc DMA engine slave config failed\n");
				return ret;
			}

			ret = sg_alloc_table(&fmc->dma_ecc_sg,
					     nb_sect, GFP_KERNEL);
			if (ret)
				return ret;

			/* Calculate ecc length needed for one sector */
			fmc->dma_ecc_len = chip->ecc.strength == FMC_ECC_HAM ?
					   FMC_HECCR_LEN : FMC_BCHDSRS_LEN;

			/* Allocate a buffer to store ecc status registers */
			fmc->ecc_buf = devm_kzalloc(fmc->dev,
						    fmc->dma_ecc_len * nb_sect,
						    GFP_KERNEL);
			if (!fmc->ecc_buf)
				return -ENOMEM;
		} else {
			dev_err(fmc->dev, "ecc DMA not defined in the device tree\n");
			return -ENOENT;
		}
	}

	return 0;
}

/* NAND callbacks setup */
static void stm32_fmc_nand_callbacks_setup(struct stm32_fmc *fmc)
{
	struct nand_chip *chip = &fmc->chip;

	/*
	 * Specific callbacks to read/write a page depending on
	 * the mode (manual/sequencer) and the algo used (HAMMING, BCH).
	 */
	if (fmc->dma_data_ch) {
		/* DMA => use sequencer mode callbacks */
		chip->ecc.write_page = stm32_fmc_sequencer_write_page;
		chip->ecc.read_page = stm32_fmc_sequencer_read_page;
		chip->ecc.write_page_raw = stm32_fmc_sequencer_write_page_raw;
		chip->ecc.read_page_raw = stm32_fmc_sequencer_read_page_raw;
		chip->options |= NAND_USE_BOUNCE_BUFFER;
		chip->ecc.options |= NAND_ECC_CUSTOM_PAGE_ACCESS;
	} else {
		/* No DMA => use manual mode callbacks */
		chip->ecc.hwctl = stm32_fmc_hwctl;
		if (chip->ecc.strength == FMC_ECC_HAM) {
			/* HAMMING is used */
			chip->ecc.calculate = stm32_fmc_ham_calculate;
			chip->ecc.correct = stm32_fmc_ham_correct;
		} else {
			/* BCH is used */
			chip->ecc.read_page = stm32_fmc_read_page;
			chip->ecc.calculate = stm32_fmc_bch_calculate;
			chip->ecc.correct = stm32_fmc_bch_correct;
		}
	}

	/* Specific configurations depending on the algo used (HAMMING, BCH) */
	if (chip->ecc.strength == FMC_ECC_HAM) {
		chip->ecc.bytes = chip->options & NAND_BUSWIDTH_16 ? 4 : 3;
		chip->ecc.options |= NAND_ECC_GENERIC_ERASED_CHECK;
	} else if (chip->ecc.strength == FMC_ECC_BCH8) {
		chip->ecc.bytes = chip->options & NAND_BUSWIDTH_16 ? 14 : 13;
	} else {
		chip->ecc.bytes = chip->options & NAND_BUSWIDTH_16 ? 8 : 7;
	}
}

/* FMC layout */
static int stm32_fmc_nand_ooblayout_ecc(struct mtd_info *mtd, int section,
					struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	if (section)
		return -ERANGE;

	oobregion->length = ecc->total;
	oobregion->offset = FMC_BBM_LEN;

	return 0;
}

static int stm32_fmc_nand_ooblayout_free(struct mtd_info *mtd, int section,
					 struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	if (section)
		return -ERANGE;

	oobregion->length = mtd->oobsize - ecc->total - FMC_BBM_LEN;
	oobregion->offset = ecc->total + FMC_BBM_LEN;

	return 0;
}

const struct mtd_ooblayout_ops stm32_fmc_nand_ooblayout_ops = {
	.ecc = stm32_fmc_nand_ooblayout_ecc,
	.free = stm32_fmc_nand_ooblayout_free,
};

static bool stm32_fmc_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node;
	struct stm32_fmc *fmc = platform_get_drvdata(pdev);
	int ret;

	/* Get the bank used, bank 3 is used by default */
	if (of_property_read_u32(dn, "st,fmc_bank_used", &fmc->bank))
		fmc->bank = 3;

	/* Common bank timings */
	ret = of_property_read_u8_array(dn, "st,fmc_timings",
					(u8 *)&fmc->timings,
					sizeof(fmc->timings));

	return ret ? false : true;
}

static int stm32_fmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rstc;
	struct stm32_fmc *fmc;
	struct resource *res;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	phys_addr_t data_phys_base, io_phys_base;
	u16 ecc_space_needed;
	u8 nb_sect;
	int ret, irq;
	bool timings_def;

	fmc = devm_kzalloc(dev, sizeof(*fmc), GFP_KERNEL);
	if (!fmc)
		return -ENOMEM;

	fmc->dev = dev;
	platform_set_drvdata(pdev, fmc);

	timings_def = stm32_fmc_parse_dt(pdev);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fmc_regs");
	fmc->io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(fmc->io_base))
		return PTR_ERR(fmc->io_base);

	io_phys_base = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fmc_common");
	fmc->common_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(fmc->common_base))
		return PTR_ERR(fmc->common_base);

	data_phys_base = res->start + FMC_DATA_SECTION;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fmc_attrib");
	fmc->attrib_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(fmc->attrib_base))
		return PTR_ERR(fmc->attrib_base);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev, irq, stm32_fmc_irq, 0,
			       dev_name(dev), fmc);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		return ret;
	}

	init_completion(&fmc->complete);
	init_completion(&fmc->dma_data_complete);
	init_completion(&fmc->dma_ecc_complete);

	fmc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(fmc->clk))
		return PTR_ERR(fmc->clk);

	ret = clk_prepare_enable(fmc->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		return ret;
	}

	rstc = devm_reset_control_get(dev, NULL);
	if (!IS_ERR(rstc)) {
		reset_control_assert(rstc);
		reset_control_deassert(rstc);
	}

	mtd = nand_to_mtd(&fmc->chip);
	chip = &fmc->chip;
	nand_set_controller_data(chip, fmc);
	nand_set_flash_node(chip, dev->of_node);
	mtd->dev.parent = dev;

	/* Set NAND IO addresses and command/ready functions */
	chip->IO_ADDR_R = fmc->common_base + FMC_DATA_SECTION;
	chip->IO_ADDR_W = fmc->common_base + FMC_DATA_SECTION;
	chip->cmd_ctrl = stm32_fmc_cmd_ctrl;
	chip->chip_delay = FMC_RB_DELAY_US;
	chip->options |= NAND_BUSWIDTH_AUTO | NAND_NO_SUBPAGE_WRITE;

	/* FMC init routine */
	stm32_fmc_init(fmc);
	if (timings_def)
		stm32_fmc_timings_init(fmc);
	else
		chip->setup_data_interface = stm32_fmc_setup_interface;

	/*
	 * Only NAND_ECC_HW mode is actually supported
	 * HAMMING => ecc.strength = 1
	 * BCH4 => ecc.strength = 4
	 * BCH8 => ecc.strength = 8 (default)
	 */
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.size = 512;
	chip->ecc.strength = FMC_ECC_BCH8;

	/* Scan to find existence of the device */
	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret)
		goto err_scan_ident;

	if (chip->ecc.size != 512) {
		dev_err(dev, "ecc_step_size is not well defined in the DT\n");
		ret = -EINVAL;
		goto err_scan_ident;
	}

	if ((chip->ecc.strength != FMC_ECC_BCH8) &&
	    (chip->ecc.strength != FMC_ECC_BCH4) &&
	    (chip->ecc.strength != FMC_ECC_HAM)) {
		dev_err(dev, "ecc_strength is not well defined in the DT\n");
		ret = -EINVAL;
		goto err_scan_ident;
	}

	nb_sect = mtd->writesize / chip->ecc.size;
	if (nb_sect > FMC_MAX_SG_COUNT) {
		dev_err(dev, "nand page size is not supported\n");
		ret = -EINVAL;
		goto err_scan_ident;
	}

	if (chip->bbt_options & NAND_BBT_USE_FLASH)
		chip->bbt_options |= NAND_BBT_NO_OOB;

	/* FMC setup routine */
	stm32_fmc_setup(fmc);

	/* DMA setup */
	ret = stm32_fmc_nand_dma_setup(fmc, nb_sect, data_phys_base,
				       io_phys_base);
	if (ret)
		goto err_dma_setup;

	/* NAND callbacks setup */
	stm32_fmc_nand_callbacks_setup(fmc);

	/*
	 * Check if parity bits can be stored in OOB
	 * The first 2 bytes are skipped (BBM)
	 */
	ecc_space_needed = chip->ecc.bytes * nb_sect;
	if (mtd->oobsize < (ecc_space_needed + FMC_BBM_LEN)) {
		dev_err(dev, "not enough OOB bytes required = %d, available=%d\n",
			ecc_space_needed, mtd->oobsize);
		ret = -EINVAL;
		goto err_dma_setup;
	}

	/* Define ECC layout */
	mtd_set_ooblayout(mtd, &stm32_fmc_nand_ooblayout_ops);

	/* Scan the device to fill MTD data-structures */
	if (nand_scan_tail(mtd)) {
		ret = -ENXIO;
		goto err_dma_setup;
	}

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret)
		goto err_dma_setup;

	return 0;

err_dma_setup:
	if (fmc->dma_ecc_ch)
		dma_release_channel(fmc->dma_ecc_ch);

	if (fmc->dma_data_ch)
		dma_release_channel(fmc->dma_data_ch);

	sg_free_table(&fmc->dma_data_sg);
	sg_free_table(&fmc->dma_ecc_sg);

err_scan_ident:
	clk_disable_unprepare(fmc->clk);

	return ret;
}

static int stm32_fmc_remove(struct platform_device *pdev)
{
	struct stm32_fmc *fmc = platform_get_drvdata(pdev);
	struct mtd_info *mtd = nand_to_mtd(&fmc->chip);

	if (fmc->dma_ecc_ch)
		dma_release_channel(fmc->dma_ecc_ch);

	if (fmc->dma_data_ch)
		dma_release_channel(fmc->dma_data_ch);

	sg_free_table(&fmc->dma_data_sg);
	sg_free_table(&fmc->dma_ecc_sg);

	clk_disable_unprepare(fmc->clk);

	nand_release(mtd);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stm32_fmc_suspend(struct device *dev)
{
	struct stm32_fmc *fmc = dev_get_drvdata(dev);

	clk_disable_unprepare(fmc->clk);

	return 0;
}

static int stm32_fmc_resume(struct device *dev)
{
	struct stm32_fmc *fmc = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(fmc->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		return ret;
	}

	stm32_fmc_init(fmc);
	stm32_fmc_timings_init(fmc);
	stm32_fmc_setup(fmc);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(stm32_fmc_pm_ops, stm32_fmc_suspend, stm32_fmc_resume);

static const struct of_device_id stm32_fmc_match[] = {
	{.compatible = "st,stm32mp1-fmc"},
	{}
};
MODULE_DEVICE_TABLE(of, stm32_fmc_match);

static struct platform_driver stm32_fmc_driver = {
	.probe	= stm32_fmc_probe,
	.remove	= stm32_fmc_remove,
	.driver	= {
		.name = "stm32-fmc",
		.of_match_table = stm32_fmc_match,
		.pm = &stm32_fmc_pm_ops,
	},
};
module_platform_driver(stm32_fmc_driver);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Christophe Kerello <christophe.kerello@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 fmc nand driver");
MODULE_LICENSE("GPL v2");
