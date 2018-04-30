/*
 * Driver for stm32 quadspi controller
 *
 * Copyright (C) 2017 STMicroelectronics
 * Copyright (C) 2018 Exceet Electronics GmbH
 *
 * Authors: Ludovic Barre <ludovic.barre@st.com>
 *          Frieder Schrempf <frieder.schrempf@exceet.de>
 *
 * License terms: GNU General Public License (GPL), version 2
 */
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define QUADSPI_CR		0x00
#define CR_EN			BIT(0)
#define CR_ABORT		BIT(1)
#define CR_DMAEN		BIT(2)
#define CR_TCEN			BIT(3)
#define CR_SSHIFT		BIT(4)
#define CR_DFM			BIT(6)
#define CR_FSEL			BIT(7)
#define CR_FTHRES_SHIFT		8
#define CR_FTHRES_MASK		GENMASK(12, 8)
#define CR_FTHRES(n)		(((n) << CR_FTHRES_SHIFT) & CR_FTHRES_MASK)
#define CR_TEIE			BIT(16)
#define CR_TCIE			BIT(17)
#define CR_FTIE			BIT(18)
#define CR_SMIE			BIT(19)
#define CR_TOIE			BIT(20)
#define CR_PRESC_SHIFT		24
#define CR_PRESC_MASK		GENMASK(31, 24)
#define CR_PRESC(n)		(((n) << CR_PRESC_SHIFT) & CR_PRESC_MASK)

#define QUADSPI_DCR		0x04
#define DCR_CSHT_SHIFT		8
#define DCR_CSHT_MASK		GENMASK(10, 8)
#define DCR_CSHT(n)		(((n) << DCR_CSHT_SHIFT) & DCR_CSHT_MASK)
#define DCR_FSIZE_SHIFT		16
#define DCR_FSIZE_MASK		GENMASK(20, 16)
#define DCR_FSIZE(n)		(((n) << DCR_FSIZE_SHIFT) & DCR_FSIZE_MASK)

#define QUADSPI_SR		0x08
#define SR_TEF			BIT(0)
#define SR_TCF			BIT(1)
#define SR_FTF			BIT(2)
#define SR_SMF			BIT(3)
#define SR_TOF			BIT(4)
#define SR_BUSY			BIT(5)
#define SR_FLEVEL_SHIFT		8
#define SR_FLEVEL_MASK		GENMASK(13, 8)

#define QUADSPI_FCR		0x0c
#define FCR_CTCF		BIT(1)

#define QUADSPI_DLR		0x10

#define CCR_BUSWIDTH(x)		fls(x)

#define QUADSPI_CCR		0x14
#define CCR_INST_SHIFT		0
#define CCR_INST_MASK		GENMASK(7, 0)
#define CCR_INST(n)		(((n) << CCR_INST_SHIFT) & CCR_INST_MASK)
#define CCR_IMODE_SHIFT		8
#define CCR_IMODE(x)		(CCR_BUSWIDTH(x) << CCR_IMODE_SHIFT)
#define CCR_ADMODE_SHIFT	10
#define CCR_ADMODE(x)		(CCR_BUSWIDTH(x) << CCR_ADMODE_SHIFT)
#define CCR_ADSIZE_SHIFT	12
#define CCR_ADSIZE_MASK		GENMASK(13, 12)
#define CCR_ADSIZE(n)		(((n) << CCR_ADSIZE_SHIFT) & CCR_ADSIZE_MASK)
#define CCR_ABMODE_SHIFT	14
#define CCR_ABMODE(x)		(CCR_BUSWIDTH(x) << CCR_ABMODE_SHIFT)
#define CCR_ABSIZE_8		(0U << 16)
#define CCR_ABSIZE_16		(1U << 16)
#define CCR_ABSIZE_24		(2U << 16)
#define CCR_ABSIZE_32		(3U << 16)
#define CCR_DCYC_SHIFT		18
#define CCR_DCYC_MASK		GENMASK(22, 18)
#define CCR_DCYC(n)		(((n) << CCR_DCYC_SHIFT) & CCR_DCYC_MASK)
#define CCR_DMODE_SHIFT		24
#define CCR_DMODE(x)		(CCR_BUSWIDTH(x) << CCR_DMODE_SHIFT)
#define CCR_FMODE_INDW		(0U << 26)
#define CCR_FMODE_INDR		(1U << 26)
#define CCR_FMODE_APM		(2U << 26)
#define CCR_FMODE_MM		(3U << 26)

#define QUADSPI_AR		0x18
#define QUADSPI_ABR		0x1c
#define QUADSPI_DR		0x20
#define QUADSPI_PSMKR		0x24
#define QUADSPI_PSMAR		0x28
#define QUADSPI_PIR		0x2c
#define QUADSPI_LPTR		0x30
#define LPTR_DFT_TIMEOUT	0x10

#define FSIZE_VAL(size)		(__fls(size) - 1)

#define STM32_MAX_MMAP_SZ	SZ_256M
#define STM32_MAX_CHIP		2

#define STM32_QSPI_FIFO_SZ	32
#define STM32_QSPI_FIFO_TIMEOUT_US 30000
#define STM32_QSPI_BUSY_TIMEOUT_US 100000

struct stm32_qspi {
	struct device *dev;
	void __iomem *io_base;
	void __iomem *mm_base;
	resource_size_t mm_size;
	struct clk *clk;
	u32 clk_rate;
	struct completion cmd_completion;

	/*
	 * to protect device configuration, could be different between
	 * 2 flash access (bk1, bk2)
	 */
	struct mutex lock;
};

static int stm32_qspi_wait_cmd(struct stm32_qspi *qspi)
{
	u32 cr;
	int err = 0;

	if (readl_relaxed(qspi->io_base + QUADSPI_SR) & SR_TCF)
		return 0;

	reinit_completion(&qspi->cmd_completion);
	cr = readl_relaxed(qspi->io_base + QUADSPI_CR);
	writel_relaxed(cr | CR_TCIE, qspi->io_base + QUADSPI_CR);

	if (!wait_for_completion_interruptible_timeout(&qspi->cmd_completion,
						       msecs_to_jiffies(1000)))
		err = -ETIMEDOUT;

	writel_relaxed(cr, qspi->io_base + QUADSPI_CR);
	return err;
}

static int stm32_qspi_wait_nobusy(struct stm32_qspi *qspi)
{
	u32 sr;

	return readl_relaxed_poll_timeout(qspi->io_base + QUADSPI_SR, sr,
					  !(sr & SR_BUSY), 10,
					  STM32_QSPI_BUSY_TIMEOUT_US);
}

static int stm32_qspi_check_buswidth(struct stm32_qspi *qspi, u8 width)
{
	switch(width) {
	case 1:
	case 2:
	case 4:
		return 0;

	default:
		break;
	}

	return -ENOTSUPP;
}

static bool stm32_qspi_supports_op(struct spi_mem *mem,
				   const struct spi_mem_op *op)
{
	struct stm32_qspi *qspi = spi_controller_get_devdata(mem->spi->master);
	int ret;

	ret = stm32_qspi_check_buswidth(qspi, op->cmd.buswidth);
	
	if (op->addr.nbytes)
		ret |= stm32_qspi_check_buswidth(qspi, op->addr.buswidth);

	if (op->dummy.nbytes)
		ret |= stm32_qspi_check_buswidth(qspi, op->dummy.buswidth);

	if (op->data.nbytes)
		ret |= stm32_qspi_check_buswidth(qspi, op->data.buswidth);

	if (ret)
		return false;

	/* Max 32-bit address. */
	if (op->addr.nbytes > 4)
		return false;

	/* Max 31 dummy clock cycles. */
	if (op->dummy.nbytes * 8 / op->dummy.buswidth > 31)
		return false;

	return true;
}

static void stm32_qspi_select_mem(struct stm32_qspi *qspi, struct spi_device *spi)
{
	u32 presc, temp;
	static bool selected = false;

	if (selected)
		return;

	temp = readl_relaxed(qspi->io_base + QUADSPI_DCR) & ~DCR_FSIZE_MASK;
	temp |= DCR_FSIZE(0x1b); // set fixed size of 256M
	writel_relaxed(temp, qspi->io_base + QUADSPI_DCR);

	presc = DIV_ROUND_UP(qspi->clk_rate, spi->max_speed_hz) - 1;

	temp = readl_relaxed(qspi->io_base + QUADSPI_CR);
	temp &= ~CR_PRESC_MASK & ~CR_FSEL;
	temp |= CR_PRESC(presc);
	temp |= spi->chip_select ? CR_FSEL:0;
	writel_relaxed(temp, qspi->io_base + QUADSPI_CR);

	selected = true;
}

static void stm32_qspi_read_fifo(u8 *val, void __iomem *addr)
{
	*val = readb_relaxed(addr);
}

static void stm32_qspi_write_fifo(u8 *val, void __iomem *addr)
{
	writeb_relaxed(*val, addr);
}

static int stm32_qspi_tx_poll(struct stm32_qspi *qspi,
			      const struct spi_mem_op *op)
{
	void (*tx_fifo)(u8 *, void __iomem *);
	u32 len = op->data.nbytes, sr;
	u8 *buf = op->data.buf.in;
	int ret;

	if (op->data.dir == SPI_MEM_DATA_OUT)
		tx_fifo = stm32_qspi_write_fifo;
	else
		tx_fifo = stm32_qspi_read_fifo;

	while (len--) {
		ret = readl_relaxed_poll_timeout(qspi->io_base + QUADSPI_SR,
						 sr, (sr & SR_FTF), 10,
						 STM32_QSPI_FIFO_TIMEOUT_US);
		if (ret) {
			dev_err(qspi->dev, "fifo timeout (stat:%#x)\n", sr);
			return ret;
		}
		tx_fifo(buf++, qspi->io_base + QUADSPI_DR);
	}

	return 0;
}

static int stm32_qspi_exec_op(struct spi_mem *mem,
			      const struct spi_mem_op *op)
{
	struct stm32_qspi *qspi = spi_controller_get_devdata(mem->spi->master);
	u32 reg = 0;
	int err;

	mutex_lock(&qspi->lock);

	err = stm32_qspi_wait_nobusy(qspi);
	if (err)
		goto abort;

	stm32_qspi_select_mem(qspi, mem->spi);

	// set data size, direction and buswidth
	if(op->data.nbytes) {
		writel_relaxed(op->data.nbytes - 1, qspi->io_base + QUADSPI_DLR);
		reg = op->data.dir == SPI_MEM_DATA_IN ?
		                      CCR_FMODE_INDR : CCR_FMODE_INDW;
		reg |= CCR_DMODE(op->data.buswidth);
	}
	else
		writel_relaxed(0, qspi->io_base + QUADSPI_DLR);

	// set instruction buswidth
	reg |= CCR_IMODE(op->cmd.buswidth);;

	// set address size and buswidth
	if(op->addr.nbytes) {
		reg |= CCR_ADSIZE(op->addr.nbytes - 1);
		reg |= CCR_ADMODE(op->addr.buswidth);
	}

	// set number of dummy clock cycles
	reg |= CCR_DCYC(op->dummy.nbytes * 8 / op->dummy.buswidth);

	// set instruction opcode
	reg |= CCR_INST(op->cmd.opcode);
	writel_relaxed(reg, qspi->io_base + QUADSPI_CCR);

	// set address
	if(op->addr.nbytes)
		writel_relaxed(op->addr.val, qspi->io_base + QUADSPI_AR);

	// send/receive data
	if(op->data.nbytes) {
		err = stm32_qspi_tx_poll(qspi, op);
		if (err)
			goto abort;
	}

	err = stm32_qspi_wait_cmd(qspi);
	if (err)
		goto abort;

	writel_relaxed(FCR_CTCF, qspi->io_base + QUADSPI_FCR);
	mutex_unlock(&qspi->lock);

	return err;

abort:
	reg = readl_relaxed(qspi->io_base + QUADSPI_CR) | CR_ABORT;
	writel_relaxed(reg, qspi->io_base + QUADSPI_CR);
	mutex_unlock(&qspi->lock);

	dev_err(qspi->dev, "%s abort err:%d\n", __func__, err);
	return err;
}

static int stm32_qspi_adjust_op_size(struct spi_mem *mem,
				     struct spi_mem_op *op)
{
	return 0;
}

static irqreturn_t stm32_qspi_irq(int irq, void *dev_id)
{
	struct stm32_qspi *qspi = (struct stm32_qspi *)dev_id;
	u32 cr, sr, fcr = 0;

	cr = readl_relaxed(qspi->io_base + QUADSPI_CR);
	sr = readl_relaxed(qspi->io_base + QUADSPI_SR);

	if ((cr & CR_TCIE) && (sr & SR_TCF)) {
		/* tx complete */
		fcr |= FCR_CTCF;
		complete(&qspi->cmd_completion);
	} else {
		dev_info_ratelimited(qspi->dev, "spurious interrupt\n");
	}

	writel_relaxed(fcr, qspi->io_base + QUADSPI_FCR);

	return IRQ_HANDLED;
}

static const struct spi_controller_mem_ops stm32_qspi_mem_ops = {
	.adjust_op_size = stm32_qspi_adjust_op_size,
	.supports_op = stm32_qspi_supports_op,
	.exec_op = stm32_qspi_exec_op,
};

static int stm32_qspi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct device *dev = &pdev->dev;
	struct reset_control *rstc;
	struct stm32_qspi *qspi;
	struct resource *res;
	int ret, irq;
	u32 num_chips;

	ctlr = spi_alloc_master(dev, sizeof(*qspi));
	if (!ctlr)
		return -ENOMEM;

	ctlr->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD |
			  SPI_TX_DUAL | SPI_TX_QUAD;
	qspi = spi_controller_get_devdata(ctlr);
	qspi->dev = dev;

	platform_set_drvdata(pdev, qspi);

	num_chips = of_get_child_count(dev->of_node);
	if (num_chips > STM32_MAX_CHIP) {
		ret = -ENODEV;
		goto err_put_ctrl;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi");
	qspi->io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(qspi->io_base)) {
		ret = PTR_ERR(qspi->io_base);
		goto err_put_ctrl;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi_mm");
	qspi->mm_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(qspi->mm_base)) {
		ret = PTR_ERR(qspi->io_base);
		goto err_put_ctrl;
	}

	qspi->mm_size = resource_size(res);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev, irq, stm32_qspi_irq, 0,
			       dev_name(dev), qspi);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto err_put_ctrl;
	}

	init_completion(&qspi->cmd_completion);

	qspi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(qspi->clk)) {
		ret = PTR_ERR(qspi->clk);
		goto err_put_ctrl;
	}

	qspi->clk_rate = clk_get_rate(qspi->clk);
	if (!qspi->clk_rate) {
		ret = -EINVAL;
		goto err_put_ctrl;
	}

	ret = clk_prepare_enable(qspi->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		goto err_put_ctrl;
	}

	rstc = devm_reset_control_get(dev, NULL);
	if (!IS_ERR(rstc)) {
		reset_control_assert(rstc);
		udelay(2);
		reset_control_deassert(rstc);
	}

	ctlr->bus_num = -1;
	ctlr->num_chipselect = 2;
	ctlr->mem_ops = &stm32_qspi_mem_ops;

	mutex_init(&qspi->lock);

	writel_relaxed(LPTR_DFT_TIMEOUT, qspi->io_base + QUADSPI_LPTR);

	writel_relaxed(CR_FTHRES(3) | CR_TCEN | CR_SSHIFT
		       | CR_EN, qspi->io_base + QUADSPI_CR);

	writel_relaxed(DCR_CSHT(1), qspi->io_base + QUADSPI_DCR);

	ctlr->dev.of_node = pdev->dev.of_node;

	ret = spi_register_controller(ctlr);
	if (ret)
		goto err_disable_clk;

	return 0;

err_disable_clk:
	clk_disable_unprepare(qspi->clk);

err_put_ctrl:
	spi_controller_put(ctlr);

	dev_err(dev, "STM32 QuadSPI probe failed\n");
	return ret;
}

static int stm32_qspi_remove(struct platform_device *pdev)
{
	struct stm32_qspi *qspi = platform_get_drvdata(pdev);

	/* disable qspi */
	writel_relaxed(0, qspi->io_base + QUADSPI_CR);

	//stm32_qspi_mtd_free(qspi);
	mutex_destroy(&qspi->lock);

	clk_disable_unprepare(qspi->clk);
	return 0;
}

static const struct of_device_id stm32_qspi_match[] = {
	{.compatible = "st,stm32f469-qspi"},
	{}
};
MODULE_DEVICE_TABLE(of, stm32_qspi_match);

static struct platform_driver stm32_qspi_driver = {
	.probe	= stm32_qspi_probe,
	.remove	= stm32_qspi_remove,
	.driver	= {
		.name = "stm32-quadspi",
		.of_match_table = stm32_qspi_match,
	},
};
module_platform_driver(stm32_qspi_driver);

MODULE_AUTHOR("Ludovic Barre <ludovic.barre@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 quad spi driver");
MODULE_LICENSE("GPL v2");
