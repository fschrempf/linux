/*
 * Driver for stm32 ipcc
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author(s): Ludovic Barre author <ludovic.barre@st.com>.
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
 *
 * You should have received a copy of the GNU General Public License along with
 * This program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#define IPCC_XCR		0x000
#define XCR_RXOIE		BIT(0)
#define XCR_TXOIE		BIT(16)

#define IPCC_XIMR		0x004
#define IPCC_XSCR		0x008
#define IPCC_XTOYSR		0x00c

#define IPCC_PROC_OFFST		0x010

#define IPCC_HWCFGR		0x3f0
#define IPCFGR_BUFF_SHIFT	0
#define IPCFGR_BUFF_MASK	GENMASK(7, 0)

#define IPCC_VER		0x3f4
#define VER_MINREV_SHIFT	0
#define VER_MINREV_MASK		GENMASK(3, 0)
#define VER_MAJREV_SHIFT	4
#define VER_MAJREV_MASK		GENMASK(7, 4)

#define RX_BIT_SHIFT		0
#define RX_BIT_MASK		GENMASK(15, 0)
#define RX_BIT_BUFF(buf_idx)	BIT(buf_idx)
#define TX_BIT_SHIFT		16
#define TX_BIT_MASK		GENMASK(31, 16)
#define TX_BIT_BUFF(buf_idx)	BIT(TX_BIT_SHIFT + (buf_idx))

#define STM32_MAX_PROCS		2
#define STM32_MAX_BUF		16
#define STM32_INAME_SZ		32

enum {
	IPCC_IRQ_RX,
	IPCC_IRQ_TX_ACK,
	IPCC_IRQ_WKUP,
	IPCC_IRQ_NUM,
};

struct stm32_chan {
	struct stm32_proc *proc;
	u32 buf_idx;
};

struct stm32_proc {
	struct stm32_ipcc *ipcc;
	void __iomem *io_proc;
	char irqs_name[IPCC_IRQ_NUM][STM32_INAME_SZ];
	int irqs[IPCC_IRQ_NUM];
	/* to protect irq request/free and used */
	struct mutex lock;
	u32 used;
	u32 id;
};

struct stm32_ipcc {
	void __iomem *io_base;
	struct mbox_controller controller;
	struct clk *clk;
	struct stm32_proc *procs;
	struct stm32_chan *chans;
	u32 n_buffs;
	u32 ip_ver;
	int wakeup;
};

static inline void stm32_ipcc_set_bits(void __iomem *reg, u32 mask)
{
	writel_relaxed(readl_relaxed(reg) | mask, reg);
}

static inline void stm32_ipcc_clr_bits(void __iomem *reg, u32 mask)
{
	writel_relaxed(readl_relaxed(reg) & ~mask, reg);
}

static inline int stm32_ipcc_to_chan_id(struct stm32_proc *proc, u32 buf_idx)
{
	return (proc->id * proc->ipcc->n_buffs) + buf_idx;
}

static irqreturn_t stm32_ipcc_rx_irq(int irq, void *data)
{
	struct stm32_proc *proc_dst = data;
	struct stm32_proc *proc_src;
	struct stm32_ipcc *ipcc = proc_dst->ipcc;
	struct device *dev = ipcc->controller.dev;
	u32 status, imr, tosr, idx, chan_id;
	irqreturn_t ret = IRQ_NONE;

	/* find proc source to read status xTOy */
	proc_src = &ipcc->procs[proc_dst->id ^ 0x1];
	tosr = readl_relaxed(proc_src->io_proc + IPCC_XTOYSR);
	imr = readl_relaxed(proc_dst->io_proc + IPCC_XIMR);
	status = tosr & ((~imr & RX_BIT_MASK) >> RX_BIT_SHIFT);

	for (idx = 0; idx < ipcc->n_buffs; idx++) {
		if (!(status & (1 << idx)))
			continue;

		chan_id = stm32_ipcc_to_chan_id(proc_dst, idx);

		dev_dbg(dev, "%s: proc:%d buff:%d chan:%d rx\n",
			__func__, proc_dst->id, idx, chan_id);

		mbox_chan_received_data(&ipcc->controller.chans[chan_id], NULL);

		stm32_ipcc_set_bits(proc_dst->io_proc + IPCC_XSCR,
				    RX_BIT_BUFF(idx));

		ret = IRQ_HANDLED;
	}

	return ret;
}

static irqreturn_t stm32_ipcc_tx_irq(int irq, void *data)
{
	struct stm32_proc *proc_src = data;
	struct stm32_ipcc *ipcc = proc_src->ipcc;
	struct device *dev = ipcc->controller.dev;
	u32 status, imr, tosr, idx, chan_id;
	irqreturn_t ret = IRQ_NONE;

	/* read status and mask of proc source */
	tosr = readl_relaxed(proc_src->io_proc + IPCC_XTOYSR);
	imr = readl_relaxed(proc_src->io_proc + IPCC_XIMR);
	status = tosr | ((imr & TX_BIT_MASK) >> TX_BIT_SHIFT);

	for (idx = 0; idx < ipcc->n_buffs ; idx++) {
		if (status & (1 << idx))
			continue;

		chan_id = stm32_ipcc_to_chan_id(proc_src, idx);

		dev_dbg(dev, "%s: proc:%d buff:%d chan:%d tx ack\n",
			__func__, proc_src->id, idx, chan_id);

		/* mask individual tx free buffer interrupt */
		stm32_ipcc_set_bits(proc_src->io_proc + IPCC_XIMR,
				    TX_BIT_BUFF(idx));

		mbox_chan_txdone(&ipcc->controller.chans[chan_id], 0);

		ret = IRQ_HANDLED;
	}

	return ret;
}

/*
 * Note that we are not resending the lost device interrupts.
 * We assume that the wake-up interrupt just needs to wake-up the
 * system, and then _resume() can deal with the situation.
 *
 * Mbox could has 2 wakeups interrupt. we can't use
 * dev_pm_set_dedicated_wake_irq wich manage only one
 */
static irqreturn_t stm32_ipcc_wake_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int stm32_ipcc_send_data(struct mbox_chan *link, void *data)
{
	struct stm32_chan *ch_priv = link->con_priv;
	struct stm32_proc *proc = ch_priv->proc;

	dev_dbg(proc->ipcc->controller.dev, "%s: proc:%d buff:%d chan:%d\n",
		__func__, proc->id, ch_priv->buf_idx,
		stm32_ipcc_to_chan_id(proc, ch_priv->buf_idx));

	/* set buffer n occupied */
	stm32_ipcc_set_bits(proc->io_proc + IPCC_XSCR,
			    TX_BIT_BUFF(ch_priv->buf_idx));

	/* unmask individual tx free buffer interrupt */
	stm32_ipcc_clr_bits(proc->io_proc + IPCC_XIMR,
			    TX_BIT_BUFF(ch_priv->buf_idx));

	return 0;
}

static void stm32_ipcc_free_irq(struct stm32_ipcc *ipcc, u32 id)
{
	struct stm32_proc *proc = &ipcc->procs[id];
	struct device *dev = ipcc->controller.dev;
	int i;

	/* mask proc rx/tx irq */
	stm32_ipcc_clr_bits(proc->io_proc + IPCC_XCR, XCR_RXOIE | XCR_TXOIE);

	for (i = 0; i < IPCC_IRQ_NUM; i++) {
		if (proc->irqs[i] != NO_IRQ) {
			devm_free_irq(dev, proc->irqs[i], proc);
			proc->irqs[i] = NO_IRQ;
		}
	}

	proc->used = 0;
}

static int stm32_ipcc_request_irq(struct stm32_ipcc *ipcc, u32 id)
{
	struct stm32_proc *proc = &ipcc->procs[id];
	struct device *dev = ipcc->controller.dev;
	static const char * const irq_tpl[] = {"rx", "tx", "wakeup"};
	irq_handler_t hdl[] = {stm32_ipcc_rx_irq,
		stm32_ipcc_tx_irq,
		stm32_ipcc_wake_irq};
	char irq_name[8];
	int err, irq, i;
	u32 irq_num;

	irq_num = device_may_wakeup(dev) ? IPCC_IRQ_NUM : IPCC_IRQ_WKUP;

	for (i = 0; i < irq_num; i++) {
		snprintf(irq_name, sizeof(irq_name), "%s%d", irq_tpl[i], id);
		snprintf(proc->irqs_name[i], STM32_INAME_SZ, "%s:%s",
			 dev_name(dev), irq_name);

		irq = of_irq_get_byname(dev->of_node, irq_name);
		if (irq < 0) {
			dev_err(dev, "no IRQ specified %s\n", irq_name);
			err = irq;
			goto err_irqs;
		}

		err = devm_request_threaded_irq(dev, irq, NULL, hdl[i],
						IRQF_ONESHOT,
						proc->irqs_name[i], proc);
		if (err) {
			dev_err(dev, "failed to request irq (%s, err:%d)\n",
				irq_name, err);
			goto err_irqs;
		}

		proc->irqs[i] = irq;
	}

	stm32_ipcc_set_bits(proc->io_proc + IPCC_XCR, XCR_RXOIE | XCR_TXOIE);

	return 0;

err_irqs:
	stm32_ipcc_free_irq(ipcc, id);
	return err;
}

static int stm32_ipcc_startup(struct mbox_chan *link)
{
	struct stm32_chan *ch_priv = link->con_priv;
	struct stm32_proc *proc = ch_priv->proc;
	int ret = 0;

	/* unmask individual rx buffer interrupt */
	stm32_ipcc_clr_bits(proc->io_proc + IPCC_XIMR,
			    RX_BIT_BUFF(ch_priv->buf_idx));

	/* first start for this proc */
	mutex_lock(&proc->lock);
	if (proc->used++ == 0)
		ret = stm32_ipcc_request_irq(proc->ipcc, proc->id);
	mutex_unlock(&proc->lock);

	return ret;
}

static void stm32_ipcc_shutdown(struct mbox_chan *link)
{
	struct stm32_chan *ch_priv = link->con_priv;
	struct stm32_proc *proc = ch_priv->proc;

	/* mask individual rx/tx buffer interrupt */
	stm32_ipcc_set_bits(proc->io_proc + IPCC_XIMR,
			    RX_BIT_BUFF(ch_priv->buf_idx) |
			    TX_BIT_BUFF(ch_priv->buf_idx));

	/* last stop for this proc */
	mutex_lock(&proc->lock);
	if (--proc->used <= 0)
		stm32_ipcc_free_irq(proc->ipcc, proc->id);
	mutex_unlock(&proc->lock);
}

static const struct mbox_chan_ops stm32_ipcc_ops = {
	.send_data	= stm32_ipcc_send_data,
	.startup	= stm32_ipcc_startup,
	.shutdown	= stm32_ipcc_shutdown,
};

static struct mbox_chan *stm32_ipcc_of_xlate(struct mbox_controller *controller,
					     const struct of_phandle_args *sp)
{
	struct stm32_ipcc *ipcc = dev_get_drvdata(controller->dev);
	unsigned int proc_id, buf_idx;
	unsigned int chan_id;
	struct stm32_chan *ch_priv;
	struct stm32_proc *proc;
	struct mbox_chan *chan;

	if (WARN_ON(sp->args_count != 2))
		return ERR_PTR(-EINVAL);

	proc_id = sp->args[0];
	buf_idx = sp->args[1];

	if (buf_idx >= ipcc->n_buffs || proc_id >= STM32_MAX_PROCS)
		return ERR_PTR(-EINVAL);

	proc = &ipcc->procs[proc_id];
	chan_id = stm32_ipcc_to_chan_id(proc, buf_idx);
	ch_priv = &ipcc->chans[chan_id];
	ch_priv->buf_idx = buf_idx;
	ch_priv->proc = proc;

	chan = &controller->chans[chan_id];
	chan->con_priv = ch_priv;

	dev_dbg(ipcc->controller.dev, "%s: proc:%d buff:%d chan:%d\n",
		__func__, proc_id, buf_idx, chan_id);

	return chan;
}

static const struct of_device_id stm32_ipcc_of_match[] = {
	{ .compatible = "st,stm32-ipcc" },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_ipcc_of_match);

static int stm32_ipcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct stm32_ipcc *ipcc;
	struct resource *res;
	int i, j, ret;

	if (!np) {
		dev_err(&pdev->dev, "No DT found\n");
		return -ENODEV;
	}

	ipcc = devm_kzalloc(dev, sizeof(*ipcc), GFP_KERNEL);
	if (!ipcc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ipcc->io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ipcc->io_base))
		return PTR_ERR(ipcc->io_base);

	/* clock get & prepare */
	ipcc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ipcc->clk))
		return PTR_ERR(ipcc->clk);

	ret = clk_prepare_enable(ipcc->clk);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		return ret;
	}

	ipcc->ip_ver = readl_relaxed(ipcc->io_base + IPCC_VER);
	ipcc->n_buffs = readl_relaxed(ipcc->io_base + IPCC_HWCFGR);
	ipcc->n_buffs &= IPCFGR_BUFF_MASK;

	if (ipcc->n_buffs > STM32_MAX_BUF) {
		ret = -EINVAL;
		goto err;
	}

	ipcc->controller.dev = dev;
	ipcc->controller.txdone_irq = true;
	ipcc->controller.ops = &stm32_ipcc_ops;
	ipcc->controller.of_xlate = &stm32_ipcc_of_xlate;
	ipcc->controller.num_chans = ipcc->n_buffs * STM32_MAX_PROCS;

	ipcc->controller.chans = devm_kcalloc(dev, ipcc->controller.num_chans,
					      sizeof(*ipcc->controller.chans),
					      GFP_KERNEL);

	ipcc->chans = devm_kcalloc(dev, ipcc->controller.num_chans,
				   sizeof(*ipcc->chans), GFP_KERNEL);

	ipcc->procs = devm_kcalloc(dev, STM32_MAX_PROCS, sizeof(*ipcc->procs),
				   GFP_KERNEL);

	if (!ipcc->procs || !ipcc->controller.chans || !ipcc->chans) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < STM32_MAX_PROCS; i++) {
		struct stm32_proc *proc = &ipcc->procs[i];

		proc->id = i;
		proc->ipcc = ipcc;
		proc->io_proc = ipcc->io_base + (i * IPCC_PROC_OFFST);
		for (j = 0; j < IPCC_IRQ_NUM; j++)
			proc->irqs[j] = NO_IRQ;

		mutex_init(&proc->lock);
	}

	ret = mbox_controller_register(&ipcc->controller);
	if (ret)
		goto err;

	device_init_wakeup(dev, of_property_read_bool(np, "wakeup-source"));

	platform_set_drvdata(pdev, ipcc);
	dev_info(dev, "mailbox rev:%ld.%ld enabled, num chans:%d\n",
		 (ipcc->ip_ver & VER_MAJREV_MASK) >> VER_MAJREV_SHIFT,
		 (ipcc->ip_ver & VER_MINREV_MASK) >> VER_MINREV_SHIFT,
		 ipcc->controller.num_chans);

	return 0;

err:
	clk_disable_unprepare(ipcc->clk);
	return ret;
}

static int stm32_ipcc_remove(struct platform_device *pdev)
{
	struct stm32_ipcc *ipcc = platform_get_drvdata(pdev);

	mbox_controller_unregister(&ipcc->controller);
	clk_disable_unprepare(ipcc->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void stm32_ipcc_set_irq_wake(struct device *dev, bool enable)
{
	struct stm32_ipcc *ipcc = dev_get_drvdata(dev);
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < STM32_MAX_PROCS; i++) {
			struct stm32_proc *proc = &ipcc->procs[i];

			if (proc->irqs[IPCC_IRQ_RX] == NO_IRQ ||
			    proc->irqs[IPCC_IRQ_TX_ACK] == NO_IRQ ||
			    proc->irqs[IPCC_IRQ_WKUP] == NO_IRQ)
				continue;

			irq_set_irq_wake(proc->irqs[IPCC_IRQ_RX], enable);
			irq_set_irq_wake(proc->irqs[IPCC_IRQ_TX_ACK], enable);
			irq_set_irq_wake(proc->irqs[IPCC_IRQ_WKUP], enable);
		}
	}
}

static int stm32_ipcc_suspend(struct device *dev)
{
	stm32_ipcc_set_irq_wake(dev, true);
	return 0;
}

static int stm32_ipcc_resume(struct device *dev)
{
	stm32_ipcc_set_irq_wake(dev, false);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(stm32_ipcc_pm_ops,
			 stm32_ipcc_suspend, stm32_ipcc_resume);
static struct platform_driver stm32_ipcc_driver = {
	.driver = {
		.name = "stm32-ipcc",
		.owner = THIS_MODULE,
		.pm = &stm32_ipcc_pm_ops,
		.of_match_table = stm32_ipcc_of_match,
	},
	.probe		= stm32_ipcc_probe,
	.remove		= stm32_ipcc_remove,
};

module_platform_driver(stm32_ipcc_driver);

MODULE_AUTHOR("Ludovic Barre <ludovic.barre@st.com>");
MODULE_DESCRIPTION("STM32 IPCC driver");
MODULE_LICENSE("GPL v2");
