/*
 * stm32_rproc.c
 *
 * Copyright (C) 2016, STMicroelectronics
 * Author: Ludovic Barre <ludovic.barre@st.com>
 *
 * License terms: GNU General Public License (GPL), version 2
 */
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"

enum boot {
	HOLD_BOOT,
	RELEASE_BOOT,
};

#define STM32_SMC_RCC		0x82001000
#define STM32_SMC_REG_WRITE	0x1

struct stm32_rproc_cfg {
	bool has_reset;
};

struct stm32_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

struct stm32_mbox {
	struct mbox_chan *chan;
	struct mbox_client client;
	int vring_id;
};

struct stm32_rproc {
	struct stm32_rproc_cfg *config;
	struct reset_control *rst;
	struct regmap *hold_syscon;
	struct stm32_rproc_mem ram[2];
	struct stm32_mbox mb[2];
	bool smc_boot;
	u32 hold_reg;
	u32 hold_mask;
};

static irqreturn_t stm32_rproc_wdg(int irq, void *dev_id)
{
	struct rproc *rproc = dev_id;

	rproc_report_crash(rproc, RPROC_WATCHDOG);

	return IRQ_HANDLED;
}

static void stm32_rproc_mb_callback(struct mbox_client *cl, void *data)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct stm32_mbox *mb = container_of(cl, struct stm32_mbox, client);

	if (rproc_vq_interrupt(rproc, mb->vring_id) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message was found in vqid %d\n",
			mb->vring_id);
}

static void stm32_rproc_mb_free_chans(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int i;

	for (i = 0; i < ARRAY_SIZE(ddata->mb); i++) {
		if (ddata->mb[i].chan) {
			mbox_free_channel(ddata->mb[i].chan);
			ddata->mb[i].chan = NULL;
		}
	}
}

static int stm32_rproc_mb_init(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = &rproc->dev;
	int ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(ddata->mb); i++) {
		struct mbox_client *cl = &ddata->mb[i].client;

		cl->dev = dev->parent;
		cl->tx_done = NULL;
		cl->rx_callback = stm32_rproc_mb_callback;
		cl->tx_block = false;
		cl->knows_txdone = false;
		ddata->mb[i].vring_id = i;

		ddata->mb[i].chan = mbox_request_channel(cl, i);
		if (IS_ERR(ddata->mb[i].chan)) {
			ret = PTR_ERR(ddata->mb[i].chan);
			ddata->mb[i].chan = NULL;
			dev_err(dev, "failed to get mbox chan:%d err:%d\n",
				i, ret);
			stm32_rproc_mb_free_chans(rproc);
			break;
		}
	}

	return ret;
}

static bool stm32_rproc_is_hold(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	u32 hold_boot;
	int err;

	err = regmap_read(ddata->hold_syscon, ddata->hold_reg, &hold_boot);
	if (err) {
		dev_err(&rproc->dev, "failed to read status of mcu\n");
		return err;
	}

	hold_boot &= ddata->hold_mask;

	return hold_boot == HOLD_BOOT ? true : false;
}

static int stm32_rproc_hold_boot(struct rproc *rproc, bool hold)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct arm_smccc_res smc_res;
	int val, err;

	val = hold ? HOLD_BOOT : RELEASE_BOOT;

	if (ddata->smc_boot) {
		arm_smccc_smc(STM32_SMC_RCC, STM32_SMC_REG_WRITE,
			      ddata->hold_reg, val, 0, 0, 0, 0, &smc_res);
		err = smc_res.a0;
	} else {
		err = regmap_update_bits(ddata->hold_syscon, ddata->hold_reg,
					 ddata->hold_mask, val);
	}

	if (err)
		dev_err(&rproc->dev, "failed to write hold boot\n");

	return err;
}

static int stm32_rproc_start(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	struct device *dev = rproc->dev.parent;
	int err;

	if (!stm32_rproc_is_hold(rproc)) {
		dev_err(&rproc->dev, "failed, mcu not stopped\n");
		return -EBUSY;
	}

	if (of_find_property(dev->of_node, "mboxes", NULL)) {
		err = stm32_rproc_mb_init(rproc);
		if (err)
			return err;
	} else {
		dev_info(dev, "rproc without mboxes\n");
	}

	err = stm32_rproc_hold_boot(rproc, false);
	if (err)
		return err;

	return reset_control_assert(ddata->rst);
}

static int stm32_rproc_stop(struct rproc *rproc)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	err = stm32_rproc_hold_boot(rproc, true);
	if (err)
		return err;

	err = reset_control_assert(ddata->rst);
	if (err)
		return err;

	stm32_rproc_mb_free_chans(rproc);

	return 0;
}

static void stm32_rproc_kick(struct rproc *rproc, int vqid)
{
	struct stm32_rproc *ddata = rproc->priv;
	int err;

	/* send the index of the triggered virtqueue in the mailbox payload */
	err = mbox_send_message(ddata->mb[vqid].chan, (void *)vqid);
	if (err < 0)
		dev_err(&rproc->dev, "%s: failed (vqid%d, err:%d)\n",
			__func__, vqid, err);
}

static void *stm32_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct stm32_rproc *ddata = rproc->priv;
	void *va = NULL;
	u32 offset;
	int i;

	for (i = 0; i < 2; i++) {
		if (da >= ddata->ram[i].dev_addr && da + len <=
		    ddata->ram[i].dev_addr + ddata->ram[i].size) {
			offset = da - ddata->ram[i].dev_addr;
			/* __force to make sparse happy with type conversion */
			va = (__force void *)(ddata->ram[i].cpu_addr + offset);
			break;
		}
	}

	return va;
}

static struct rproc_ops st_rproc_ops = {
	.start		= stm32_rproc_start,
	.stop		= stm32_rproc_stop,
	.kick		= stm32_rproc_kick,
	.da_to_va	= stm32_rproc_da_to_va,
};

static const struct stm32_rproc_cfg stm32_m4_rproc_cfg = {
	.has_reset = true,
};

static const struct of_device_id stm32_rproc_match[] = {
	{ .compatible = "st,stm32-m4-rproc", .data = &stm32_m4_rproc_cfg },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_rproc_match);

static int stm32_rproc_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct stm32_rproc *ddata = rproc->priv;
	const char *mem_names[2] = {"retram", "mcusram"};
	struct resource *res;
	int i, err, irq;

	irq = platform_get_irq_byname(pdev, "wdg");
	if (irq > 0) {
		err = devm_request_irq(dev, irq, stm32_rproc_wdg, 0,
				       dev_name(dev), rproc);
		if (err) {
			dev_err(dev, "failed to request wdg irq\n");
			return err;
		}

		dev_info(dev, "wdg irq registered\n");
	}

	if (ddata->config->has_reset) {
		ddata->rst = devm_reset_control_get(dev, "mcu_rst");
		if (IS_ERR(ddata->rst)) {
			dev_err(dev, "failed to get mcu reset\n");
			return PTR_ERR(ddata->rst);
		}
	}

	/*
	 * if platform is secured the hold boot bit must be written by
	 * smc call and read normally.
	 * if not secure the hold boot bit could be read/write normally
	 */
	ddata->smc_boot = of_property_read_bool(np, "st,smc_boot");

	ddata->hold_syscon = syscon_regmap_lookup_by_phandle(np,
							     "st,hold_boot");
	if (IS_ERR(ddata->hold_syscon))
		return PTR_ERR(ddata->hold_syscon);

	err = of_property_read_u32_index(np, "st,hold_boot",
					 1, &ddata->hold_reg);
	if (err) {
		dev_err(dev, "mcu boot ctrl offset required\n");
		return err;
	}

	err = of_property_read_u32_index(np, "st,hold_boot",
					 2, &ddata->hold_mask);
	if (err) {
		dev_err(dev, "mcu boot ctrl mask required\n");
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		ddata->ram[i].cpu_addr = devm_ioremap_resource(dev, res);
		if (IS_ERR(ddata->ram[i].cpu_addr))
			return err;

		ddata->ram[i].bus_addr = res->start;
		ddata->ram[i].size = resource_size(res);

		/*
		 * the m4 has retram at address 0 in its view (DA)
		 * so for retram DA=0x0 PA=bus_addr else DA=PA=bus_addr
		 */
		if (i == 0)
			ddata->ram[i].dev_addr = 0x0;
		else
			ddata->ram[i].dev_addr = ddata->ram[i].bus_addr;
	}

	err = of_reserved_mem_device_init(dev);
	if (err) {
		dev_err(dev, "failed to obtain shared memory\n");
		return err;
	}

	return err;
}

static int stm32_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct stm32_rproc *ddata;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	int ret;

	match = of_match_device(stm32_rproc_match, dev);
	if (!match || !match->data) {
		dev_err(dev, "no device match found\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(dev, np->name, &st_rproc_ops, NULL, sizeof(*ddata));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	ddata = rproc->priv;
	ddata->config = (struct stm32_rproc_cfg *)match->data;

	platform_set_drvdata(pdev, rproc);

	ret = stm32_rproc_parse_dt(pdev);
	if (ret)
		goto free_rproc;

	stm32_rproc_stop(rproc);

	ret = rproc_add(rproc);
	if (ret)
		goto free_rproc;

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int stm32_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);

	of_reserved_mem_device_release(&pdev->dev);

	rproc_free(rproc);

	return 0;
}

static struct platform_driver stm32_rproc_driver = {
	.probe = stm32_rproc_probe,
	.remove = stm32_rproc_remove,
	.driver = {
		.name = "stm32-rproc",
		.of_match_table = of_match_ptr(stm32_rproc_match),
	},
};
module_platform_driver(stm32_rproc_driver);

MODULE_DESCRIPTION("STM32 Remote Processor Control Driver");
MODULE_AUTHOR("Ludovic Barre <ludovic.barre@st.com>");
MODULE_LICENSE("GPL v2");

