/*
 * STMicroelectronics Multi-Function eXpander (ST-MFX) Core Driver
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com> for STMicroelectronics.
 *
 * License terms: GPL V2.0.
 *
 * st-mfx Core Driver is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * st-mfx Core Driver is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * st-mfx Core Driver. If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/st-mfx.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>

/**
 * struct mfx_priv - MFX MFD private structure
 * @dev: device, mostly for logs
 * @regmap: register map
 * @mfx: MFX MFD public structure, to share information with subdrivers
 * @irq_lock: IRQ bus lock
 * @irq_domain: IRQ domain
 * @irq: IRQ number for mfx
 * @irqen: cache of IRQ_SRC_EN register for bus_lock
 * @oldirqen: cache of IRQ_SRC_EN register for bus_lock
 */
struct mfx_priv {
	struct device *dev;
	struct regmap *regmap;
	struct mfx mfx;
	struct mutex irq_lock; /* IRQ bus lock */
	struct irq_domain *irq_domain;
	int irq;
	u8 irqen;
	u8 oldirqen;
};

#define to_mfx_priv(_mfx) container_of(_mfx, struct mfx_priv, mfx)

/* MFX boot time is around 10ms, so after reset, we have to wait this delay */
#define MFX_BOOT_TIME 10

static int __mfx_enable(struct mfx_priv *mfx_priv, u32 blocks, bool enable)
{
	u8 mask = 0;

	if (blocks & MFX_BLOCK_GPIO)
		mask |= MFX_REG_SYS_CTRL_GPIO_EN;
	else
		mask &= ~MFX_REG_SYS_CTRL_GPIO_EN;

	if (blocks & MFX_BLOCK_TS)
		mask |= MFX_REG_SYS_CTRL_TS_EN;
	else
		mask &= ~MFX_REG_SYS_CTRL_TS_EN;

	if (blocks & MFX_BLOCK_IDD)
		mask |= MFX_REG_SYS_CTRL_IDD_EN;
	else
		mask &= ~MFX_REG_SYS_CTRL_IDD_EN;

	if (blocks & MFX_BLOCK_ALTGPIO)
		mask |= MFX_REG_SYS_CTRL_ALTGPIO_EN;
	else
		mask &= ~MFX_REG_SYS_CTRL_ALTGPIO_EN;

	return regmap_update_bits(mfx_priv->regmap, MFX_REG_SYS_CTRL, mask,
				  enable ? mask : 0);
}

static int __mfx_reset(struct mfx_priv *mfx_priv)
{
	int ret;

	ret = regmap_update_bits(mfx_priv->regmap, MFX_REG_SYS_CTRL,
				 MFX_REG_SYS_CTRL_SWRST,
				 MFX_REG_SYS_CTRL_SWRST);

	if (ret < 0)
		return ret;

	msleep(MFX_BOOT_TIME);

	return ret;
}

int mfx_block_read(struct mfx *mfx, u8 reg, u8 length, u8 *values)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);

	return regmap_bulk_read(mfx_priv->regmap, reg, values, length);
}
EXPORT_SYMBOL_GPL(mfx_block_read);

int mfx_block_write(struct mfx *mfx, u8 reg, u8 length, const u8 *values)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);

	return regmap_bulk_write(mfx_priv->regmap, reg, values, length);
}
EXPORT_SYMBOL_GPL(mfx_block_write);

int mfx_reg_read(struct mfx *mfx, u8 reg)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);
	u32 val;
	int ret;

	ret = regmap_read(mfx_priv->regmap, reg, &val);

	return ret ? ret : val;
}
EXPORT_SYMBOL_GPL(mfx_reg_read);

int mfx_reg_write(struct mfx *mfx, u8 reg, u8 val)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);

	return regmap_write(mfx_priv->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(mfx_reg_write);

int mfx_set_bits(struct mfx *mfx, u8 reg, u8 mask, u8 val)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);

	return regmap_update_bits(mfx_priv->regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(mfx_set_bits);

int mfx_enable(struct mfx *mfx, u32 blocks)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);

	return __mfx_enable(mfx_priv, blocks, true);
}
EXPORT_SYMBOL_GPL(mfx_enable);

int mfx_disable(struct mfx *mfx, u32 blocks)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(mfx);

	return __mfx_enable(mfx_priv, blocks, false);
}
EXPORT_SYMBOL_GPL(mfx_disable);

static const struct resource mfx_gpio_resources[] = {
	{
		.name	= "GPIO",
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell mfx_gpio_cell[] = {
	{
		.name		= "st-mfx-gpio",
		.of_compatible	= "st,mfx-gpio",
		.resources	= mfx_gpio_resources,
		.num_resources	= ARRAY_SIZE(mfx_gpio_resources),
	},
};

static irqreturn_t mfx_irq(int irq, void *data)
{
	struct mfx_priv *mfx_priv = data;
	int ret;
	u8 status, ack;

	ret = mfx_reg_read(&mfx_priv->mfx, MFX_REG_IRQ_PENDING);
	if (ret < 0) {
		dev_err(mfx_priv->dev, "can't get IRQ_PENDING: %d\n", ret);
		return IRQ_NONE;
	}

	/* It can be GPIO, IDD, ERROR, TS* IRQs */
	status = ret & mfx_priv->irqen;
	if (!status) {
		dev_dbg(mfx_priv->dev, "spurious IT: IRQ_PENDING=%#x\n", ret);
		return IRQ_NONE;
	}

	/*
	 * There is no ACK for GPIO, MFX_REG_IRQ_PENDING_GPIO is a logical OR
	 * of MFX_REG_IRQ_GPI _PENDING1/_PENDING2/_PENDING3
	 */
	ack = status & ~MFX_REG_IRQ_PENDING_GPIO;

	while (status) {
		int bit = __ffs(status);
		int nestedirq = irq_find_mapping(mfx_priv->irq_domain, bit);

		handle_nested_irq(nestedirq);
		status &= ~(BIT(bit));
	}

	if (ack)
		mfx_reg_write(&mfx_priv->mfx, MFX_REG_IRQ_ACK, ack);

	return IRQ_HANDLED;
}

static void mfx_irq_lock(struct irq_data *data)
{
	struct mfx_priv *mfx_priv = irq_data_get_irq_chip_data(data);

	mutex_lock(&mfx_priv->irq_lock);
}

static void mfx_irq_sync_unlock(struct irq_data *data)
{
	struct mfx_priv *mfx_priv = irq_data_get_irq_chip_data(data);
	u8 new = mfx_priv->irqen;
	u8 old = mfx_priv->oldirqen;

	if (new == old)
		goto unlock;

	mfx_priv->oldirqen = new;
	mfx_reg_write(&mfx_priv->mfx, MFX_REG_IRQ_SRC_EN, new);
unlock:
	mutex_unlock(&mfx_priv->irq_lock);
}

static void mfx_irq_mask(struct irq_data *data)
{
	struct mfx_priv *mfx_priv = irq_data_get_irq_chip_data(data);
	int offset = data->hwirq;
	int mask = BIT(offset % 8);

	mfx_priv->irqen &= ~mask;
}

static void mfx_irq_unmask(struct irq_data *data)
{
	struct mfx_priv *mfx_priv = irq_data_get_irq_chip_data(data);
	int offset = data->hwirq;
	int mask = BIT(offset % 8);

	mfx_priv->irqen |= mask;
}

static struct irq_chip mfx_irq_chip = {
	.name			= "mfx",
	.irq_bus_lock		= mfx_irq_lock,
	.irq_bus_sync_unlock	= mfx_irq_sync_unlock,
	.irq_mask		= mfx_irq_mask,
	.irq_unmask		= mfx_irq_unmask,
};

static int mfx_irq_map(struct irq_domain *d, unsigned int virq,
		       irq_hw_number_t hwirq)
{
	struct mfx_priv *mfx_priv = d->host_data;
	struct irq_chip *chip = &mfx_irq_chip;

	irq_set_chip_data(virq, mfx_priv);
	irq_set_chip_and_handler(virq, chip, handle_edge_irq);
	irq_set_nested_thread(virq, 1);
	irq_set_noprobe(virq);

	return 0;
}

static void mfx_irq_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops mfx_irq_ops = {
	.map = mfx_irq_map,
	.unmap = mfx_irq_unmap,
	.xlate = irq_domain_xlate_twocell,
};

static int mfx_irq_init(struct mfx_priv *mfx_priv, struct device_node *np)
{
	int irqoutpin = MFX_REG_IRQ_OUT_PIN_TYPE; /* Push-Pull */
	int irqtrigger, ret;

	mfx_priv->irq = of_irq_get(np, 0);
	if (mfx_priv->irq > 0) {
		irqtrigger = irq_get_trigger_type(mfx_priv->irq);
	} else {
		dev_err(mfx_priv->dev, "failed to get irq: %d\n",
			mfx_priv->irq);
		return mfx_priv->irq;
	}

	if ((irqtrigger & IRQ_TYPE_EDGE_FALLING) ||
	    (irqtrigger & IRQ_TYPE_LEVEL_LOW))
		irqoutpin &= ~MFX_REG_IRQ_OUT_PIN_POL; /* Active Low */
	else
		irqoutpin |= MFX_REG_IRQ_OUT_PIN_POL; /* Active High */

	mfx_reg_write(&mfx_priv->mfx, MFX_REG_IRQ_OUT_PIN, irqoutpin);

	mfx_priv->irq_domain = irq_domain_add_simple(np, MFX_NR_IRQ_SRC, 0,
						     &mfx_irq_ops, mfx_priv);
	if (!mfx_priv->irq_domain) {
		dev_err(mfx_priv->dev, "failed to create irqdomain\n");
		return -ENOMEM;
	}

	ret = devm_request_threaded_irq(mfx_priv->dev, mfx_priv->irq,
					NULL, mfx_irq,
					irqtrigger | IRQF_ONESHOT, "mfx",
					mfx_priv);
	if (ret) {
		dev_err(mfx_priv->dev, "failed to request irq: %d\n", ret);
		irq_domain_remove(mfx_priv->irq_domain);
		return ret;
	}

	return 0;
}

static void mfx_irq_remove(struct mfx_priv *mfx_priv)
{
	int hwirq;

	for (hwirq = 0; hwirq < MFX_NR_IRQ_SRC; hwirq++)
		irq_dispose_mapping(irq_find_mapping(mfx_priv->irq_domain,
						     hwirq));
	irq_domain_remove(mfx_priv->irq_domain);
}

static int mfx_chip_init(struct mfx_priv *mfx_priv, u16 i2c_addr)
{
	int ret;
	int id;
	u8 version[2];

	id = mfx_reg_read(&mfx_priv->mfx, MFX_REG_CHIP_ID);
	if (id < 0) {
		dev_err(mfx_priv->dev, "error reading chip id: %d\n", id);
		return id;
	}

	ret = mfx_block_read(&mfx_priv->mfx, MFX_REG_FW_VERSION_MSB,
			     ARRAY_SIZE(version), version);
	if (ret < 0) {
		dev_err(mfx_priv->dev, "error reading fw version: %d\n", ret);
		return ret;
	}

	/*
	 * Check that ID is the complement of the I2C address:
	 * MFX I2C address follows the 7-bit format (MSB), that's why
	 * i2c_addr is shifted.
	 *
	 * MFX_I2C_ADDR |         MFX         |        Linux
	 *  input pin   | I2C device address  | I2C device address
	 *--------------------------------------------------------
	 *      0       | b: 1000 010x h:0x84 |       0x42
	 *      1       | b: 1000 011x h:0x86 |       0x43
	 */
	if (FIELD_GET(MFX_REG_CHIP_ID_MASK, ~id) != (i2c_addr << 1)) {
		dev_err(mfx_priv->dev, "unknown chip id: %#x\n", id);
		return -EINVAL;
	}

	dev_info(mfx_priv->dev, "ST-MFX chip id: %#x, fw version: %x.%02x\n",
		 id, version[0], version[1]);

	/* Disable all features, subdrivers should enable what they need */
	ret = mfx_disable(&mfx_priv->mfx, ~0);
	if (ret)
		return ret;

	return __mfx_reset(mfx_priv);
}

static int mfx_add_devices(struct mfx_priv *mfx_priv)
{
	int ret;

	if (mfx_priv->mfx.blocks & MFX_BLOCK_GPIO) {
		ret = devm_mfd_add_devices(mfx_priv->dev, PLATFORM_DEVID_AUTO,
					   mfx_gpio_cell,
					   ARRAY_SIZE(mfx_gpio_cell),
					   NULL, 0, mfx_priv->irq_domain);
		if (ret) {
			dev_err(mfx_priv->dev,
				"failed to add gpio device: ret=%d\n", ret);
			return ret;
		}
	}
	/* Here we could find other blocks like MFX_BLOCK_TS or MFX_BLOCK_IDD */

	return 0;
}

static const struct regmap_config mfx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct of_device_id mfx_of_match[] = {
	{ .compatible = "st,mfx" },
	{ }
};
MODULE_DEVICE_TABLE(of, mfx_of_match);

static int mfx_of_probe(struct device_node *np, struct mfx_priv *mfx_priv)
{
	struct device_node *child;

	if (!np)
		return -ENODEV;

	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, "st,mfx-gpio")) {
			mfx_priv->mfx.blocks |= MFX_BLOCK_GPIO;
			mfx_priv->mfx.num_gpio = 16;
		}
		/*
		 * Here we could find other children like "st,mfx-ts" or
		 * "st,mfx-idd.
		 */
	}

	if (!(mfx_priv->mfx.blocks & MFX_BLOCK_TS) &&
	    !(mfx_priv->mfx.blocks & MFX_BLOCK_IDD)) {
		mfx_priv->mfx.blocks |= MFX_BLOCK_ALTGPIO;
		mfx_priv->mfx.num_gpio += 8;
	}

	/*
	 * TODO: aGPIO[3:0] and aGPIO[7:4] can be used independently:
	 * - if IDD is used but not TS, aGPIO[3:0] can be used as GPIO,
	 * - if TS is used but not TS: aGPIO[7:4] can be used as GPIO.
	 */

	return 0;
}

int mfx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct mfx_priv *mfx_priv;
	int ret;

	mfx_priv = devm_kzalloc(&client->dev, sizeof(struct mfx_priv),
				GFP_KERNEL);
	if (!mfx_priv)
		return -ENOMEM;

	mfx_priv->regmap = devm_regmap_init_i2c(client, &mfx_regmap_config);
	if (IS_ERR(mfx_priv->regmap)) {
		ret = PTR_ERR(mfx_priv->regmap);
		dev_err(&client->dev, "failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	mfx_priv->dev = &client->dev;
	i2c_set_clientdata(client, &mfx_priv->mfx);

	ret = mfx_of_probe(np, mfx_priv);
	if (ret < 0)
		return ret;

	mutex_init(&mfx_priv->irq_lock);

	ret = mfx_chip_init(mfx_priv, client->addr);
	if (ret) {
		if (ret == -ETIMEDOUT)
			return -EPROBE_DEFER;

		return ret;
	}

	ret = mfx_irq_init(mfx_priv, np);
	if (ret < 0)
		return ret;

	ret = mfx_add_devices(mfx_priv);
	if (ret) {
		dev_err(mfx_priv->dev, "failed to add chilren: %d\n", ret);
		mfd_remove_devices(mfx_priv->dev);
		return ret;
	}

	dev_info(mfx_priv->dev, "ST-MFX (CORE) initialized\n");

	return 0;
}

static int mfx_remove(struct i2c_client *client)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(i2c_get_clientdata(client));

	mfx_irq_remove(mfx_priv);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mfx_suspend(struct device *dev)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(dev_get_drvdata(dev));

	if (device_may_wakeup(dev))
		enable_irq_wake(mfx_priv->irq);

	/*
	 * TODO: Do we put MFX in STANDBY mode ?
	 * (Wakeup by rising edge on MFX_wakeup pin)
	 */

	return 0;
}

static int mfx_resume(struct device *dev)
{
	struct mfx_priv *mfx_priv = to_mfx_priv(dev_get_drvdata(dev));

	if (device_may_wakeup(dev))
		disable_irq_wake(mfx_priv->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mfx_dev_pm_ops, mfx_suspend, mfx_resume);

static const struct i2c_device_id mfx_i2c_id[] = {
	{ "mfx", },
	{},
};
MODULE_DEVICE_TABLE(i2c, mfx_id);

static struct i2c_driver mfx_driver = {
	.driver = {
		.name = "st-mfx",
		.pm = &mfx_dev_pm_ops,
		.of_match_table = mfx_of_match,
	},
	.probe = mfx_probe,
	.remove = mfx_remove,
	.id_table = mfx_i2c_id,
};

static int __init mfx_init(void)
{
	return i2c_add_driver(&mfx_driver);
}
subsys_initcall(mfx_init);

static void __exit mfx_exit(void)
{
	i2c_del_driver(&mfx_driver);
}
module_exit(mfx_exit);

MODULE_DESCRIPTION("STMicroelectronics Multi-Function eXpander MFD core driver");
MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@st.com>");
MODULE_LICENSE("GPL v2");
