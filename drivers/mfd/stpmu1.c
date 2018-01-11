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

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/stpmu1.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

static bool stpmu1_reg_readable(struct device *dev, unsigned int reg);
static bool stpmu1_reg_writeable(struct device *dev, unsigned int reg);
static bool stpmu1_reg_volatile(struct device *dev, unsigned int reg);

const struct regmap_config stpmu1_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = PMIC_MAX_REGISTER_ADDRESS,
	.readable_reg = stpmu1_reg_readable,
	.writeable_reg = stpmu1_reg_writeable,
	.volatile_reg = stpmu1_reg_volatile,
};

#define FILL_IRQS(_index, _irq, _mask) \
	[(_index)] = { \
		.reg_offset = ((_irq) >> 3), \
		.mask = (1 << (7 - ((_mask) % 8))), \
	}

static const struct regmap_irq stpmu1_irqs[] = {
	FILL_IRQS(IT_SWOUT_R, IT_SWOUT_R, IT_SWOUT_R),
	FILL_IRQS(IT_SWOUT_F, IT_SWOUT_F, IT_SWOUT_F),
	FILL_IRQS(IT_VBUS_OTG_R, IT_VBUS_OTG_R, IT_VBUS_OTG_R),
	FILL_IRQS(IT_VBUS_OTG_F, IT_VBUS_OTG_F, IT_VBUS_OTG_F),
	FILL_IRQS(IT_WAKEUP_R, IT_WAKEUP_R, IT_WAKEUP_R),
	FILL_IRQS(IT_WAKEUP_F, IT_WAKEUP_F, IT_WAKEUP_F),
	FILL_IRQS(IT_PONKEY_R, IT_PONKEY_R, IT_PONKEY_R),
	FILL_IRQS(IT_PONKEY_F, IT_PONKEY_F, IT_PONKEY_F),
	FILL_IRQS(IT_OVP_BOOST, IT_OVP_BOOST, IT_OVP_BOOST),
	FILL_IRQS(IT_OCP_BOOST, IT_OCP_BOOST, IT_OCP_BOOST),
	FILL_IRQS(IT_OCP_SWOUT, IT_OCP_SWOUT, IT_OCP_SWOUT),
	FILL_IRQS(IT_OCP_OTG, IT_OCP_OTG, IT_OCP_OTG),
	FILL_IRQS(IT_CURLIM_BUCK4, IT_CURLIM_BUCK4, IT_CURLIM_BUCK4),
	FILL_IRQS(IT_CURLIM_BUCK3, IT_CURLIM_BUCK3, IT_CURLIM_BUCK3),
	FILL_IRQS(IT_CURLIM_BUCK2, IT_CURLIM_BUCK2, IT_CURLIM_BUCK2),
	FILL_IRQS(IT_CURLIM_BUCK1, IT_CURLIM_BUCK1, IT_CURLIM_BUCK1),
	FILL_IRQS(IT_SHORT_SWOUT, IT_SHORT_SWOUT, IT_SHORT_SWOUT),
	FILL_IRQS(IT_SHORT_SWOTG, IT_SHORT_SWOTG, IT_SHORT_SWOTG),
	FILL_IRQS(IT_CURLIM_LDO6, IT_CURLIM_LDO6, IT_CURLIM_LDO6),
	FILL_IRQS(IT_CURLIM_LDO5, IT_CURLIM_LDO5, IT_CURLIM_LDO5),
	FILL_IRQS(IT_CURLIM_LDO4, IT_CURLIM_LDO4, IT_CURLIM_LDO4),
	FILL_IRQS(IT_CURLIM_LDO3, IT_CURLIM_LDO3, IT_CURLIM_LDO3),
	FILL_IRQS(IT_CURLIM_LDO2, IT_CURLIM_LDO2, IT_CURLIM_LDO2),
	FILL_IRQS(IT_CURLIM_LDO1, IT_CURLIM_LDO1, IT_CURLIM_LDO1),
	FILL_IRQS(IT_SWIN_R, IT_SWIN_R, IT_SWIN_R),
	FILL_IRQS(IT_SWIN_F, IT_SWIN_F, IT_SWIN_F),
	FILL_IRQS(IT_VINLOW_R, IT_VINLOW_R, IT_VINLOW_R),
	FILL_IRQS(IT_VINLOW_F, IT_VINLOW_F, IT_VINLOW_F),
	FILL_IRQS(IT_TWARN_R, IT_TWARN_R, IT_TWARN_R),
	FILL_IRQS(IT_TWARN_F, IT_TWARN_F, IT_TWARN_F),
};

static const struct regmap_irq_chip stpmu1_regmap_irq_chip = {
	.name = "pmic_irq",
	.status_base = ITLATCH1_REG,
	.mask_base = ITCLEARMASK1_REG,
	.unmask_base = ITSETMASK1_REG,
	.ack_base = ITCLEARLATCH1_REG,
	.num_regs = STPMU1_PMIC_NUM_IRQ_REGS,
	.irqs = stpmu1_irqs,
	.num_irqs = ARRAY_SIZE(stpmu1_irqs),
};

static bool stpmu1_reg_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TURN_ON_REG:
	case TURN_OFF_REG:
	case ICC_LDO_TURN_OFF_REG:
	case ICC_BUCK_TURN_OFF_REG:
	case RESET_STATUS_REG:
	case VERSION_STATUS_REG:
	case MAIN_CONTROL_REG:
	case PADS_PULL_REG:
	case BUCK_PULL_DOWN_REG:
	case LDO14_PULL_DOWN_REG:
	case LDO56_PULL_DOWN_REG:
	case VIN_CONTROL_REG:
	case PONKEY_TURNOFF_REG:
	case MASK_RANK_BUCK_REG:
	case MASK_RESET_BUCK_REG:
	case MASK_RANK_LDO_REG:
	case MASK_RESET_LDO_REG:
	case WATCHDOG_CONTROL_REG:
	case WATCHDOG_TIMER_REG:
	case BUCK_ICC_TURNOFF_REG:
	case LDO_ICC_TURNOFF_REG:
	case BUCK1_CONTROL_REG:
	case BUCK2_CONTROL_REG:
	case BUCK3_CONTROL_REG:
	case BUCK4_CONTROL_REG:
	case VREF_DDR_CONTROL_REG:
	case LDO1_CONTROL_REG:
	case LDO2_CONTROL_REG:
	case LDO3_CONTROL_REG:
	case LDO4_CONTROL_REG:
	case LDO5_CONTROL_REG:
	case LDO6_CONTROL_REG:
	case BUCK1_PWRCTRL_REG:
	case BUCK2_PWRCTRL_REG:
	case BUCK3_PWRCTRL_REG:
	case BUCK4_PWRCTRL_REG:
	case VREF_DDR_PWRCTRL_REG:
	case LDO1_PWRCTRL_REG:
	case LDO2_PWRCTRL_REG:
	case LDO3_PWRCTRL_REG:
	case LDO4_PWRCTRL_REG:
	case LDO5_PWRCTRL_REG:
	case LDO6_PWRCTRL_REG:
	case USB_CONTROL_REG:
	case ITLATCH1_REG:
	case ITLATCH2_REG:
	case ITLATCH3_REG:
	case ITLATCH4_REG:
	case ITSETLATCH1_REG:
	case ITSETLATCH2_REG:
	case ITSETLATCH3_REG:
	case ITSETLATCH4_REG:
	case ITCLEARLATCH1_REG:
	case ITCLEARLATCH2_REG:
	case ITCLEARLATCH3_REG:
	case ITCLEARLATCH4_REG:
	case ITMASK1_REG:
	case ITMASK2_REG:
	case ITMASK3_REG:
	case ITMASK4_REG:
	case ITSETMASK1_REG:
	case ITSETMASK2_REG:
	case ITSETMASK3_REG:
	case ITSETMASK4_REG:
	case ITCLEARMASK1_REG:
	case ITCLEARMASK2_REG:
	case ITCLEARMASK3_REG:
	case ITCLEARMASK4_REG:
	case ITSOURCE1_REG:
	case ITSOURCE2_REG:
	case ITSOURCE3_REG:
	case ITSOURCE4_REG:
	case NVM_SECTOR3_0:
	case NVM_SECTOR3_1:
	case NVM_SECTOR3_2:
	case NVM_SECTOR3_3:
	case NVM_SECTOR3_4:
	case NVM_SECTOR3_5:
	case NVM_SECTOR3_6:
	case NVM_SECTOR3_7:
		return true;
	default:
		return false;
	}
}

static bool stpmu1_reg_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAIN_CONTROL_REG:
	case PADS_PULL_REG:
	case BUCK_PULL_DOWN_REG:
	case LDO14_PULL_DOWN_REG:
	case LDO56_PULL_DOWN_REG:
	case VIN_CONTROL_REG:
	case PONKEY_TURNOFF_REG:
	case MASK_RANK_BUCK_REG:
	case MASK_RESET_BUCK_REG:
	case MASK_RANK_LDO_REG:
	case MASK_RESET_LDO_REG:
	case WATCHDOG_CONTROL_REG:
	case WATCHDOG_TIMER_REG:
	case BUCK_ICC_TURNOFF_REG:
	case LDO_ICC_TURNOFF_REG:
	case BUCK1_CONTROL_REG:
	case BUCK2_CONTROL_REG:
	case BUCK3_CONTROL_REG:
	case BUCK4_CONTROL_REG:
	case VREF_DDR_CONTROL_REG:
	case LDO1_CONTROL_REG:
	case LDO2_CONTROL_REG:
	case LDO3_CONTROL_REG:
	case LDO4_CONTROL_REG:
	case LDO5_CONTROL_REG:
	case LDO6_CONTROL_REG:
	case BUCK1_PWRCTRL_REG:
	case BUCK2_PWRCTRL_REG:
	case BUCK3_PWRCTRL_REG:
	case BUCK4_PWRCTRL_REG:
	case VREF_DDR_PWRCTRL_REG:
	case LDO1_PWRCTRL_REG:
	case LDO2_PWRCTRL_REG:
	case LDO3_PWRCTRL_REG:
	case LDO4_PWRCTRL_REG:
	case LDO5_PWRCTRL_REG:
	case LDO6_PWRCTRL_REG:
	case USB_CONTROL_REG:
	case ITSETLATCH1_REG:
	case ITSETLATCH2_REG:
	case ITSETLATCH3_REG:
	case ITSETLATCH4_REG:
	case ITCLEARLATCH1_REG:
	case ITCLEARLATCH2_REG:
	case ITCLEARLATCH3_REG:
	case ITCLEARLATCH4_REG:
	case ITSETMASK1_REG:
	case ITSETMASK2_REG:
	case ITSETMASK3_REG:
	case ITSETMASK4_REG:
	case ITCLEARMASK1_REG:
	case ITCLEARMASK2_REG:
	case ITCLEARMASK3_REG:
	case ITCLEARMASK4_REG:
	case NVM_SECTOR3_0:
	case NVM_SECTOR3_1:
	case NVM_SECTOR3_2:
	case NVM_SECTOR3_3:
	case NVM_SECTOR3_4:
	case NVM_SECTOR3_5:
	case NVM_SECTOR3_6:
	case NVM_SECTOR3_7:
		return true;
	default:
		return false;
	}
}

static bool stpmu1_reg_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TURN_ON_REG:
	case TURN_OFF_REG:
	case ICC_LDO_TURN_OFF_REG:
	case ICC_BUCK_TURN_OFF_REG:
	case RESET_STATUS_REG:
	case ITLATCH1_REG:
	case ITLATCH2_REG:
	case ITLATCH3_REG:
	case ITLATCH4_REG:
	case ITSOURCE1_REG:
	case ITSOURCE2_REG:
	case ITSOURCE3_REG:
	case ITSOURCE4_REG:
	case WATCHDOG_CONTROL_REG:
		return true;
	default:
		return false;
	}
}

/*
 * stpmu1 pmic has switched off due to (SWITCH_OFF_STATUS):
 * 0x01 Swoff bit programming
 * 0x02 VIN below VIN_OK_f
 * 0x04 Thermal shutdown threshold reached
 * 0x08 Short Circuit detection
 * 0x10 Watchdog expired
 *
 * If Short Circuit is cause of Switch Off , get details of guilty Regulator
 */
static ssize_t show_switch_off_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int value, turn_off_ldo = 0, turn_off_buck = 0;
	struct stpmu1_dev *pmic = dev_get_drvdata(dev);

	ret = regmap_read(pmic->regmap, TURN_OFF_REG, &value);
	if (ret < 0)
		return ret;

	if (value == TURN_OFF_REG_ICC_EVENT) {
		ret = regmap_read(pmic->regmap, ICC_LDO_TURN_OFF_REG,
				  &turn_off_ldo);

		if (ret < 0)
			return ret;

		ret = regmap_read(pmic->regmap, ICC_BUCK_TURN_OFF_REG,
				  &turn_off_buck);

		if (ret < 0)
			return ret;
	}

	if ((!turn_off_ldo) && (!turn_off_buck))
		return sprintf(buf, "%#x\n", value);

	return sprintf(buf, "%#x LDO:%#x BUCK:%#x\n", value, turn_off_ldo,
		       turn_off_buck);
}

/*
 * stpmu1 pmic has turned on due to (TURN_ON_STATUS):
 * 0x01 PonKey
 * 0x02 WakeUp
 * 0x04 VbusDet
 * 0x08 SWOUT detection
 * 0x10 AutoTunON
 */
static ssize_t show_turn_on_status(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int value;
	struct stpmu1_dev *pmic = dev_get_drvdata(dev);

	ret = regmap_read(pmic->regmap, TURN_ON_REG, &value);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%#x\n", value);
}

/*
 * stpmu1 pmic has reseted due to (RESET_STATUS):
 * 0x01 Reset from SoC/MPU
 * 0x02 Reset due to SWOFF with Restart Request
 * 0x04 Reset due to Watchdog with Restart Request
 * 0x08 Reset due to POnKey with Restart Request
 * 0x10 Reset due to VinOK with VBUS present
 * 0x30 LDO4 status inputs
 * 0x80 PMIC Power Mode (HP or LP)
 */
static ssize_t show_reset_mode_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int value;
	struct stpmu1_dev *pmic = dev_get_drvdata(dev);

	ret = regmap_read(pmic->regmap, RESET_STATUS_REG, &value);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%#x\n", value);
}

static ssize_t show_hw_version(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int value;
	struct stpmu1_dev *pmic = dev_get_drvdata(dev);

	ret = regmap_read(pmic->regmap, VERSION_STATUS_REG, &value);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%#x\n", value);
}

static DEVICE_ATTR(switch_off_status, 0444, show_switch_off_status, NULL);
static DEVICE_ATTR(turn_on_status, 0444, show_turn_on_status, NULL);
static DEVICE_ATTR(reset_mode_status, 0444, show_reset_mode_status, NULL);
static DEVICE_ATTR(hw_version, 0444, show_hw_version, NULL);

static struct attribute *stpmu1_sysfs_entries[] = {
	&dev_attr_switch_off_status.attr,
	&dev_attr_turn_on_status.attr,
	&dev_attr_reset_mode_status.attr,
	&dev_attr_hw_version.attr,
	NULL,
};

static struct attribute_group stpmu1_attr_group = {
	.attrs = stpmu1_sysfs_entries,
};

static int stpmu1_configure_from_dt(struct stpmu1_dev *pmic_dev)
{
	struct device_node *np = pmic_dev->np;
	u32 reg = 0;
	int ret = 0;
	int irq;

	irq = of_irq_get(np, 0);
	if (irq <= 0) {
		dev_err(pmic_dev->dev,
			"Failed to get irq config: %d\n", irq);
		return irq ? irq : -ENODEV;
	}
	pmic_dev->irq = irq;

	if (!of_property_read_u32(np, "st,main_control_register", &reg)) {
		ret = regmap_update_bits(pmic_dev->regmap,
					 MAIN_CONTROL_REG,
					 PWRCTRL_POLARITY_HIGH |
					 PWRCTRL_PIN_VALID |
					 RESTART_REQUEST_ENABLED,
					 reg);
		if (ret) {
			dev_err(pmic_dev->dev,
				"Failed to update main control register: %d\n",
				ret);
			return ret;
		}
	}

	if (!of_property_read_u32(np, "st,pads_pull_register", &reg)) {
		ret = regmap_update_bits(pmic_dev->regmap,
					 PADS_PULL_REG,
					 WAKEUP_DETECTOR_DISABLED |
					 PWRCTRL_PD_ACTIVE |
					 PWRCTRL_PU_ACTIVE |
					 WAKEUP_PD_ACTIVE,
					 reg);
		if (ret) {
			dev_err(pmic_dev->dev,
				"Failed to update pads control register: %d\n",
				ret);
			return ret;
		}
	}

	if (!of_property_read_u32(np, "st,vin_control_register", &reg)) {
		ret = regmap_update_bits(pmic_dev->regmap,
					 VIN_CONTROL_REG,
					 VINLOW_CTRL_REG_MASK,
					 reg);
		if (ret) {
			dev_err(pmic_dev->dev,
				"Failed to update vin control register: %d\n",
				ret);
			return ret;
		}
	}

	if (!of_property_read_u32(np, "st,usb_control_register", &reg)) {
		ret = regmap_update_bits(pmic_dev->regmap, USB_CONTROL_REG,
					 BOOST_OVP_DISABLED |
					 VBUS_OTG_DETECTION_DISABLED |
					 SW_OUT_DISCHARGE |
					 VBUS_OTG_DISCHARGE |
					 OCP_LIMIT_HIGH,
					 reg);
		if (ret) {
			dev_err(pmic_dev->dev,
				"Failed to update usb control register: %d\n",
				ret);
			return ret;
		}
	}

	return 0;
}

/* API function to finalize Sw shutdown procedure and switch-off PMIC Hw */
int stpmu1_sw_switch_off(struct stpmu1_dev *pmic_dev)
{
	/* Update bit 0 of MAIN_CONTROL_REG */
	return (regmap_update_bits
		(pmic_dev->regmap, MAIN_CONTROL_REG,
		 SOFTWARE_SWITCH_OFF_ENABLED,
		 SOFTWARE_SWITCH_OFF_ENABLED));
}
EXPORT_SYMBOL_GPL(stpmu1_sw_switch_off);

int stpmu1_device_init(struct stpmu1_dev *pmic_dev)
{
	int ret;
	unsigned int val;

	pmic_dev->regmap =
	    devm_regmap_init_i2c(pmic_dev->i2c, &stpmu1_regmap_config);

	if (IS_ERR(pmic_dev->regmap)) {
		ret = PTR_ERR(pmic_dev->regmap);
		dev_err(pmic_dev->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = stpmu1_configure_from_dt(pmic_dev);
	if (ret < 0) {
		dev_err(pmic_dev->dev,
			"Unable to configure PMIC from Device Tree: %d\n", ret);
		return ret;
	}

	/* Read Version ID */
	ret = regmap_read(pmic_dev->regmap, VERSION_STATUS_REG, &val);
	if (ret < 0) {
		dev_err(pmic_dev->dev, "Unable to read pmic version\n");
		return ret;
	}
	dev_dbg(pmic_dev->dev, "PMIC Chip Version: 0x%x\n", val);

	/* Initialize PMIC IRQ Chip & IRQ domains associated */
	ret = devm_regmap_add_irq_chip(pmic_dev->dev, pmic_dev->regmap,
				       pmic_dev->irq, IRQF_TRIGGER_FALLING,
				       0, &stpmu1_regmap_irq_chip,
				       &pmic_dev->irq_data);

	if (ret < 0) {
		dev_err(pmic_dev->dev, "IRQ Chip registration failed: %d\n",
			ret);
		goto err;
	}

	ret = sysfs_create_group(&pmic_dev->dev->kobj, &stpmu1_attr_group);
	if (ret) {
		dev_err(pmic_dev->dev, "error creating sysfs entries\n");
		goto err;
	}

err:
	return ret;
}

void stpmu1_device_remove(struct stpmu1_dev *pmic_dev)
{
	sysfs_remove_group(&pmic_dev->dev->kobj, &stpmu1_attr_group);
}

static const struct of_device_id stpmu1_dt_match[] = {
	{.compatible = "st,stpmu1"},
	{},
};

MODULE_DEVICE_TABLE(of, stpmu1_dt_match);

static int stpmu1_remove(struct i2c_client *i2c)
{
	struct stpmu1_dev *pmic_dev = i2c_get_clientdata(i2c);

	of_platform_depopulate(pmic_dev->dev);
	stpmu1_device_remove(pmic_dev);

	return 0;
}

static int stpmu1_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	struct stpmu1_dev *pmic;
	struct device *dev = &i2c->dev;
	int ret = 0;

	pmic = devm_kzalloc(dev, sizeof(struct stpmu1_dev), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	pmic->np = dev->of_node;

	dev_set_drvdata(dev, pmic);
	pmic->dev = dev;
	pmic->i2c = i2c;

	ret = stpmu1_device_init(pmic);
	if (ret < 0)
		goto err;

	ret = of_platform_populate(pmic->np, NULL, NULL, pmic->dev);

	dev_dbg(dev, "stpmu1 driver probed\n");
err:
	return ret;
}

static const struct i2c_device_id stpmu1_id[] = {
	{"stpmu1", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, stpmu1_id);

#ifdef CONFIG_PM_SLEEP
static int stpmu1_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct stpmu1_dev *pmic_dev = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(pmic_dev->irq);
	return 0;
}

static int stpmu1_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct stpmu1_dev *pmic_dev = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		disable_irq_wake(pmic_dev->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(stpmu1_pm, stpmu1_suspend, stpmu1_resume);

static struct i2c_driver stpmu1_driver = {
	.driver = {
		   .name = "stpmu1",
		   .owner = THIS_MODULE,
		   .pm = &stpmu1_pm,
		   .of_match_table = of_match_ptr(stpmu1_dt_match),
		   },
	.probe = stpmu1_probe,
	.remove = stpmu1_remove,
	.id_table = stpmu1_id,
};

module_i2c_driver(stpmu1_driver);

MODULE_DESCRIPTION("STPMU1 PMIC I2C Client");
MODULE_AUTHOR("<philippe.peurichard@st.com>");
MODULE_LICENSE("GPL");
