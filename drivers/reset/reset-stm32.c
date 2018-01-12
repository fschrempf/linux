/*
 *
 * Copyright (C) STMicroelectronics SA 2017
 * Author(s): Gabriel Fernandez <gabriel.fernandez@st.com>
 * for STMicroelectronics.
 *
 * License terms: GPL V2.0.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/arm-smccc.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

struct reset_secure {
	u32 id_service;
	int (*is_soc_secured)(struct reset_controller_dev *rcdev);
	int (*is_id_secured)(unsigned long id);
};

struct reset_stm32_data {
	struct regmap *regmap;
	struct reset_controller_dev rcdev;
	u32 clr_offset;
	struct reset_secure *secure;
	bool soc_secured;
};

static inline struct reset_stm32_data *
to_reset_stm32_data(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct reset_stm32_data, rcdev);
}

static int reset_stm32_is_id_secured(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);

	if (data->secure->is_id_secured)
		return data->secure->is_id_secured(id);

	return 0;
}

static int reset_stm32_secure_update(struct reset_controller_dev *rcdev,
				     unsigned long id, bool assert)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);
	struct arm_smccc_res res;
	int bank = id / BITS_PER_LONG;
	int offset = id % BITS_PER_LONG;

	if (assert)
		arm_smccc_smc(data->secure->id_service, 0x1,
			      (bank * 4), BIT(offset), 0, 0, 0, 0, &res);
	else
		arm_smccc_smc(data->secure->id_service, 0x1,
			      (bank * 4) + 0x4, BIT(offset), 0, 0, 0, 0, &res);

	return 0;
}

static int reset_stm32_update(struct reset_controller_dev *rcdev,
			      unsigned long id, bool assert)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);
	int reg_width = sizeof(u32);
	int bank = id / (reg_width * BITS_PER_BYTE);
	int offset = id % (reg_width * BITS_PER_BYTE);
	unsigned int reg, val, mask;

	val = BIT(offset);
	mask = BIT(offset);
	reg = (bank * reg_width);

	if (assert) {
		regmap_write_bits(data->regmap, reg, mask, val);
	} else {
		if (data->clr_offset) {
			reg += data->clr_offset;
			mask = ~0;
		} else {
			val = 0;
		}

		regmap_write_bits(data->regmap, reg, mask, val);
	}

	return 0;
}

static int reset_stm32_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);

	if (data->soc_secured && reset_stm32_is_id_secured(rcdev, id))
		return reset_stm32_secure_update(rcdev, id, true);

	return reset_stm32_update(rcdev, id, true);
}

static int reset_stm32_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);

	if (data->soc_secured && reset_stm32_is_id_secured(rcdev, id))
		return reset_stm32_secure_update(rcdev, id, false);

	return reset_stm32_update(rcdev, id, false);

}

static int reset_stm32_status(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);
	int reg_width = sizeof(u32);
	int bank = id / (reg_width * BITS_PER_BYTE);
	int offset = id % (reg_width * BITS_PER_BYTE);
	u32 reset_state, ret;

	ret = regmap_read(data->regmap, (bank * reg_width), &reset_state);
	if (ret)
		return ret;

	return (reset_state & BIT(offset));
}

const struct reset_control_ops reset_stm32_ops = {
	.assert		= reset_stm32_assert,
	.deassert	= reset_stm32_deassert,
	.status		= reset_stm32_status,
};

struct reset_stm32_devdata {
	u32 clr_offset;
	struct reset_secure *secure;
};

#define SMT32_SPI6_R 3136
#define STM32_AXIM_R 3216

static int is_stm32_id_secured(unsigned long id)
{
	if (id >= SMT32_SPI6_R && id <= STM32_AXIM_R)
		return 1;

	return 0;
}

#define STM32_RCC_TZCR 0x0

static int is_stm32_soc_secured(struct reset_controller_dev *rcdev)
{
	struct reset_stm32_data *data = to_reset_stm32_data(rcdev);
	int tzc_val = 0;

	regmap_read(data->regmap, STM32_RCC_TZCR, &tzc_val);

	return tzc_val & 0x1;
}

#define STM32MP1_SVC_RCC 0x82001000

struct reset_secure stm32_secure = {
	.id_service = STM32MP1_SVC_RCC,
	.is_soc_secured = is_stm32_soc_secured,
	.is_id_secured = is_stm32_id_secured,
};

struct reset_stm32_devdata reset_stm32 = {
	.clr_offset = 0x4,
	.secure = &stm32_secure,
};

static const struct of_device_id reset_stm32_dt_ids[] = {
	{ .compatible = "st,stm32mp1-rcc-rst", &reset_stm32 },
	{ /* sentinel */ },
};

static int reset_stm32_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const struct reset_stm32_devdata *devdata;
	struct reset_stm32_data *data;
	struct regmap *regmap;
	struct resource res;

	devdata = of_device_get_match_data(dev);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	of_address_to_resource(np->parent, 0, &res);

	data->regmap = regmap;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = resource_size(&res) * BITS_PER_BYTE;
	data->rcdev.ops = &reset_stm32_ops;
	data->rcdev.of_node = dev->of_node;

	if (devdata) {
		data->clr_offset = devdata->clr_offset;

		if (devdata->secure) {
			data->secure = devdata->secure;
			data->soc_secured =
				data->secure->is_soc_secured(&data->rcdev);
		}
	}

	return devm_reset_controller_register(dev, &data->rcdev);
}

static struct platform_driver reset_stm32_driver = {
	.probe	= reset_stm32_probe,
	.driver = {
		.name		= "stm32-reset",
		.of_match_table	= reset_stm32_dt_ids,
	},
};

static int __init stm32_reset_init(void)
{
	return platform_driver_register(&reset_stm32_driver);
}
postcore_initcall(stm32_reset_init);
