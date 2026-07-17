// SPDX-License-Identifier: GPL-2.0+
/*
 * Broadcom AVS RO thermal sensor driver
 *
 * Ported from the Linux bcm2711_thermal driver
 * (drivers/thermal/broadcom/bcm2711_thermal.c,
 * Copyright (C) 2020 Stefan Wahren) to the U-Boot driver model.
 *
 * The sensor lives inside the AVS monitor block, which is described in the
 * device tree as a "syscon"/"simple-mfd" node.  This driver binds to the
 * "brcm,bcm2711-thermal" child of that block and reads the temperature via
 * the parent's regmap, exactly like the Linux driver.
 *
 * U-Boot has no thermal-zone infrastructure, so the calibration coefficients
 * that Linux reads from the thermal-zone node are taken from an optional
 * "coefficients = <slope offset>" property on this sensor node instead.  When
 * absent the BCM2712 defaults are used.
 */

#include <dm.h>
#include <regmap.h>
#include <syscon.h>
#include <thermal.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>
#include <linux/err.h>

#define AVS_RO_TEMP_STATUS		0x200
#define AVS_RO_TEMP_STATUS_VALID_MSK	(BIT(16) | BIT(10))
#define AVS_RO_TEMP_STATUS_DATA_MSK	GENMASK(9, 0)

/* BCM2712 calibration: temp(mC) = -550 * raw + 450000 */
#define BCM2712_DEFAULT_SLOPE		(-550)
#define BCM2712_DEFAULT_OFFSET		450000

struct bcm2711_thermal_priv {
	struct regmap *regmap;
	int slope;
	int offset;
};

static int bcm2711_thermal_get_temp(struct udevice *dev, int *temp)
{
	struct bcm2711_thermal_priv *priv = dev_get_priv(dev);
	uint val;
	int ret;
	long t;

	ret = regmap_read(priv->regmap, AVS_RO_TEMP_STATUS, &val);
	if (ret)
		return ret;

	if (!(val & AVS_RO_TEMP_STATUS_VALID_MSK))
		return -EIO;

	val &= AVS_RO_TEMP_STATUS_DATA_MSK;

	/* Convert a HW code to a temperature reading (millidegree celsius) */
	t = priv->slope * (long)val + priv->offset;

	*temp = t < 0 ? 0 : t;

	return 0;
}

static int bcm2711_thermal_probe(struct udevice *dev)
{
	struct bcm2711_thermal_priv *priv = dev_get_priv(dev);
	u32 coef[2];

	priv->regmap = syscon_get_regmap(dev_get_parent(dev));
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "failed to get regmap from parent\n");
		return PTR_ERR(priv->regmap);
	}

	if (dev_read_u32_array(dev, "coefficients", coef, ARRAY_SIZE(coef))) {
		priv->slope = BCM2712_DEFAULT_SLOPE;
		priv->offset = BCM2712_DEFAULT_OFFSET;
	} else {
		priv->slope = (int)coef[0];
		priv->offset = (int)coef[1];
	}

	return 0;
}

static const struct dm_thermal_ops bcm2711_thermal_ops = {
	.get_temp	= bcm2711_thermal_get_temp,
};

static const struct udevice_id bcm2711_thermal_match[] = {
	{ .compatible = "brcm,bcm2711-thermal" },
	{ }
};

U_BOOT_DRIVER(bcm2711_thermal) = {
	.name		= "bcm2711_thermal",
	.id		= UCLASS_THERMAL,
	.of_match	= bcm2711_thermal_match,
	.ops		= &bcm2711_thermal_ops,
	.probe		= bcm2711_thermal_probe,
	.priv_auto	= sizeof(struct bcm2711_thermal_priv),
};
