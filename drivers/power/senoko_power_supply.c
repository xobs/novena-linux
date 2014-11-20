/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

struct senoko;

int senoko_read(struct senoko *senoko, int offset);
int senoko_write(struct senoko *senoko, int offset, u8 value);

#define REG_POWER                       0x0f
#define REG_POWER_STATE_MASK            (3 << 0)
#define REG_POWER_STATE_ON              (0 << 0)
#define REG_POWER_STATE_OFF             (1 << 0)
#define REG_POWER_STATE_REBOOT          (2 << 0)
#define REG_POWER_WDT_MASK              (1 << 2)
#define REG_POWER_WDT_DISABLE           (0 << 2)
#define REG_POWER_WDT_ENABLE            (1 << 2)
#define REG_POWER_WDT_STATE             (1 << 2)
#define REG_POWER_AC_STATUS_MASK        (1 << 3)
#define REG_POWER_AC_STATUS_SHIFT       (3)
#define REG_POWER_PB_STATUS_MASK        (1 << 4)
#define REG_POWER_PB_STATUS_SHIFT       (4)
#define REG_POWER_KEY_MASK              (3 << 6)
#define REG_POWER_KEY_READ              (1 << 6)
#define REG_POWER_KEY_WRITE             (2 << 6)

struct senoko_power_supply {
	struct senoko *senoko;
	struct regmap *regmap;
	struct device *dev;
	bool wakeup_enabled;
	struct work_struct work;
	struct power_supply *supply;
	struct power_supply_desc supply_desc;
	int num_supplicants;
	int irq;
	char **supplicant_names;
	void (*old_power_off)(void);
};

static enum power_supply_property senoko_power_supply_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static void senoko_supply_post_change(struct work_struct *_work)
{
	struct senoko_power_supply *ss;
	ss = container_of(_work, struct senoko_power_supply, work);
	power_supply_changed(ss->supply);
}

static irqreturn_t senoko_supply_changed(int irq, void *devid)
{
	struct senoko_power_supply *ss = devid;

	schedule_work(&ss->work);

	return IRQ_HANDLED;
}

static int senoko_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct senoko_power_supply *ss;
	int ret;

	ss = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = senoko_read(ss->senoko, REG_POWER);
		if (ret < 0)
			return -ENODATA;

		val->intval = !!(ret & REG_POWER_AC_STATUS_MASK);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int senoko_power_supply_probe(struct platform_device *pdev)
{
	struct senoko_power_supply *ss;
	struct power_supply_desc *desc;
	struct power_supply_config psy_cfg = {};
	struct device_node *np = pdev->dev.parent->of_node;
	int ret;

	ss = devm_kzalloc(&pdev->dev, sizeof(*ss), GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->irq = platform_get_irq(pdev, 0);
	if (ss->irq < 0)
		return ss->irq;

	ss->senoko = dev_get_drvdata(pdev->dev.parent);
	ss->dev = &pdev->dev;
	INIT_WORK(&ss->work, senoko_supply_post_change);

	desc = &ss->supply_desc;

	desc->name = "senoko-ac-supply";
	desc->type = POWER_SUPPLY_TYPE_MAINS;
	desc->properties = senoko_power_supply_props;
	desc->num_properties = ARRAY_SIZE(senoko_power_supply_props);
	desc->get_property = senoko_get_property;

	ss->num_supplicants = of_property_count_strings(np, "supplied-to");
	if (ss->num_supplicants > 0) {
		int i;
		int count = ss->num_supplicants;
		char **names = devm_kzalloc(&pdev->dev,
					    count * sizeof(*names),
					    GFP_KERNEL);
		for (i = 0; i < count; i++) {
			const char *s;
			of_property_read_string_index(np, "supplied-to", i, &s);
			names[i] = devm_kstrdup(&pdev->dev, s, GFP_KERNEL);
		}
		ss->supplicant_names = names;
	}

	psy_cfg.supplied_to = ss->supplicant_names;
	psy_cfg.num_supplicants = ss->num_supplicants;
	psy_cfg.drv_data = ss;

	ss->supply = power_supply_register(ss->dev, desc, &psy_cfg);
	if (IS_ERR(ss->supply)) {
		ret = PTR_ERR(ss->supply);
		dev_err(&pdev->dev, "failed to register supply: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, ss);
	dev_set_drvdata(&pdev->dev, ss);

	ret = devm_request_threaded_irq(&pdev->dev, ss->irq, NULL,
					senoko_supply_changed,
					IRQF_ONESHOT,
					"senoko-supply", ss);

        if (ret < 0) {
		dev_err(&pdev->dev, "unable to request irq: %d\n", ret);
		return ret;
	}

	device_init_wakeup(&pdev->dev, 0);

	schedule_work(&ss->work);

	return 0;
}

static int senoko_power_supply_remove(struct platform_device *pdev)
{
	struct senoko_power_supply *ss = platform_get_drvdata(pdev);

	pm_power_off = ss->old_power_off;
	cancel_work_sync(&ss->work);
	power_supply_unregister(ss->supply);
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

static int senoko_power_supply_suspend(struct device *dev)
{
	struct senoko_power_supply *ss = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(ss->irq);

	return 0;
}

static int senoko_power_supply_resume(struct device *dev)
{
	struct senoko_power_supply *ss = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(ss->irq);

	schedule_work(&ss->work);

	return 0;
}

static SIMPLE_DEV_PM_OPS(senoko_power_supply_pm_ops,
			 senoko_power_supply_suspend,
			 senoko_power_supply_resume);

static const struct of_device_id senoko_dt_ids[] = {
	{ .compatible = "kosagi,senoko-power-supply" },
	{ }
};
MODULE_DEVICE_TABLE(of, senoko_dt_ids);

static struct platform_driver senoko_power_supply_driver = {
	.driver = {
		.name		= "senoko-power-supply",
		.owner		= THIS_MODULE,
		.of_match_table	= senoko_dt_ids,
		.pm 		= &senoko_power_supply_pm_ops,
	},
	.probe = senoko_power_supply_probe,
	.remove = senoko_power_supply_remove,
};
module_platform_driver(senoko_power_supply_driver);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Power supply driver for Kosagi Senoko");
MODULE_LICENSE("GPL v2");
