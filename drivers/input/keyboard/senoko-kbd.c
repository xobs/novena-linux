/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License, version 2
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

struct senoko;

int senoko_read(struct senoko *senoko, int offset);
int senoko_write(struct senoko *senoko, int offset, u8 value);

#define REG_POWER			0x0f
#define REG_POWER_STATE_MASK		(3 << 0)
#define REG_POWER_STATE_ON		(0 << 0)
#define REG_POWER_STATE_OFF		(1 << 0)
#define REG_POWER_STATE_REBOOT		(2 << 0)
#define REG_POWER_WDT_MASK		(1 << 2)
#define REG_POWER_WDT_DISABLE		(0 << 2)
#define REG_POWER_WDT_ENABLE		(1 << 2)
#define REG_POWER_WDT_STATE		(1 << 2)
#define REG_POWER_AC_STATUS_MASK	(1 << 3)
#define REG_POWER_AC_STATUS_SHIFT	(3)
#define REG_POWER_PB_STATUS_MASK	(1 << 4)
#define REG_POWER_PB_STATUS_SHIFT	(4)
#define REG_POWER_KEY_MASK		(3 << 6)
#define REG_POWER_KEY_READ		(1 << 6)
#define REG_POWER_KEY_WRITE		(2 << 6)

struct senoko_keypad {
	struct senoko *senoko;
	struct input_dev *input;
	int irq;
};

static irqreturn_t senoko_keypad_irq(int irq, void *dev)
{
	struct senoko_keypad *keypad = dev;
	struct senoko *senoko = keypad->senoko;
	struct input_dev *input = keypad->input;
	bool up = false;
	int val;

	val = senoko_read(senoko, REG_POWER);
	if (val < 0) {
		dev_err(&input->dev, "Unable to read key status: %d\n", val);
		return IRQ_HANDLED;
	}

	if (! (val & REG_POWER_PB_STATUS_MASK))
		up = true;

	input_report_key(input, KEY_POWER, !up);
	input_sync(input);

	return IRQ_HANDLED;
}

static int senoko_keypad_probe(struct platform_device *pdev)
{
	struct senoko *senoko = dev_get_drvdata(pdev->dev.parent);
	struct senoko_keypad *keypad;
	struct input_dev *input;
	int error;

	keypad = devm_kzalloc(&pdev->dev, sizeof(struct senoko_keypad),
			      GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	keypad->irq = platform_get_irq(pdev, 0);
	if (keypad->irq < 0)
		return keypad->irq;

	input = devm_input_allocate_device(&pdev->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Senoko keypad";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &pdev->dev;
	input_set_capability(input, EV_KEY, KEY_POWER);

	keypad->senoko = senoko;
	keypad->input = input;

	error = devm_request_threaded_irq(&pdev->dev, keypad->irq,
					  NULL, senoko_keypad_irq,
					  IRQF_ONESHOT, "senoko-keypad", keypad);
	if (error) {
		dev_err(&pdev->dev, "unable to get irq: %d\n", error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(&pdev->dev,
			"unable to register input device: %d\n", error);
		return error;
	}

	dev_set_drvdata(&pdev->dev, keypad);
	device_init_wakeup(&pdev->dev, 1);
	platform_set_drvdata(pdev, keypad);

	return 0;
}

static int senoko_keypad_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

static int senoko_keypad_suspend(struct device *dev)
{
	struct senoko_keypad *keypad = dev_get_drvdata(dev);

        if (device_may_wakeup(dev))
		enable_irq_wake(keypad->irq);

	return 0;
}

static int senoko_keypad_resume(struct device *dev)
{
	struct senoko_keypad *keypad = dev_get_drvdata(dev);

        if (device_may_wakeup(dev))
		disable_irq_wake(keypad->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(senoko_keypad_pm_ops,
			 senoko_keypad_suspend,
			 senoko_keypad_resume);

static struct platform_driver senoko_keypad_driver = {
	.driver.name	= "senoko-keypad",
	.driver.owner	= THIS_MODULE,
	.driver.pm	= &senoko_keypad_pm_ops,
	.probe		= senoko_keypad_probe,
	.remove		= senoko_keypad_remove,
};
module_platform_driver(senoko_keypad_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Senoko keypad driver");
MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
