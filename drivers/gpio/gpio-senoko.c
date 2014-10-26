/*
 * GPIO Chip driver for Analog Devices
 * SENOKO/ADP5587 I/O Expander and QWERTY Keypad Controller
 *
 * Copyright 2009-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#define SIGNATURE 0
#define VERSION_MAJOR 1
#define VERSION_MINOR 1
#define GPIO_DIR_A 0x10
#define GPIO_VAL_A 0x12
#define GPIO_IRQ_RISE_A 0x14
#define GPIO_IRQ_FALL_A 0x16
#define GPIO_IRQ_STAT_A 0x18

struct i2c_registers {
	uint8_t signature;		/* 0x00 */
	uint8_t version_major;		/* 0x01 */
	uint8_t version_minor;		/* 0x02 */
	uint8_t uptime[4];		/* 0x03 - 0x06 */
	uint8_t power;			/* 0x07 */
	uint8_t wdt_seconds;		/* 0x08 */
	uint8_t ier;			/* 0x09 */
	uint8_t irq_status;		/* 0x0a */
	uint8_t key_status;		/* 0x0b */
	uint8_t padding0[4];		/* 0x0b - 0x0f */

	/* -- GPIO block -- */
	uint8_t gpio_dir_a;		/* 0x10 */
	uint8_t gpio_dir_b;		/* 0x11 */
	uint8_t gpio_val_a;		/* 0x12 */
	uint8_t gpio_val_b;		/* 0x13 */
	uint8_t gpio_irq_rise_a;	/* 0x14 */
	uint8_t gpio_irq_rise_b;	/* 0x15 */
	uint8_t gpio_irq_fall_a;	/* 0x16 */
	uint8_t gpio_irq_fall_b;	/* 0x17 */
	uint8_t gpio_irq_stat_a;	/* 0x18 */
	uint8_t gpio_irq_stat_b;	/* 0x19 */
	uint8_t padding1[6];		/* 0x1a - 0x1f */

	/* -- RTC block -- */
	uint8_t seconds[4];		/* 0x20 - 0x23 */
};

#define SENOKO_MAXGPIO 8

struct senoko_gpio {
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	struct i2c_registers registers;
	struct mutex lock;	/* protect cached dir, dat_out */
	/* protect serialized access to the interrupt controller bus */
	struct mutex irq_lock;
	int irq_gpio;
	enum of_gpio_flags irq_trigger;
	int irq;
	unsigned gpio_start;

	uint8_t int_en_fall;
	uint8_t int_en_rise;
	uint8_t irq_mask;
};

static inline struct senoko_gpio *to_senoko_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct senoko_gpio, gpio_chip);
}

static int senoko_read(struct senoko_gpio *dev, int offset)
{
	return i2c_master_recv(dev->client,
			(char *)&dev->registers,
			sizeof(dev->registers));
}

static int senoko_write(struct senoko_gpio *dev, int offset)
{
	u8 bfr[sizeof(dev->registers) + 1];

	bfr[0] = 0;
	memcpy(bfr + 1, &dev->registers, sizeof(dev->registers));
	return i2c_master_send(dev->client, bfr, sizeof(bfr));
}

static int senoko_gpio_get_value(struct gpio_chip *chip, unsigned off)
{
	struct senoko_gpio *dev = to_senoko_gpio(chip);
	int val;

	mutex_lock(&dev->lock);

	senoko_read(dev, GPIO_VAL_A);
	val = dev->registers.gpio_val_a;

	mutex_unlock(&dev->lock);

	return !!(val & (1 << off));
}

static void senoko_gpio_set_value(struct gpio_chip *chip,
				   unsigned off, int val)
{
	struct senoko_gpio *dev = to_senoko_gpio(chip);

	mutex_lock(&dev->lock);
	if (val)
		dev->registers.gpio_val_a |= (1 << off);
	else
		dev->registers.gpio_val_a &= ~(1 << off);

	senoko_write(dev, GPIO_VAL_A);
	mutex_unlock(&dev->lock);
}

static int senoko_gpio_direction_input(struct gpio_chip *chip, unsigned off)
{
	int ret;
	struct senoko_gpio *dev = to_senoko_gpio(chip);

	mutex_lock(&dev->lock);
	dev->registers.gpio_dir_a &= ~(1 << off);
	ret = senoko_write(dev, GPIO_DIR_A);
	mutex_unlock(&dev->lock);

	return ret;
}

static int senoko_gpio_direction_output(struct gpio_chip *chip,
					 unsigned off, int val)
{
	int ret;
	struct senoko_gpio *dev = to_senoko_gpio(chip);

	mutex_lock(&dev->lock);
	dev->registers.gpio_dir_a |= (1 << off);

	if (val)
		dev->registers.gpio_val_a |= (1 << off);
	else
		dev->registers.gpio_val_a &= ~(1 << off);

	ret = senoko_write(dev, GPIO_VAL_A);
	//ret |= senoko_write(dev, GPIO_DIR_A);
	mutex_unlock(&dev->lock);

	return ret;
}

static void senoko_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct senoko_gpio *dev = to_senoko_gpio(gc);

	mutex_lock(&dev->irq_lock);
}

 /*
  * genirq core code can issue chip->mask/unmask from atomic context.
  * This doesn't work for slow busses where an access needs to sleep.
  * bus_sync_unlock() is therefore called outside the atomic context,
  * syncs the current irq mask state with the slow external controller
  * and unlocks the bus.
  */

static void senoko_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct senoko_gpio *dev = to_senoko_gpio(gc);

	dev->registers.gpio_irq_rise_a = dev->int_en_rise & ~dev->irq_mask;
	dev->registers.gpio_irq_fall_a = dev->int_en_fall & ~dev->irq_mask;
	senoko_write(dev, -1);

	mutex_unlock(&dev->irq_lock);
}

static void senoko_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct senoko_gpio *dev = to_senoko_gpio(gc);
	unsigned gpio = d->hwirq;

	dev->irq_mask &= ~gpio;
}

static void senoko_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct senoko_gpio *dev = to_senoko_gpio(gc);
	unsigned gpio = d->hwirq;

	dev->irq_mask |= gpio;
}

static int senoko_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct senoko_gpio *dev = to_senoko_gpio(gc);
	uint16_t gpio = d->hwirq;

	if (type & IRQ_TYPE_LEVEL_HIGH)
		dev->registers.gpio_irq_rise_a |= (1 << gpio);
	else
		dev->registers.gpio_irq_rise_a &= ~(1 << gpio);

	if (type & IRQ_TYPE_LEVEL_LOW)
		dev->registers.gpio_irq_fall_a |= (1 << gpio);
	else
		dev->registers.gpio_irq_fall_a &= ~(1 << gpio);

	senoko_write(dev, GPIO_IRQ_RISE_A);
	//senoko_write(dev GPIO_IRQ_FALL_A);

	return 0;
}

static struct irq_chip senoko_irq_chip = {
	.name			= "senoko",
	.irq_mask		= senoko_irq_mask,
	.irq_unmask		= senoko_irq_unmask,
	.irq_bus_lock		= senoko_irq_bus_lock,
	.irq_bus_sync_unlock	= senoko_irq_bus_sync_unlock,
	.irq_set_type		= senoko_irq_set_type,
};

static irqreturn_t senoko_irq_handler(int irq, void *devid)
{
	struct senoko_gpio *dev = devid;
	unsigned status, gpio;
	int ret;

	ret = senoko_read(dev, GPIO_IRQ_STAT_A);

	if (!ret) {
		status = dev->registers.gpio_val_a;
		for (gpio = 0; gpio < SENOKO_MAXGPIO; gpio++) {
			if (status & (1 << gpio)) {
				int child_irq = irq_find_mapping(
					dev->gpio_chip.irqdomain, gpio);
				handle_nested_irq(child_irq);
			}
		}
	}

	dev->registers.gpio_irq_stat_a = 0;
	senoko_write(dev, GPIO_IRQ_STAT_A);

	return IRQ_HANDLED;
}

static int senoko_gpio_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct senoko_gpio *dev;
	struct gpio_chip *gc;
	struct device_node *np = client->dev.of_node;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL)
		return -ENOMEM;

	mutex_init(&dev->irq_lock);
	mutex_init(&dev->lock);

	dev->client = client;

	gc = &dev->gpio_chip;
	gc->direction_input = senoko_gpio_direction_input;
	gc->direction_output = senoko_gpio_direction_output;
	gc->get = senoko_gpio_get_value;
	gc->set = senoko_gpio_set_value;
	gc->can_sleep = true;

	gc->base = -1;
	gc->of_node = np;
	gc->dev = &client->dev;
	gc->ngpio = SENOKO_MAXGPIO;
	gc->label = client->name;
	gc->owner = THIS_MODULE;
	gc->names = NULL; //pdata->names;

	ret = senoko_read(dev, 0);
	if (ret < 0)
		goto err;

	dev_info(&client->dev, "Senoko '%c' version %d.%d\n",
			dev->registers.signature,
			dev->registers.version_major,
			dev->registers.version_minor);

	dev->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0,
						&dev->irq_trigger);
	ret = devm_gpio_request_one(&client->dev, dev->irq_gpio,
					GPIOF_DIR_IN, "senoko-irq");
	dev->irq = gpio_to_irq(dev->irq_gpio);
	ret = devm_request_threaded_irq(&client->dev, dev->irq, NULL,
			senoko_irq_handler, dev->irq_trigger | IRQF_ONESHOT,
			"senoko-gpio", dev);
	if (ret) {
		dev_err(&client->dev, "unable to get irq: %d\n", ret);
		goto out_disable;
	}
	ret =  gpiochip_irqchip_add(gc,
				    &senoko_irq_chip,
				    0,
				    handle_simple_irq,
				    IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&client->dev, "could not connect irqchip to gpiochip\n");
		return ret;
	}

        ret = gpiochip_add(gc);
        if (ret) {
                dev_err(&client->dev, "unable to add gpiochip: %d\n", ret);
                goto out_disable;
        }

	i2c_set_clientdata(client, dev);

	return 0;

out_disable:
err:
	kfree(dev);
	return ret;
}

static int senoko_gpio_remove(struct i2c_client *client)
{
	struct senoko_gpio *dev = i2c_get_clientdata(client);

	gpiochip_remove(&dev->gpio_chip);

	kfree(dev);
	return 0;
}

static const struct of_device_id senoko_of_match[] = {
	{ .compatible = "kosagi,senoko", },
	{},
};
MODULE_DEVICE_TABLE(of, senoko_of_match);

static const struct i2c_device_id senoko_gpio_id[] = {
	{"senoko-gpio", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, senoko_gpio_id);

static struct i2c_driver senoko_gpio_driver = {
	.driver = {
		.name = "senoko-gpio",
		.of_match_table = senoko_of_match,
	},
	.probe = senoko_gpio_probe,
	.remove = senoko_gpio_remove,
	.id_table = senoko_gpio_id,
};

module_i2c_driver(senoko_gpio_driver);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Senoko GPIO Driver");
MODULE_LICENSE("GPL");
