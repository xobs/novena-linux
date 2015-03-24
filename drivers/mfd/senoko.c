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
#include <linux/mfd/core.h>
#include <linux/regmap.h>

#define SIGNATURE 0
#define VERSION_MAJOR 1
#define VERSION_MINOR 1

/*
 * IRQ 0: GPIO
 * IRQ 1: Keypad
 * IRQ 2: Power supply
 * IRQ 3: Wake alarm
 */
enum senoko_irq {
	senoko_gpio_irq = 0,
	senoko_keypad_irq = 1,
	senoko_senoko_power_supply_irq = 2,
	senoko_rtc_irq = 3,
	__senoko_num_irqs = 4,
};

struct i2c_registers {
	uint8_t signature;		/* 0x00 */
	uint8_t version_major;		/* 0x01 */
	uint8_t version_minor;		/* 0x02 */
	uint8_t features;		/* 0x03 */
	uint8_t uptime[4];		/* 0x04 - 0x07 */
	uint8_t irq_enable;		/* 0x08 */
	uint8_t irq_status;		/* 0x09 */
	uint8_t padding0[5];		/* 0x0a - 0x0e */
	uint8_t power;			/* 0x0f */

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
	uint8_t gpio_pull_ena_a;	/* 0x1a */
	uint8_t gpio_pull_ena_b;	/* 0x1b */
	uint8_t gpio_pull_dir_a;	/* 0x1c */
	uint8_t gpio_pull_dir_b;	/* 0x1d */
	uint8_t padding1[2];		/* 0x1e - 0x1f */

	/* -- RTC block -- */
	uint8_t seconds[4];		/* 0x20 - 0x23 */
	uint8_t alarm_seconds[4];	/* 0x24 - 0x27 */
	uint8_t wdt_seconds;		/* 0x28 */
};

struct senoko {
	struct i2c_client *client;
	struct device *dev;
	struct irq_domain *domain;

	/* protect serialized access to the interrupt controller bus */
	struct mutex irq_lock;

	int num_irqs;
	int irq_gpio;
	enum of_gpio_flags irq_trigger;
	int irq;

	/* IRQ Enable Register */
	int ier;
	int oldier;

	/* IRQ Wake Register, which IRQs are active during suspend */
	int iwr;

	/* A list of reported features */
	int features;

	struct regmap *regmap;

	/*
	 * If there's an error, delay by some amount.  This prevents
	 * overrunning the kernel log when Senoko is in reset.
	 */
	unsigned long timeout_backoff;

	/* Previous pm_power_off function */
	void (*old_power_off)(void);
};

static struct senoko *g_senoko;

#define REG_SIGNATURE			0x00
#define REG_VERSION_MAJOR		0x01
#define REG_VERSION_MINOR		0x02
#define REG_FEATURES			0x03
#define REG_FEATURES_BATTERY		(1 << 0)
#define REG_FEATURES_GPIO		(1 << 1)

#define REG_IRQ_ENABLE			0x08
#define REG_IRQ_STATUS			0x09
#define REG_IRQ_GPIO_MASK		(1 << 0)
#define REG_IRQ_KEYPAD_MASK		(1 << 1)
#define REG_IRQ_POWER_MASK		(1 << 2)
#define REG_IRQ_ALARM_MASK		(1 << 3)

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

#define REG_WATCHDOG_SECONDS		0x28

static bool senoko_regmap_is_volatile(struct device *dev, unsigned int reg)
{
        switch (reg) {
        case REG_SIGNATURE:
	case REG_VERSION_MAJOR:
	case REG_VERSION_MINOR:
	case REG_FEATURES:
                return false;
        default:
                return true;
        }
}

static bool senoko_regmap_is_writeable(struct device *dev, unsigned int reg)
{
        switch (reg) {
        case REG_IRQ_ENABLE:
	case REG_IRQ_STATUS:
	case REG_POWER:
	case REG_WATCHDOG_SECONDS:
                return true;
        default:
                return false;
        }
}

static struct regmap_config senoko_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = REG_WATCHDOG_SECONDS,
	.writeable_reg = senoko_regmap_is_writeable,
	.volatile_reg = senoko_regmap_is_volatile,
};

static struct resource senoko_gpio_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_gpio_cell = {
	.name		= "senoko-gpio",
	.of_compatible	= "kosagi,senoko-gpio",
	.resources	= senoko_gpio_resources,
	.num_resources	= ARRAY_SIZE(senoko_gpio_resources),
};

static struct resource senoko_keypad_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_keypad_cell = {
	.name		= "senoko-keypad",
	.of_compatible	= "kosagi,senoko-keypad",
	.resources	= senoko_keypad_resources,
	.num_resources	= ARRAY_SIZE(senoko_keypad_resources),
};

static struct resource senoko_senoko_power_supply_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_senoko_power_supply_cell = {
	.name		= "senoko-power-supply",
	.of_compatible	= "kosagi,senoko-power-supply",
	.resources	= senoko_senoko_power_supply_resources,
	.num_resources	= ARRAY_SIZE(senoko_senoko_power_supply_resources),
};

static struct resource senoko_rtc_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_rtc_cell = {
	.name		= "senoko-rtc",
	.of_compatible	= "kosagi,senoko-rtc",
	.resources	= senoko_rtc_resources,
	.num_resources	= ARRAY_SIZE(senoko_rtc_resources),
};

int senoko_read(struct senoko *senoko, int offset)
{
	unsigned int value;
	int ret;

	if (senoko->timeout_backoff
			&& !time_after(jiffies, senoko->timeout_backoff))
		return -ETIMEDOUT;
	senoko->timeout_backoff = 0;

	ret = regmap_read(senoko->regmap, offset, &value);
	if (ret < 0) {
		senoko->timeout_backoff = jiffies + msecs_to_jiffies(3000);
		return ret;
	}

	return value;
}
EXPORT_SYMBOL(senoko_read);

int senoko_write(struct senoko *senoko, int offset, u8 value)
{
	int ret;

	if (senoko->timeout_backoff
			&& !time_after(jiffies, senoko->timeout_backoff))
		return -ETIMEDOUT;
	senoko->timeout_backoff = 0;

	ret = regmap_write(senoko->regmap, offset, value);
	if (ret < 0)
		senoko->timeout_backoff = jiffies + msecs_to_jiffies(3000);

	return ret;
}
EXPORT_SYMBOL(senoko_write);

static void senoko_supply_power_off(void)
{
	struct senoko *senoko = g_senoko;
	int err;

	dev_info(senoko->dev, "shutting down\n");

	err = senoko_write(senoko, REG_POWER,
			   REG_POWER_STATE_OFF | REG_POWER_KEY_WRITE);
	if (err) {
		dev_err(senoko->dev, "unable to power off: %d\n", err);
		return;
	}

	/* Board should be off now */
	while(1);
}

static irqreturn_t senoko_irq_handler(int irq_ignored, void *devid)
{
	struct senoko *senoko = devid;
	unsigned irq;
	int status;

	status = senoko_read(senoko, REG_IRQ_STATUS);
	if (status < 0) {
		/*
		 * This happens when we reflash Senoko, so ignore
		 * this condition.
		 */
		dev_dbg(senoko->dev,
			"Error when reading IRQ status: %d\n", status);
		return IRQ_HANDLED;
	}

	dev_dbg(senoko->dev,
		"IRQ status: %d\n", status);

	for (irq = 0; irq < senoko->num_irqs; irq++) {
		if (status & (1 << irq)) {
			int child_irq = irq_create_mapping(senoko->domain, irq);
			handle_nested_irq(child_irq);
			status &= ~(1 << irq);
		}
	}

	senoko_write(senoko, REG_IRQ_STATUS, status);

	return IRQ_HANDLED;
}

static void senoko_irq_lock(struct irq_data *data)
{
	struct senoko *senoko = irq_data_get_irq_chip_data(data);

	mutex_lock(&senoko->irq_lock);
}

static void senoko_irq_sync_unlock(struct irq_data *data)
{
	struct senoko *senoko = irq_data_get_irq_chip_data(data);

	u8 new = senoko->ier;
	u8 old = senoko->oldier;

	if (new != old) {
		senoko->oldier = new;
		senoko_write(senoko, REG_IRQ_ENABLE, new);
	}

	mutex_unlock(&senoko->irq_lock);
}

static void senoko_irq_mask(struct irq_data *data)
{
	struct senoko *senoko = irq_data_get_irq_chip_data(data);
	int offset = data->hwirq;
	int mask = 1 << (offset % 8);

	senoko->ier &= ~mask;
}

static void senoko_irq_unmask(struct irq_data *data)
{
	struct senoko *senoko = irq_data_get_irq_chip_data(data);
	int offset = data->hwirq;
	int mask = 1 << (offset % 8);

	senoko->ier |= mask;
}

int senoko_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct senoko *senoko = irq_data_get_irq_chip_data(data);
	int offset = data->hwirq;
	int mask = 1 << (offset % 8);

	if (on)
		senoko->iwr |= mask;
	else
		senoko->iwr &= ~mask;
	return 0;
}

static struct irq_chip senoko_irq_chip = {
	.name			= "senoko",
	.irq_bus_lock		= senoko_irq_lock,
	.irq_bus_sync_unlock	= senoko_irq_sync_unlock,
	.irq_mask		= senoko_irq_mask,
	.irq_set_wake		= senoko_irq_set_wake,
	.irq_unmask		= senoko_irq_unmask,
};

static int senoko_irq_map(struct irq_domain *d, unsigned int virq,
				irq_hw_number_t hwirq)
{
	struct senoko *senoko = d->host_data;
	struct irq_chip *chip = NULL;

	chip = &senoko_irq_chip;

	irq_set_chip_data(virq, senoko);
	irq_set_chip_and_handler(virq, chip, handle_edge_irq);
	irq_set_nested_thread(virq, 1);
#ifdef CONFIG_ARM
	set_irq_flags(virq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif

	return 0;
}

static void senoko_irq_unmap(struct irq_domain *d, unsigned int virq)
{
#ifdef CONFIG_ARM
	set_irq_flags(virq, 0);
#endif
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static struct irq_domain_ops senoko_irq_ops = {
	.map	= senoko_irq_map,
	.unmap	= senoko_irq_unmap,
	.xlate	= irq_domain_xlate_twocell,
};

static int senoko_irq_init(struct senoko *senoko, struct device_node *np)
{
	int base = 0;
	int num_irqs = senoko->num_irqs;

	senoko->domain = irq_domain_add_simple(np, num_irqs, base,
					       &senoko_irq_ops, senoko);
	if (!senoko->domain) {
		dev_err(senoko->dev, "Failed to create irqdomain\n");
		return -ENOSYS;
	}

	return 0;
}

static void senoko_resource_irq_update(const struct mfd_cell *cell, int irq)
{
	int j;
	for (j = 0; j < cell->num_resources; j++) {
		struct resource *res = (struct resource *) &cell->resources[j];

		/* Dynamically fill in a block's IRQ. */
		if (res->flags & IORESOURCE_IRQ)
			res->start = res->end = irq + j;
	}
}

static int senoko_devices_init(struct senoko *senoko)
{
	int ret = -EINVAL;
	int blocknum = 0;

	senoko_resource_irq_update(&senoko_gpio_cell, senoko_gpio_irq);
	ret = mfd_add_devices(senoko->dev, blocknum++,
			      &senoko_gpio_cell, 1, NULL, 0, senoko->domain);
	if (ret)
		return ret;

	senoko_resource_irq_update(&senoko_keypad_cell, senoko_keypad_irq);
	ret = mfd_add_devices(senoko->dev, blocknum++,
			      &senoko_keypad_cell, 1, NULL, 0, senoko->domain);
	if (ret)
		return ret;

	senoko_resource_irq_update(&senoko_senoko_power_supply_cell,
				   senoko_senoko_power_supply_irq);
	ret = mfd_add_devices(senoko->dev, blocknum++,
			      &senoko_senoko_power_supply_cell,
			      1, NULL, 0, senoko->domain);
	if (ret)
		return ret;

	senoko_resource_irq_update(&senoko_rtc_cell, senoko_rtc_irq);
	ret = mfd_add_devices(senoko->dev, blocknum++,
			      &senoko_rtc_cell, 1, NULL, 0, senoko->domain);
	if (ret)
		return ret;

	return ret;
}

static int senoko_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct senoko *senoko;
	struct device_node *np = client->dev.of_node;
	int ret;
	int signature, ver_major, ver_minor;

	senoko = devm_kzalloc(&client->dev, sizeof(*senoko), GFP_KERNEL);
	if (senoko == NULL)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, senoko);

	mutex_init(&senoko->irq_lock);

	senoko->client = client;
	senoko->num_irqs = __senoko_num_irqs;
	senoko->dev = &client->dev;

	senoko->regmap = devm_regmap_init_i2c(client, &senoko_regmap_config);
	if (IS_ERR(senoko->regmap)) {
		dev_err(senoko->dev, "Unable to allocate register map: %ld\n",
			PTR_ERR(senoko->regmap));
		return PTR_ERR(senoko->regmap);
	}

	senoko->features = senoko_read(senoko, REG_FEATURES);
	if (senoko->features < 0)
		return -ENODEV;

	signature = senoko_read(senoko, REG_SIGNATURE);
	if (signature < 0)
		return -ENODEV;

	ver_major = senoko_read(senoko, REG_VERSION_MAJOR);
	if (ver_major < 0)
		return -ENODEV;

	ver_minor = senoko_read(senoko, REG_VERSION_MINOR);
	if (ver_minor < 0)
		return -ENODEV;

	dev_info(senoko->dev, "Senoko '%c' version %d.%d (features: 0x%02x)\n",
		signature, ver_major, ver_minor, senoko->features);

	senoko->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0,
						   &senoko->irq_trigger);
	dev_info(senoko->dev, "GPIO IRQ: %d\n", senoko->irq_gpio);
	dev_info(senoko->dev, "GPIO IRQ trigger: %x\n", senoko->irq_trigger);
	ret = devm_gpio_request_one(&client->dev, senoko->irq_gpio,
				    GPIOF_DIR_IN, "senoko");
	if (ret) {
		dev_err(senoko->dev, "failed to request IRQ GPIO: %d\n", ret);
		goto err_free;
	}
	senoko->irq = gpio_to_irq(senoko->irq_gpio);

	senoko_irq_init(senoko, np);

	ret = devm_request_threaded_irq(senoko->dev, senoko->irq, NULL,
			senoko_irq_handler, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"senoko", senoko);
	if (ret) {
		dev_err(&client->dev, "unable to get irq: %d\n", ret);
		goto err_domain;
	}

	i2c_set_clientdata(client, senoko);

	ret = senoko_devices_init(senoko);
	if (ret)
		goto remove_devices;

	senoko->old_power_off = pm_power_off;
	g_senoko = senoko;
	pm_power_off = senoko_supply_power_off;

	return 0;

remove_devices:
	mfd_remove_devices(senoko->dev);

err_domain:
	irq_domain_remove(senoko->domain);

err_free:
	return ret;
}

static int senoko_remove(struct i2c_client *client)
{
	struct senoko *senoko = i2c_get_clientdata(client);

	pm_power_off = senoko->old_power_off;
	mfd_remove_devices(senoko->dev);
	irq_domain_remove(senoko->domain);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int senoko_suspend(struct device *dev)
{
	struct senoko *senoko = dev_get_drvdata(dev);

	if (senoko->iwr)
		enable_irq_wake(senoko->irq);
	senoko_write(senoko, REG_IRQ_ENABLE, senoko->iwr);

	return 0;
}

static int senoko_resume(struct device *dev)
{
	struct senoko *senoko = dev_get_drvdata(dev);

	if (senoko->iwr)
		disable_irq_wake(senoko->irq);
	senoko_write(senoko, REG_IRQ_ENABLE, senoko->ier);

	return 0;
}
#endif

static const struct of_device_id senoko_of_match[] = {
	{ .compatible = "kosagi,senoko", },
	{},
};
MODULE_DEVICE_TABLE(of, senoko_of_match);

static const struct i2c_device_id senoko_id[] = {
	{"senoko", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, senoko_id);

static SIMPLE_DEV_PM_OPS(senoko_pm_ops, senoko_suspend, senoko_resume);

static struct i2c_driver senoko_driver = {
	.driver = {
		.name = "senoko",
		.of_match_table = senoko_of_match,
		.pm = &senoko_pm_ops,
	},
	.probe = senoko_probe,
	.remove = senoko_remove,
	.id_table = senoko_id,
};

module_i2c_driver(senoko_driver);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Senoko MFD Driver");
MODULE_LICENSE("GPL");
