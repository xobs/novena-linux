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

#define SIGNATURE 0
#define VERSION_MAJOR 1
#define VERSION_MINOR 1
#define IER 0x09
#define IRQ_STATUS 0x0a

/*
 * IRQ 0: GPIO
 * IRQ 1: Keypad
 * IRQ 2: Power supply
 * IRQ 3: Wake alarm
 */
enum senoko_irq {
	senoko_gpio_irq = 0,
	senoko_keypad_irq = 1,
	senoko_power_supply_irq = 2,
	senoko_rtc_irq = 3,
	__senoko_num_irqs = 4,
};

struct senoko_i2c_registers {
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

struct senoko {
	struct i2c_client *client;
	struct device *dev;
	struct irq_domain *domain;
	struct mutex lock;
	struct senoko_i2c_registers registers;

	/* protect serialized access to the interrupt controller bus */
	struct mutex irq_lock;

	int num_irqs;
	int irq_gpio;
	enum of_gpio_flags irq_trigger;
	int irq;

	int ier;
	int oldier;
};

static struct resource senoko_gpio_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_gpio_cell = {
	.name           = "senoko-gpio",
	.of_compatible  = "kosagi,senoko-gpio",
	.resources      = senoko_gpio_resources,
	.num_resources  = ARRAY_SIZE(senoko_gpio_resources),
};

static struct resource senoko_keypad_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_keypad_cell = {
	.name           = "senoko-keypad",
	.of_compatible  = "kosagi,senoko-keypad",
	.resources      = senoko_keypad_resources,
	.num_resources  = ARRAY_SIZE(senoko_keypad_resources),
};

static struct resource senoko_power_supply_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_power_supply_cell = {
	.name           = "senoko-power_supply",
	.of_compatible  = "kosagi,senoko-power_supply",
	.resources      = senoko_power_supply_resources,
	.num_resources  = ARRAY_SIZE(senoko_power_supply_resources),
};

static struct resource senoko_rtc_resources[] = {
	/* Start and end filled dynamically */
	{
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell senoko_rtc_cell = {
	.name           = "senoko-rtc",
	.of_compatible  = "kosagi,senoko-rtc",
	.resources      = senoko_rtc_resources,
	.num_resources  = ARRAY_SIZE(senoko_rtc_resources),
};

int senoko_read_all(struct senoko *senoko)
{
	return i2c_master_recv(senoko->client,
			(char *)&senoko->registers,
			sizeof(senoko->registers));
}
EXPORT_SYMBOL(senoko_read_all);

int senoko_read(struct senoko *senoko, int offset)
{
	int ret;

	ret = i2c_master_recv(senoko->client,
			(char *)&senoko->registers,
			sizeof(senoko->registers));
	if (ret < 0)
		return ret;
	return ((char *)&senoko->registers)[offset];
}
EXPORT_SYMBOL(senoko_read);

int senoko_write_all(struct senoko *senoko, int offset)
{
	u8 bfr[sizeof(senoko->registers) + 1];

	bfr[0] = 0;
	memcpy(bfr + 1, &senoko->registers, sizeof(senoko->registers));
	return i2c_master_send(senoko->client, bfr, sizeof(bfr));
}
EXPORT_SYMBOL(senoko_write_all);

int senoko_write(struct senoko *senoko, int offset, u8 value)
{
	u8 bfr[2];

	bfr[0] = offset;
	bfr[1] = value;

	((char *)&senoko->registers)[offset] = value;
	return i2c_master_send(senoko->client, bfr, sizeof(bfr));
}
EXPORT_SYMBOL(senoko_write);

static irqreturn_t senoko_irq_handler(int irq_ignored, void *devid)
{
	struct senoko *senoko = devid;
	unsigned irq;
	int status;

	status = senoko_read(senoko, IRQ_STATUS);
	if (status < 0)
		return IRQ_NONE;

	status &= senoko->ier;

	for (irq = 0; irq < senoko->num_irqs; irq++) {
		if (status & (1 << irq)) {
			int child_irq = irq_create_mapping(senoko->domain, irq);
			handle_nested_irq(child_irq);
			status &= ~(1 << irq);
		}
	}

	senoko_write(senoko, IRQ_STATUS, senoko->registers.irq_status);

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
		senoko_write(senoko, IER, new);
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

static struct irq_chip senoko_irq_chip = {
	.name			= "senoko",
	.irq_bus_lock		= senoko_irq_lock,
	.irq_bus_sync_unlock	= senoko_irq_sync_unlock,
	.irq_mask		= senoko_irq_mask,
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

	senoko_resource_irq_update(&senoko_power_supply_cell,
				   senoko_power_supply_irq);
	ret = mfd_add_devices(senoko->dev, blocknum++,
			      &senoko_power_supply_cell, 1, NULL, 0,
			      senoko->domain);
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

	senoko = devm_kzalloc(&client->dev, sizeof(*senoko), GFP_KERNEL);
	if (senoko == NULL)
		return -ENOMEM;

	mutex_init(&senoko->irq_lock);
	mutex_init(&senoko->lock);

	senoko->client = client;
	senoko->num_irqs = __senoko_num_irqs;
	senoko->dev = &client->dev;

	ret = senoko_read_all(senoko);
	if (ret < 0)
		goto err_free;

	dev_info(senoko->dev, "Senoko '%c' version %d.%d\n",
		senoko->registers.signature,
		senoko->registers.version_major,
		senoko->registers.version_minor);

	senoko->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0,
						&senoko->irq_trigger);
	ret = devm_gpio_request_one(&client->dev, senoko->irq_gpio,
					GPIOF_DIR_IN, "senoko");
	if (ret) {
		dev_err(senoko->dev, "failed to request IRQ GPIO: %d\n", ret);
		goto err_free;
	}
	senoko->irq = gpio_to_irq(senoko->irq_gpio);

	senoko_irq_init(senoko, np);

	ret = devm_request_threaded_irq(senoko->dev, senoko->irq, NULL,
			senoko_irq_handler, senoko->irq_trigger | IRQF_ONESHOT,
			"senoko", senoko);
	if (ret) {
		dev_err(&client->dev, "unable to get irq: %d\n", ret);
		goto err_free;
	}

	i2c_set_clientdata(client, senoko);

	ret = senoko_devices_init(senoko);
	if (ret)
		goto remove_devices;

	return 0;

remove_devices:
	mfd_remove_devices(senoko->dev);

err_free:
	return ret;
}

static int senoko_remove(struct i2c_client *client)
{
	struct senoko *senoko = i2c_get_clientdata(client);

	mfd_remove_devices(senoko->dev);
	return 0;
}

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

static struct i2c_driver senoko_driver = {
	.driver = {
		.name = "senoko",
		.of_match_table = senoko_of_match,
	},
	.probe = senoko_probe,
	.remove = senoko_remove,
	.id_table = senoko_id,
};

module_i2c_driver(senoko_driver);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Senoko GPIO Driver");
MODULE_LICENSE("GPL");
