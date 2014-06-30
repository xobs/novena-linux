/*
 * Copyright (C) 2014 Sean Cross
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

struct it6251_bridge {
	struct i2c_client *client;
	struct i2c_client *lvds_client;
	struct regulator *regulator;
	uint16_t saved_state[0x10];
};

/* HW register definitions */

#define IT6251_VENDOR_ID_LOW				0x00
#define IT6251_VENDOR_ID_HIGH				0x01
#define IT6251_DEVICE_ID_LOW				0x02
#define IT6251_DEVICE_ID_HIGH				0x03

/* HW access functions */

static int
it6251_write(struct it6251_bridge *priv, uint8_t addr, uint8_t val)
{
	struct i2c_client *client = priv->client;
	uint8_t buf[2] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0) {
		dev_err(&client->dev, "error %d writing to edp addr 0x%x\n",
			   ret, addr);
		return -1;
	}
	return 0;
}

static int
it6251_lvds_write(struct it6251_bridge *priv, uint8_t addr, uint8_t val)
{
	struct i2c_client *client = priv->lvds_client;
	uint8_t buf[2] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0) {
		dev_err(&client->dev, "error %d writing to lvds addr 0x%x\n",
			   ret, addr);
		return -1;
	}
	return 0;
}

static int
it6251_read(struct it6251_bridge *priv, uint8_t addr)
{
	struct i2c_client *client = priv->client;
	uint8_t val;
	int ret;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, (void *)&val, sizeof(val));
	if (ret < 0)
		goto fail;

	return val;

fail:
	dev_err(&client->dev, "Error %d reading from subaddress 0x%x\n",
		   ret, addr);
	return -1;
}

static int it6251_init(struct it6251_bridge *priv)
{
	it6251_write(priv, 0x05, 0x00);

	it6251_write(priv, 0xfd, 0xbc); // set LVDSRX address, and enable
	it6251_write(priv, 0xfe, 0x01);

	// LVDSRX
	it6251_lvds_write(priv, 0x05, 0xff);   // reset LVDSRX
	it6251_lvds_write(priv, 0x05, 0x00);

	it6251_lvds_write(priv, 0x3b, 0x42);  // reset LVDSRX PLL
	it6251_lvds_write(priv, 0x3b, 0x43);

	it6251_lvds_write(priv, 0x3c, 0x08);  // something with SSC PLL
	it6251_lvds_write(priv, 0x0b, 0x88);  // don't swap links, but writing reserved registers

	it6251_lvds_write(priv, 0x2c, 0x01);  // JEIDA, 8-bit depth  0x11   // orig 0x42
	it6251_lvds_write(priv, 0x32, 0x04);  // "reserved"
	it6251_lvds_write(priv, 0x35, 0xe0);  // "reserved"
	it6251_lvds_write(priv, 0x2b, 0x24);  // "reserved" + clock delay

	it6251_lvds_write(priv, 0x05, 0x02);  // reset LVDSRX pix clock
	it6251_lvds_write(priv, 0x05, 0x00);

	// DPTX
	it6251_write(priv, 0x16, 0x02); // set for two lane mode, normal op, no swapping, no downspread

	it6251_write(priv, 0x23, 0x40); // some AUX channel EDID magic

	it6251_write(priv, 0x5c, 0xf3); // power down lanes 3-0

	it6251_write(priv, 0x5f, 0x06); // enable DP scrambling, change EQ CR phase

	it6251_write(priv, 0x60, 0x02); // color mode RGB, pclk/2
	it6251_write(priv, 0x61, 0x04); // dual pixel input mode, no EO swap, no RGB swap
	it6251_write(priv, 0x62, 0x01); // M444B24 video format

	// vesa range / not interlace / vsync high / hsync high
	//  00001111
	it6251_write(priv, 0xa0, 0x0F);

	it6251_write(priv, 0xc9, 0xf5); // hpd event timer set to 1.6-ish ms

	it6251_write(priv, 0xca, 0x4d); // more reserved magic
	it6251_write(priv, 0xcb, 0x37);

	it6251_write(priv, 0xd3, 0x03); // enhanced framing mode, auto video fifo reset, video mute disable

	it6251_write(priv, 0xd4, 0x45); // "vidstmp" and some reserved stuff

	it6251_write(priv, 0xe7, 0xa0); // queue number -- reserved
	it6251_write(priv, 0xe8, 0x33); // info frame packets  and reserved
	it6251_write(priv, 0xec, 0x00); // more AVI stuff

	it6251_write(priv, 0x23, 0x42); // select PC master reg for aux channel?

	it6251_write(priv, 0x24, 0x00); // send PC request commands
	it6251_write(priv, 0x25, 0x00);
	it6251_write(priv, 0x26, 0x00);

	it6251_write(priv, 0x2b, 0x00); // native aux read
	it6251_write(priv, 0x23, 0x40); // back to internal

	it6251_write(priv, 0x17, 0x01); // start link training

	return 0;
}


static int it6251_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *priv = i2c_get_clientdata(client);
	int ret;

	ret = regulator_disable(priv->regulator);
	if (ret) {
		dev_err(&client->dev, "unable to enable regulator\n");
		return ret;
	}

	return 0;
}

static int it6251_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *priv = i2c_get_clientdata(client);
	int vendor_id_lo, vendor_id_hi, device_id_lo, device_id_hi;
	int ret;

	priv = i2c_get_clientdata(client);

	ret = regulator_enable(priv->regulator);
	if (ret) {
		dev_err(&client->dev, "Unable to enable regulator\n");
		return ret;
	}

	vendor_id_lo = it6251_read(priv, IT6251_VENDOR_ID_LOW);
	vendor_id_hi = it6251_read(priv, IT6251_VENDOR_ID_HIGH);
	device_id_lo = it6251_read(priv, IT6251_DEVICE_ID_LOW);
	device_id_hi = it6251_read(priv, IT6251_DEVICE_ID_HIGH);

	if ((vendor_id_lo == -1) || (vendor_id_hi == -1)
		|| (device_id_lo == -1) || (device_id_hi == -1)) {
		dev_err(&client->dev, "unable to read product id, deferring\n");
		return -EPROBE_DEFER;
	}

	dev_info(&client->dev, "resuming IT6251 0x%02x%02x by 0x%02x%02x\n",
			device_id_lo, device_id_hi, vendor_id_lo, vendor_id_hi);

	it6251_init(priv);
	return 0;
}

/* I2C driver functions */

static int
it6251_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct it6251_bridge *priv;
	int vendor_id_lo, vendor_id_hi, device_id_lo, device_id_hi;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate private data\n");
		return -ENOMEM;
	}

	priv->regulator = devm_regulator_get(&client->dev, "power");
	if (IS_ERR(priv->regulator)) {
		dev_err(&client->dev, "Unable to get regulator\n");
		return 0;
	}

	ret = regulator_enable(priv->regulator);
	if (ret)
		dev_err(&client->dev, "Unable to enable regulator\n");

	priv->client = client;
	i2c_set_clientdata(client, priv);

	vendor_id_lo = it6251_read(priv, IT6251_VENDOR_ID_LOW);
	vendor_id_hi = it6251_read(priv, IT6251_VENDOR_ID_HIGH);
	device_id_lo = it6251_read(priv, IT6251_DEVICE_ID_LOW);
	device_id_hi = it6251_read(priv, IT6251_DEVICE_ID_HIGH);

	if ((vendor_id_lo == -1) || (vendor_id_hi == -1)
		|| (device_id_lo == -1) || (device_id_hi == -1)) {
		dev_err(&client->dev, "unable to read product id, deferring\n");
		ret = -EPROBE_DEFER;
		goto err;
	}

	/* The LVDS-half of the chip shows up at address 0x5e */
	priv->lvds_client = i2c_new_dummy(priv->client->adapter, 0x5e);
	if (!priv->lvds_client)
		return -1;

	dev_info(&client->dev, "detected IT6251 0x%02x%02x by 0x%02x%02x\n",
			device_id_lo, device_id_hi, vendor_id_lo, vendor_id_hi);
	ret = it6251_init(priv);
	if (ret) {
		dev_err(&client->dev, "Unable to initialize device");
		return ret;
	}


	return 0;

err:
	kfree(priv);
	return ret;
}

static int it6251_remove(struct i2c_client *client)
{
	struct it6251_bridge *priv = i2c_get_clientdata(client);
	int ret;

	if (priv->lvds_client)
		i2c_unregister_device(priv->lvds_client);

	ret = regulator_disable(priv->regulator);
	if (ret) {
		dev_err(&client->dev, "unable to enable regulator\n");
		return ret;
	}

	kfree(priv);
	return 0;
}

static SIMPLE_DEV_PM_OPS(it6251_dev_pm_ops,
			it6251_suspend, it6251_resume);

static struct i2c_device_id it6251_ids[] = {
	{ "it6251", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, it6251_ids);

static const struct of_device_id it6251_of_match[] = {
	{ .compatible = "it,it6251", },
	{ }
};
MODULE_DEVICE_TABLE(of, it6251_of_match);

static struct i2c_driver it6251_driver = {
	.driver = {
		.name	= "it6251",
		.owner	= THIS_MODULE,
		.pm 	= &it6251_dev_pm_ops,
		.of_match_table = it6251_of_match,
	},
	.probe		= it6251_probe,
	.remove		= it6251_remove,
	.id_table	= it6251_ids,
};

module_i2c_driver(it6251_driver);

/* Module initialization */
MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("ITE Tech 6251 LVDS to DisplayPort encoder");
MODULE_LICENSE("GPL");
