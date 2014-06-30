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
#include <linux/workqueue.h>

struct it6251_bridge {
	struct i2c_client *client;
	struct i2c_client *lvds_client;
	struct regulator *regulator;
	struct delayed_work init_work;
	int delay_jiffies;
	int delay_tries;
};

/* HW register definitions */

#define IT6251_VENDOR_ID_LOW				0x00
#define IT6251_VENDOR_ID_HIGH				0x01
#define IT6251_DEVICE_ID_LOW				0x02
#define IT6251_DEVICE_ID_HIGH				0x03
#define IT6251_SYSTEM_STATUS				0x0d
#define IT6251_SYSTEM_STATUS_RINTSTATUS			(1 << 0)
#define IT6251_SYSTEM_STATUS_RHPDSTATUS			(1 << 1)
#define IT6251_SYSTEM_STATUS_RVIDEOSTABLE		(1 << 2)
#define IT6251_SYSTEM_STATUS_RPLL_IOLOCK		(1 << 3)
#define IT6251_SYSTEM_STATUS_RPLL_XPLOCK		(1 << 4)
#define IT6251_SYSTEM_STATUS_RPLL_SPLOCK		(1 << 5)
#define IT6251_SYSTEM_STATUS_RAUXFREQ_LOCK		(1 << 6)
#define IT6251_REF_STATE				0x0e
#define IT6251_REF_STATE_MAIN_LINK_DISABLED		(1 << 0)
#define IT6251_REF_STATE_AUX_CHANNEL_READ		(1 << 1)
#define IT6251_REF_STATE_CR_PATTERN			(1 << 2)
#define IT6251_REF_STATE_EQ_PATTERN			(1 << 3)
#define IT6251_REF_STATE_NORMAL_OPERATION		(1 << 4)
#define IT6251_REF_STATE_MUTED				(1 << 5)

#define IT6251_REG_PCLK_CNT_LOW				0x57
#define IT6251_REG_PCLK_CNT_HIGH			0x58

#define INIT_RETRY_DELAY_START msecs_to_jiffies(50)
#define INIT_RETRY_DELAY_MAX msecs_to_jiffies(3000)
#define INIT_RETRY_DELAY_INC msecs_to_jiffies(50)
#define INIT_RETRY_MAX_TRIES 20

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

static int
it6251_lvds_read(struct it6251_bridge *priv, uint8_t addr)
{
	struct i2c_client *client = priv->lvds_client;
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

static int it6251_is_stable(struct it6251_bridge *priv)
{
	int status;
	int clkcnt;
	int rpclkcnt;
	int refstate;

	rpclkcnt = ((it6251_read(priv, 0x13) & 0xff)
		| ((it6251_read(priv, 0x14) << 8) & 0x0f00));
	dev_info(&priv->client->dev, "RPCLKCnt: %d\n", rpclkcnt);

	status = it6251_read(priv, IT6251_SYSTEM_STATUS);
	dev_info(&priv->client->dev, "System status: 0x%02x\n", status);

	clkcnt = ((it6251_lvds_read(priv, IT6251_REG_PCLK_CNT_LOW) & 0xff) |
		 ((it6251_lvds_read(priv, IT6251_REG_PCLK_CNT_HIGH) << 8) & 0x0f00));
	dev_info(&priv->client->dev, "Clock: 0x%02x\n", clkcnt);

	refstate = it6251_lvds_read(priv, IT6251_REF_STATE);
	dev_info(&priv->client->dev, "Ref Link State: 0x%02x\n", refstate);

//	if (rpclkcnt != 2260)
//		return 0;

	if ((refstate & 0x1f) != 0)
		return 0;

	/* If video is muted, that's a failure */
	if (refstate & IT6251_REF_STATE_MUTED)
		return 0;

	if (!(status & IT6251_SYSTEM_STATUS_RVIDEOSTABLE))
		return 0;

	if (clkcnt != 0x193)
		return 0;

	return 1;
}

static void it6251_init(struct work_struct *work)
{
	int reg;
	int tries;
	struct it6251_bridge *priv = container_of(work, struct it6251_bridge,
						  init_work.work);

	it6251_write(priv, 0x05, 0x00);
	udelay(1000);

	it6251_write(priv, 0xfd, 0xbc); // set LVDSRX address, and enable
	it6251_write(priv, 0xfe, 0x01);

	// LVDSRX
	/* This write always fails, because the chip goes into reset */
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

	it6251_write(priv, 0x19, 0xff); // voltage swing level 3
	it6251_write(priv, 0x1a, 0xff); // pre-emphasis level 3

	it6251_write(priv, 0x17, 0x01); // start link training

	
	for (tries = 0; tries < 100; tries++) {
		reg = it6251_read(priv, 0x17);
		if (reg == 0xe0) {
			reg = it6251_read(priv, IT6251_SYSTEM_STATUS);
			if (reg & IT6251_SYSTEM_STATUS_RVIDEOSTABLE)
				break;
		}
		udelay(2000);
	}
	
	/*
	 * If we couldn't stabilize, requeue and try again, because it means
	 * that the LVDS channel isn't stable yet.
	 */
	if (!it6251_is_stable(priv)) {

		dev_err(&priv->client->dev,
			"Display didn't stabilize.  This may be because "
			"the LVDS port is still in powersave mode.");
		if (priv->delay_tries++ > INIT_RETRY_MAX_TRIES) {
			dev_err(&priv->client->dev,
				"Too many retries, abandoning.\n");
		}
		else {
			priv->delay_jiffies += INIT_RETRY_DELAY_INC;
			if (priv->delay_jiffies > INIT_RETRY_DELAY_MAX)
				priv->delay_jiffies = INIT_RETRY_DELAY_MAX;
			dev_err(&priv->client->dev,
				"Will try again in %d msecs\n",
				jiffies_to_msecs(priv->delay_jiffies));
			schedule_delayed_work(&priv->init_work,
					      priv->delay_jiffies);
		}
	}

	return;
}

static int it6251_power_up(struct i2c_client *client, struct it6251_bridge *priv)
{
	int vendor_id_lo, vendor_id_hi, device_id_lo, device_id_hi;
	int ret;
	int i;
	int max_tries = 5;

	ret = regulator_enable(priv->regulator);
	if (ret) {
		dev_err(&client->dev, "Unable to enable regulator\n");
		return ret;
	}

	/* Sometimes it seems like multiple tries are needed */
	for (i = 0; i < max_tries; i++) {
		vendor_id_lo = it6251_read(priv, IT6251_VENDOR_ID_LOW);
		vendor_id_hi = it6251_read(priv, IT6251_VENDOR_ID_HIGH);
		device_id_lo = it6251_read(priv, IT6251_DEVICE_ID_LOW);
		device_id_hi = it6251_read(priv, IT6251_DEVICE_ID_HIGH);

		if ((vendor_id_lo != -1)
		 && (vendor_id_hi != -1)
		 && (device_id_lo != -1)
		 && (device_id_hi != -1))
			break;
		usleep_range(100000, 200000);
	}

	if ((vendor_id_lo == -1) || (vendor_id_hi == -1)
	 || (device_id_lo == -1) || (device_id_hi == -1)) {

		dev_err(&client->dev, "unable to read product id, deferring\n");

		ret = regulator_disable(priv->regulator);
		if (ret)
			dev_err(&client->dev, "unable to disable regulator\n");

		return -EPROBE_DEFER;
	}

	return 0;
}

static int it6251_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *priv = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&priv->init_work);

	if (regulator_is_enabled(priv->regulator)) {
		int ret;

		ret = regulator_disable(priv->regulator);
		if (ret) {
			dev_err(&client->dev, "unable to disable regulator\n");
			return ret;
		}
	}

	return 0;
}

static int it6251_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *priv = i2c_get_clientdata(client);
	int ret;

	priv = i2c_get_clientdata(client);

	ret = it6251_power_up(client, priv);
	if (ret)
		return ret;

	priv->delay_jiffies = INIT_RETRY_DELAY_START;
	priv->delay_tries = 0;
	schedule_delayed_work(&priv->init_work, priv->delay_jiffies);
	return 0;
}


/* I2C driver functions */

static int
it6251_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct it6251_bridge *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate private data\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&priv->init_work, it6251_init);

	priv->regulator = devm_regulator_get(&client->dev, "power");
	if (IS_ERR(priv->regulator)) {
		dev_err(&client->dev, "Unable to get regulator\n");
		ret = PTR_ERR(priv->regulator);
		priv->regulator = NULL;
		goto err;
	}

	priv->client = client;
	i2c_set_clientdata(client, priv);

	ret = it6251_power_up(client, priv);
	if (ret) {
		dev_err(&client->dev, "unable to power up chip\n");
		goto err;
	}

	/* The LVDS-half of the chip shows up at address 0x5e */
	priv->lvds_client = i2c_new_dummy(priv->client->adapter, 0x5e);
	if (!priv->lvds_client) {
		ret = -ENODEV;
		goto err;
	}

	priv->delay_tries = 0;
	priv->delay_jiffies = INIT_RETRY_DELAY_START;
	schedule_delayed_work(&priv->init_work, priv->delay_jiffies);

	return 0;

err:
	if (priv->lvds_client)
		i2c_unregister_device(priv->lvds_client);

	dev_err(&client->dev, "Returning error: %d\n", ret);
	return ret;
}

static int it6251_remove(struct i2c_client *client)
{
	struct it6251_bridge *priv = i2c_get_clientdata(client);
	int ret;

	cancel_delayed_work_sync(&priv->init_work);

	if (priv->lvds_client)
		i2c_unregister_device(priv->lvds_client);

	ret = regulator_disable(priv->regulator);
	if (ret) {
		dev_err(&client->dev, "unable to enable regulator\n");
		return ret;
	}

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
