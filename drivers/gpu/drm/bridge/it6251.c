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
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <drm/drm_panel.h>

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"
#include "drm_atomic_helper.h"

struct it6251_bridge {
	struct drm_connector connector;
	struct drm_bridge bridge;
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *lvds_client;
	struct regulator *regulator;
	struct drm_panel *panel;
};

#define LVDS_ADDR 0x5e

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
#define IT6251_RPC_REQ					0x2b
#define IT6251_RPC_REQ_RPC_FIFOFULL			(1 << 6)
#define IT6251_RPC_REQ_RPC_FIFOEMPTY			(1 << 7)

#define IT6251_REG_PCLK_CNT_LOW				0x57
#define IT6251_REG_PCLK_CNT_HIGH			0x58

#define IT6251_REG_LVDS_PORT_ADDR			0xfd
#define IT6251_REG_LVDS_PORT_CTRL			0xfe
#define IT6251_REG_LVDS_PORT_CTRL_EN			(1 << 0)

#define INIT_RETRY_DELAY_START msecs_to_jiffies(350)
#define INIT_RETRY_DELAY_MAX msecs_to_jiffies(3000)
#define INIT_RETRY_DELAY_INC msecs_to_jiffies(50)
#define INIT_RETRY_MAX_TRIES 4

#define it6251_lvds_write(it6251, addr, val) \
	do { \
		int ret; \
		ret = _it6251_lvds_write(it6251, addr, val); \
		if (ret) \
			dev_err(&it6251->lvds_client->dev, "it6251.c:%s:%d error %d writing %d to %d\n", __func__, __LINE__, ret, val, addr); \
	} while(0)

#define it6251_write(it6251, addr, val) \
	do { \
		int ret; \
		ret = _it6251_write(it6251, addr, val); \
		if (ret) \
			dev_err(&it6251->lvds_client->dev, "it6251.c:%s:%d error %d writing %d to %d\n", __func__, __LINE__, ret, val, addr); \
	} while(0)

static inline struct it6251_bridge *
		bridge_to_it6251(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6251_bridge, bridge);
}

static inline struct it6251_bridge *
		connector_to_it6251(struct drm_connector *connector)
{
	return container_of(connector, struct it6251_bridge, connector);
}

/* HW access functions */

static int
_it6251_write(struct it6251_bridge *it6251, uint8_t addr, uint8_t val)
{
	struct i2c_client *client = it6251->client;
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
_it6251_lvds_write(struct it6251_bridge *it6251, uint8_t addr, uint8_t val)
{
	struct i2c_client *client = it6251->lvds_client;
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
it6251_read(struct it6251_bridge *it6251, uint8_t addr)
{
	struct i2c_client *client = it6251->client;
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
it6251_lvds_read(struct it6251_bridge *it6251, uint8_t addr)
{
	struct i2c_client *client = it6251->lvds_client;
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

static int it6251_is_stable(struct it6251_bridge *it6251)
{
	int status;
	int rpclkcnt;
	int clkcnt;
	int refstate;
	int rpcreq;
	u16 hactive;
	u16 vactive;

	status = it6251_read(it6251, IT6251_SYSTEM_STATUS);
	dev_info(it6251->dev, "System status: 0x%02x\n", status);

	if (!(status & IT6251_SYSTEM_STATUS_RVIDEOSTABLE))
		return 0;

	rpclkcnt = ((it6251_read(it6251, 0x13) & 0xff)
		| ((it6251_read(it6251, 0x14) << 8) & 0x0f00));
	dev_info(it6251->dev, "RPCLKCnt: %d\n", rpclkcnt);

	clkcnt = ((it6251_lvds_read(it6251, IT6251_REG_PCLK_CNT_LOW) & 0xff) |
		 ((it6251_lvds_read(it6251, IT6251_REG_PCLK_CNT_HIGH) << 8) & 0x0f00));
	dev_info(it6251->dev, "Clock: 0x%02x\n", clkcnt);

	refstate = it6251_lvds_read(it6251, IT6251_REF_STATE);
	dev_info(it6251->dev, "Ref Link State: 0x%02x\n", refstate);

	rpcreq = it6251_lvds_read(it6251, IT6251_RPC_REQ);
	dev_info(it6251->dev, "RPC Req: 0x%02x\n", rpcreq);

	hactive = it6251_read(it6251, 0xa5);
	hactive = ((it6251_read(it6251, 0xa6) & 0x1f) << 8) + hactive;
	dev_info(it6251->dev, "hactive: %d\n", hactive);

	vactive = it6251_read(it6251, 0xaf);
	vactive = ((it6251_read(it6251, 0xb0) & 0x0f) << 8) + vactive;
	dev_info(it6251->dev, "vactive: %d\n", vactive);

	if ((refstate & 0x1f) != 0)
		return 0;

	if (rpcreq & IT6251_RPC_REQ_RPC_FIFOFULL) {
		dev_err(it6251->dev, "RPC fifofull is set, might be an error\n");
		return 0;
	}

	/* If video is muted, that's a failure */
	if (refstate & IT6251_REF_STATE_MUTED)
		return 0;

	if (it6251->panel) {
		struct drm_panel *panel = it6251->panel;

		if (panel->connector) {
			struct drm_display_mode *mode;

			list_for_each_entry(mode, &panel->connector->modes, head) {
				if ((mode->hdisplay == hactive)
				 && (mode->vdisplay == vactive))
					return 1;
			}
		}
		else
			dev_info(it6251->dev, "no panel connector\n");
	}
	else
		dev_info(it6251->dev, "no panel\n");

	dev_info(it6251->dev, "no match found\n");

	if (vactive == 1080)
		return 1;

	return 0;
}

static int it6251_init(struct it6251_bridge *it6251)
{
	int reg;
	int stable_delays;

	/* Reset DisplayPort half (setting bit 2 causes it to not respond
	 * over i2c, which is considered "normal".  This write will report
	 * failure, but will actually succeed. */
	it6251_write(it6251, 0x05, 0xff);
	it6251_write(it6251, 0x05, 0x00);

	/* Configure LVDS receiver */
	it6251_write(it6251, IT6251_REG_LVDS_PORT_ADDR,
			LVDS_ADDR << 1);
	it6251_write(it6251, IT6251_REG_LVDS_PORT_CTRL,
			IT6251_REG_LVDS_PORT_CTRL_EN);

	// LVDSRX
	it6251_lvds_write(it6251, 0x05, 0xff);   // reset LVDSRX
	it6251_lvds_write(it6251, 0x05, 0x00);

	it6251_lvds_write(it6251, 0x3b, 0x42);  // reset LVDSRX PLL
	it6251_lvds_write(it6251, 0x3b, 0x43);

	it6251_lvds_write(it6251, 0x3c, 0x08);  // something with SSC PLL
	it6251_lvds_write(it6251, 0x0b, 0x88);  // don't swap links, but writing reserved registers

	it6251_lvds_write(it6251, 0x2c, 0x01);  // JEIDA, 8-bit depth  0x11   // orig 0x42
	it6251_lvds_write(it6251, 0x32, 0x04);  // "reserved"
	it6251_lvds_write(it6251, 0x35, 0xe0);  // "reserved"
	it6251_lvds_write(it6251, 0x2b, 0x24);  // "reserved" + clock delay

	it6251_lvds_write(it6251, 0x05, 0x02);  // reset LVDSRX pix clock
	it6251_lvds_write(it6251, 0x05, 0x00);

	// DPTX
	it6251_write(it6251, 0x16, 0x02); // set for two lane mode, normal op, no swapping, no downspread

	it6251_write(it6251, 0x23, 0x40); // some AUX channel EDID magic

	it6251_write(it6251, 0x5c, 0xf3); // power down lanes 3-0

	it6251_write(it6251, 0x5f, 0x06); // enable DP scrambling, change EQ CR phase

	it6251_write(it6251, 0x60, 0x02); // color mode RGB, pclk/2
	it6251_write(it6251, 0x61, 0x04); // dual pixel input mode, no EO swap, no RGB swap
	it6251_write(it6251, 0x62, 0x01); // M444B24 video format

	// vesa range / not interlace / vsync high / hsync high
	//  00001111
	it6251_write(it6251, 0xa0, 0x0F);

	it6251_write(it6251, 0xc9, 0xf5); // hpd event timer set to 1.6-ish ms

	it6251_write(it6251, 0xca, 0x4d); // more reserved magic
	it6251_write(it6251, 0xcb, 0x37);

	it6251_write(it6251, 0xd3, 0x03); // enhanced framing mode, auto video fifo reset, video mute disable

	it6251_write(it6251, 0xd4, 0x45); // "vidstmp" and some reserved stuff

	it6251_write(it6251, 0xe7, 0xa0); // queue number -- reserved
	it6251_write(it6251, 0xe8, 0x33); // info frame packets  and reserved
	it6251_write(it6251, 0xec, 0x00); // more AVI stuff

	it6251_write(it6251, 0x23, 0x42); // select PC master reg for aux channel?

	it6251_write(it6251, 0x24, 0x00); // send PC request commands
	it6251_write(it6251, 0x25, 0x00);
	it6251_write(it6251, 0x26, 0x00);

	it6251_write(it6251, 0x2b, 0x00); // native aux read
	it6251_write(it6251, 0x23, 0x40); // back to internal

	it6251_write(it6251, 0x19, 0xff); // voltage swing level 3
	it6251_write(it6251, 0x1a, 0xff); // pre-emphasis level 3

	it6251_write(it6251, 0x17, 0x01); // start link training

	for (stable_delays = 0; stable_delays < 100; stable_delays++) {
		reg = it6251_read(it6251, 0x0e);
		if ((reg & 0x1f) == 0x10) {
			reg = it6251_read(it6251, IT6251_SYSTEM_STATUS);
			if (reg & IT6251_SYSTEM_STATUS_RVIDEOSTABLE)
				break;
		}
		udelay(2000);
	}

	/*
	 * If we couldn't stabilize, requeue and try again, because it means
	 * that the LVDS channel isn't stable yet.
	 */
	if (!it6251_is_stable(it6251)) {
		dev_err(it6251->dev, "warning: bridge is not stable\n");
		return 1;
	}

	return 0;
}

static int it6251_power_up(struct it6251_bridge *it6251)
{
	struct i2c_client *client = it6251->client;
	int vendor_id_lo, vendor_id_hi, device_id_lo, device_id_hi;
	int ret;
	int i;
	int max_tries = 5;

	ret = regulator_enable(it6251->regulator);
	if (ret) {
		dev_err(&client->dev, "Unable to enable regulator\n");
		return ret;
	}

	/* Sometimes it seems like multiple tries are needed */
	for (i = 0; i < max_tries; i++) {
		vendor_id_lo = it6251_read(it6251, IT6251_VENDOR_ID_LOW);
		vendor_id_hi = it6251_read(it6251, IT6251_VENDOR_ID_HIGH);
		device_id_lo = it6251_read(it6251, IT6251_DEVICE_ID_LOW);
		device_id_hi = it6251_read(it6251, IT6251_DEVICE_ID_HIGH);

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

		ret = regulator_disable(it6251->regulator);
		if (ret)
			dev_err(&client->dev, "unable to disable regulator\n");

		return -EPROBE_DEFER;
	}

	return 0;
}

static int it6251_suspend(struct device *dev, bool do_power_down)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *it6251 = i2c_get_clientdata(client);

	if (do_power_down && regulator_is_enabled(it6251->regulator)) {
		int ret;

		ret = regulator_disable(it6251->regulator);
		if (ret) {
			dev_err(&client->dev, "unable to disable regulator\n");
			return ret;
		}
	}

	return 0;
}

static int it6251_resume(struct device *dev, bool do_power_up)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *it6251 = i2c_get_clientdata(client);

	it6251 = i2c_get_clientdata(client);

	if (do_power_up) {
		int ret;

		ret = it6251_power_up(it6251);
		if (ret)
			return ret;
	}

	return 0;
}

/* I2C driver functions */

static int it6251_pm_suspend(struct device *dev)
{
	return it6251_suspend(dev, true);
}

static int it6251_pm_resume(struct device *dev)
{
	return it6251_resume(dev, true);
}

static int it6251_pm_freeze(struct device *dev)
{
	return it6251_suspend(dev, false);
}

static int it6251_pm_thaw(struct device *dev)
{
	return it6251_resume(dev, false);
}

static void it6251_pre_enable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int ret;

	if (drm_panel_prepare(it6251->panel)) {
		DRM_ERROR("failed to prepare panel\n");
		return;
	}

	ret = it6251_power_up(it6251);
	if (ret)
		dev_err(it6251->dev, "unable to power up chip\n");
}

static void it6251_enable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int tries;

	if (drm_panel_enable(it6251->panel)) {
		DRM_ERROR("failed to enable panel\n");
		return;
	}

	for (tries = 0; tries < 5; tries++) {
		int ret;

		ret = it6251_init(it6251);
		if (!ret)
			break;

		/* If the resolution isn't correct, restart the chip */
		ret = regulator_disable(it6251->regulator);
		if (ret)
			dev_err(it6251->dev, "unable to disable regulator\n");

		ret = it6251_power_up(it6251);
		if (ret)
			dev_err(it6251->dev, "unable to power up\n");
	}
}

static void it6251_disable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	if (drm_panel_disable(it6251->panel)) {
		DRM_ERROR("failed to disable panel\n");
		return;
	}
}

static void it6251_post_disable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	if (drm_panel_unprepare(it6251->panel)) {
		DRM_ERROR("failed to unprepare panel\n");
		return;
	}
}

static int it6251_get_modes(struct drm_connector *connector)
{
        struct it6251_bridge *it6251;

        it6251 = connector_to_it6251(connector);

        return drm_panel_get_modes(it6251->panel);
}

static struct drm_encoder *it6251_best_encoder(struct drm_connector *connector)
{
        struct it6251_bridge *it6251;

        it6251 = connector_to_it6251(connector);

        return it6251->bridge.encoder;
}

static const struct drm_connector_helper_funcs it6251_connector_helper_funcs = {
        .get_modes = it6251_get_modes,
        .best_encoder = it6251_best_encoder,
};

static enum drm_connector_status it6251_detect(struct drm_connector *connector,
                                                                bool force)
{
        return connector_status_connected;
}

static void it6251_connector_destroy(struct drm_connector *connector)
{
        drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs it6251_connector_funcs = {
        .dpms = drm_atomic_helper_connector_dpms,
        .fill_modes = drm_helper_probe_single_connector_modes,
        .detect = it6251_detect,
        .destroy = it6251_connector_destroy,
        .reset = drm_atomic_helper_connector_reset,
        .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
        .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int it6251_attach(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	it6251->connector.polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_init(bridge->dev, &it6251->connector,
			&it6251_connector_funcs, DRM_MODE_CONNECTOR_eDP);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}
	drm_atomic_helper_connector_reset(&it6251->connector);
	drm_connector_helper_add(&it6251->connector,
				 &it6251_connector_helper_funcs);
	drm_mode_connector_attach_encoder(&it6251->connector, bridge->encoder);

	if (it6251->panel)
		drm_panel_attach(it6251->panel, &it6251->connector);
	else
		dev_err(it6251->dev, "no panel found for attach\n");

	drm_helper_hpd_irq_event(it6251->connector.dev);

	return 0;
}

static const struct drm_bridge_funcs it6251_bridge_funcs = {
        .pre_enable = it6251_pre_enable,
        .enable = it6251_enable,
        .disable = it6251_disable,
        .post_disable = it6251_post_disable,
        .attach = it6251_attach,
};

static int
it6251_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct it6251_bridge *it6251;
	struct device_node *endpoint, *panel_node;
	int ret;

	it6251 = devm_kzalloc(&client->dev, sizeof(*it6251), GFP_KERNEL);
	if (!it6251) {
		dev_err(&client->dev, "unable to allocate it6251 data\n");
		return -ENOMEM;
	}

	it6251->dev = dev;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (endpoint) {
		panel_node = of_graph_get_remote_port_parent(endpoint);
		if (panel_node) {
			it6251->panel = of_drm_find_panel(panel_node);
			of_node_put(panel_node);
			if (!it6251->panel) {
				dev_warn(&client->dev,
					 "Unable to find panel, deferring\n");
				return -EPROBE_DEFER;
			}
		}
	}

	it6251->regulator = devm_regulator_get(&client->dev, "power");
	if (IS_ERR(it6251->regulator)) {
		dev_err(&client->dev, "Unable to get regulator\n");
		ret = PTR_ERR(it6251->regulator);
		it6251->regulator = NULL;
		goto err;
	}

	it6251->client = client;
	i2c_set_clientdata(client, it6251);

	it6251->bridge.funcs = &it6251_bridge_funcs;
	it6251->bridge.of_node = dev->of_node;
	ret = drm_bridge_add(&it6251->bridge);
	if (ret) {
		DRM_ERROR("Failed to add bridge\n");
		return ret;
	}

	/* The LVDS-half of the chip shows up at address 0x5e */
	it6251->lvds_client = i2c_new_dummy(it6251->client->adapter, LVDS_ADDR);
	if (!it6251->lvds_client) {
		ret = -ENODEV;
		goto err;
	}

	dev_err(&client->dev, "Succeeded in probing it6251\n");
	return 0;

err:
	if (it6251->lvds_client)
		i2c_unregister_device(it6251->lvds_client);

	dev_err(&client->dev, "Returning error: %d\n", ret);
	return ret;
}

static int it6251_remove(struct i2c_client *client)
{
	struct it6251_bridge *it6251 = i2c_get_clientdata(client);
	int ret;

	if (it6251->lvds_client)
		i2c_unregister_device(it6251->lvds_client);

	ret = regulator_disable(it6251->regulator);
	if (ret) {
		dev_err(&client->dev, "unable to enable regulator\n");
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops it6251_dev_pm_ops = {
	.suspend = it6251_pm_suspend,
	.resume = it6251_pm_resume,
	.freeze = it6251_pm_freeze,
	.thaw = it6251_pm_thaw,
	.poweroff = it6251_pm_freeze,
	.restore = it6251_pm_resume,
};

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
		.name		= "it6251",
		.owner		= THIS_MODULE,
		.pm 		= &it6251_dev_pm_ops,
		.of_match_table	= it6251_of_match,
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
