/*
 * Copyright (C) 2013 Sean Cross
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
#include <linux/slab.h>

#include <linux/byteorder/generic.h>

struct stdp4028_priv {
	struct i2c_client *client;
	int power_switch;
	uint16_t saved_state[0x10];
};

#define STDP4028_I2C_ADDR_SLAVE			0x73

/* HW register definitions */

#define STDP4028_PRODUCT_ID				0x0
# define STDP4028_PRODUCT_ID_SILICON_VERSION_MASK		0x00ff
# define STDP4028_PRODUCT_ID_CHIP_ID_MASK			0xff00
#define STDP4028_SYSTEM_CONTROL				0x1
# define STDP4028_SYSTEM_CONTROL_RESET				0x0001
# define STDP4028_SYSTEM_CONTROL_POWER_UP			0x0002
# define STDP4028_SYSTEM_CONTROL_DP_CABLE_DETECT		0x0004
# define STDP4028_SYSTEM_CONTROL_DP_INTR_OUT_POL_SET		0x0008
# define STDP4028_SYSTEM_CONTROL_AUDIO_PORT_SELECT_MASK		0x0030
# define STDP4028_SYSTEM_CONTROL_SEND_USER_DATA			0x0040
# define STDP4028_SYSTEM_CONTROL_ENABLE_UART_OVER_AUX		0x0080
# define STDP4028_SYSTEM_CONTROL_IR_DATA_ENABLE_MASK		0x0300
# define STDP4028_SYSTEM_CONTROL_AUXILIARY_DATA_TXFER_EN	0x0400
#define STDP4028_INTERRUPT_OUT_CONFIG			0x2
# define STDP4028_INTERRUPT_OUT_CONFIG_DPTX_IRQ_EN		0x0010
# define STDP4028_INTERRUPT_OUT_CONFIG_VIDEO_PROC_IRQ_EN	0x0040
# define STDP4028_INTERRUPT_OUT_CONFIG_DATA_PROC_IRQ_EN		0x0080
# define STDP4028_INTERRUPT_OUT_CONFIG_IR_DATA_AVAIL_IRQ_EN	0x0100
# define STDP4028_INTERRUPT_OUT_CONFIG_USER_DATA_AVAIL_IRQ_EN	0x0200
#define STDP4028_INTERRUPT_STATUS		0x3
# define STDP4028_INTERRUPT_OUT_CONFIG_DPTX_IRQ_STS		0x0010
# define STDP4028_INTERRUPT_OUT_CONFIG_VIDEO_PROC_IRQ_STS	0x0040
# define STDP4028_INTERRUPT_OUT_CONFIG_DATA_PROC_IRQ_STS	0x0080
# define STDP4028_INTERRUPT_OUT_CONFIG_IR_DATA_AVAIL_IRQ_STS	0x0100
# define STDP4028_INTERRUPT_OUT_CONFIG_USER_DATA_AVAIL_IRQ_STS	0x0200
#define STDP4028_FW_VERSION	0x4
# define STDP4028_FW_VERSION_SUB_REVISION_MASK	0x00ff
# define STDP4028_FW_VERSION_MINOR_REVISION_MASK	0x3f00
# define STDP4028_FW_VERSION_MAJOR_REVISION_MASK	0xc000
#define STDP4028_UNIQUE_DEVICE_IDENTIFIER_0	0x5
#define STDP4028_UNIQUE_DEVICE_IDENTIFIER_1	0x6
#define STDP4028_UNIQUE_DEVICE_IDENTIFIER_2	0x7
#define STDP4028_I2C_ADDRESS_CONTROL		0x8
#define STDP4028_DIP_BUS_FORMAT			0x9
#define STDP4028_TTL_RX_CLK_DELAY_CTRL		0xa
#define STDP4028_LVDS_DATA_FORMAT		0xb
# define STDP4028_LVDS_DATA_FORMAT_SELECT_MASK			0x0007
# define STDP4028_LVDS_DATA_FORMAT_SELECT_THINE_MODE1		0x0000
# define STDP4028_LVDS_DATA_FORMAT_SELECT_THINE_MODE2		0x0001
# define STDP4028_LVDS_DATA_FORMAT_SELECT_THINE_MODE3		0x0002
# define STDP4028_LVDS_DATA_FORMAT_SELECT_NATIONAL		0x0003
# define STDP4028_LVDS_DATA_FORMAT_SELECT_JEIDA			0x0004
# define STDP4028_LVDS_DATA_FORMAT_SELECT_NON_JEIDA		0x0005
# define STDP4028_LVDS_DATA_FORMAT_SELECT_VESA			0x0006
# define STDP4028_LVDS_DATA_FORMAT_BUS_WIDTH_MASK		0x0030
# define STDP4028_LVDS_DATA_FORMAT_BUS_WIDTH_10_BITS		0x0000
# define STDP4028_LVDS_DATA_FORMAT_BUS_WIDTH_8_BITS		0x0010
# define STDP4028_LVDS_DATA_FORMAT_BUS_WIDTH_6_BITS		0x0020
#define STDP4028_LVDS_DIG_CTRL_0		0xc
# define STDP4028_LVDS_DIG_CTRL_0_BUS_SEL_MASK			0x0003
# define STDP4028_LVDS_DIG_CTRL_0_BUS_SEL_SINGLE		0x0000
# define STDP4028_LVDS_DIG_CTRL_0_BUS_SEL_DUAL			0x0001
# define STDP4028_LVDS_DIG_CTRL_0_BUS_SEL_QUAD			0x0002
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_MASK		0x003c
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_SINGLE_E0	0x0000
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_SINGLE_O0	0x0004
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_SINGLE_E1	0x0008
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_SINGLE_O1	0x000c
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_E0E1	0x0000
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_E1E0	0x0004
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_E0O0	0x0008
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_O0E0	0x000c
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_O0O1	0x0010
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_O1O0	0x0014
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_E0E1O0O1	0x0000
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_E0E1O1O0	0x0004
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_E1E0O0O1	0x0008
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_E1E0O1O0	0x000c
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_O0O1E0E1	0x0010
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_O0O1E1E0	0x0014
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_O1O0E0E1	0x0018
# define STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_QUAD_O1O0E1E0	0x001c
# define STDP4028_LVDS_DIG_CTRL_0_LSB_MSB_SWAP			0x0040
# define STDP4028_LVDS_DIG_CTRL_0_DEN_INV			0x0080
# define STDP4028_LVDS_DIG_CTRL_0_DHS_INV			0x0100
# define STDP4028_LVDS_DIG_CTRL_0_DVS_INV			0x0200
# define STDP4028_LVDS_DIG_CTRL_0_DODD_INV			0x0400
# define STDP4028_LVDS_DIG_CTRL_0_LVDS_REVERSE_CHANNEL		0x0800
# define STDP4028_LVDS_DIG_CTRL_0_LVDS_CLK_COARSE_DEL_MASK	0xf000
#define STDP4028_LVDS_DIG_CTRL_1		0xd
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_RES_TRIM_MASK		0x001f
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_RES_TRIM(x)		((x)<<0)
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_PLL_SEL_MASK		0x006f
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_PLL_SEL_E0		0x0000
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_PLL_SEL_E1		0x0020
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_PLL_SEL_O0		0x0040
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_PLL_SEL_O1		0x0060
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_CLK7X_DLY_MASK		0x0180
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_CLK7X_DLY_CH_0		0x0080
# define STDP4028_LVDS_DIG_CTRL_1_LVDS_CLK7X_DLY_CH_1		0x0100
#define STDP4028_LVDS_DIG_CTRL_2		0xe
# define STDP4028_LVDS_DIG_CTRL_2_CLK_S2P_LOAD_DLY_MASK		0x007f
# define STDP4028_LVDS_DIG_CTRL_2_CLK_S2P_LOAD_DLY(x)	\
		((1<<x)&STDP4028_LVDS_DIG_CTRL_2_CLK_S2P_LOAD_DLY_MASK)
# define STDP4028_LVDS_DIG_CTRL_2_CLK1X_PHASE_MASK		0x3f80
# define STDP4028_LVDS_DIG_CTRL_2_CLK1X_PHASE(x)		((x)<<7)
# define STDP4028_LVDS_DIG_CTRL_2_CLKDBL_RANGE_MASK		0xc000
# define STDP4028_LVDS_DIG_CTRL_2_CLKDBL_RANGE_LE_10_MHZ	0x0000
# define STDP4028_LVDS_DIG_CTRL_2_CLKDBL_RANGE_GT_10_MHZ_L3_40_MHZ	0x4000
# define STDP4028_LVDS_DIG_CTRL_2_CLKDBL_RANGE_GT_40_MHZ_L3_100_MHZ	0x8000
#define STDP4028_LVDS_PLL_PHASE_ADJUST		0xf
# define STDP4028_LVDS_PLL_PHASE_ADJUST_AUTO_ADJUST_CRC_EN	0x0008
# define STDP4028_LVDS_PLL_PHASE_ADJUST_PLL_PH_COARSE_MASK	0x03f0
# define STDP4028_LVDS_PLL_PHASE_ADJUST_PLL_BW_TRACK		0x0400
# define STDP4028_LVDS_PLL_PHASE_ADJUST_PLL_BW_4M		0x0800

/* HW access functions */

static int
stdp4028_write(struct stdp4028_priv *priv, uint8_t addr, uint16_t val)
{
	struct i2c_client *client = priv->client;
	uint8_t buf[3] = {addr};
	int ret;

	val = cpu_to_be16(val);
	memcpy(buf+1, &val, sizeof(val));

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0) {
		dev_err(&client->dev, "Error %d writing to subaddress 0x%x\n",
			   ret, addr);
		return -1;
	}
	return 0;
}

static int
stdp4028_read(struct stdp4028_priv *priv, uint8_t addr)
{
	struct i2c_client *client = priv->client;
	uint16_t val;
	int ret;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, (void *)&val, sizeof(val));
	if (ret < 0)
		goto fail;

	return be16_to_cpu(val);

fail:
	dev_err(&client->dev, "Error %d reading from subaddress 0x%x\n",
		   ret, addr);
	return -1;
}

static void
stdp4028_save_state(struct stdp4028_priv *priv)
{
	int i;

	for (i = 0x8; i <= 0xf; i++)
		priv->saved_state[i] = stdp4028_read(priv, i);
}

static void
stdp4028_restore_state(struct stdp4028_priv *priv)
{
	int i;

	for (i = 0x8; i <= 0xf; i++)
		stdp4028_write(priv, i, priv->saved_state[i]);
}

static void
stdp4028_set_power_state(struct stdp4028_priv *priv, bool on)
{
	uint16_t control = stdp4028_read(priv, STDP4028_SYSTEM_CONTROL);

	/* 0 means powered up, 1 means standby */
	if (on)
		control &= ~STDP4028_SYSTEM_CONTROL_POWER_UP;
	else
		control |= STDP4028_SYSTEM_CONTROL_POWER_UP;

	stdp4028_write(priv, STDP4028_SYSTEM_CONTROL, control);
}

static int clk_phase = 0x63;
module_param_named(phase, clk_phase, int, 0444);

static int res_trim = 0x18;
module_param_named(trim, res_trim, int, 0444);

static int phase_adj = 0x0fb3;
module_param_named(phase_adj, phase_adj, int, 0444);

static void
stdp4028_init(struct stdp4028_priv *priv)
{
	stdp4028_write(priv, STDP4028_LVDS_DATA_FORMAT,
		STDP4028_LVDS_DATA_FORMAT_SELECT_JEIDA |
		STDP4028_LVDS_DATA_FORMAT_BUS_WIDTH_8_BITS);

	stdp4028_write(priv, STDP4028_LVDS_DIG_CTRL_0,
		STDP4028_LVDS_DIG_CTRL_0_BUS_SEL_DUAL |
		STDP4028_LVDS_DIG_CTRL_0_INPUT_CONFIG_DUAL_O0E0);

	stdp4028_write(priv, STDP4028_LVDS_DIG_CTRL_1,
		STDP4028_LVDS_DIG_CTRL_1_LVDS_RES_TRIM(res_trim));

	stdp4028_write(priv, STDP4028_LVDS_DIG_CTRL_2,
		STDP4028_LVDS_DIG_CTRL_2_CLK_S2P_LOAD_DLY(6) |
		STDP4028_LVDS_DIG_CTRL_2_CLK1X_PHASE(clk_phase) |
		STDP4028_LVDS_DIG_CTRL_2_CLKDBL_RANGE_GT_40_MHZ_L3_100_MHZ);

	/* This number removes *most* shimmer from the red channel */
	stdp4028_write(priv, STDP4028_LVDS_PLL_PHASE_ADJUST, phase_adj);
}



/* I2C driver functions */

static int
stdp4028_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stdp4028_priv *priv;
	struct device_node *np = client->dev.of_node;
	int device, si_version, major, minor, rev;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate private data\n");
		return -ENOMEM;
	}

	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->power_switch = of_get_named_gpio(np, "power-switch", 0);
	if (gpio_is_valid(priv->power_switch)) {
		ret = devm_gpio_request_one(&client->dev,
					priv->power_switch,
					GPIOF_OUT_INIT_HIGH,
					"STDP4028 power switch");
		if (ret) {
			dev_err(&client->dev, "unable to get power switch\n");
			goto err;
		}

		usleep_range(900*1000,1000*1000);
	}

	device = stdp4028_read(priv, STDP4028_PRODUCT_ID) >> 8;
	if (device == -1) {
		dev_err(&client->dev, "unable to read product id, deferring\n");
		ret = -EPROBE_DEFER;
		goto err;
	}

	si_version = stdp4028_read(priv, STDP4028_PRODUCT_ID) &
		STDP4028_PRODUCT_ID_SILICON_VERSION_MASK;
	major = (stdp4028_read(priv, STDP4028_FW_VERSION) >> 14) & 0x3;
	minor = (stdp4028_read(priv, STDP4028_FW_VERSION) >> 8) & 0x3f;
	rev = stdp4028_read(priv, STDP4028_FW_VERSION) & 0xff;

	dev_info(&client->dev, "detected device %x:%d.%d.%d v%d\n",
			device, major, minor, rev, si_version);
	stdp4028_init(priv);
	//stdp4028_set_power_state(priv, 1);

	return 0;

err:
	kfree(priv);
	return ret;
}

static int
stdp4028_remove(struct i2c_client *client)
{
	struct stdp4028_priv *priv;
	priv = i2c_get_clientdata(client);

	gpio_set_value(priv->power_switch, 0);
	
	kfree(priv);
	return 0;
}


static struct i2c_device_id stdp4028_ids[] = {
	{ "stdp4028", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stdp4028_ids);

static struct i2c_driver stdp4028_driver = {
	.driver = {
		.name = "stdp4028",
		.owner = THIS_MODULE,
	},
	.probe = stdp4028_probe,
	.remove = stdp4028_remove,
	.id_table = stdp4028_ids,
};

module_i2c_driver(stdp4028_driver);

/* Module initialization */
MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("STMicroelectronics STDP4028 DisplayPort encoder");
MODULE_LICENSE("GPL");
