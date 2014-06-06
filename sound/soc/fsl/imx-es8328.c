/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk-provider.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "imx-audmux.h"

#define DAI_NAME_SIZE	32
#define IMX6Q_SYSCLK 0x00

struct imx_es8328_data {
	struct device *dev;
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct regulator *codec_regulator;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	struct clk *codec_clk_src;
	struct clk *codec_clk_sel;
	struct clk *codec_clk_post_div;
	struct clk *system_cko;
	unsigned int clk_freq_src;
	unsigned int clk_frequency;
	int power_gpio;
	int jack_gpio;
};

static struct snd_soc_jack_gpio headset_jack_gpios[] = {
	{
		.gpio = -1,
		.name = "headset-gpio",
		.report = SND_JACK_HEADSET,
		.invert = 0,
		.debounce_time = 200,
	},
};

static struct snd_soc_jack headset_jack;

static int imx_es8328_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_es8328_data *data = container_of(rtd->card,
					struct imx_es8328_data, card);
	struct device *dev = rtd->card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, IMX6Q_SYSCLK,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params to %d\n",
			data->clk_frequency);
		return ret;
	}

	/* Headphone jack detection */
	/*
	if (gpio_is_valid(data->jack_gpio)) {
		ret = snd_soc_jack_new(rtd->codec, "Headphone",
				       SND_JACK_HEADPHONE | SND_JACK_BTN_0,
				       &headset_jack);
		if (ret)
			return ret;

		headset_jack_gpios[0].gpio = data->jack_gpio;
		ret = snd_soc_jack_add_gpios(&headset_jack,
					     ARRAY_SIZE(headset_jack_gpios),
					     headset_jack_gpios);
	}
	*/

	return ret;
}

static const struct snd_soc_dapm_widget imx_es8328_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_REGULATOR_SUPPLY("audio-amp", 1, 0),
};

static int imx_set_frequency(struct imx_es8328_data *data, int freq) {
	int ret;

	ret = clk_set_parent(data->system_cko, data->codec_clk);
	if (ret) {
		dev_err(data->dev, "unable to set clk output");
		return ret;
	}

	ret = clk_set_parent(data->codec_clk_sel, data->codec_clk_post_div);
	if (ret) {
		dev_err(data->dev, "unable to set clk parent");
		return ret;
	}

	data->clk_freq_src = clk_round_rate(data->codec_clk_src, freq * 32);
	data->clk_frequency = clk_round_rate(data->codec_clk, freq);
	dev_dbg(data->dev, "clock source frequency: %d\n", data->clk_freq_src);
	dev_dbg(data->dev, "clock frequency: %d\n", data->clk_frequency);

	ret = clk_set_rate(data->codec_clk_src, data->clk_freq_src);
	if (ret) {
		dev_err(data->dev, "unable to set source clock rate\n");
		return ret;
	}

	ret = clk_set_rate(data->codec_clk, data->clk_frequency);
	if (ret) {
		dev_err(data->dev, "unable to set codec clock rate\n");
		return ret;
	}

	ret = clk_prepare_enable(data->codec_clk);
	if (ret) {
		dev_err(data->dev, "unable to prepare codec clk\n");
		return ret;
	}

	ret = clk_prepare_enable(data->codec_clk_src);
	if (ret) {
		dev_err(data->dev, "unable to prepare codec clk source\n");
		return ret;
	}
	return ret;
}

static int imx_es8328_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct imx_es8328_data *data;
	int int_port, ext_port;
	int ret;
	struct device *dev = &pdev->dev;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->dev = dev;

	data->jack_gpio = of_get_named_gpio(pdev->dev.of_node,
				"jack-gpio", 0);

	data->power_gpio = of_get_named_gpio(pdev->dev.of_node,
				"power-gpio", 0);
	if (gpio_is_valid(data->power_gpio))
		devm_gpio_request_one(&pdev->dev, data->power_gpio,
				    GPIOF_OUT_INIT_HIGH,
				    "audio codec power switch");

	/* Setup clocks */
	data->codec_clk = devm_clk_get(dev, "cko1");
	if (IS_ERR(data->codec_clk)) {
		dev_err(dev,
			"codec clock missing or invalid\n");
		goto fail;
	}

	data->codec_clk_sel = devm_clk_get(dev, "cko1_sel");
	if (IS_ERR(data->codec_clk_sel)) {
		dev_err(dev,
			"codec clock select missing or invalid\n");
		goto fail;
	}

	data->codec_clk_src = devm_clk_get(dev, "pll4_audio");
	if (IS_ERR(data->codec_clk_src)) {
		dev_err(dev,
			"codec clock source missing or invalid\n");
		goto fail;
	}

	data->codec_clk_post_div = devm_clk_get(dev, "pll4_post_div");
	if (IS_ERR(data->codec_clk_post_div)) {
		dev_err(dev,
			"codec clock post-div missing or invalid\n");
		goto fail;
	}

	data->system_cko = devm_clk_get(dev, "cko_sel");
	if (IS_ERR(data->system_cko)) {
		dev_err(dev,
			"system clock missing or invalid\n");
		goto fail;
	}

	ret = imx_set_frequency(data, 22579200);
	if (ret)
		goto fail;

	data->codec_regulator = devm_regulator_get(dev, "codec");
	if (IS_ERR(data->codec_regulator)) {
		dev_err(dev, "No codec regulator\n");
		data->codec_regulator = NULL;
	}
	else {
		ret = regulator_enable(data->codec_regulator);
		if (ret)
			dev_err(dev,
				"Unable to enable codec regulator: %d\n", ret);
	}

	data->dai.name = "hifi";
	data->dai.stream_name = "hifi";
	data->dai.codec_dai_name = "es8328-hifi-analog";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_of_node = ssi_np;
	data->dai.platform_of_node = ssi_np;
	data->dai.init = &imx_es8328_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	data->card.dapm_widgets = imx_es8328_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_es8328_dapm_widgets);
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;

	ret = snd_soc_register_card(&data->card);
	if (ret)
		goto fail;

	platform_set_drvdata(pdev, data);
fail:
	if (ssi_np)
		of_node_put(ssi_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_es8328_remove(struct platform_device *pdev)
{
	struct imx_es8328_data *data = platform_get_drvdata(pdev);

//	snd_soc_jack_free_gpios(&headset_jack, ARRAY_SIZE(headset_jack_gpios),
//				headset_jack_gpios);
	if (data->codec_regulator) {
		int ret;
		ret = regulator_disable(data->codec_regulator);
		if (ret)
			dev_err(&pdev->dev,
				"Unable to disable codec regulator: %d\n", ret);
	}

	if (data->codec_clk)
		clk_disable_unprepare(data->codec_clk);

	if (data->codec_clk_src)
		clk_disable_unprepare(data->codec_clk_src);

	snd_soc_unregister_card(&data->card);

	return 0;
}

static const struct of_device_id imx_es8328_dt_ids[] = {
	{ .compatible = "kosagi,imx-audio-es8328", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_es8328_dt_ids);

static struct platform_driver imx_es8328_driver = {
	.driver = {
		.name = "imx-es8328",
		.owner = THIS_MODULE,
		.of_match_table = imx_es8328_dt_ids,
	},
	.probe = imx_es8328_probe,
	.remove = imx_es8328_remove,
};
module_platform_driver(imx_es8328_driver);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Kosagi i.MX6 ES8328 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-audio-es8328");
