/*
 * es8328.c  --  WM8728 ALSA SoC Audio driver
 *
 * Copyright 2008 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define ES8328_DACLVOL 46
#define ES8328_DACRVOL 47
#define ES8328_DACCTL 28

#define ES8328_CONTROL1		0x00
#define ES8328_CONTROL2		0x01
#define ES8328_CHIPPOWER	0x02
#define ES8328_ADCPOWER		0x03
#define ES8328_DACPOWER		0x04
#define ES8328_CHIPLOPOW1	0x05
#define ES8328_CHIPLOPOW2	0x06
#define ES8328_ANAVOLMANAG	0x07
#define ES8328_MASTERMODE	0x08
#define ES8328_ADCCONTROL1	0x09
#define ES8328_ADCCONTROL2	0x0a
#define ES8328_ADCCONTROL3	0x0b
#define ES8328_ADCCONTROL4	0x0c
#define ES8328_ADCCONTROL5	0x0d
#define ES8328_ADCCONTROL6	0x0e
#define ES8328_ADCCONTROL7	0x0f
#define ES8328_ADCCONTROL8	0x10
#define ES8328_ADCCONTROL9	0x11
#define ES8328_ADCCONTROL10	0x12
#define ES8328_ADCCONTROL11	0x13
#define ES8328_ADCCONTROL12	0x14
#define ES8328_ADCCONTROL13	0x15
#define ES8328_ADCCONTROL14	0x16

#define ES8328_DACCONTROL1	0x17
#define ES8328_DACCONTROL2	0x18
#define ES8328_DACCONTROL3	0x19
#define ES8328_DACCONTROL4	0x1a
#define ES8328_DACCONTROL5	0x1b
#define ES8328_DACCONTROL6	0x1c
#define ES8328_DACCONTROL7	0x1d
#define ES8328_DACCONTROL8	0x1e
#define ES8328_DACCONTROL9	0x1f
#define ES8328_DACCONTROL10	0x20
#define ES8328_DACCONTROL11	0x21
#define ES8328_DACCONTROL12	0x22
#define ES8328_DACCONTROL13	0x23
#define ES8328_DACCONTROL14	0x24
#define ES8328_DACCONTROL15	0x25
#define ES8328_DACCONTROL16	0x26
#define ES8328_DACCONTROL17	0x27
#define ES8328_DACCONTROL18	0x28
#define ES8328_DACCONTROL19	0x29
#define ES8328_DACCONTROL20	0x2a
#define ES8328_DACCONTROL21	0x2b
#define ES8328_DACCONTROL22	0x2c
#define ES8328_DACCONTROL23	0x2d
#define ES8328_DACCONTROL24	0x2e
#define ES8328_DACCONTROL25	0x2f
#define ES8328_DACCONTROL26	0x30
#define ES8328_DACCONTROL27	0x31
#define ES8328_DACCONTROL28	0x32
#define ES8328_DACCONTROL29	0x33
#define ES8328_DACCONTROL30	0x34
#define ES8328_SYSCLK		0

#define ES8328_REG_MAX		0x35

#define ES8328_PLL1		0
#define ES8328_PLL2		1

/* clock inputs */
#define ES8328_MCLK		0
#define ES8328_PCMCLK		1

/* clock divider id's */
#define ES8328_PCMDIV		0
#define ES8328_BCLKDIV		1
#define ES8328_VXCLKDIV		2

/* PCM clock dividers */
#define ES8328_PCM_DIV_1	(0 << 6)
#define ES8328_PCM_DIV_3	(2 << 6)
#define ES8328_PCM_DIV_5_5	(3 << 6)
#define ES8328_PCM_DIV_2	(4 << 6)
#define ES8328_PCM_DIV_4	(5 << 6)
#define ES8328_PCM_DIV_6	(6 << 6)
#define ES8328_PCM_DIV_8	(7 << 6)

/* BCLK clock dividers */
#define ES8328_BCLK_DIV_1	(0 << 7)
#define ES8328_BCLK_DIV_2	(1 << 7)
#define ES8328_BCLK_DIV_4	(2 << 7)
#define ES8328_BCLK_DIV_8	(3 << 7)

/* VXCLK clock dividers */
#define ES8328_VXCLK_DIV_1	(0 << 6)
#define ES8328_VXCLK_DIV_2	(1 << 6)
#define ES8328_VXCLK_DIV_4	(2 << 6)
#define ES8328_VXCLK_DIV_8	(3 << 6)
#define ES8328_VXCLK_DIV_16	(4 << 6)

#define ES8328_DAI_HIFI		0
#define ES8328_DAI_VOICE	1

#define ES8328_1536FS		1536
#define ES8328_1024FS		1024
#define ES8328_768FS		768
#define ES8328_512FS		512
#define ES8328_384FS		384
#define ES8328_256FS		256
#define ES8328_128FS		128


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

/* We run at 22.5792 MHz */
enum es8328_rate {
	ES8328_RATE_8019,
	ES8328_RATE_11025,
	ES8328_RATE_22050,
	ES8328_RATE_44100,
};

uint8_t sample_ratios[] = {
	[ES8328_RATE_8019 ] = 0x9,
	[ES8328_RATE_11025] = 0x7,
	[ES8328_RATE_22050] = 0x4,
	[ES8328_RATE_44100] = 0x2,
};

/* codec private data */
struct es8328_priv {
	struct regmap *regmap;
	int sysclk;
};

static const DECLARE_TLV_DB_SCALE(play_tlv, -3000, 100, 0);
static const DECLARE_TLV_DB_SCALE(cap_tlv, -9600, 50, 0);
static const DECLARE_TLV_DB_SCALE(pga_tlv, 0, 300, 0);

static const struct snd_kcontrol_new es8328_snd_controls[] = {
SOC_DOUBLE_R_TLV("Speaker Playback Volume",
		ES8328_DACCONTROL26, ES8328_DACCONTROL27, 0, 0x24, 0, play_tlv),
SOC_DOUBLE_R_TLV("Headphone Playback Volume",
		ES8328_DACCONTROL24, ES8328_DACCONTROL25, 0, 0x24, 0, play_tlv),

SOC_DOUBLE_R_TLV("Mic Capture Volume",
		ES8328_ADCCONTROL8, ES8328_ADCCONTROL9, 0, 0xc0, 1, cap_tlv),
SOC_DOUBLE_TLV("Mic PGA Volume",
		ES8328_ADCCONTROL1, 4, 0, 0x08, 0, pga_tlv),
};

/*
 * DAPM controls.
 */
static const struct snd_soc_dapm_widget es8328_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("Speaker Volume", "HiFi Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("VOUTL"),
	SND_SOC_DAPM_OUTPUT("VOUTR"),
        SND_SOC_DAPM_INPUT("LINE_IN"),
        SND_SOC_DAPM_INPUT("MIC_IN"),
        SND_SOC_DAPM_OUTPUT("HP_OUT"),
        SND_SOC_DAPM_OUTPUT("SPK_OUT"),
};


static const struct snd_soc_dapm_route es8328_intercon[] = {
	{"VOUTL", NULL, "DAC"},
	{"VOUTR", NULL, "DAC"},
};

static int es8328_mute(struct snd_soc_dai *dai, int mute)
{
#if 0
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = snd_soc_read(codec, ES8328_DACCTL);

	if (mute)
		snd_soc_write(codec, ES8328_DACCTL, mute_reg | 1);
	else
		snd_soc_write(codec, ES8328_DACCTL, mute_reg & ~1);
#endif
	return 0;
}

static int es8328_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		u8 dac = snd_soc_read(codec, ES8328_DACCONTROL2);
		dac &= ~0x1f;

		switch (params_rate(params)) {
		case 8000:
			dac |= sample_ratios[ES8328_RATE_8019];
			break;
		case 11025:
			dac |= sample_ratios[ES8328_RATE_11025];
			break;
		case 22050:
			dac |= sample_ratios[ES8328_RATE_22050];
			break;
		case 44100:
			dac |= sample_ratios[ES8328_RATE_44100];
			break;
		default:
			dev_err(codec->dev, "%s: unknown rate %d\n",
				 __func__, params_rate(params));
			return -EINVAL;
		}
		snd_soc_write(codec, ES8328_DACCONTROL2, dac);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		u8 adc = snd_soc_read(codec, ES8328_ADCCONTROL5);
		adc &= ~0x1f;

		switch (params_rate(params)) {
		case 8000:
			adc |= sample_ratios[ES8328_RATE_8019];
			break;
		case 11025:
			adc |= sample_ratios[ES8328_RATE_11025];
			break;
		case 22050:
			adc |= sample_ratios[ES8328_RATE_22050];
			break;
		case 44100:
			adc |= sample_ratios[ES8328_RATE_44100];
			break;
		default:
			dev_err(codec->dev, "%s: unknown rate %d\n",
				 __func__, params_rate(params));
			return -EINVAL;
		}
		snd_soc_write(codec, ES8328_ADCCONTROL5, adc);
	}


#if 0
	dac &= ~0x18;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		dac |= 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dac |= 0x08;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ES8328_DACCTL, dac);
#endif
	snd_soc_write(codec, ES8328_CHIPPOWER,  0x00);

	return 0;
}

static int es8328_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	pr_err("Setting FMT to 0x%x\n", fmt);
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = snd_soc_read(codec, ES8328_IFCTL);

	/* Currently only I2S is supported by the driver, though the
	 * hardware is more flexible.
	 */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 1;
		break;
	default:
		return -EINVAL;
	}

	/* The hardware only support full slave mode */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface &= ~0x22;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |=  0x20;
		iface &= ~0x02;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x02;
		iface &= ~0x20;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x22;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ES8328_IFCTL, iface);
#endif
	return 0;
}

/* set codec sysclk */
static int es8328_set_dai_sysclk(struct snd_soc_dai *codec_dai,
			         int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es8328_priv *es8328 = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case 0:
		es8328->sysclk = freq;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int es8328_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
#if 0
	struct es8328_priv *es8328 = snd_soc_codec_get_drvdata(codec);
	u16 reg;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			/* Power everything up... */
			reg = snd_soc_read(codec, ES8328_DACCTL);
			snd_soc_write(codec, ES8328_DACCTL, reg & ~0x4);

			/* ..then sync in the register cache. */
			regcache_sync(es8328->regmap);
		}
		break;

	case SND_SOC_BIAS_OFF:
		reg = snd_soc_read(codec, ES8328_DACCTL);
		snd_soc_write(codec, ES8328_DACCTL, reg | 0x4);
		break;
	}
	codec->dapm.bias_level = level;
#endif
	return 0;
}

static int es8328_adc_enable(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, ES8328_ADCPOWER, 0x00);

	/* Set up microphone to be differential input */
	snd_soc_write(codec, ES8328_ADCCONTROL2, 0xf0);

	/* Set ADC to act as I2S master */
	snd_soc_write(codec, ES8328_ADCCONTROL3, 0x02);

	/* Set I2S to 16-bit mode */
	snd_soc_write(codec, ES8328_ADCCONTROL4, 0x18);

	/* Frequency clock of 272 */
	snd_soc_write(codec, ES8328_ADCCONTROL5, 0x02);

	return 0;
}

static int es8328_dac_enable(struct snd_soc_codec *codec)
{
	/* Power up LOUT2 ROUT2, and power down xOUT1 */
	snd_soc_write(codec, ES8328_DACPOWER,  0x3c);

	/* Set I2S to 16-bit mode */
	snd_soc_write(codec, ES8328_DACCONTROL1, 0x18);

	/* Frequency clock of 256, double-speed mode */
	snd_soc_write(codec, ES8328_DACCONTROL2, 0x02);

	/* No attenuation */
	snd_soc_write(codec, ES8328_DACCONTROL4, 0x00);
	snd_soc_write(codec, ES8328_DACCONTROL5, 0x00);

	/* Set LIN2 for the output mixer */
	snd_soc_write(codec, ES8328_DACCONTROL16, 0x09);



	/* Point the left DAC at the left mixer */
	snd_soc_write(codec, ES8328_DACCONTROL17, 0x80);
	/* Point the right DAC at the right mixer */
	snd_soc_write(codec, ES8328_DACCONTROL20, 0x80);

	/* Disable all other outputs */
	snd_soc_write(codec, ES8328_DACCONTROL18, 0x00);
	snd_soc_write(codec, ES8328_DACCONTROL19, 0x00);


	/* Disable mono mode for DACL, and mute DACR */
	snd_soc_write(codec, ES8328_DACCONTROL7, 0x00);

	return 0;
}


static int es8328_init(struct snd_soc_codec *codec)
{
	/* Master serial port mode */
	snd_soc_write(codec, ES8328_MASTERMODE, 0xc0);

	/* Power everything down and reset the cip */
	snd_soc_write(codec, ES8328_CHIPPOWER, 0xf3);

	/* Power up.  Set ADC and DAC to use different frequency ratios */
	snd_soc_write(codec, ES8328_CONTROL1, 0x04);

	/* Power up more blocks */
	snd_soc_write(codec, ES8328_CONTROL2, 0x40);


	/* Power up ADC */
	es8328_adc_enable(codec);

	/* Power up DAC */
	es8328_dac_enable(codec);

	/* Power on the chip */
	snd_soc_write(codec, ES8328_CHIPPOWER, 0x00);


	/* Turn off muting */
	snd_soc_write(codec, ES8328_DACCONTROL3, 0x30);

	return 0;
}

#define ES8328_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_22050)
#define ES8328_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

static const struct snd_soc_dai_ops es8328_dai_ops = {
	.hw_params	= es8328_hw_params,
	.digital_mute	= es8328_mute,
	.set_fmt	= es8328_set_dai_fmt,
	.set_sysclk	= es8328_set_dai_sysclk,
};

static struct snd_soc_dai_driver es8328_dai = {
	.name = "es8328",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES8328_RATES,
		.formats = ES8328_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES8328_RATES,
		.formats = ES8328_FORMATS,
	},
	.ops = &es8328_dai_ops,
};

static int es8328_suspend(struct snd_soc_codec *codec)
{
	es8328_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int es8328_resume(struct snd_soc_codec *codec)
{
	es8328_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int es8328_probe(struct snd_soc_codec *codec)
{
	int ret;
	struct device *dev = codec->dev;

	dev_dbg(dev, "probing audio codec\n");
	ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(dev, "failed to configure cache I/O: %d\n",
		       ret);
		return ret;
	}

	/* power on device */
	es8328_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	es8328_init(codec);

	return ret;
}

static int es8328_remove(struct snd_soc_codec *codec)
{
	es8328_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es8328 = {
	.probe =		es8328_probe,
	.remove =		es8328_remove,
	.suspend =		es8328_suspend,
	.resume =		es8328_resume,
	.set_bias_level =	es8328_set_bias_level,
	.controls =		es8328_snd_controls,
	.num_controls =		ARRAY_SIZE(es8328_snd_controls),
	.dapm_widgets =		es8328_dapm_widgets,
	.num_dapm_widgets =	ARRAY_SIZE(es8328_dapm_widgets),
	.dapm_routes =		es8328_intercon,
	.num_dapm_routes =	ARRAY_SIZE(es8328_intercon),
};

static const struct of_device_id es8328_of_match[] = {
	{ .compatible = "everest,es8328", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8328_of_match);

static const struct regmap_config es8328_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ES8328_REG_MAX,

	.cache_type = REGCACHE_NONE,
};

#if defined(CONFIG_SPI_MASTER)
static int es8328_spi_probe(struct spi_device *spi)
{
	struct es8328_priv *es8328;
	int ret;

	es8328 = devm_kzalloc(&spi->dev, sizeof(struct es8328_priv),
			      GFP_KERNEL);
	if (es8328 == NULL)
		return -ENOMEM;

	es8328->regmap = devm_regmap_init_spi(spi, &es8328_regmap);
	if (IS_ERR(es8328->regmap))
		return PTR_ERR(es8328->regmap);

	spi_set_drvdata(spi, es8328);

	ret = snd_soc_register_codec(&spi->dev,
			&soc_codec_dev_es8328, &es8328_dai, 1);

	return ret;
}

static int es8328_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);

	return 0;
}

static struct spi_driver es8328_spi_driver = {
	.driver = {
		.name	= "es8328",
		.owner	= THIS_MODULE,
		.of_match_table = es8328_of_match,
	},
	.probe		= es8328_spi_probe,
	.remove		= es8328_spi_remove,
};
#endif /* CONFIG_SPI_MASTER */

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int es8328_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8328_priv *es8328;
	int ret;

	es8328 = devm_kzalloc(&i2c->dev, sizeof(struct es8328_priv),
			      GFP_KERNEL);
	if (es8328 == NULL)
		return -ENOMEM;

	es8328->regmap = devm_regmap_init_i2c(i2c, &es8328_regmap);
	if (IS_ERR(es8328->regmap))
		return PTR_ERR(es8328->regmap);

	i2c_set_clientdata(i2c, es8328);

	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_es8328, &es8328_dai, 1);

	return ret;
}

static int es8328_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id es8328_i2c_id[] = {
	{ "es8328", 0x11 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8328_i2c_id);

static struct i2c_driver es8328_i2c_driver = {
	.driver = {
		.name = "es8328",
		.owner = THIS_MODULE,
		.of_match_table = es8328_of_match,
	},
	.probe =    es8328_i2c_probe,
	.remove =   es8328_i2c_remove,
	.id_table = es8328_i2c_id,
};
#endif

static int __init es8328_modinit(void)
{
	int ret = 0;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	pr_debug("es8328: adding i2c driver\n");
	ret = i2c_add_driver(&es8328_i2c_driver);
	if (ret != 0) {
		pr_err("failed to register es8328 I2C driver: %d\n", ret);
	}
#endif
#if defined(CONFIG_SPI_MASTER)
	pr_debug("es8328: adding spi driver\n");
	ret = spi_register_driver(&es8328_spi_driver);
	if (ret != 0) {
		pr_err("Failed to register es8328 SPI driver: %d\n", ret);
	}
#endif
	return ret;
}
module_init(es8328_modinit);

static void __exit es8328_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&es8328_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&es8328_spi_driver);
#endif
}
module_exit(es8328_exit);

MODULE_DESCRIPTION("ASoC WM8728 driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
