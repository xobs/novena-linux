/*
 * es8328.c  --  ES8328 ALSA SoC Audio driver
 *
 * Copyright 2014 Sutajio Ko-Usagi PTE LTD
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "es8328.h"

/* Run the codec at 22.5792 MHz to support these rates */
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

#define ES8328_RATES (SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_11025)
#define ES8328_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

/* codec private data */
struct es8328_priv {
	struct regmap *regmap;
	int sysclk;
};

/*
 * ES8328 Controls
 */

static const char *deemph_txt[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static SOC_ENUM_SINGLE_DECL(deemph,
			    ES8328_DACCONTROL6, 6, deemph_txt);

static const char *adcpol_txt[] = {"Normal", "L Invert", "R Invert",
				   "L + R Invert"};
static SOC_ENUM_SINGLE_DECL(adcpol,
			    ES8328_ADCCONTROL6, 6, adcpol_txt);

static const DECLARE_TLV_DB_SCALE(play_tlv, -3000, 100, 0);
static const DECLARE_TLV_DB_SCALE(dac_adc_tlv, -9600, 50, 0);
static const DECLARE_TLV_DB_SCALE(pga_tlv, 0, 300, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, -1500, 300, 0);
static const DECLARE_TLV_DB_SCALE(mic_tlv, 0, 300, 0);

static const struct snd_kcontrol_new es8328_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Capture Digital Volume",
		ES8328_ADCCONTROL8, ES8328_ADCCONTROL9,
		 0, 0xc0, 0, dac_adc_tlv),
	SOC_SINGLE("Capture ZC Switch", ES8328_ADCCONTROL7, 6, 1, 0),

	SOC_ENUM("Playback De-emphasis", deemph),

	SOC_ENUM("Capture Polarity", adcpol),

	SOC_SINGLE_TLV("Left Mixer Left Bypass Volume",
			ES8328_DACCONTROL17, 3, 7, 1, bypass_tlv),
	SOC_SINGLE_TLV("Left Mixer Right Bypass Volume",
			ES8328_DACCONTROL19, 3, 7, 1, bypass_tlv),
	SOC_SINGLE_TLV("Right Mixer Left Bypass Volume",
			ES8328_DACCONTROL18, 3, 7, 1, bypass_tlv),
	SOC_SINGLE_TLV("Right Mixer Right Bypass Volume",
			ES8328_DACCONTROL20, 3, 7, 1, bypass_tlv),

	SOC_DOUBLE_R_TLV("PCM Volume",
			ES8328_LDACVOL, ES8328_RDACVOL,
			0, ES8328_DACVOL_MAX, 1, dac_adc_tlv),

	SOC_DOUBLE_R_TLV("Output 1 Playback Volume",
			ES8328_LOUT1VOL, ES8328_ROUT1VOL,
			0, ES8328_OUT1VOL_MAX, 0, play_tlv),

	SOC_DOUBLE_R_TLV("Output 2 Playback Volume",
			ES8328_LOUT2VOL, ES8328_ROUT2VOL,
			0, ES8328_OUT2VOL_MAX, 0, play_tlv),

	SOC_DOUBLE_TLV("Mic PGA Gain", ES8328_ADCCONTROL1, 4, 0, 8, 0, mic_tlv),
};

/*
 * DAPM Controls
 */

static const char *es8328_line_texts[] = {
	"Line 1", "Line 2", "PGA", "Differential"};

static const unsigned int es8328_line_values[] = {
	0, 1, 2, 3};

static const struct soc_enum es8328_lline_enum =
	SOC_VALUE_ENUM_SINGLE(ES8328_DACCONTROL16, 3, 7,
			      ARRAY_SIZE(es8328_line_texts),
			      es8328_line_texts,
			      es8328_line_values);
static const struct snd_kcontrol_new es8328_left_line_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8328_lline_enum);

static const struct soc_enum es8328_rline_enum =
	SOC_VALUE_ENUM_SINGLE(ES8328_DACCONTROL16, 0, 7,
			      ARRAY_SIZE(es8328_line_texts),
			      es8328_line_texts,
			      es8328_line_values);
static const struct snd_kcontrol_new es8328_right_line_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8328_lline_enum);

/* Left Mixer */
static const struct snd_kcontrol_new es8328_left_mixer_controls[] = {
	SOC_DAPM_SINGLE("Playback Switch", ES8328_DACCONTROL17, 8, 1, 0),
	SOC_DAPM_SINGLE("Left Bypass Switch", ES8328_DACCONTROL17, 7, 1, 0),
	SOC_DAPM_SINGLE("Right Playback Switch", ES8328_DACCONTROL18, 8, 1, 0),
	SOC_DAPM_SINGLE("Right Bypass Switch", ES8328_DACCONTROL18, 7, 1, 0),
};

/* Right Mixer */
static const struct snd_kcontrol_new es8328_right_mixer_controls[] = {
	SOC_DAPM_SINGLE("Left Playback Switch", ES8328_DACCONTROL19, 8, 1, 0),
	SOC_DAPM_SINGLE("Left Bypass Switch", ES8328_DACCONTROL19, 7, 1, 0),
	SOC_DAPM_SINGLE("Playback Switch", ES8328_DACCONTROL20, 8, 1, 0),
	SOC_DAPM_SINGLE("Right Bypass Switch", ES8328_DACCONTROL20, 7, 1, 0),
};

static const char *es8328_pga_sel[] = {
	"Line 1", "Line 2", "Line 3", "Differential"};
static const unsigned int es8328_pga_val[] = { 0, 1, 2, 3 };

/* Left PGA Mux */
static const struct soc_enum es8328_lpga_enum =
	SOC_VALUE_ENUM_SINGLE(ES8328_ADCCONTROL2, 6, 3,
			      ARRAY_SIZE(es8328_pga_sel),
			      es8328_pga_sel,
			      es8328_pga_val);
static const struct snd_kcontrol_new es8328_left_pga_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8328_lpga_enum);

/* Right PGA Mux */
static const struct soc_enum es8328_rpga_enum =
	SOC_VALUE_ENUM_SINGLE(ES8328_ADCCONTROL2, 4, 3,
			      ARRAY_SIZE(es8328_pga_sel),
			      es8328_pga_sel,
			      es8328_pga_val);
static const struct snd_kcontrol_new es8328_right_pga_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8328_rpga_enum);

/* Differential Mux */
static const char *es8328_diff_sel[] = {"Line 1", "Line 2"};
static SOC_ENUM_SINGLE_DECL(diffmux,
			    ES8328_ADCCONTROL3, 7, es8328_diff_sel);
static const struct snd_kcontrol_new es8328_diffmux_controls =
	SOC_DAPM_ENUM("Route", diffmux);

/* Mono ADC Mux */
static const char *es8328_mono_mux[] = {"Stereo", "Mono (Left)",
	"Mono (Right)", "Digital Mono"};
static SOC_ENUM_SINGLE_DECL(monomux,
			    ES8328_ADCCONTROL3, 3, es8328_mono_mux);
static const struct snd_kcontrol_new es8328_monomux_controls =
	SOC_DAPM_ENUM("Route", monomux);

static const struct snd_soc_dapm_widget es8328_dapm_widgets[] = {
	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
		&es8328_diffmux_controls),
	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
		&es8328_monomux_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
		&es8328_monomux_controls),

	SND_SOC_DAPM_MUX("Left PGA Mux", ES8328_ADCPOWER,
			ES8328_ADCPOWER_AINL_OFF, 1,
			&es8328_left_pga_controls),
	SND_SOC_DAPM_MUX("Right PGA Mux", ES8328_ADCPOWER,
			ES8328_ADCPOWER_AINR_OFF, 1,
			&es8328_right_pga_controls),

	SND_SOC_DAPM_MUX("Left Line Mux", SND_SOC_NOPM, 0, 0,
		&es8328_left_line_controls),
	SND_SOC_DAPM_MUX("Right Line Mux", SND_SOC_NOPM, 0, 0,
		&es8328_right_line_controls),

	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ES8328_ADCPOWER,
			ES8328_ADCPOWER_ADCR_OFF, 1),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ES8328_ADCPOWER,
			ES8328_ADCPOWER_ADCL_OFF, 1),

	SND_SOC_DAPM_MICBIAS("Mic Bias", ES8328_ADCPOWER,
			ES8328_ADCPOWER_MIC_BIAS_OFF, 1),
	SND_SOC_DAPM_SUPPLY("Mic Bias Gen", ES8328_ADCPOWER,
			ES8328_ADCPOWER_ADC_BIAS_GEN_OFF, 1, NULL, 0),

	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", ES8328_DACPOWER,
			ES8328_DACPOWER_RDAC_OFF, 1),
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", ES8328_DACPOWER,
			ES8328_DACPOWER_LDAC_OFF, 1),

	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
		&es8328_left_mixer_controls[0],
		ARRAY_SIZE(es8328_left_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
		&es8328_right_mixer_controls[0],
		ARRAY_SIZE(es8328_right_mixer_controls)),

	SND_SOC_DAPM_PGA("Right Out 2", ES8328_DACPOWER,
			ES8328_DACPOWER_ROUT2_ON, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", ES8328_DACPOWER,
			ES8328_DACPOWER_LOUT2_ON, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 1", ES8328_DACPOWER,
			ES8328_DACPOWER_ROUT1_ON, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 1", ES8328_DACPOWER,
			ES8328_DACPOWER_LOUT1_ON, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	SND_SOC_DAPM_OUTPUT("VREF"),

	SND_SOC_DAPM_INPUT("LINPUT1"),
	SND_SOC_DAPM_INPUT("LINPUT2"),
	SND_SOC_DAPM_INPUT("RINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT2"),
};

static const struct snd_soc_dapm_route es8328_dapm_routes[] = {

	{ "Left Line Mux", "Line 1", "LINPUT1" },
	{ "Left Line Mux", "Line 2", "LINPUT2" },
	{ "Left Line Mux", "PGA", "Left PGA Mux" },
	{ "Left Line Mux", "Differential", "Differential Mux" },

	{ "Right Line Mux", "Line 1", "RINPUT1" },
	{ "Right Line Mux", "Line 2", "RINPUT2" },
	{ "Right Line Mux", "PGA", "Right PGA Mux" },
	{ "Right Line Mux", "Differential", "Differential Mux" },

	{ "Left PGA Mux", "Line 1", "LINPUT1" },
	{ "Left PGA Mux", "Line 2", "LINPUT2" },
	{ "Left PGA Mux", "Differential", "Differential Mux" },

	{ "Right PGA Mux", "Line 1", "RINPUT1" },
	{ "Right PGA Mux", "Line 2", "RINPUT2" },
	{ "Right PGA Mux", "Differential", "Differential Mux" },

	{ "Differential Mux", "Line 1", "LINPUT1" },
	{ "Differential Mux", "Line 1", "RINPUT1" },
	{ "Differential Mux", "Line 2", "LINPUT2" },
	{ "Differential Mux", "Line 2", "RINPUT2" },

	{ "Left ADC Mux", "Stereo", "Left PGA Mux" },
	{ "Left ADC Mux", "Mono (Left)", "Left PGA Mux" },
	{ "Left ADC Mux", "Digital Mono", "Left PGA Mux" },

	{ "Right ADC Mux", "Stereo", "Right PGA Mux" },
	{ "Right ADC Mux", "Mono (Right)", "Right PGA Mux" },
	{ "Right ADC Mux", "Digital Mono", "Right PGA Mux" },

	{ "Left ADC", NULL, "Left ADC Mux" },
	{ "Right ADC", NULL, "Right ADC Mux" },

	{ "Left Line Mux", "Line 1", "LINPUT1" },
	{ "Left Line Mux", "Line 2", "LINPUT2" },
	{ "Left Line Mux", "PGA", "Left PGA Mux" },
	{ "Left Line Mux", "Differential", "Differential Mux" },

	{ "Right Line Mux", "Line 1", "RINPUT1" },
	{ "Right Line Mux", "Line 2", "RINPUT2" },
	{ "Right Line Mux", "PGA", "Right PGA Mux" },
	{ "Right Line Mux", "Differential", "Differential Mux" },

	{ "Left Out 1", NULL, "Left DAC" },
	{ "Right Out 1", NULL, "Right DAC" },
	{ "Left Out 2", NULL, "Left DAC" },
	{ "Right Out 2", NULL, "Right DAC" },
	
	{ "Left Mixer", "Playback Switch", "Left DAC" },
	{ "Left Mixer", "Left Bypass Switch", "Left Line Mux" },
	{ "Left Mixer", "Right Playback Switch", "Right DAC" },
	{ "Left Mixer", "Right Bypass Switch", "Right Line Mux" },

	{ "Right Mixer", "Left Playback Switch", "Left DAC" },
	{ "Right Mixer", "Left Bypass Switch", "Left Line Mux" },
	{ "Right Mixer", "Playback Switch", "Right DAC" },
	{ "Right Mixer", "Right Bypass Switch", "Right Line Mux" },

	{ "Left Out 1", NULL, "Left Mixer" },
	{ "LOUT1", NULL, "Left Out 1" },
	{ "Right Out 1", NULL, "Right Mixer" },
	{ "ROUT1", NULL, "Right Out 1" },

	{ "Left Out 2", NULL, "Left Mixer" },
	{ "LOUT2", NULL, "Left Out 2" },
	{ "Right Out 2", NULL, "Right Mixer" },
	{ "ROUT2", NULL, "Right Out 2" },
};

static int es8328_mute(struct snd_soc_dai *dai, int mute)
{
	return snd_soc_update_bits(dai->codec, ES8328_DACCONTROL3,
			ES8328_DACCONTROL3_DACMUTE,
			mute ? ES8328_DACCONTROL3_DACMUTE : 0);
}

static int es8328_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		u8 dac = snd_soc_read(codec, ES8328_DACCONTROL2);
		dac &= ~ES8328_DACCONTROL2_RATEMASK;

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
		adc &= ~ES8328_ADCCONTROL5_RATEMASK;

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

	return 0;
}

static int es8328_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 mode = ES8328_DACCONTROL1_DACWL_16;

	/* set master/slave audio interface */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBM_CFM)
		return -EINVAL;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		mode |= ES8328_DACCONTROL1_DACFORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		mode |= ES8328_DACCONTROL1_DACFORMAT_RJUST;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		mode |= ES8328_DACCONTROL1_DACFORMAT_LJUST;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	snd_soc_write(codec, ES8328_DACCONTROL1, mode);
	snd_soc_write(codec, ES8328_ADCCONTROL4, mode);

	/* Master serial port mode */
	snd_soc_write(codec, ES8328_MASTERMODE,
			ES8328_MASTERMODE_MCLKDIV2 |
			ES8328_MASTERMODE_MSC);
	return 0;
}

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
	u8 cc1_reg = snd_soc_read(codec, ES8328_CONTROL1) & ~0x07;
	u8 pwr_reg = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* VREF, VMID=2x50k, digital enabled */
		snd_soc_write(codec, ES8328_CHIPPOWER, pwr_reg);
		snd_soc_write(codec, ES8328_CONTROL1,
				cc1_reg |
				ES8328_CONTROL1_VMIDSEL_50k |
				ES8328_CONTROL1_ENREF);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			snd_soc_write(codec, ES8328_CONTROL1,
					cc1_reg |
					ES8328_CONTROL1_VMIDSEL_5k |
					ES8328_CONTROL1_ENREF);

			/* Charge caps */
			msleep(100);
		}

		snd_soc_write(codec, ES8328_CONTROL2,
				ES8328_CONTROL2_OVERCURRENT_ON |
				ES8328_CONTROL2_THERMAL_SHUTDOWN_ON);

		/* VREF, VMID=2*500k, digital stopped */
		snd_soc_write(codec, ES8328_CONTROL1,
				cc1_reg |
				ES8328_CONTROL1_VMIDSEL_500k |
				ES8328_CONTROL1_ENREF);
		snd_soc_write(codec, ES8328_CHIPPOWER,
				pwr_reg |
				ES8328_CHIPPOWER_DACDIG_OFF |
				ES8328_CHIPPOWER_ADCDIG_OFF |
				ES8328_CHIPPOWER_ADCVREF_OFF |
				ES8328_CHIPPOWER_DACVREF_OFF);
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, ES8328_CONTROL1, cc1_reg);
		snd_soc_write(codec, ES8328_CHIPPOWER,
				ES8328_CHIPPOWER_ADCDIG_OFF |
				ES8328_CHIPPOWER_DACDIG_OFF |
				ES8328_CHIPPOWER_ADCSTM_RESET |
				ES8328_CHIPPOWER_DACSTM_RESET |
				ES8328_CHIPPOWER_ADCPLL_OFF |
				ES8328_CHIPPOWER_DACPLL_OFF |
				ES8328_CHIPPOWER_ADCVREF_OFF |
				ES8328_CHIPPOWER_DACVREF_OFF);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static const struct snd_soc_dai_ops es8328_dai_ops = {
	.hw_params	= es8328_hw_params,
	.digital_mute	= es8328_mute,
	.set_fmt	= es8328_set_dai_fmt,
	.set_sysclk	= es8328_set_dai_sysclk,
};

static struct snd_soc_dai_driver es8328_dai = {
	.name = "es8328-hifi-analog",
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
	struct es8328_priv *es8328 = snd_soc_codec_get_drvdata(codec);

	ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_REGMAP);
//	ret = snd_soc_codec_set_cache_io(codec, es8328->regmap);
	if (ret < 0) {
		dev_err(dev, "failed to configure cache I/O: %d\n", ret);
		return ret;
	}

	/* power on device */
	es8328_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int es8328_remove(struct snd_soc_codec *codec)
{
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
	.dapm_routes =		es8328_dapm_routes,
	.num_dapm_routes =	ARRAY_SIZE(es8328_dapm_routes),
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
	if (ret < 0)
		dev_err(&spi->dev, "unable to register codec: %d\n", ret);

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
		.name = "es8328-codec",
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
	ret = i2c_add_driver(&es8328_i2c_driver);
	if (ret != 0) {
		pr_err("failed to register es8328 I2C driver: %d\n", ret);
	}
#endif
#if defined(CONFIG_SPI_MASTER)
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

MODULE_DESCRIPTION("ASoC ES8328 driver");
MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_LICENSE("GPL");

#if 0
static int wm8988_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct wm8988_priv *wm8988 = snd_soc_codec_get_drvdata(codec);

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	if (!wm8988->sysclk) {
		dev_err(codec->dev,
			"No MCLK configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   wm8988->sysclk_constraints);

	return 0;
}

static int wm8988_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct wm8988_priv *wm8988 = snd_soc_codec_get_drvdata(codec);
	u16 iface = snd_soc_read(codec, WM8988_IFACE) & 0x1f3;
	u16 srate = snd_soc_read(codec, WM8988_SRATE) & 0x180;
	int coeff;

	coeff = get_coeff(wm8988->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(wm8988->sysclk / 2, params_rate(params));
		srate |= 0x40;
	}
	if (coeff < 0) {
		dev_err(codec->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), wm8988->sysclk);
		return coeff;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x000c;
		break;
	}

	/* set iface & srate */
	snd_soc_write(codec, WM8988_IFACE, iface);
	if (coeff >= 0)
		snd_soc_write(codec, WM8988_SRATE, srate |
			(coeff_div[coeff].sr << 1) | coeff_div[coeff].usb);

	return 0;
}

#endif
