/*
 * DesignWare HDMI audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Written and tested against the (alleged) DW HDMI Tx found in iMX6S.
 */
#include <linux/io.h>
#include <linux/interrupt.h>

#include <sound/asoundef.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include "imx-hdmi.h"
#include "dw-hdmi-audio.h"

#define DRIVER_NAME "dw-hdmi-audio"

/* Provide some bits rather than bit offsets */
enum {
	HDMI_AHB_DMA_CONF0_SW_FIFO_RST = HDMI_AHB_DMA_CONF0_SW_FIFO_RST_MASK,
	HDMI_AHB_DMA_CONF0_EN_HLOCK = HDMI_AHB_DMA_CONF0_EN_HLOCK_MASK,
	HDMI_AHB_DMA_START_START = BIT(HDMI_AHB_DMA_START_START_OFFSET),
	HDMI_AHB_DMA_STOP_STOP = BIT(HDMI_AHB_DMA_STOP_STOP_OFFSET),
	HDMI_IH_MUTE_AHBDMAAUD_STAT0_ALL =
		HDMI_IH_MUTE_AHBDMAAUD_STAT0_ERROR |
		HDMI_IH_MUTE_AHBDMAAUD_STAT0_LOST |
		HDMI_IH_MUTE_AHBDMAAUD_STAT0_RETRY |
		HDMI_IH_MUTE_AHBDMAAUD_STAT0_DONE |
		HDMI_IH_MUTE_AHBDMAAUD_STAT0_BUFFFULL |
		HDMI_IH_MUTE_AHBDMAAUD_STAT0_BUFFEMPTY,
	HDMI_IH_AHBDMAAUD_STAT0_ALL =
		HDMI_IH_AHBDMAAUD_STAT0_ERROR |
		HDMI_IH_AHBDMAAUD_STAT0_LOST |
		HDMI_IH_AHBDMAAUD_STAT0_RETRY |
		HDMI_IH_AHBDMAAUD_STAT0_DONE |
		HDMI_IH_AHBDMAAUD_STAT0_BUFFFULL |
		HDMI_IH_AHBDMAAUD_STAT0_BUFFEMPTY,
};

struct snd_dw_hdmi {
	struct snd_card *card;
	struct snd_pcm *pcm;
	void __iomem *base;
	int irq;
	struct imx_hdmi *hdmi;
	struct snd_pcm_substream *substream;
	void (*reformat)(struct snd_dw_hdmi *, size_t, size_t);
	void *buf_base;
	dma_addr_t buf_addr;
	unsigned buf_offset;
	unsigned buf_period;
	unsigned buf_size;
	unsigned channels;
	uint8_t revision;
	uint8_t iec_offset;
	uint8_t cs[192][8];
};

static void dw_hdmi_writeb(unsigned long val, void __iomem *ptr)
{
	writeb(val, ptr);
}

static unsigned dw_hdmi_readb(void __iomem *ptr)
{
	return readb(ptr);
}

static void dw_hdmi_writel(unsigned long val, void __iomem *ptr)
{
	writeb_relaxed(val, ptr);
	writeb_relaxed(val >> 8, ptr + 1);
	writeb_relaxed(val >> 16, ptr + 2);
	writeb_relaxed(val >> 24, ptr + 3);
}

/*
 * Convert to hardware format: The userspace buffer contains IEC958 samples,
 * with the PCUV bits in bits 31..28 and audio samples in bits 27..4.  We
 * need these to be in bits 27..24, with the IEC B bit in bit 28, and audio
 * samples in 23..0.
 *
 * Default preamble in bits 3..0: 8 = block start, 4 = even 2 = odd
 *
 * Ideally, we could do with having the data properly formatted in userspace.
 */
static void dw_hdmi_reformat_iec958(struct snd_dw_hdmi *dw,
	size_t offset, size_t bytes)
{
	uint32_t *ptr = dw->buf_base + offset;
	uint32_t *end = dw->buf_base + offset + bytes;

	do {
		uint32_t b, sample = *ptr;

		b = (sample & 8) << (28 - 3);

		sample >>= 4;

		*ptr++ = sample | b;
	} while (ptr < end);
}

static uint32_t parity(uint32_t sample)
{
	sample ^= sample >> 16;
	sample ^= sample >> 8;
	sample ^= sample >> 4;
	sample ^= sample >> 2;
	sample ^= sample >> 1;
	return (sample & 1) << 27;
}

static void dw_hdmi_reformat_s24(struct snd_dw_hdmi *dw,
	size_t offset, size_t bytes)
{
	uint32_t *ptr = dw->buf_base + offset;
	uint32_t *end = dw->buf_base + offset + bytes;

	do {
		unsigned i;
		uint8_t *cs;

		cs = dw->cs[dw->iec_offset++];
		if (dw->iec_offset >= 192)
			dw->iec_offset = 0;

		i = dw->channels;
		do {
			uint32_t sample = *ptr;

			sample &= ~0xff000000;
			sample |= *cs++ << 24;
			sample |= parity(sample & ~0xf8000000);

			*ptr++ = sample;
		} while (--i);
	} while (ptr < end);
}

static void dw_hdmi_create_cs(struct snd_dw_hdmi *dw,
	struct snd_pcm_runtime *runtime)
{
	uint8_t cs[3];
	unsigned ch, i, j;

	cs[0] = IEC958_AES0_CON_NOT_COPYRIGHT | IEC958_AES0_CON_EMPHASIS_NONE;
	cs[1] = IEC958_AES1_CON_GENERAL;
	cs[2] = IEC958_AES2_CON_SOURCE_UNSPEC;
	cs[3] = IEC958_AES3_CON_CLOCK_1000PPM;

	switch (runtime->rate) {
	case 32000:
		cs[3] |= IEC958_AES3_CON_FS_32000;
		break;
	case 44100:
		cs[3] |= IEC958_AES3_CON_FS_44100;
		break;
	case 48000:
		cs[3] |= IEC958_AES3_CON_FS_48000;
		break;
	case 88200:
		cs[3] |= IEC958_AES3_CON_FS_88200;
		break;
	case 96000:
		cs[3] |= IEC958_AES3_CON_FS_96000;
		break;
	case 176400:
		cs[3] |= IEC958_AES3_CON_FS_176400;
		break;
	case 192000:
		cs[3] |= IEC958_AES3_CON_FS_192000;
		break;
	}

	memset(dw->cs, 0, sizeof(dw->cs));

	for (ch = 0; ch < 8; ch++) {
		cs[2] &= ~IEC958_AES2_CON_CHANNEL;
		cs[2] |= (ch + 1) << 4;

		for (i = 0; i < ARRAY_SIZE(cs); i++) {
			unsigned c = cs[i];

			for (j = 0; j < 8; j++, c >>= 1)
				dw->cs[i * 8 + j][ch] = (c & 1) << 2;
		}
	}
	dw->cs[0][0] |= BIT(4);
}

static void dw_hdmi_start_dma(struct snd_dw_hdmi *dw)
{
	unsigned long start, stop;

	start = dw->buf_addr + dw->buf_offset;
	stop = start + dw->buf_period - 1;

	dw->reformat(dw, dw->buf_offset, dw->buf_period);

	/* Setup the hardware start/stop addresses */
	dw_hdmi_writel(start, dw->base + HDMI_AHB_DMA_STRADDR0);
	dw_hdmi_writel(stop, dw->base + HDMI_AHB_DMA_STPADDR0);

	/* Clear all irqs before enabling irqs and starting DMA */
	dw_hdmi_writeb(HDMI_IH_AHBDMAAUD_STAT0_ALL,
		       dw->base + HDMI_IH_AHBDMAAUD_STAT0);
	dw_hdmi_writeb(~HDMI_AHB_DMA_DONE, dw->base + HDMI_AHB_DMA_MASK);
	dw_hdmi_writeb(HDMI_AHB_DMA_START_START, dw->base + HDMI_AHB_DMA_START);
}

static void dw_hdmi_stop_dma(struct snd_dw_hdmi *dw)
{
	dw->substream = NULL;

	/* Disable interrupts before disabling DMA */
	dw_hdmi_writeb(~0, dw->base + HDMI_AHB_DMA_MASK);
	dw_hdmi_writeb(HDMI_AHB_DMA_STOP_STOP, dw->base + HDMI_AHB_DMA_STOP);
}

static irqreturn_t snd_dw_hdmi_irq(int irq, void *data)
{
	struct snd_dw_hdmi *dw = data;
	struct snd_pcm_substream *substream;
	unsigned stat;

	stat = dw_hdmi_readb(dw->base + HDMI_IH_AHBDMAAUD_STAT0);
	if (!stat)
		return IRQ_NONE;

	dw_hdmi_writeb(stat, dw->base + HDMI_IH_AHBDMAAUD_STAT0);

	substream = dw->substream;
	if (stat & HDMI_IH_AHBDMAAUD_STAT0_DONE && substream) {
		dw->buf_offset += dw->buf_period;
		if (dw->buf_offset >= dw->buf_size)
			dw->buf_offset = 0;

		snd_pcm_period_elapsed(substream);
		if (dw->substream)
			dw_hdmi_start_dma(dw);
	}

	return IRQ_HANDLED;
}

static struct snd_pcm_hardware dw_hdmi_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE |
		   SNDRV_PCM_FMTBIT_S24_LE,
	.rates = SNDRV_PCM_RATE_32000 |
		 SNDRV_PCM_RATE_44100 |
		 SNDRV_PCM_RATE_48000 |
		 SNDRV_PCM_RATE_88200 |
		 SNDRV_PCM_RATE_96000 |
		 SNDRV_PCM_RATE_176400 |
		 SNDRV_PCM_RATE_192000,
	.channels_min = 2,
	.channels_max = 8,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 8192,	/* ERR004323: must limit to 8k */
	.periods_min = 2,
	.periods_max = 16,
	.fifo_size = 0,
};

static int dw_hdmi_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dw_hdmi *dw = substream->private_data;
	int ret;

	/* Clear FIFO */
	dw_hdmi_writeb(HDMI_AHB_DMA_CONF0_SW_FIFO_RST,
		       dw->base + HDMI_AHB_DMA_CONF0);

	/* Configure interrupt polarities */
	dw_hdmi_writeb(~0, dw->base + HDMI_AHB_DMA_POL);
	dw_hdmi_writeb(~0, dw->base + HDMI_AHB_DMA_BUFFPOL);

	/* Keep interrupts masked */
	dw_hdmi_writeb(~0, dw->base + HDMI_AHB_DMA_MASK);

	ret = request_irq(dw->irq, snd_dw_hdmi_irq, IRQF_SHARED,
			  "dw-hdmi-audio", dw);
	if (ret)
		return ret;

	/* Un-mute done interrupt */
	dw_hdmi_writeb(HDMI_IH_MUTE_AHBDMAAUD_STAT0_ALL &
		       ~HDMI_IH_MUTE_AHBDMAAUD_STAT0_DONE,
		       dw->base + HDMI_IH_MUTE_AHBDMAAUD_STAT0);

	runtime->hw = dw_hdmi_hw;
	snd_pcm_limit_hw_rates(runtime);

	return 0;
}

static int dw_hdmi_close(struct snd_pcm_substream *substream)
{
	struct snd_dw_hdmi *dw = substream->private_data;

	/* Mute all interrupts */
	dw_hdmi_writeb(HDMI_IH_MUTE_AHBDMAAUD_STAT0_ALL,
		       dw->base + HDMI_IH_MUTE_AHBDMAAUD_STAT0);

	free_irq(dw->irq, dw);

	return 0;
}

static int dw_hdmi_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int dw_hdmi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(params));
}

static int dw_hdmi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dw_hdmi *dw = substream->private_data;
	uint8_t threshold, conf0, conf1;

	/* Setup as per 3.0.5 FSL 4.1.0 BSP */
	switch (dw->revision) {
	case 0x0a:
		conf0 = HDMI_AHB_DMA_CONF0_BURST_MODE |
			HDMI_AHB_DMA_CONF0_INCR4;
		if (runtime->channels == 2)
			threshold = 126;
		else
			threshold = 124;
		break;
	case 0x1a:
		conf0 = HDMI_AHB_DMA_CONF0_BURST_MODE |
			HDMI_AHB_DMA_CONF0_INCR8;
		threshold = 128;
		break;
	default:
		/* NOTREACHED */
		return -EINVAL;
	}

	imx_hdmi_set_sample_rate(dw->hdmi, runtime->rate);

	/* Minimum number of bytes in the fifo. */
	runtime->hw.fifo_size = threshold * 32;

	conf0 |= HDMI_AHB_DMA_CONF0_EN_HLOCK;
	conf1 = (1 << runtime->channels) - 1;

	dw_hdmi_writeb(threshold, dw->base + HDMI_AHB_DMA_THRSLD);
	dw_hdmi_writeb(conf0, dw->base + HDMI_AHB_DMA_CONF0);
	dw_hdmi_writeb(conf1, dw->base + HDMI_AHB_DMA_CONF1);

	switch (runtime->format) {
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:
		dw->reformat = dw_hdmi_reformat_iec958;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dw_hdmi_create_cs(dw, runtime);
		dw->reformat = dw_hdmi_reformat_s24;
		break;
	}
	dw->iec_offset = 0;
	dw->channels = runtime->channels;
	dw->buf_base = runtime->dma_area;
	dw->buf_addr = runtime->dma_addr;
	dw->buf_period = snd_pcm_lib_period_bytes(substream);
	dw->buf_size = snd_pcm_lib_buffer_bytes(substream);

	return 0;
}

static int dw_hdmi_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_dw_hdmi *dw = substream->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		dw->buf_offset = 0;
		dw->substream = substream;
		dw_hdmi_start_dma(dw);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		dw_hdmi_stop_dma(dw);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t dw_hdmi_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dw_hdmi *dw = substream->private_data;

	return bytes_to_frames(runtime, dw->buf_offset);
}

static struct snd_pcm_ops snd_dw_hdmi_ops = {
	.open = dw_hdmi_open,
	.close = dw_hdmi_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = dw_hdmi_hw_params,
	.hw_free = dw_hdmi_hw_free,
	.prepare = dw_hdmi_prepare,
	.trigger = dw_hdmi_trigger,
	.pointer = dw_hdmi_pointer,
};

int snd_dw_hdmi_probe(struct snd_dw_hdmi **dwp, struct device *dev,
	void __iomem *base, int irq, struct imx_hdmi *hdmi)
{
	struct snd_dw_hdmi *dw;
	struct snd_card *card;
	struct snd_pcm *pcm;
	unsigned revision;
	int ret;

	dw_hdmi_writeb(HDMI_IH_MUTE_AHBDMAAUD_STAT0_ALL,
		       base + HDMI_IH_MUTE_AHBDMAAUD_STAT0);
	revision = dw_hdmi_readb(base + HDMI_REVISION_ID);
	if (revision != 0x0a && revision != 0x1a) {
		dev_err(dev, "dw-hdmi-audio: unknown revision 0x%02x\n",
			revision);
		return -ENXIO;
	}

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			      THIS_MODULE, sizeof(struct snd_dw_hdmi), &card);
	if (ret < 0)
		return ret;

	snd_card_set_dev(card, dev);

	strlcpy(card->driver, DRIVER_NAME, sizeof(card->driver));
	strlcpy(card->shortname, "DW-HDMI", sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "%s rev 0x%02x, irq %d", card->shortname, revision, irq);

	dw = card->private_data;
	dw->card = card;
	dw->base = base;
	dw->irq = irq;
	dw->hdmi = hdmi;
	dw->revision = revision;

	ret = snd_pcm_new(card, "DW HDMI", 0, 1, 0, &pcm);
	if (ret < 0)
		goto err;

	dw->pcm = pcm;
	pcm->private_data = dw;
	strlcpy(pcm->name, DRIVER_NAME, sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_dw_hdmi_ops);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
			NULL, 0, 64 * 1024);

	ret = snd_card_register(card);
	if (ret < 0)
		goto err;

	*dwp = dw;

	return 0;

err:
	snd_card_free(card);
	return ret;
}

void snd_dw_hdmi_remove(struct snd_dw_hdmi *dw)
{
	snd_card_free(dw->card);
}
