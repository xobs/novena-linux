#ifndef DW_HDMI_AUDIO_H
#define DW_HDMI_AUDIO_H

struct imx_hdmi;

struct dw_hdmi_audio_data {
	void __iomem *base;
	int irq;
	struct imx_hdmi *hdmi;
};

#endif
