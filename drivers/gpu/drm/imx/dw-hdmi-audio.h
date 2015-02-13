#ifndef DW_HDMI_AUDIO_H
#define DW_HDMI_AUDIO_H

struct imx_hdmi;

struct dw_hdmi_audio_data {
	phys_addr_t phys;
	void __iomem *base;
	int irq;
	struct imx_hdmi *hdmi;
	void (*set_sample_rate)(struct imx_hdmi *, unsigned);
};

#endif
