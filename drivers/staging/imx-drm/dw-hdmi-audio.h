#ifndef DW_HDMI_AUDIO_H
#define DW_HDMI_AUDIO_H

#include <linux/irqreturn.h>

struct snd_dw_hdmi;
struct imx_hdmi;

int snd_dw_hdmi_probe(struct snd_dw_hdmi **dwp, struct device *,
	void __iomem *, int, struct imx_hdmi *);
void snd_dw_hdmi_remove(struct snd_dw_hdmi *dw);

#endif
