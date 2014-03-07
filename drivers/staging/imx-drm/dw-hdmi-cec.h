#ifndef DW_HDMI_CEC_H
#define DW_HDMI_CEC_H

struct dw_hdmi_cec_ops {
	void (*enable)(void *);
	void (*disable)(void *);
};

struct dw_hdmi_cec_data {
	void __iomem *base;
	int irq;
	const struct dw_hdmi_cec_ops *ops;
	void *ops_data;
};

#endif
