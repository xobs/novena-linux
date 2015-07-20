/* http://git.freescale.com/git/cgit.cgi/imx/linux-2.6-imx.git/tree/drivers/mxc/hdmi-cec/mxc_hdmi-cec.c?h=imx_3.0.35_4.1.0 */
#include <linux/cec-dev.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "dw_hdmi.h"
#include "dw_hdmi-cec.h"

#define DEV_NAME "mxc_hdmi_cec"

enum {
	CEC_STAT_DONE		= BIT(0),
	CEC_STAT_EOM		= BIT(1),
	CEC_STAT_NACK		= BIT(2),
	CEC_STAT_ARBLOST	= BIT(3),
	CEC_STAT_ERROR_INIT	= BIT(4),
	CEC_STAT_ERROR_FOLL	= BIT(5),
	CEC_STAT_WAKEUP		= BIT(6),

	CEC_CTRL_START		= BIT(0),
	CEC_CTRL_NORMAL		= 1 << 1,
};

struct dw_hdmi_cec {
	struct cec_dev cec;

	struct device *dev;
	void __iomem *base;
	const struct dw_hdmi_cec_ops *ops;
	void *ops_data;
	int irq;
};

static void dw_hdmi_set_address(struct cec_dev *cec_dev, unsigned addresses)
{
	struct dw_hdmi_cec *cec = container_of(cec_dev, struct dw_hdmi_cec, cec);

	writeb(addresses & 255, cec->base + HDMI_CEC_ADDR_L);
	writeb(addresses >> 8, cec->base + HDMI_CEC_ADDR_H);
}

static void dw_hdmi_send_message(struct cec_dev *cec_dev, u8 *msg,
	size_t count)
{
	struct dw_hdmi_cec *cec = container_of(cec_dev, struct dw_hdmi_cec, cec);
	unsigned i;

	for (i = 0; i < count; i++)
		writeb(msg[i], cec->base + HDMI_CEC_TX_DATA0 + i);

	writeb(count, cec->base + HDMI_CEC_TX_CNT);
	writeb(CEC_CTRL_NORMAL | CEC_CTRL_START, cec->base + HDMI_CEC_CTRL);
}

static irqreturn_t dw_hdmi_cec_irq(int irq, void *data)
{
	struct dw_hdmi_cec *cec = data;
	struct cec_dev *cec_dev = &cec->cec;
	unsigned stat = readb(cec->base + HDMI_IH_CEC_STAT0);

	if (stat == 0)
		return IRQ_NONE;

	writeb(stat, cec->base + HDMI_IH_CEC_STAT0);

	if (stat & CEC_STAT_ERROR_INIT) {
		if (cec->cec.retries) {
			unsigned v = readb(cec->base + HDMI_CEC_CTRL);
			writeb(v | CEC_CTRL_START, cec->base + HDMI_CEC_CTRL);
			cec->cec.retries -= 1;
		} else {
			cec->cec.write_busy = 0;
			cec_dev_event(cec_dev, MESSAGE_TYPE_SEND_ERROR, NULL, 0);
		}
	} else if (stat & (CEC_STAT_DONE | CEC_STAT_NACK))
		cec_dev_send_complete(cec_dev, stat & CEC_STAT_DONE);

	if (stat & CEC_STAT_EOM) {
		unsigned len, i;
		u8 msg[MAX_MESSAGE_LEN];

		len = readb(cec->base + HDMI_CEC_RX_CNT);
		if (len > sizeof(msg))
			len = sizeof(msg);

		for (i = 0; i < len; i++)
			msg[i] = readb(cec->base + HDMI_CEC_RX_DATA0 + i);

		writeb(0, cec->base + HDMI_CEC_LOCK);

		cec_dev_receive(cec_dev, msg, len);
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(dw_hdmi_cec_irq);

static void dw_hdmi_cec_release(struct cec_dev *cec_dev)
{
	struct dw_hdmi_cec *cec = container_of(cec_dev, struct dw_hdmi_cec, cec);

	writeb(~0, cec->base + HDMI_CEC_MASK);
	writeb(~0, cec->base + HDMI_IH_MUTE_CEC_STAT0);
	writeb(0, cec->base + HDMI_CEC_POLARITY);

	free_irq(cec->irq, cec);

	cec->ops->disable(cec->ops_data);
}

static int dw_hdmi_cec_open(struct cec_dev *cec_dev)
{
	struct dw_hdmi_cec *cec = container_of(cec_dev, struct dw_hdmi_cec, cec);
	unsigned irqs;
	int ret;

	writeb(0, cec->base + HDMI_CEC_CTRL);
	writeb(~0, cec->base + HDMI_IH_CEC_STAT0);
	writeb(0, cec->base + HDMI_CEC_LOCK);

	ret = request_irq(cec->irq, dw_hdmi_cec_irq, IRQF_SHARED,
			  DEV_NAME, cec);
	if (ret < 0)
		return ret;

	dw_hdmi_set_address(cec_dev, cec_dev->addresses);

	cec->ops->enable(cec->ops_data);

	irqs = CEC_STAT_ERROR_INIT | CEC_STAT_NACK | CEC_STAT_EOM |
	       CEC_STAT_DONE;
	writeb(irqs, cec->base + HDMI_CEC_POLARITY);
	writeb(~irqs, cec->base + HDMI_CEC_MASK);
	writeb(~irqs, cec->base + HDMI_IH_MUTE_CEC_STAT0);

	return 0;
}

static int dw_hdmi_cec_probe(struct platform_device *pdev)
{
	struct dw_hdmi_cec_data *data = dev_get_platdata(&pdev->dev);
	struct dw_hdmi_cec *cec;

	if (!data)
		return -ENXIO;

	cec = devm_kzalloc(&pdev->dev, sizeof(*cec), GFP_KERNEL);
	if (!cec)
		return -ENOMEM;

	cec->dev = &pdev->dev;
	cec->base = data->base;
	cec->irq = data->irq;
	cec->ops = data->ops;
	cec->ops_data = data->ops_data;
	cec->cec.open = dw_hdmi_cec_open;
	cec->cec.release = dw_hdmi_cec_release;
	cec->cec.send_message = dw_hdmi_send_message;
	cec->cec.set_address = dw_hdmi_set_address;

	cec_dev_init(&cec->cec, THIS_MODULE);

	/* FIXME: soft-reset the CEC interface */

	dw_hdmi_set_address(&cec->cec, cec->cec.addresses);
	writeb(0, cec->base + HDMI_CEC_TX_CNT);
	writeb(~0, cec->base + HDMI_CEC_MASK);
	writeb(~0, cec->base + HDMI_IH_MUTE_CEC_STAT0);
	writeb(0, cec->base + HDMI_CEC_POLARITY);

	platform_set_drvdata(pdev, cec);

	/*
	 * Our device is just a convenience - we want to link to the real
	 * hardware device here, so that userspace can see the association
	 * between the HDMI hardware and its associated CEC chardev.
	 */
	return cec_dev_add(&cec->cec, cec->dev->parent, DEV_NAME);
}

static int dw_hdmi_cec_remove(struct platform_device *pdev)
{
	struct dw_hdmi_cec *cec = platform_get_drvdata(pdev);

	cec_dev_remove(&cec->cec);

	return 0;
}

static struct platform_driver dw_hdmi_cec_driver = {
	.probe	= dw_hdmi_cec_probe,
	.remove	= dw_hdmi_cec_remove,
	.driver = {
		.name = "dw-hdmi-cec",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(dw_hdmi_cec_driver);

MODULE_AUTHOR("Russell King <rmk+kernel@arm.linux.org.uk>");
MODULE_DESCRIPTION("Synopsis Designware HDMI CEC driver for i.MX");
MODULE_LICENSE("GPL");
MODULE_ALIAS(PLATFORM_MODULE_PREFIX "dw-hdmi-cec");
