/* http://git.freescale.com/git/cgit.cgi/imx/linux-2.6-imx.git/tree/drivers/mxc/hdmi-cec/mxc_hdmi-cec.c?h=imx_3.0.35_4.1.0 */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "imx-hdmi.h"
#include "dw-hdmi-cec.h"

#define MAX_MESSAGE_LEN 16
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

static struct class *cec_class;
static int cec_major;

struct dw_hdmi_cec {
	struct device *dev;
	struct cdev cdev;
	void __iomem *base;
	const struct dw_hdmi_cec_ops *ops;
	void *ops_data;
	int irq;

	struct mutex mutex;
	unsigned users;

	spinlock_t lock;
	wait_queue_head_t waitq;
	struct list_head events;
	uint8_t write_busy;

	uint8_t	retries;
	uint16_t addresses;
	uint16_t physical;
};

enum {
	MESSAGE_TYPE_RECEIVE_SUCCESS = 1,
	MESSAGE_TYPE_NOACK,
	MESSAGE_TYPE_DISCONNECTED,
	MESSAGE_TYPE_CONNECTED,
	MESSAGE_TYPE_SEND_SUCCESS,
	MESSAGE_TYPE_SEND_ERROR,
};

enum {
	HDMICEC_IOC_MAGIC = 'H',
	/* This is wrong: we pass the argument as a number, not a pointer */
	HDMICEC_IOC_O_SETLOGICALADDRESS	= _IOW(HDMICEC_IOC_MAGIC, 1, unsigned char),
	HDMICEC_IOC_SETLOGICALADDRESS	= _IO(HDMICEC_IOC_MAGIC, 1),
	HDMICEC_IOC_STARTDEVICE		= _IO(HDMICEC_IOC_MAGIC, 2),
	HDMICEC_IOC_STOPDEVICE		= _IO(HDMICEC_IOC_MAGIC, 3),
	HDMICEC_IOC_GETPHYADDRESS	= _IOR(HDMICEC_IOC_MAGIC, 4, unsigned char[4]),
};

struct dw_hdmi_cec_user_event {
	uint32_t event_type;
	uint32_t msg_len;
	uint8_t msg[MAX_MESSAGE_LEN];
};

struct dw_hdmi_cec_event {
	struct dw_hdmi_cec_user_event usr;
	struct list_head node;
};

static void dw_hdmi_event(struct dw_hdmi_cec *cec, int type)
{
	struct dw_hdmi_cec_event *event;
	unsigned long flags;

	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (event) {
		event->usr.event_type = type;

		if (type == MESSAGE_TYPE_RECEIVE_SUCCESS) {
			unsigned i;

			event->usr.msg_len = readb(cec->base + HDMI_CEC_RX_CNT);

			for (i = 0; i < event->usr.msg_len; i++)
				event->usr.msg[i] = readb(cec->base + HDMI_CEC_RX_DATA0 + i);

			writeb(0, cec->base + HDMI_CEC_LOCK);
		}

		spin_lock_irqsave(&cec->lock, flags);
		list_add_tail(&event->node, &cec->events);
		spin_unlock_irqrestore(&cec->lock, flags);
		wake_up(&cec->waitq);
	}
}

static void dw_hdmi_set_address(struct dw_hdmi_cec *cec)
{
	writeb(cec->addresses & 255, cec->base + HDMI_CEC_ADDR_L);
	writeb(cec->addresses >> 8, cec->base + HDMI_CEC_ADDR_H);
}

static void dw_hdmi_send_message(struct dw_hdmi_cec *cec, uint8_t *msg,
	size_t count)
{
	unsigned long flags;
	unsigned i;

	for (i = 0; i < count; i++)
		writeb(msg[i], cec->base + HDMI_CEC_TX_DATA0 + i);

	writeb(count, cec->base + HDMI_CEC_TX_CNT);

	spin_lock_irqsave(&cec->lock, flags);
	cec->retries = 5;
	cec->write_busy = 1;
	writeb(CEC_CTRL_NORMAL | CEC_CTRL_START, cec->base + HDMI_CEC_CTRL);
	spin_unlock_irqrestore(&cec->lock, flags);
}

static int dw_hdmi_lock_write(struct dw_hdmi_cec *cec, struct file *file)
	__acquires(cec->mutex)
{
	int ret;

	do {
		if (file->f_flags & O_NONBLOCK) {
			if (cec->write_busy)
				return -EAGAIN;
		} else {
			ret = wait_event_interruptible(cec->waitq,
						       !cec->write_busy);
			if (ret)
				break;
		}

		ret = mutex_lock_interruptible(&cec->mutex);
		if (ret)
			break;

		if (!cec->write_busy)
			break;

		mutex_unlock(&cec->mutex);
	} while (1);

	return ret;
}

static irqreturn_t dw_hdmi_cec_irq(int irq, void *data)
{
	struct dw_hdmi_cec *cec = data;
	unsigned stat = readb(cec->base + HDMI_IH_CEC_STAT0);

	if (stat == 0)
		return IRQ_NONE;

	writeb(stat, cec->base + HDMI_IH_CEC_STAT0);

	if (stat & CEC_STAT_ERROR_INIT) {
		if (cec->retries) {
			unsigned v = readb(cec->base + HDMI_CEC_CTRL);
			writeb(v | CEC_CTRL_START, cec->base + HDMI_CEC_CTRL);
			cec->retries -= 1;
		} else {
			cec->write_busy = 0;
			dw_hdmi_event(cec, MESSAGE_TYPE_SEND_ERROR);
		}
	} else if (stat & (CEC_STAT_DONE | CEC_STAT_NACK)) {
		cec->retries = 0;
		cec->write_busy = 0;
		if (stat & CEC_STAT_DONE) {
			dw_hdmi_event(cec, MESSAGE_TYPE_SEND_SUCCESS);
		} else {
			dw_hdmi_event(cec, MESSAGE_TYPE_NOACK);
		}
	}

	if (stat & CEC_STAT_EOM)
		dw_hdmi_event(cec, MESSAGE_TYPE_RECEIVE_SUCCESS);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(dw_hdmi_cec_irq);

static ssize_t dw_hdmi_cec_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct dw_hdmi_cec *cec = file->private_data;
	ssize_t ret;

	if (count > sizeof(struct dw_hdmi_cec_user_event))
		count = sizeof(struct dw_hdmi_cec_user_event);

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	do {
		struct dw_hdmi_cec_event *event = NULL;
		unsigned long flags;

		spin_lock_irqsave(&cec->lock, flags);
		if (!list_empty(&cec->events)) {
			event = list_first_entry(&cec->events,
					struct dw_hdmi_cec_event, node);
			list_del(&event->node);
		}
		spin_unlock_irqrestore(&cec->lock, flags);

		if (event) {
			ret = __copy_to_user(buf, &event->usr, count) ?
				 -EFAULT : count;
			kfree(event);
			break;
		}

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		ret = wait_event_interruptible(cec->waitq,
					       !list_empty(&cec->events));
		if (ret)
			break;
	} while (1);

	return ret;
}

static ssize_t dw_hdmi_cec_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	struct dw_hdmi_cec *cec = file->private_data;
	uint8_t msg[MAX_MESSAGE_LEN];
	int ret;

	if (count > sizeof(msg))
		return -E2BIG;

	if (copy_from_user(msg, buf, count))
		return -EFAULT;

	ret = dw_hdmi_lock_write(cec, file);
	if (ret)
		return ret;

	dw_hdmi_send_message(cec, msg, count);

	mutex_unlock(&cec->mutex);

	return count;
}

static long dw_hdmi_cec_ioctl(struct file *file, u_int cmd, unsigned long arg)
{
	struct dw_hdmi_cec *cec = file->private_data;
	int ret;

	switch (cmd) {
	case HDMICEC_IOC_O_SETLOGICALADDRESS:
	case HDMICEC_IOC_SETLOGICALADDRESS:
		if (arg > 15) {
			ret = -EINVAL;
			break;
		}

		ret = dw_hdmi_lock_write(cec, file);
		if (ret == 0) {
			unsigned char msg[1];

			cec->addresses = BIT(arg);
			dw_hdmi_set_address(cec);

			/*
			 * Send a ping message with the source and destination
			 * set to our address; the result indicates whether
			 * unit has chosen our address simultaneously.
			 */
			msg[0] = arg << 4 | arg;
			dw_hdmi_send_message(cec, msg, sizeof(msg));
			mutex_unlock(&cec->mutex);
		}
		break;

	case HDMICEC_IOC_STARTDEVICE:
		ret = mutex_lock_interruptible(&cec->mutex);
		if (ret == 0) {
			cec->addresses = BIT(15);
			dw_hdmi_set_address(cec);
			mutex_unlock(&cec->mutex);
		}
		break;

	case HDMICEC_IOC_STOPDEVICE:
		ret = 0;
		break;

	case HDMICEC_IOC_GETPHYADDRESS:
		ret = put_user(cec->physical, (uint16_t __user *)arg);
		ret = -ENOIOCTLCMD;
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static unsigned dw_hdmi_cec_poll(struct file *file, poll_table *wait)
{
	struct dw_hdmi_cec *cec = file->private_data;
	unsigned mask = 0;

	poll_wait(file, &cec->waitq, wait);

	if (cec->write_busy == 0)
		mask |= POLLOUT | POLLWRNORM;
	if (!list_empty(&cec->events))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int dw_hdmi_cec_open(struct inode *inode, struct file *file)
{
	struct dw_hdmi_cec *cec = container_of(inode->i_cdev,
					       struct dw_hdmi_cec, cdev);
	int ret = 0;

	nonseekable_open(inode, file);

	file->private_data = cec;

	ret = mutex_lock_interruptible(&cec->mutex);
	if (ret)
		return ret;

	if (cec->users++ == 0) {
		unsigned irqs;

		writeb(0, cec->base + HDMI_CEC_CTRL);
		writeb(~0, cec->base + HDMI_IH_CEC_STAT0);
		writeb(0, cec->base + HDMI_CEC_LOCK);

		ret = request_irq(cec->irq, dw_hdmi_cec_irq, IRQF_SHARED,
				  DEV_NAME, cec);
		if (ret < 0) {
			cec->users = 0;
			goto unlock;
		}

		cec->addresses = BIT(15);
		dw_hdmi_set_address(cec);

		cec->ops->enable(cec->ops_data);

		irqs = CEC_STAT_ERROR_INIT | CEC_STAT_NACK | CEC_STAT_EOM |
		       CEC_STAT_DONE;
		writeb(irqs, cec->base + HDMI_CEC_POLARITY);
		writeb(~irqs, cec->base + HDMI_CEC_MASK);
		writeb(~irqs, cec->base + HDMI_IH_MUTE_CEC_STAT0);
	}
 unlock:
	mutex_unlock(&cec->mutex);

	return ret;
}

static int dw_hdmi_cec_release(struct inode *inode, struct file *file)
{
	struct dw_hdmi_cec *cec = file->private_data;

	mutex_lock(&cec->mutex);
	if (cec->users >= 1)
		cec->users -= 1;
	if (cec->users == 0) {
		/*
		 * Wait for any write to complete before shutting down.
		 * A message should complete in a maximum of 2.75ms *
		 * 160 bits + 4.7ms, or 444.7ms.  Let's call that 500ms.
		 * If we time out, shutdown anyway.
		 */
		wait_event_timeout(cec->waitq, !cec->write_busy,
				   msecs_to_jiffies(500));

		writeb(~0, cec->base + HDMI_CEC_MASK);
		writeb(~0, cec->base + HDMI_IH_MUTE_CEC_STAT0);
		writeb(0, cec->base + HDMI_CEC_POLARITY);

		free_irq(cec->irq, cec);

		cec->ops->disable(cec->ops_data);

		while (!list_empty(&cec->events)) {
			struct dw_hdmi_cec_event *event;

			event = list_first_entry(&cec->events,
					struct dw_hdmi_cec_event, node);
			list_del(&event->node);
			kfree(event);
		}
	}
	mutex_unlock(&cec->mutex);
	return 0;
}

static const struct file_operations hdmi_cec_fops = {
	.owner = THIS_MODULE,
	.read = dw_hdmi_cec_read,
	.write = dw_hdmi_cec_write,
	.open = dw_hdmi_cec_open,
	.unlocked_ioctl = dw_hdmi_cec_ioctl,
	.release = dw_hdmi_cec_release,
	.poll = dw_hdmi_cec_poll,
};

static int dw_hdmi_cec_probe(struct platform_device *pdev)
{
	struct dw_hdmi_cec_data *data = dev_get_platdata(&pdev->dev);
	struct dw_hdmi_cec *cec;
	struct device *cd;
	dev_t devn = MKDEV(cec_major, 0);
	int ret;

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

	INIT_LIST_HEAD(&cec->events);
	init_waitqueue_head(&cec->waitq);
	spin_lock_init(&cec->lock);
	mutex_init(&cec->mutex);

	/* FIXME: soft-reset the CEC interface */

	cec->addresses = BIT(15);
	dw_hdmi_set_address(cec);
	writeb(0, cec->base + HDMI_CEC_TX_CNT);
	writeb(~0, cec->base + HDMI_CEC_MASK);
	writeb(~0, cec->base + HDMI_IH_MUTE_CEC_STAT0);
	writeb(0, cec->base + HDMI_CEC_POLARITY);

	cdev_init(&cec->cdev, &hdmi_cec_fops);
	cec->cdev.owner = THIS_MODULE;
	ret = cdev_add(&cec->cdev, devn, 1);
	if (ret < 0)
		goto err_cdev;

	cd = device_create(cec_class, cec->dev, devn, NULL, DEV_NAME);
	if (IS_ERR(cd)) {
		ret = PTR_ERR(cd);
		dev_err(cec->dev, "can't create device: %d\n", ret);
		goto err_dev;
	}

	return 0;

 err_dev:
	cdev_del(&cec->cdev);
 err_cdev:
	return ret;
}

static int dw_hdmi_cec_remove(struct platform_device *pdev)
{
	struct dw_hdmi_cec *cec = platform_get_drvdata(pdev);
	dev_t devn = MKDEV(cec_major, 0);

	device_destroy(cec_class, devn);
	cdev_del(&cec->cdev);

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

static int dw_hdmi_cec_init(void)
{
	dev_t dev;
	int ret;

	cec_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(cec_class)) {
		ret = PTR_ERR(cec_class);
		pr_err("cec: can't create cec class: %d\n", ret);
		goto err_class;
	}

	ret = alloc_chrdev_region(&dev, 0, 1, DEV_NAME);
	if (ret) {
		pr_err("cec: can't create character devices: %d\n", ret);
		goto err_chrdev;
	}

	cec_major = MAJOR(dev);

	ret = platform_driver_register(&dw_hdmi_cec_driver);
	if (ret)
		goto err_driver;

	return 0;

 err_driver:
	unregister_chrdev_region(MKDEV(cec_major, 0), 1);
 err_chrdev:
	class_destroy(cec_class);
 err_class:
	return ret;
}
module_init(dw_hdmi_cec_init);

static void dw_hdmi_cec_exit(void)
{
	platform_driver_unregister(&dw_hdmi_cec_driver);
	unregister_chrdev_region(MKDEV(cec_major, 0), 1);
	class_destroy(cec_class);
}
module_exit(dw_hdmi_cec_exit);

MODULE_AUTHOR("Russell King <rmk+kernel@arm.linux.org.uk>");
MODULE_DESCRIPTION("Synopsis Designware HDMI CEC driver for i.MX");
MODULE_LICENSE("GPL");
MODULE_ALIAS(PLATFORM_MODULE_PREFIX "dw-hdmi-cec");
