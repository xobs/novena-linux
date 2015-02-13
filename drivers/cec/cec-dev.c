/*
 * HDMI Consumer Electronics Control
 *
 * This provides the user API for communication with HDMI CEC complaint
 * devices in kernel drivers, and is based upon the protocol developed
 * by Freescale for their i.MX SoCs.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/cec-dev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>

struct cec_event {
	struct cec_user_event usr;
	struct list_head node;
};

static struct class *cec_class;
static int cec_major;

static void cec_dev_send_message(struct cec_dev *cec_dev, u8 *msg,
	size_t count)
{
	unsigned long flags;

	spin_lock_irqsave(&cec_dev->lock, flags);
	cec_dev->retries = 5;
	cec_dev->write_busy = 1;
	cec_dev->send_message(cec_dev, msg, count);
	spin_unlock_irqrestore(&cec_dev->lock, flags);
}

void cec_dev_event(struct cec_dev *cec_dev, int type, u8 *msg, size_t len)
{
	struct cec_event *event;
	unsigned long flags;

	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (event) {
		event->usr.event_type = type;
		event->usr.msg_len = len;
		if (msg)
			memcpy(event->usr.msg, msg, len);

		spin_lock_irqsave(&cec_dev->lock, flags);
		list_add_tail(&event->node, &cec_dev->events);
		spin_unlock_irqrestore(&cec_dev->lock, flags);
		wake_up(&cec_dev->waitq);
	}
}
EXPORT_SYMBOL_GPL(cec_dev_event);

static int cec_dev_lock_write(struct cec_dev *cec_dev, struct file *file)
	__acquires(cec_dev->mutex)
{
	int ret;

	do {
		if (file->f_flags & O_NONBLOCK) {
			if (cec_dev->write_busy)
				return -EAGAIN;
		} else {
			ret = wait_event_interruptible(cec_dev->waitq,
						       !cec_dev->write_busy);
			if (ret)
				break;
		}

		ret = mutex_lock_interruptible(&cec_dev->mutex);
		if (ret)
			break;

		if (!cec_dev->write_busy)
			break;

		mutex_unlock(&cec_dev->mutex);
	} while (1);

	return ret;
}

static ssize_t cec_dev_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct cec_dev *cec_dev = file->private_data;
	ssize_t ret;

	if (count > sizeof(struct cec_user_event))
		count = sizeof(struct cec_user_event);

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	do {
		struct cec_event *event = NULL;
		unsigned long flags;

		spin_lock_irqsave(&cec_dev->lock, flags);
		if (!list_empty(&cec_dev->events)) {
			event = list_first_entry(&cec_dev->events,
					struct cec_event, node);
			list_del(&event->node);
		}
		spin_unlock_irqrestore(&cec_dev->lock, flags);

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

		ret = wait_event_interruptible(cec_dev->waitq,
					       !list_empty(&cec_dev->events));
		if (ret)
			break;
	} while (1);

	return ret;
}

static ssize_t cec_dev_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	struct cec_dev *cec_dev = file->private_data;
	u8 msg[MAX_MESSAGE_LEN];
	int ret;

	if (count > sizeof(msg))
		return -E2BIG;

	if (copy_from_user(msg, buf, count))
		return -EFAULT;

	ret = cec_dev_lock_write(cec_dev, file);
	if (ret)
		return ret;

	cec_dev_send_message(cec_dev, msg, count);

	mutex_unlock(&cec_dev->mutex);

	return count;
}

static long cec_dev_ioctl(struct file *file, u_int cmd, unsigned long arg)
{
	struct cec_dev *cec_dev = file->private_data;
	int ret;

	switch (cmd) {
	case HDMICEC_IOC_O_SETLOGICALADDRESS:
	case HDMICEC_IOC_SETLOGICALADDRESS:
		if (arg > 15) {
			ret = -EINVAL;
			break;
		}

		ret = cec_dev_lock_write(cec_dev, file);
		if (ret == 0) {
			unsigned char msg[1];

			cec_dev->addresses = BIT(arg);
			cec_dev->set_address(cec_dev, cec_dev->addresses);

			/*
			 * Send a ping message with the source and destination
			 * set to our address; the result indicates whether
			 * unit has chosen our address simultaneously.
			 */
			msg[0] = arg << 4 | arg;
			cec_dev_send_message(cec_dev, msg, sizeof(msg));
			mutex_unlock(&cec_dev->mutex);
		}
		break;

	case HDMICEC_IOC_STARTDEVICE:
		ret = mutex_lock_interruptible(&cec_dev->mutex);
		if (ret == 0) {
			cec_dev->addresses = BIT(15);
			cec_dev->set_address(cec_dev, cec_dev->addresses);
			mutex_unlock(&cec_dev->mutex);
		}
		break;

	case HDMICEC_IOC_STOPDEVICE:
		ret = 0;
		break;

	case HDMICEC_IOC_GETPHYADDRESS:
		ret = put_user(cec_dev->physical, (u16 __user *)arg);
		ret = -ENOIOCTLCMD;
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static unsigned cec_dev_poll(struct file *file, poll_table *wait)
{
	struct cec_dev *cec_dev = file->private_data;
	unsigned mask = 0;

	poll_wait(file, &cec_dev->waitq, wait);

	if (cec_dev->write_busy == 0)
		mask |= POLLOUT | POLLWRNORM;
	if (!list_empty(&cec_dev->events))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int cec_dev_release(struct inode *inode, struct file *file)
{
	struct cec_dev *cec_dev = file->private_data;

	mutex_lock(&cec_dev->mutex);
	if (cec_dev->users >= 1)
		cec_dev->users -= 1;
	if (cec_dev->users == 0) {
		/*
		 * Wait for any write to complete before shutting down.
		 * A message should complete in a maximum of 2.75ms *
		 * 160 bits + 4.7ms, or 444.7ms.  Let's call that 500ms.
		 * If we time out, shutdown anyway.
		 */
		wait_event_timeout(cec_dev->waitq, !cec_dev->write_busy,
				   msecs_to_jiffies(500));

		cec_dev->release(cec_dev);

		while (!list_empty(&cec_dev->events)) {
			struct cec_event *event;

			event = list_first_entry(&cec_dev->events,
					struct cec_event, node);
			list_del(&event->node);
			kfree(event);
		}
	}
	mutex_unlock(&cec_dev->mutex);
	return 0;
}

static int cec_dev_open(struct inode *inode, struct file *file)
{
	struct cec_dev *cec_dev = container_of(inode->i_cdev, struct cec_dev,
					       cdev);
	int ret = 0;

	nonseekable_open(inode, file);

	file->private_data = cec_dev;

	ret = mutex_lock_interruptible(&cec_dev->mutex);
	if (ret)
		return ret;

	if (cec_dev->users++ == 0) {
		cec_dev->addresses = BIT(15);

		ret = cec_dev->open(cec_dev);
		if (ret < 0)
			cec_dev->users = 0;
	}
	mutex_unlock(&cec_dev->mutex);

	return ret;
}

static const struct file_operations hdmi_cec_fops = {
	.owner = THIS_MODULE,
	.read = cec_dev_read,
	.write = cec_dev_write,
	.open = cec_dev_open,
	.unlocked_ioctl = cec_dev_ioctl,
	.release = cec_dev_release,
	.poll = cec_dev_poll,
};

void cec_dev_init(struct cec_dev *cec_dev, struct module *module)
{
	cec_dev->devn = MKDEV(cec_major, 0);

	INIT_LIST_HEAD(&cec_dev->events);
	init_waitqueue_head(&cec_dev->waitq);
	spin_lock_init(&cec_dev->lock);
	mutex_init(&cec_dev->mutex);

	cec_dev->addresses = BIT(15);

	cdev_init(&cec_dev->cdev, &hdmi_cec_fops);
	cec_dev->cdev.owner = module;
}
EXPORT_SYMBOL_GPL(cec_dev_init);

int cec_dev_add(struct cec_dev *cec_dev, struct device *dev, const char *name)
{
	struct device *cd;
	int ret;

	ret = cdev_add(&cec_dev->cdev, cec_dev->devn, 1);
	if (ret < 0)
		goto err_cdev;

	cd = device_create(cec_class, dev, cec_dev->devn, NULL, name);
	if (IS_ERR(cd)) {
		ret = PTR_ERR(cd);
		dev_err(dev, "can't create device: %d\n", ret);
		goto err_dev;
	}

	return 0;

 err_dev:
	cdev_del(&cec_dev->cdev);
 err_cdev:
	return ret;
}
EXPORT_SYMBOL_GPL(cec_dev_add);

void cec_dev_remove(struct cec_dev *cec_dev)
{
	device_destroy(cec_class, cec_dev->devn);
	cdev_del(&cec_dev->cdev);
}
EXPORT_SYMBOL_GPL(cec_dev_remove);

static int cec_init(void)
{
	dev_t dev;
	int ret;

	cec_class = class_create(THIS_MODULE, "hdmi-cec");
	if (IS_ERR(cec_class)) {
		ret = PTR_ERR(cec_class);
		pr_err("cec: can't create cec class: %d\n", ret);
		goto err_class;
	}

	ret = alloc_chrdev_region(&dev, 0, 1, "hdmi-cec");
	if (ret) {
		pr_err("cec: can't create character devices: %d\n", ret);
		goto err_chrdev;
	}

	cec_major = MAJOR(dev);

	return 0;

 err_chrdev:
	class_destroy(cec_class);
 err_class:
	return ret;
}
subsys_initcall(cec_init);

static void cec_exit(void)
{
	unregister_chrdev_region(MKDEV(cec_major, 0), 1);
	class_destroy(cec_class);
}
module_exit(cec_exit);

MODULE_AUTHOR("Russell King <rmk+kernel@arm.linux.org.uk>");
MODULE_DESCRIPTION("Generic HDMI CEC driver");
MODULE_LICENSE("GPL");
