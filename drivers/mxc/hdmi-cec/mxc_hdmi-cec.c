/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_hdmi-cec.c
 *
 * @brief HDMI CEC system initialization and file operation implementation
 *
 * @ingroup HDMI
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/sizes.h>

#include <linux/console.h>
#include <linux/types.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/pinctrl/consumer.h>

#include <video/mxc_hdmi.h>

#include "mxc_hdmi-cec.h"

struct hdmi_cec_priv {
	int  receive_error;
	int  send_error;
	u8 Logical_address;
	bool cec_state;
	bool write_busy;
	u8 last_msg[MAX_MESSAGE_LEN];
	u8 msg_len;
	u8 latest_cec_stat;
	spinlock_t irq_lock;
	struct delayed_work hdmi_cec_work;
	struct mutex lock;
};

struct hdmi_cec_event {
	int event_type;
	int msg_len;
	u8 msg[MAX_MESSAGE_LEN];
	struct list_head list;
};

static LIST_HEAD(head);

static int hdmi_cec_major;
static struct class *hdmi_cec_class;
static struct hdmi_cec_priv hdmi_cec_data;
static u8 open_count;

static wait_queue_head_t hdmi_cec_queue;
static irqreturn_t mxc_hdmi_cec_isr(int irq, void *data)
{
	struct hdmi_cec_priv *hdmi_cec = data;
	u8 cec_stat = 0;
	unsigned long flags;
	irqreturn_t ret = IRQ_HANDLED;

	spin_lock_irqsave(&hdmi_cec->irq_lock, flags);

	hdmi_writeb(0x7f, HDMI_IH_MUTE_CEC_STAT0);

	cec_stat = hdmi_readb(HDMI_IH_CEC_STAT0);
	hdmi_writeb(cec_stat, HDMI_IH_CEC_STAT0);

	if ((cec_stat & (HDMI_IH_CEC_STAT0_ERROR_INIT | \
		HDMI_IH_CEC_STAT0_NACK | HDMI_IH_CEC_STAT0_EOM | \
		HDMI_IH_CEC_STAT0_DONE)) == 0) {

		hdmi_cec->latest_cec_stat = 0;
		ret = IRQ_NONE;
	} else {
		pr_debug("HDMI CEC interrupt received\n");
		hdmi_cec->latest_cec_stat = cec_stat;
	}

	schedule_delayed_work(&(hdmi_cec->hdmi_cec_work), msecs_to_jiffies(20));
	spin_unlock_irqrestore(&hdmi_cec->irq_lock, flags);

	return ret;
}

void mxc_hdmi_cec_handle(u16 cec_stat)
{
	u8 val = 0, i = 0;
	struct hdmi_cec_event *event = NULL;

	pr_debug("function: %s \n", __func__);
	if (!open_count)
		return;

	/* The current transmission is successful (for initiator only). */
	/* A frame is not acknowledged in a directly addressed message.
	 * Or a frame is negatively acknowledged in
	 * a broadcast message (for initiator only).
	 */
	if (cec_stat & (HDMI_IH_CEC_STAT0_DONE | HDMI_IH_CEC_STAT0_NACK)) {

		event = vmalloc(sizeof(struct hdmi_cec_event));
		if (NULL == event) {
			pr_err("%s: Not enough memory!\n", __func__);
			return;
		}
		memset(event, 0, sizeof(struct hdmi_cec_event));

		event->msg_len = min((int)hdmi_cec_data.msg_len, 2);
		for (i = 0; i < event->msg_len; i++)
			event->msg[i] = hdmi_cec_data.last_msg[i];

		event->event_type = (cec_stat & HDMI_IH_CEC_STAT0_DONE) ? 
			MESSAGE_TYPE_SEND_SUCCESS : MESSAGE_TYPE_NOACK;

		mutex_lock(&hdmi_cec_data.lock);
		list_add_tail(&event->list, &head);
		mutex_unlock(&hdmi_cec_data.lock);

		wake_up(&hdmi_cec_queue);
	}

	/* EOM is detected so that the received data is ready
	 * in the receiver data buffer
	 */
	if (cec_stat & HDMI_IH_CEC_STAT0_EOM) {

		hdmi_writeb(0x02, HDMI_IH_CEC_STAT0);

		event = vmalloc(sizeof(struct hdmi_cec_event));
		if (NULL == event) {
			pr_err("%s: Not enough memory!\n", __func__);
			return;
		}
		memset(event, 0, sizeof(struct hdmi_cec_event));

		event->msg_len = hdmi_readb(HDMI_CEC_RX_CNT);
		if (!event->msg_len) {
			pr_err("%s: Invalid CEC message length!\n", __func__);
			return;
		}
		event->event_type = MESSAGE_TYPE_RECEIVE_SUCCESS;

		for (i = 0; i < event->msg_len; i++)
			event->msg[i] = hdmi_readb(HDMI_CEC_RX_DATA0+i);
		hdmi_writeb(0x0, HDMI_CEC_LOCK);

		mutex_lock(&hdmi_cec_data.lock);
		list_add_tail(&event->list, &head);
		mutex_unlock(&hdmi_cec_data.lock);

		wake_up(&hdmi_cec_queue);
	}

	/* An error is detected on cec line (for initiator only). */
	if (cec_stat & HDMI_IH_CEC_STAT0_ERROR_INIT) {

		mutex_lock(&hdmi_cec_data.lock);
		hdmi_cec_data.send_error++;
		if (hdmi_cec_data.send_error > 5) {
			pr_err("%s:Re-transmission is attempted more than 5 times!\n",
					__func__);
			hdmi_cec_data.send_error = 0;
			mutex_unlock(&hdmi_cec_data.lock);
			return;
		}

		for (i = 0; i < hdmi_cec_data.msg_len; i++) {
			hdmi_writeb(hdmi_cec_data.last_msg[i],
						HDMI_CEC_TX_DATA0 + i);
		}
		hdmi_writeb(hdmi_cec_data.msg_len, HDMI_CEC_TX_CNT);

		val = hdmi_readb(HDMI_CEC_CTRL);
		val |= 0x01;
		hdmi_writeb(val, HDMI_CEC_CTRL);
		mutex_unlock(&hdmi_cec_data.lock);
	}

	/* An error is notified by a follower.
	 * Abnormal logic data bit error (for follower).
	 */
	if (cec_stat & HDMI_IH_CEC_STAT0_ERROR_FOLL) {
		hdmi_cec_data.receive_error++;
	}

	/* HDMI cable connected */
	/* HDMI cable disconnected */
	if (cec_stat & (0x80 | 0x100)) {
		event = vmalloc(sizeof(struct hdmi_cec_event));
		if (NULL == event) {
			pr_err("%s: Not enough memory!\n", __func__);
			return;
		}
		memset(event, 0, sizeof(struct hdmi_cec_event));
		event->event_type = (cec_stat & 0x80) ? 
			MESSAGE_TYPE_CONNECTED : MESSAGE_TYPE_DISCONNECTED;

		mutex_lock(&hdmi_cec_data.lock);
		list_add_tail(&event->list, &head);
		mutex_unlock(&hdmi_cec_data.lock);

		wake_up(&hdmi_cec_queue);
	}

    return;
}
EXPORT_SYMBOL(mxc_hdmi_cec_handle);

static void mxc_hdmi_cec_worker(struct work_struct *work)
{
	u8 val;

	if (hdmi_cec_data.latest_cec_stat)
		mxc_hdmi_cec_handle(hdmi_cec_data.latest_cec_stat);

	val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL |
			HDMI_IH_CEC_STAT0_ARB_LOST;
	hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);
}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int hdmi_cec_open(struct inode *inode, struct file *filp)
{
	mutex_lock(&hdmi_cec_data.lock);
	if (open_count) {
		mutex_unlock(&hdmi_cec_data.lock);
		return -EBUSY;
	}

	open_count = 1;
	filp->private_data = (void *)(&hdmi_cec_data);
	hdmi_cec_data.Logical_address = 15;
	hdmi_cec_data.cec_state = false;
	mutex_unlock(&hdmi_cec_data.lock);

	return 0;
}

static ssize_t hdmi_cec_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	struct hdmi_cec_priv *hdmi_cec = file->private_data;
	int ret = 0;

	pr_debug("function : %s\n", __func__);
	if (!open_count)
		return -ENODEV;

	count = min(count, sizeof(struct hdmi_cec_event) - sizeof(struct list_head));

	do {
		unsigned long flags;    
		struct hdmi_cec_event *event = NULL;

		spin_lock_irqsave(&hdmi_cec->irq_lock, flags);
		if (!list_empty(&head)) {
			event = list_first_entry(&head, struct hdmi_cec_event, list);
			list_del(&event->list);
		}
		spin_unlock_irqrestore(&hdmi_cec->irq_lock, flags);
		wake_up(&hdmi_cec_queue);

		if (event) {
			ret = copy_to_user(buf, event, count) ? -EFAULT : count;
			vfree(event);
		}
		else if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
		}
		else if (wait_event_interruptible(hdmi_cec_queue, (!list_empty(&head)))) {
			ret = -ERESTARTSYS;
		}
	} while(!ret);

	return ret;
}

static ssize_t hdmi_cec_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct hdmi_cec_priv *hdmi_cec = file->private_data;
	int ret = 0 , i = 0;
	u8 msg[MAX_MESSAGE_LEN];
	u8 val = 0;

	pr_debug("function : %s\n", __func__);
	if (!open_count)
		return -ENODEV;

	if (count > MAX_MESSAGE_LEN)
		return -E2BIG;

	memset(&msg, 0, MAX_MESSAGE_LEN);
	if (copy_from_user(&msg, buf, count))
		return -EFAULT;

	do {
		if (file->f_flags & O_NONBLOCK) {
			if (hdmi_cec->write_busy)
				ret = -EAGAIN;
		} else if (wait_event_interruptible(hdmi_cec_queue, (!hdmi_cec->write_busy))) {
				ret = -ERESTARTSYS;
		}
		if (ret)
			break;
		
		mutex_lock(&hdmi_cec->lock);
		hdmi_cec->write_busy = true;

		hdmi_cec->send_error = 0;
		hdmi_writeb(count, HDMI_CEC_TX_CNT);
		for (i = 0; i < count; i++) {
			hdmi_writeb(msg[i], HDMI_CEC_TX_DATA0+i);
		}

		val = hdmi_readb(HDMI_CEC_CTRL);
		val |= 0x01;
		hdmi_writeb(val, HDMI_CEC_CTRL);
		memcpy(hdmi_cec->last_msg, msg, count);
		hdmi_cec->msg_len = count;

		mutex_unlock(&hdmi_cec->lock);

		if (!(wait_event_timeout(hdmi_cec_queue, (!(hdmi_readb(HDMI_CEC_CTRL) & 0x01)), msecs_to_jiffies(500)))) {
			hdmi_cec->msg_len = 0;
			ret = -EIO;
		} else {
			ret = count;
		}
	} while(!ret);

	hdmi_cec->write_busy = false;
	return ret;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static long hdmi_cec_ioctl(struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = 0;
	u8 val = 0, msg = 0;
	struct mxc_edid_cfg hdmi_edid_cfg;

	pr_debug("function : %s\n", __func__);
	if (!open_count)
		return -ENODEV;

	switch (cmd) {
	case HDMICEC_IOC_SETLOGICALADDRESS:
		mutex_lock(&hdmi_cec_data.lock);
		if (false == hdmi_cec_data.cec_state) {
			mutex_unlock(&hdmi_cec_data.lock);
			return -EACCES;
		}

		hdmi_cec_data.Logical_address = (u8)arg;

		if (hdmi_cec_data.Logical_address <= 7) {
			val = 1 << hdmi_cec_data.Logical_address;
			hdmi_writeb(val, HDMI_CEC_ADDR_L);
			hdmi_writeb(0, HDMI_CEC_ADDR_H);
		} else if (hdmi_cec_data.Logical_address <= 15) {
			val = 1 << (hdmi_cec_data.Logical_address - 8);
			hdmi_writeb(val, HDMI_CEC_ADDR_H);
			hdmi_writeb(0, HDMI_CEC_ADDR_L);
		} else {
			ret = -EINVAL;
		}

		/* Send Polling message with same source
		 * and destination address
		 */
		if (0 == ret && 15 != hdmi_cec_data.Logical_address) {
			msg = (hdmi_cec_data.Logical_address << 4) |
					hdmi_cec_data.Logical_address;
			hdmi_writeb(1, HDMI_CEC_TX_CNT);
			hdmi_writeb(msg, HDMI_CEC_TX_DATA0);

			val = hdmi_readb(HDMI_CEC_CTRL);
			val |= 0x01;
			hdmi_writeb(val, HDMI_CEC_CTRL);
		}
		mutex_unlock(&hdmi_cec_data.lock);
		break;

	case HDMICEC_IOC_STARTDEVICE:
		val = hdmi_readb(HDMI_MC_CLKDIS);
		val &= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;
		hdmi_writeb(val, HDMI_MC_CLKDIS);

		hdmi_writeb(0x02, HDMI_CEC_CTRL);

		val = HDMI_IH_CEC_STAT0_ERROR_INIT | HDMI_IH_CEC_STAT0_NACK |
			HDMI_IH_CEC_STAT0_EOM | HDMI_IH_CEC_STAT0_DONE;
		hdmi_writeb(val, HDMI_CEC_POLARITY);

		val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL |
			HDMI_IH_CEC_STAT0_ARB_LOST;
		hdmi_writeb(val, HDMI_CEC_MASK);
		hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);

		mutex_lock(&hdmi_cec_data.lock);
		hdmi_cec_data.cec_state = true;
		mutex_unlock(&hdmi_cec_data.lock);
		break;

	case HDMICEC_IOC_STOPDEVICE:
		mutex_lock(&hdmi_cec_data.lock);
		hdmi_cec_data.cec_state = false;
		mutex_unlock(&hdmi_cec_data.lock);
		hdmi_writeb(0x10, HDMI_CEC_CTRL);

		val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL |
			HDMI_IH_CEC_STAT0_ERROR_INIT | HDMI_IH_CEC_STAT0_ARB_LOST |
			HDMI_IH_CEC_STAT0_NACK | HDMI_IH_CEC_STAT0_EOM |
			HDMI_IH_CEC_STAT0_DONE;
		hdmi_writeb(val, HDMI_CEC_MASK);
		hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);

		hdmi_writeb(0x0, HDMI_CEC_POLARITY);

		val = hdmi_readb(HDMI_MC_CLKDIS);
		val |= HDMI_MC_CLKDIS_CECCLK_DISABLE;
		hdmi_writeb(val, HDMI_MC_CLKDIS);
		break;

	case HDMICEC_IOC_GETPHYADDRESS:
		hdmi_get_edid_cfg(&hdmi_edid_cfg);
		ret = copy_to_user((void __user *)arg,
					 &hdmi_edid_cfg.physical_address,
					 4*sizeof(u8))?-EFAULT:0;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
* @brief Release function for vpu file operation
* @return  0 on success or negative error code on error
*/
static int hdmi_cec_release(struct inode *inode, struct file *filp)
{
	struct hdmi_cec_priv *hdmi_cec = filp->private_data;

	pr_debug("function : %s\n", __func__);

	if (open_count) {
		if (hdmi_cec->cec_state)
			hdmi_cec_ioctl(filp, HDMICEC_IOC_STOPDEVICE, 0);

		mutex_lock(&hdmi_cec->lock);
		if (!(wait_event_timeout(hdmi_cec_queue, !hdmi_cec->write_busy, msecs_to_jiffies(500))))
			hdmi_cec->write_busy = false;

		open_count = 0;
		while (!list_empty(&head)) {
			struct hdmi_cec_event *event;

			event = list_first_entry(&head, struct hdmi_cec_event, list);
			list_del(&event->list);
			vfree(event);
		}
		mutex_unlock(&hdmi_cec->lock);
	}
	return 0;
}

static unsigned int hdmi_cec_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	pr_debug("function : %s\n", __func__);

	poll_wait(file, &hdmi_cec_queue, wait);

	mutex_lock(&hdmi_cec_data.lock);
	if (!hdmi_cec_data.write_busy) 
		mask = (POLLOUT | POLLWRNORM);
	if (!list_empty(&head))
		mask |= (POLLIN | POLLRDNORM);
	mutex_unlock(&hdmi_cec_data.lock);

	return mask;
}

const struct file_operations hdmi_cec_fops = {
	.owner = THIS_MODULE,
	.read = hdmi_cec_read,
	.write = hdmi_cec_write,
	.open = hdmi_cec_open,
	.unlocked_ioctl = hdmi_cec_ioctl,
	.release = hdmi_cec_release,
	.poll = hdmi_cec_poll,
};

static int hdmi_cec_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *temp_class;
	struct resource *res;
	struct pinctrl *pinctrl;
	int irq = platform_get_irq(pdev, 0);

	hdmi_cec_major = register_chrdev(hdmi_cec_major,
				"mxc_hdmi_cec", &hdmi_cec_fops);
	if (hdmi_cec_major < 0) {
		dev_err(&pdev->dev, "hdmi_cec: unable to get a major for HDMI CEC\n");
		err = -EBUSY;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(res == NULL)) {
		dev_err(&pdev->dev, "hdmi_cec:No HDMI irq line provided\n");
		goto err_out_chrdev;
	}

	spin_lock_init(&hdmi_cec_data.irq_lock);

	err = devm_request_irq(&pdev->dev, irq, mxc_hdmi_cec_isr, IRQF_SHARED,
			dev_name(&pdev->dev), &hdmi_cec_data);
	if (err < 0) {
		dev_err(&pdev->dev, "hdmi_cec:Unable to request irq: %d\n", err);
		goto err_out_chrdev;
	}

	hdmi_cec_class = class_create(THIS_MODULE, "mxc_hdmi_cec");
	if (IS_ERR(hdmi_cec_class)) {
		err = PTR_ERR(hdmi_cec_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(hdmi_cec_class, NULL,
			MKDEV(hdmi_cec_major, 0), NULL, "mxc_hdmi_cec");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select CEC pinctrl\n");
		goto err_out_class;
	}

	init_waitqueue_head(&hdmi_cec_queue);

	INIT_LIST_HEAD(&head);

	mutex_init(&hdmi_cec_data.lock);

	hdmi_cec_data.Logical_address = 15;

	platform_set_drvdata(pdev, &hdmi_cec_data);

	INIT_DELAYED_WORK(&hdmi_cec_data.hdmi_cec_work, mxc_hdmi_cec_worker);

	dev_info(&pdev->dev, "HDMI CEC initialized\n");
	goto out;

err_out_class:
	device_destroy(hdmi_cec_class, MKDEV(hdmi_cec_major, 0));
	class_destroy(hdmi_cec_class);
err_out_chrdev:
	unregister_chrdev(hdmi_cec_major, "mxc_hdmi_cec");
out:
	return err;
}

static int hdmi_cec_dev_remove(struct platform_device *pdev)
{
	if (open_count)
		hdmi_cec_release(0, 0);

	if (hdmi_cec_major > 0) {
		device_destroy(hdmi_cec_class, MKDEV(hdmi_cec_major, 0));
		class_destroy(hdmi_cec_class);
		unregister_chrdev(hdmi_cec_major, "mxc_hdmi_cec");
		hdmi_cec_major = 0;
	}
	return 0;
}

static const struct of_device_id imx_hdmi_cec_match[] = {
	{ .compatible = "fsl,imx6q-hdmi-cec", },
	{ .compatible = "fsl,imx6dl-hdmi-cec", },
	{ /* sentinel */ }
};

static struct platform_driver mxc_hdmi_cec_driver = {
	.probe = hdmi_cec_dev_probe,
	.remove = hdmi_cec_dev_remove,
	.driver = {
		.name = "mxc_hdmi_cec",
		.of_match_table	= imx_hdmi_cec_match,
	},
};

module_platform_driver(mxc_hdmi_cec_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux HDMI CEC driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxc_hdmi_cec");

