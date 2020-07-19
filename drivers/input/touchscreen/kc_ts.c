/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 *
 * drivers/input/touchscreen/kc_ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/namei.h>
#include <linux/mutex.h>
#include <linux/cdev.h>

#include "kc_ts.h"

struct sysfs_data_{
	char command;
	u16 start_addr;
	u16 size;
} sdata;

/* sysfs ctrl command list */
#define KC_SYSFS_KDBGLEVEL	'l'
#define KC_SYSFS_POLLING	'p'
#define KC_SYSFS_CONFIG		'c'
#define KC_SYSFS_STATUS		's'
/* pixart command list */

/* Global Variables */
unsigned int ts_event_control = 0;
EXPORT_SYMBOL_GPL(ts_event_control);
unsigned int ts_log_level = 0;
EXPORT_SYMBOL_GPL(ts_log_level);
unsigned int ts_esd_recovery = 0;
EXPORT_SYMBOL_GPL(ts_esd_recovery);
unsigned int ts_config_switching = 1;
EXPORT_SYMBOL_GPL(ts_config_switching);
unsigned int ts_error_status = 0;
EXPORT_SYMBOL_GPL(ts_error_status);

static int kc_ts_open(struct inode *inode, struct file *file)
{
	struct kc_ts_data *ts =
		container_of(inode->i_cdev, struct kc_ts_data, device_cdev);
	KC_TS_DEV_DBG("%s() is called.\n", __func__);

	file->private_data = ts;
	return 0;
};

static int kc_ts_release(struct inode *inode, struct file *file)
{
	KC_TS_DEV_DBG("%s() is called.\n", __func__);
	file->private_data = NULL;
	return 0;
};

static long kc_ts_diag_data_start(struct kc_ts_data *ts)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	mutex_lock(&ts->lock);
	if (ts->diag_data == NULL) {
		ts->diag_data = kzalloc(sizeof(struct ts_diag_type), GFP_KERNEL);
		if (!ts->diag_data) {
			mutex_unlock(&ts->lock);
			dev_err(ts->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}
	}
	mutex_unlock(&ts->lock);

	return 0;
}

static long kc_ts_diag_data_end(struct kc_ts_data *ts)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	mutex_lock(&ts->lock);
	if (ts->diag_data != NULL) {
		kfree(ts->diag_data);
		ts->diag_data = NULL;
		KC_TS_DEV_DBG("%s ts->diag_data = NULL\n", __func__);
	}
	mutex_unlock(&ts->lock);
	return 0;
}

static long kc_ts_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct kc_ts_data *ts = (struct kc_ts_data *)file->private_data;
	struct device *dev;
	long err = 0;
	int ret;
	u8 *system_info = NULL;
#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
	int switch_stat = 0;
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */

	KC_TS_DEV_DBG("%s() is called.\n", __func__);

	if (!ts) {
		dev_err(dev, "%s: kc_ts data is not set.\n", __func__);
		return -EINVAL;
	}

	dev = ts->dev;

	switch (cmd) {
	case IOCTL_DIAG_START:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_START\n", __func__);
		err = kc_ts_diag_data_start(ts);
		break;

	case IOCTL_MULTI_GET:
	case IOCTL_COODINATE_GET:
		KC_TS_DEV_DBG("%s: IOCTL_MULTI_GET\n", __func__);
		KC_TS_DEV_DBG("%s: IOCTL_COODINATE_GET\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&ts->lock);
 		if (ts->diag_data) {
			if (ts->tops->get_touch_info) {
				ts->tops->get_touch_info(ts);
			}
			err = copy_to_user((void __user *)arg, ts->diag_data,
						sizeof(struct ts_diag_type));
			if (err)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
		} else {
			dev_err(dev, "Touchscreen Diag not active!\n");
		}
		mutex_unlock(&ts->lock);
		break;

	case IOCTL_DIAG_END:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_END\n", __func__);
		err = kc_ts_diag_data_end(ts);
		break;
	case IOCTL_DIAG_MODE_CHECK:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_MODE_CHECK\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		if (ts->tops->mode_check) {
			ret = ts->tops->mode_check(ts);
			if(ret < 0){
				pr_err("%s: mode_check is fail\n",__func__);
				err = -1;
				break;
			}
			err = copy_to_user((void __user *)arg, &ret, sizeof(int));
			if (err)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
		}
		break;
	case IOCTL_DIAG_GET_SYSTEM_INFO:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_GET_SYSTEM_INFO\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		
		if(!system_info){
			system_info = kcalloc(512, sizeof(u8), GFP_KERNEL);
			if (!system_info) {
				dev_err(dev, "%s: Failed to allocate buffer\n", __func__);
				goto done;
			}
		}
		else
			pr_err("%s: system_info has been allocated.\n", __func__);

		if (ts->tops->get_system_info) {
			err = ts->tops->get_system_info(ts, system_info);
			if (err){
				pr_err("%s: IC is suspend or Fail.\n", __func__);
				if (system_info != NULL) {
					kfree(system_info);
				}
				break;
			}
			err = copy_to_user((void __user *)arg, system_info, 512);
			if (err)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
		}
		if (system_info != NULL) {
			kfree(system_info);
		}
		break;
#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
	case IOCTL_SET_SENSOR_SWITCH:
		KC_TS_DEV_DBG("%s: IOCTL_SET_SENSOR_SWITCH\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		if (copy_from_user(&switch_stat, (void __user *)arg, sizeof(int))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			goto done;
		}
		dev_dbg(dev, " %s: change sensor switch. [%d]\n",__func__,switch_stat);
		if (ts->tops->sensor_switch) {
			err = ts->tops->sensor_switch(ts, !!switch_stat);
			if (err)
				dev_err(dev, "%s: sensor_switch Fail.\n", __func__);
		}
		break;
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */
#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL
	case IOCTL_SET_TOUCHMODE:
		dev_dbg(dev, "%s: IOCTL_SET_TOUCHMODE\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		if (copy_from_user(&switch_stat, (void __user *)arg, sizeof(int))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			goto done;
		}
		dev_dbg(dev, " %s: TOUCHMODE[%d]\n",__func__,switch_stat);
		if (ts->tops->sensor_switch) {
			err = ts->tops->set_touchmode(ts, !!switch_stat);
			if (err)
				dev_err(dev, "%s: set_touchmode Fail.\n", __func__);
		}
		break;
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL */
	case IOCTL_GET_TOUCHEVENTCONTROL:
		KC_TS_DEV_DBG("%s: IOCTL_GET_TOUCHEVENTCONTROL\n", __func__);

		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = copy_to_user((void __user *)arg, &ts_event_control, sizeof(unsigned int));
		if (err)
			dev_err(dev, "%s: copy_to_user error\n", __func__);
		break;
	default:
		dev_err(dev, "%s: cmd error[%X]\n", __func__, cmd);
		return -EINVAL;
		break;
	}
done:
	return err;
}

int ts_ctrl_init(struct kc_ts_data *ts, const struct file_operations *fops)
{
	struct cdev *device_cdev = &ts->device_cdev;
	int device_major = ts->device_major;
	struct class *device_class = ts->device_class;

	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;
	int ret;

	ret = alloc_chrdev_region(&device_t, 0, 1, "ts_ctrl");
	if (ret)
		goto error;

	device_major = MAJOR(device_t);

	cdev_init(device_cdev, fops);
	device_cdev->owner = THIS_MODULE;
	device_cdev->ops = fops;
	ret = cdev_add(device_cdev, MKDEV(device_major, 0), 1);
	if (ret)
		goto err_unregister_chrdev;

	device_class = class_create(THIS_MODULE, "ts_ctrl");
	if (IS_ERR(device_class)) {
		ret = -1;
		goto err_cleanup_cdev;
	};

	class_dev_t = device_create(device_class, NULL,
		MKDEV(device_major, 0), NULL, "ts_ctrl");
	if (IS_ERR(class_dev_t)) {
		ret = -1;
		goto err_destroy_class;
	}

	return 0;

err_destroy_class:
	class_destroy(device_class);
err_cleanup_cdev:
	cdev_del(device_cdev);
err_unregister_chrdev:
	unregister_chrdev_region(device_t, 1);
error:
	return ret;
}
EXPORT_SYMBOL_GPL(ts_ctrl_init);

int ts_ctrl_exit(struct kc_ts_data *ts)
{
	struct cdev *device_cdev = &ts->device_cdev;
	int device_major = ts->device_major;
	struct class *device_class = ts->device_class;
	dev_t device_t = MKDEV(device_major, 0);

	if (device_class) {
		device_destroy(device_class, MKDEV(device_major, 0));
		class_destroy(device_class);
	}
	if (device_cdev) {
		cdev_del(device_cdev);
		unregister_chrdev_region(device_t, 1);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ts_ctrl_exit);

const struct file_operations kc_ts_fops = {
	.owner = THIS_MODULE,
	.open = kc_ts_open,
	.unlocked_ioctl = kc_ts_ioctl,
	.release = kc_ts_release,
};

static ssize_t kc_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	switch (buf[0]) {
	case KC_SYSFS_KDBGLEVEL:
		KC_TS_DEV_DBG("%s: KC_SYSFS_KDBGLEVEL\n", __func__);
		sscanf(buf, "%c %x", &sdata.command,
					(unsigned int *) &ts_log_level);
		KC_TS_DEV_DBG("%s: ts_log_level = %x\n", __func__, ts_log_level);
		break;
	case KC_SYSFS_POLLING:
		KC_TS_DEV_DBG("%s: KC_SYSFS_POLLING\n", __func__);
		sscanf(buf, "%c %x", &sdata.command,
					(unsigned int *) &ts_esd_recovery);
		KC_TS_DEV_DBG("ts_esd_recovery is set to %d\n",
							ts_esd_recovery);
		break;
	case KC_SYSFS_CONFIG:
		KC_TS_DEV_DBG("%s: KC_SYSFS_CONFIG\n", __func__);
		sscanf(buf, "%c %x", &sdata.command,
					(unsigned int *) &ts_config_switching);
		KC_TS_DEV_DBG("%s: ts_config_switching is set to %d\n", __func__,
							ts_config_switching);
		break;
	case KC_SYSFS_STATUS:
		KC_TS_DEV_DBG("%s: KC_SYSFS_STATUS\n", __func__);
		sscanf(buf, "%c", &sdata.command);
		break;
	default:
		break;
	}
	return count;
}

static ssize_t kc_ctrl_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int count = 0;

	switch (sdata.command) {
	case KC_SYSFS_KDBGLEVEL:
		KC_TS_DEV_DBG("%s: KC_SYSFS_KDBGLEVEL\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count,
				   "ts_log_level is [%d]\n",
				   ts_log_level);
		break;
	case KC_SYSFS_POLLING:
		KC_TS_DEV_DBG("%s: KC_SYSFS_POLLING\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count, "ts_esd_recovery is "
							"[%d]\n", ts_esd_recovery);
		break;
	case KC_SYSFS_CONFIG:
		KC_TS_DEV_DBG("%s: KC_SYSFS_CONFIG\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count,
				   "config switching is [%d]\n",
				   ts_config_switching);
		break;
	case KC_SYSFS_STATUS:
		/* T.B.D. */
		break;
	default:
		break;
	}
	return count;
}

static DEVICE_ATTR(ctrl, S_IRUGO|S_IWUSR, kc_ctrl_show, kc_ctrl_store);

static struct attribute *kc_attrs[] = {
	&dev_attr_ctrl.attr,
	NULL
};

static const struct attribute_group kc_attr_group = {
	.attrs = kc_attrs,
};

int kc_ts_probe(struct kc_ts_data *ts)
{
	int ret = 0;

	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (!ts) {
		pr_err("%s: kc_ts data is not set.\n", __func__);
		return -EINVAL;
	}

	if (!ts->dev) {
		pr_err("%s: dev is not set.\n", __func__);
		return -EINVAL;
	}

	if (!ts->tops) {
		pr_err("%s: tops is not set.\n", __func__);
		return -EINVAL;
	}

	mutex_init(&ts->lock);

	/* Create cdev file ts_ctrl */
	ret = ts_ctrl_init(ts, &kc_ts_fops);
	if (ret) {
		pr_err("%s: Fail to create cdev.\n", __func__);
		goto err_cdev;
	}

	/* Create sysfs */
	ret = sysfs_create_group(&ts->dev->kobj, &kc_attr_group);
	if (ret) {
		pr_err("%s: Fail to create sysfs.\n", __func__);
		goto err_sysfs;
	}

	KC_TS_DEV_DBG("%s is completed.\n", __func__);

	return ret;

err_sysfs:
	ts_ctrl_exit(ts);
err_cdev:
	mutex_destroy(&ts->lock);

	return ret;
}
EXPORT_SYMBOL(kc_ts_probe);

void kc_ts_remove(struct kc_ts_data *ts)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	if (!ts || !ts->dev) {
		pr_err("%s: kc_ts data is not set.\n", __func__);
		return;
	}

	mutex_destroy(&ts->lock);
	sysfs_remove_group(&ts->dev->kobj, &kc_attr_group);
	ts_ctrl_exit(ts);

	KC_TS_DEV_DBG("%s is completed.\n", __func__);

	return;

}
EXPORT_SYMBOL(kc_ts_remove);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("kc touchscreen driver");
MODULE_LICENSE("GPL");

