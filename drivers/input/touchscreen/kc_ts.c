/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#include <linux/kc_ts.h>

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
#define KC_SYSFS_NOTICE		'n'
/* pixart command list */

/* Global Variables */
unsigned int ts_event_report = 1;
EXPORT_SYMBOL_GPL(ts_event_report);
unsigned int ts_log_level = 0;
EXPORT_SYMBOL_GPL(ts_log_level);
unsigned int ts_esd_recovery = 1;
EXPORT_SYMBOL_GPL(ts_esd_recovery);
unsigned int ts_config_switching = 1;
EXPORT_SYMBOL_GPL(ts_config_switching);

static int kc_ts_open(struct inode *inode, struct file *file)
{
	struct kc_ts_data *kd =
		container_of(inode->i_cdev, struct kc_ts_data, device_cdev);
	KC_TS_DEV_DBG("%s() is called.\n", __func__);

	file->private_data = kd;
	return 0;
};

static int kc_ts_release(struct inode *inode, struct file *file)
{
	KC_TS_DEV_DBG("%s() is called.\n", __func__);
	file->private_data = NULL;
	return 0;
};

static long kc_ts_diag_data_start(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	mutex_lock(&kd->lock);
	if (kd->diag_data == NULL) {
		kd->diag_data = kzalloc(sizeof(struct ts_diag_type), GFP_KERNEL);
		if (!kd->diag_data) {
			mutex_unlock(&kd->lock);
			dev_err(kd->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}
	}
	mutex_unlock(&kd->lock);

	return 0;
}

static long kc_ts_diag_data_end(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	mutex_lock(&kd->lock);
	if (kd->diag_data != NULL) {
		kfree(kd->diag_data);
		kd->diag_data = NULL;
		KC_TS_DEV_DBG("%s ts->diag_data = NULL\n", __func__);
	}
	mutex_unlock(&kd->lock);
	return 0;
}

static long kc_ts_frame_data_alloc(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (kd->frame_data == NULL) {
		kd->frame_data = kzalloc(sizeof(struct frame_buffer_read_data), GFP_KERNEL);
		if (!kd->frame_data) {
			dev_err(kd->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static long kc_ts_frame_data_free(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	if (kd->frame_data != NULL) {
		kfree(kd->frame_data);
		kd->frame_data = NULL;
		KC_TS_DEV_DBG("%s kd->frame_data = NULL\n", __func__);
	}
	return 0;
}
static long kc_ts_test_result_alloc(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (kd->open_short_result == NULL) {
		kd->open_short_result = kzalloc(sizeof(*kd->open_short_result), GFP_KERNEL);
		if (!kd->open_short_result) {
			dev_err(kd->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}
	}

	return 0;
}
static long kc_ts_test_result_free(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	if (kd->open_short_result != NULL) {
		kfree(kd->open_short_result);
		kd->open_short_result = NULL;
		KC_TS_DEV_DBG("%s ts->diag_data = NULL\n", __func__);
	}
	return 0;
}
static long kc_ts_press_diff_test_result_alloc(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (kd->press_diff_test_data == NULL) {
		kd->press_diff_test_data = kzalloc(sizeof(*kd->press_diff_test_data), GFP_KERNEL);
		if (!kd->press_diff_test_data) {
			dev_err(kd->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}
	}
	return 0;
}
static long kc_ts_press_diff_test_result_free(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	if (kd->press_diff_test_data != NULL) {
		kfree(kd->press_diff_test_data);
		kd->press_diff_test_data = NULL;
		KC_TS_DEV_DBG("%s kd->press_diff_test_data = NULL\n", __func__);
	}
	return 0;
}
static long kc_ts_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct kc_ts_data *kd = (struct kc_ts_data *)file->private_data;
	struct device *dev = kd->dev;
	long err = 0;
	int ret;
	u8 *system_info = NULL;

	KC_TS_DEV_DBG("%s() is called.\n", __func__);

	if (!kd) {
		dev_err(dev, "%s: kc_ts data is not set.\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case IOCTL_DIAG_START:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_START\n", __func__);
		err = kc_ts_diag_data_start(kd);
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
		mutex_lock(&kd->lock);
		if (kd->diag_data) {
			if (kd->tops->get_touch_info) {
				kd->tops->get_touch_info(kd);
			}
			err = copy_to_user((void __user *)arg, kd->diag_data,
						sizeof(struct ts_diag_type));
			if (err)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
		} else {
			dev_err(dev, "Touchscreen Diag not active!\n");
		}
		mutex_unlock(&kd->lock);
		break;

	case IOCTL_DIAG_END:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_END\n", __func__);
		err = kc_ts_diag_data_end(kd);
		break;

	case IOCTL_DIAG_SHORT_TEST:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_SHORT_TEST\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		if (kd->tops->short_test) {
			err = kd->tops->short_test(kd);
		}
		ret = copy_to_user((void __user *)arg, &kd->test_result,
					sizeof(kd->test_result));
		if (ret)
			dev_err(dev, "%s: copy_to_user error\n", __func__);
		mutex_unlock(&kd->lock);
		break;
	case IOCTL_DIAG_OPEN_TEST:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_OPEN_TEST\n", __func__);
		mutex_lock(&kd->lock);
		if (kd->tops->open_test) {
			err = kd->tops->open_test(kd);
		}
		mutex_unlock(&kd->lock);
		break;
	case IOCTL_DIAG_OPEN_SHORT_TEST:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_SHORT_TEST\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		err = kc_ts_frame_data_alloc(kd);
		if (!err) {
			err = kc_ts_test_result_alloc(kd);
			if (!err) {
				if (kd->tops->open_short_test) {
					memcpy(kd->open_short_result->req_param.req_param,
							((struct ts_test_result *)arg)->req_param.req_param,
							TS_DIAG_TEST_REQ_NUM);
					err = kd->tops->open_short_test(kd);
				}
				ret = copy_to_user((void __user *)arg, kd->open_short_result,
							sizeof(*kd->open_short_result));
				if (ret)
					dev_err(dev, "%s: copy_to_user error\n", __func__);
				kc_ts_test_result_free(kd);
			}
			kc_ts_frame_data_free(kd);
		}
		mutex_unlock(&kd->lock);
		break;
	case IOCTL_DIAG_GET_RAW_DATA:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_GET_RAW_DATA\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		err = kc_ts_frame_data_alloc(kd);
		if(!err){
			if (kd->tops->get_raw_data) {
				err = kd->tops->get_raw_data(kd);
			}
			ret = copy_to_user((void __user *)arg, kd->frame_data,
						sizeof(struct frame_buffer_read_data));
			if (ret)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
			kc_ts_frame_data_free(kd);
		}
		mutex_unlock(&kd->lock);
		break;

	case IOCTL_DIAG_GET_RAW_DATA_NEW:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_GET_RAW_DATA_NEW\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		err = kc_ts_frame_data_alloc(kd);
		if(!err){
			if (kd->tops->get_raw_data_new) {
				err = kd->tops->get_raw_data_new(kd);
			}
			ret = copy_to_user((void __user *)arg, kd->frame_data,
						sizeof(struct frame_buffer_read_data));
			if (ret)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
			kc_ts_frame_data_free(kd);
		}
		mutex_unlock(&kd->lock);
		break;

	case IOCTL_DIAG_GET_TOUCH_DELTA_DATA:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_GET_TOUCH_DELTA_DATA\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		err = kc_ts_frame_data_alloc(kd);
		if(!err){
			if (kd->tops->get_touch_delta_data) {
				err = kd->tops->get_touch_delta_data(kd);
			}
			ret = copy_to_user((void __user *)arg, kd->frame_data,
						sizeof(struct frame_buffer_read_data));
			if (ret)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
			kc_ts_frame_data_free(kd);
		}
		mutex_unlock(&kd->lock);
		break;

	case IOCTL_DIAG_MODE_CHECK:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_MODE_CHECK\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		if (kd->tops->mode_check) {
			ret = kd->tops->mode_check(kd);
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

		if(!system_info)
			system_info = kcalloc(512, sizeof(u8), GFP_KERNEL);
		else
			pr_err("%s: system_info has been allocated.\n", __func__);

		if (kd->tops->get_system_info) {
			err = kd->tops->get_system_info(kd, system_info);
			if (err){
				pr_err("%s: IC is suspend or Fail.\n", __func__);
				kfree(system_info);
				break;
			}
			err = copy_to_user((void __user *)arg, system_info, 512);
			if (err)
				dev_err(dev, "%s: copy_to_user error\n", __func__);
		}
		kfree(system_info);
		break;
	case IOCTL_SET_CONF_STAT:
		KC_TS_DEV_DBG("%s: IOCTL_SET_CONF_STAT\n", __func__);
		break;
	case IOCTL_SET_USER_DATA:
		KC_TS_DEV_DBG("%s: IOCTL_SET_USER_DATA\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		if (kd->tops->set_user_data) {
			err = kd->tops->set_user_data(kd, *((char *)arg));
		}
		mutex_unlock(&kd->lock);
		break;
	case IOCTL_DIAG_PRESS_DIFF_TEST:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_PRESS_DIFF_TEST\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&kd->lock);
		err = kc_ts_frame_data_alloc(kd);
		if (!err) {
			err = kc_ts_press_diff_test_result_alloc(kd);
			if (!err) {
				if (kd->tops->press_diff_test) {
					kd->press_diff_test_data->param =
							((struct ts_press_diff_test_data *)arg)->param;
					kd->press_diff_test_data->judgment_value =
							((struct ts_press_diff_test_data *)arg)->judgment_value;
					err = kd->tops->press_diff_test(kd);
				}
				ret = copy_to_user((void __user *)arg, kd->press_diff_test_data,
							sizeof(*kd->press_diff_test_data));
				if (ret)
					dev_err(dev, "%s: copy_to_user error\n", __func__);
				kc_ts_press_diff_test_result_free(kd);
			}
			kc_ts_frame_data_free(kd);
		}
		mutex_unlock(&kd->lock);
		break;
	default:
		dev_err(dev, "%s: cmd error[%X]\n", __func__, cmd);
		return -EINVAL;
		break;
	}
done:
	return err;
}

int ts_ctrl_init(struct kc_ts_data *kd, const struct file_operations *fops)
{
	struct cdev *device_cdev = &kd->device_cdev;
	int device_major = kd->device_major;
	struct class *device_class = kd->device_class;

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

int ts_ctrl_exit(struct kc_ts_data *kd)
{
	struct cdev *device_cdev = &kd->device_cdev;
	int device_major = kd->device_major;
	struct class *device_class = kd->device_class;
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
	case KC_SYSFS_NOTICE:
		KC_TS_DEV_DBG("%s: KC_SYSFS_NOTICE\n", __func__);

		sscanf(buf, "%c %x", &sdata.command,
					(unsigned int *) &ts_event_report);

		KC_TS_DEV_DBG("%s: ts_event_report is set to %x\n",
						__func__, ts_event_report);
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
		break;
	case KC_SYSFS_NOTICE:
		KC_TS_DEV_DBG("%s: KC_SYSFS_NOTICE\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count,
				   "event control is [%d]\n",
				   ts_event_report);
	default:
		break;
	}
	return count;
}

static DEVICE_ATTR(ctrl, S_IRUSR|S_IWUSR, kc_ctrl_show, kc_ctrl_store);

static struct attribute *kc_attrs[] = {
	&dev_attr_ctrl.attr,
	NULL
};

static const struct attribute_group kc_attr_group = {
	.attrs = kc_attrs,
};

int kc_ts_probe(struct kc_ts_data *kd)
{
	int ret = 0;

	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (!kd) {
		pr_err("%s: kc_ts data is not set.\n", __func__);
		return -EINVAL;
	}

	if (!kd->dev) {
		pr_err("%s: dev is not set.\n", __func__);
		return -EINVAL;
	}

	if (!kd->tops) {
		pr_err("%s: tops is not set.\n", __func__);
		return -EINVAL;
	}

	mutex_init(&kd->lock);

	/* Create cdev file ts_ctrl */
	ret = ts_ctrl_init(kd, &kc_ts_fops);
	if (ret) {
		pr_err("%s: Fail to create cdev.\n", __func__);
		goto err_cdev;
	}

	/* Create sysfs */
	ret = sysfs_create_group(&kd->dev->kobj, &kc_attr_group);
	if (ret) {
		pr_err("%s: Fail to create sysfs.\n", __func__);
		goto err_sysfs;
	}

	KC_TS_DEV_DBG("%s is completed.\n", __func__);

	return ret;

err_sysfs:
	ts_ctrl_exit(kd);
err_cdev:
	mutex_destroy(&kd->lock);

	return ret;
}
EXPORT_SYMBOL(kc_ts_probe);

void kc_ts_remove(struct kc_ts_data *kd)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);

	if (!kd || !kd->dev) {
		pr_err("%s: kc_ts data is not set.\n", __func__);
		return;
	}

	mutex_destroy(&kd->lock);
	sysfs_remove_group(&kd->dev->kobj, &kc_attr_group);
	ts_ctrl_exit(kd);

	KC_TS_DEV_DBG("%s is completed.\n", __func__);

	return;

}
EXPORT_SYMBOL(kc_ts_remove);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("kc touchscreen driver");
MODULE_LICENSE("GPL");

