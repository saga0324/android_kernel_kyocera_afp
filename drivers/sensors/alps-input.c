/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/* drivers/input/misc/alps-input.c
 *
 * Input device driver for alps sensor
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef ALPS_IDEV_DEBUG
#define DEBUG 1
#endif

#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "alps_compass_io.h"
#include "alps-input.h"

static DEFINE_MUTEX(alps_lock);

#define ALPS_INPUT_DEVICE_NAME	"alps_compass"
#define ALPS_IDEV_LOG_TAG	"[ALPS_IDEV], "

#define EVENT_TYPE_ACCEL_X	REL_X
#define EVENT_TYPE_ACCEL_Y	REL_Y
#define EVENT_TYPE_ACCEL_Z	REL_Z

#define EVENT_TYPE_MAGV_X	REL_RX
#define EVENT_TYPE_MAGV_Y	REL_RY
#define EVENT_TYPE_MAGV_Z	REL_RZ

#define ALPS_POLL_INTERVAL	100	/* msecs */

/*driver data*/
struct alps_input_data {
	struct input_dev		*idev;
	struct hrtimer			timer;
	struct work_struct		work_data;
	struct workqueue_struct *wq;
	int flgM, flgA;
	int flgSuspend;
	int delay;
	int acc_data[3];
	int mag_data[3];
};

static struct alps_input_data *adev;

static int sw_c_data[3] = {0};
static int acc_imit_data[3] = {0};
static int acc_imit_onoff = 0;
static int acc_imit_data_toggle = 0;
/*--------------------------------------------------------------------------
 * I/O Control
 *--------------------------------------------------------------------------*/
static long alps_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = -1, tmpval;

	switch (cmd) {
	case ALPSIO_SET_MAGACTIVATE:
		ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
		if (ret) {
			dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
				"error: alps_ioctl(ALPSIO_SET_MAGACTIVATE)\n");
			return -EFAULT;
		}
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"alps_ioctl(ALPSIO_SET_MAGACTIVATE), flgM = %d\n",
			tmpval);
		mutex_lock(&alps_lock);
		hscdtd_activate(1, tmpval, adev->delay);
		if (!adev->flgM && !adev->flgA && tmpval)
			hrtimer_start(&adev->timer,
				ns_to_ktime((u64)adev->delay * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);

		adev->flgM = tmpval;

		if ((adev->flgM | adev->flgA) == 0)
				hrtimer_cancel(&adev->timer);

		mutex_unlock(&alps_lock);
		break;

	case ALPSIO_SET_ACCACTIVATE:
		ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
		if (ret) {
			dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
				"error: alps_ioctl(ALPSIO_SET_ACCACTIVATE)\n");
			return -EFAULT;
		}
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"alps_ioctl(ALPSIO_SET_ACCACTIVATE), flgA = %d\n",
			tmpval);
		mutex_lock(&alps_lock);
		accsns_activate(1, tmpval, adev->delay);
		if (!adev->flgM && !adev->flgA && tmpval)
			hrtimer_start(&adev->timer,
				ns_to_ktime((u64)adev->delay * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);

		adev->flgA = tmpval;

		if ((adev->flgM | adev->flgA) == 0)
				hrtimer_cancel(&adev->timer);

		mutex_unlock(&alps_lock);
		break;

	case ALPSIO_SET_DELAY:
		ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
		if (ret) {
			dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
				"error: alps_ioctl(ALPSIO_SET_DELAY)\n");
			return -EFAULT;
		}
		mutex_lock(&alps_lock);
		if (adev->flgM) {
			if (tmpval <= 15)
				tmpval = 10;
			else if (tmpval <= 45)
				tmpval = 20;
			else if (tmpval <= 140)
				tmpval = 70;
			else
				tmpval = 200;
		} else {
#if 0
			if (tmpval <  10)
				tmpval = 10;
			else if (tmpval > 200)
				tmpval = 200;
#else
            if (tmpval > 200)
                tmpval = 200;
#endif
		}
		adev->delay = tmpval;
		accsns_activate(1, adev->flgA, adev->delay);
		hscdtd_activate(1, adev->flgM, adev->delay);
		mutex_unlock(&alps_lock);
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"alps_ioctl(ALPSIO_SET_DELAY), delay = %d\n",
			adev->delay);
		break;

	case ALPSIO_ACT_SELF_TEST_A:
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"alps_ioctl(ALPSIO_ACT_SELF_TEST_A)\n");
		mutex_lock(&alps_lock);
		ret = hscdtd_self_test_A();
		mutex_unlock(&alps_lock);
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"Self test-A result : %d\n", ret);
		if (copy_to_user(argp, &ret, sizeof(ret))) {
			dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
				"error: alps_ioctl(ALPSIO_ACT_SELF_TEST_A)\n");
			return -EFAULT;
		}
		break;

	case ALPSIO_ACT_SELF_TEST_B:
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"alps_ioctl(ALPSIO_ACT_SELF_TEST_B)\n");
		mutex_lock(&alps_lock);
		ret = hscdtd_self_test_B();
		mutex_unlock(&alps_lock);
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"Self test-B result : %d\n", ret);
		if (copy_to_user(argp, &ret, sizeof(ret))) {
			dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
				"error: alps_ioctl(ALPSIO_ACT_SELF_TEST_B)\n");
			return -EFAULT;
		}
		break;

	case ALPSIO_GET_HWDATA:
	{
		int xyz[3];
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"alps_ioctl(ALPSIO_GET_HWDATA)\n");
		mutex_lock(&alps_lock);
		ret = hscdtd_get_hardware_data(xyz);
		mutex_unlock(&alps_lock);
		dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"get hw data, %d, %d, %d\n",
			xyz[0], xyz[1], xyz[2]);
		if (copy_to_user(argp, xyz, sizeof xyz)) {
			dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
				"error: alps_ioctl(ALPSIO_GET_HWDATA)\n");
			return -EFAULT;
		}
	}
	break;

	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations alps_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= alps_ioctl,
};

static struct miscdevice alps_ioctl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "alps_compass_io",
	.fops  = &alps_fops,
};


/*--------------------------------------------------------------------------
 * sysfs
 *--------------------------------------------------------------------------*/
static ssize_t alps_idev_self_test_A_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;

	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"%s\n", __func__);
	mutex_lock(&alps_lock);
	ret = hscdtd_self_test_A();
	mutex_unlock(&alps_lock);
	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"Self test-A result : %d\n", ret);

	return sprintf(buf, "%d\n", ret);
}

static ssize_t alps_idev_self_test_B_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;

	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"%s\n", __func__);
	mutex_lock(&alps_lock);
	ret = hscdtd_self_test_B();
	mutex_unlock(&alps_lock);
	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"Self test-B result : %d\n", ret);

	return sprintf(buf, "%d\n", ret);
}

static ssize_t alps_idev_get_hw_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int xyz[3], ret = -1;

	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"%s\n", __func__);
	mutex_lock(&alps_lock);
	ret = hscdtd_get_hardware_data(xyz);
	mutex_unlock(&alps_lock);
	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"get hw data, %d, %d, %d\n",
		xyz[0], xyz[1], xyz[2]);

	return sprintf(buf, "%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
}

static ssize_t alps_idev_sw_correction_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d\n", sw_c_data[0], sw_c_data[1], sw_c_data[2]);
}
static ssize_t alps_idev_sw_correction_data_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"%s\n", __func__);

	sscanf(buf, "%d %d %d", &sw_c_data[0], &sw_c_data[1], &sw_c_data[2]);

	return count;
}

static ssize_t alps_idev_delay_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", adev->delay);
}
static ssize_t alps_idev_delay_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    int tmpval;

    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "%s\n", __func__);

    sscanf(buf, "%d", &tmpval);

    mutex_lock(&alps_lock);
    if (adev->flgM) {
        if (tmpval <= 15)
            tmpval = 10;
        else if (tmpval <= 45)
            tmpval = 20;
        else if (tmpval <= 140)
            tmpval = 70;
        else
            tmpval = 200;
    } else {
#if 0
        if (tmpval <  10)
            tmpval = 10;
        else if (tmpval > 200)
            tmpval = 200;
#else
        if (tmpval > 200)
            tmpval = 200;
#endif
    }
    adev->delay = tmpval;
    accsns_activate(1, adev->flgA, adev->delay);
    hscdtd_activate(1, adev->flgM, adev->delay);
    mutex_unlock(&alps_lock);

    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
            "%s delay = %d\n",
            __func__, adev->delay);

	return count;
}


static ssize_t alps_idev_acc_enable_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "enable %d\n",adev->flgA);
    return sprintf(buf, "%d\n", adev->flgA);
}

static ssize_t alps_idev_acc_enable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    unsigned long enable = 0;

    ret = strict_strtoul(buf, 10, &enable);
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "enable %d\n",(int)enable);
    mutex_lock(&alps_lock);
    accsns_activate(1, (int)enable, adev->delay);
    if (!adev->flgM && !adev->flgA && enable)
        hrtimer_start(&adev->timer,
            ns_to_ktime((u64)adev->delay * NSEC_PER_MSEC),
            HRTIMER_MODE_REL);

    adev->flgA = (int)enable;

    if ((adev->flgM | adev->flgA) == 0)
        hrtimer_cancel(&adev->timer);

    mutex_unlock(&alps_lock);

    return count;
}

static ssize_t alps_idev_acc_data_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "data %d %d %d\n",adev->acc_data[0],adev->acc_data[1],adev->acc_data[2]);
    return sprintf(buf, "%d %d %d\n", adev->acc_data[0], adev->acc_data[1], adev->acc_data[2]);
}


static ssize_t alps_idev_mag_enable_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "enable %d\n",adev->flgM);
    return sprintf(buf, "%d\n", adev->flgM);
}

static ssize_t alps_idev_mag_enable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    unsigned long enable = 0;

    ret = strict_strtoul(buf, 10, &enable);
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "enable %d\n",(int)enable);
    mutex_lock(&alps_lock);
    hscdtd_activate(1, (int)enable, adev->delay);
    if (!adev->flgM && !adev->flgA && enable)
        hrtimer_start(&adev->timer,
            ns_to_ktime((u64)adev->delay * NSEC_PER_MSEC),
            HRTIMER_MODE_REL);

    adev->flgM = (int)enable;

    if ((adev->flgM | adev->flgA) == 0)
        hrtimer_cancel(&adev->timer);

    mutex_unlock(&alps_lock);

    return count;
}

static ssize_t alps_idev_mag_data_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "data %d %d %d\n",adev->mag_data[0],adev->mag_data[1],adev->mag_data[2]);
    return sprintf(buf, "%d %d %d\n", adev->mag_data[0], adev->mag_data[1], adev->mag_data[2]);
}

static ssize_t alps_idev_acc_imit_data_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    mutex_lock(&alps_lock);
    sscanf(buf, "%d %d %d %d", &acc_imit_onoff, &acc_imit_data[0], &acc_imit_data[1], &acc_imit_data[2]);
    dev_dbg(&adev->idev->dev, ALPS_IDEV_LOG_TAG
        "imit = %d, data = %d %d %d\n",acc_imit_onoff, acc_imit_data[0], acc_imit_data[1], acc_imit_data[2]);
    mutex_unlock(&alps_lock);

    return count;
}

static struct device_attribute attributes[] = {
	__ATTR(self_test_A, S_IRUGO,
		alps_idev_self_test_A_show, NULL),
	__ATTR(self_test_B, S_IRUGO,
		alps_idev_self_test_B_show, NULL),
	__ATTR(get_hw_data, S_IRUGO,
		alps_idev_get_hw_data_show, NULL),
	__ATTR(sw_correction_data, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		alps_idev_sw_correction_data_show, alps_idev_sw_correction_data_store),
	__ATTR(delay, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		alps_idev_delay_show, alps_idev_delay_store),
	__ATTR(acc_enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		alps_idev_acc_enable_show, alps_idev_acc_enable_store),
	__ATTR(acc_data, S_IRUSR|S_IRGRP,
		alps_idev_acc_data_show, NULL),
	__ATTR(mag_enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		alps_idev_mag_enable_show, alps_idev_mag_enable_store),
	__ATTR(mag_data, S_IRUSR|S_IRGRP,
		alps_idev_mag_data_show, NULL),
	__ATTR(imit_data, S_IWUSR|S_IWGRP,
		NULL, alps_idev_acc_imit_data_store)
};

static int alps_idev_create_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto out_sysfs;
	return 0;

out_sysfs:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "Unable to create interface\n");
	return -EIO;
}

static void alps_idev_remove_sysfs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}


/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------
 * input device
 *--------------------------------------------------------------------------*/
static void accsns_poll(struct input_dev *idev)
{
	int xyz[3];

	if (accsns_get_acceleration_data(xyz) == 0) {
        if( acc_imit_onoff ) {
            acc_imit_data_toggle ^= 1;
            xyz[0] = acc_imit_data[0] + acc_imit_data_toggle;
            xyz[1] = acc_imit_data[1] + acc_imit_data_toggle;
            xyz[2] = acc_imit_data[2] + acc_imit_data_toggle;
        }
		input_event(idev, EV_REL, EVENT_TYPE_ACCEL_X, xyz[0]);
		input_event(idev, EV_REL, EVENT_TYPE_ACCEL_Y, xyz[1]);
		input_event(idev, EV_REL, EVENT_TYPE_ACCEL_Z, xyz[2]);
		input_event(idev, EV_SYN, SYN_REPORT, 0);
        adev->acc_data[0] = xyz[0];
        adev->acc_data[1] = xyz[1];
        adev->acc_data[2] = xyz[2];
	}
}

static void magsns_poll(struct input_dev *idev)
{
	int xyz[3];

	if (hscdtd_get_magnetic_field_data(xyz) == 0) {
		input_event(idev, EV_REL, EVENT_TYPE_MAGV_X, xyz[0]);
		input_event(idev, EV_REL, EVENT_TYPE_MAGV_Y, xyz[1]);
		input_event(idev, EV_REL, EVENT_TYPE_MAGV_Z, xyz[2]);
		input_event(idev, EV_SYN, SYN_REPORT, 2);
        adev->mag_data[0] = xyz[0];
        adev->mag_data[1] = xyz[1];
        adev->mag_data[2] = xyz[2];
	}
}

static void alps_poll(struct work_struct *work)
{
	struct input_dev *idev = adev->idev;

	mutex_lock(&alps_lock);

	if (!adev->flgSuspend) {
		if (adev->flgM)
			magsns_poll(idev);
		if (adev->flgA)
			accsns_poll(idev);
	}
	mutex_unlock(&alps_lock);
}

static enum hrtimer_restart alps_timer_function(struct hrtimer *timer)
{
	queue_work(adev->wq, &adev->work_data);
	hrtimer_forward_now(&adev->timer,
		ns_to_ktime((u64)adev->delay * NSEC_PER_MSEC));

	return HRTIMER_RESTART;
}

/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static int __init alps_init(void)
{
	int ret;

	adev = kzalloc(sizeof(struct alps_input_data), GFP_KERNEL);
	if (!adev) {
		ret = -ENOMEM;
		goto out_region;
	}

	adev->idev = input_allocate_device();
	if (!adev->idev) {
		ret = -ENOMEM;
		pr_err(ALPS_IDEV_LOG_TAG "input_allocate_device\n");
		goto out_kzalloc;
	}
	pr_info(ALPS_IDEV_LOG_TAG "input_allocate_device\n");

	/* initialize the input class */
	adev->idev->name = ALPS_INPUT_DEVICE_NAME;
	adev->idev->id.bustype = BUS_I2C;
	adev->idev->evbit[0] = BIT_MASK(EV_REL);

	input_set_capability(adev->idev, EV_REL, EVENT_TYPE_ACCEL_X);
	input_set_capability(adev->idev, EV_REL, EVENT_TYPE_ACCEL_Y);
	input_set_capability(adev->idev, EV_REL, EVENT_TYPE_ACCEL_Z);

	input_set_capability(adev->idev, EV_REL, EVENT_TYPE_MAGV_X);
	input_set_capability(adev->idev, EV_REL, EVENT_TYPE_MAGV_Y);
	input_set_capability(adev->idev, EV_REL, EVENT_TYPE_MAGV_Z);

	ret = input_register_device(adev->idev);
	if (ret) {
		dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"error: input_register_device\n");
		goto out_alc_poll;
	}
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"input_register_device\n");

	ret = misc_register(&alps_ioctl_device);
	if (ret) {
		dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"error: alps_io_device register failed\n");
		goto out_reg_poll;
	}
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"misc_register\n");

	hrtimer_init(&adev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	adev->timer.function = alps_timer_function;

	adev->wq = create_singlethread_workqueue("alps_wq");
	if (!adev->wq) {
		ret = -ENOMEM;
		goto out_misc;
	}
	INIT_WORK(&adev->work_data, alps_poll);

	ret = alps_idev_create_sysfs(&adev->idev->dev);
	if (ret) {
		dev_err(&adev->idev->dev, ALPS_IDEV_LOG_TAG
			"error: alps_idev_create_sysfs\n");
		goto out_misc;
	}
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"alps_idev_create_sysfs\n");

	mutex_lock(&alps_lock);
	adev->flgM = 0;
	adev->flgA = 0;
	adev->flgSuspend = 0;
	adev->delay = ALPS_POLL_INTERVAL;
	mutex_unlock(&alps_lock);

	return 0;

out_misc:
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"misc_deregister\n");
	misc_deregister(&alps_ioctl_device);
out_reg_poll:
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"input_free_device\n");
	input_unregister_device(adev->idev);
out_alc_poll:
	pr_info(ALPS_IDEV_LOG_TAG
		"input_free_device\n");
	input_free_device(adev->idev);
out_kzalloc:
	kzfree(adev);
out_region:
	return ret;
}

static void __exit alps_exit(void)
{
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"sysfs_remove_group\n");
	alps_idev_remove_sysfs(&adev->idev->dev);
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"alps_idev_remove_sysfs\n");
	misc_deregister(&alps_ioctl_device);
	dev_info(&adev->idev->dev, ALPS_IDEV_LOG_TAG
		"input_unregister_device\n");
	input_unregister_device(adev->idev);
	pr_info(ALPS_IDEV_LOG_TAG
		" input_free_device\n");
	input_free_device(adev->idev);
	kzfree(adev);
	pr_info(ALPS_IDEV_LOG_TAG "kzfree\n");
}

module_init(alps_init);
module_exit(alps_exit);

MODULE_DESCRIPTION("Alps Input Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
