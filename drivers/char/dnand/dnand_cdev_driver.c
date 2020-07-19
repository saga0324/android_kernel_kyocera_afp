/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#define pr_fmt(fmt) "DNANDCDEV: " fmt

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/dnand_cdev_driver.h>
#include "dnand_drv.h"
#include "dnand_k_api_internal.h"

static DEFINE_MUTEX(dnand_mutex);
#define MAX_SIZE 262144
static uint8_t data_buffer[MAX_SIZE] = {0};
static int dnand_cdev_driver_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int dnand_cdev_driver_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long dnand_cdev_driver_ioctl(struct file *filp,
				   unsigned int cmd, unsigned long data)
{
	int ret;
	uint8_t *p_kbuf;
	__user uint8_t *p_ubuf;
	dnand_data_type ddatabuf;

	mutex_lock(&dnand_mutex);

	memset(data_buffer, 0x00, sizeof(data_buffer));

	ret = copy_from_user(&ddatabuf, (void *)data, sizeof(ddatabuf));
	if (ret) {
		pr_warn("copy_from_user failed\n");
		ret = -ENOMEM;
		goto out;
	}

	if (ddatabuf.size > MAX_SIZE) {
		p_kbuf = (uint8_t*)kmalloc(ddatabuf.size, GFP_KERNEL);
		if (!p_kbuf) {
			pr_warn("Could not alloc memory of %d\n", ddatabuf.size);
			ret = -ENOMEM;
			goto out;
		}
	} else {
		p_kbuf = data_buffer;
	}
	p_ubuf = (__user uint8_t *)ddatabuf.pbuf;

	switch (cmd) {
	case DNAND_CDEV_DRIVER_IOCTL_01:
		ret = copy_from_user(p_kbuf, p_ubuf, ddatabuf.size);
		if (ret) {
			ret = -ENOMEM;
			goto out_free;
		}
		ret = kdnand_id_write(ddatabuf.cid, ddatabuf.offset,
				      p_kbuf, ddatabuf.size);
		if (ret == DNAND_NO_ERROR) {
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	case DNAND_CDEV_DRIVER_IOCTL_02:
		ret = kdnand_id_read(ddatabuf.cid, ddatabuf.offset,
				     p_kbuf, ddatabuf.size);
		if (ret == DNAND_NO_ERROR) {
			ret = 0;
		} else {
			ret = -EINVAL;
			goto out_free;
		}
		ret = copy_to_user(p_ubuf, p_kbuf, ddatabuf.size);
		if (ret) {
			ret = -ENOMEM;
			goto out_free;
		}
		break;
	default:
		pr_warn("unsupported ioctl\n");
		ret = -EINVAL;
		break;
	}

out_free:
	if (ddatabuf.size > MAX_SIZE) {
		kfree(p_kbuf);
	}
out:
	mutex_unlock(&dnand_mutex);
	return ret;
}

static const struct file_operations dnand_cdev_driverfops = {
	.owner = THIS_MODULE,
	.open = dnand_cdev_driver_open,
	.release = dnand_cdev_driver_release,
	.unlocked_ioctl = dnand_cdev_driver_ioctl,
};

static struct miscdevice dnandcdev = {
	.fops       = &dnand_cdev_driverfops,
	.name       = "dnand_cdev",
	.minor      = MISC_DYNAMIC_MINOR,
};

static int __init dnand_cdev_driver_init(void) {
	int ret;

	ret = kdnand_k_api_init();
	if (ret != DNAND_NO_ERROR)
		return -ENOMEM;

	pr_info("dnand_cdev initialized.");

	return misc_register(&dnandcdev);
}

static void __exit dnand_cdev_driver_exit(void) {
	misc_deregister(&dnandcdev);
}

late_initcall(dnand_cdev_driver_init);
module_exit(dnand_cdev_driver_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("DNAND CDEV Driver");
MODULE_LICENSE("GPL v2");

