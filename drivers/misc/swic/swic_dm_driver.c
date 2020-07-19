/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include "swic_dm_driver.h"


static int32_t swic_dm_driver_open(struct inode *inode, struct file *file)
{
	printk(KERN_DEBUG "%s START", __func__);
	return 0;
}


static long swic_dm_driver_ioctl(struct file* filp,
                           unsigned int cmd, unsigned long data)
{
	int32_t ret = 0;
	int32_t the_v[2];
	printk(KERN_DEBUG "%s START", __func__);
	memset(the_v, 0x00, sizeof(the_v));

	switch (cmd)
	{
		case SWIC_DM_DRIVER_IOCTL_01:
		{
			swic_get_dminfo(SWIC_DM_DRIVER_GET_SW_SET_MODE, the_v);
			if (copy_to_user((void *)data, the_v, sizeof(the_v)))
			{
				ret = -EFAULT;
			}
		}
		break;
		case SWIC_DM_DRIVER_IOCTL_02:
		{
			swic_get_dminfo(SWIC_DM_DRIVER_GET_DETECTION_ID_STATE, the_v);
			if (copy_to_user((void *)data, the_v, sizeof(the_v)))
			{
				ret = -EFAULT;
			}
		}
		break;
		case SWIC_DM_DRIVER_IOCTL_03:
		{
			swic_get_dminfo(SWIC_DM_DRIVER_GET_INTERRUPT_CASE, the_v);
			if (copy_to_user((void *)data, the_v, sizeof(the_v)))
			{
				ret = -EFAULT;
			}
		}
		break;
		case SWIC_DM_DRIVER_IOCTL_04:
		{
			swic_get_dminfo(SWIC_DM_DRIVER_GET_CHG_DET_MODE, the_v);
			if (copy_to_user((void *)data, the_v, sizeof(the_v)))
			{
				ret = -EFAULT;
			}
		}
		break;
		case SWIC_DM_DRIVER_IOCTL_05:
		{
			if (copy_from_user(the_v, (void *) data, sizeof(sizeof(the_v))))
			{
				ret = -EFAULT;
				break;
			}
		}
		break;
		case SWIC_DM_DRIVER_IOCTL_06:
		{
			if (copy_from_user(the_v, (void *) data, sizeof(sizeof(the_v))))
			{
				ret = -EFAULT;
				break;
			}
			swic_set_swic_det_wait(the_v);
		}
		break;
		default:
		{
			ret = -1;
		}
		break;
	}
	printk(KERN_DEBUG "%s END", __func__);
	return ret;
}


static const struct file_operations swic_dm_driverfops = {
	.owner			= THIS_MODULE,
	.open			= swic_dm_driver_open,
	.unlocked_ioctl	= swic_dm_driver_ioctl,
};

static struct miscdevice swic_dm = {
	.fops			= &swic_dm_driverfops,
	.name			= "swic_dm",
	.minor			= MISC_DYNAMIC_MINOR,
};


static int32_t __init swic_dm_driver_init(void)
{
	printk(KERN_DEBUG "%s START", __func__);
	return misc_register(&swic_dm);
}


static void __exit swic_dm_driver_exit(void)
{
	printk(KERN_DEBUG "%s START", __func__);
	misc_deregister(&swic_dm);
}


module_init(swic_dm_driver_init);
module_exit(swic_dm_driver_exit);

MODULE_AUTHOR("KC");
MODULE_DESCRIPTION("CHANGER DM Driver");
MODULE_LICENSE("GPL v2");
