/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */
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

#include <linux/key_dm_driver.h>
#include <linux/gpio_keys.h>
#include <linux/input/pmic8xxx-pwrkey.h>

#ifdef KEYLOG_ENABLE
#define KEY_LOG_PRINT(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#else
#define KEY_LOG_PRINT(fmt, ...)
#endif

unsigned char   g_key_dm_port_check = 0;
unsigned char   g_key_dm_port_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
unsigned char   g_key_dm_port_data_idx = 0;
unsigned char   g_key_dm_ic_data[8] = {0x00,};
unsigned char   g_key_dm_ic_data_idx = 0;

struct c2p {
unsigned char cgr;
unsigned char scan;
};

struct c2p changer2port[] = {
{0xF0,0x01},{0xE4,0x01},{0xC0,0x00},{0xC4,0x00},{0xA0,0x00},
{0x10,0x03},{0x11,0x04},{0x12,0x02}};

void key_dm_driver_set_port(unsigned char in_swmode)
{
    KEY_LOG_PRINT( "%s: sw:%x \n", __func__, in_swmode);
	if( g_key_dm_port_check )
	{	
		unsigned char i = 0;
		for(i = 0; i < ARRAY_SIZE(changer2port); i++)
		{
			if( changer2port[i].cgr == in_swmode )
			{
				g_key_dm_port_data[g_key_dm_port_data_idx] = changer2port[i].scan;
				g_key_dm_port_data_idx++;
				if( ARRAY_SIZE(g_key_dm_port_data) <= g_key_dm_port_data_idx )
				{
					g_key_dm_port_data_idx = 0;
				}
			}
		}
		g_key_dm_ic_data[g_key_dm_ic_data_idx] = in_swmode;
		g_key_dm_ic_data_idx++;
		if( ARRAY_SIZE(g_key_dm_ic_data) <= g_key_dm_ic_data_idx )
		{
			g_key_dm_ic_data_idx = 0;
		}
	}
}
EXPORT_SYMBOL(key_dm_driver_set_port);

static int key_dm_driver_open(struct inode *inode, struct file *file)
{
    KEY_LOG_PRINT( "key_dm_driver_open\n");
    return 0;
}

static long key_dm_driver_ioctl(struct file* filp, unsigned int cmd, unsigned long data)
{
    int ret = KEY_DM_DRIVER_OK;
	int *the_v = NULL;
	int the_vsize = sizeof(int) * 2;

	the_v = (int*)kmalloc(the_vsize, GFP_KERNEL);
	if( !the_v )
	{
		return -ENOMEM;
	}

    KEY_LOG_PRINT( "%s: cmd:%d \n", __func__, cmd );

	memset(the_v, 0x00, the_vsize);

    switch(cmd){
	case KEY_DM_DRIVER_IOCTL_02:
		if (copy_from_user(the_v, (void *) data, the_vsize)) {
			ret = -EFAULT;
			break;
		}
		key_cmd(KEY_DM_CHECK_COMMAND, the_v);
		pwrkey_cmd(KEY_DM_CHECK_COMMAND, the_v);
        matrixkey_cmd(KEY_DM_CHECK_COMMAND, the_v);
		break;


	case KEY_DM_DRIVER_IOCTL_03:
		key_cmd(KEY_DM_KEY_GET_EVENT_COOMAND, the_v);
        if (copy_to_user((void *)data, the_v, the_vsize))
        {
			 ret = -EFAULT;
		}
		break;


	case KEY_DM_DRIVER_IOCTL_04:
		if( g_key_dm_port_check )
		{
			 ret = -EFAULT;
		}else{
		g_key_dm_port_check = 1;
		}
		break;


	case KEY_DM_DRIVER_IOCTL_05:
		if( g_key_dm_port_check )
		{
			if (copy_to_user((void *)data, &g_key_dm_port_data, ARRAY_SIZE(g_key_dm_port_data)))
			{
				ret = -EFAULT;
			}
			g_key_dm_port_data_idx = 0;
			memset( &g_key_dm_port_data, 0xff, ARRAY_SIZE(g_key_dm_port_data) );
			memset( &g_key_dm_ic_data, 0x00, ARRAY_SIZE(g_key_dm_ic_data) );
		}else{
			 ret = -EFAULT;
		}
		break;

	case KEY_DM_DRIVER_IOCTL_06:
		if( g_key_dm_port_check )
		{
			g_key_dm_port_check = 0;
		}else{
			 ret = -EFAULT;
		}
		break;


    default:
        ret = -ESRCH;
        break;
    }
    
	kfree(the_v);

    return ret;
}

static const struct file_operations key_dm_driverfops = {
    .owner      = THIS_MODULE,
    .open       = key_dm_driver_open,
    .unlocked_ioctl      = key_dm_driver_ioctl,
};

static struct miscdevice kdm = {
    .fops       = &key_dm_driverfops,
    .name       = "key_dm",
    .minor      = MISC_DYNAMIC_MINOR,
};

static int __init key_dm_driver_init(void)
{
    KEY_LOG_PRINT( "%s: \n", __func__);
    return misc_register(&kdm);
}

static void __exit key_dm_driver_exit(void)
{
    KEY_LOG_PRINT( "%s: \n", __func__);
    misc_deregister(&kdm);
}

module_init(key_dm_driver_init);
module_exit(key_dm_driver_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("KEY DM Driver");
MODULE_LICENSE("GPL v2");
