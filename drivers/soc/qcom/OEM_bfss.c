/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/atomic.h>

#define USE_REAL_SMEM
#ifdef USE_REAL_SMEM
#include <soc/qcom/smsm.h>
#endif
/*----------------------------------------------------------------------------*/
#define OEM_BFSS_LOG_01 KERN_ERR
#define OEM_BFSS_LOG_02 KERN_ERR
#define OEM_BFSS_LOG_03 KERN_ERR
#define OEM_BFSS_LOG_04 KERN_DEBUG
#define OEM_BFSS_LOG_05 KERN_DEBUG
#define OEM_BFSS_LOG_06 KERN_DEBUG

#ifdef USE_REAL_SMEM
#define OEM_BFSS_1KB	1024
#define OEM_BFSS_SIZE	64*OEM_BFSS_1KB
#endif
/*----------------------------------------------------------------------------*/
#ifdef USE_REAL_SMEM
static unsigned int *oem_bfss_ptr = NULL;
#endif
atomic_t oem_bfss_open_counter = ATOMIC_INIT(0);
/*----------------------------------------------------------------------------*/
/*                            read                                            */
/*----------------------------------------------------------------------------*/
static ssize_t OEM_bfss_read(struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
	int bytes_read = 0;

	printk(OEM_BFSS_LOG_06 "[%04d]:%s() start.  count = %zu\n", __LINE__, __func__, count);
#ifdef USE_REAL_SMEM
	if(NULL != oem_bfss_ptr) {
		if(copy_to_user((void *)buf, oem_bfss_ptr, count)) {
			printk(OEM_BFSS_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
			return -EINVAL;
		}
		else
		{
			bytes_read += count;
		}
	}
#endif
	printk(OEM_BFSS_LOG_04 "[%04d]:%s() end.  bytes_read = %d\n", __LINE__, __func__, bytes_read);
	return bytes_read;
}
/*----------------------------------------------------------------------------*/
/*                            write                                           */
/*----------------------------------------------------------------------------*/
static ssize_t OEM_bfss_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	printk(OEM_BFSS_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
#ifdef USE_REAL_SMEM
	if(NULL != oem_bfss_ptr) {
		if(copy_from_user(oem_bfss_ptr, (void *)buf, count)) {
			printk(OEM_BFSS_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
			return -EINVAL;
		}
	}
#endif
	printk(OEM_BFSS_LOG_04 "[%04d]:%s() end.  count = %zu\n", __LINE__, __func__, count);

	return count;
}
/*----------------------------------------------------------------------------*/
/*                            open                                            */
/*----------------------------------------------------------------------------*/
static int OEM_bfss_open(struct inode *ip, struct file *fp)
{
	int	ret;

	printk(OEM_BFSS_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(atomic_read(&oem_bfss_open_counter))
	{
		return -EBUSY;
	}
	ret = nonseekable_open(ip, fp);
	printk(OEM_BFSS_LOG_04 "[%04d]:%s() end.  ret = %d\n", __LINE__, __func__, ret);
	if (ret)
	{
		return ret;
	}
#ifdef USE_REAL_SMEM
	if(NULL == oem_bfss_ptr)
		oem_bfss_ptr = (unsigned int *)kc_smem_alloc(SMEM_BFSS_DATA, OEM_BFSS_SIZE);
#endif
	atomic_set(&oem_bfss_open_counter, 1);
	return 0;
}
/*----------------------------------------------------------------------------*/
/*                            release                                         */
/*----------------------------------------------------------------------------*/
static int OEM_bfss_release(struct inode *ip, struct file *fp)
{
	printk(OEM_BFSS_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	atomic_set(&oem_bfss_open_counter, 0);
	printk(OEM_BFSS_LOG_04 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static const struct file_operations OEM_bfss_fops = {
	.owner			= THIS_MODULE,
	.read			= OEM_bfss_read,
	.write			= OEM_bfss_write,
	.open			= OEM_bfss_open,
	.release		= OEM_bfss_release,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice OEM_bfss_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "OEM_bfss",
	.fops = &OEM_bfss_fops,
};
/*----------------------------------------------------------------------------*/
static int __init OEM_bfss_init(void)
{
	return misc_register(&OEM_bfss_dev);
}
/*----------------------------------------------------------------------------*/
module_init(OEM_bfss_init);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_LICENSE("GPL");
