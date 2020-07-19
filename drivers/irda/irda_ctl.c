/*****************************************************************************
*
* Filename:      irda_ctl.c
* Version:       0.1
* Description:   IrDA driver
* Status:        Experimental
* Author:        KYOCERA Corporation
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2019 KYOCERA Corporation
*
*	This program is free software; you can redistribute it and/or
*   modify it under the terms of the GNU General Public License
*   as published by the Free Software Foundation; only version 2.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/gpio.h>

#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "irda_ctl.h"
#include <linux/of_gpio.h>

#include <linux/regulator/consumer.h>

static struct irda_ctl_info *irdactl_info;
static struct platform_device *pDev;
static int gpio_pwdown_port = -1;
static int gpio_irled_enable_port = -1;

static int irdactl_fop_open(struct inode *inode, struct file *file) ;
static int irdactl_fop_release(struct inode *inode, struct file *file) ;

static struct regulator *l16;

/* ------------------------------------------------------------------------------------
 *		irdactl file operation
 * ------------------------------------------------------------------------------------ */
static int irdactl_fop_open(struct inode *inode, struct file *file)
{

	int err;
	
	printk(MODULE_NAME ":irdactl_fop_open start.\n");
	file->private_data = irdactl_info ;
	
	err = regulator_enable(l16);
	if (err) {
		pr_err("could not enable L16\n");
		return err;
	}

	//wait 150ƒÊs
	usleep_range(150, 150);
	
	gpio_set_value_cansleep(gpio_irled_enable_port, 1);
	//wait 50ƒÊs
	usleep_range(50, 50);
	gpio_set_value_cansleep(gpio_pwdown_port, 1);

	return 0;
}

static int irdactl_fop_release(struct inode *inode, struct file *file)
{
    int err;
	
	printk(MODULE_NAME ":irdactl_fop_release start.\n");

	if(gpio_pwdown_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_pwdown_port, 0);
	}
	else{
	}
	
	//wait 500ƒÊs 
	usleep_range(500, 500);

	if(gpio_irled_enable_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_irled_enable_port, 0);
	}
	else{
	}
	
	err = regulator_disable(l16);
	if (err) {
		pr_err("could not disable L16\n");
		return err;
	}

	return 0;
}


static struct class *irdactl_class;

struct file_operations irdactl_fops =
{
	.owner		= THIS_MODULE,
	.open		= irdactl_fop_open,
	.release	= irdactl_fop_release,
};

static char *irdactl_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = 0666;
	return kasprintf(GFP_KERNEL,"%s", dev_name(dev));
}


static int irdactl_suspend (struct platform_device *pdev, pm_message_t state)
{
	if(gpio_pwdown_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_pwdown_port, 0);
	}
	if(gpio_irled_enable_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_irled_enable_port, 0);
	}
	return 0;
}

static int irdactl_resume (struct platform_device *pdev)
{
	if(gpio_pwdown_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_pwdown_port, 0);
	}
	if(gpio_irled_enable_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_irled_enable_port, 0);
	}
	return 0;
}


static int irdactl_probe(struct platform_device *pdev)
{
	int rc;

	/* PM_GPIO */
	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		return -EINVAL;
	}

	pDev = pdev;

/* Set IrDA PowerDown port */
	gpio_pwdown_port = of_get_named_gpio(pDev->dev.of_node, "kc,ir_pwdown", 0);
	if (gpio_pwdown_port < 0) {
		pr_err("%s of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(gpio_pwdown_port, "ir_pwdown");
	if (rc) {
		pr_err("%s gpio_request failed.\n", __func__);
		return -EINVAL;
	}

/* regulator */
	printk(MODULE_NAME ":regulator_get 8916_l16.\n");
	l16 = regulator_get(NULL, "8916_l16");
	if (IS_ERR(l16)) {
		pr_err("could not get 8916_l16");
		return PTR_ERR(l16);
	}
	
	return 0;
}

//static int __devexit irdactl_remove(struct platform_device *pdev)
static int irdactl_remove(struct platform_device *pdev)
{

	gpio_free(gpio_pwdown_port);
	gpio_pwdown_port = -1;

	gpio_free(gpio_irled_enable_port);
	gpio_irled_enable_port = -1;

	return 0;
}

static const struct of_device_id irdactl_of_match[] = {
	{ .compatible = "kc,irda_ctl", },
	{},
};

static struct platform_driver irdactl_pd = {
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = irdactl_of_match,
	},
	.probe = irdactl_probe,
//	.remove = __devexit_p(irdactl_remove),
	.remove = irdactl_remove,
	.suspend	= irdactl_suspend,
	.resume		= irdactl_resume,
};


static int init_irdactl( void )
{
	int ret = 0;

	printk( KERN_NOTICE"IRDACTL module is beeing initialized.\n" ) ;
	platform_driver_register(&irdactl_pd);

	irdactl_info = kzalloc(sizeof(*irdactl_info), GFP_KERNEL);
	if (irdactl_info == NULL) {
		pr_err(MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	irdactl_class = class_create(THIS_MODULE, MODULE_NAME);

	ret = alloc_chrdev_region(&irdactl_info->dev_num, 0, 1, MODULE_NAME);
	if (ret) {
		printk(MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}

	irdactl_class->devnode = irdactl_devnode;

	irdactl_info->dev = device_create(irdactl_class, NULL, irdactl_info->dev_num,
				      irdactl_info, MODULE_NAME);
	if (IS_ERR(irdactl_info->dev)) {
		printk(MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}
	irdactl_info->cdev = cdev_alloc();
	if (irdactl_info->cdev == NULL) {
		printk(MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(irdactl_info->cdev, &irdactl_fops);
	irdactl_info->cdev->owner = THIS_MODULE;

	ret = cdev_add(irdactl_info->cdev, irdactl_info->dev_num, 1);
	if (ret)
		printk(MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		printk(MODULE_NAME ":irdactl init OK..\n");

	printk( " %s driver installed.\n", MODULE_NAME );

	return ret;

}

static void exit_irdactl( void )
{
	cdev_del(irdactl_info->cdev);
	device_destroy(irdactl_class, irdactl_info->dev_num);
	unregister_chrdev_region(irdactl_info->dev_num, 1);

	kfree(irdactl_info);
	printk( "IRDACTL module is removed.\n" ) ;
}

module_init( init_irdactl ) ;
module_exit( exit_irdactl ) ;

