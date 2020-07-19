/*****************************************************************************
*
* Filename:      irda_sd.c
* Version:       0.1
* Description:   IrDA driver
* Status:        Experimental
* Author:        KYOCERA Corporation
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2016 KYOCERA Corporation
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
#include <asm/gpio.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "irda_sd.h"
#include <linux/of_gpio.h>

static struct irda_sd_info *irdasd_info;
static struct platform_device *pDev;
static struct irda_sd_gpio_data *irda_sd_port;

static int irdasd_fop_open(struct inode *inode, struct file *file) ;
static int irdasd_fop_release(struct inode *inode, struct file *file) ;

struct irda_sd_gpio_data {
	int gpio_shutdown_port;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_inactive;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

};

/* ------------------------------------------------------------------------------------
 *		irdasd file operation
 * ------------------------------------------------------------------------------------ */
static int irdasd_fop_open(struct inode *inode, struct file *file)
{
	file->private_data = irdasd_info ;

	gpio_set_value_cansleep(irda_sd_port->gpio_shutdown_port, 0);
	return 0;
}

static int irdasd_fop_release(struct inode *inode, struct file *file)
{
	if(irda_sd_port->gpio_shutdown_port>0){
		gpio_set_value_cansleep((unsigned int)irda_sd_port->gpio_shutdown_port, 1);
	}
	else{
	}
	return 0;
}


static struct class *irdasd_class;

struct file_operations irdasd_fops =
{
	.owner		= THIS_MODULE,
	.open		= irdasd_fop_open,
	.release	= irdasd_fop_release,
};

static char *irdasd_devnode(struct device *dev, mode_t *mode)
{
	if (mode)
		*mode = 0666;
	return kasprintf(GFP_KERNEL,"%s", dev_name(dev));
}



static int irdasd_suspend (struct platform_device *pdev, pm_message_t state)
{
	int ret;
/* 	printk( KERN_NOTICE"irdasd_suspend.\n" ) ; */

	if(irda_sd_port->gpio_shutdown_port>0){
/*		gpio_tlmm_config(GPIO_CFG(gpio_shutdown_port, 0, GPIO_CFG_OUTPUT,GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE); */

		ret = pinctrl_select_state(irda_sd_port->pinctrl,
				irda_sd_port->gpio_state_suspend);
		if (ret)
			pr_err("Failed to pinctrl set_state suspend\n");
	}
	return 0;
}

static int irdasd_resume (struct platform_device *pdev)
{
	int ret;
/*	printk( KERN_NOTICE"irdasd_resume.\n" ) ; */
	if(irda_sd_port->gpio_shutdown_port>0){
/*		gpio_tlmm_config(GPIO_CFG(gpio_shutdown_port, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE); */

		ret = pinctrl_select_state(irda_sd_port->pinctrl,
				irda_sd_port->gpio_state_inactive);
		if (ret)
			pr_err("Failed to pinctrl set_state inactive\n");
	}
	return 0;
}


static int irdasd_probe(struct platform_device *pdev)
{
	int rc;
	struct pinctrl_state *set_state;


	irda_sd_port = devm_kzalloc(&pdev->dev, sizeof(struct irda_sd_gpio_data),
				 GFP_KERNEL);
	if (irda_sd_port == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}


	/* PM_GPIO */
	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		return -EINVAL;
	}

	pDev = pdev;

/* for PINCTRL */
	irda_sd_port->pinctrl = devm_pinctrl_get(&pdev->dev);
	pr_debug("Pinctrl %p\n", irda_sd_port->pinctrl);
	if (IS_ERR_OR_NULL(irda_sd_port->pinctrl)) {
		pr_debug("Pinctrl not defined");
	} else {

		set_state = pinctrl_lookup_state(irda_sd_port->pinctrl,
						"irdasd_inactive");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for inactive state\n");
			goto pinctrl_fail;
		}

		pr_debug("Pinctrl state inactive %p\n", set_state);
		irda_sd_port->gpio_state_inactive = set_state;

		set_state = pinctrl_lookup_state(irda_sd_port->pinctrl,
						"irdasd_active");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for active state\n");
			goto pinctrl_fail;
		}

		pr_debug("Pinctrl state active %p\n", set_state);
		irda_sd_port->gpio_state_active = set_state;

		set_state = pinctrl_lookup_state(irda_sd_port->pinctrl,
						"irdasd_sleep");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for sleep state\n");
			goto pinctrl_fail;
		}

		pr_debug("Pinctrl state sleep %p\n", set_state);
		irda_sd_port->gpio_state_suspend = set_state;
	}
/*	platform_set_drvdata(pdev, irda_sd_port); */


/* Set IrDA SD port */
	irda_sd_port->gpio_shutdown_port = of_get_named_gpio(pDev->dev.of_node, "kc,irda-sd", 0);
	if (irda_sd_port->gpio_shutdown_port < 0) {
		pr_err("%s of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(irda_sd_port->gpio_shutdown_port, "irda-sd");
	if (rc) {
		pr_err("%s gpio_request failed.\n", __func__);
		return -EINVAL;
	}

	return 0;

pinctrl_fail:
	irda_sd_port->pinctrl = NULL;
	devm_kfree(&pdev->dev, irda_sd_port);
	return -EINVAL;
}

/* static int __devexit irdasd_remove(struct platform_device *pdev) */
static int irdasd_remove(struct platform_device *pdev)
{
	struct irda_sd_gpio_data *irda_sd_port =
	    (struct irda_sd_gpio_data *)platform_get_drvdata(pdev);

	gpio_free(irda_sd_port->gpio_shutdown_port);
	irda_sd_port->gpio_shutdown_port = -1;

	devm_kfree(&pdev->dev, irda_sd_port);

	return 0;
}

static const struct of_device_id irdasd_of_match[] = {
	{ .compatible = "kc,irdasd", },
	{},
};

static struct platform_driver irdasd_pd = {
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = irdasd_of_match,
	},
	.probe = irdasd_probe,
/*	.remove = __devexit_p(irdasd_remove), */
	.remove = irdasd_remove,
	.suspend	= irdasd_suspend,
	.resume		= irdasd_resume,
};


static int init_irdasd( void )
{
	int ret = 0;

	printk( KERN_NOTICE"IRDASD module is beeing initialized.\n" ) ;
	platform_driver_register(&irdasd_pd);

	irdasd_info = kzalloc(sizeof(*irdasd_info), GFP_KERNEL);
	if (irdasd_info == NULL) {
		pr_err(MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	irdasd_class = class_create(THIS_MODULE, MODULE_NAME);

	ret = alloc_chrdev_region(&irdasd_info->dev_num, 0, 1, MODULE_NAME);
	if (ret) {
		printk(MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}
	irdasd_class->devnode = irdasd_devnode;
	irdasd_info->dev = device_create(irdasd_class, NULL, irdasd_info->dev_num,
				      irdasd_info, MODULE_NAME);
	if (IS_ERR(irdasd_info->dev)) {
		printk(MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}

	irdasd_info->cdev = cdev_alloc();
	if (irdasd_info->cdev == NULL) {
		printk(MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(irdasd_info->cdev, &irdasd_fops);
	irdasd_info->cdev->owner = THIS_MODULE;

	ret = cdev_add(irdasd_info->cdev, irdasd_info->dev_num, 1);
	if (ret)
		printk(MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		printk(MODULE_NAME ":irdasd init OK..\n");

	printk( " %s driver installed.\n", MODULE_NAME );

	return ret;

}

static void exit_irdasd( void )
{
	cdev_del(irdasd_info->cdev);
	device_destroy(irdasd_class, irdasd_info->dev_num);
	unregister_chrdev_region(irdasd_info->dev_num, 1);

	kfree(irdasd_info);
	printk( "IRDASD module is removed.\n" ) ;
}

module_init( init_irdasd ) ;
module_exit( exit_irdasd ) ;

