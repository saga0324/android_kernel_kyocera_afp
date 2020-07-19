/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
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

#define pr_fmt(fmt)	"WLED %s: " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include <linux/kc_led.h>
//#include <mach/kc_board.h>
#include "kc_wled.h"
#ifdef CONFIG_DISCARD_BL_CONTROL
#include <linux/mutex.h>
#endif

#define DRV_NAME "kc_wled"
//#define BACKLIGHT_INFO "backlightinfo"

#define BACKLIGHT_INFO "lcd-backlight"

struct kc_wled_data {
	struct led_classdev	st_cdev;
	struct work_struct	work;
	struct delayed_work	monitor_dwork;
	int					lcd_bl_drv_en_gpio;
};

typedef void (*kc_wled_func_type)(int level, enum kc_wled_table tbl, int gpio);

static kc_wled_func_type kc_wled_func = cat4004b_work;

static enum kc_wled_table kc_wled_tbl = WLED_TBL_NORM;

#ifdef CONFIG_DISCARD_BL_CONTROL
static struct kc_wled_data *the_data;
static DEFINE_MUTEX(kc_wled_lock);
static atomic_t lcd_status = ATOMIC_INIT(1);
#endif

static int32_t light_led_disp_set(e_light_main_wled_disp disp_status);

static void kc_wled_brightness_set(struct led_classdev *pst_cdev, enum led_brightness value)
{
	struct kc_wled_data *data = container_of(pst_cdev, struct kc_wled_data, st_cdev);

	pr_err("[Disp]%s name:%s value:0x%08x\n", __func__, pst_cdev->name, data->st_cdev.brightness);
	
	if (value > data->st_cdev.max_brightness) {
		pr_err("invalid value:%d max:%d\n", value, data->st_cdev.max_brightness);
		value = data->st_cdev.max_brightness;
	}

	data->st_cdev.brightness = value;
	schedule_work(&data->work);

	pr_debug("name:%s value:0x%08x\n", pst_cdev->name, value);
	return;
}

static enum led_brightness kc_wled_brightness_get(struct led_classdev *pst_cdev)
{
	struct kc_wled_data *data = container_of(pst_cdev, struct kc_wled_data, st_cdev);
	int32_t lret = data->st_cdev.brightness;

	pr_err("[Disp]%s ret:0x%02x\n", __func__, lret);
	pr_debug("ret:0x%02x\n", lret);
	return lret;
}

static void kc_wled_work_handler(struct work_struct *work)
{
	struct kc_wled_data *data;

	pr_err("[Disp]%s \n", __func__);
	
	mutex_lock(&kc_wled_lock);

	data = container_of(work, struct kc_wled_data, work);

#ifdef CONFIG_DISCARD_BL_CONTROL
	if ((data->st_cdev.brightness != WLED_BRIGHTNESS_OFF) &&
		(!atomic_read(&lcd_status)))
		pr_info("lcd is off, discard backlight brightness:%d\n", data->st_cdev.brightness);
	else
#endif
	(*kc_wled_func)(
		data->st_cdev.brightness,
		kc_wled_tbl,
		data->lcd_bl_drv_en_gpio);

	mutex_unlock(&kc_wled_lock);

	return;
}

static int kc_wled_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct kc_wled_data *data;

	pr_err("[Disp]%s \n", __func__);
	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		rc = -EINVAL;
		goto exit;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(struct kc_wled_data), GFP_KERNEL);
	if (!data) {
		pr_err("kzalloc fail\n");
		rc = -ENOMEM;
		goto exit;
	}

#ifdef CONFIG_DISCARD_BL_CONTROL
	the_data = data;
#endif

	INIT_WORK(&data->work, kc_wled_work_handler);

	pr_err("[Disp]%s INIT_WORK_end\n", __func__);
	
	platform_set_drvdata(pdev, data);
	data->st_cdev.max_brightness = WLED_BRIGHTNESS_MAX;
	data->st_cdev.brightness = WLED_BRIGHTNESS_DEFAULT;
	data->st_cdev.brightness_set = kc_wled_brightness_set;
	data->st_cdev.brightness_get = kc_wled_brightness_get;
	data->st_cdev.name = BACKLIGHT_INFO;

	rc = led_classdev_register(&pdev->dev, &data->st_cdev);
	if (rc) {
		pr_err("unable to register led %s\n", data->st_cdev.name);
		goto error;
	}

	data->lcd_bl_drv_en_gpio = of_get_named_gpio(pdev->dev.of_node, "kc,lcd-bl-drv-en-gpio", 0);
	if (!gpio_is_valid(data->lcd_bl_drv_en_gpio)) {
		pr_err("of_get_named_gpio failed.\n");
		rc = -EINVAL;
		goto error;
	}

	rc = gpio_request(data->lcd_bl_drv_en_gpio, DRV_NAME);
	if (rc) {
		pr_err("gpio_request failed.\n");
		goto error;
	}

	//pr_info("initialization is completed!! GPIO(%d)=%d\n",
	pr_err("[Disp]initialization is completed!! GPIO(%d)=%d\n",
		data->lcd_bl_drv_en_gpio, gpio_get_value(data->lcd_bl_drv_en_gpio));

	return 0;

error:
	led_classdev_unregister(&data->st_cdev);
	cancel_work_sync(&data->work);
	devm_kfree(&pdev->dev, data);

exit:
	pr_err("failed.\n");

	return rc;
}

static int kc_wled_remove(struct platform_device *pdev)
{
	struct kc_wled_data *data = platform_get_drvdata(pdev);

	pr_err("[Disp]%s \n", __func__);
	led_classdev_unregister(&data->st_cdev);
	cancel_work_sync(&data->work);
	devm_kfree(&pdev->dev, data);

	return 0;
}

static const struct of_device_id kc_wled_of_match[] = {
	{ .compatible = DRV_NAME, },
};

static struct platform_driver kc_wled_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = kc_wled_of_match,
	},
	.probe = kc_wled_probe,
	.remove = kc_wled_remove,
};

int __init kc_wled_init(void)
{
	
	pr_err("[Disp]%s \n", __func__);
	return platform_driver_register(&kc_wled_driver);
}

static void __exit kc_wled_exit(void)
{
	
	pr_err("[Disp]%s \n", __func__);
	
	platform_driver_unregister(&kc_wled_driver);
}



int32_t cat4004b_light_led_disp_set_panel(e_light_main_wled_disp disp_status, e_light_lcd_panel panel_class)
{
	pr_debug("[IN] panel_class=0x%x", panel_class);
	switch( panel_class ){
	case LIGHT_LCD_PANEL0:
		pr_debug("panel class = LIGHT_LCD_PANEL0");
		break;
	default:
		pr_debug("unknown panel class");
		break;
	}

	return light_led_disp_set(disp_status);
}
EXPORT_SYMBOL(cat4004b_light_led_disp_set_panel);

int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
	pr_debug("DISABLE_DISP_DETECT");
	return 0;
}
EXPORT_SYMBOL(light_led_disp_set);


#ifdef CONFIG_DISCARD_BL_CONTROL
int32_t cat4004b_light_led_disp_power_set(e_light_main_wled_disp disp_status)
{
	int32_t ret = 0;
	
	pr_err("[Disp]%s \n", __func__);
	mutex_lock(&kc_wled_lock);
	atomic_set(&lcd_status, disp_status);
	if (disp_status) {
		if (the_data->st_cdev.brightness != WLED_BRIGHTNESS_OFF) {
			(*kc_wled_func)(
				the_data->st_cdev.brightness,
				kc_wled_tbl,
				the_data->lcd_bl_drv_en_gpio);
		}
	} else {
		if (the_data->st_cdev.brightness != WLED_BRIGHTNESS_OFF) {
			(*kc_wled_func)(
				WLED_BRIGHTNESS_OFF,
				kc_wled_tbl,
				the_data->lcd_bl_drv_en_gpio);
		}
	}
	pr_err("Disp]%s set lcd_status:%d brightness:%d\n", __func__, atomic_read(&lcd_status), the_data->st_cdev.brightness);
	//pr_debug("set lcd_status:%d brightness:%d\n", atomic_read(&lcd_status), the_data->st_cdev.brightness);
	mutex_unlock(&kc_wled_lock);
	return ret;
}
EXPORT_SYMBOL(cat4004b_light_led_disp_power_set);
#endif

void kc_wled_off(void)
{
	
	pr_err("[Disp]%s \n", __func__);
	mutex_lock(&kc_wled_lock);
	(*kc_wled_func)(
		WLED_BRIGHTNESS_OFF,
		WLED_TBL_NORM,
		the_data->lcd_bl_drv_en_gpio);
	mutex_unlock(&kc_wled_lock);
}
EXPORT_SYMBOL(kc_wled_off);

module_init(kc_wled_init);
module_exit(kc_wled_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("KC WLED Driver");
MODULE_LICENSE("GPL");
