/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
/* This program is free software; you can redistribute it and/or
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/input/matrix_keypad.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/input/mt.h>
#include "kckeyptr.h"

#define FT_CENTER 0x10
#define FT_UP     0x08
#define FT_DOWN   0x04
#define FT_LEFT   0x02
#define FT_RIGHT  0x01

#define KEYPTR_KEYMATRIX_ENABLE

struct keyptr_data {
	struct input_dev		*input_dev;
	struct delayed_work		event_work;
	struct workqueue_struct	*event_wq;
	struct mutex			report_lock;
	struct class			*class;
	struct device			*device;
	dev_t					dev_num;
	struct					cdev cdev;
	int						keycode;
	int						last_keycode;
	int						q_loop_x;
	int						q_loop_y;
	int						panel_width;
	int						panel_height;
	int						move_step;
	int						move_init_interval;
	int						move_interval;
	int						onoff;
	bool					enabled;
};

#define CLASS_NAME "keyptr"
#define DRIVER_NAME "keyptr"

void keyptr_matrix_prepar(void *pkeyptr)
{
	struct keyptr_data *keyptr = (struct keyptr_data *)pkeyptr;

	if (keyptr == NULL)
		return;

	if ((!keyptr->onoff)||(!keyptr->enabled))
		return;

	keyptr->last_keycode = keyptr->keycode;
}

bool keyptr_matrix_scan(void *pkeyptr, unsigned short keycode, uint32_t new_state)
{
	struct keyptr_data *keyptr = (struct keyptr_data *)pkeyptr;
	struct input_dev *input_dev = keyptr->input_dev;
	int work_key_code;
	bool key_rel = false;
	bool fake_touch = false;

	if ((keyptr == NULL) || (!keyptr->enabled))
		return 0;

	if (!keyptr->onoff) {
		if (!keyptr->keycode) {
			return 0;
		}
	}

	pr_debug("%s: + keycode=%x onoff=%d enabled=%d\n",
				__func__, keycode, keyptr->onoff, keyptr->enabled);

	if ( keyptr->onoff &&
		 (keycode == 0x67 || keycode == 0x6c ||
		  keycode == 0x69 || keycode == 0x6a || keycode == 0x1c)) {
		fake_touch = true;
		if(new_state){
			switch(keycode){
				case 0x67 : keyptr->keycode |= FT_UP;     break;
		case 0x69 : keyptr->keycode |= FT_LEFT;   break;
				case 0x6a : keyptr->keycode |= FT_RIGHT;  break;
				case 0x6c : keyptr->keycode |= FT_DOWN;   break;
				case 0x1c : keyptr->keycode |= FT_CENTER; break;
				default   : keyptr->keycode  = 0x00;      break;
			}
		} else {
			switch(keycode){
				case 0x67 : work_key_code = FT_UP;     break;
				case 0x69 : work_key_code = FT_LEFT;   break;
				case 0x6a : work_key_code = FT_RIGHT;  break;
				case 0x6c : work_key_code = FT_DOWN;   break;
				case 0x1c : work_key_code = FT_CENTER; break;
				default   :
					work_key_code = 0x00;
					keyptr->keycode  = 0x00;
					break;
			}
			if (keyptr->keycode & work_key_code)
				keyptr->keycode ^= work_key_code;
			else
				key_rel = true;
		}
		pr_debug("%s: Fake Touch: last:%d, now:%d, push/release:%d\n",
							__func__, keyptr->last_keycode, keyptr->keycode, new_state);
		if( key_rel || (!keyptr->last_keycode && !new_state)){
			pr_debug("%s: Key Release\n",__func__);
			fake_touch = false;
//			input_report_key(input_dev, keycodes, new_state);
//			MATRIX_KEYPAD_DEBUG_LOG_PRINT("input_report_key code %d %d  \n", keycodes, new_state[col] & (1 << row));
		}
	} else {
		if(!keyptr->onoff && keyptr->keycode){
			pr_debug("%s: Pointer Release\n",__func__);
			keyptr->keycode = 0x00;
			if(keyptr->event_wq){
				cancel_delayed_work_sync(&keyptr->event_work);
				flush_workqueue(keyptr->event_wq);
			}
			mutex_lock(&keyptr->report_lock);
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_sync(input_dev);
			mutex_unlock(&keyptr->report_lock);
		}
		fake_touch = false;

//				input_report_key(input_dev,
//						 keycodes[code],
//						 new_state[col] & (1 << row));
//				MATRIX_KEYPAD_DEBUG_LOG_PRINT("input_report_key code %d %d  \n", keycodes[code], new_state[col] & (1 << row));
	}

	return fake_touch;
}

void keyptr_input_report(void *pkeyptr)
{
	struct keyptr_data *keyptr = (struct keyptr_data *)pkeyptr;
	int first_x, first_y;
	struct input_dev *input_dev = keyptr->input_dev;
	int move;
	int center_x;
	int center_y;

	if (keyptr == NULL)
		return;

	if ((!keyptr->onoff)||(!keyptr->enabled))
		return;

	mutex_lock(&keyptr->report_lock);

	move = keyptr->move_step;
	center_x = keyptr->panel_width/2;
	center_y = keyptr->panel_height/2;
	switch(keyptr->keycode){
		case FT_CENTER|FT_DOWN:
		case FT_DOWN   : first_x = center_x;        first_y = center_y + move; break;
		case FT_CENTER|FT_UP:
		case FT_UP     : first_x = center_x;        first_y = center_y - move; break;
		case FT_CENTER|FT_RIGHT:
		case FT_RIGHT  : first_x = center_x + move; first_y = center_y;        break;
		case FT_CENTER|FT_LEFT:
		case FT_LEFT   : first_x = center_x - move; first_y = center_y;        break;

		case FT_CENTER : first_x = center_x;        first_y = center_y;        break;

		case FT_CENTER|FT_UP|FT_RIGHT:
		case FT_UP|FT_RIGHT   : first_x = center_x + move; first_y = center_y - move; break;
		case FT_CENTER|FT_UP|FT_LEFT:
		case FT_UP|FT_LEFT    : first_x = center_x - move; first_y = center_y - move; break;
		case FT_CENTER|FT_DOWN|FT_RIGHT:
		case FT_DOWN|FT_RIGHT : first_x = center_x + move; first_y = center_y + move; break;
		case FT_CENTER|FT_DOWN|FT_LEFT:
		case FT_DOWN|FT_LEFT  : first_x = center_x - move; first_y = center_y + move; break;
		case 0x00 :
		default :
			pr_debug("%s: All-Release or Invalid combination\n",__func__);
			if(keyptr->event_wq){
				cancel_delayed_work_sync(&keyptr->event_work);
				flush_workqueue(keyptr->event_wq);
			}
//			mutex_lock(&keyptr->report_lock);
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_sync(input_dev);
//			mutex_unlock(&keyptr->report_lock);
			goto fake_end;
	}

	if(!keyptr->last_keycode){
//		mutex_lock(&keyptr->report_lock);
		pr_debug("%s: First notification\n",__func__);
		keyptr->q_loop_x = first_x;
		keyptr->q_loop_y = first_y;

		input_report_abs(input_dev, ABS_X, center_x);
		input_report_abs(input_dev, ABS_Y, center_y);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_sync(input_dev);

		input_report_abs(input_dev, ABS_X, keyptr->q_loop_x);
		input_report_abs(input_dev, ABS_Y, keyptr->q_loop_y);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_sync(input_dev);

		if(keyptr->event_wq)
			queue_delayed_work(keyptr->event_wq, &keyptr->event_work, msecs_to_jiffies(keyptr->move_init_interval));
//		mutex_unlock(&keyptr->report_lock);
	}

fake_end:
	mutex_unlock(&keyptr->report_lock);
	pr_debug("%s: -\n", __func__);
}

static void keyptr_event_work(struct work_struct *work)
{
	struct keyptr_data *keyptr = container_of(work, struct keyptr_data, event_work.work);
	struct input_dev *input_dev = keyptr->input_dev;
	int move;

	pr_debug("%s: Touch: keycode=%x\n", __func__, keyptr->keycode);

	mutex_lock(&keyptr->report_lock);

	move = keyptr->move_step;
	switch(keyptr->keycode){
		case FT_CENTER|FT_DOWN:
		case FT_DOWN   : keyptr->q_loop_y += move; break;
		case FT_CENTER|FT_UP:
		case FT_UP     : keyptr->q_loop_y -= move; break;
		case FT_CENTER|FT_RIGHT:
		case FT_RIGHT  : keyptr->q_loop_x += move; break;
		case FT_CENTER|FT_LEFT:
		case FT_LEFT   : keyptr->q_loop_x -= move; break;
		case FT_CENTER|FT_UP|FT_RIGHT:
		case FT_UP|FT_RIGHT   : keyptr->q_loop_x += move; keyptr->q_loop_y -= move; break;
		case FT_CENTER|FT_UP|FT_LEFT :
		case FT_UP|FT_LEFT    : keyptr->q_loop_x -= move; keyptr->q_loop_y -= move; break;
		case FT_CENTER|FT_DOWN|FT_RIGHT:
		case FT_DOWN|FT_RIGHT : keyptr->q_loop_x += move; keyptr->q_loop_y += move; break;
		case FT_CENTER|FT_DOWN|FT_LEFT:
		case FT_DOWN|FT_LEFT  : keyptr->q_loop_x -= move; keyptr->q_loop_y += move; break;
		case FT_CENTER : break;
		default :
			pr_debug("%s: keycode = %d\n",__func__, keyptr->keycode);
			goto end;
	}
	input_report_abs(input_dev, ABS_X, keyptr->q_loop_x);
	input_report_abs(input_dev, ABS_Y, keyptr->q_loop_y);
	input_report_key(input_dev, BTN_TOUCH, 1);
	input_sync(input_dev);
	if(keyptr->event_wq)
		queue_delayed_work(keyptr->event_wq, &keyptr->event_work, msecs_to_jiffies(keyptr->move_interval));
end:
	mutex_unlock(&keyptr->report_lock);

}

int keyptr_get_dt(struct device_node *np, struct keyptr_data *keyptr)
{
	int ret = 0;
	u32 tmp;

	if( (!np) | (!keyptr) ) {
		pr_err("%s null\n",__func__);
		return 1;
	}

	keyptr->enabled = of_property_read_bool(np,
		"kc,enabled");
	if (!keyptr->enabled) {
		pr_err("%s: Key pointer is disabled\n", __func__);
		return 1;
	}

	ret = of_property_read_u32(np, "kc,panel-width", &tmp);
	if (ret) {
		pr_err("%s end - fail get panel width\n",__func__);
		return 1;
	}
	keyptr->panel_width = tmp;

	ret = of_property_read_u32(np, "kc,panel-height", &tmp);
	if (ret) {
		pr_err("%s end - fail get panel height\n",__func__);
		return 1;
	}
	keyptr->panel_height = tmp;

	ret = of_property_read_u32(np, "kc,move-step", &tmp);
	if (ret) {
		pr_err("%s end - fail get move step\n",__func__);
		keyptr->move_step = 10;
	} else {
		keyptr->move_step = tmp;
	}

	ret = of_property_read_u32(np, "kc,move-init-interval", &tmp);
	if (ret) {
		pr_err("%s end - fail get move init interval\n",__func__);
		keyptr->move_init_interval = 50;
	} else {
		keyptr->move_init_interval = tmp;
	}

	ret = of_property_read_u32(np, "kc,move-interval", &tmp);
	if (ret) {
		pr_err("%s end - fail get move interval\n",__func__);
		keyptr->move_interval = 15;
	} else {
		keyptr->move_interval = tmp;
	}

	pr_err("%s: panel_width=%d panel_height=%d move_step=%d move_init_interval=%d move_interval=%d\n",
			__func__, keyptr->panel_width, keyptr->panel_height, keyptr->move_step,
			keyptr->move_init_interval, keyptr->move_interval);

	return ret;
}

static ssize_t keyptr_onoff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct keyptr_data *keyptr = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", keyptr->onoff);
}

static ssize_t keyptr_onoff_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct keyptr_data *keyptr = dev_get_drvdata(dev);
	int rc = 0;

	rc = kstrtoint(buf, 10, &keyptr->onoff);
	if (rc) {
		pr_err("kstrtoint failed. rc=%d\n", rc);
		return rc;
	}

	return count;
}

static DEVICE_ATTR(onoff, S_IRUGO | S_IWUSR,
	keyptr_onoff_show, keyptr_onoff_store);

static struct attribute *keyptr_attrs[] = {
	&dev_attr_onoff.attr,
	NULL,
};

static struct attribute_group keyptr_attr_group = {
	.attrs = keyptr_attrs,
};

static const struct file_operations keyptr_fops = {
	.owner = THIS_MODULE,
};

static int keyptr_init_sysfs(struct keyptr_data *keyptr)
{
	int ret = 0;

	ret = alloc_chrdev_region(&keyptr->dev_num, 0, 1, DRIVER_NAME);
	if (ret  < 0) {
		pr_err("alloc_chrdev_region failed ret = %d\n", ret);
		goto done;
	}

	keyptr->class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(keyptr->class)) {
		pr_err("%s class_create error!\n",__func__);
		goto done;
	}

	keyptr->device = device_create(keyptr->class, NULL,
		keyptr->dev_num, NULL, DRIVER_NAME);
	if (IS_ERR(keyptr->device)) {
		ret = PTR_ERR(keyptr->device);
		pr_err("device_create failed %d\n", ret);
		goto done;
	}

	dev_set_drvdata(keyptr->device, keyptr);

	cdev_init(&keyptr->cdev, &keyptr_fops);
	ret = cdev_add(&keyptr->cdev,
			MKDEV(MAJOR(keyptr->dev_num), 0), 1);
	if (ret < 0) {
		pr_err("cdev_add failed %d\n", ret);
		goto done;
	}

	ret = sysfs_create_group(&keyptr->device->kobj,
			&keyptr_attr_group);
	if (ret)
		pr_err("unable to register rotator sysfs nodes\n");

done:
	return ret;
}

static int keyptr_probe(struct platform_device *pdev)
{
	struct keyptr_data *keyptr;
	struct input_dev *input_dev;
	int err;
	int ret = 0;

	pr_notice("%s: +\n", __func__);

	keyptr = kzalloc(sizeof(struct keyptr_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!keyptr || !input_dev) {
		err = -ENOMEM;
		pr_err("%s Fail to null\n",__func__);
		goto err_free_mem;
	}

	input_dev->name		= "keyptr";
	input_dev->id.bustype	= BUS_HOST;
	input_dev->dev.parent	= &pdev->dev;

	keyptr->input_dev = input_dev;
	keyptr->keycode = 0;
	keyptr->last_keycode = 0;
	keyptr->q_loop_x = 0;
	keyptr->q_loop_y = 0;
	keyptr->onoff = 0;
	keyptr->enabled = 0;

	ret = keyptr_get_dt( pdev->dev.of_node, keyptr);
	if (ret) {
		keyptr->enabled = 0;
		pr_err("%s key pointer disable\n",__func__);
		goto err_free_mem;
	}

	mutex_init(&keyptr->report_lock);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_POINTER, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_X,	0, keyptr->panel_width, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,	0, keyptr->panel_height, 0, 0);
	input_set_drvdata(input_dev, keyptr);

	keyptr->event_wq = alloc_workqueue("kckeypter_event_wq", WQ_MEM_RECLAIM, 1);
	if ( !keyptr->event_wq ){
		pr_err("%s: Fail to allocate workqueue!\n", __func__);
		goto err_free_mem;
	}
	INIT_DELAYED_WORK(&keyptr->event_work, keyptr_event_work);

	err = input_register_device(keyptr->input_dev);
	if (err) {
		pr_err("%s: Fail to input_register_device!\n", __func__);
		goto err_free_wq;
	}

	platform_set_drvdata(pdev, keyptr);

#ifdef KEYPTR_KEYMATRIX_ENABLE
	matrix_keyptr_set_info(keyptr);
#endif

	keyptr_init_sysfs(keyptr);

	pr_notice("%s: -\n", __func__);
	return 0;


err_free_wq:
	if(keyptr->event_wq){
		cancel_delayed_work_sync(&keyptr->event_work);
		flush_workqueue(keyptr->event_wq);
	}
	destroy_workqueue(keyptr->event_wq);

err_free_mem:
#ifdef KEYPTR_KEYMATRIX_ENABLE
	matrix_keyptr_set_info(NULL);
#endif
	input_free_device(input_dev);
	kfree(keyptr);
	return err;
}

static int keyptr_remove(struct platform_device *pdev)
{
	struct keyptr_data *keyptr = platform_get_drvdata(pdev);;

	device_init_wakeup(&pdev->dev, 0);
	mutex_destroy(&keyptr->report_lock);
	input_unregister_device(keyptr->input_dev);
	kfree(keyptr);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id keyptr_of_match[] = {
	{ .compatible = "kc,keyptr", },
};

static struct platform_driver keyptr_driver = {
	.driver = {
		.name = "keyptr",
		.owner = THIS_MODULE,
		.of_match_table = keyptr_of_match,
	},
	.probe = keyptr_probe,
	.remove = keyptr_remove,
};

int __init keyptr_init(void)
{
	pr_err("%s: \n", __func__);
	return platform_driver_register(&keyptr_driver);
}

static void __exit keyptr_exit(void)
{
	platform_driver_unregister(&keyptr_driver);
}

module_init(keyptr_init);
module_exit(keyptr_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("KEYPOINTER Driver");
MODULE_LICENSE("GPL");
