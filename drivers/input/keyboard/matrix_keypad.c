/*
 *  GPIO driven matrix keyboard driver
 *
 *  Copyright (c) 2008 Marek Vasut <marek.vasut@gmail.com>
 *  Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 *  Based on corgikbd.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input/matrix_keypad.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <linux/key_dm_driver.h>

#ifndef QUALCOMM_ORIGINAL_FEATURE
#include <linux/wakelock.h>
#include <linux/gpio_keys.h>

#define IRQS_PENDING 0x00000200
#define istate core_internal_state__do_not_mess_with_it
struct wake_lock matrix_keypad_wake_lock;
struct wake_lock matrix_keypad_scan_wake_lock;
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

struct matrix_keypad {
	const struct matrix_keypad_platform_data *pdata;
	struct input_dev *input_dev;
	unsigned int row_shift;

	DECLARE_BITMAP(disabled_gpios, MATRIX_MAX_ROWS);

	uint32_t last_key_state[MATRIX_MAX_COLS];
	struct delayed_work work;
	struct mutex lock;
	bool scan_pending;
	bool stopped;
	bool gpio_all_disabled;
#ifndef QUALCOMM_ORIGINAL_FEATURE
	struct pinctrl *matrix_keypad_pinctrl;
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */
};

#ifndef QUALCOMM_ORIGINAL_FEATURE
#define KEYPAD_GROUP_NUM 4

static int matrix_keypad_key_group[MATRIX_MAX_ROWS][MATRIX_MAX_COLS] =
{
	{ 0,0,0,0,0,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 0,0,0,0,0,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 0,0,0,0,0,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,1,2,2,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,2,2,3,3,3,3,3,3,3,3,3,3, },
	{ 0,0,0,0,0,1,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
	{ 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3, },
 };
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

#ifdef QUALCOMM_ORIGINAL_FEATURE
#define MATRIX_KEYPAD_ERR_LOG_PRINT(fmt, ...)
#define MATRIX_KEYPAD_DEBUG_LOG_PRINT(fmt, ...)
#define MATRIX_KEYPAD_PR_LOG_PRINT(fmt, args...)
#else
static bool matrix_keypad_debug;
module_param(matrix_keypad_debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(matrix_keypad_debug, "matrix_keypad debug messages");

#define MATRIX_KEYPAD_ERR_LOG_PRINT(fmt, ...) printk(KERN_ERR "%s: " fmt, __func__, ##__VA_ARGS__)
#define MATRIX_KEYPAD_DEBUG_LOG_PRINT(fmt, ...) printk(KERN_DEBUG "%s: " fmt, __func__, ##__VA_ARGS__)
#define MATRIX_KEYPAD_PR_LOG_PRINT(fmt, args...)    do { if (matrix_keypad_debug)	     \
                                                         printk(KERN_DEBUG "%s: " fmt, __func__, ##args); \
                                                   } while(0)
#endif

static bool g_kdm_matrixkey_check = false;

#ifndef QUALCOMM_ORIGINAL_FEATURE
static int matrix_keypad_pinctrl_configure(struct matrix_keypad *keypad,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(keypad->matrix_keypad_pinctrl,
						"matrix_keypad_active");
		if (IS_ERR(set_state)) {
			MATRIX_KEYPAD_ERR_LOG_PRINT("cannot get ts pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state =
			pinctrl_lookup_state(keypad->matrix_keypad_pinctrl,
						"matrix_keypad_prescan");
		if (IS_ERR(set_state)) {
			MATRIX_KEYPAD_ERR_LOG_PRINT("cannot get gpiokey pinctrl prescan state\n");

			printk(KERN_ERR "%s: cannot get gpiokey pinctrl prescan state\n", __func__);
			return PTR_ERR(set_state);
		}
	}
	retval = pinctrl_select_state(keypad->matrix_keypad_pinctrl, set_state);
	if (retval) {
		MATRIX_KEYPAD_ERR_LOG_PRINT("cannot set ts pinctrl state\n");
		return retval;
	}

	return 0;
}
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

/*
 * NOTE: normally the GPIO has to be put into HiZ when de-activated to cause
 * minmal side effect when scanning other columns, here it is configured to
 * be input, and it should work on most platforms.
 */
static void __activate_col(const struct matrix_keypad_platform_data *pdata,
			   int col, bool on)
{
	bool level_on = !pdata->active_low;

	if (on) {
		gpio_direction_output(pdata->col_gpios[col], level_on);
	} else {
#ifdef QUALCOMM_ORIGINAL_FEATURE
		gpio_set_value_cansleep(pdata->col_gpios[col], !level_on);
#else
		gpio_direction_input(pdata->col_gpios[col]);
		gpio_direction_output(pdata->col_gpios[col], !level_on);
    	if (pdata->col_scan_delay_us)
    		udelay(pdata->col_scan_delay_us);
#endif
		gpio_direction_input(pdata->col_gpios[col]);
	}
}

static void activate_col(const struct matrix_keypad_platform_data *pdata,
			 int col, bool on)
{
	__activate_col(pdata, col, on);

	if (on && pdata->col_scan_delay_us)
		udelay(pdata->col_scan_delay_us);
}

static void activate_all_cols(const struct matrix_keypad_platform_data *pdata,
			      bool on)
{
	int col;

	for (col = 0; col < pdata->num_col_gpios; col++)
		__activate_col(pdata, col, on);
}

static bool row_asserted(const struct matrix_keypad_platform_data *pdata,
			 int row)
{
	return gpio_get_value_cansleep(pdata->row_gpios[row]) ?
			!pdata->active_low : pdata->active_low;
}

static void enable_row_irqs(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i;

	if (pdata->clustered_irq > 0)
		enable_irq(pdata->clustered_irq);
	else {
		for (i = 0; i < pdata->num_row_gpios; i++)
			enable_irq(gpio_to_irq(pdata->row_gpios[i]));
	}
}

static void disable_row_irqs(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i;

	if (pdata->clustered_irq > 0)
		disable_irq_nosync(pdata->clustered_irq);
	else {
		for (i = 0; i < pdata->num_row_gpios; i++)
			disable_irq_nosync(gpio_to_irq(pdata->row_gpios[i]));
	}
}

unsigned char matrixkey_cmd(unsigned char cmd, int *val)
{
	uint8_t ret = 1;

	MATRIX_KEYPAD_DEBUG_LOG_PRINT("cmd:%d\n", cmd);
	switch (cmd) {
	case KEY_DM_CHECK_COMMAND:
		MATRIX_KEYPAD_DEBUG_LOG_PRINT("%x %x\n",g_kdm_matrixkey_check, val[0]);
		if (val[0]) {
			g_kdm_matrixkey_check = true;
		} else {
			g_kdm_matrixkey_check = false;
		}
		ret = 0;
		break;

	default:
		MATRIX_KEYPAD_ERR_LOG_PRINT("%d\n", cmd);
		break;
	}
	return ret;
}
EXPORT_SYMBOL(matrixkey_cmd);

/*
 * This gets the keys from keyboard and reports it to input subsystem
 */
static void matrix_keypad_scan(struct work_struct *work)
{
	struct matrix_keypad *keypad =
		container_of(work, struct matrix_keypad, work.work);
	struct input_dev *input_dev = keypad->input_dev;
	const unsigned short *keycodes = input_dev->keycode;
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	uint32_t new_state[MATRIX_MAX_COLS];
	int row, col, code;
#ifndef QUALCOMM_ORIGINAL_FEATURE
	int count = 0;
    struct irq_desc *desc;
    int gcnt[KEYPAD_GROUP_NUM], i;
    bool folder_state;
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

    MATRIX_KEYPAD_PR_LOG_PRINT("start \n");

#ifndef QUALCOMM_ORIGINAL_FEATURE
    if(!keypad->scan_pending){
         keypad->scan_pending = true;
    }

    folder_state = gpio_keys_is_stateon(SW_LID);

    if(!folder_state){
    if(keypad->matrix_keypad_pinctrl){
        matrix_keypad_pinctrl_configure(keypad, false);
    }
    else{
        MATRIX_KEYPAD_ERR_LOG_PRINT("matrix_keypad_pinctrl 1st err %p\n", keypad->matrix_keypad_pinctrl);
    }

    for(i=0;i<KEYPAD_GROUP_NUM;i++)
        gcnt[i]=0;
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

	/* de-activate all columns for scanning */
	activate_all_cols(pdata, false);

	memset(new_state, 0, sizeof(new_state));

	/* assert each column and read the row status out */
	for (col = 0; col < pdata->num_col_gpios; col++) {

		activate_col(pdata, col, true);

		for (row = 0; row < pdata->num_row_gpios; row++)
#ifdef QUALCOMM_ORIGINAL_FEATURE
			new_state[col] |=
				row_asserted(pdata, row) ? (1 << row) : 0;
#else
			if( row_asserted(pdata, row) ){
				count++;
				gcnt[matrix_keypad_key_group[row][col]]++;
				new_state[col] |= (1 << row);
			}
#endif

		activate_col(pdata, col, false);
	}

#ifndef QUALCOMM_ORIGINAL_FEATURE
	if( (count != 0) && !(keypad->stopped) && !gpio_keys_is_stateon(SW_LID))
		wake_lock(&matrix_keypad_scan_wake_lock);

	for (col = 0; col < pdata->num_col_gpios; col++) {
		for (row = 0; row < pdata->num_row_gpios; row++) {
			switch (matrix_keypad_key_group[row][col]) {
				case 0:
					if(gcnt[0] > 2) {
						if( keypad->last_key_state[col] & (1 << row) )
							new_state[col] |= (1 << row);
						else
							new_state[col] &= ~(1 << row);

						continue;
					}
					break;

				case 1:
					break;

				case 2:
					if(gcnt[2] > 2) {
						if( keypad->last_key_state[col] & (1 << row) )
							new_state[col] |= (1 << row);
						else
							new_state[col] &= ~(1 << row);

						continue;
					}
					break;

				case 3:
					break;

				default:
					break;
			}
		}
	}
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

	for (col = 0; col < pdata->num_col_gpios; col++) {
		uint32_t bits_changed;

		bits_changed = keypad->last_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
			continue;

		for (row = 0; row < pdata->num_row_gpios; row++) {
			if ((bits_changed & (1 << row)) == 0)
				continue;

			code = MATRIX_SCAN_CODE(row, col, keypad->row_shift);
#ifdef QUALCOMM_ORIGINAL_FEATURE
			input_event(input_dev, EV_MSC, MSC_SCAN, code);
#endif /* QUALCOMM_ORIGINAL_FEATURE */
			if (g_kdm_matrixkey_check) {
				if((new_state[col] & (1 << row))){
					key_set_code(keycodes[code]);
				}
			} else {
#ifndef QUALCOMM_ORIGINAL_FEATURE
            wake_lock_timeout(&matrix_keypad_wake_lock,HZ);
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */
			input_report_key(input_dev,
					 keycodes[code],
					 new_state[col] & (1 << row));
            MATRIX_KEYPAD_DEBUG_LOG_PRINT("input_report_key code %d %d  \n", keycodes[code], new_state[col] & (1 << row));
            }
		}
	}
	if (!g_kdm_matrixkey_check) {
	input_sync(input_dev);
	}

	memcpy(keypad->last_key_state, new_state, sizeof(new_state));

#ifdef QUALCOMM_ORIGINAL_FEATURE
	activate_all_cols(pdata, true);
#else
    if(keypad->matrix_keypad_pinctrl){
        matrix_keypad_pinctrl_configure(keypad, true);
    }
    else{
        MATRIX_KEYPAD_ERR_LOG_PRINT("matrix_keypad_pinctrl 2st err %p\n", keypad->matrix_keypad_pinctrl);
    }
#endif
#ifndef QUALCOMM_ORIGINAL_FEATURE
	for (row = 0; row < pdata->num_row_gpios; row++) {
		for (col = 0; col < pdata->num_col_gpios; col++) {
			if ((keypad->last_key_state[col] & (1 << row))){
        		desc = irq_to_desc(gpio_to_irq(pdata->row_gpios[row]));
                desc->istate &= ~IRQS_PENDING;
                break;
            }
		}
	}
	}
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */
	mutex_lock(&keypad->lock);
#ifndef QUALCOMM_ORIGINAL_FEATURE
	if(count==0 || keypad->stopped || gpio_keys_is_stateon(SW_LID)){
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */
	keypad->scan_pending = false;
	enable_row_irqs(keypad);
	wake_unlock(&matrix_keypad_scan_wake_lock);
    MATRIX_KEYPAD_PR_LOG_PRINT("end cnt = %d stop = %d fd = %d \n", count, keypad->stopped, gpio_keys_is_stateon(SW_LID));
#ifndef QUALCOMM_ORIGINAL_FEATURE
	}else {
		schedule_delayed_work(&keypad->work,
			msecs_to_jiffies(keypad->pdata->debounce_ms));
        MATRIX_KEYPAD_PR_LOG_PRINT("polling  cnt = %d stop = %d fd = %d \n", count, keypad->stopped, gpio_keys_is_stateon(SW_LID));
	}
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */
	mutex_unlock(&keypad->lock);
}

static irqreturn_t matrix_keypad_interrupt(int irq, void *id)
{
	struct matrix_keypad *keypad = id;

    MATRIX_KEYPAD_PR_LOG_PRINT(" %d\n", irq);
	mutex_lock(&keypad->lock);

	/*
	 * See if another IRQ beaten us to it and scheduled the
	 * scan already. In that case we should not try to
	 * disable IRQs again.
	 */
#ifdef QUALCOMM_ORIGINAL_FEATURE
	if (unlikely(keypad->scan_pending || keypad->stopped))
		goto out;
#else
	if (unlikely(keypad->scan_pending || keypad->stopped)){
        MATRIX_KEYPAD_PR_LOG_PRINT(" %d pending \n", irq);
		goto out;
    }
#endif

	disable_row_irqs(keypad);
	keypad->scan_pending = true;
	schedule_delayed_work(&keypad->work,
		msecs_to_jiffies(keypad->pdata->debounce_ms));

out:
	mutex_unlock(&keypad->lock);
	return IRQ_HANDLED;
}

static int matrix_keypad_start(struct input_dev *dev)
{
	struct matrix_keypad *keypad = input_get_drvdata(dev);

	keypad->stopped = false;
	mb();

	/*
	 * Schedule an immediate key scan to capture current key state;
	 * columns will be activated and IRQs be enabled after the scan.
	 */
	schedule_delayed_work(&keypad->work, 0);

	return 0;
}

static void matrix_keypad_stop(struct input_dev *dev)
{
	struct matrix_keypad *keypad = input_get_drvdata(dev);

	keypad->stopped = true;
	mb();
#ifdef QUALCOMM_ORIGINAL_FEATURE
	flush_work(&keypad->work.work);
#else
	flush_delayed_work(&keypad->work);
	msleep(2 * keypad->pdata->debounce_ms);
#endif
	/*
	 * matrix_keypad_scan() will leave IRQs enabled;
	 * we should disable them now.
	 */
	disable_row_irqs(keypad);
}

#ifdef CONFIG_PM_SLEEP
static void matrix_keypad_enable_wakeup(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	unsigned int gpio;
	int i;

	if (pdata->clustered_irq > 0) {
		if (enable_irq_wake(pdata->clustered_irq) == 0)
			keypad->gpio_all_disabled = true;
	} else {

		for (i = 0; i < pdata->num_row_gpios; i++) {
			if (!test_bit(i, keypad->disabled_gpios)) {
				gpio = pdata->row_gpios[i];

				if (enable_irq_wake(gpio_to_irq(gpio)) == 0)
					__set_bit(i, keypad->disabled_gpios);
			}
		}
	}
}

static void matrix_keypad_disable_wakeup(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	unsigned int gpio;
	int i;

	if (pdata->clustered_irq > 0) {
		if (keypad->gpio_all_disabled) {
			disable_irq_wake(pdata->clustered_irq);
			keypad->gpio_all_disabled = false;
		}
	} else {
		for (i = 0; i < pdata->num_row_gpios; i++) {
			if (test_and_clear_bit(i, keypad->disabled_gpios)) {
				gpio = pdata->row_gpios[i];
				disable_irq_wake(gpio_to_irq(gpio));
			}
		}
	}
}

static int matrix_keypad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);
#ifndef QUALCOMM_ORIGINAL_FEATURE
	int folder_status;
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */


    MATRIX_KEYPAD_PR_LOG_PRINT("\n");

	matrix_keypad_stop(keypad->input_dev);

#ifdef QUALCOMM_ORIGINAL_FEATURE
	if (device_may_wakeup(&pdev->dev))
		matrix_keypad_enable_wakeup(keypad);
#else
	folder_status = gpio_keys_is_stateon(SW_LID);
    if (device_may_wakeup(&pdev->dev) && !folder_status){
		MATRIX_KEYPAD_DEBUG_LOG_PRINT("enable_wakeup %d\n", folder_status);

		matrix_keypad_enable_wakeup(keypad);
    }
    else{
		MATRIX_KEYPAD_DEBUG_LOG_PRINT(" Not enable_wakeup %d\n", folder_status);
    }
#endif

	return 0;
}

static int matrix_keypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);

    MATRIX_KEYPAD_PR_LOG_PRINT("\n");

	if (device_may_wakeup(&pdev->dev))
		matrix_keypad_disable_wakeup(keypad);

	matrix_keypad_start(keypad->input_dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(matrix_keypad_pm_ops,
			 matrix_keypad_suspend, matrix_keypad_resume);

static int matrix_keypad_init_gpio(struct platform_device *pdev,
				   struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i, err;

	/* initialized strobe lines as outputs, activated */
	for (i = 0; i < pdata->num_col_gpios; i++) {
		err = gpio_request(pdata->col_gpios[i], "matrix_kbd_col");
		if (err) {
			dev_err(&pdev->dev,
				"failed to request GPIO%d for COL%d\n",
				pdata->col_gpios[i], i);
			goto err_free_cols;
		}

		gpio_direction_output(pdata->col_gpios[i], !pdata->active_low);
	}

	for (i = 0; i < pdata->num_row_gpios; i++) {
		err = gpio_request(pdata->row_gpios[i], "matrix_kbd_row");
		if (err) {
			dev_err(&pdev->dev,
				"failed to request GPIO%d for ROW%d\n",
				pdata->row_gpios[i], i);
			goto err_free_rows;
		}

		gpio_direction_input(pdata->row_gpios[i]);
	}

	if (pdata->clustered_irq > 0) {
		err = request_irq(pdata->clustered_irq,
				matrix_keypad_interrupt,
				pdata->clustered_irq_flags,
				"matrix-keypad", keypad);
		if (err < 0) {
			dev_err(&pdev->dev,
				"Unable to acquire clustered interrupt\n");
			goto err_free_rows;
		}
	} else {
		for (i = 0; i < pdata->num_row_gpios; i++) {
			err = request_threaded_irq(
					gpio_to_irq(pdata->row_gpios[i]),
					NULL,
					matrix_keypad_interrupt,
					IRQF_DISABLED | IRQF_ONESHOT |
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"matrix-keypad", keypad);
			if (err < 0) {
				dev_err(&pdev->dev,
					"Unable to acquire interrupt for GPIO line %i\n",
					pdata->row_gpios[i]);
				goto err_free_irqs;
			}
		}
	}

	/* initialized as disabled - enabled by input->open */
	disable_row_irqs(keypad);
	return 0;

err_free_irqs:
	while (--i >= 0)
		free_irq(gpio_to_irq(pdata->row_gpios[i]), keypad);
	i = pdata->num_row_gpios;
err_free_rows:
	while (--i >= 0)
		gpio_free(pdata->row_gpios[i]);
	i = pdata->num_col_gpios;
err_free_cols:
	while (--i >= 0)
		gpio_free(pdata->col_gpios[i]);

	return err;
}

static void matrix_keypad_free_gpio(struct matrix_keypad *keypad)
{
	const struct matrix_keypad_platform_data *pdata = keypad->pdata;
	int i;

	if (pdata->clustered_irq > 0) {
		free_irq(pdata->clustered_irq, keypad);
	} else {
		for (i = 0; i < pdata->num_row_gpios; i++)
			free_irq(gpio_to_irq(pdata->row_gpios[i]), keypad);
	}

	for (i = 0; i < pdata->num_row_gpios; i++)
		gpio_free(pdata->row_gpios[i]);

	for (i = 0; i < pdata->num_col_gpios; i++)
		gpio_free(pdata->col_gpios[i]);
}

#ifdef CONFIG_OF
static struct matrix_keypad_platform_data *
matrix_keypad_parse_dt(struct device *dev)
{
	struct matrix_keypad_platform_data *pdata;
	struct device_node *np = dev->of_node;
	unsigned int *gpios;
	int i, nrow, ncol;

	if (!np) {
		dev_err(dev, "device lacks DT data\n");
		return ERR_PTR(-ENODEV);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	pdata->num_row_gpios = nrow = of_gpio_named_count(np, "row-gpios");
	pdata->num_col_gpios = ncol = of_gpio_named_count(np, "col-gpios");
	if (nrow <= 0 || ncol <= 0) {
		dev_err(dev, "number of keypad rows/columns not specified\n");
		return ERR_PTR(-EINVAL);
	}

	if (of_get_property(np, "linux,no-autorepeat", NULL))
		pdata->no_autorepeat = true;
	if (of_get_property(np, "linux,wakeup", NULL))
		pdata->wakeup = true;
	if (of_get_property(np, "gpio-activelow", NULL))
		pdata->active_low = true;

	of_property_read_u32(np, "debounce-delay-ms", &pdata->debounce_ms);
	of_property_read_u32(np, "col-scan-delay-us",
						&pdata->col_scan_delay_us);

	gpios = devm_kzalloc(dev,
			     sizeof(unsigned int) *
				(pdata->num_row_gpios + pdata->num_col_gpios),
			     GFP_KERNEL);
	if (!gpios) {
		dev_err(dev, "could not allocate memory for gpios\n");
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < pdata->num_row_gpios; i++)
		gpios[i] = of_get_named_gpio(np, "row-gpios", i);

	for (i = 0; i < pdata->num_col_gpios; i++)
		gpios[pdata->num_row_gpios + i] =
			of_get_named_gpio(np, "col-gpios", i);

	pdata->row_gpios = gpios;
	pdata->col_gpios = &gpios[pdata->num_row_gpios];

	return pdata;
}
#else
static inline struct matrix_keypad_platform_data *
matrix_keypad_parse_dt(struct device *dev)
{
	dev_err(dev, "no platform data defined\n");

	return ERR_PTR(-EINVAL);
}
#endif

static int matrix_keypad_probe(struct platform_device *pdev)
{
	const struct matrix_keypad_platform_data *pdata;
	struct matrix_keypad *keypad;
	struct input_dev *input_dev;
	int err;

    MATRIX_KEYPAD_PR_LOG_PRINT("start\n");

#ifndef QUALCOMM_ORIGINAL_FEATURE
    wake_lock_init(&matrix_keypad_wake_lock,WAKE_LOCK_SUSPEND,"matrix_keypad_wake_lock");
    wake_lock_init(&matrix_keypad_scan_wake_lock,WAKE_LOCK_SUSPEND,"matrix_keypad_scan_wake_lock");
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		pdata = matrix_keypad_parse_dt(&pdev->dev);
		if (IS_ERR(pdata)) {
			dev_err(&pdev->dev, "no platform data defined\n");
			return PTR_ERR(pdata);
		}
	} else if (!pdata->keymap_data) {
		dev_err(&pdev->dev, "no keymap data defined\n");
		return -EINVAL;
	}

	keypad = kzalloc(sizeof(struct matrix_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!keypad || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}


#ifndef QUALCOMM_ORIGINAL_FEATURE
	/* Get pinctrl if target uses pinctrl */
	keypad->matrix_keypad_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(keypad->matrix_keypad_pinctrl)) {
		if (PTR_ERR(keypad->matrix_keypad_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		MATRIX_KEYPAD_ERR_LOG_PRINT("Target does not use pinctrl\n");
		keypad->matrix_keypad_pinctrl = NULL;
	}

	if (keypad->matrix_keypad_pinctrl) {
		err = matrix_keypad_pinctrl_configure(keypad, true);
		if (err) {
			MATRIX_KEYPAD_ERR_LOG_PRINT("cannot set ts pinctrl active state\n");
			goto err_free_gpio;
		}
	}
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

	keypad->input_dev = input_dev;
	keypad->pdata = pdata;
	keypad->row_shift = get_count_order(pdata->num_col_gpios);
	keypad->stopped = true;
	INIT_DELAYED_WORK(&keypad->work, matrix_keypad_scan);
	mutex_init(&keypad->lock);

#ifdef QUALCOMM_ORIGINAL_FEATURE
	input_dev->name		= pdev->name;
#else
	input_dev->name		= "matrix_keypad.67";
#endif

	input_dev->id.bustype	= BUS_HOST;
	input_dev->dev.parent	= &pdev->dev;
	input_dev->open		= matrix_keypad_start;
	input_dev->close	= matrix_keypad_stop;

	err = matrix_keypad_build_keymap(pdata->keymap_data, NULL,
					 pdata->num_row_gpios,
					 pdata->num_col_gpios,
					 NULL, input_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		goto err_free_mem;
	}

#ifndef QUALCOMM_ORIGINAL_FEATURE
	__set_bit(INPUT_PROP_NO_DUMMY_RELEASE, input_dev->propbit);
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

	if (!pdata->no_autorepeat)
		__set_bit(EV_REP, input_dev->evbit);
#ifdef QUALCOMM_ORIGINAL_FEATURE
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
#endif /* QUALCOMM_ORIGINAL_FEATURE */
	input_set_drvdata(input_dev, keypad);

	err = matrix_keypad_init_gpio(pdev, keypad);
	if (err)
		goto err_free_mem;

	err = input_register_device(keypad->input_dev);
	if (err)
		goto err_free_gpio;

	device_init_wakeup(&pdev->dev, pdata->wakeup);
	platform_set_drvdata(pdev, keypad);

    MATRIX_KEYPAD_PR_LOG_PRINT("end\n");
	return 0;

err_free_gpio:
	matrix_keypad_free_gpio(keypad);
err_free_mem:
	input_free_device(input_dev);
	kfree(keypad);
	return err;
}

static int matrix_keypad_remove(struct platform_device *pdev)
{
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);

#ifndef QUALCOMM_ORIGINAL_FEATURE
    wake_lock_destroy(&matrix_keypad_wake_lock);
    wake_lock_destroy(&matrix_keypad_scan_wake_lock);
#endif /* Not QUALCOMM_ORIGINAL_FEATURE */

	matrix_keypad_free_gpio(keypad);
	mutex_destroy(&keypad->lock);
	input_unregister_device(keypad->input_dev);
	kfree(keypad);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id matrix_keypad_dt_match[] = {
	{ .compatible = "gpio-matrix-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, matrix_keypad_dt_match);
#endif

static struct platform_driver matrix_keypad_driver = {
	.probe		= matrix_keypad_probe,
	.remove		= matrix_keypad_remove,
	.driver		= {
		.name	= "matrix-keypad",
		.owner	= THIS_MODULE,
		.pm	= &matrix_keypad_pm_ops,
		.of_match_table = of_match_ptr(matrix_keypad_dt_match),
	},
};
module_platform_driver(matrix_keypad_driver);

MODULE_AUTHOR("Marek Vasut <marek.vasut@gmail.com>");
MODULE_DESCRIPTION("GPIO Driven Matrix Keypad Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:matrix-keypad");
