/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 * drivers/input/touchscreen/pct13xx_ts.c
 *
 * PixArt pct13xx TouchScreen driver.
 *
 * Copyright (c) 2015 PixArt Imaging Inc.
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



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/input/mt.h>
#include <linux/async.h>
#include <linux/regulator/consumer.h>

#include <linux/fb.h>
#include "pct13xx_touch.h"
#include "pct13xx_ts.h"
#include "pct1332qn_fw.h"

#include <linux/moduleparam.h>

#define MODULE_NAME			"PCT13XX: "
#define MODULE_MOD_TIME		"20151230"
#define DRIVER_VERSION		"PixArt PCT13XX - v1.9"
#define PCT13XX_I2C_NAME	"pct13xx_ts"
#define PCT13XX_DEV_NAME	PCT13XX_I2C_NAME
#define PCT13XX_DTS_NAME	"pct,pct13xx_ts"

#define PCT13XX_REG_INT					0x50
#define PCT13XX_REG_RESET				0x65
#define PCT13XX_I2C_RETRY_MAX			(2)

#define GESTURE_SAMPLE

#define DIAG_TEST_RESULT_SIZE ((TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM) * 2)

/* Variable */
struct i2c_client *drv_client;
static struct sysfs_data user_cmd;
static uint16_t max_x, max_y;
static uint8_t max_finger, drive_num, sense_num;
static DEFINE_MUTEX(pix_mutex);
static int prob_err_state;
module_param(prob_err_state, int, S_IRUSR);

static int pct1332qn_raw_data_wnd(uint8_t drive_start, uint8_t drive_stop, uint8_t sense_start, uint8_t sense_stop);
static int pct13xx_get_rawdata_new(struct kc_ts_data *kd);
static int pct13xx_get_TouchDeltaData(struct kc_ts_data *kd);
static int pct13xx_confirm_device(struct pct13xx_data *pd, uint8_t type);
static int pct13xx_set_extended(struct pct13xx_data *pd);
static int pct13xx_enter_control(struct pct13xx_data *pd, uint8_t type);

static int i2c_read(uint8_t command, uint8_t *data, uint16_t length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = drv_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = drv_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	mutex_lock(&pix_mutex);
	for (retry = 0; retry < PCT13XX_I2C_RETRY_MAX; retry++) {
		if (i2c_transfer(drv_client->adapter, msg, 2) == 2)
			break;
	}
	mutex_unlock(&pix_mutex);
	if (retry == PCT13XX_I2C_RETRY_MAX) {
		TOUCH_E_LOG("i2c_read_block retry over %d", retry);
		return -EIO;
	}

	TOUCH_D_LOG("[TOUCH][%s][0x%02X] len[%d] data0[%02x]", __func__, command, length, data[0]);

	return 0;
}

static int i2c_write(uint8_t command, uint8_t data)
{
	uint8_t retry;
	uint8_t buf[2];

	struct i2c_msg msg[] = {
		{
			.addr = drv_client->addr,
			.flags = 0,
			.len = sizeof(buf),
			.buf = buf,
		}
	};

	TOUCH_D_LOG("[0x%02X:0x%02X]", command, data);

	buf[0] = command;
	buf[1] = data;
	mutex_lock(&pix_mutex);
	for (retry = 0; retry < PCT13XX_I2C_RETRY_MAX; retry++) {
		if (i2c_transfer(drv_client->adapter, msg, 1) == 1)
			break;
	}
	mutex_unlock(&pix_mutex);

	if (retry == PCT13XX_I2C_RETRY_MAX) {
		TOUCH_E_LOG("i2c_write retry over %d", retry);
		return -EIO;
	}

	return 0;
}

static int pct13xx_switch_bank(uint8_t target_bank)
{

	TOUCH_N_LOG("start bank:%d", target_bank);

	if(i2c_write(PCT13XX_REG_SWITCH_BANK, target_bank) < 0){
		TOUCH_E_LOG("switch bank fail due to write 0x7F");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_enable_handshaking(void)
{
	uint8_t temp = 0;
	int retry;

	TOUCH_N_LOG("start");

	if( pct13xx_switch_bank(0) < 0 ){
		pr_err("%s: handshaking enable fail due to swtich bank 0",__func__);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_HAND_SHAKING, PCT13XX_RW_ENABLE) < 0){
		pr_err("%s: handshaking enable fail due to write 0x7C",__func__);
		return -EIO;
	}

	retry = 50;
	do {
		i2c_read(PCT13XX_REG_HAND_SHAKING, &temp, 1);
		if (PCT13XX_DEV_READY == temp)
		{
			break;
		}
		else if(PCT13XX_RW_ENABLE != temp)
		{
			if(i2c_write(PCT13XX_REG_HAND_SHAKING, PCT13XX_RW_ENABLE) < 0){
				pr_err("%s: handshaking enable fail due to write 0x7C",__func__);
				return -EIO;
			}
		}
		usleep_range(10000, 10000);
	} while (--retry > 0);
	if (!retry){
		pr_err("%s: handshaking enable fail due to wait 0xA5. received: 0x%x",__func__, temp);
		return -EIO;
	}

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_disable_handshaking(void)
{
	TOUCH_N_LOG("start");

	if( pct13xx_switch_bank(0) < 0 ){
		pr_err("%s: handshaking disalbe fail due to swtich bank 0",__func__);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_HAND_SHAKING, PCT13XX_RW_DISABLE) < 0){
		pr_err("%s: handshaking disable fail due to write 0x7C",__func__);
		return -EIO;
	}

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_enable_handshaking_with_check(struct pct13xx_data *pd)
{
	TOUCH_N_LOG("start");

	mutex_lock(&pd->handshake_lock);
	if(!pd->handshake_stat) {
		if (pct13xx_enable_handshaking() < 0){
			TOUCH_E_LOG("fail due to enable handshaking");
			mutex_unlock(&pd->handshake_lock);
			return -EIO;
		}
	}
	else {
		TOUCH_N_LOG("Not enabled. already handshaking");
	}
	pd->handshake_stat++;
	mutex_unlock(&pd->handshake_lock);

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_disable_handshaking_with_check(struct pct13xx_data *pd)
{
	TOUCH_N_LOG("start");

	mutex_lock(&pd->handshake_lock);
	pd->handshake_stat--;
	if(!pd->handshake_stat) {
		if (pct13xx_disable_handshaking() < 0){
			TOUCH_E_LOG("fail due to disable handshaking");
			mutex_unlock(&pd->handshake_lock);
			return -EIO;
		}
	}
	else {
		TOUCH_N_LOG("Not disabled. num of needed: %d", pd->handshake_stat);
	}
	mutex_unlock(&pd->handshake_lock);

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_enable_handshaking_and_switch_bank6(struct pct13xx_data *pd)
{
	TOUCH_N_LOG("start");

	if( pct13xx_enable_handshaking_with_check(pd) < 0 ){
		TOUCH_E_LOG("fail due to enable handshaking with check");
		return -EIO;
	}
	if( pct13xx_switch_bank(6) < 0 ){
		TOUCH_E_LOG("fail due to swtich bank");
		return -EIO;
	}

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_clear_handshaking_state(struct pct13xx_data *pd)
{
	TOUCH_N_LOG("start");

	mutex_lock(&pd->handshake_lock);
	pd->handshake_stat = 0;
	mutex_unlock(&pd->handshake_lock);

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_check_bank_identifier(void)
{
	uint8_t temp = 0;
	if( pct13xx_switch_bank(5) < 0 ){
		pr_err("%s: check pid in bank5 fail due to swtich bank 5",__func__);
		return -EIO;
	}
	i2c_read(PCT13XX_REG_ANOTHER_PID, &temp, 1);
	if(PCT1332QN_PID == temp)
		return 0;
	else
		return -1;
}

static int pct13xx_reg_user_read(struct pct13xx_data *pd, uint8_t reg, uint8_t *data)
{

	TOUCH_N_LOG("start");

	if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
	{
		TOUCH_E_LOG("end %d", EIO);
		return -EIO;
	}
	if( i2c_write(PCT13XX_REG_USER_ADDR, reg) < 0){
		TOUCH_E_LOG("user read fail due to write 0x51");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	udelay(PCT13XX_DELAY_US_BETWEEN_I2C_REG);
	if( i2c_read(PCT13XX_REG_USER_DATA, data, 1) < 0){
		TOUCH_E_LOG("user read fail due to read 0x52");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	udelay(PCT13XX_DELAY_US_BETWEEN_I2C_REG);
	if(pct13xx_disable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("end %d", EIO);
		return -EIO;
	}

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_reg_user_write(struct pct13xx_data *pd, uint8_t reg, uint8_t data)
{

	TOUCH_N_LOG("start");

	if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
	{
		TOUCH_E_LOG("end %d", EIO);
		return -EIO;
	}
	if( i2c_write(PCT13XX_REG_USER_ADDR, reg) < 0){
		TOUCH_E_LOG("user write fail due to write 0x51");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	udelay(PCT13XX_DELAY_US_BETWEEN_I2C_REG);
	if( i2c_write(PCT13XX_REG_USER_DATA, data) < 0){
		TOUCH_E_LOG("user write fail due to write 0x52");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	udelay(PCT13XX_DELAY_US_BETWEEN_I2C_REG);
	if(pct13xx_disable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("end %d", EIO);
		return -EIO;
	}

	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_gpio_regulator_init(struct pct13xx_data *pd, struct device *dev)
{
	int rst_gpio = of_get_named_gpio(dev->of_node, "pct,reset-gpio", 0);
	int irq_gpio = of_get_named_gpio(dev->of_node, "pct,irq-gpio", 0);
	int rc = 0;

	pd->reset_gpio = rst_gpio;
	pd->irq_gpio = irq_gpio;
	pd->vdd_gpio = of_get_named_gpio(pd->dev->of_node, "pct,i2c-gpio", 0);

	rc = gpio_request(pd->vdd_gpio, "vtps_ldo_en");
	if (rc < 0) {
		dev_err(dev,
			"%s: Fail request gpio=%d\n", __func__,
			pd->vdd_gpio);
	}

	rc = gpio_request(rst_gpio, "pct13xx_rst");
	if (rc < 0) {
		gpio_free(rst_gpio);
		rc = gpio_request(rst_gpio, "pct13xx_rst");
	}
	if (rc < 0) {
		dev_err(dev,
			"%s: Fail request gpio=%d\n", __func__,
			rst_gpio);
	} else {
		rc = gpio_direction_output(rst_gpio, 0);
		if (rc < 0) {
			pr_err("%s: Fail set output gpio=%d\n",
				__func__, rst_gpio);
			gpio_free(rst_gpio);
		} else {
			rc = gpio_request(irq_gpio, "pct13xx_irq");
			if (rc < 0) {
				gpio_free(irq_gpio);
				rc = gpio_request(irq_gpio,
					"pct13xx_irq");
			}
			if (rc < 0) {
				dev_err(dev,
					"%s: Fail request gpio=%d\n",
					__func__, irq_gpio);
				gpio_free(rst_gpio);
			} else {
				gpio_direction_input(irq_gpio);
			}
		}
	}

	dev_info(dev, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int __pct13xx_device_power_on(struct pct13xx_data *pd)
{
	TOUCH_N_LOG("start");

	gpio_set_value(pd->reset_gpio, 1);
	pd->is_suspended = false;
	msleep(20);

	TOUCH_N_LOG("end");
	return 0;
}

static int __pct13xx_device_power_off(struct pct13xx_data *pd)
{
	TOUCH_N_LOG("start");

	gpio_set_value(pd->reset_gpio, 0);
	usleep_range(4000, 4000);

	TOUCH_N_LOG("end");
	return 0;
}

static int __pct13xx_device_reset(struct pct13xx_data *pd)
{
	int ret;
	uint8_t retry = DEVICE_RESET_RETRY_CNT;

	TOUCH_N_LOG("start");

	do{
		usleep_range(10000, 10000);
		gpio_set_value(pd->reset_gpio, 0);
		usleep_range(1000, 1000);
		gpio_set_value(pd->reset_gpio, 1);
		pd->is_suspended = false;
		msleep(20);
		if(pct13xx_clear_handshaking_state(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
		ret = pct13xx_confirm_device(pd, BOOT_NAV_READY);
		if( ret == 0 ){
			pct13xx_set_extended(pd);
			break;
		}
	} while (retry-- > 0);

	TOUCH_N_LOG("end");
	return ret;
}

static int pct13xx_cmd_resume(struct pct13xx_data *pd, bool sys)
{
	TOUCH_N_LOG("start: sys = %d", sys);

	if(sys){
		if(pct13xx_switch_bank(6) < 0) {
			TOUCH_E_LOG("fail due to switch bank 6");
			return -EIO;
		}
		i2c_write(PCT13XX_REG_SHUTDOWN, PCT13XX_RESUME);
		pd->is_suspended = false;
		usleep_range(5000,5000);
		pct13xx_enter_control(pd, BOOT_NAV_READY);
	}

	if (!pd->irq_enabled) {
		pct13xx_set_extended(pd);
		enable_irq(pd->client->irq);
		pd->irq_enabled = 1;
		if (pd->esd_wq) {
			queue_delayed_work(pd->esd_wq, &pd->esdwork,
					msecs_to_jiffies(ESD_POLLING_TIME));
		}
	}

	return 0;
}

static int pct13xx_cmd_suspend(struct pct13xx_data *pd, bool sys)
{
	TOUCH_N_LOG("start: sys = %d", sys);

	if(pd->is_suspended) {
		TOUCH_N_LOG("already suspended");
		return 0;
	}

	if(sys){
		if (pd->irq_enabled) {
			disable_irq_nosync(pd->client->irq);
			pd->irq_enabled = 0;
		}
		if (pd->esd_wq) {
			cancel_delayed_work_sync(&pd->esdwork);
			flush_workqueue(pd->esd_wq);
		}
	}

	if(pct13xx_switch_bank(6) < 0)
	{
		TOUCH_E_LOG("fail due to swtich bank");
		return -EIO;
	}
	i2c_write(PCT13XX_REG_SHUTDOWN, PCT13XX_SHUTDOWN);
	pd->is_suspended = true;
	if(pct13xx_clear_handshaking_state(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}

	TOUCH_N_LOG("end");

	return 0;
}

void pct13xx_resume_work(struct work_struct *work)
{
	struct pct13xx_data *pd = container_of(work, struct pct13xx_data, resumework);
	struct i2c_client *client = pd->client;

	TOUCH_N_LOG("start");

	mutex_lock(&pd->system_lock);
	usleep_range(1000,1000);

	i2c_pinctrl_set_active(client->adapter);
	usleep_range(9000,9000);

	__pct13xx_device_power_on(pd);
	pd->power = TOUCH_POWERON;

	pct13xx_confirm_device(pd, BOOT_NAV_READY);

	if(pd->app_power == TOUCH_POWERON)
		pct13xx_cmd_resume(pd, 0);
	else
		pct13xx_cmd_suspend(pd, 0);
	mutex_unlock(&pd->system_lock);

	TOUCH_N_LOG("end");
}

int pct13xx_trans_run_mode(struct pct13xx_data *pd, int order)
{
	TOUCH_N_LOG("start :order%d", order);
	if(!pd){
		TOUCH_E_LOG("Error: pct13xx_data is NULL");
		return -ENODEV;
	}

	mutex_lock(&pd->system_lock);

	if(order == 1){
		gpio_set_value(pd->vdd_gpio, 1);
		usleep_range(5000,5000);
		
	} else if(order == 2){
		if(likely(pd->resume_wq))
			queue_work(pd->resume_wq, &pd->resumework);
		else
			TOUCH_E_LOG("Error: resume_wq is Null");
	} else
		TOUCH_E_LOG("Error: Unexpected order = %d", order);

	mutex_unlock(&pd->system_lock);
	TOUCH_N_LOG("end");
	return 0;
}

int pct13xx_trans_shutdown_mode(struct pct13xx_data *pd, int order)
{
	struct i2c_client *client = pd->client;

	TOUCH_N_LOG("start :order%d", order);
	if(!pd){
		TOUCH_E_LOG("Error: pct13xx_data is NULL");
		return -ENODEV;
	}

	mutex_lock(&pd->system_lock);

	if(order == 1){
		if(pd->app_power == TOUCH_POWERON)
			pct13xx_cmd_suspend(pd, 1);

		__pct13xx_device_power_off(pd);
		pd->power = TOUCH_POWEROFF;
		i2c_pinctrl_set_default(client->adapter);
		usleep_range(1000,1000);
	} else if(order == 2){
		usleep_range(5000,5000);
		gpio_set_value(pd->vdd_gpio, 0);
	} else
		TOUCH_E_LOG("Error: Unexpected order = %d", order);

	mutex_unlock(&pd->system_lock);
	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_enter_control(struct pct13xx_data *pd, uint8_t type)
{
	uint8_t val = 0, retry, bit_event;

	TOUCH_N_LOG("start");

	switch (type) {
	case BOOT_COMPLETE:
		bit_event = PCT13XX_BOOT_READY;
		break;
	case BOOT_NAV_READY:
		bit_event = PCT13XX_CONTROL_READY | PCT13XX_BOOT_READY;
		break;

	default:
		WARN_ON(1);
		TOUCH_N_LOG("end %d", EINVAL);
		return -EINVAL;
	}

	if(pct13xx_enable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}
	retry = 38;
	do {
		if( pct13xx_switch_bank(6) < 0 ){
			TOUCH_E_LOG("enter control fail due to swtich bank");
			usleep_range(20000, 20000);
			continue;
		}
		i2c_read(PCT13XX_REG_BOOT_STA, &val, 1);
		if (IS_BIT_SET(val, bit_event))
			break;
		usleep_range(20000, 20000);
	} while (retry-- > 0);
	if(pct13xx_disable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}
	if(!retry){
		TOUCH_E_LOG("enter control fail due to wait boot stat");
		TOUCH_N_LOG("end %d", EIO);
		return -EBUSY;
	}

	TOUCH_N_LOG("end");
	return 0;
}

#if ENABLE_FW_DOWNLOAD

static int pct13xx_wait_boot(struct pct13xx_data *pd, uint8_t type, uint8_t enable_handshaking)
{
	int retry;
	uint8_t val = 0, bit_event;

	TOUCH_N_LOG("start");

	switch (type) {
	case BOOT_COMPLETE:
		bit_event = PCT13XX_BOOT_READY;
		break;

	default:
		WARN_ON(1);
		TOUCH_N_LOG("end %d", EINVAL);
		return -EINVAL;
	}

	/* wait for reset complete */
	if (enable_handshaking == 1)
	{
		if(pct13xx_enable_handshaking_with_check(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
	}
	retry = 50;
	do {
		if(pct13xx_switch_bank(6) < 0) {
			TOUCH_E_LOG("fail due to switch bank 6");
			usleep_range(15000, 15000);
			continue;
		}
		i2c_read(PCT13XX_REG_BOOT_STA, &val, 1);
		if (IS_BIT_SET(val, bit_event))
			break;
		usleep_range(15000, 15000);
	} while (retry-- > 0);
	if (enable_handshaking == 1)
	{
		if(pct13xx_disable_handshaking_with_check(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
	}
	if (!retry)
		goto err_boot_fail;

	TOUCH_N_LOG("end");
	return 0;

err_boot_fail:
	TOUCH_E_LOG("boot stat reg=0x%02x", val);

	TOUCH_N_LOG("end %d", EBUSY);
	return -EBUSY;
}

/*
 * There is no side-effect to write 0xAA/0xCC to 0x7A without
 * enable_handshaking
 * */
static int pct13xx_reset(struct pct13xx_data *pd, uint8_t type, uint8_t enable_handshaking)
{
	uint8_t reset;

	TOUCH_N_LOG("start");

	if(pct13xx_switch_bank(6) < 0) {
		TOUCH_E_LOG("fail due to switch bank 6");
		goto err_write_fail;
	}

	if (i2c_write(PCT13XX_REG_SHUTDOWN, PCT13XX_SHUTDOWN) < 0)
		goto err_write_fail;

	switch (type) {
	case POWER_HARD_RESET:
		reset = PCT13XX_HARD_RESET;
		break;
	default:
		WARN_ON(1);
		TOUCH_N_LOG("end %d", EINVAL);
		return -EINVAL;
	}

	if (i2c_write(PCT13XX_REG_SHUTDOWN, reset) < 0)
		goto err_write_fail;
	pd->is_suspended = false;
	if(pct13xx_clear_handshaking_state(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}

	msleep(20);

	/* wait for reset complete - retry 1 second */
	if (pct13xx_wait_boot(pd, BOOT_COMPLETE, enable_handshaking) < 0) {
		TOUCH_N_LOG("end %d", EBUSY);
		return -EBUSY;
	}

	TOUCH_N_LOG("end");
	return 0;

err_write_fail:
	TOUCH_E_LOG("write reg failed");

	TOUCH_N_LOG("end %d", ENODEV);
	return -ENODEV;
}

static int pct1332qn_write_sram(uint8_t *data, int sramDataOffset,int pageLen)
{
	int i = 0, len = 4096, index = 0;

	TOUCH_N_LOG("start");

	if(pct13xx_switch_bank(2) < 0){
		TOUCH_E_LOG("write sram fail due to switch bank");
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_SRAM_SEL, PCT13XX_SRAM_SEL) < 0){
		TOUCH_E_LOG("write sram fail due to write 0x09");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_ONE) <0){
		TOUCH_E_LOG("write sram fail due to write 0x0C(1)");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;

	}

	while(i < len)
	{
		if(i >= sramDataOffset && i < sramDataOffset + pageLen){
			if(i2c_write(PCT13XX_REG_SRAM_DATA, *(data + index)) < 0){
				TOUCH_E_LOG("write sram fail due to write 0x0D(1)");
				TOUCH_N_LOG("end %d", EIO);
				return -EIO;
			}

			index++;
		}
		else{
			if(i2c_write(PCT13XX_REG_SRAM_DATA, 0x00) < 0){
				TOUCH_E_LOG("write sram fail due to write 0x0D(2)");
				TOUCH_N_LOG("end %d", EIO);
				return -EIO;
			}
		}
		i++;
	}

	if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_TWO) <0){
		TOUCH_E_LOG("write sram fail due to write 0x0C(2)");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;

	}


	TOUCH_N_LOG("end");
	return 0;
}

static int pct1332qn_wirte_flash_from_sram(int address)
{
	int retry;
	uint8_t val = 0;

	TOUCH_N_LOG("start");

	if(pct13xx_switch_bank(1) < 0){
		TOUCH_E_LOG("write flash from sram fail due to switch bank");
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_FLASH_START, (uint8_t)address) < 0){
		TOUCH_E_LOG("write flash from sram fail due to write 0x55");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_FLASH_SIZE, PCT13XX_FLASH_SIZE) < 0){
		TOUCH_E_LOG("write flash from sram fail due to write 0x56");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_BIST_CONTROL, PCT13XX_BIST_ONE) < 0){
		TOUCH_E_LOG("write flash from sram fail due to write 0x50(1)");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	retry = 1000;
	do {
		i2c_read(PCT13XX_REG_BIST_FIN, &val, 1);
		if (val != 0)
			break;
		usleep_range(10000, 10000);
	} while (--retry > 0);
	if (!retry) {
		TOUCH_E_LOG("wirte flash from sram fail due to flash busy");
		TOUCH_N_LOG("end %d", EBUSY);
		return -EBUSY;
	}

	if(i2c_read(PCT13XX_REG_BIST_PASS, &val, 1) < 0){
		TOUCH_E_LOG("write flash from sram fail due to read 0x53");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	if(IS_BIT_CLEAR(val, 1)){
		TOUCH_E_LOG("wirte flash from sram fail due to pass_signal");

		if(i2c_write(PCT13XX_REG_BIST_CONTROL, PCT13XX_BIST_TWO) < 0){
			TOUCH_E_LOG("write flash from sram fail due to write 0x50(2)");
			TOUCH_N_LOG("end %d", EIO);
			return -EIO;
		}

		TOUCH_N_LOG("end %d", EBUSY);
		return -EBUSY;
	}

	if(i2c_write(PCT13XX_REG_BIST_CONTROL, PCT13XX_BIST_TWO) < 0){
		TOUCH_E_LOG("write flash from sram fail due to write 0x50");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}


	TOUCH_N_LOG("end");
	return 0;
}

static int pct1332qn_write_flash_page(int pageIndex, uint8_t *pageData, int pageLen)
{
	int sramDataOffset = (pageIndex & 3) << 10;

	TOUCH_N_LOG("start");

	if(pct1332qn_write_sram(pageData, sramDataOffset, pageLen) < 0){
		TOUCH_E_LOG("write flash page fail due to write sram");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	if(pct1332qn_wirte_flash_from_sram(pageIndex) < 0){
		TOUCH_E_LOG("write flash page fail due to write flash from sram");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	TOUCH_N_LOG("end");
	return 0;
}

static int pct1332qn_write_flash(struct pct13xx_data *pd, int address, uint8_t *data, uint8_t force_download)
{
	int len = sizeof(fw_pct1332qn);
	int pageSize = 1024, remainedSize;
	uint8_t pageData[pageSize];

	TOUCH_N_LOG("start");

	if(!force_download)
	{
		if( pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
	}
	else
	{
		if( pct13xx_switch_bank(6) < 0 ){
			TOUCH_E_LOG("fail due to swtich bank");
			return -EIO;
		}
	}
	if(i2c_write(PCT13XX_REG_WD, PCT13XX_WD_DISABLE) < 0){
		TOUCH_E_LOG("write flash fail due to write 0x7D");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}
	if(!force_download)
	{
		if( pct13xx_disable_handshaking_with_check(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
	}

	if(pct13xx_switch_bank(1) < 0){
		TOUCH_E_LOG("write flash fail due to switch bank 1");
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_PD, PCT13XX_PD) < 0){
		TOUCH_E_LOG("write flash fail due to write 0x7D");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	while(1)
	{
		remainedSize = len - (address << 10);
		if(pageSize < remainedSize){

			memcpy(pageData, data + (address << 10), 1024);
			if(pct1332qn_write_flash_page(address, pageData, 1024) < 0){
				TOUCH_E_LOG("write flash fail due to write flash page(1)");
				TOUCH_N_LOG("end %d", EIO);
				return -EIO;
			}
			TOUCH_N_LOG("updating: %d/%d", len, len - remainedSize);
			address++;
		}
		else{
			uint8_t last_pageData[remainedSize];
			memcpy(last_pageData, data + (address << 10), sizeof(last_pageData));
			if(pct1332qn_write_flash_page(address, last_pageData, remainedSize) < 0){
				TOUCH_E_LOG("write flash fail due to write flash page(2)");
				TOUCH_N_LOG("end %d", EIO);
				return -EIO;
			}
			TOUCH_N_LOG("updating: %d/%d", len, len);
			break;
		}
	}

	if(pct13xx_reset(pd, POWER_HARD_RESET, 1) < 0){
		pr_err("%s: write flash fail due to reset\n", __func__);
		return -EIO;
	}

	TOUCH_N_LOG("end");
	return 0;

}
static int pct1332qn_update_firmware(struct pct13xx_data *pd, uint8_t *flashcode, uint8_t force_download)
{

	TOUCH_N_LOG("start");

	if(pct1332qn_write_flash(pd, 0, flashcode, force_download) < 0 ){
		TOUCH_E_LOG("update firmware fail due to write flash");
		TOUCH_N_LOG("end %d", EIO);
		return -EIO;
	}

	TOUCH_N_LOG("end");
	return 0;
}
static int pct1332qn_calc_crc(struct pct13xx_data *pd, uint8_t *crc_result, uint8_t crc_type)
{
	int retry;
	uint8_t val = PCT13XX_CRC_BUSY;

	if(pct13xx_enable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("fail due to enable handshaking with check");
		return -EIO;
	}
	pct13xx_reg_user_write(pd, PCT13XX_REG_CRC_CALC, crc_type);

	retry = 100;
	do {
		pct13xx_reg_user_read(pd, PCT13XX_REG_CRC_CALC, &val);
		if (IS_BIT_CLEAR(val, PCT13XX_CRC_BUSY))
			break;
		msleep(50);
	} while (--retry > 0);
	if (!retry){
		pr_err(MODULE_NAME "%s: wait crc calculation too long\n", __func__);
		return -EIO;
	}
	pct13xx_reg_user_read(pd, PCT13XX_REG_DEBUG_LO, crc_result);
	pct13xx_reg_user_read(pd, PCT13XX_REG_DEBUG_HI, crc_result + 1);
	if(pct13xx_disable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("fail due to disable handshaking with check");
		return -EIO;
	}

	return 0;
}

static int pct1332qn_load_fw(struct pct13xx_data *pd, uint8_t force_download)
{
	uint8_t fw_id_hi = 0, fw_id_lo = 0, crc_result[2];
	uint8_t *pfw = fw_pct1332qn;
	uint16_t desired_fw_id = PCT1332QN_FW_VERSION;
	uint16_t fw_id;

	TOUCH_N_LOG("start");

#if FORCE_FW_DOWNLOAD
	force_download = 1;
#endif

	if (!force_download) {
		pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MJREV_ID, &fw_id_hi);
		pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MNREV_ID, &fw_id_lo);
		fw_id = (fw_id_hi << 8) | fw_id_lo;
		TOUCH_N_LOG("fw_ver of current/target is : %d/%d ", fw_id, desired_fw_id);
		if (fw_id == 0) {
			pct13xx_reset(pd, POWER_HARD_RESET, 1);
			pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MJREV_ID, &fw_id_hi);
			pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MNREV_ID, &fw_id_lo);
			fw_id = (fw_id_hi << 8) | fw_id_lo;
		}
		/* FW download when 1. force download enabled. 2. fw_id great than
		 * desired id.  */
		if (fw_id == 0xffff) {
			pr_err("%s: IC FWver -> %d\n", __func__, fw_id);
		} else if (fw_id > desired_fw_id) {
			pr_info("%s: Acceptable FW version already present "
					"(rev %d). No reflash needed.\n", __func__, fw_id);
			TOUCH_N_LOG("end");
			return 0;
		} else if (fw_id == desired_fw_id) {
			if(pct1332qn_calc_crc(pd, crc_result, PCT13XX_CRC_FW) < 0 ){
				pr_err(MODULE_NAME "%s: check fw integrity fail due to calc crc\n", __func__);
				return -EIO;
			}
			if(crc_result[1] == PCT1332QN_CR_CRC_HI && crc_result[0] == PCT1332QN_CR_CRC_LO){
				pr_info("%s: FW %d -> CRC match. No reflash needed.\n", __func__, fw_id);
				TOUCH_N_LOG("end");
				return 0;
			}
			pr_err("%s: CRC mismatch. target:0x%02x%02x , IC:0x%02x%02x\n", __func__,
				PCT1332QN_CR_CRC_HI, PCT1332QN_CR_CRC_LO, crc_result[1], crc_result[0]);
		}
		pr_err("%s: IC FWver->%d.  FWDL run->%d.\n", __func__, fw_id, desired_fw_id);
	}

	pct1332qn_update_firmware(pd, pfw, force_download);
	/*check crc*/
	if(pct1332qn_calc_crc(pd, crc_result, PCT13XX_CRC_FW) < 0 ){
		pr_err(MODULE_NAME "%s: update fw fail due to calc crc\n", __func__);
		return -EIO;
	}
	if(crc_result[1] != PCT1332QN_CR_CRC_HI || crc_result[0] != PCT1332QN_CR_CRC_LO){
		pr_err(MODULE_NAME "%s: calc crc fail due to target crc 0x%02x%02x != calc crc 0x%02x%02x\n",
				__func__, PCT1332QN_CR_CRC_HI, PCT1332QN_CR_CRC_LO, crc_result[1], crc_result[0]);
		return -EIO;
	}
	pr_info("%s: FW update -> current CRC = 0x%02x%02x\n",
				__func__, crc_result[1], crc_result[0]);

	TOUCH_N_LOG("end");
	return 0;
}

static void pct1332qn_calc_param_crc(uint8_t *crc_result)
{
	uint32_t crc = 0, i, len = sizeof(param_pct1332qn);

	for(i = 0 ; i < len ; i++){
		crc += param_pct1332qn[i];
		crc ^= 0x000000d8u;
		crc &= 0x0000ffffu;
		crc <<= 1;
		if (crc >= 0x00010000u)
			crc += 229u;
		crc &= 0x0000ffffu;
	}

	*(crc_result) = (uint8_t)(crc >> 8);
	*(crc_result + 1) = (uint8_t)crc;

}

static int pct1332qn_flash_controller_read(uint8_t pageIndex, uint8_t address, uint8_t len, uint8_t *buffer)
{
	int i;

	if(pct13xx_switch_bank(1) < 0){
		pr_err(MODULE_NAME "%s: flash controller read fail due to switch bank 1\n", __func__);
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_USER_MODE, PCT13XX_USER_CMD) < 0){
		pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x5E(1)\n", __func__);
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_IFREN, PCT13XX_IB_ENABLE) < 0){
		pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x6E(1)\n", __func__);
		return -EIO;
	}

	for(i = 0; i < len; i++){
		if(i2c_write(PCT13XX_REG_FLASH_CMD, PCT13XX_FLASH_STANDBY) < 0){
			pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x62(1)\n", __func__);
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_FLASH_CMD, PCT13XX_FLASH_READY) < 0){
			pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x62(2)\n", __func__);
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_FLASH_XADR, (pageIndex << 3) | (((address + i)>> 7) & 0x07)) < 0){
			pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x64\n", __func__);
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_FLASH_YADR, ((address + i) & 0x7f)) < 0){
			pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x65\n", __func__);
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_START, PCT13XX_START) < 0){
			pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x62(1)\n", __func__);
			return -EIO;
		}
		if(i2c_read(PCT13XX_REG_FLASH_DATA, (buffer + i), 1) < 0){
			pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x62(1)\n", __func__);
			return -EIO;
		}
	}

	if(i2c_write(PCT13XX_REG_IFREN, PCT13xx_IB_DISABLE) < 0){
		pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x6E(2)\n", __func__);
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_USER_MODE, PCT13XX_CPU_READ) < 0){
		pr_err(MODULE_NAME "%s: flash controller read fail due to write 0x5E(2)\n", __func__);
		return -EIO;
	}

	return 0;
}

static int pct1332_read_flash_IB(struct pct13xx_data *pd, uint8_t addr, uint8_t len, uint8_t *buffer)
{
	if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_WD, PCT13XX_WD_DISABLE) < 0){
		pr_err(MODULE_NAME "%s: read flash IB fail due to write 0x7D\n", __func__);
		return -EIO;
	}
	if(pct13xx_disable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}

	if(pct13xx_switch_bank(1) < 0){
		pr_err(MODULE_NAME "%s: read flash IB fail due to switch bank 1\n", __func__);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_PD, PCT13XX_PD) < 0){
		pr_err(MODULE_NAME "%s: read flash IB fail due to write 0x7D\n", __func__);
		return -EIO;
	}

	if(pct1332qn_flash_controller_read(0, addr, len, buffer) < 0){
		pr_err(MODULE_NAME "%s: read flash IB fail due to flash controller read\n", __func__);
		return -EIO;
	}

	if(pct13xx_reset(pd, POWER_HARD_RESET, 1) < 0){
		pr_err("%s: write flash fail due to reset\n", __func__);
		return -EIO;
	}

	return 0;
}

static int pct1332qn_read_user_param(struct pct13xx_data *pd, uint8_t *user_config_page)
{
	uint8_t CPAndFTParam[16], user_param_crc[2], version = 0x01;
	int len = sizeof(param_pct1332qn);
	if(pct1332_read_flash_IB(pd, PCT13XX_IB_PARAM_OFFSET, sizeof(CPAndFTParam), CPAndFTParam) < 0){
		pr_err(MODULE_NAME "%s: read user param fail due to read flash IB\n", __func__);
		return -EIO;
	}

	pct1332qn_calc_param_crc(user_param_crc);

    memcpy(user_config_page, CPAndFTParam, sizeof(CPAndFTParam));
	*(user_config_page + PCT13XX_PARAM_VER_OFFSET) = version;
	*(user_config_page + PCT13XX_PARAM_LEN_OFFSET) = len >> 1;
	*(user_config_page + PCT13XX_PARAM_CRC_OFFSET) = user_param_crc[0];
	*(user_config_page + PCT13XX_PARAM_CRC_OFFSET + 1) = user_param_crc[1];
	memcpy(user_config_page + PCT13XX_PARAM_OFFSET, param_pct1332qn, len);

	return 0;
}

static int pct1332qn_wirte_flash_config(struct pct13xx_data *pd, uint8_t *data, int address)
{
	if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_WD, PCT13XX_WD_DISABLE) < 0){
		pr_err(MODULE_NAME "%s: write flash config fail due to write 0x7D\n", __func__);
		return -EIO;
	}
	if(pct13xx_disable_handshaking_with_check(pd) < 0)
	{
		TOUCH_E_LOG("end");
		return -EIO;
	}

	if(pct13xx_switch_bank(1) < 0){
		pr_err(MODULE_NAME "%s: write flash config fail due to switch bank 1\n", __func__);
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_PD, PCT13XX_PD) < 0){
		pr_err(MODULE_NAME "%s: write flash config fail due to write 0x7D\n", __func__);
		return -EIO;
	}

	if(pct1332qn_write_flash_page(address, data , 1024) < 0){
		pr_err(MODULE_NAME "%s: write flash config fail due to write flash page\n", __func__);
		return -EIO;
	}

	if(pct13xx_reset(pd, POWER_HARD_RESET, 1) < 0){
		pr_err("%s: write flash fail due to reset\n", __func__);
		return -EIO;
	}
	return 0;
}

static int pct1332qn_write_user_param(struct pct13xx_data *pd)
{
	int pageSize = 1024;
	uint8_t user_param_flashpage[pageSize];
	memset(user_param_flashpage, 0x00, sizeof(user_param_flashpage));

	if(pct1332qn_read_user_param(pd, user_param_flashpage) < 0){
		pr_err(MODULE_NAME "%s: write user param fail due to read param\n", __func__);
		return -EIO;
	}

	if(pct1332qn_wirte_flash_config(pd, user_param_flashpage, 31) < 0){
		pr_err(MODULE_NAME "%s: write user param fail due to read param\n", __func__);
		return -EIO;
	}
	return 0;
}

static int pct1332qn_update_user_param(struct pct13xx_data *pd)
{
	if(pct1332qn_write_user_param(pd) < 0 ){
		pr_err(MODULE_NAME "%s: update user param fail due to write user param\n", __func__);
		return -EIO;
	}
	return 0;
}

static int pct1332qn_flash_config(struct pct13xx_data *pd)
{
	uint8_t user_param_target_crc[2], user_param_current_crc[2];

	pct1332qn_calc_param_crc(user_param_target_crc);

	if(pct1332qn_calc_crc(pd, user_param_current_crc, PCT13XX_CRC_REG) < 0 ){
		pr_err(MODULE_NAME "%s: flash config fail due to calc crc\n", __func__);
		return -EIO;
	}
/*whther update*/
	if(FORCE_CONFIG_DOWNLOAD || user_param_target_crc[1] != user_param_current_crc[1] || user_param_target_crc[0] != user_param_current_crc[0]){
		pr_info(MODULE_NAME "target config crc != current crc (%02x%02x != %02x%02x) or force download. \n",
					 user_param_target_crc[1], user_param_target_crc[0], user_param_current_crc[1], user_param_current_crc[0]);
		pr_info(MODULE_NAME "begin to update user parameters...\n");
		if(pct1332qn_update_user_param(pd) < 0 ){
			pr_err(MODULE_NAME "%s: flash config fail due to update user param\n", __func__);
			return -EIO;
		}
/*check crc after updating user parameters*/
		msleep(200);
		if(pct1332qn_calc_crc(pd, user_param_current_crc, PCT13XX_CRC_REG) < 0 ){
			pr_err(MODULE_NAME "%s: flash config fail due to calc crc\n", __func__);
			return -EIO;
		}

		if(user_param_target_crc[1] != user_param_current_crc[1] || user_param_target_crc[0] != user_param_current_crc[0]){
			pr_err(MODULE_NAME "flash config fail due to target crc:%02x%02x != current crc:%02x%02x\n",
					     user_param_target_crc[1], user_param_target_crc[0], user_param_current_crc[1], user_param_current_crc[0]);
			return -EIO;
		}

		pr_info(MODULE_NAME "end of updating\n");

		pr_info(MODULE_NAME "after update, target config crc equals current crc (%02x%02x = %02x%02x).\n",
					 user_param_target_crc[1], user_param_target_crc[0], user_param_current_crc[1], user_param_current_crc[0]);
	}
	else{
		pr_info(MODULE_NAME "target config crc equals current crc (%02x%02x = %02x%02x). No reflash needed.\n",
					 user_param_target_crc[1], user_param_target_crc[0], user_param_current_crc[1], user_param_current_crc[0]);
	}

	return 0;
}
#endif
/*end of FW download*/

static int pct13xx_set_extended(struct pct13xx_data *pd)
{
	struct ts_config_nv *config_nv;
	int err = 0, i;

	TOUCH_N_LOG("start");
	if(pd->app_power == TOUCH_POWERON){
		TOUCH_N_LOG("app_power == ON");
		config_nv = &pd->config_nv[TS_EXTENDED];
		if(!config_nv->data){
			pr_err("%s: No nv Extended data.\n",__func__);
			return 0;
		}
		for(i = 0; i < config_nv->size; ){
			err |= pct13xx_reg_user_write(pd, config_nv->data[i],   config_nv->data[i+2]);
			if(err){
				pr_err("%s: %d: I2C Access Error. Addr 0x%02x\n", __func__, __LINE__, config_nv->data[i+2]);
				TOUCH_N_LOG("end");
				return err;
			}
			i += 3;
		}
	} else
		TOUCH_E_LOG("Touch Power is OFF");
	TOUCH_N_LOG("end");

	return 0;
}

static int pct13xx_init_panel(struct pct13xx_data *pd)
{
	uint8_t ret = 0;
	uint8_t val_hi = 0, val_lo = 0;

	TOUCH_N_LOG("start");

#if ENABLE_FW_DOWNLOAD
	ret = pct1332qn_load_fw(pd, 0);
	if (ret) {
		 TOUCH_E_LOG("pct1332qn_load_fw failed. %d", ret);
		 return ret;
	}
	ret = pct1332qn_flash_config(pd);
	if (ret) {
		 TOUCH_E_LOG("pct1332qn_flash_config failed. %d", ret);
		 return ret;
	}
#endif

	pct13xx_reg_user_read(pd, PCT13XX_REG_REPORT_NUM, &max_finger);
	if (max_finger > PCT13XX_MAX_REPORT_NUM) {
		TOUCH_E_LOG("Incorrect report point %d", max_finger);
		ret = -EINVAL;
	}
	if(pct13xx_switch_bank(0) < 0){
		pr_err(MODULE_NAME "%s: init panel fail due to switch bank 0\n", __func__);
		return -EIO;
	}
	if(i2c_read(PCT13XX_REG_DRIVE_NUM, &drive_num, 1) < 0){
		pr_err(MODULE_NAME "%s: init panel fail due to read 0x04\n", __func__);
		return -EIO;
	}
	drive_num++;
	if(i2c_read(PCT13XX_REG_SENSE_NUM, &sense_num, 1) < 0){
		pr_err(MODULE_NAME "%s: init panel fail due to read 0x05\n", __func__);
		return -EIO;
	}
	sense_num++;

	pct13xx_reg_user_read(pd, PCT13XX_REG_HEIGHT_HI, &val_hi);
	pct13xx_reg_user_read(pd, PCT13XX_REG_HEIGHT_LO, &val_lo);
	max_x = (val_hi << 8) | val_lo;

	pct13xx_reg_user_read(pd, PCT13XX_REG_WIDTH_HI, &val_hi);
	pct13xx_reg_user_read(pd, PCT13XX_REG_WIDTH_LO, &val_lo);
	max_y = (val_hi << 8) | val_lo;

	TOUCH_N_LOG("resolution is (%d/%d) ", max_x, max_y);

	TOUCH_N_LOG("end %d", ret);
	return ret;
}

#ifdef GESTURE_SAMPLE
static void pct13xx_gesture_sample(void)
{
	uint8_t gesture = 0, rel_x[2] = {0}, rel_y[2] = {0};

	if(pct13xx_switch_bank(5) < 0){
		TOUCH_E_LOG("fail due to switch bank 5");
		return;
	}
	i2c_read(PCT13XX_REG_GEST_TYPE, &gesture, 1);

	if(gesture == PCT13XX_ZOOM){
		i2c_read(PCT13XX_SEQ_RW + PCT13XX_REG_REL_X , rel_x, 2);
		TOUCH_D_LOG("zoom triggered mx = %d", (short)((rel_x[1] << 8) | rel_x[0]));

	}
	else if(gesture == PCT13XX_MOVE_CURSOR){
		i2c_read(PCT13XX_SEQ_RW + PCT13XX_REG_REL_X, rel_x, 2);
		i2c_read(PCT13XX_SEQ_RW + PCT13XX_REG_REL_Y, rel_y, 2);
		TOUCH_D_LOG("move cursor triggered (mx, my)=(%d, %d)", (short)((rel_x[1] << 8) | rel_x[0]), (short)((rel_y[1] << 8) | rel_y[0]));
	}

}

#endif
static int pct13xx_get_report(struct pct13xx_data *pd, struct touch_report *report)
{
	int i;
	uint8_t data[16] = {0};

	TOUCH_N_LOG("start");

	if(pct13xx_switch_bank(5) < 0){
		TOUCH_E_LOG("fail due to switch bank 5");
		goto err_get_report;
	}

	i2c_read(PCT13XX_REG_OBJ_NUM, data, 1);

	report->total_touch = data[0];
	TOUCH_D_LOG("total_touch:%d", data[0]);
	if(report->total_touch > 0) {
		for (i = 0; i < report->total_touch; i++) {

			if(i2c_read(PCT13XX_SEQ_RW + PCT13XX_REG_ID_BASE + (i * 0x10), data, 10) < 0){
				TOUCH_E_LOG("i2c_read : get report error");
				goto err_get_report;

			}
			report->point_data[i].id = data[0] & 0x07;
			report->point_data[i].x = (data[2] << 8) | data[1];
			report->point_data[i].y = (data[4] << 8) | data[3];
			report->point_data[i].area = ((data[6] << 8)& 0x03) | data[5];
			report->point_data[i].force = (data[8] << 8) | data[7];
		}
#ifdef GESTURE_SAMPLE
		pct13xx_gesture_sample();
#endif
	}

	TOUCH_N_LOG("end");
	return 0;

err_get_report:
	TOUCH_N_LOG("end %d", EINVAL);
	return -EIO;
}

static void pct13xx_work_func(void)
{
	struct pct13xx_data *pd = i2c_get_clientdata(drv_client);
	int i, id;
	struct touch_report touch_report;
	DECLARE_BITMAP(used, PCT13XX_ID_MAX);
	struct input_dev *input_dev = pd->input;

	TOUCH_N_LOG("start");

	if(unlikely(pct13xx_get_report(pd, &touch_report) < 0)){
		TOUCH_E_LOG("get report error");
		pd->err_irq_cnt++;
		goto end_of_work_func;
	}

	bitmap_zero(used, PCT13XX_ID_MAX);
	if(touch_report.total_touch > 0){
		for (i = 0; i < touch_report.total_touch; i++)
		{
			id = touch_report.point_data[i].id;
			__set_bit(id, used);
			if (likely(ts_event_report)) {
				input_mt_slot(input_dev, id);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(pd->input, ABS_MT_POSITION_X,
					touch_report.point_data[i].x);
				input_report_abs(pd->input, ABS_MT_POSITION_Y,
					touch_report.point_data[i].y);
				input_report_abs(pd->input, ABS_MT_PRESSURE,
					touch_report.point_data[i].force);
				input_report_abs(pd->input, ABS_MT_TOUCH_MAJOR,
					touch_report.point_data[i].area);
			}
			mutex_lock(&pd->mt_lock);
			pd->touch_info[id].x = touch_report.point_data[i].x;
			pd->touch_info[id].y = touch_report.point_data[i].y;
			pd->touch_info[id].force = touch_report.point_data[i].force;
			pd->touch_info[id].area = touch_report.point_data[i].area;
			mutex_unlock(&pd->mt_lock);
			TOUCH_D_LOG("[id:%d] [x:%04d] [y:%04d] [press:%04d] [major:%d]",
					id,
					touch_report.point_data[i].x,
					touch_report.point_data[i].y,
					touch_report.point_data[i].force,
					touch_report.point_data[i].area);
		}
	}
	pd->touch_cnt = touch_report.total_touch;
	for (i = 0; i < PCT13XX_ID_MAX; i++) {
		if (test_bit(i, used))
			continue;
		if (likely(ts_event_report)) {
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
		pd->touch_info[i].x = 0;
		pd->touch_info[i].y = 0;
		pd->touch_info[i].force = 0;
		pd->touch_info[i].area = 0;
		TOUCH_D_LOG("[slot:%d:0]", i);
	}

	if (likely(ts_event_report)) {
		if (touch_report.total_touch) {
			TOUCH_D_LOG("input_report_key: 1");
			input_report_key(input_dev, BTN_TOUCH, 1);
		}
		else {
			TOUCH_D_LOG("input_report_key: 0");
			input_report_key(input_dev, BTN_TOUCH, 0);
		}

		input_sync(input_dev);
	}

end_of_work_func:
	if(unlikely(pct13xx_switch_bank(6) < 0)){
		TOUCH_E_LOG("fail due to switch bank 6");
		pd->err_irq_cnt++;
		return;
	}
	i2c_write(PCT13XX_REG_STATUS,0);

	TOUCH_N_LOG("end");
}

static void pct13xx_watchdog_handler(struct pct13xx_data *pd)
{
	struct input_dev *input_dev = pd->input;
	int i;

	pr_info("IC error: %s is called.\n",__func__);

	for (i = 0; i < pd->touch_cnt; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);
	pd->touch_cnt = 0;

	if(pct13xx_reset(pd, POWER_HARD_RESET, 1) < 0){
		pr_err(MODULE_NAME "%s: watchdog reset fail\n", __func__);
	}
}

static irqreturn_t pct13xx_irq_handler(int irq, void *dev_id)
{
	struct pct13xx_data *pd = (struct pct13xx_data*)dev_id;
	uint8_t status = 0;

	TOUCH_N_LOG("start");

	mutex_lock(&pd->system_lock);
	if(unlikely((pd->power == TOUCH_POWEROFF)
			 || (pd->app_power == TOUCH_POWEROFF))){
		TOUCH_E_LOG("end : Touch Power OFF");
		mutex_unlock(&pd->system_lock);
		return IRQ_HANDLED;
	}
	if(unlikely(pct13xx_switch_bank(6) < 0)){
		TOUCH_E_LOG("fail due to switch bank 6");
		mutex_unlock(&pd->system_lock);
		pd->err_irq_cnt++;
		return IRQ_HANDLED;
	}
	i2c_read(PCT13XX_REG_STATUS, &status, 1);

	if(unlikely(IS_BIT_SET(status, WATCHDOG_RESET_STATUS))){
		TOUCH_E_LOG("WATCHDOG RESET: status = %02x. ",status);
		pct13xx_watchdog_handler(pd);
	}
	else if (likely(IS_BIT_SET(status, MOTION_REPORT_STATUS))){
		pct13xx_work_func();
	} else {
		TOUCH_E_LOG("MOTION REPORT STATUS check fail status == %02x. ",status);
		i2c_write(PCT13XX_REG_STATUS, 0);
	}
	mutex_unlock(&pd->system_lock);

	TOUCH_N_LOG("end");
	return IRQ_HANDLED;
}

static int pct13xx_free_fingers(struct kc_ts_data *kd)
{

	unsigned char i;
	struct input_dev *input_dev;
	struct pct13xx_data *pd;

	TOUCH_N_LOG("start");

	if (!kd) {
		TOUCH_E_LOG("kc_ts_data is null");
		return -1;
	}

	pd = (struct pct13xx_data *)kd->vdata;
	input_dev = pd->input;

	mutex_lock(&pd->mt_lock);

	for (i = 0; i < pd->touch_cnt; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		pd->touch_info[i].x = 0;
		pd->touch_info[i].y = 0;
		pd->touch_info[i].force = 0;
		pd->touch_info[i].area = 0;
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	pd->touch_cnt = 0;

	mutex_unlock(&pd->mt_lock);

	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_esd_proc(struct pct13xx_data *pd)
{
	int err = 0;

	TOUCH_N_LOG("start");

	err = pct13xx_check_bank_identifier();
	if( err ){
		TOUCH_E_LOG("IC Error, Recovery Start.");
		pd->err_esd_cnt++;
		err = __pct13xx_device_reset(pd);
		if(err){
			TOUCH_E_LOG("Failed to restart");
		}
	}

	TOUCH_N_LOG("end");

	return err;
}

static void pct13xx_esd_work(struct work_struct *work)
{
	struct pct13xx_data *pd = container_of(work, struct pct13xx_data, esdwork.work);
	int err = 0;

	if ((ts_esd_recovery) && mutex_trylock(&pd->system_lock)) {
		if(pd->app_power == TOUCH_POWERON)
			err = pct13xx_esd_proc(pd);
		mutex_unlock(&pd->system_lock);
	}
	if (pd->esd_wq && !err) {
		if(pd->app_power == TOUCH_POWERON)
			queue_delayed_work(pd->esd_wq, &pd->esdwork,
				msecs_to_jiffies(ESD_POLLING_TIME));
	} else {
		pr_err("%s: Failed to ESD work!\n", __func__);
	}
}

static int pct13xx_confirm_device(struct pct13xx_data *pd, uint8_t type)
{
	int err = 0;
	uint8_t val = 0;
	uint16_t fw_id;
	uint8_t fw_id_mj = 0, fw_id_mn = 0;

	TOUCH_N_LOG("start");

	err = pct13xx_enter_control(pd, type);
	if (err < 0){
		TOUCH_E_LOG("Boot state get fail.");
		goto err_confirm_device;
	}
	err = pct13xx_reg_user_read(pd, PCT13XX_REG_PID, &pd->pid);
	if (err < 0){
		TOUCH_E_LOG("PID get fail.");
		goto err_confirm_device;
	}
	if (!IS_BIT_SET(pd->pid, PCT1332QN_PID)){
		err = -ENODEV;
		TOUCH_E_LOG("PID check fail.");
		goto err_confirm_device;
	}
	pct13xx_reg_user_read(pd, PCT13XX_REG_HW_REV_ID, &val);
	if (!IS_BIT_SET(val, PCT1332QN_HW_REV_ID)){
		err = -ENODEV;
		TOUCH_E_LOG("HW REV ID check fail.");
		goto err_confirm_device;
	}
	pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MJREV_ID, &fw_id_mj);
	TOUCH_D_LOG("PCT13XX_REG_FW_MJREV_ID[0x%02x]", fw_id_mj);
	pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MNREV_ID, &fw_id_mn);
	TOUCH_D_LOG("PCT13XX_REG_FW_MNREV_ID[0x%02x]", fw_id_mn);
	fw_id = (fw_id_mj << 8) | fw_id_mn;
	if (fw_id < PCT1332QN_FW_VERSION) {
		err = -ENODEV;
		TOUCH_E_LOG("FW ID check fail.");
		goto err_confirm_device;
	}

err_confirm_device:
	TOUCH_N_LOG("end");
	return err;
}

static void pct13xx_get_touch_info(struct kc_ts_data *kd)
{
	int i;
	struct pct13xx_data *pd;

	TOUCH_N_LOG("start");
	if (!kd) {
		TOUCH_E_LOG("kd is null");
		return;
	}

	pd = (struct pct13xx_data *)kd->vdata;

	mutex_lock(&pd->mt_lock);

	if (!kd->diag_data) {
		TOUCH_E_LOG("diag_data has no data.");
	}

	for (i = 0; i < PCT13XX_ID_MAX; i++) {
		kd->diag_data->ts[i].x = pd->touch_info[i].x;
		kd->diag_data->ts[i].y = pd->touch_info[i].y;
		kd->diag_data->ts[i].width = pd->touch_info[i].area;
	}
	kd->diag_data->diag_count = pd->touch_cnt;

	mutex_unlock(&pd->mt_lock);

	TOUCH_N_LOG("end");
	return;
}

static s16 pct13xx_calc_cell(u16 idx, char *arry)
{
	s16 ret;
	ret = ((arry[idx + 1] << 8) | arry[idx]);

	return ret;
}


static int pct13xx_short_test(struct kc_ts_data *kd)
{
	TOUCH_N_LOG("start");
	TOUCH_N_LOG("end");
	return 0;
}
static int pct13xx_open_test(struct kc_ts_data *kd)
{
	TOUCH_N_LOG("start");
	TOUCH_N_LOG("end");
	return 0;
}
static int pct13xx_open_short_test(struct kc_ts_data *kd)
{
	char max_arry[DIAG_TEST_RESULT_SIZE] = {0};
	char min_arry[DIAG_TEST_RESULT_SIZE] = {0};
	u8 diff_arry[DIAG_TEST_RESULT_SIZE] = {0};
	u8 loop_cnt_1, loop_max;
	u16 loop_cnt_2;
	s16 max, min;
	u16 diff;
	int ret = 0;
	s16 raw_data,raw_data_min,raw_data_max;
	u16 raw_data_diff;
	struct pct13xx_data *pd;

	TOUCH_N_LOG("start");
	pd = (struct pct13xx_data *)kd->vdata;
	if(pd->power == TOUCH_POWEROFF){
		pr_err("%s: Error: Touch Power is OFF\n",__func__);
		return -1;
	}

	ret = pct13xx_enter_control(pd, BOOT_NAV_READY);
	if (ret < 0){
		TOUCH_E_LOG("Boot state get fail.");
		return -ENODEV;
	}

	loop_max = kd->open_short_result->req_param.req_param[6];
	if (loop_max == 0 || loop_max > 5)
		loop_max = 5;

	for (loop_cnt_1 = 0; loop_cnt_1 < loop_max ; loop_cnt_1++) {
		ret = pct13xx_get_rawdata_new(kd);
		if (ret) {
			TOUCH_E_LOG("fail get Raw Data");
			return  -ENODEV;
		}
		for (loop_cnt_2 = 0; loop_cnt_2 < DIAG_TEST_RESULT_SIZE ; loop_cnt_2 += 2) {
			raw_data = pct13xx_calc_cell(loop_cnt_2 ,kd->frame_data->buffer);
			if ((loop_cnt_1 == 0) ||
				(pct13xx_calc_cell(loop_cnt_2,max_arry) < raw_data)) {
				max_arry[loop_cnt_2] = kd->frame_data->buffer[loop_cnt_2];
				max_arry[loop_cnt_2 + 1] = kd->frame_data->buffer[loop_cnt_2 + 1];
			}

			if ((loop_cnt_1 == 0) ||
				(pct13xx_calc_cell(loop_cnt_2 , min_arry) > raw_data) ||
				(pct13xx_calc_cell(loop_cnt_2 , min_arry) == 0)) {
				if(raw_data >= 10000){
					min_arry[loop_cnt_2] = kd->frame_data->buffer[loop_cnt_2];
					min_arry[loop_cnt_2 + 1] = kd->frame_data->buffer[loop_cnt_2 + 1];
				} else {
					TOUCH_E_LOG("Error!! Mode switch is not complete. min = %d", raw_data);
				}
			}
		}
	}

	for (loop_cnt_2 = 0; loop_cnt_2 < DIAG_TEST_RESULT_SIZE ; loop_cnt_2 += 2) {
		raw_data_max = pct13xx_calc_cell(loop_cnt_2 ,max_arry);
		raw_data_min = pct13xx_calc_cell(loop_cnt_2 ,min_arry);
		diff_arry[loop_cnt_2] = (0x00FF & (raw_data_max - raw_data_min));
		diff_arry[loop_cnt_2 + 1] = (0xFF00 & (raw_data_max - raw_data_min)) >> 8;
	}
	max = (kd->open_short_result->req_param.req_param[0] << 8)
			| kd->open_short_result->req_param.req_param[1];
	min = (kd->open_short_result->req_param.req_param[2] << 8)
			| kd->open_short_result->req_param.req_param[3];
	diff = (kd->open_short_result->req_param.req_param[4] << 8)
			| kd->open_short_result->req_param.req_param[5];
	if (max) {
		for (loop_cnt_2 = 0; loop_cnt_2 < DIAG_TEST_RESULT_SIZE ; loop_cnt_2 += 2) {
			raw_data_max = pct13xx_calc_cell(loop_cnt_2 ,max_arry);
			if (max < raw_data_max) {
				ret |= 0x1A;
				break;
			}
		}
	}
	if (min) {
		for (loop_cnt_2 = 0; loop_cnt_2 < DIAG_TEST_RESULT_SIZE ; loop_cnt_2 += 2) {
			raw_data_min = pct13xx_calc_cell(loop_cnt_2 ,min_arry);
			if (min > raw_data_min) {
				ret |= 0x2A;
				break;
			}
		}
	}
	if (diff) {
		for (loop_cnt_2 = 0; loop_cnt_2 < DIAG_TEST_RESULT_SIZE ; loop_cnt_2 += 2) {
			raw_data_diff = ((diff_arry[loop_cnt_2 + 1] << 8) | diff_arry[loop_cnt_2]);
			if (diff < raw_data_diff) {
				ret |= 0x4A;
				break;
			}
		}
	}

	memcpy(kd->open_short_result->res_result.result_max, max_arry, DIAG_TEST_RESULT_SIZE);
	memcpy(kd->open_short_result->res_result.result_min, min_arry, DIAG_TEST_RESULT_SIZE);
	TOUCH_N_LOG("end");
	return ret;
}
static int pct13xx_mode_check_ioctl(struct kc_ts_data *kd)
{
	TOUCH_N_LOG("start");
	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_get_system_info_ioctl(struct kc_ts_data *kd, u8 *dp)
{
	TOUCH_N_LOG("start");
	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_RawDataRead(uint8_t* buff, uint8_t idx, uint16_t length)
{
	TOUCH_N_LOG("start");

	if (!buff) {
		TOUCH_E_LOG("buff is null");
		return -1;
	}

	if(pct13xx_switch_bank(2) < 0){
		TOUCH_E_LOG("write sram fail due to switch bank2");
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_SRAM_SEL, idx) < 0){
		TOUCH_E_LOG("write sram fail due to write b2:0x09");
		return -EIO;
	}
	if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_ONE)){
		TOUCH_E_LOG("raw data read fail due to write b2:0x0c(1)");
		return -EIO;
	}
	if(i2c_read(PCT13XX_REG_SRAM_DATA, buff, length) < 0){
		TOUCH_E_LOG("read frame buffer fail!!");
	}
	if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_TWO)){
		TOUCH_E_LOG("raw data read fail due to write b2:0x0c(2)");
		return -EIO;
	}
	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_read_SRAM(struct pct13xx_data *pd, uint8_t* buff, uint16_t* length, int type)
{
	uint8_t frm_idx, wk_idx, wk_drive_num, wk_sense_num;
	int err = 0;
	int retry;
	uint8_t val;

	TOUCH_N_LOG("start");

	if (!buff) {
		TOUCH_E_LOG("buff is null");
		return -1;
	}
	if (!length) {
		TOUCH_E_LOG("length is null");
		return -1;
	}
	if(pd->app_power == TOUCH_POWEROFF){
		pr_err("%s: Error: Touch App Power is OFF\n",__func__);
		return -1;
	}

	if(type == GET_DATA_TYPE_RAW_DATA){
		if(pct13xx_switch_bank(2) < 0){
			TOUCH_E_LOG("write sram fail due to switch bank2");
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_BYPASS, 0x02) < 0){
			TOUCH_E_LOG("write sram fail due to write b2:0x04");
			return -EIO;
		}
	}

	if(pct13xx_switch_bank(0) < 0){
		TOUCH_E_LOG("write sram fail due to switch bank0");
		return -EIO;
	}
	if(i2c_read(0x04, &wk_drive_num, 1) < 0){
		TOUCH_E_LOG("fail due to read b0:0x04");
		return -EIO;
	}
	wk_drive_num++;
	if(i2c_read(0x05, &wk_sense_num, 1) < 0){
		TOUCH_E_LOG("fail due to read b0:0x05");
		return -EIO;
	}
	wk_sense_num++;

	if(pct13xx_switch_bank(2) < 0){
		TOUCH_E_LOG("write sram fail due to switch bank2");
		return -EIO;
	}
	if(i2c_read(PCT13XX_REG_FRM_STATUS, &frm_idx, 1) < 0){
		TOUCH_E_LOG("fail due to read b2:0x18");
		return -EIO;
	}

	if(pct1332qn_raw_data_wnd(0, wk_drive_num - 1, 0, wk_sense_num -1) < 0){
		TOUCH_E_LOG("fail write_RawDataWND");
		return -EIO;
	}
	if(type == GET_DATA_TYPE_RAW_DATA){
		wk_idx = 0;
		if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_ALC, PCT13XX_ALC_OFF) < 0){
			TOUCH_E_LOG("write sram fail due to write 0x55");
			return -EIO;
		}
		if(pct13xx_disable_handshaking_with_check(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
	} else {
		wk_idx = (frm_idx == 1 ? PCT13XX_RAW_DATA_FRM_0 : PCT13XX_RAW_DATA_FRM_1);
	}
	if(type == GET_DATA_TYPE_RAW_DATA_NEW){
		if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_ALC, PCT13XX_ALC_OFF) < 0){
			TOUCH_E_LOG("write sram fail due to write 0x55");
			return -EIO;
		}
		retry = 17;
		do {
			if(i2c_read(PCT13XX_REG_ALC, &val, 1) < 0){
				TOUCH_E_LOG("failed to read 0x55.");
				return -EIO;
			}
			if (val == 0x23)
				break;
			msleep(30);
		} while (--retry > 0);
		if(pct13xx_disable_handshaking_with_check(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
		if (!retry){
			TOUCH_E_LOG("wait ALC off too long");
			return -EIO;
		}
		TOUCH_N_LOG("ALC off wait time was %d ms", (100 - retry) * 30);
	}
	*length = wk_drive_num * wk_sense_num * 2;

	err = pct13xx_RawDataRead(buff, wk_idx, *length );

	if(type == GET_DATA_TYPE_RAW_DATA){
		if(pct13xx_enable_handshaking_and_switch_bank6(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
		if(i2c_write(PCT13XX_REG_ALC, PCT13XX_ALC_ON) < 0){
			TOUCH_E_LOG("write sram fail due to write 0x55");
			return -EIO;
		}
		if(pct13xx_disable_handshaking_with_check(pd) < 0)
		{
			TOUCH_E_LOG("end");
			return -EIO;
		}
	}

	if( err < 0){
		TOUCH_E_LOG("fail pct13xx_RawDataRead");
		return -EIO;
	}

	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_get_RawData(struct kc_ts_data *kd)
{
	int err = 0;
	struct pct13xx_data *pd;

	TOUCH_N_LOG("start");
	pd = (struct pct13xx_data *)kd->vdata;
	if(pd->power == TOUCH_POWEROFF){
		pr_err("%s: Error: Touch Power is OFF\n",__func__);
		return -1;
	}

	mutex_lock(&pd->system_lock);
	if( pct13xx_read_SRAM(pd, kd->frame_data->buffer, &kd->frame_data->length, GET_DATA_TYPE_RAW_DATA) < 0){
		TOUCH_E_LOG("fail get Raw Data");
		err = -ENODEV;
	}
	mutex_unlock(&pd->system_lock);
	TOUCH_N_LOG("end");
	return err;
}

static int pct13xx_get_rawdata_new(struct kc_ts_data *kd)
{
	int err = 0;
	int err_rst = 0;
	struct pct13xx_data *pd;

	TOUCH_N_LOG("start");
	pd = (struct pct13xx_data *)kd->vdata;
	if(pd->power == TOUCH_POWEROFF){
		pr_err("%s: Error: Touch Power is OFF\n",__func__);
		return -1;
	}

	mutex_lock(&pd->system_lock);
	if( pct13xx_read_SRAM(pd, kd->frame_data->buffer, &kd->frame_data->length, GET_DATA_TYPE_RAW_DATA_NEW) < 0){
		TOUCH_E_LOG("fail get Raw Data");
		err = -ENODEV;
	}

	err_rst = __pct13xx_device_reset(pd);
	if(err_rst){
		TOUCH_E_LOG("Failed to reset");
	}
	err |=err_rst;

	mutex_unlock(&pd->system_lock);
	TOUCH_N_LOG("end");
	return err;
}

static int pct13xx_get_TouchDeltaData(struct kc_ts_data *kd)
{
	int err = 0;
	struct pct13xx_data *pd;

	TOUCH_N_LOG("start");
	pd = (struct pct13xx_data *)kd->vdata;
	if(pd->power == TOUCH_POWEROFF){
		pr_err("%s: Error: Touch Power is OFF\n",__func__);
		return -1;
	}

	mutex_lock(&pd->system_lock);
	if( pct13xx_read_SRAM(pd, kd->frame_data->buffer, &kd->frame_data->length, GET_DATA_TYPE_TOUCH_DELTA_DATA) < 0){
		TOUCH_E_LOG("fail get Touch Delta Data");
		err = -ENODEV;
	}
	mutex_unlock(&pd->system_lock);
	TOUCH_N_LOG("end");
	return err;
}
static s16 pct13xx_get_max_value(char *max, char *arry, u16 cnt)
{
	s16 max_val;

	TOUCH_N_LOG("start");
	max_val = pct13xx_calc_cell(cnt, max);
	if (max_val < pct13xx_calc_cell(cnt, arry)) {
		max[cnt] = arry[cnt];
		max[cnt + 1] = arry[cnt + 1];
		max_val = pct13xx_calc_cell(cnt, max);
	}
	TOUCH_N_LOG("end");
	return max_val;
}
static s16 pct13xx_get_min_value(char *min, char *arry, u16 cnt)
{
	s16 min_val;

	TOUCH_N_LOG("start");
	min_val = pct13xx_calc_cell(cnt, min);
	if (min_val > pct13xx_calc_cell(cnt, arry)) {
		min[cnt] = arry[cnt];
		min[cnt + 1] = arry[cnt + 1];
		min_val = pct13xx_calc_cell(cnt, min);
	}
	TOUCH_N_LOG("end");
	return min_val;
}
static u16 pct13xx_get_diff_value(u8 *diff, s16 max, s16 min, u16 cnt)
{
	TOUCH_N_LOG("start");
	diff[cnt] = (0x00FF & (max - min));
	diff[cnt + 1] = (0xFF00 & (max - min)) >> 8;
	TOUCH_N_LOG("end");
	return (u16)pct13xx_calc_cell(cnt, diff);
}
static int pct13xx_press_diff_test(struct kc_ts_data *kd)
{
	int err = 0;

	u8 param = kd->press_diff_test_data->param;
	u16 diff = kd->press_diff_test_data->judgment_value;
	static struct press_diff_data *press_diff_data = NULL;
	u16 loop_cnt;
	char max_arry[DIAG_TEST_RESULT_SIZE] = {0};
	char min_arry[DIAG_TEST_RESULT_SIZE] = {0};
	u8 diff_arry[DIAG_TEST_RESULT_SIZE] = {0};
	s16 delta_data_min,delta_data_max;
	u16 delta_data_diff;
	struct pct13xx_data *pd;

	pr_info("%s:start param:[%02x]\n", __func__, param);
	if ((press_diff_data == NULL) && (param != 1)) {
		TOUCH_E_LOG("%s:press_diff_data is NULL ",__func__);
		return -1;
	}
	pd = (struct pct13xx_data *)kd->vdata;
	if(pd->power == TOUCH_POWEROFF){
		pr_err("%s: Error: Touch Power is OFF\n",__func__);
		return -1;
	}

	switch (param) {
	case 1:
		if (press_diff_data == NULL)
			press_diff_data = kzalloc(sizeof (struct press_diff_data), GFP_KERNEL);
		else
			return -1;
		if (pct13xx_get_TouchDeltaData(kd) < 0) {
			TOUCH_E_LOG("fail get Touch Delta Data");
			return -1;
		}
		memcpy(press_diff_data->delta_data1, kd->frame_data->buffer,
				sizeof(kd->frame_data->buffer));
		break;
	case 2:
		if (pct13xx_get_TouchDeltaData(kd) < 0) {
			TOUCH_E_LOG("fail get Touch Delta Data");
			return -1;
		}
		memcpy(press_diff_data->delta_data2, kd->frame_data->buffer,
				sizeof(kd->frame_data->buffer));
		break;
	case 3:
		if (pct13xx_get_TouchDeltaData(kd) < 0) {
			TOUCH_E_LOG("fail get Touch Delta Data");
			return -1;
		}
		memcpy(press_diff_data->delta_data3, kd->frame_data->buffer,
				sizeof(kd->frame_data->buffer));
		break;
	case 4:
		if (pct13xx_get_TouchDeltaData(kd) < 0) {
			TOUCH_E_LOG("fail get Touch Delta Data");
			return -1;
		}
		memcpy(press_diff_data->delta_data4, kd->frame_data->buffer,
				sizeof(kd->frame_data->buffer));
		break;
	case 5:
		if (pct13xx_get_TouchDeltaData(kd) < 0) {
			TOUCH_E_LOG("fail get Touch Delta Data");
			return -1;
		}
		memcpy(press_diff_data->delta_data5, kd->frame_data->buffer,
				sizeof(kd->frame_data->buffer));
		break;
	case 6:
		memcpy(max_arry, press_diff_data->delta_data1,
				sizeof(press_diff_data->delta_data1));
		memcpy(min_arry, press_diff_data->delta_data1,
				sizeof(press_diff_data->delta_data1));
		for (loop_cnt = 0; loop_cnt < DIAG_TEST_RESULT_SIZE; loop_cnt += 2) {
			delta_data_max = pct13xx_get_max_value(max_arry,
					press_diff_data->delta_data3, loop_cnt);
			delta_data_min = pct13xx_get_min_value(min_arry,
					press_diff_data->delta_data3, loop_cnt);
			delta_data_max = pct13xx_get_max_value(max_arry,
					press_diff_data->delta_data5, loop_cnt);
			delta_data_min = pct13xx_get_min_value(min_arry,
					press_diff_data->delta_data5, loop_cnt);
			delta_data_diff = pct13xx_get_diff_value(diff_arry,
					delta_data_max,delta_data_min, loop_cnt);
			if (delta_data_diff > diff)
				kd->press_diff_test_data->press_diff_test_result.cmd_result = -1;
		}
		memcpy(kd->press_diff_test_data->press_diff_test_result.delta_data_diff,
				diff_arry, sizeof(diff_arry));
		break;
	case 7:
		memcpy(max_arry, press_diff_data->delta_data2,
				sizeof(press_diff_data->delta_data2));
		memcpy(min_arry, press_diff_data->delta_data2,
				sizeof(press_diff_data->delta_data2));
		for (loop_cnt = 0; loop_cnt < DIAG_TEST_RESULT_SIZE; loop_cnt += 2) {
			delta_data_max = pct13xx_get_max_value(max_arry,
					press_diff_data->delta_data4, loop_cnt);
			delta_data_min = pct13xx_get_min_value(min_arry,
					press_diff_data->delta_data4, loop_cnt);
			delta_data_diff = pct13xx_get_diff_value(diff_arry,
					delta_data_max, delta_data_min, loop_cnt);
			if (delta_data_diff > diff)
				kd->press_diff_test_data->press_diff_test_result.cmd_result = -1;
		}
		memcpy(kd->press_diff_test_data->press_diff_test_result.delta_data_diff,
				diff_arry, sizeof(diff_arry));
		break;
	case 8:
		if (press_diff_data != NULL) {
			kfree(press_diff_data);
			press_diff_data = NULL;
		}
		break;
	default:
		TOUCH_E_LOG("%s: unsupported param", __func__);
		break;
	}

	pr_info("%s:end err:[%02x]\n", __func__, err);
	return err;
}

static int pct13xx_set_user_data(struct kc_ts_data *kd, unsigned char color)
{
	int err = 0;
	struct pct13xx_data *pd;
	TOUCH_N_LOG("start");
	pr_info("%s:set color is %d \n", __func__, color);
	pd = (struct pct13xx_data *)kd->vdata;

	switch(color){
	case 0:
		pd->config_nv[TS_EXTENDED].data = pct13xx_extended_data0;
		pd->config_nv[TS_EXTENDED].size = ARRAY_SIZE(pct13xx_extended_data0);
		break;
	case 1:
		pd->config_nv[TS_EXTENDED].data = pct13xx_extended_data1;
		pd->config_nv[TS_EXTENDED].size = ARRAY_SIZE(pct13xx_extended_data1);
		break;
	case 2:
		pd->config_nv[TS_EXTENDED].data = pct13xx_extended_data2;
		pd->config_nv[TS_EXTENDED].size = ARRAY_SIZE(pct13xx_extended_data2);
		break;
	case 3:
		pd->config_nv[TS_EXTENDED].data = pct13xx_extended_data3;
		pd->config_nv[TS_EXTENDED].size = ARRAY_SIZE(pct13xx_extended_data3);
		break;
	case 4:
		pd->config_nv[TS_EXTENDED].data = pct13xx_extended_data4;
		pd->config_nv[TS_EXTENDED].size = ARRAY_SIZE(pct13xx_extended_data4);
		break;
	default:
		TOUCH_E_LOG("%s: unsupported color", __func__);
		break;
	}

	TOUCH_N_LOG("end");
	return err;
}

static const struct kc_ts_operations pct13xx_dm_ops = {
	.get_touch_info		= pct13xx_get_touch_info,
	.short_test			= pct13xx_short_test,
	.open_test			= pct13xx_open_test,
	.mode_check			= pct13xx_mode_check_ioctl,
	.get_system_info	= pct13xx_get_system_info_ioctl,
	.free_fingers		= pct13xx_free_fingers,
	.get_raw_data		= pct13xx_get_RawData,
	.get_touch_delta_data		= pct13xx_get_TouchDeltaData,
	.get_raw_data_new		= pct13xx_get_rawdata_new,
	.set_user_data		= pct13xx_set_user_data,
	.open_short_test		= pct13xx_open_short_test,
	.press_diff_test			= pct13xx_press_diff_test,
};


static int pct1332qn_raw_data_wnd(uint8_t drive_start, uint8_t drive_stop, uint8_t sense_start, uint8_t sense_stop)
{
	TOUCH_N_LOG("start");

	if(pct13xx_switch_bank(2) < 0){
		TOUCH_E_LOG("fail due to switch bank 2");
		return -EIO;
	}

	if(i2c_write(PCT13XX_REG_DRIVE_START, drive_start) < 0
			|| i2c_write(PCT13XX_REG_DRIVE_STOP, drive_stop) < 0
			|| i2c_write(PCT13XX_REG_SENSE_START, sense_start) < 0
			|| i2c_write(PCT13XX_REG_SENSE_STOP, sense_stop) < 0){
		pr_err(MODULE_NAME "%s: raw data wnd fail due to i2c write\n", __func__);
		return -EIO;
	}
	TOUCH_N_LOG("end");
	return 0;
}

/*
 * Debugging options via sysfs
 */
static ssize_t pct13xx_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct13xx_data *pd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;

	TOUCH_N_LOG("start");
	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		goto pct13xx_drv_debug_store_exit;
	}

	switch (value) {
	case PT_DBG_SUSPEND:
		dev_info(dev, "%s: SUSPEND (pd=%p)\n", __func__, pd);
		mutex_lock(&pd->system_lock);
		if(pd->power == TOUCH_POWERON)
			rc = pct13xx_cmd_suspend(pd, 1);
		pd->app_power = TOUCH_POWEROFF;
		mutex_unlock(&pd->system_lock);
		if (rc)
			dev_err(dev, "%s: Suspend failed rc=%d\n",
				__func__, rc);
		else
			dev_info(dev, "%s: Suspend succeeded\n", __func__);
		break;
	case PT_DBG_RESUME:
		dev_info(dev, "%s: RESUME (pd=%p)\n", __func__, pd);
		mutex_lock(&pd->system_lock);
		pd->app_power = TOUCH_POWERON;
		if(pd->power == TOUCH_POWERON)
			rc = pct13xx_cmd_resume(pd, 1);
		mutex_unlock(&pd->system_lock);
		if (rc)
			dev_err(dev, "%s: Resume failed rc=%d\n",
				__func__, rc);
		else
			dev_info(dev, "%s: Resume succeeded\n", __func__);
		break;
	case PT_HANDSHAKE_ENABLE:
			dev_info(dev, "%s: PT_HANDSHAKE_ENABLE\n", __func__);
			mutex_lock(&pd->system_lock);
			if(pct13xx_enable_handshaking() < 0)
				pr_err(MODULE_NAME "%s: Fail: enable handshaking\n", __func__);
			mutex_unlock(&pd->system_lock);
			break;
	case PT_HANDSHAKE_DISABLE:
			dev_info(dev, "%s: PT_HANDSHAKE_DISABLE\n", __func__);
			mutex_lock(&pd->system_lock);
			if(pct13xx_disable_handshaking() < 0)
				pr_err(MODULE_NAME "%s: Fail: disable handshaking\n", __func__);
			mutex_unlock(&pd->system_lock);
			break;
	default:
		dev_err(dev, "%s: Invalid value\n", __func__);
	}

pct13xx_drv_debug_store_exit:
	TOUCH_N_LOG("end");
	return size;
}

static ssize_t pct13xx_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct13xx_data *pd = dev_get_drvdata(dev);
	int rc = 0;

	TOUCH_N_LOG("start");
	mutex_lock(&pd->system_lock);
	if(pd->power == TOUCH_POWERON){
		rc = __pct13xx_device_reset(pd);
		if (rc < 0)
			dev_err(dev, "%s: HW reset error\n", __func__);
	} else {
		pr_err("%s: Error: Touch Power is OFF\n",__func__);
	}
	mutex_unlock(&pd->system_lock);

	TOUCH_N_LOG("end");
	return size;
}

static ssize_t pct13xx_man_cal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct13xx_data *pd = dev_get_drvdata(dev);

	TOUCH_D_LOG("start");
	mutex_lock(&pd->system_lock);
	if ((pd->app_power == TOUCH_POWERON) &&
		(pd->power == TOUCH_POWERON)){
		TOUCH_D_LOG("ALC reset.");
		pct13xx_reg_user_write(pd, 0x74, 0x01);
	}
	mutex_unlock(&pd->system_lock);

	TOUCH_D_LOG("end");
	return size;
}
static ssize_t pct13xx_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct13xx_data *pd = dev_get_drvdata(dev);
	ssize_t ret;

	TOUCH_N_LOG("start");
	mutex_lock(&pd->system_lock);
	if (pd->irq_enabled)
		ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE,
			"Driver interrupt is ENABLED\n");
	else
		ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE,
			"Driver interrupt is DISABLED\n");
	mutex_unlock(&pd->system_lock);

	TOUCH_N_LOG("end");
	return ret;
}

static ssize_t pct13xx_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pct13xx_data *pd = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	TOUCH_N_LOG("start");
	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		goto pct13xx_drv_irq_store_error_exit;
	}

	mutex_lock(&pd->system_lock);
	switch (value) {
	case 0:
		if (pd->irq_enabled) {
			pd->irq_enabled = false;
			disable_irq_nosync(pd->client->irq);
			dev_info(dev, "%s: Driver IRQ now disabled\n",
				__func__);
		} else
			dev_info(dev, "%s: Driver IRQ already disabled\n",
				__func__);
		break;

	case 1:
		if (pd->irq_enabled == false) {
			pd->irq_enabled = true;
			enable_irq(pd->client->irq);
			dev_info(dev, "%s: Driver IRQ now enabled\n",
				__func__);
		} else
			dev_info(dev, "%s: Driver IRQ already enabled\n",
				__func__);
		break;

	default:
		dev_err(dev, "%s: Invalid value\n", __func__);
		break;
	}
	mutex_unlock(&pd->system_lock);

pct13xx_drv_irq_store_error_exit:
	TOUCH_N_LOG("end");
	return size;
}

static ssize_t pct13xx_debug_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pct13xx_data *pd = dev_get_drvdata(dev);
	ssize_t ret;
	char *str;
	uint8_t fw_id_mj, fw_id_mn;

	TOUCH_N_LOG("start");
	mutex_lock(&pd->system_lock);
	if(pd->power == TOUCH_POWERON){
		if(pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MJREV_ID, &fw_id_mj) < 0) {
			ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE, "I2C ERROR\n");
			goto exit_pct13xx_debug_info_show;
		}
		if(pct13xx_reg_user_read(pd, PCT13XX_REG_FW_MNREV_ID, &fw_id_mn) < 0) {
			ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE, "I2C ERROR\n");
			goto exit_pct13xx_debug_info_show;
		}

		if (pd->power)
			str = "Sleep ON";
		else
			str = "Sleep OFF";

		ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE,
			"%s: 0x%02X\n"
			"%s: 0x%02X\n"
			"%s \n"
			"%s: %u\n"
			"%s: %u\n",
			"Firmware Major Version", fw_id_mj,
			"Firmware Minor Version", fw_id_mn,
			str,
			"ESD Error Count", pd->err_esd_cnt,
			"IRQ Error Count", pd->err_irq_cnt
		);
	} else
		ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE, "Touch Power is OFF\n");

exit_pct13xx_debug_info_show:
	mutex_unlock(&pd->system_lock);
	TOUCH_N_LOG("end");
	return ret;
}
static struct device_attribute attributes[] = {
	__ATTR(drv_debug, S_IWUSR | S_IWGRP, NULL, pct13xx_drv_debug_store),
	__ATTR(hw_reset, S_IWUSR, NULL, pct13xx_hw_reset_store),
	__ATTR(drv_irq, S_IRUSR | S_IWUSR, pct13xx_drv_irq_show, pct13xx_drv_irq_store),
	__ATTR(debug_info, S_IRUSR, pct13xx_debug_info_show, NULL),
	__ATTR(man_cal, S_IWUSR, NULL, pct13xx_man_cal_store),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

/**
 *  sysfs allocates a buffer of size (PAGE_SIZE) and passes it to the
 *  method. Sysfs will call the method exactly once for each read or
 *  write. This forces the following behavior on the method
 */
static uint8_t frame_buffer[PCT13XX_MAX_DRIVE_NUM * PCT13XX_MAX_SENSE_NUM * 2];
static ssize_t pct13xx_user_result(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i = 0;
	uint8_t val = 0;
	uint16_t len = (uint16_t) user_cmd.number;
	int ret = 0;
	uint16_t length = drive_num * sense_num;
	struct pct13xx_data *pd = i2c_get_clientdata(to_i2c_client(dev));

	TOUCH_N_LOG("start");
	switch (user_cmd.command) {
	case SYSFS_READ:
		mutex_lock(&pd->system_lock);
		if(pd->power == TOUCH_POWERON){
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "    ");
			for (i = 0; i < 16; i++)
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%2x ", i);

			i = user_cmd.reg[0] & 0xf0;
			for (; i < ((user_cmd.reg[0] + len + 15) & 0xf0); i++) {
				/* new line: show reg index */
				if (!(i & 0xf))
					ret += scnprintf(buf + ret, PAGE_SIZE - ret,
						"\n%02x: ", i);

				if (i >= user_cmd.reg[0] && i < user_cmd.reg[0] + len) {
					i2c_read(i, &val, 1);
					ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%02x ", val);
				}
				else
					ret += scnprintf(buf + ret, PAGE_SIZE - ret, "   ");
			}
		} else
			ret = snprintf(buf, PCT13XX_MAX_PRBUF_SIZE, "Touch Power is OFF\n");
		mutex_unlock(&pd->system_lock);
		break;
	case SYSFS_VERSION:
		ret = scnprintf(buf, PAGE_SIZE, "%s\n", DRIVER_VERSION);
		break;
	case SYSFS_SWITCH_BANK:
		ret = scnprintf(buf, PAGE_SIZE, "switch to %02x\n", user_cmd.data);
		break;
	case SYSFS_PREPARE_RAW_DATA:

		for(i = 0; i < length*2; i=i+2){
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%02x%02x\n",
			frame_buffer[i], frame_buffer[i+1]);
		}
		break;

	default:
		break;
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	TOUCH_N_LOG("end");
	return (ssize_t)ret;
}

/**
 *  Chip control via user commands.
 */
static ssize_t pct13xx_user_cmd(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int offset;
	const char *p;
	uint32_t param[6];
	uint8_t frm_idx;
	uint16_t length = drive_num * sense_num << 1;
	struct pct13xx_data *pd = i2c_get_clientdata(to_i2c_client(dev));

	TOUCH_N_LOG("start");
	switch (buf[0]) {
	case 'r':
		user_cmd.command = SYSFS_READ;
		p = buf + 1;
		sscanf(p, "%x %x", param, param + 1);
		user_cmd.reg[0] = (uint8_t) param[0];
		user_cmd.number = (uint16_t) param[1];
		pr_info("mode:read, reg:%02x, number:%02x\n", user_cmd.reg[0], user_cmd.number);
		break;

	case 'w':
		mutex_lock(&pd->system_lock);
		user_cmd.command = SYSFS_WRITE;
		if(pd->power == TOUCH_POWERON){
			p = buf + 1;
			while (sscanf(p, "%x %x%n", param, param + 1, &offset) == 2) {
				user_cmd.reg[0] = (uint8_t) param[0];
				user_cmd.data   = (uint8_t) param[1];
				p += offset;
				i2c_write(user_cmd.reg[0], user_cmd.data);
				pr_info("mode:write, reg:%02x, data:%02x\n", user_cmd.reg[0], user_cmd.data);
			}
		} else
			pr_err("%s: Error: Touch Power is OFF\n",__func__);
		mutex_unlock(&pd->system_lock);
		break;
	case 'v':
		user_cmd.command = SYSFS_VERSION;
		pr_info(MODULE_NAME "Ver: %s\n", DRIVER_VERSION);
		break;
	case 's':
		mutex_lock(&pd->system_lock);
		user_cmd.command = SYSFS_SWITCH_BANK;
		if(pd->power == TOUCH_POWERON){
			p = buf + 1;
			sscanf(p, "%x", param);
			user_cmd.data = param[0];
			pr_info(MODULE_NAME "Switch to Bank %02x\n", user_cmd.data);
			if(pct13xx_switch_bank(user_cmd.data) < 0){
				pr_err(MODULE_NAME "user cmd fail due to switch Bank %02x\n", user_cmd.data);
			}
		} else
			pr_err("%s: Error: Touch Power is OFF\n",__func__);
		mutex_unlock(&pd->system_lock);
		break;
	case 'd':
		mutex_lock(&pd->system_lock);
		user_cmd.command = SYSFS_MED3_DATA;
		if(pd->power == TOUCH_POWERON){
			if(pct13xx_switch_bank(2) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to switch bank 2\n", __func__);
				goto err_lock_release;
			}
			if(i2c_read(PCT13XX_REG_FRM_STATUS, &frm_idx, 1) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to read 0x18\n", __func__);
				goto err_lock_release;
			}
			if(pct1332qn_raw_data_wnd(0, drive_num - 1, 0, sense_num -1) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to wnd\n", __func__);
				goto err_lock_release;
			}
			disable_irq(pd->client->irq);
			if(pct13xx_switch_bank(2) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to switch bank 2\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_write(PCT13XX_REG_SRAM_SEL, frm_idx == 1 ? PCT13XX_RAW_DATA_FRM_0 : PCT13XX_RAW_DATA_FRM_1)){
				pr_err(MODULE_NAME "%s: raw data read fail due to write 0x09\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_ONE)){
				pr_err(MODULE_NAME "%s: raw data read fail due to write 0x0c(1)\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_read(PCT13XX_REG_SRAM_DATA, frame_buffer, length) < 0){
				pr_err(MODULE_NAME "%s: read frame buffer fail!!\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_TWO)){
				pr_err(MODULE_NAME "%s: raw data read fail due to write 0x0c(2)\n", __func__);
				goto err_enable_irq;
			}
			enable_irq(pd->client->irq);
		} else
			pr_err("%s: Error: Touch Power is OFF\n",__func__);
		mutex_unlock(&pd->system_lock);
		break;
	case 'p':
		user_cmd.command = SYSFS_PREPARE_RAW_DATA;
		break;
	case 'm':
		mutex_lock(&pd->system_lock);
		user_cmd.command = SYSFS_RAW_DATA;
		if(pd->power == TOUCH_POWERON){
			if(pct13xx_switch_bank(2) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to switch bank 2\n", __func__);
				goto err_lock_release;
			}
			if(i2c_read(PCT13XX_REG_FRM_STATUS, &frm_idx, 1) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to read 0x18\n", __func__);
				goto err_lock_release;
			}
			if(pct1332qn_raw_data_wnd(0, drive_num - 1, 0, sense_num -1) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to wnd\n", __func__);
				goto err_lock_release;
			}
			disable_irq(pd->client->irq);
			if(pct13xx_switch_bank(2) < 0){
				pr_err(MODULE_NAME "%s: user reslut fail due to switch bank 2\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_write(PCT13XX_REG_SRAM_SEL, PCT13XX_MED3_DATA)){
				pr_err(MODULE_NAME "%s: raw data read fail due to write 0x09\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_ONE)){
				pr_err(MODULE_NAME "%s: raw data read fail due to write 0x0c(1)\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_read(PCT13XX_REG_SRAM_DATA, frame_buffer, length) < 0){
				pr_err(MODULE_NAME "%s: read frame buffer fail!!\n", __func__);
				goto err_enable_irq;
			}
			if(i2c_write(PCT13XX_REG_SRAM_NCS, PCT13XX_SRAM_NCS_TWO)){
				pr_err(MODULE_NAME "%s: raw data read fail due to write 0x0c(2)\n", __func__);
				goto err_enable_irq;
			}
			enable_irq(pd->client->irq);
		} else
			pr_err("%s: Error: Touch Power is OFF\n",__func__);
		mutex_unlock(&pd->system_lock);
		break;
	default:
		break;
	}

	TOUCH_N_LOG("end");
	return count;

err_enable_irq:
	enable_irq(pd->client->irq);
err_lock_release:
	mutex_unlock(&pd->system_lock);
	TOUCH_E_LOG("end");
	return count;
}

static DEVICE_ATTR(m, S_IWUSR | S_IRUSR,
		pct13xx_user_result, pct13xx_user_cmd);

static struct attribute *tp_attributes[] = {
	&dev_attr_m.attr,
	NULL
};

static const struct attribute_group tp_attr_group = {
	.attrs = tp_attributes,
};

/**
 *  Sysfs init.
 */
static int pct13xx_sysfs_create(struct pct13xx_data *pd)
{

	TOUCH_N_LOG("start");
	/* Register sysfs hooks */
	if (sysfs_create_group(&drv_client->dev.kobj, &tp_attr_group)) {
		pr_err(MODULE_NAME "%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_sysfs_remove(struct pct13xx_data *pd)
{

	TOUCH_N_LOG("start");
	/* Free sysfs hooks */
	sysfs_remove_group(&drv_client->dev.kobj, &tp_attr_group);
	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_ts_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pct13xx_data *pd;
	struct input_dev *input_dev;
	int err = 0, ret;

	TOUCH_N_LOG("start");
	TOUCH_N_LOG("init i2c. ver is: %s", DRIVER_VERSION);

	drv_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOUCH_E_LOG("need I2C_FUNC_I2C");
		err = -ENODEV;
	goto err_check_func;
	}
	pd = kzalloc(sizeof (struct pct13xx_data), GFP_KERNEL);
	if (!pd) {
		err = -ENOMEM;
		TOUCH_E_LOG("out of mem");
		goto err_alocate_fail;
	}
	pct13xx_display_touch(pd);
	pd->handshake_stat = 0;
	mutex_init(&pd->handshake_lock);

	pd->client = client;
	i2c_set_clientdata(client, pd);

	pd->dev = &client->dev;

	prob_err_state = 0;
	ret = pct13xx_gpio_regulator_init(pd, pd->dev);
	if (ret) {
		prob_err_state = 1;
		err = -EIO;
		goto err_req_gpio;
	}

	gpio_set_value(pd->vdd_gpio, 1);
	usleep_range(5000,5000);
	// L15 ON (Processed in aboot)
	usleep_range(1000,1000);
	i2c_pinctrl_set_active(client->adapter);
	usleep_range(9000,9000);

	ret = __pct13xx_device_power_on(pd);
	if (ret) {
		prob_err_state = 2;
		err = -ENODEV;
		goto err_device_power_on;
	}

	ret = pct13xx_confirm_device(pd, BOOT_COMPLETE);
	if (ret < 0) {
#if ENABLE_FW_DOWNLOAD
		TOUCH_E_LOG("confirm_device fail.");
		ret = pct1332qn_load_fw(pd, 1);
		if (ret != 0) {
			prob_err_state = 3;
			err = -EIO;
			TOUCH_E_LOG("load fw failed.");
			goto err_device_init_fail;
		}
		ret = pct13xx_confirm_device(pd, BOOT_COMPLETE);
		if (ret < 0) {
			prob_err_state = 4;
			err = -ENODEV;
			TOUCH_E_LOG("confirm_device fail.");
			goto err_device_init_fail;
		}
#else
		prob_err_state = 5;
		err = -ENODEV;
		TOUCH_E_LOG("confirm_device fail.");
		goto err_device_init_fail;
#endif
	}

	pd->config_nv[TS_EXTENDED].data = pct13xx_extended_data0;
	pd->config_nv[TS_EXTENDED].size = ARRAY_SIZE(pct13xx_extended_data0);

	ret = pct13xx_init_panel(pd);
	if (ret != 0) {
		prob_err_state = 6;
		err = -EINVAL;
		TOUCH_E_LOG("init panel failed.");
		goto err_device_init_fail;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		prob_err_state = 7;
		err = -ENOMEM;
		TOUCH_E_LOG("failed to allocate input device");
		goto err_input_dev_alloc_fail;
	}

	pd->client = client;
	pd->input = input_dev;
	pd->power = TOUCH_POWERON;
	input_dev->name = PCT13XX_DEV_NAME;
	input_dev->dev.parent = &client->dev;
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);

	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_POINTER, input_dev->propbit);

	input_mt_init_slots(input_dev, PCT13XX_ID_MAX, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			0, PCT13XX_FORCE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, PCT13XX_AREA_MAX, 0, 0);

	TOUCH_D_LOG("client irq %d", client->irq);

	if (request_threaded_irq(client->irq, NULL, pct13xx_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							client->dev.driver->name, pd)){
		prob_err_state = 8;
		err = -EBUSY;
		TOUCH_E_LOG("irq %d busy? (%s)", client->irq,
				client->dev.driver->name);
		goto err_free_input_dev;
	}
	TOUCH_N_LOG("touchscreen, irq %d", client->irq);
	disable_irq(client->irq);

    pd->esd_wq = alloc_workqueue("kc_ts_esd_wq", WQ_MEM_RECLAIM, 1);
	pd->resume_wq = alloc_workqueue("kc_ts_resume_wq", WQ_MEM_RECLAIM, 1);
	if ( (!pd->esd_wq) || (!pd->resume_wq) ){
		prob_err_state = 9;
        pr_err("%s: Fail to allocate workqueue!\n", __func__);
        err = -ENOMEM;
        goto err_free_irq;
    }

	ret = add_sysfs_interfaces(pd->dev);
	if (ret < 0) {
		prob_err_state = 10;
		err = -ENODEV;
		dev_err(pd->dev, "%s: Error, fail sysfs init\n", __func__);
		goto err_free_irq;
	}

	INIT_DELAYED_WORK(&pd->esdwork, pct13xx_esd_work);
	INIT_WORK(&pd->resumework, pct13xx_resume_work);
	queue_delayed_work(pd->esd_wq, &pd->esdwork,
					msecs_to_jiffies(ESD_FIRST_POLLING_TIME));

	ret = input_register_device(input_dev);
	if (ret) {
		prob_err_state = 11;
		err = -ENODEV;
		TOUCH_E_LOG("input_register_device ");
		goto err_free_wq;
	}

	TOUCH_N_LOG("PixArt pct13xx TouchScreen initialized");

	enable_irq(client->irq);
	pd->irq_enabled = 1;
	pct13xx_sysfs_create(pd);

	mutex_init(&pd->system_lock);
	mutex_init(&pd->mt_lock);
	pd->kc_ts.dev = &client->dev;
	pd->kc_ts.tops = &pct13xx_dm_ops;
	pd->kc_ts.vdata = pd;
	ret = kc_ts_probe(&pd->kc_ts);
	if (ret) {
		prob_err_state = 12;
		err = -ENOMEM;
		TOUCH_E_LOG("Failed to create kc cdev/sysfs");
		goto error_startup_btn;
	}

	pd->is_suspended = false;
	ret = pct13xx_cmd_suspend(pd, 1);
	pd->app_power = TOUCH_POWEROFF;

	TOUCH_N_LOG("end");
	return 0;

error_startup_btn:
	mutex_destroy(&pd->system_lock);
	mutex_destroy(&pd->mt_lock);
err_free_wq:
	if (pd->esd_wq) {
		cancel_delayed_work_sync(&pd->esdwork);
		flush_workqueue(pd->esd_wq);
	}
	destroy_workqueue(pd->esd_wq);
	destroy_workqueue(pd->resume_wq);
	remove_sysfs_interfaces(pd->dev);
err_free_irq:
	free_irq(client->irq, pd);

err_free_input_dev:
	input_free_device(input_dev);

err_input_dev_alloc_fail:
err_device_init_fail:
err_device_power_on:
	gpio_free(pd->reset_gpio);
	gpio_free(pd->irq_gpio);
	gpio_free(pd->vdd_gpio);
err_req_gpio:
	mutex_destroy(&pd->handshake_lock);
	kfree(pd);
err_alocate_fail:
err_check_func:
	TOUCH_E_LOG("probe pct13xx TouchScreen failed, %d", err);

	TOUCH_E_LOG("set NULL pointer to Display\n");
	pct13xx_display_touch(NULL);
	TOUCH_N_LOG("end");
	return err;
}


static int pct13xx_ts_suspend(struct device *dev)
{
	TOUCH_N_LOG("start");

	TOUCH_N_LOG("end");
	return 0;
}

static int pct13xx_ts_resume(struct device *dev)
{
	TOUCH_N_LOG("start");

	TOUCH_N_LOG("end");
	return 0;
}
static int pct13xx_ts_remove(struct i2c_client *client)
{
	struct pct13xx_data *pd = i2c_get_clientdata(client);

	TOUCH_N_LOG("start");

	if (pd->esd_wq) {
		cancel_delayed_work_sync(&pd->esdwork);
		flush_workqueue(pd->esd_wq);
	}

	remove_sysfs_interfaces(pd->dev);

	if(client->irq){
		disable_irq_nosync(client->irq);
		free_irq(client->irq, pd);
	}

	i2c_set_clientdata(client,NULL);
	input_unregister_device(pd->input);
	pct13xx_sysfs_remove(pd);
	mutex_destroy(&pd->system_lock);
	mutex_destroy(&pd->mt_lock);
	mutex_destroy(&pd->handshake_lock);
	kfree(pd);

	TOUCH_N_LOG("end");
	return 0;
}

static void pct13xx_ts_shutdown(struct i2c_client *client)
{
	TOUCH_N_LOG("start");
	TOUCH_N_LOG("end");
	return;
}

static const struct dev_pm_ops pct13xx_pm_ops = {

	 .suspend = pct13xx_ts_suspend,
	 .resume  = pct13xx_ts_resume,

};

static const struct i2c_device_id pct13xx_ts_id[]={
	{PCT13XX_I2C_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pct13xx_ts_id);

static struct of_device_id cy8c20237_match_table[] = {
	{ .compatible = PCT13XX_DTS_NAME,},
	{ },
};

static struct i2c_driver pct13xx_ts_driver = {
	.probe = pct13xx_ts_probe,
	.remove = pct13xx_ts_remove,
	.shutdown = pct13xx_ts_shutdown,
	.id_table = pct13xx_ts_id,
	.driver = {
		.name = PCT13XX_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &pct13xx_pm_ops,
		.of_match_table = cy8c20237_match_table,
	},
};

static int __init pct13xx_ts_init(void)
{
	int32_t ret;
	TOUCH_N_LOG("start");

	ret = i2c_add_driver(&pct13xx_ts_driver);
	if (ret) {
		TOUCH_E_LOG("can't add i2c driver: %d", ret);
	}

	TOUCH_N_LOG("end");
	return 0;
}

static void __exit pct13xx_ts_exit(void)
{

	TOUCH_N_LOG("start");

	i2c_del_driver(&pct13xx_ts_driver);

	TOUCH_N_LOG("end");
}

module_init(pct13xx_ts_init);
module_exit(pct13xx_ts_exit);

MODULE_AUTHOR("YaoChung Hsu <yaochung_hsu@pixart.com>");
MODULE_DESCRIPTION("PixArt PCT13XX TouchScreen driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

