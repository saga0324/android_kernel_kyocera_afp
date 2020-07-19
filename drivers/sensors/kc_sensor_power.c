/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/sensor_power.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>

#define DRV_NAME "sensor-power-driver"

#undef SENSOR_LOG_ON

#ifdef SENSOR_LOG_ON
#define SENSOR_LOG_I(arg...) printk(KERN_INFO "[SP]"arg)
#define SENSOR_LOG_D(arg...) printk("[SP]"arg)
#else
#define SENSOR_LOG_I(arg...)
#define SENSOR_LOG_D(arg...)
#endif

#define SENSOR_INIT_VSENSOR			0x00000001
#define SENSOR_INIT_DONE			0x00000002
#define SENSOR_INIT_ACC_ON			0x00000100
#define SENSOR_INIT_MAG_ON			0x00000200
#define SENSOR_INIT_GYRO_ON			0x00000400
#define SENSOR_INIT_ORI_ON			0x00000800
#define SENSOR_INIT_APDS			0x00001000
#define SENSOR_INIT_DEVICE			0x0000FF00
#if 0
#define SENSOR_GPIO_VMICON			(52)
#endif
#ifdef CONFIG_USE_VSENSORVDD
#define SENSOR_GPIO_VSENSOR_VDD		(35)
#define SENSOR_PINCTRL_SUSPEND		0
#define SENSOR_PINCTRL_ACTIVE		1
struct sensor_power_pinctrl {
	struct pinctrl *sensor_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};
static struct sensor_power_pinctrl *sensor_data;
static uint32_t sensor_power_VSENSOR = 0;
#endif
static spinlock_t sensor_power_spin_lock;
static struct mutex sensor_power_mutex_lock;
static uint32_t sensor_init_status=0;
#if 0
static struct regulator *vpro_vreg;
static int sleep_clk_gpio;
#endif

struct sensor_power_callback* sensor_power_cb_tbl[SENSOR_INDEX_DEVICE_MAX];

#define SENSOR_INIT_STATUS_SET(status,val) \
	if(val) {\
		sensor_init_status |= status;\
	} else {\
		sensor_init_status &= ~(status);\
	}

#define SENSOR_INIT_STATUS_GET(status) \
	(sensor_init_status&(status))

static void kc_sensor_power_on_cbfunc(void);
static void kc_sensor_power_off_cbfunc(void);

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */

static int sensor_power_pinctrl_init(struct platform_device *pdev)
{
	int retval = 0;

	sensor_data->sensor_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(sensor_data->sensor_pinctrl)) {
		pr_err("%s: Target does not use pinctrl\n", __func__);
		retval = PTR_ERR(sensor_data->sensor_pinctrl);
		sensor_data->sensor_pinctrl = NULL;
		return retval;
	}

	sensor_data->gpio_state_active
		= pinctrl_lookup_state(sensor_data->sensor_pinctrl, "sensor_power_active");
	if (IS_ERR_OR_NULL(sensor_data->gpio_state_active)) {
		pr_err("%s: Can not get ts active pinstate\n", __func__);
		retval = PTR_ERR(sensor_data->gpio_state_active);
		sensor_data->sensor_pinctrl = NULL;
		return retval;
	}

	sensor_data->gpio_state_suspend
		= pinctrl_lookup_state(sensor_data->sensor_pinctrl, "sensor_power_sleep");
	if (IS_ERR_OR_NULL(sensor_data->gpio_state_suspend)) {
		pr_err("%s: Can not get ts sleep pinstate\n", __func__);
		retval = PTR_ERR(sensor_data->gpio_state_suspend);
		sensor_data->sensor_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int sensor_power_select(int on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pr_info("%s is called. on = %d\n",__func__,on);

	if(sensor_data == NULL){
		pr_err("%s: Error! sensor_data is NULL.\n",__func__);
		return -1;
	}

	if(on == SENSOR_PINCTRL_ACTIVE) {
		pins_state = sensor_data->gpio_state_active;
	} else if(on == SENSOR_PINCTRL_SUSPEND) {
		pins_state = sensor_data->gpio_state_suspend;
	} else {
		pr_err("%s: unsupport arg %d",__func__, on);
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(sensor_data->sensor_pinctrl, pins_state);
		if (ret) {
			pr_err("%s: can not set %s pins\n", __func__,
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		pr_err("%s: not a valid '%s' pinstate\n", __func__,
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	return 0;
}

void kc_sensor_power_off( void )
{
#if 0
	int32_t rc;
#endif
	
	SENSOR_LOG_I("%s(): start\n", __func__);
	kc_sensor_power_off_cbfunc();
#if 0
	rc = regulator_disable(vpro_vreg);
#endif
#ifdef CONFIG_USE_VSENSORVDD
	gpio_set_value(sensor_power_VSENSOR, 0);
	msleep(50);
#endif
#if 0
	SENSOR_LOG_D("%s(): regulator_disable rc=%d \n", __func__, rc);
	if (!rc) {
		SENSOR_LOG_D("%s():i2c reset done\n", __func__);
		SENSOR_INIT_STATUS_SET(SENSOR_INIT_DONE, 0);
	}
#endif
	SENSOR_INIT_STATUS_SET(SENSOR_INIT_VSENSOR, 0);

	SENSOR_LOG_I("%s(): end\n", __func__);
}

static void kc_sensor_power_shutdown(struct platform_device *pdev)
{
#if 0
	int32_t rc;
	struct qpnp_pin_cfg sleep_clk_gpio_param = {
		.mode		= QPNP_PIN_MODE_DIG_IN,
		.output_type	= QPNP_PIN_OUT_BUF_CMOS,
		.invert		= QPNP_PIN_INVERT_DISABLE,
		.pull		= QPNP_PIN_GPIO_PULL_DN,
		.vin_sel	= QPNP_PIN_VIN2,
		.out_strength	= QPNP_PIN_OUT_STRENGTH_LOW,
		.src_sel	= QPNP_PIN_SEL_FUNC_CONSTANT,
		.master_en	= QPNP_PIN_MASTER_ENABLE,
	};

	SENSOR_LOG_I("%s(): start\n", __func__);

	qpnp_pin_config(sleep_clk_gpio, &sleep_clk_gpio_param);
#endif
	kc_sensor_power_off_cbfunc();
#if 0
	rc = regulator_disable(vpro_vreg);
	msleep(10);
	SENSOR_LOG_D("%s(): regulator_disable rc=%d \n", __func__, rc);
	if (!rc) {
		SENSOR_LOG_D("%s():i2c reset done\n", __func__);
		SENSOR_INIT_STATUS_SET(SENSOR_INIT_DONE, 0);
	}
	SENSOR_INIT_STATUS_SET(SENSOR_INIT_VSENSOR, 0);

	gpio_set_value(SENSOR_GPIO_VMICON, 0);

	SENSOR_LOG_I("%s(): end\n", __func__);
#else
    SENSOR_INIT_STATUS_SET(SENSOR_INIT_VSENSOR, 0);
#endif
}

void sensor_power_on(enum sensor_index id)
{
#if 0
	int32_t rc;
#endif
	SENSOR_LOG_I("%s(): start id:%d\n", __func__, (int)id);

	mutex_lock(&sensor_power_mutex_lock);
	SENSOR_INIT_STATUS_SET((1<<id)<<8, 1);
	
	if(!(SENSOR_INIT_STATUS_GET(SENSOR_INIT_VSENSOR))){ 
#ifdef CONFIG_USE_VSENSORVDD
		gpio_set_value(sensor_power_VSENSOR, 1);
		msleep(50);
		//usleep_range(500,600);
#endif
#if 0
		rc = regulator_enable(vpro_vreg);
		SENSOR_LOG_D("%s(): regulator_enable rc=%d\n", __func__, rc);
		msleep(50);
		if (!rc) {
			SENSOR_LOG_D("%s():i2c reset done\n", __func__);
			SENSOR_INIT_STATUS_SET(SENSOR_INIT_DONE, 1);
		}
#endif
		SENSOR_INIT_STATUS_SET(SENSOR_INIT_VSENSOR, 1);
		kc_sensor_power_on_cbfunc();
	}
	mutex_unlock(&sensor_power_mutex_lock);

	SENSOR_LOG_I("%s(): status=0x%x end\n", __func__, sensor_init_status);
}
EXPORT_SYMBOL(sensor_power_on);

void sensor_power_off(enum sensor_index id)
{
	uint32_t devices, init_vsensor;

	SENSOR_LOG_I("%s(): start\n", __func__);

	mutex_lock(&sensor_power_mutex_lock);
	SENSOR_INIT_STATUS_SET((1<<id)<<8, 0);
	devices = SENSOR_INIT_STATUS_GET(SENSOR_INIT_DEVICE);
	init_vsensor = SENSOR_INIT_STATUS_GET(SENSOR_INIT_VSENSOR);
	mutex_unlock( &sensor_power_mutex_lock );

	SENSOR_LOG_I("%s(): status=0x%x end\n", __func__, sensor_init_status);
}
EXPORT_SYMBOL(sensor_power_off);

void sensor_power_reset(enum sensor_index id)
{
	SENSOR_LOG_I("%s(): start id=0x%x\n", __func__, id);

	mutex_lock(&sensor_power_mutex_lock);
	kc_sensor_power_off();
	mutex_unlock(&sensor_power_mutex_lock);
	sensor_power_on(id);

	SENSOR_LOG_I("%s(): end id=0x%x\n", __func__, id);
}
EXPORT_SYMBOL(sensor_power_reset);

int32_t sensor_power_reg_cbfunc(struct sensor_power_callback* cb)
{
	int32_t i;
	int32_t ret = -1;

	SENSOR_LOG_I("%s(): start\n", __func__);

	spin_lock(&sensor_power_spin_lock);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] == (struct sensor_power_callback*)NULL ||
			sensor_power_cb_tbl[i] == cb) {
			sensor_power_cb_tbl[i] = cb;
			ret = 0;
			break;
		}
	}
	spin_unlock(&sensor_power_spin_lock);

	SENSOR_LOG_I("%s(): end\n", __func__);

	return ret;
}
EXPORT_SYMBOL(sensor_power_reg_cbfunc);

int32_t sensor_power_unreg_cbfunc(struct sensor_power_callback* cb)
{
	int32_t i;
	int32_t ret = -1;

	SENSOR_LOG_I("%s(): start\n", __func__);

	spin_lock(&sensor_power_spin_lock);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] == cb) {
			sensor_power_cb_tbl[i] = NULL;
			ret = 0;
			break;
		}
	}
	spin_unlock(&sensor_power_spin_lock);

	SENSOR_LOG_I("%s(): end\n", __func__);

	return ret;
}
EXPORT_SYMBOL(sensor_power_unreg_cbfunc);

void kc_sensor_power_on_cbfunc(void)
{
	int32_t i;

	SENSOR_LOG_I("%s(): start\n", __func__);

	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] != (struct sensor_power_callback*)NULL) {
			SENSOR_LOG_D("%s(): sensor_power_cb_tbl[%d]->power_on()\n", __func__, i);
			sensor_power_cb_tbl[i]->power_on();
		}
	}

	SENSOR_LOG_I("%s(): end\n", __func__);

	return;
}

static void kc_sensor_power_off_cbfunc(void)
{
	int32_t i;

	SENSOR_LOG_I("%s(): start\n", __func__);

	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (sensor_power_cb_tbl[i] != (struct sensor_power_callback*)NULL) {
			SENSOR_LOG_D("%s(): sensor_power_cb_tbl[%d]->power_off()\n", __func__, i);
			sensor_power_cb_tbl[i]->power_off();
		}
	}

	SENSOR_LOG_I("%s(): end\n", __func__);

	return ;
}

static int32_t kc_sensor_power_resume(struct platform_device *pdev){
	SENSOR_LOG_I("%s is called\n",__func__);
	sensor_power_select(SENSOR_PINCTRL_ACTIVE);
	return 0;
}

static int32_t kc_sensor_power_suspend(struct platform_device *pdev, pm_message_t mesg ){
	SENSOR_LOG_I("%s is called\n",__func__);
	sensor_power_select(SENSOR_PINCTRL_SUSPEND);
	return 0;
}

static int32_t kc_sensor_power_probe(struct platform_device *pdev)
{
	int32_t rc;
	int value = 0;
#if 0
	struct qpnp_pin_cfg sleep_clk_gpio_param = {
		.mode		= QPNP_PIN_MODE_DIG_OUT,
		.output_type	= QPNP_PIN_OUT_BUF_CMOS,
		.invert		= QPNP_PIN_INVERT_DISABLE,
		.pull		= QPNP_PIN_GPIO_PULL_NO,
		.vin_sel	= QPNP_PIN_VIN2,
		.out_strength	= QPNP_PIN_OUT_STRENGTH_LOW,
		.src_sel	= QPNP_PIN_SEL_FUNC_2,
		.master_en	= QPNP_PIN_MASTER_ENABLE,
	};
#endif

	SENSOR_LOG_I("%s: \n", __func__);

	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		return -EINVAL;
	}
#ifdef CONFIG_USE_VSENSORVDD
    value = of_get_named_gpio_flags(pdev->dev.of_node, "vsensor_18", 0, NULL);
    if(value < 0) {
        pr_err("LAPIS,rst-gpio property not found");
        return -EINVAL;
    }
    sensor_power_VSENSOR = value;
    pr_debug("vsensor_18[%d]", sensor_power_VSENSOR);
    rc = gpio_request(sensor_power_VSENSOR, DRV_NAME);
    if (rc < 0)
    {
        pr_err("[SP]failed to request GPIO[%d] ret[%d]\n",sensor_power_VSENSOR,rc);
        return -EINVAL;
    }

    rc = gpio_direction_output(sensor_power_VSENSOR, 1);
    if (rc < 0){
        pr_err("[SP]failed to gpio_direction_output sensor_power_VSENSOR[%d]",rc);
    }

	sensor_data = kzalloc(sizeof(*sensor_data), GFP_KERNEL);
	if (!sensor_data) {
		pr_err("%s: sensor_data allocation failure",__func__);
		return -ENOMEM;
	}

    rc = sensor_power_pinctrl_init(pdev);
    if (rc){
        pr_err("%s: pinctrl_init failed. rc = %d", __func__, rc);
        rc = -EINVAL;
		goto err;
    }

	sensor_power_select(SENSOR_PINCTRL_ACTIVE);

#endif
#if 0
	sleep_clk_gpio = of_get_named_gpio(pdev->dev.of_node, "sens_sleep_clk-gpio", 0);
	if (sleep_clk_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		rc = -EINVAL;
		goto err;
	}

	vpro_vreg = regulator_get(&pdev->dev, "vsensor_io");
	if (IS_ERR(vpro_vreg)) {
		pr_err("%s: Cannot get regulator.\n", __func__);
		rc = PTR_ERR(vpro_vreg);
		goto err;
	}

	rc = regulator_set_voltage(vpro_vreg, 0, 0);
	if (rc) {
		pr_err("%s: Cannot set voltage.\n", __func__);
		goto err;
	}
#endif

	spin_lock_init(&sensor_power_spin_lock);
	mutex_init(&sensor_power_mutex_lock);
	memset((void*)sensor_power_cb_tbl, 0,
			sizeof(struct sensor_power_callback*)*SENSOR_INDEX_DEVICE_MAX);

	sensor_power_on(SENSOR_INDEX_ACC);
	//SENSOR_INIT_STATUS_SET(SENSOR_INIT_ACC_ON|SENSOR_INIT_GYRO_ON|SENSOR_INIT_APDS, 1);
	SENSOR_INIT_STATUS_SET(SENSOR_INIT_ACC_ON, 1);
	pr_err("%s(TEST): sensor_init_status = 0x%08X\n", __func__, sensor_init_status);
#if 0
	qpnp_pin_config(sleep_clk_gpio, &sleep_clk_gpio_param);
#endif

	return 0;

#ifdef CONFIG_USE_VSENSORVDD
err:
	kfree(sensor_data);
	gpio_free(sensor_power_VSENSOR);
#endif
#if 0
	if (vpro_vreg)
		regulator_put(vpro_vreg);
#endif
	return rc;
}

static int32_t kc_sensor_power_remove(struct platform_device *pdev)
{
	SENSOR_LOG_I("%s: \n", __func__);

#ifdef CONFIG_USE_VSENSORVDD
	gpio_free(sensor_power_VSENSOR);
#endif
#if 0
	if (vpro_vreg)
		regulator_put(vpro_vreg);
#endif
	return 0;
}

static const struct of_device_id sensor_power_of_match[] = {
	{ .compatible = "kc,sensor-power-driver", },
};

static struct platform_driver sensor_power_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sensor_power_of_match,
	},
	.probe = kc_sensor_power_probe,
	.remove = kc_sensor_power_remove,
    .suspend  = kc_sensor_power_suspend,
    .resume   = kc_sensor_power_resume,
	.shutdown = kc_sensor_power_shutdown,
};

static int32_t __init kc_sensor_power_init(void)
{
	int32_t ret = 0;

	ret = platform_driver_register(&sensor_power_driver);
	if (ret)
		goto out_region;

	SENSOR_LOG_I("%s: platform_driver_register\n", __func__);

	return 0;

out_region:
	return ret;
}

static void __exit kc_sensor_power_exit(void)
{
	platform_driver_unregister(&sensor_power_driver);
	SENSOR_LOG_I("%s: platform_driver_unregister\n", __func__);
}

module_init(kc_sensor_power_init);
module_exit(kc_sensor_power_exit);

MODULE_DESCRIPTION("Sensor power driver");
MODULE_AUTHOR("KYOCERA Corporation");
MODULE_LICENSE("GPL");
