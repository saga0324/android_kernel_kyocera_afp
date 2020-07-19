/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
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
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/atomic.h>

#include "qlpinctrl.h"
#include "qlpower.h"
#include <linux/ql/qlpower_ext.h>

#define LOGE(msg, ...) pr_err("%s(%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define LOGI(msg, ...) pr_info("%s(%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define LOGD(msg, ...) pr_debug("%s(%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

enum {
	CTRL_OFF,
	CTRL_PINCTRL,
	CTRL_PM_LDO,
	CTRL_PM_GPIO,
	CTRL_MAX,
};

#ifdef CONFIG_QL_PMGPIO_CTRL
#include <linux/qpnp/pin.h>
struct qpnp_pin_cfg sleep_clk_gpio_enable_param = {
	.mode		= QPNP_PIN_MODE_DIG_OUT,
	.output_type	= QPNP_PIN_OUT_BUF_CMOS,
	.invert		= QPNP_PIN_INVERT_DISABLE,
	.pull		= QPNP_PIN_GPIO_PULL_NO,
	.vin_sel	= QPNP_PIN_VIN3,
	.out_strength	= QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel	= QPNP_PIN_SEL_FUNC_1,
	.master_en	= QPNP_PIN_MASTER_ENABLE,
};
struct qpnp_pin_cfg sleep_clk_gpio_disable_param = {
	.mode		= QPNP_PIN_MODE_DIG_IN,
	.output_type	= QPNP_PIN_OUT_BUF_CMOS,
	.invert		= QPNP_PIN_INVERT_DISABLE,
	.pull		= QPNP_PIN_GPIO_PULL_DN,
	.vin_sel	= QPNP_PIN_VIN3,
	.out_strength	= QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel	= QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en	= QPNP_PIN_MASTER_ENABLE,
};
#endif

#define NAME_MAX_SIZE 30
struct regulator_pm_data {
	struct regulator *vreg;
	int32_t voltage_min;
	int32_t voltage_max;
	int32_t load;
};

struct control_data {
	const char name[NAME_MAX_SIZE];
	int32_t ctrl;
	enum qlpin_id pinctrl_active;
	enum qlpin_id pinctrl_suspend;
	struct regulator_pm_data pm;
	int32_t gpio;
#ifdef CONFIG_QL_PMGPIO_CTRL
	struct qpnp_pin_cfg *qpnp_active;
	struct qpnp_pin_cfg *qpnp_suspend;
#endif /*CONFIG_QL_PMGPIO_CTRL*/
};

static struct control_data vmicon_11 = { "vmicon_11" };
static struct control_data vmicon_18 = { "vmicon_18" };
static struct control_data sens_sleep_clk = { "sens_sleep_clk" };
static struct control_data vsensor_18 = { "vsensor_18" };
static struct control_data* data_list[] = { &vmicon_11, &vmicon_18, &sens_sleep_clk, &vsensor_18 };

#define SENSOR_INDEX_DEVICE_MAX 10
static struct qlpower_callback* qlpower_cb_tbl[SENSOR_INDEX_DEVICE_MAX];

static spinlock_t qlpower_spin_lock;
static struct mutex qlpower_mutex_lock;
static int32_t qlpower_enabled = -1;
static int32_t qlpower_discharged = 0;
static int32_t qlpower_spi_request_count = 0;

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */
static int32_t qlpower_parse_dt(struct device_node *node)
{
	struct device_node *child_node;
	struct control_data *data;
	int32_t ret = 0;
	int32_t i = 0;

	LOGD("start");

	for (i = 0; i < ARRAY_SIZE(data_list); ++i) {
		data = data_list[i];
		child_node = of_get_child_by_name(node, data->name);

		if (!child_node) {
			LOGE("%s is not found", data->name);
			data->ctrl = 0;
			continue;
		}

		if (of_property_read_u32(child_node, "ctrl", &data->ctrl)) {
			LOGE("%s ctrl is not found", data->name);
			return -EINVAL;
		}

		if (data->ctrl >= CTRL_MAX) {
			LOGE("%s ctrl[%d] is invalid", data->name, data->ctrl);
			data->ctrl = 0;
			return -EINVAL;
		}

		if (data->ctrl == CTRL_PM_LDO) {
			ret |= of_property_read_u32(child_node, "voltage-min", &data->pm.voltage_min);
			ret |= of_property_read_u32(child_node, "voltage-max", &data->pm.voltage_max);
			ret |= of_property_read_u32(child_node, "load", &data->pm.load);
			if (ret) {
				LOGE("%s pm_data is not found", data->name);
				data->ctrl = 0;
				return -EINVAL;
			}
		}
		else if (data->ctrl == CTRL_PM_GPIO) {
			data->gpio = of_get_named_gpio_flags(child_node, "gpio", 0, NULL);
			if (data->gpio < 0) {
				LOGE("%s gpio is not found", data->name);
				data->ctrl = 0;
				return -EINVAL;
			}
		}
	}

	LOGD("end");

	return 0;
}

static int32_t qlpower_setup(struct device *dev, bool enable)
{
	struct control_data *data;
	int32_t i = 0;

	LOGD("start");

	for (i = 0; i < ARRAY_SIZE(data_list); ++i) {
		data = data_list[i];

		if (enable) {
			if (data->ctrl == CTRL_PINCTRL) {
				if (data == &vmicon_11) {
					data->pinctrl_active = QLPIN_VMICON_11_ACTIVE;
					data->pinctrl_suspend = QLPIN_VMICON_11_SUSPEND;
				}
				else if (data == &vmicon_18) {
					data->pinctrl_active = QLPIN_VMICON_18_ACTIVE;
					data->pinctrl_suspend = QLPIN_VMICON_18_SUSPEND;
				}
				else if (data == &sens_sleep_clk) {
					data->pinctrl_active = QLPIN_SLEEP_CLK_ACTIVE;
					data->pinctrl_suspend = QLPIN_SLEEP_CLK_SUSPEND;
				}
				else if (data == &vsensor_18) {
					data->pinctrl_active = QLPIN_VSENSOR_18_ACTIVE;
					data->pinctrl_suspend = QLPIN_VSENSOR_18_SUSPEND;
				}
			}
			else if (data->ctrl == CTRL_PM_LDO) {
				if (data == &sens_sleep_clk) {
					LOGE("%s cannot use pm ldo", data->name);
					return -1;
				}
				data->pm.vreg = regulator_get(dev, data->name);
				if (IS_ERR(data->pm.vreg)) {
					LOGE("regulator[%s] cannot get", data->name);
					return -1;
				}
			}
			else if (data->ctrl == CTRL_PM_GPIO) {
				if (data == &sens_sleep_clk) {
#ifdef CONFIG_QL_PMGPIO_CTRL
					data->qpnp_active = &sleep_clk_gpio_enable_param;
					data->qpnp_suspend = &sleep_clk_gpio_disable_param;
#endif /*CONFIG_QL_PMGPIO_CTRL*/
				}
				else {
					LOGE("%s cannot use pm gpio", data->name);
					return -1;
				}
			}
		}
		else {
			if (data->ctrl == CTRL_PM_LDO && data->pm.vreg > 0) {
				regulator_put(data->pm.vreg);
			}
		}
	}

	LOGD("end");

	return 0;
}

static int32_t qlpower_control_enable(struct control_data *data, bool enable)
{
	int32_t ret = 0;

	LOGD("start name[%s], ctrl[%d], enable[%d]", data->name, data->ctrl, enable);

	if (enable) {
		if (data->ctrl == CTRL_PINCTRL) {
			ret = qlpinctrl_select(data->pinctrl_active);
			if (ret) {
				LOGE("%s pinctrl_active fail. ret=%d", data->name, ret);
			}
		}
		else if (data->ctrl == CTRL_PM_LDO) {
			ret = regulator_set_voltage(data->pm.vreg, data->pm.voltage_min, data->pm.voltage_max);
			if (ret) {
				LOGE("%s regulator_set_voltage fail. ret=%d", data->name, ret);
			}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
			ret = regulator_set_load(data->pm.vreg, data->pm.load);
#else
			ret = regulator_set_optimum_mode(data->pm.vreg, data->pm.load);
#endif
			if (ret < 0) {
				LOGE("%s regulator_set_load fail. ret=%d", data->name, ret);
			}
			ret = regulator_enable(data->pm.vreg);
			if (ret) {
				LOGE("%s regulator_enable fail. ret=%d", data->name, ret);
			}
		}
		else if (data->ctrl == CTRL_PM_GPIO) {
#ifdef CONFIG_QL_PMGPIO_CTRL
			ret = qpnp_pin_config(data->gpio, data->qpnp_active);
			if (ret) {
				LOGE("%s qpnp_active fail. ret=%d", data->name, ret);
			}
#endif /*CONFIG_QL_PMGPIO_CTRL*/
		}
	} else {
		if (data->ctrl == CTRL_PINCTRL) {
			ret = qlpinctrl_select(data->pinctrl_suspend);
			if (ret) {
				LOGE("%s pinctrl_active fail. ret=%d", data->name, ret);
			}
		}
		else if (data->ctrl == CTRL_PM_LDO) {
			ret = regulator_disable(data->pm.vreg);
			if (ret) {
				LOGE("%s regulator_disable fail. ret=%d", data->name, ret);
			}
		}
		else if (data->ctrl == CTRL_PM_GPIO) {
#ifdef CONFIG_QL_PMGPIO_CTRL
			ret = qpnp_pin_config(data->gpio, data->qpnp_suspend);
			if (ret) {
				LOGE("%s qpnp_suspend fail. ret=%d", data->name, ret);
			}
#endif /*CONFIG_QL_PMGPIO_CTRL*/
		}
	}

	LOGD("end");

	return ret;
}

static int32_t qlpower_enable( bool enable )
{
	int32_t ret = 0;

	LOGD("start enable[%d]", enable);

	if (enable == 1 && qlpower_enabled == 0) {
		if (!qlpower_discharged) {
			ret = qlpower_control_enable(&vmicon_18, 1);
			if (ret) {
				LOGE("qlpower_control_enable vmicon_18 enable fail. ret=%d", ret);
				goto err;
			}
			usleep_range(10000, 10100);

			ret = qlpower_control_enable(&vmicon_11, 1);
			if (ret) {
				LOGE("qlpower_control_enable vmicon_11 enable fail. ret=%d", ret);
				goto err;
			}
			usleep_range(15000, 15100);

			ret = qlpower_control_enable(&vsensor_18, 1);
			if (ret) {
				LOGE("qlpower_control_enable vsensor_18 enable fail. ret=%d", ret);
				goto err;
			}
			usleep_range(1000, 1000);

#ifndef CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK
			ret = qlpower_control_enable(&sens_sleep_clk, 1);
			if (ret) {
				LOGE("qpnp_pin_config discharge enable fail. ret=%d", ret);
				goto err;
			}
#endif /* CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK */
			usleep_range(4000, 4100);

			ret = qlpower_control_enable(&vsensor_18, 0);
			if (ret) {
				LOGE("qlpower_control_enable vsensor_18 enable fail. ret=%d", ret);
				goto err;
			}
			usleep_range(2000, 2000);

			ret = qlpower_control_enable(&vmicon_11, 0);
			if (ret) {
				LOGE("qlpower_control_enable vmicon_11 disable fail. ret=%d", ret);
				goto err;
			}
			usleep_range(15000, 15000);

			ret = qlpower_control_enable(&vmicon_18, 0);
			if (ret) {
				LOGE("qlpower_control_enable vmicon_18 disable fail. ret=%d", ret);
				goto err;
			}
#ifndef CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK
			usleep_range(4000, 4000);

			ret = qlpower_control_enable(&sens_sleep_clk, 0);
			if (ret) {
				LOGE("qpnp_pin_config discharge disable fail. ret=%d", ret);
				goto err;
			}
#endif /* CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK */
			usleep_range(10000, 10000);

			qlpower_discharged = 1;
		}

		ret = qlpower_control_enable(&vmicon_18, 1);
		if (ret) {
			LOGE("qlpower_control_enable vmicon_18 enable fail. ret=%d", ret);
			goto err;
		}
		usleep_range(15000, 15000);

		ret = qlpower_control_enable(&vmicon_11, 1);
		if (ret) {
			LOGE("qlpower_control_enable vmicon_11 enable fail. ret=%d", ret);
			goto err;
		}
		usleep_range(15000, 15000);

		ret = qlpower_control_enable(&vsensor_18, 1);
		if (ret) {
			LOGE("qlpower_control_enable vsensor_18 enable fail. ret=%d", ret);
			goto err;
		}
		usleep_range(1000, 1000);
#ifndef CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK
		ret = qlpower_control_enable(&sens_sleep_clk, 1);
		if (ret) {
			LOGE("qpnp_pin_config sens_sleep_clk enable fail. ret=%d", ret);
			goto err;
		}
		usleep_range(1000, 1000);
#endif /* CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK */
		ret = qlpinctrl_select(QLPIN_CS_RESET);
		if (ret) {
			LOGE("qlpinctrl_select cs_reset fail. ret=%d", ret);
			goto err;
		}
		usleep_range(1000, 1000);

		ret = qlpinctrl_select(QLPIN_CS_ACTIVE);
		if (ret) {
			LOGE("qlpinctrl_select cs_active fail. ret=%d", ret);
			goto err;
		}
		usleep_range(1000,1000);

		ret = qlpinctrl_select(QLPIN_RST_ACTIVE);
		if (ret) {
			LOGE("qlpinctrl_select rst_active fail. ret=%d", ret);
			goto err;
		}
		qlpower_enabled = 1;
	} else if (enable == 0 && qlpower_enabled == 1) {
		ret = qlpinctrl_select(QLPIN_RST_SUSPEND);
		if (ret) {
			LOGE("qlpinctrl_select rst_suspend fail. ret=%d", ret);
			goto err;
		}
		udelay(20);

		ret = qlpinctrl_select(QLPIN_CS_SUSPEND);
		if (ret) {
			LOGE("qlpinctrl_select cs_suspend fail. ret=%d", ret);
			goto err;
		}
		usleep_range(1000, 1000);

		ret = qlpower_control_enable(&vsensor_18, 0);
		if (ret) {
			LOGE("qlpower_control_enable vsensor_18 disable fail. ret=%d", ret);
			goto err;
		}
		usleep_range(2000, 2000);

		ret = qlpower_control_enable(&vmicon_11, 0);
		if (ret) {
			LOGE("qlpower_control_enable vmicon_11 disable fail. ret=%d", ret);
		}
		usleep_range(15000, 15000);

		ret = qlpower_control_enable(&vmicon_18, 0);
		if (ret) {
			LOGE("qlpower_control_enable vmicon_18 disable fail. ret=%d", ret);
		}

#ifndef CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK
		usleep_range(15000, 15000);

		ret = qlpower_control_enable(&sens_sleep_clk, 0);
		if (ret) {
			LOGE("qpnp_pin_config sens_sleep_clk disable fail. ret=%d", ret);
		}
#endif /* CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK */

		qlpower_enabled = 0;
	}
	else {
		LOGI("skip, enable %d, current %d", enable, qlpower_enabled);
	}

err:
	LOGD("end");

	return ret;
}

void qlpower_reset(void)
{
	int32_t ret;
	LOGI("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return;
	}

	mutex_lock(&qlpower_mutex_lock);
	ret = qlpinctrl_select(QLPIN_CS_RESET);
	if (ret) {
		LOGE("qlpinctrl_select cs_reset fail. ret=%d", ret);
	}
	usleep_range(1000, 1000);
	ret = qlpinctrl_select(QLPIN_RST_SUSPEND);
	if (ret) {
		LOGE("qlpinctrl_select rst_suspend fail. ret=%d", ret);
	}
	ret = qlpower_control_enable(&vsensor_18, 0);
	if (ret) {
		LOGE("qlpower_control_enable vsensor_18 disable fail. ret=%d", ret);
	}
	usleep_range(3000, 3000);
	ret = qlpinctrl_select(QLPIN_RST_ACTIVE);
	if (ret) {
		LOGE("qlpinctrl_select rst_active fail. ret=%d", ret);
	}
	ret = qlpower_control_enable(&vsensor_18, 1);
	if (ret) {
		LOGE("qlpower_control_enable vsensor_18 enable fail. ret=%d", ret);
	}
	usleep_range(3000, 3000);
	ret = qlpinctrl_select(QLPIN_CS_ACTIVE);
	if (ret) {
		LOGE("qlpinctrl_select cs_active fail. ret=%d", ret);
	}
	mutex_unlock(&qlpower_mutex_lock);

	LOGI("end");
}

void qlpower_on_cbfunc(void)
{
	int32_t i;
	LOGI("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return;
	}

	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (qlpower_cb_tbl[i] != (struct qlpower_callback*)NULL) {
			LOGD("qlpower_cb_tbl[%d]->power_on()", i);
			qlpower_cb_tbl[i]->power_on();
		}
	}

	LOGI("end");
	return;
}

void qlpower_off_cbfunc(void)
{
	int32_t i;
	LOGI("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return;
	}

	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (qlpower_cb_tbl[i] != (struct qlpower_callback*)NULL) {
			LOGD("qlpower_cb_tbl[%d]->power_off()", i);
			qlpower_cb_tbl[i]->power_off();
		}
	}

	LOGI("end");
	return ;
}

void qlpower_on(void)
{
	LOGI("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return;
	}

	mutex_lock(&qlpower_mutex_lock);
	qlpower_enable(1);
	mutex_unlock(&qlpower_mutex_lock);

	LOGI("end");
}

void qlpower_off(void)
{
	LOGI("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return;
	}

	mutex_lock(&qlpower_mutex_lock);
	qlpower_enable(0);
	mutex_unlock(&qlpower_mutex_lock);

	LOGI("end");
}

int32_t qlpower_initialize(struct device *dev)
{
	int32_t ret = 0;

	LOGI("start");

	if (qlpower_enabled != -1) {
		LOGI("already initialized");
		return 0;
	}

	spin_lock_init(&qlpower_spin_lock);
	mutex_init(&qlpower_mutex_lock);
	memset((void*)qlpower_cb_tbl, 0,
			sizeof(struct qlpower_callback*)*SENSOR_INDEX_DEVICE_MAX);

	if (!dev->of_node) {
		LOGE("No spi_device supplied from device tree");
		return -EINVAL;
	}

	ret = qlpower_parse_dt(dev->of_node);
	if (ret < 0) {
		LOGE("qlpower_parse_dt error");
		return -EINVAL;
	}

	ret = qlpower_setup(dev, 1);
	if (ret < 0) {
		LOGE("qlpower_setup error");
		return -EINVAL;
	}

	qlpower_enabled = 0;

	LOGI("end");

	return 0;
}

int32_t qlpower_remove(struct device *dev)
{
	LOGI("start");
	qlpower_setup(dev, 0);
	LOGI("end");
	return 0;
}

int32_t qlpower_reg_cbfunc(struct qlpower_callback* cb)
{
	int32_t i;
	int32_t ret = -1;

	LOGD("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return -EINVAL;
	}

	spin_lock(&qlpower_spin_lock);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (qlpower_cb_tbl[i] == (struct qlpower_callback*)NULL ||
			qlpower_cb_tbl[i] == cb) {
			qlpower_cb_tbl[i] = cb;
			ret = 0;
			break;
		}
	}
	spin_unlock(&qlpower_spin_lock);

	LOGD("end");

	return ret;
}

int32_t qlpower_unreg_cbfunc(struct qlpower_callback* cb)
{
	int32_t i;
	int32_t ret = -1;

	LOGD("start");

	if (qlpower_enabled == -1) {
		LOGE("initialize is not called");
		return -EINVAL;
	}

	spin_lock(&qlpower_spin_lock);
	for (i = 0; i < SENSOR_INDEX_DEVICE_MAX; ++i) {
		if (qlpower_cb_tbl[i] == cb) {
			qlpower_cb_tbl[i] = NULL;
			ret = 0;
			break;
		}
	}
	spin_unlock(&qlpower_spin_lock);

	LOGD("end");

	return ret;
}

void qlpower_spi_request(char* name)
{
	qlpower_spi_request_count++;
	LOGI("%s count[%d]", name, qlpower_spi_request_count);
}

void qlpower_spi_release(char* name)
{
	qlpower_spi_request_count--;
	LOGI("%s count[%d]", name, qlpower_spi_request_count);

	if (qlpower_spi_request_count == 0) {
		qlpower_off();
	}
}
