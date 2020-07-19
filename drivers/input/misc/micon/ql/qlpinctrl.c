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
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/device.h>

#include "qlpinctrl.h"

#define LOGE(msg, ...) pr_err("%s(%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define LOGI(msg, ...) pr_info("%s(%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define LOGD(msg, ...) pr_debug("%s(%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

static const char * const qlpinctrl_names[QLPIN_ID_MAX] = {
	[QLPIN_DEFAULT]                 = "sensor_micon_default",
	[QLPIN_ACTIVE]                  = "sensor_micon_active",
	[QLPIN_SUSPEND]                 = "sensor_micon_sleep",
	[QLPIN_CS_ACTIVE]               = "sensor_micon_spi_cs_active",
	[QLPIN_CS_SUSPEND]              = "sensor_micon_spi_cs_suspend",
	[QLPIN_CS_RESET]                = "sensor_micon_spi_cs_reset",
	[QLPIN_RST_ACTIVE]              = "sensor_micon_rst_active",
	[QLPIN_RST_SUSPEND]             = "sensor_micon_rst_suspend",
	[QLPIN_VMICON_11_ACTIVE]        = "sensor_micon_vmicon_11_active",
	[QLPIN_VMICON_11_SUSPEND]       = "sensor_micon_vmicon_11_suspend",
	[QLPIN_VMICON_18_ACTIVE]        = "sensor_micon_vmicon_18_active",
	[QLPIN_VMICON_18_SUSPEND]       = "sensor_micon_vmicon_18_suspend",
	[QLPIN_SLEEP_CLK_ACTIVE]        = "sensor_micon_sleep_clk_active",
	[QLPIN_SLEEP_CLK_SUSPEND]       = "sensor_micon_sleep_clk_suspend",
	[QLPIN_VSENSOR_18_ACTIVE]       = "sensor_micon_vsensor_18_active",
	[QLPIN_VSENSOR_18_SUSPEND]      = "sensor_micon_vsensor_18_suspend",
#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
	[QLPIN_SPI_ACTIVE]              = "spi_default",
	[QLPIN_SPI_SUSPEND]             = "spi_sleep",
	[QLPIN_SPI_MICON_CS_REQUEST]    = "sensor_micon_spi_cs_request",
#endif /*CONFIG_QL_SPI_CTRL_FROMCLIENT*/
};
static struct pinctrl *qlpinctrl;
static struct pinctrl_state *qlpinctrl_state[QLPIN_ID_MAX];
static struct mutex qlpinctrl_mutex_lock;

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */
int32_t qlpinctrl_select(enum qlpin_id id)
{
	int ret;

	LOGD("Start");

	if (!qlpinctrl) {
		LOGE("pinctrl is not initialized");
		return -EINVAL;
	}

	if (qlpinctrl_state[id] == NULL) {
		LOGE("qlpinctrl_state[%d] is invalid", id);
		return -EINVAL;
	}

	mutex_lock(&qlpinctrl_mutex_lock);
	ret = pinctrl_select_state(qlpinctrl, qlpinctrl_state[id]);
	if (ret) {
		LOGE("cannot select state[%d]", id);
		ret = -EINVAL;
		goto exit;
	}

	LOGD("Selected state[%d]", id);

exit:
	mutex_unlock(&qlpinctrl_mutex_lock);
	return ret;
}

int32_t qlpinctrl_initialize(struct device *dev)
{
	int ret = 0;
	int i;

	LOGI("start");

	if (qlpinctrl) {
		LOGI("already initialized");
		return 0;
	}

	mutex_init(&qlpinctrl_mutex_lock);

	qlpinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(qlpinctrl)) {
		if (PTR_ERR(qlpinctrl) == -EPROBE_DEFER) {
			LOGE("pinctrl not ready");
			qlpinctrl = NULL;
			ret = -EPROBE_DEFER;
			goto err;
		}
		LOGE("Target does not use pinctrl");
		qlpinctrl = NULL;
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < QLPIN_ID_MAX; i++) {
		const char *n = qlpinctrl_names[i];
		struct pinctrl_state *state = pinctrl_lookup_state(qlpinctrl, n);
		if (IS_ERR(state)) {
			LOGD("cannot find '%s'", n);
			qlpinctrl_state[i] = NULL;
			continue;
		}
		LOGI("found pin control %s", n);
		qlpinctrl_state[i] = state;
	}

	LOGI("end");
	return 0;

err:
	if (qlpinctrl) {
		devm_pinctrl_put(qlpinctrl);
		qlpinctrl = NULL;
	}

	return ret;
}

int32_t qlpinctrl_remove(struct device *dev)
{
	LOGI("start");
	if (qlpinctrl) {
		devm_pinctrl_put(qlpinctrl);
		qlpinctrl = NULL;
	}
	LOGI("end");
	return 0;
}
