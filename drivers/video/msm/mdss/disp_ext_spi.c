/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include <linux/fb.h>
#include "disp_ext_spi.h"
#include "disp_ext_sub_dbg.h"

#define SPI_RETRY_TIMES 5

#define HW_SPI

#ifdef HW_SPI
static struct spi_device *disp_spi_dev_p;

static int32_t disp_ext_spi_probe(struct spi_device *client);

#define DISP_EXT_SPI_DRIVER_NAME    "disp_spi"

static struct of_device_id disp_ext_spi_match_table[] = {
	{ .compatible = DISP_EXT_SPI_DRIVER_NAME,},
	{ },
};
static struct spi_driver disp_ext_spi_driver = {
	.probe       = disp_ext_spi_probe,
	.driver = {
		.name    = DISP_EXT_SPI_DRIVER_NAME,
		.bus     = &spi_bus_type,
		.owner   = THIS_MODULE,
		.of_match_table = disp_ext_spi_match_table,
	},
};

static int32_t disp_ext_spi_probe(struct spi_device *client)
{
	int32_t ret = 0;

	if (!client) {
		pr_err("%s : spi devides NULL!\n", __func__);
		ret = -1;
	}

	disp_spi_dev_p = client;

	return ret;

}
#endif /* HW_SPI */

static int disp_ext_spi_data_init(struct disp_ext_spi_data *sdata)
{
	int ret = 0;
	pr_debug("%s start\n", __func__);

	if (!sdata){
		pr_err("%s end - null sdata[%x]\n", __func__, (int)sdata);
		return -ENODEV;
	}

#ifndef HW_SPI
	if (!gpio_is_valid(sdata->cs_gpio)) {
		pr_err("%s: spi_cs gpio not specified\n", __func__);
		return -ENODEV;
	}
	if (!gpio_is_valid(sdata->clk_gpio)) {
		pr_err("%s: spi_clk gpio not specified\n", __func__);
		return -ENODEV;
	}
	if (!gpio_is_valid(sdata->data_gpio)) {
		pr_err("%s: spi_data gpio not specified\n", __func__);
		return -ENODEV;
	}
#endif /* no HW_SPI */
	if (!gpio_is_valid(sdata->rs_gpio)) {
		pr_err("%s: spi_rs gpio not specified\n", __func__);
		return -ENODEV;
	}

#ifndef HW_SPI
	ret = gpio_request(sdata->cs_gpio, "subdisp_spi_cs");
	if (ret) {
		pr_err("%s: spi cs gpio failed, rc=%d\n", __func__, ret);
		gpio_free(sdata->cs_gpio);
		return ret;
	}
	ret = gpio_request(sdata->clk_gpio, "subdisp_spi_clk");
	if (ret) {
		pr_err("%s: spi clk gpio failed, rc=%d\n", __func__, ret);
		gpio_free(sdata->cs_gpio);
		gpio_free(sdata->clk_gpio);
		return ret;
	}
	ret = gpio_request(sdata->data_gpio, "subdisp_spi_data");
	if (ret) {
		pr_err("%s: spi data gpio failed, rc=%d\n", __func__, ret);
		gpio_free(sdata->cs_gpio);
		gpio_free(sdata->clk_gpio);
		gpio_free(sdata->data_gpio);
		return ret;
	}
#endif /* no HW_SPI */
	ret = gpio_request(sdata->rs_gpio, "subdisp_spi_rs");
	if (ret) {
		pr_err("%s: spi rs gpio failed, rc=%d\n", __func__, ret);
#ifndef HW_SPI
		gpio_free(sdata->cs_gpio);
		gpio_free(sdata->clk_gpio);
		gpio_free(sdata->data_gpio);
#endif /* no HW_SPI */
		gpio_free(sdata->rs_gpio);
		return ret;
	}

	pr_debug("%s end\n", __func__);

	return ret;
}

int disp_ext_spi_init(struct disp_ext_spi_data *sdata)
{
	int ret = 0;
	pr_debug("%s start\n", __func__);

	if (!sdata) {
		pr_err("%s end - null sdata[%x]\n", __func__, (int)sdata);
		return -ENODEV;
	}
#ifdef HW_SPI
	ret = spi_register_driver(&disp_ext_spi_driver);
#endif /* HW_SPI */

	if (0==ret) {
		ret = disp_ext_spi_data_init(sdata);
	}
	pr_debug("%s end\n", __func__);

	return ret;
}

void disp_ext_spi_ctrl_cs(struct disp_ext_spi_data *sdata, uint8_t level)
{
	(void)sdata;
	(void)level;

#ifndef HW_SPI
	if (!sdata) {
		pr_err("%s end - null sdata[%x]\n", __func__, (int)sdata);
		return;
	}
	pr_debug("%s cs_gpio set=%d\n", __func__, level);

	gpio_set_value(sdata->cs_gpio, level);
#else
	pr_debug("%s cs_gpio set=%d\n", __func__, level);
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	disp_ext_spi_pinctrl_set_state(sdata, !level);
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
#endif /* HW_SPI */
}

void disp_ext_spi_ctrl_rs(struct disp_ext_spi_data *sdata, uint8_t level)
{
	if (!sdata) {
		pr_err("%s end - null sdata[%x]\n", __func__, (int)sdata);
		return;
	}

	pr_debug("%s rs_gpio set=%d\n", __func__, level);

	gpio_set_value(sdata->rs_gpio, level);
}

#ifndef HW_SPI
int32_t disp_ext_spi_write(struct disp_ext_spi_data *sdata, uint8_t *data_p, uint32_t size)
{
	int32_t ret = 0;
	uint8_t bit_cnt;
	uint8_t data_cnt;
	uint8_t tx_data;

	if (!sdata) {
		pr_err("%s end - null sdata[%x]\n", __func__, (int)sdata);
		return -ENODEV;
	}
	if (!data_p) {
		pr_err("%s : data NULL!\n", __func__);
		return -ENODEV;
	}

	for (data_cnt = 0; data_cnt < size; data_cnt++) {
		tx_data = data_p[data_cnt];
		for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
			//clk
			gpio_set_value(sdata->clk_gpio, 0);
			udelay(1);

			//data
			if (tx_data & 0x80)
				gpio_set_value(sdata->data_gpio, 1);
			else
				gpio_set_value(sdata->data_gpio, 0);

			udelay(1);

			//clk
			gpio_set_value(sdata->clk_gpio, 1);
			udelay(1);

			tx_data <<= 1;
		}
	}

	return ret;
}
#else /* no HW_SPI */
int32_t disp_ext_spi_write(struct disp_ext_spi_data *sdata, uint8_t *data_p, uint32_t size)
{
	int32_t ret = 0;
	struct spi_message msg;
	struct spi_transfer traf;
	int retry = 0;

	(void)sdata;

	if (!data_p) {
		pr_err("%s : data NULL!\n", __func__);
	}
	if (!size) {
		pr_err("%s : data size zero!\n", __func__);
	}

	do {
		memset(&traf, 0, sizeof traf);

		spi_setup(disp_spi_dev_p);
		spi_message_init(&msg);
		spi_message_add_tail(&traf, &msg);

		traf.tx_buf = data_p;
		traf.rx_buf = NULL;
		traf.len = size;
		traf.bits_per_word = 8;

		ret = spi_sync(disp_spi_dev_p, &msg);
	} while (ret && ++retry <= SPI_RETRY_TIMES);

	pr_debug("%s end size=%d ret=%d\n", __func__, size, ret);

	return ret;
}
#endif /* HW_SPI */

int disp_ext_sub_get_spi_dt(struct device_node * np, struct device *dev, struct disp_ext_spi_data *sdata)
{
	int ret = 0;
	pr_debug("%s start\n", __func__);

	if ((!np) | (!sdata)) {
		pr_err("%s end - null np[%x] sdata[%x]\n", __func__, (int)np, (int)sdata);
		return -ENODEV;
	}

#ifndef HW_SPI
	sdata->cs_gpio = of_get_named_gpio(np, "kc,disp-ext-sub-spi-cs-gpio", 0);
	sdata->clk_gpio = of_get_named_gpio(np, "kc,disp-ext-sub-spi-clk-gpio", 0);
	sdata->data_gpio = of_get_named_gpio(np, "kc,disp-ext-sub-spi-data-gpio", 0);
#endif
	sdata->rs_gpio = of_get_named_gpio(np, "kc,disp-ext-sub-spi-rs-gpio", 0);
#ifdef HW_SPI
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	ret = disp_ext_spi_pinctrl_init(dev, sdata);
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
#endif
	pr_debug("%s end\n", __func__);
	return ret;
}

#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
int disp_ext_spi_pinctrl_init(struct device *dev, struct disp_ext_spi_data *sdata)
{
	pr_debug("%s start\n", __func__);
	sdata->pin_res.pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(sdata->pin_res.pinctrl)) {
		pr_err("%s: failed to get pinctrl\n", __func__);
		return -ENODEV;
	}

	sdata->pin_res.gpio_state_active = pinctrl_lookup_state(sdata->pin_res.pinctrl, "slcd_spi_default");
	if (IS_ERR_OR_NULL(sdata->pin_res.gpio_state_active))
		pr_warn("%s: can not get default pinstate\n", __func__);

	sdata->pin_res.gpio_state_suspend = pinctrl_lookup_state(sdata->pin_res.pinctrl, "slcd_spi_sleep");
	if (IS_ERR_OR_NULL(sdata->pin_res.gpio_state_suspend))
		pr_warn("%s: can not get sleep pinstate\n", __func__);

	pr_debug("%s end\n", __func__);
	return 0;
}

int disp_ext_spi_pinctrl_set_state(struct disp_ext_spi_data *sdata, bool active)
{
	struct pinctrl_state *pin_state;
	int rc = -EFAULT;

	pr_debug("%s: start\n", __func__);
	if (IS_ERR_OR_NULL(sdata->pin_res.pinctrl))
		return PTR_ERR(sdata->pin_res.pinctrl);

	pin_state = active ? sdata->pin_res.gpio_state_active : sdata->pin_res.gpio_state_suspend;

	if (!IS_ERR_OR_NULL(pin_state)) {
#ifdef HW_SPI
		rc = pinctrl_select_state(sdata->pin_res.pinctrl, pin_state);
#endif
		if (rc)
			pr_warn("%s: can not set pins\n", __func__);
	} else {
		pr_err("%s: invalid pinstate\n", __func__);
	}
	pr_debug("%s: end\n", __func__);
	return rc;
}
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */


