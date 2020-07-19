/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
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
#include <mach/gpiomux.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>

#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "disp_ext_sub_ctrl.h"
#include "disp_ext_spi.h"

/* Draw data */
#define DISP_SUBLCD_BIT0_MASK           0x08        /*  VRAM bit0 data mask*/
#define DISP_SUBLCD_BIT1_MASK           0x10        /*  VRAM bit1 data mask*/
#define DISP_SUBLCD_BIT0_SHIFT          3           /*  VRAM bit0 data mask*/
#define DISP_SUBLCD_BIT1_SHIFT          4           /*  VRAM bit1 data mask*/

/* SubLCD data */
#define DISP_SUBLCD_PAGE_START_ADDR         0x00        /*  VRAM start page addr */
#define DISP_SUBLCD_PAGE_END_ADDR           0x0B        /*  VRAM end page addr */
#define DISP_SUBLCD_PAGE_NUM                0x0C        /*  VRAM page addr 96pixel(1Byte step) */
#define DISP_SUBLCD_COLUMN_NUM              0x60        /*  VRAM column addr 96pixel*/
#define DISP_SUBLCD_PAGE_ALL_NUM            0x10        /*  VRAM page addr 0-15 */
#define DISP_SUBLCD_COLUMN_ALL_NUM          0x80        /*  VRAM column addr 0-127 */
#define DISP_SUBLCD_LINE_NUM                0x08        /*  VRAM line/Page  */
#define DISP_SUBLCD_UNIT_NUM                DISP_SUBLCD_LINE_NUM * DISP_SUBLCD_COLUMN_NUM   /* Page per Pixel */
#define DISP_SUBLCD_SETPAGE_ADDR            0xB0        /*  VRAM page addr format */
#define DISP_SUBLCD_SETCOLUMN_ADDR_MSB      0x11        /*  VRAM column addr MSB side */
#define DISP_SUBLCD_SETCOLUMN_ADDR_LSB      0x00        /*  VRAM column addr LSB side */
#define DISP_SUBLCD_ALL_SETCOLUMN_ADDR_MSB  0x10        /*  VRAM column addr MSB side all area */
#define DISP_SUBLCD_ALL_SETCOLUMN_ADDR_LSB  0x00        /*  VRAM column addr LSB side all area */

int disp_ext_sub_temp_thresh_tbl[] =
{
	-200, -150, -100, -50, 0,
	50, 100, 150, 200, 250,
	300, 350, 400, 450, 500,
	550, 600, 650, 700
};

static struct fb_var_subdispinfo subdispinfo_default = {
	FB_SUB_USERCONT_TYPE_3,
	0x30,
	{0xF5, 0xF6, 0xF7, 0xF9, 0xFB, 0xFC, 0xFE, 0xFF, 0x00, 0x00, 0x01, 0x02, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04},
	{0xFD, 0xFE, 0x00, 0x02, 0x03},
	FB_SUB_LANGUAGE_TYPE_JP,
	FB_SUB_WATCH_TYPE_DIGITAL,
	FB_SUB_CLOCK_TYPE_24H,
	0,
	{0x00},
};

static void disp_ext_sub_conv_img(struct fb_var_screeninfo *var, uint8_t* src_p, uint8_t* dst_p);
static int disp_ext_sub_send_img(struct disp_ext_sub_pdata *pdata, uint8_t* img_p);
static int disp_ext_sub_exe_seq(struct disp_ext_sub_pdata *pdata, struct disp_ext_sub_cmds *sub_cmds_p);
static int disp_ext_sub_set_wait(void * payload_p, unsigned char payload_len);
static int disp_ext_sub_set_sig(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
static void disp_ext_sub_set_gpio(int gpio_handle, int on);
static int disp_ext_sub_get_cmd_dt(struct device_node *np, struct disp_ext_sub_cmds *cmd_p, char *cmd_key);

static int disp_ext_sub_panel_set_status_off(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_sub_cmds *sub_cmds_p = NULL;

	pr_debug("%s start\n", __func__);

	switch (pdata->state) {
	case DISP_SUB_STATE_ALLWAYSON:
		ret = disp_ext_sub_mdm_change_dispctl(false, pdata);
		if (ret) {
			pr_err("%s modem stop change error %d\n", __func__, ret);
			break;
		}
		// fallthrough
	case DISP_SUB_STATE_ON:
	case DISP_SUB_STATE_PWR_ON:
		pr_debug("%s: OFF seq start\n", __func__);
		sub_cmds_p = &(pdata->off_cmd);
		ret = disp_ext_sub_exe_seq(pdata, sub_cmds_p);
		if (ret) {
			pr_err("%s OFF error %d\n", __func__, ret);
			break;
		}
		pr_debug("%s: OFF seq end\n", __func__);
		break;
	case DISP_SUB_STATE_OFF:
		break;
	case DISP_SUB_STATE_PWR_OFF:
	default:
		pr_err("%s end - bad state %d\n", __func__, pdata->state);
		ret = -EPERM;
		break;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static int disp_ext_sub_panel_set_status_on(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_sub_cmds *sub_cmds_p = NULL;

	pr_debug("%s start\n", __func__);

	switch (pdata->state) {
	case DISP_SUB_STATE_PWR_ON:
	case DISP_SUB_STATE_OFF:
		pr_debug("%s: ON seq start\n", __func__);
		sub_cmds_p = &(pdata->on_cmd);
		ret = disp_ext_sub_exe_seq(pdata, sub_cmds_p);
		if (ret) {
			pr_err("%s: ON error %d\n", __func__, ret);
			break;
		}
		pdata->first_update = true;
		disp_ext_sub_set_contrast(pdata, true);	/* no error handling */
		pr_debug("%s: ON seq end\n", __func__);
		break;
	case DISP_SUB_STATE_ALLWAYSON:
		ret = disp_ext_sub_mdm_change_dispctl(false, pdata);
		if (ret) {
			pr_err("%s modem stop change error %d\n", __func__, ret);
			break;
		}
		pdata->first_update = true;
		disp_ext_sub_set_contrast(pdata, true);	/* no error handling */
		break;
	case DISP_SUB_STATE_ON:
		break;
	case DISP_SUB_STATE_PWR_OFF:
	default:
		pr_err("%s end - bad state %d\n", __func__, pdata->state);
		ret = -EPERM;
		break;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static int disp_ext_sub_panel_set_status_alwayson(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_sub_cmds *sub_cmds_p = NULL;

	pr_debug("%s start\n", __func__);

	switch (pdata->state) {
	case DISP_SUB_STATE_PWR_ON:
	case DISP_SUB_STATE_OFF:
		pr_debug("%s: ON seq start\n", __func__);
		sub_cmds_p = &(pdata->on_cmd);
		ret = disp_ext_sub_exe_seq(pdata, sub_cmds_p);
		if (ret) {
			pr_err("%s: ON error %d\n", __func__, ret);
			break;
		}
		pdata->first_update = true;
		disp_ext_sub_set_contrast(pdata, true);	/* no error handling */
		pr_debug("%s: ON seq end\n", __func__);
		// fallthrough
	case DISP_SUB_STATE_ON:
		ret = disp_ext_sub_mdm_change_dispctl(true, pdata);
		if (ret) {
			pr_err("%s modem start change error %d\n", __func__, ret);
			break;
		}
		break;
	case DISP_SUB_STATE_ALLWAYSON:
		break;
	case DISP_SUB_STATE_PWR_OFF:
	default:
		pr_err("%s end - bad state %d\n", __func__, pdata->state);
		ret = -EPERM;
		break;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static int disp_ext_sub_panel_set_status_power_off(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_sub_cmds *sub_cmds_p = NULL;

	pr_debug("%s start\n", __func__);

	switch (pdata->state) {
	case DISP_SUB_STATE_ALLWAYSON:
		ret = disp_ext_sub_mdm_change_dispctl(false, pdata);
		if (ret) {
			pr_err("%s modem stop change error %d\n", __func__, ret);
			break;
		}
		// fallthrough
	case DISP_SUB_STATE_ON:
		pr_debug("%s: OFF seq start\n", __func__);
		sub_cmds_p = &(pdata->off_cmd);
		ret = disp_ext_sub_exe_seq(pdata, sub_cmds_p);
		if (ret) {
			pr_err("%s OFF error %d\n", __func__, ret);
			break;
		}
		pr_debug("%s: OFF seq end\n", __func__);
		// fallthrough
	case DISP_SUB_STATE_PWR_ON:
	case DISP_SUB_STATE_OFF:
		pr_debug("%s: PWR_OFF seq start\n", __func__);
		sub_cmds_p = &(pdata->pwroff_cmd);
		ret = disp_ext_sub_exe_seq(pdata, sub_cmds_p);
		if (ret) {
			pr_err("%s PWR_OFF error %d\n", __func__, ret);
			break;
		}
		pr_debug("%s: PWR_OFF seq end\n", __func__);
		break;
	case DISP_SUB_STATE_PWR_OFF:
	default:
		pr_err("%s end - bad state %d\n", __func__, pdata->state);
		ret = -EPERM;
		break;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static int disp_ext_sub_panel_set_status_power_on(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_sub_cmds *sub_cmds_p = NULL;

	pr_debug("%s start\n", __func__);

	if (pdata->state != DISP_SUB_STATE_PWR_OFF) {
		pr_debug("%s subdisplay already power on\n", __func__);
		return 0;
	}
	pr_debug("%s: PWR_ON seq start\n", __func__);
	sub_cmds_p = &(pdata->pwron_cmd);
	ret = disp_ext_sub_exe_seq(pdata, sub_cmds_p);
	pr_debug("%s: PWR_ON seq end\n", __func__);

	pr_debug("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_panel_set_status(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;

	pr_debug("%s start\n", __func__);

	if (!pdata) {
		pr_err("%s end - pdata is null\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s current:%d next:%d\n", __func__, pdata->state, next_status);

	switch (next_status) {
	case DISP_SUB_STATE_OFF:
		ret = disp_ext_sub_panel_set_status_off(next_status, pdata);
		break;
	case DISP_SUB_STATE_ON:
		ret = disp_ext_sub_panel_set_status_on(next_status, pdata);
		break;
	case DISP_SUB_STATE_ALLWAYSON:
		ret = disp_ext_sub_panel_set_status_alwayson(next_status, pdata);
		break;
	case DISP_SUB_STATE_PWR_OFF:
		ret = disp_ext_sub_panel_set_status_power_off(next_status, pdata);
		break;
	case DISP_SUB_STATE_PWR_ON:
		ret = disp_ext_sub_panel_set_status_power_on(next_status, pdata);
		break;
	default:
		pr_err("%s : undefine state\n", __func__);
		ret = -1;
		break;
	}
	if (!ret) {
		pdata->state = next_status;
	}
	pr_debug("%s end %d\n", __func__, ret);
	return ret;
}

int disp_ext_sub_panel_update(struct fb_var_screeninfo *var, struct fb_info *info, uint8_t* apps_img_p)
{
	int ret = 0;
	uint8_t* send_img_p;
	uint32_t size;
	struct disp_ext_sub_info* sub_info_p;
	struct disp_ext_sub_pdata* pdata;

	pr_debug("%s start\n", __func__);

	if ((!var) || (!info) || (!apps_img_p)) {
		pr_err("%s end - bad parm: var[%x] info[%x] img[%x]\n", __func__, (int)var, (int)info, (int)apps_img_p);
		return -ENODEV;
	}

	sub_info_p = (struct disp_ext_sub_info*)info->par;
	pdata = &(sub_info_p->pdata);
	if (pdata->state != DISP_SUB_STATE_ON) {
		pr_err("%s end - bad state:%d\n", __func__, pdata->state);
		return -EPERM;
	}

	size = (var->xres) * (var->yres) * 2 / 8;
	send_img_p = (uint8_t*)kmalloc(size, GFP_KERNEL);
	if (!send_img_p) {
		pr_err("%s : fail malloc\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	disp_ext_sub_img_dump((void*)apps_img_p, ((var->xres)*(var->yres)*2), 2);
	disp_ext_sub_conv_img(var, apps_img_p, send_img_p);
	disp_ext_sub_img_dump((void*)send_img_p, ((var->xres)*(var->yres)), 1);

	disp_ext_sub_set_contrast(pdata, false);

	ret = disp_ext_sub_send_img(pdata, send_img_p);
	if (ret) {
		pr_err("%s: disp_ext_sub_send_img error:%d, do recovery\n", __func__, ret);
		ret = disp_ext_sub_exe_seq(pdata, &pdata->on_cmd);
		pdata->first_update = true;
		ret = disp_ext_sub_send_img(pdata, send_img_p);
	}
	kfree(send_img_p);

	if (pdata->first_update) {
		ret = disp_ext_sub_exe_seq(pdata, &pdata->post_on_cmd);
		if (ret) {
			pr_err("%s : Display ON command fail.\n", __func__);
			goto exit;
		}
		pr_debug("%s Display ON command send for first time update.\n", __func__);
		pdata->first_update = false;
	}

exit:
	pr_debug("%s end\n", __func__);
	return ret;
}

static void disp_ext_sub_conv_img(struct fb_var_screeninfo *var, uint8_t* src_p, uint8_t* dst_p)
{
	uint16_t i, j;
	uint8_t  bit1_dat, bit0_dat;
	uint16_t *page_ptr = NULL;
	uint16_t *column_ptr = NULL;
	uint16_t *line_ptr = NULL;
	uint16_t    tmp_dat;
	uint8_t    dot_shift;

	pr_debug("%s start\n", __func__);

	page_ptr = (uint16_t *)src_p;
	/* Page address [0 - 11] */
	for (i = 0; i < DISP_SUBLCD_PAGE_NUM; i++) {
		column_ptr = page_ptr;
		/* Column [10h - 6Fh] */
		for (j = 0; j < DISP_SUBLCD_COLUMN_NUM; j++) {
			bit0_dat = 0;
			bit1_dat = 0;
			line_ptr = column_ptr;
			/* Data [0 - 8]*/
			for (dot_shift = 0; dot_shift < DISP_SUBLCD_LINE_NUM; dot_shift++) {
				tmp_dat = ((*line_ptr) & DISP_SUBLCD_BIT1_MASK) >> DISP_SUBLCD_BIT1_SHIFT;
				bit1_dat |= tmp_dat << dot_shift;
				tmp_dat = ((*line_ptr) & DISP_SUBLCD_BIT0_MASK) >> DISP_SUBLCD_BIT0_SHIFT;
				bit0_dat |= tmp_dat << dot_shift;
				line_ptr += DISP_SUBLCD_COLUMN_NUM;
			}
			*dst_p++ = bit1_dat;
			*dst_p++ = bit0_dat;

			column_ptr++;
		}
		page_ptr += DISP_SUBLCD_UNIT_NUM;
	}
	
	pr_debug("%s end\n", __func__);
	return;
}

static int disp_ext_sub_send_img(struct disp_ext_sub_pdata *pdata, uint8_t* img_p)
{
	int ret;
	uint16_t   i;
	uint16_t  page_n;
	struct disp_ext_spi_data *sdata;
	uint8_t *page_addr;
	uint8_t page_data = 0;
	uint8_t *img_p_colum = img_p;

	pr_debug("%s start\n", __func__);

	if ((!pdata) || (!img_p)) {
		pr_err("%s end - bad parm: pdata[%x] img[%x]\n", __func__, (int)pdata, (int)img_p);
		return -ENODEV;
	}

	sdata = &pdata->spi_data;

	disp_ext_spi_ctrl_cs(sdata, 0);
	
	/* Page address [0 - 11] */
	page_addr = (uint8_t *)pdata->ram_wr_cmd.cmd_p[0].payload_p;
	page_data = *page_addr;
	for (i = 0; i < DISP_SUBLCD_PAGE_NUM; i++) {
		disp_ext_spi_ctrl_rs( sdata, 0 );

		page_n = (uint16_t)(i + DISP_SUBLCD_PAGE_START_ADDR);
		*page_addr = page_data | page_n;
		ret = disp_ext_sub_exe_seq(pdata, &pdata->ram_wr_cmd);
		if (ret) {
			pr_err("%s : Write in GRAM command fail.\n", __func__);
			break;
		}

		disp_ext_spi_ctrl_rs(sdata, 1);

		/* Column [10h - 6Fh] */
		disp_ext_spi_write(sdata, img_p_colum, DISP_SUBLCD_COLUMN_NUM * 2);
		img_p_colum += DISP_SUBLCD_COLUMN_NUM * 2;
	}

	disp_ext_spi_ctrl_rs(sdata, 0);
	disp_ext_spi_ctrl_cs(sdata, 1);
	
	*page_addr = page_data;
	
	pr_debug("%s end\n", __func__);
	return 0;
}

static int disp_ext_sub_exe_seq(struct disp_ext_sub_pdata *pdata, struct disp_ext_sub_cmds *sub_cmds_p)
{
	int ret = 0;
	unsigned int cmd_cnt;
	unsigned char ctrl_kind;
	unsigned char payload_len;
	void * payload_p;
	int i;

	pr_debug("%s start\n", __func__);

	if (!sub_cmds_p) {
		pr_err("%s end - null cmd data[%x]\n", __func__, (int)sub_cmds_p);
		return -ENODEV;
	}

	cmd_cnt = sub_cmds_p->cmd_cnt;

	for (i=0; i<cmd_cnt; i++) {
		ctrl_kind = sub_cmds_p->cmd_p[i].cmd_hdr.ctrl_kind;
		payload_len = sub_cmds_p->cmd_p[i].cmd_hdr.payload_len;
		payload_p = sub_cmds_p->cmd_p[i].payload_p;

		switch (ctrl_kind) {
		case DISP_SUB_CTRL_CMD:
			ret = disp_ext_sub_set_cmd(payload_p, payload_len, pdata);
			break;
		case DISP_SUB_CTRL_WAIT:
			ret = disp_ext_sub_set_wait(payload_p, payload_len);
			break;
		case DISP_SUB_CTRL_SIG:
			ret = disp_ext_sub_set_sig(payload_p, payload_len, pdata);
			break;
		default:
			break;
		}
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_set_cmd2(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_spi_data *sdata;

	pr_debug("%s start\n", __func__);

	sdata = &pdata->spi_data;

	disp_ext_spi_ctrl_cs(sdata, 0);
	disp_ext_spi_ctrl_rs(sdata, 0);

	ret = disp_ext_sub_set_cmd(payload_p, payload_len, pdata);

	disp_ext_spi_ctrl_rs(sdata, 0);
	disp_ext_spi_ctrl_cs(sdata, 1);

	pr_debug("%s end %d\n", __func__, ret);
	return ret;
}

int disp_ext_sub_set_cmd(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_spi_data *sdata;

	if ((!payload_p) || (!pdata)) {
		pr_err("%s end - null payload[%x] pdata[%d]\n", __func__, (int)payload_p, (int)pdata);
		return -ENODEV;
	}
	if (!payload_len) {
		pr_err("%s end - bad length :payload_len[%x]\n", __func__, (int)payload_len);
		return -ENODEV;
	}

	sdata = &pdata->spi_data;
	ret = disp_ext_spi_write(sdata, payload_p, payload_len);

	return ret;
}

static int disp_ext_sub_set_wait(void * payload_p, unsigned char payload_len)
{
	int ret = 0;
	unsigned char *data_p;
	unsigned char offset;
	unsigned long us_time = 0;
	int i;

	if (!payload_p) {
		pr_err("%s end - null payload[%x]\n", __func__, (int)payload_p);
		return -ENODEV;
	}
	if (!payload_len) {
		pr_err("%s end - bad length :payload_len[%x]\n", __func__, (int)payload_len);
		return -ENODEV;
	}

	data_p = (unsigned char*)payload_p;
	offset = payload_len;

	for (i=0; i<payload_len; i++) {
		offset--;
		us_time |= data_p[i]<<(offset*8);
	}

	pr_debug("%s: wait [%d]us\n", __func__, (int)us_time);
	usleep(us_time);

	return ret;
}

static int disp_ext_sub_set_sig(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	unsigned char *data_p;
	unsigned char sig_kind;
	int on;
	int gpio_handle = 0;
	struct disp_ext_spi_data *sdata;

	sdata = &pdata->spi_data;


	pr_debug("%s start\n", __func__);

	if ((!payload_p) || (!pdata)) {
		pr_err("%s end - null payload[%x] pdata[%d]\n", __func__, (int)payload_p, (int)pdata);
		return -ENODEV;
	}
	
	if (!payload_len) {
		pr_err("%s end - bad length :payload_len[%x]\n", __func__, (int)payload_len);
		return -ENODEV;
	}
	
	data_p = (unsigned char*)payload_p;
	sig_kind = data_p[0];
	on = (int)data_p[1];

	switch (sig_kind) {
	case DISP_SUB_SIG_RESET:
		gpio_handle = pdata->rst_gpio;
		pr_debug("%s: set Reset\n", __func__);
		break;
	case DISP_SUB_SIG_SPI_CS:
		disp_ext_spi_ctrl_cs(sdata, on);
		pr_debug("%s: set cs by spi driver\n", __func__);
		return 0;
	case DISP_SUB_SIG_SPI_RS:
		disp_ext_spi_ctrl_rs(sdata, on);
		pr_debug("%s: set rs by spi driver\n", __func__);
		return 0;
	default:
		pr_err("%s : undefine sig[%x]\n", __func__, (int)sig_kind);
		break;
	}

	if (gpio_handle) {
		disp_ext_sub_set_gpio(gpio_handle, on);
	} else {
		ret = -1;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static void disp_ext_sub_set_gpio(int gpio_handle, int on)
{
	pr_debug("%s gpio[%d] -> %d\n", __func__,  gpio_handle, on);
	gpio_set_value(gpio_handle, on);
	return;
}

int disp_ext_sub_panel_bus_init(struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_spi_data *sdata;

	pr_debug("%s start\n", __func__);

	if (!pdata) {
		pr_err("%s end - null pdata[%x]\n", __func__, (int)pdata);
		return -ENODEV;
	}

	sdata = &pdata->spi_data;
	ret = disp_ext_spi_init(sdata);

	pr_debug("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_panel_signal_init(struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	pr_err("%s start\n", __func__);

	if (!pdata) {
		pr_err("%s end - null pdata[%x]\n", __func__, (int)pdata);
		return -ENODEV;
	}

	if (!gpio_is_valid(pdata->rst_gpio)) {
		pr_err("%s: reset gpio not specified\n",__func__ );
		return -ENODEV;
	} 

	ret = gpio_request(pdata->rst_gpio, "subdisp_rst_n");
	if (ret) {
		pr_err("%s: request reset gpio failed, rc=%d\n", __func__, ret);
		gpio_free(pdata->rst_gpio);
		return ret;
	}

	disp_ext_sub_set_gpio(pdata->rst_gpio, 1);

	pr_err("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_get_panel_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	u32 tmp;

	pr_debug("%s start\n", __func__);

	if ((!np) || (!pdata)) {
		pr_err("%s end - null np[%x] pdata[%x]\n", __func__, (int)np, (int)pdata);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "kc,disp-ext-sub-panel-width", &tmp);
	if (ret) {
		pr_err("%s end - fail get panel width\n", __func__);
		return -EINVAL;
	}
	pdata->xres = tmp;

	ret = of_property_read_u32(np, "kc,disp-ext-sub-panel-height", &tmp);
	if (ret) {
		pr_err("%s end - fail get panel width\n", __func__);
		return -EINVAL;
	}
	pdata->yres = tmp;

	ret = of_property_read_u32(np, "kc,disp-ext-sub-bpp", &tmp);
	if (ret) {
		pr_err("%s end - fail get bpp\n", __func__);
		return -EINVAL;
	}
	pdata->bpp = tmp;

	pr_debug("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_get_signal_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	pr_debug("%s start\n", __func__);

	if ((!np) || (!pdata)) {
		pr_err("%s end - null np[%x] pdata[%x]\n", __func__, (int)np, (int)pdata);
		return -ENODEV;
	}

	pdata->rst_gpio = of_get_named_gpio(np, "kc,disp-ext-sub-rst-gpio", 0);

	pr_debug("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_get_seq_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;

	pr_debug("%s start\n", __func__);

	if ((!np) || (!pdata)) {
		pr_err("%s end - null np[%x] pdata[%x]\n", __func__, (int)np, (int)pdata);
		return -ENODEV;
	}

	ret = disp_ext_sub_get_cmd_dt(np,&(pdata->pwron_cmd), "kc,disp-ext-sub-pwron-cmd");
	if (ret) {
		pr_err("%s end - fail get power on command\n", __func__);
		return -EINVAL;
	}

	ret = disp_ext_sub_get_cmd_dt(np, &(pdata->on_cmd), "kc,disp-ext-sub-on-cmd");
	if (ret) {
		pr_err("%s end - fail get on command\n", __func__);
		return -EINVAL;
	}

	ret = disp_ext_sub_get_cmd_dt(np, &(pdata->post_on_cmd), "kc,disp-ext-sub-post-on-cmd");
	if (ret) {
		pr_err("%s end - fail get post on command\n", __func__);
		return -EINVAL;
	}

	ret = disp_ext_sub_get_cmd_dt(np, &(pdata->off_cmd), "kc,disp-ext-sub-off-cmd");
	if (ret) {
		pr_err("%s end - fail get off command\n", __func__);
		return -EINVAL;
	}

	ret = disp_ext_sub_get_cmd_dt(np, &(pdata->pwroff_cmd), "kc,disp-ext-sub-pwroff-cmd");
	if (ret) {
		pr_err("%s end - fail get power off command\n", __func__);
		return -EINVAL;
	}

	ret = disp_ext_sub_get_cmd_dt(np, &(pdata->ram_wr_cmd), "kc,disp-ext-sub-ram-wr-cmd");
	if (ret) {
		pr_err("%s end - fail get ram write command\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s end\n", __func__);
	return ret;
}

static int disp_ext_sub_get_cmd_dt(struct device_node *np, struct disp_ext_sub_cmds *sub_cmd_p, char *cmd_key)
{
	int ret = 0;

	const char *dt_data_p;
	int dt_data_len = 0;
	char *buf_p;

	struct disp_ext_sub_cmd_hdr *cmd_hdr_p;
	int dt_data_tmp_len = 0;
	char *buf_tmp_p;

	int i, cnt;

	pr_debug("%s start\n", __func__);

	dt_data_p = of_get_property(np, cmd_key, &dt_data_len);
	pr_debug("%s cmd_key[%s] dt_data_p[%x] dt_data_len[%d]\n", __func__, cmd_key ? cmd_key : "(null)", (int)dt_data_p, (int)dt_data_len);
	if (!dt_data_p) {
		pr_err("%s end - fail key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf_p = kzalloc(sizeof(char) * dt_data_len, GFP_KERNEL);
	if (!buf_p) {
		pr_err("%s end - fail kzalloc\n", __func__);
		return -ENOMEM;
	}

	memcpy(buf_p, dt_data_p, dt_data_len);

	buf_tmp_p = buf_p;
	dt_data_tmp_len = dt_data_len;
	cnt = 0;
	while (dt_data_tmp_len > sizeof(struct disp_ext_sub_cmd_hdr)) {
		cmd_hdr_p = (struct disp_ext_sub_cmd_hdr *)buf_tmp_p;
		if (cmd_hdr_p->payload_len > dt_data_tmp_len) {
			pr_err("%s payload_len[%d] dt_data_tmp_len[%d]\n", __func__, (int)(cmd_hdr_p->payload_len), (int)dt_data_tmp_len);
			goto exit_free;
		}

		buf_tmp_p += sizeof(struct disp_ext_sub_cmd_hdr);
		buf_tmp_p += cmd_hdr_p->payload_len;
		dt_data_tmp_len -= sizeof(struct disp_ext_sub_cmd_hdr);
		dt_data_tmp_len -= cmd_hdr_p->payload_len;

		cnt++;
	}

	if (dt_data_tmp_len != 0) {
		pr_err("%s dt_data_tmp_len[%d]\n", __func__, (int)dt_data_tmp_len);
		goto exit_free;
	}

	sub_cmd_p->cmd_p = kzalloc(cnt * sizeof(struct  disp_ext_sub_cmd_detail), GFP_KERNEL);

	if (!sub_cmd_p->cmd_p) {
		goto exit_free;
	}

	sub_cmd_p->buf_p = buf_p;
	sub_cmd_p->blen = dt_data_len;
	sub_cmd_p->cmd_cnt = cnt;

	buf_tmp_p = buf_p;
	dt_data_tmp_len = dt_data_len;
	for (i = 0; i < cnt; i++) {
		cmd_hdr_p = (struct disp_ext_sub_cmd_hdr *)buf_tmp_p;
		buf_tmp_p += sizeof(struct disp_ext_sub_cmd_hdr);
		sub_cmd_p->cmd_p[i].cmd_hdr = *cmd_hdr_p;
		sub_cmd_p->cmd_p[i].payload_p = buf_tmp_p;

		buf_tmp_p += cmd_hdr_p->payload_len;
		dt_data_tmp_len -= cmd_hdr_p->payload_len;
	}

	pr_debug("%s end\n", __func__);
	return ret;

exit_free:
	kfree(buf_p);
	return -ENOMEM;
}

int disp_ext_sub_subdispinfo_init(struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;

	pdata->subdispinfo = subdispinfo_default;
	pdata->current_device_elec_vol = 0;

	return ret;
}

int disp_ext_sub_set_subdispinfo(struct disp_ext_sub_pdata *pdata, struct fb_var_subdispinfo *psubdispinfo)
{
	int ret = 0;
	__u8 user_contrast = FB_SUB_USERCONT_TYPE_NONE;
	const __u8 AllF[18] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	pr_debug("%s start\n", __func__);

	pr_debug("user_contrast=%02X\n",psubdispinfo->user_contrast);
	pr_debug("elec_vol_init=%02X\n",psubdispinfo->elec_vol_init);
	pr_debug("elec_vol_temp=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
			psubdispinfo->elec_vol_temp[0], psubdispinfo->elec_vol_temp[1], psubdispinfo->elec_vol_temp[2], psubdispinfo->elec_vol_temp[3],
			psubdispinfo->elec_vol_temp[4], psubdispinfo->elec_vol_temp[5], psubdispinfo->elec_vol_temp[6], psubdispinfo->elec_vol_temp[7],
			psubdispinfo->elec_vol_temp[8], psubdispinfo->elec_vol_temp[9], psubdispinfo->elec_vol_temp[10], psubdispinfo->elec_vol_temp[11],
			psubdispinfo->elec_vol_temp[12], psubdispinfo->elec_vol_temp[13], psubdispinfo->elec_vol_temp[14], psubdispinfo->elec_vol_temp[15],
			psubdispinfo->elec_vol_temp[16], psubdispinfo->elec_vol_temp[17]);
	pr_debug("elec_vol_user=%02X %02X %02X %02X %02X\n",
			psubdispinfo->elec_vol_user[0],psubdispinfo->elec_vol_user[1], psubdispinfo->elec_vol_user[2], psubdispinfo->elec_vol_user[3],
			psubdispinfo->elec_vol_user[4]);
	pr_debug("langage=%d\n", psubdispinfo->langage);
	pr_debug("watch_dispinfo=%d\n", psubdispinfo->watch_dispinfo);
	pr_debug("clock_dispinfo=%d\n", psubdispinfo->clock_dispinfo);
	pr_debug("timezone=%d\n", psubdispinfo->timezone);

	if (psubdispinfo->user_contrast != FB_SUB_USERCONT_TYPE_NONE) {
		if (psubdispinfo->user_contrast != pdata->subdispinfo.user_contrast) {
			pdata->subdispinfo.user_contrast = user_contrast = psubdispinfo->user_contrast;
		}
	}
	if (psubdispinfo->elec_vol_init != 0xFF && psubdispinfo->elec_vol_init != 0x00) {
		pdata->subdispinfo.elec_vol_init = psubdispinfo->elec_vol_init;
	}
	if (memcmp(psubdispinfo->elec_vol_temp, AllF, sizeof(psubdispinfo->elec_vol_temp)) && psubdispinfo->elec_vol_temp[0] != 0) {
		memcpy(pdata->subdispinfo.elec_vol_temp, psubdispinfo->elec_vol_temp, sizeof(psubdispinfo->elec_vol_temp));
	}
	if (memcmp(psubdispinfo->elec_vol_user, AllF, sizeof(psubdispinfo->elec_vol_user)) && psubdispinfo->elec_vol_user[0] != 0) {
		memcpy(pdata->subdispinfo.elec_vol_user, psubdispinfo->elec_vol_user, sizeof(psubdispinfo->elec_vol_user));
	}
	if (psubdispinfo->langage != FB_SUB_LANGUAGE_TYPE_NONE) {
		pdata->subdispinfo.langage = psubdispinfo->langage;
	}
	if (psubdispinfo->watch_dispinfo != FB_SUB_WATCH_TYPE_NONE) {
		pdata->subdispinfo.watch_dispinfo = psubdispinfo->watch_dispinfo;
	}
	if (psubdispinfo->clock_dispinfo != FB_SUB_CLOCK_TYPE_NONE) {
		pdata->subdispinfo.clock_dispinfo = psubdispinfo->clock_dispinfo;
	}
	if (psubdispinfo->timezone != -1) {
		pdata->subdispinfo.timezone = psubdispinfo->timezone;
	}

	if (user_contrast != FB_SUB_USERCONT_TYPE_NONE && pdata->state == DISP_SUB_STATE_ON) {
		ret = disp_ext_sub_set_contrast(pdata, false);
	}

	pr_debug("%s end %d\n", __func__, ret);
	return ret;
}

int disp_ext_sub_get_subdispinfo(struct disp_ext_sub_pdata *pdata)
{
	pr_notice("user_contrast=%02X\n",pdata->subdispinfo.user_contrast);
	pr_notice("elec_vol_init=%02X\n",pdata->subdispinfo.elec_vol_init);
	pr_notice("elec_vol_temp=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
			pdata->subdispinfo.elec_vol_temp[0], pdata->subdispinfo.elec_vol_temp[1], pdata->subdispinfo.elec_vol_temp[2], pdata->subdispinfo.elec_vol_temp[3],
			pdata->subdispinfo.elec_vol_temp[4], pdata->subdispinfo.elec_vol_temp[5], pdata->subdispinfo.elec_vol_temp[6], pdata->subdispinfo.elec_vol_temp[7],
			pdata->subdispinfo.elec_vol_temp[8], pdata->subdispinfo.elec_vol_temp[9], pdata->subdispinfo.elec_vol_temp[10], pdata->subdispinfo.elec_vol_temp[11],
			pdata->subdispinfo.elec_vol_temp[12], pdata->subdispinfo.elec_vol_temp[13], pdata->subdispinfo.elec_vol_temp[14], pdata->subdispinfo.elec_vol_temp[15],
			pdata->subdispinfo.elec_vol_temp[16], pdata->subdispinfo.elec_vol_temp[17]);
	pr_notice("elec_vol_user=%02X %02X %02X %02X %02X\n",
			pdata->subdispinfo.elec_vol_user[0], pdata->subdispinfo.elec_vol_user[1], pdata->subdispinfo.elec_vol_user[2], pdata->subdispinfo.elec_vol_user[3],
			pdata->subdispinfo.elec_vol_user[4]);
	pr_notice("langage=%d\n", pdata->subdispinfo.langage);
	pr_notice("watch_dispinfo=%d\n", pdata->subdispinfo.watch_dispinfo);
	pr_notice("clock_dispinfo=%d\n", pdata->subdispinfo.clock_dispinfo);
	pr_notice("timezone=%d\n", pdata->subdispinfo.timezone);
	return 0;
}

int disp_ext_sub_set_data(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	struct disp_ext_spi_data *sdata;

	pr_debug("%s start\n", __func__);

	sdata = &pdata->spi_data;
	disp_ext_spi_ctrl_cs(sdata, 0);
	disp_ext_spi_ctrl_rs(sdata, 1);

	ret = disp_ext_sub_set_cmd(payload_p, payload_len, pdata);

	disp_ext_spi_ctrl_rs(sdata, 0);
	disp_ext_spi_ctrl_cs(sdata, 1);

	pr_debug("%s end\n", __func__);
	return ret;
}

int disp_ext_sub_get_battery_temp(void)
{
	struct power_supply *battery_psy = NULL;
	union power_supply_propval ret = {0,};

	battery_psy = power_supply_get_by_name("battery");

	if (battery_psy)
		battery_psy->get_property(battery_psy, POWER_SUPPLY_PROP_TEMP, &ret);

	return ret.intval;
}

int disp_ext_sub_set_contrast(struct disp_ext_sub_pdata *pdata, bool force_set)
{
	return disp_ext_sub_set_user_contrast(pdata, pdata->subdispinfo.user_contrast, force_set);
}

int disp_ext_sub_set_user_contrast(struct disp_ext_sub_pdata *pdata, int user_contrast, bool force_set)
{
	int ret = 0;
	struct fb_var_subdispinfo subdispinfo_target = pdata->subdispinfo;
	int i;
	int temp_revise = 0;
	int temp_section = 0;
	int user_revise = 0;
	int vcnt = 0;
	int battery_temp;
	uint8_t data[2];
	const __u8 AllF[18] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	pr_debug("%s start %d\n", __func__, user_contrast);

	if (user_contrast < FB_SUB_USERCONT_TYPE_1 ||
		user_contrast > FB_SUB_USERCONT_TYPE_5) {
		user_contrast = subdispinfo_default.user_contrast;
	}

	if (subdispinfo_target.elec_vol_init == 0xFF || subdispinfo_target.elec_vol_init == 0x00) {
		subdispinfo_target.elec_vol_init = subdispinfo_default.elec_vol_init;
	}
	if (!memcmp(subdispinfo_target.elec_vol_temp, AllF, sizeof(subdispinfo_target.elec_vol_temp)) || subdispinfo_target.elec_vol_temp[0] == 0) {
		memcpy(subdispinfo_target.elec_vol_temp, subdispinfo_default.elec_vol_temp, sizeof(subdispinfo_default.elec_vol_temp));
	}
	if (!memcmp(subdispinfo_target.elec_vol_user, AllF, sizeof(subdispinfo_target.elec_vol_user)) || subdispinfo_target.elec_vol_user[0] == 0) {
		memcpy(subdispinfo_target.elec_vol_user, subdispinfo_default.elec_vol_user, sizeof(subdispinfo_target.elec_vol_user));
	}

	battery_temp = disp_ext_sub_get_battery_temp();

	for (i = 0; i < ARRAY_SIZE(subdispinfo_target.elec_vol_temp); i++) {
		if (disp_ext_sub_temp_thresh_tbl[i] > battery_temp) {
			if (i > 0) {
				temp_section = i-1;
			} else {
				temp_section = 0;
			}
			break;
		}
	}
	if (i >= ARRAY_SIZE(subdispinfo_target.elec_vol_temp)) {
		temp_section = ARRAY_SIZE(subdispinfo_target.elec_vol_temp) - 1;
	}

	temp_revise = (__s8)subdispinfo_target.elec_vol_temp[temp_section];
	user_revise = (__s8)subdispinfo_target.elec_vol_user[user_contrast];
	vcnt = (int)subdispinfo_target.elec_vol_init + temp_revise + user_revise;

	pr_debug("%s: vcnt=0x%X curr=0x%X temp,revise,section=%d,%d,%d user_revise=%d\n",
		__func__, vcnt, pdata->current_device_elec_vol, battery_temp, temp_revise, temp_section, user_revise);

	if (force_set || vcnt != pdata->current_device_elec_vol) {
		data[0] = 0x81;
		data[1] = (uint8_t)vcnt;
		ret = disp_ext_sub_set_cmd2(data, 2, pdata);
		if (!ret) {
			pdata->current_device_elec_vol = vcnt;
		}
	}

	pr_debug("%s end %d\n", __func__, ret);
	return ret;
}
