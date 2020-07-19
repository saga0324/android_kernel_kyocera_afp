/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_panel.h
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
#ifndef DISP_EXT_SUB_PANEL_H
#define DISP_EXT_SUB_PANEL_H

#include <linux/mdss_io_util.h>

typedef enum {
	DISP_SUB_STATE_PWR_OFF,
	DISP_SUB_STATE_PWR_ON,
	DISP_SUB_STATE_OFF,
	DISP_SUB_STATE_ON,
	DISP_SUB_STATE_MAX
} disp_ext_sub_state_type;

typedef enum {
	DISP_SUB_CTRL_CMD =  0x01,
	DISP_SUB_CTRL_WAIT = 0x10,
	DISP_SUB_CTRL_SIG =  0x20,
	DISP_SUB_CTRL_MAX
} disp_ext_sub_ctrl_kind_type;

typedef enum {
	DISP_SUB_SIG_VH = 0,
	DISP_SUB_SIG_RESET,
	DISP_SUB_SIG_VOLED,
	DISP_SUB_SIG_MAX
} disp_ext_sub_sig_kind_type;

struct  disp_ext_sub_cmd_hdr {
	unsigned char ctrl_kind;
	unsigned char payload_len;
};

struct  disp_ext_sub_cmd_detail {
	struct  disp_ext_sub_cmd_hdr  cmd_hdr;
	void *  payload_p;
};

struct disp_ext_sub_cmds {
	char *  buf_p;
	unsigned int  blen;
	unsigned int  cmd_cnt;
	struct  disp_ext_sub_cmd_detail*  cmd_p;
};

struct disp_ext_sub_pdata {
	u32  xres;
	u32  yres;
	u32  bpp;
	int  rst_gpio;
	int  vh_gpio;
	int  voled_gpio;
	int  rs_gpio;
	struct disp_ext_sub_cmds  pwron_cmd;
	struct disp_ext_sub_cmds  on_cmd;
	struct disp_ext_sub_cmds  post_on_cmd;
	struct disp_ext_sub_cmds  off_cmd;
	struct disp_ext_sub_cmds  pwroff_cmd;
	struct disp_ext_sub_cmds  ram_wr_cmd;
	disp_ext_sub_state_type state;
	struct pinctrl            *pinctrl;
	struct dss_module_power power_data;
	bool first_update;
};

int disp_ext_sub_panel_set_status(
	disp_ext_sub_state_type next_status,
	struct disp_ext_sub_pdata *pdata );
int disp_ext_sub_panel_update(
	struct fb_var_screeninfo *var,
	struct fb_info *info,
	uint8_t* apps_img_p );
int disp_ext_sub_set_cmd(
	void * payload_p,
	unsigned char payload_len,
	struct disp_ext_sub_pdata *pdata );
int disp_ext_sub_panel_bus_init( struct disp_ext_sub_pdata *pdata );
int disp_ext_sub_panel_signal_init( struct disp_ext_sub_pdata *pdata );
int disp_ext_sub_get_panel_dt(
	struct device_node * np,
	struct disp_ext_sub_pdata *pdata
);
int disp_ext_sub_get_signal_dt(
	struct device_node * np,
	struct disp_ext_sub_pdata *pdata
);
int disp_ext_sub_get_seq_dt(
	struct device_node * np,
	struct disp_ext_sub_pdata *pdata
);
int disp_ext_sub_get_vreg_dt(
	struct device *dev,
	struct disp_ext_sub_pdata *pdata
);

#endif /* DISP_EXT_SUB_PANEL_H */
