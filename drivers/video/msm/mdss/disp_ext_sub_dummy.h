/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_dummy.h
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
#ifndef DISP_EXT_SUB_DUMMY_H
#define DISP_EXT_SUB_DUMMY_H

#include <linux/mdss_io_util.h>

typedef enum {
	DISP_SUB_STATE_PWR_OFF,
	DISP_SUB_STATE_PWR_ON,
	DISP_SUB_STATE_OFF,
	DISP_SUB_STATE_ON,
	DISP_SUB_STATE_MAX
} disp_ext_sub_state_type;

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

struct disp_ext_sub_info {
	struct fb_info *  fbi;
	struct disp_ext_sub_pdata pdata;
};

static int disp_ext_sub_probe(struct platform_device *pdev);
static int disp_ext_sub_remove(struct platform_device *pdev);
static int disp_ext_sub_open(struct fb_info *info, int user);
static int disp_ext_sub_release(struct fb_info *info, int user);
static int disp_ext_sub_mmap(struct fb_info *info, struct vm_area_struct *vma);
static void disp_ext_sub_shutdown(struct platform_device *pdev);

void disp_ext_sub_set_dmflag(int dmflag);


int disp_ext_sub_drvdata_init( struct platform_device *pdev );
int disp_ext_sub_register_framebuffer(
	struct platform_device *pdev,
	struct fb_ops *fbops );

#endif /* DISP_EXT_SUB_DUMMY_H */
