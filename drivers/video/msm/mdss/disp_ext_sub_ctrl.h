/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_ctrl.h
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
#ifndef DISP_EXT_SUB_CTRL_H
#define DISP_EXT_SUB_CTRL_H

#include <linux/of_device.h>
#include "disp_ext_sub_panel.h"
#include "disp_ext_sub_dbg.h"

struct disp_ext_sub_info {
	struct fb_info *  fbi;
	struct disp_ext_sub_pdata pdata;
};

void disp_ext_sub_set_dmflag(int dmflag);
int disp_ext_is_invalid(void);
int disp_ext_sub_blank_ctrl(int blank_mode, struct fb_info *info);
int disp_ext_sub_pan_display_ctrl (
	struct fb_var_screeninfo *var,
	struct fb_info *info );

int disp_ext_sub_drvdata_init( struct platform_device *pdev );
int disp_ext_sub_device_init( struct platform_device *pdev );
int disp_ext_sub_device_shutdown( struct platform_device *pdev );
int disp_ext_sub_get_dt( struct platform_device *pdev );
int disp_ext_sub_register_framebuffer(
	struct platform_device *pdev,
	struct fb_ops *fbops );

#endif /* DISP_EXT_SUB_CTRL_H */
