/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_mdm.h
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
#ifndef DISP_EXT_SUB_MDM_H
#define DISP_EXT_SUB_MDM_H

#include <linux/of_device.h>

enum {
	DISP_EXT_SUB_MDMREQ_DISPCTL = 0,
	DISP_EXT_SUB_MDMREQ_IMAGE_UPDATE,
	DISP_EXT_SUB_MDMREQ_DRVINFO,
	DISP_EXT_SUB_MDMREQ_SVCINFO,
	DISP_EXT_SUB_MDMREQ_SET_TIMERADJ,
	DISP_EXT_SUB_MDMREQ_SET_SSR,
	DISP_EXT_SUB_MDMREQ_DUMP_SVCIMAGE,
	DISP_EXT_SUB_MDMREQ_DUMP_DRVIMAGE,
	DISP_EXT_SUB_MDMREQ_DUMP_IMGPARTS,
	DISP_EXT_SUB_MDMREQ_DUMP_TIMEHIST,
	DISP_EXT_SUB_MDMREQ_SET_SVCDBGLVL,
	DISP_EXT_SUB_MDMREQ_SET_DRVDBGLVL,
	DISP_EXT_SUB_MDMREQ_MAX,
};

enum {
	DISP_EXT_SUB_MDMREQ_DEBUG_DISPCTL_RESET = 0,
	DISP_EXT_SUB_MDMREQ_DEBUG_MAX,
};

enum {
	DISP_EXT_SUB_MDMREQ_LOGLVL_TYPE_L = 0,
	DISP_EXT_SUB_MDMREQ_LOGLVL_TYPE_M,
	DISP_EXT_SUB_MDMREQ_LOGLVL_TYPE_H,
};

enum {
	DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_START = 0,
	DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_STOP,
	DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_MAX
};

#define DISP_EXT_SUB_MDMRES_TYPE_SUCCESS 0
#define DISP_EXT_SUB_MDMRES_TYPE_ERROR   1


/* MODEM control function */
int disp_ext_sub_modem_ioctl(struct fb_info *info,unsigned int cmd,unsigned long arg );
int disp_ext_sub_panel_set_status_alwayson(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata);
ssize_t disp_ext_sub_sysfs_mdm_stat(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t disp_ext_sub_sysfs_mdm_svcdump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_drvdump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_svcdbglvl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_drvdbglvl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_timehist(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_svcimage(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_drvimage(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t disp_ext_sub_sysfs_mdm_timeradj(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
int disp_ext_sub_mdm_init( struct disp_ext_sub_pdata *pdata );
int disp_ext_sub_smd_send_request( struct disp_ext_sub_pdata *pdata, disp_ext_sub_linux2mdm_tx_type *request);
int disp_ext_sub_smd_send_command(struct disp_ext_sub_pdata *pdata, int cmd, int param);
int disp_ext_sub_smd_send_raw_data(struct disp_ext_sub_pdata *pdata, uint8_t *buf, uint32_t len);

#endif /* DISP_EXT_SUB_MDM_H */
