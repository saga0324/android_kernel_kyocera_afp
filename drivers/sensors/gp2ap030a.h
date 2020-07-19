/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
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
#ifndef __GP2AP030A_H__
#define __GP2AP030A_H__

#include <linux/ioctl.h>

#define D_NV_DATA_MAX                           (0x55)

typedef struct _t_psals_ioctl_ps_detection
{
    unsigned long ulps_detection;
}T_PSALS_IOCTL_PS_DETECTION;

typedef struct _t_psals_ioctl_als_mean_times
{
    unsigned long ulals_mean_times;
}T_PSALS_IOCTL_ALS_MEAN_TIMES;

typedef struct _t_psals_ioctl_als_lux_ave
{
    unsigned long ulals_lux_ave;
    long lcdata;
    long lirdata;
}T_PSALS_IOCTL_ALS_LUX_AVE;

typedef struct _t_psals_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_PSALS_IOCTL_NV;

typedef struct _t_psals_ioctl_ps_report_thresh
{
    uint8_t set;
}T_PSALS_IOCTL_PS_THRESHOLD;

#define PSALS_IO             'A'

#define IOCTL_PS_DETECTION_GET        _IOR(PSALS_IO, 0x01, T_PSALS_IOCTL_PS_DETECTION)
#define IOCTL_ALS_MEAN_TIMES_SET      _IOW(PSALS_IO, 0x02, T_PSALS_IOCTL_ALS_MEAN_TIMES)
#define IOCTL_ALS_LUX_AVE_GET         _IOR(PSALS_IO, 0x03, T_PSALS_IOCTL_ALS_LUX_AVE)
#define IOCTL_PSALS_NV_DATA_SET       _IOW(PSALS_IO, 0x04, T_PSALS_IOCTL_NV)
#define IOCTL_PSALS_NV_DATA_GET       _IOR(PSALS_IO, 0x05, T_PSALS_IOCTL_NV)
#define IOCTL_PS_THRESHOLD_SET        _IOW(PSALS_IO, 0x06, T_PSALS_IOCTL_PS_THRESHOLD)

enum {
    en_NV_PROXIMITY_SENSOR_NEAR_I = 0,
    en_NV_PROXIMITY_SENSOR_FAR_I,
    en_NV_PHOTO_SENSOR_TH_0_I,
    en_NV_PHOTO_SENSOR_TH_1_I,
    en_NV_PHOTO_SENSOR_TH_2_I,
    en_NV_PHOTO_SENSOR_TH_3_I,
    en_NV_PHOTO_SENSOR_TH_4_I,
    en_NV_PHOTO_SENSOR_A_0_I,
    en_NV_PHOTO_SENSOR_A_1_I,
    en_NV_PHOTO_SENSOR_A_2_I,
    en_NV_PHOTO_SENSOR_A_3_I,
    en_NV_PHOTO_SENSOR_A_4_I,
    en_NV_PHOTO_SENSOR_B_0_I,
    en_NV_PHOTO_SENSOR_B_1_I,
    en_NV_PHOTO_SENSOR_B_2_I,
    en_NV_PHOTO_SENSOR_B_3_I,
    en_NV_PHOTO_SENSOR_B_4_I,
    en_NV_PHOTO_SENSOR_1_R_I,
    en_NV_PHOTO_SENSOR_2_R_I,
#ifdef SUPPORT_COLORVARIATION
    en_NV_PROX_PHOTO_COLVAR_I,
#endif
    en_NV_PROX_PHOTO_MAX
};

#endif /* __GP2AP030A_H__ */
