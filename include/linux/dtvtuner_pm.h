/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
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

#ifndef _DTVTUNER_PM_H_
#define _DTVTUNER_PM_H_

/* module name of tuner pm driver */
#define D_DTVTUNER_PM_DRIVER_NAME	"dtvtuner_pm"

/* dtvtuner : power on */
#define D_DTVTUNER_POWER_ON       1

/* dtvtuner : power off */
#define D_DTVTUNER_POWER_OFF      0

/* dtvtuner : HW reset */
#define D_DTVTUNER_RECOVERY       2

struct dtvtuner_pm_platform_data {
	int gpio_pwr;
	int gpio_isr;
};

#endif /* _DTVTUNER_PM_H_ */
