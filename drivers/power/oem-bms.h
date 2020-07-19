#ifndef OEM_BMS_H
#define OEM_BMS_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**
 * oem_bms_correct_soc - readjustment oem soc
 *
 */

enum {
	DETERIORATION_STATUS_BETTER,
	DETERIORATION_STATUS_GOOD,
	DETERIORATION_STATUS_BAD
};

int oem_bms_correct_soc(int in_soc);

void oem_bms_init(int fcc);

void oem_bms_start_fcc_learning(int soc_final);

void oem_bms_stop_fcc_learning(void);

int oem_bms_get_deterioration_status(void);

int oem_bms_correct_calc_soc(int calc_soc, int last_soc);

void oem_bms_set_full_suppress(bool suppress);

bool oem_bms_get_full_suppress(void);

#endif
