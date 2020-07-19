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
#ifndef SENSOR_MICON_ML630Q790_H
#define SENSOR_MICON_ML630Q790_H

#include "sensor_micon_driver.h"

/* -----------------------------------------------------
  Defines(Constants)
 ----------------------------------------------------- */
enum sensor_micon_batch_tbl_num_e_type{
    SENSOR_BATCH_TBL_ACC = 0,
    SENSOR_BATCH_TBL_MAG_UNCAL,
    SENSOR_BATCH_TBL_GYRO_UNCAL,
    SENSOR_BATCH_TBL_MAG,
    SENSOR_BATCH_TBL_GYRO,
    SENSOR_BATCH_TBL_PRESSURE,
    SENSOR_BATCH_TBL_ORTN,
    SENSOR_BATCH_TBL_GRV,
    SENSOR_BATCH_TBL_ACC_LNR,
    SENSOR_BATCH_TBL_ROT_VCTR,
    SENSOR_BATCH_TBL_GAME_ROT_VCTR,
    SENSOR_BATCH_TBL_MAG_ROT_VCTR,
    SENSOR_BATCH_TBL_ABS_STEP,
    SENSOR_BATCH_TBL_REL_STEP_2,
    SENSOR_BATCH_TBL_REL_STEP_1,
    SENSOR_BATCH_TBL_MAX,
};

enum sensor_micon_butch_id_e_type{
    SENSOR_BATCH_ID_ACC = 0x1,
    SENSOR_BATCH_ID_MAG = 0x2,
    SENSOR_BATCH_ID_GYRO = 0x3,
    SENSOR_BATCH_ID_MAG_UNCAL = 0x4,
    SENSOR_BATCH_ID_GYRO_UNCAL = 0x5,
    SENSOR_BATCH_ID_PRESSURE = 0x6,
    SENSOR_BATCH_ID_ORTN = 0x10,
    SENSOR_BATCH_ID_GRV = 0x11,
    SENSOR_BATCH_ID_ACC_LNR = 0x12,
    SENSOR_BATCH_ID_ROT_VCTR = 0x13,
    SENSOR_BATCH_ID_GAME_ROT_VCTR = 0x14,
    SENSOR_BATCH_ID_MAG_ROT_VCTR = 0x15,
    SENSOR_BATCH_ID_REL_STEP_1 = 0x20,
    SENSOR_BATCH_ID_REL_STEP_2 = 0x21,
    SENSOR_BATCH_ID_ABS_STEP = 0x22,
    SENSOR_BATCH_ID_MAX,
};

enum sensor_micon_butch_size_e_type{
    SENSOR_BATCH_SIZE_ACC = 7,
    SENSOR_BATCH_SIZE_MAG = 8,
    SENSOR_BATCH_SIZE_GYRO = 7,
    SENSOR_BATCH_SIZE_MAG_UNCAL = 6,
    SENSOR_BATCH_SIZE_GYRO_UNCAL = 6,
    SENSOR_BATCH_SIZE_ORTN = 8,
    SENSOR_BATCH_SIZE_GRV = 7,
    SENSOR_BATCH_SIZE_ACC_LNR = 7,
    SENSOR_BATCH_SIZE_ROT_VCTR = 9,
    SENSOR_BATCH_SIZE_MAG_ROT_VCTR = 10,
    SENSOR_BATCH_SIZE_GAME_ROT_VCTR = 9,
    SENSOR_BATCH_SIZE_REL_STEP_1 = 2,
    SENSOR_BATCH_SIZE_REL_STEP_2 = 1,
    SENSOR_BATCH_SIZE_ABS_STEP = 8,
    SENSOR_BATCH_SIZE_PRESSURE = 4,
};

enum {
    LOGGING_TRIGGER_NONE = 0,
    LOGGING_TRIGGER_FLUSH,
    LOGGING_TRIGGER_BATCH,
    LOGGING_TRIGGER_TIMEOUT,
    LOGGING_TRIGGER_BUF512,
    LOGGING_TRIGGER_BUFFULL,
    LOGGING_TRIGGER_MAX
};


/* -----------------------------------------------------
  structs
 ----------------------------------------------------- */

/* -----------------------------------------------------
  Variables
 ----------------------------------------------------- */

/* -----------------------------------------------------
  Function
 ----------------------------------------------------- */
int32_t ml630q790_get_fw_version(uint8_t *fw_ver);
void func_register_ml630q790(struct sensor_micon_if_funcs *f);
#endif /* SENSOR_MICON_ML630Q790_H */
