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
#ifndef SENSOR_MICONM_EOSS3_H
#define SENSOR_MICONM_EOSS3_H

#include "sensor_micon_driver.h"

/* -----------------------------------------------------
  Defines(Constants)
 ----------------------------------------------------- */
#define INIT_ACC_NR_FILT_TH_L        3
#define INIT_ACC_NR_FILT_TH_H        25
#define INIT_ACC_NR_FILT_COFF        3
#define INIT_ACC_NR_REL_TH           50
#define INIT_ACC_NR_BASE_FILT_COFF   3
#define INIT_ACC_NR_BASE_FILT_ACOFF  9
#define INIT_ACC_NR_TOLERANCE        0
#define INIT_ACC_NR_BASE_FILT_HYS    1
#define INIT_ACC_NR_FILT_LMT_TH      25
#define INIT_GYR_NR_FILT_LMT_TH      30
#define INIT_GYR_NR_FILT_TH_L        2
#define INIT_GYR_NR_FILT_TH_H        25
#define INIT_GYR_NR_FILT_COFF        4
#define INIT_GYR_NR_REL_TH           50
#define INIT_GYR_NR_BASE_FILT_COFF   3
#define INIT_GYR_NR_BASE_FILT_ACOFF  7
#define INIT_GYR_NR_TOLERANCE        0
#define INIT_GYR_NR_BASE_FILT_HYS    1
#define INIT_TS_NR_FILT_TH_US        5000
#define INIT_TS_NR_FILT_TOLERANCE_US 100
#define INIT_TS_NR_REL_TH            400
#define INIT_TS_NR_BASE_FILT_COFF    6
#define INIT_TS_NR_BASE_FILT_ACOFF   8
#define INIT_TS_NR_FILT_COFF         16
#define INIT_TS_NR_NG_TH             7
#define INIT_TS_NR_UNACCEPT_NUM      4
#define INIT_NO_NR_FILT_TERM         400

#define VERIFY_DATA_HEAD_OFS    (0x1F)
#define VERIFY_DATA_TAIL_OFS    (0x8F)
#define VERIFY_HEAD_VAL         (0xA5)
#define VERIFY_TAIL_VAL         (0x5A)

enum sensor_micon_batch_tbl_num_e_type{
    SENSOR_BATCH_TBL_ACC = 0,
    SENSOR_BATCH_TBL_MAG,
    SENSOR_BATCH_TBL_GYRO,
    SENSOR_BATCH_TBL_PRESSURE,
    SENSOR_BATCH_TBL_ORTN,
    SENSOR_BATCH_TBL_GRV,
    SENSOR_BATCH_TBL_ACC_LNR,
    SENSOR_BATCH_TBL_ROT_VCTR,
    SENSOR_BATCH_TBL_MAG_ROT_VCTR,
    SENSOR_BATCH_TBL_GAME_ROT_VCTR,
    SENSOR_BATCH_TBL_MAG_UNCAL_RAW,
    SENSOR_BATCH_TBL_MAG_UNCAL_OFS,
    SENSOR_BATCH_TBL_GYRO_UNCAL_RAW,
    SENSOR_BATCH_TBL_GYRO_UNCAL_OFS,
    SENSOR_BATCH_TBL_STEPCOUNTER,
    SENSOR_BATCH_TBL_SENSORTASK_TSOF,
    SENSOR_BATCH_TBL_STEPDETECTOR,
    SENSOR_BATCH_TBL_APPTASK_TSOF,
    SENSOR_BATCH_TBL_MAX,
};

enum sensor_micon_butch_id_e_type{
    SENSOR_BATCH_ID_ACC = 0x00,
    SENSOR_BATCH_ID_MAG = 0x01,
    SENSOR_BATCH_ID_GYRO = 0x02,
    SENSOR_BATCH_ID_PRESSURE = 0x03,
    SENSOR_BATCH_ID_ORTN = 0x04,
    SENSOR_BATCH_ID_GRV = 0x05,
    SENSOR_BATCH_ID_ACC_LNR = 0x06,
    SENSOR_BATCH_ID_ROT_VCTR = 0x07,
    SENSOR_BATCH_ID_MAG_ROT_VCTR = 0x08,
    SENSOR_BATCH_ID_GAME_ROT_VCTR = 0x09,
    SENSOR_BATCH_ID_MAG_UNCAL_RAW = 0x0A,
    SENSOR_BATCH_ID_MAG_UNCAL_OFS = 0x0B,
    SENSOR_BATCH_ID_GYRO_UNCAL_RAW = 0x0C,
    SENSOR_BATCH_ID_GYRO_UNCAL_OFS = 0x0D,
    SENSOR_BATCH_ID_SENSORTASK_TSOF = 0x3F,
    SENSOR_BATCH_ID_STEPCOUNTER = 0x40,
    SENSOR_BATCH_ID_STEPDETECTOR = 0x41,
    SENSOR_BATCH_ID_APPTASK_TSOF = 0x7F,
    SENSOR_BATCH_ID_MAX,
};

enum sensor_micon_butch_size_e_type{
    SENSOR_BATCH_SIZE_ACC = 6,
    SENSOR_BATCH_SIZE_MAG = 7,
    SENSOR_BATCH_SIZE_GYRO = 6,
    SENSOR_BATCH_SIZE_PRESSURE = 4,
    SENSOR_BATCH_SIZE_ORTN = 6,
    SENSOR_BATCH_SIZE_GRV = 6,
    SENSOR_BATCH_SIZE_ACC_LNR = 6,
    SENSOR_BATCH_SIZE_ROT_VCTR = 6,
    SENSOR_BATCH_SIZE_MAG_ROT_VCTR = 7,
    SENSOR_BATCH_SIZE_GAME_ROT_VCTR = 6,
    SENSOR_BATCH_SIZE_MAG_UNCAL_RAW = 7,
    SENSOR_BATCH_SIZE_MAG_UNCAL_OFS = 6,
    SENSOR_BATCH_SIZE_GYRO_UNCAL_RAW = 6,
    SENSOR_BATCH_SIZE_GYRO_UNCAL_OFS = 6,
    SENSOR_BATCH_SIZE_SENSORTASK_TSOF = 0,
    SENSOR_BATCH_SIZE_STEPCOUNTER = 4,
    SENSOR_BATCH_SIZE_STEPDETECTOR = 1,
    SENSOR_BATCH_SIZE_APPTASK_TSOF = 0,
};

enum sensor_micon_butch_nouse_size_e_type{
    SENSOR_BATCH_NOUSE_SIZE_ACC = 1,
    SENSOR_BATCH_NOUSE_SIZE_MAG = 0,
    SENSOR_BATCH_NOUSE_SIZE_GYRO = 1,
    SENSOR_BATCH_NOUSE_SIZE_PRESSURE = 3,
    SENSOR_BATCH_NOUSE_SIZE_ORTN = 1,
    SENSOR_BATCH_NOUSE_SIZE_GRV = 1,
    SENSOR_BATCH_NOUSE_SIZE_ACC_LNR = 1,
    SENSOR_BATCH_NOUSE_SIZE_ROT_VCTR = 1,
    SENSOR_BATCH_NOUSE_SIZE_MAG_ROT_VCTR = 0,
    SENSOR_BATCH_NOUSE_SIZE_GAME_ROT_VCTR = 1,
    SENSOR_BATCH_NOUSE_SIZE_MAG_UNCAL_RAW = 0,
    SENSOR_BATCH_NOUSE_SIZE_MAG_UNCAL_OFS = 1,
    SENSOR_BATCH_NOUSE_SIZE_GYRO_UNCAL_RAW = 1,
    SENSOR_BATCH_NOUSE_SIZE_GYRO_UNCAL_OFS = 1,
    SENSOR_BATCH_NOUSE_SIZE_SENSORTASK_TSOF = 11,
    SENSOR_BATCH_NOUSE_SIZE_STEPCOUNTER = 3,
    SENSOR_BATCH_NOUSE_SIZE_STEPDETECTOR = 6,
    SENSOR_BATCH_NOUSE_SIZE_APPTASK_TSOF = 11,
};

enum {
    LOGGING_TRIGGER_NONE = 0,
    LOGGING_TRIGGER_FLUSH,
    LOGGING_TRIGGER_BATCH,
    LOGGING_TRIGGER_M4_BUFFER_FULL,
    LOGGING_TRIGGER_REMAIN_MARGIN,
    LOGGING_TRIGGER_MAX_LATENCY,
    LOGGING_TRIGGER_M4_BUF_BLANK_FULL,
    LOGGING_TRIGGER_FFE_BUFFER_FULL,
    LOGGING_TRIGGER_FFE_BUF_BLANK,
    LOGGING_TRIGGER_CHANGE_SENSOR_STATE,
    LOGGING_TRIGGER_MAX
};

enum batch_ts_type {
    SENSOR_TASK = 0,
    APP_TASK,
    BATCH_TS_TYPE_MAX
};

enum batch_markoff_timing_type {
    START = 0,
    MIDSTREAM,
    END
};

enum sensor_verify_result {
    VERIFY_NG = -1,
    VERIFY_OK = 0,
};

/* -----------------------------------------------------
  structs
 ----------------------------------------------------- */
struct SensorDataType {
	uint32_t accel_xy;
	uint32_t accel_z;

	uint32_t mag_xy;
	uint32_t mag_zA;

	uint32_t gyro_xy;
	uint32_t gyro_z;

	uint32_t press_Pa_L;
	uint32_t press_Pa_H;

	uint32_t orientation_pr;
	uint32_t orientation_y;

	uint32_t gravity_xy;
	uint32_t gravity_z;

	uint32_t linear_accel_xy;
	uint32_t linear_accel_z;

	uint32_t rotation_vec_q01;
	uint32_t rotation_vec_q2;

	uint32_t geomag_rotation_vec_q01;
	uint32_t geomag_rotation_vec_q2A;

	uint32_t game_rotation_vec_q01;
	uint32_t game_rotation_vec_q2;

	uint32_t mag_uncalib_xy;
	uint32_t mag_uncalib_zA;

	uint32_t mag_calib_offs_xy;
	uint32_t mag_calib_offs_z;

	uint32_t gyro_uncalib_xy;
	uint32_t gyro_uncalib_z;

	uint32_t gyro_calib_offs_xy;
	uint32_t gyro_calib_offs_z;

	uint32_t step_count_L;
	uint32_t step_count_H;

	uint32_t ambient_temp_L;
	uint32_t ambient_temp_H;

	uint32_t relative_humidity_L;
	uint32_t relative_humidity_H;

	uint32_t reserved1_L;
	uint32_t reserved1_H;

	uint32_t reserved2_L;
	uint32_t reserved2_H;

	uint32_t reserved3_L;
	uint32_t reserved3_H;
};

struct get_batchbuf_info {
    uint16_t remain_size;
    uint32_t s_ptr;
    uint32_t l_limit_addr;
    uint32_t u_limit_addr;
};

union timestamp {
    uint8_t data[4];
    uint32_t ts;
};

/* -----------------------------------------------------
  Variables
 ----------------------------------------------------- */

/* -----------------------------------------------------
  Function
 ----------------------------------------------------- */
void func_register_eoss3(struct sensor_micon_if_funcs *f);
#endif /* SENSOR_MICONM_EOSS3_H */
