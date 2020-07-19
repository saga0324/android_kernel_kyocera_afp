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



#include "sensor_micon_ml630q790.h"
#include <linux/lapis/ml630q790.h>


/* ------------------------------------------------
  Define
------------------------------------------------ */
#define MEASURE_DATA_SIZE           (18)
#define PRESSURE_DEFAULT_OFFSET     (30000u)
#define FASTEST_SENSTASK_PERIOD_MS  (0x000000BC)
#define BATCH_TIMEOUT_MIN_ML630Q790 (0x00000030)
#define BATCH_TIMEOUT_MAX_ML630Q790 (0x7FFFFFFF)
/* ------------------------------------------------
  Variables
------------------------------------------------ */
struct mutex micon_hcres_mutex;
static DEFINE_MUTEX(acc_auto_cal_mutex);
//static DECLARE_BITMAP(latest_batch_status, SENSOR_MAX);
int32_t g_nFWVersion;
static int64_t g_eventbuf_cleartime;
static uint8_t g_eventbuf_data[EVENTBUF_FIFO_SIZE];
static uint8_t g_logging_data[LOGGING_FIFO_SIZE+LOGGING_RESPONSE_HEADER];
static uint8_t g_acc_conv_axis = 0x00;
static uint8_t g_gyro_conv_axis = 0x00;
static uint8_t g_mag_conv_axis = 0x00;
static uint32_t g_mag_static_matrix[9] ={10000, 0, 0, 0, 10000, 0, 0, 0, 10000};
//static HostCmdRes diag_res;
static struct wake_lock g_pedo_wakelock;

static atomic_t s_snsdrv_status = ATOMIC_INIT((int)SENSOR_POWER_OFF);
static atomic_t g_nCalX = ATOMIC_INIT(0);
static atomic_t g_nCalY = ATOMIC_INIT(0);
static atomic_t g_nCalZ = ATOMIC_INIT(0);

/* module_param */
static uint timestamp_adjust_sensor = 0;
module_param(timestamp_adjust_sensor, uint, S_IRUGO );
MODULE_PARM_DESC(timestamp_adjust_sensor, "micon-timestamp-adjust-sensor");

static uint timestamp_adjust_app = 0;
module_param(timestamp_adjust_app, uint, S_IRUGO );
MODULE_PARM_DESC(timestamp_adjust_app, "micon-timestamp-adjust-app");

static uint timestamp_adjust_dailys = 0;
module_param(timestamp_adjust_dailys, uint, S_IRUGO );
MODULE_PARM_DESC(timestamp_adjust_dailys, "micon-timestamp-adjust-dailys");

const uint8_t batch_id_to_tbl_idx[] =
{
    [SENSOR_BATCH_ID_ACC]           = SENSOR_BATCH_TBL_ACC,
    [SENSOR_BATCH_ID_MAG]           = SENSOR_BATCH_TBL_MAG,
    [SENSOR_BATCH_ID_GYRO]          = SENSOR_BATCH_TBL_GYRO,
    [SENSOR_BATCH_ID_MAG_UNCAL]     = SENSOR_BATCH_TBL_MAG_UNCAL,
    [SENSOR_BATCH_ID_GYRO_UNCAL]    = SENSOR_BATCH_TBL_GYRO_UNCAL,
    [SENSOR_BATCH_ID_PRESSURE]      = SENSOR_BATCH_TBL_PRESSURE,
    [SENSOR_BATCH_ID_ORTN]          = SENSOR_BATCH_TBL_ORTN,
    [SENSOR_BATCH_ID_GRV]           = SENSOR_BATCH_TBL_GRV,
    [SENSOR_BATCH_ID_ACC_LNR]       = SENSOR_BATCH_TBL_ACC_LNR,
    [SENSOR_BATCH_ID_ROT_VCTR]      = SENSOR_BATCH_TBL_ROT_VCTR,
    [SENSOR_BATCH_ID_GAME_ROT_VCTR] = SENSOR_BATCH_TBL_GAME_ROT_VCTR,
    [SENSOR_BATCH_ID_MAG_ROT_VCTR]  = SENSOR_BATCH_TBL_MAG_ROT_VCTR,
    [SENSOR_BATCH_ID_REL_STEP_1]    = SENSOR_BATCH_TBL_REL_STEP_1,
    [SENSOR_BATCH_ID_REL_STEP_2]    = SENSOR_BATCH_TBL_REL_STEP_2,
    [SENSOR_BATCH_ID_ABS_STEP]      = SENSOR_BATCH_TBL_ABS_STEP,
};

const uint8_t tbl_idx_to_size[] =
{
    [SENSOR_BATCH_TBL_ACC]          = SENSOR_BATCH_SIZE_ACC,
    [SENSOR_BATCH_TBL_MAG_UNCAL]    = SENSOR_BATCH_SIZE_MAG_UNCAL,
    [SENSOR_BATCH_TBL_GYRO_UNCAL]   = SENSOR_BATCH_SIZE_GYRO_UNCAL,
    [SENSOR_BATCH_TBL_MAG]          = SENSOR_BATCH_SIZE_MAG,
    [SENSOR_BATCH_TBL_GYRO]         = SENSOR_BATCH_SIZE_GYRO,
    [SENSOR_BATCH_TBL_PRESSURE]     = SENSOR_BATCH_SIZE_PRESSURE,
    [SENSOR_BATCH_TBL_ORTN]         = SENSOR_BATCH_SIZE_ORTN,
    [SENSOR_BATCH_TBL_GRV]          = SENSOR_BATCH_SIZE_GRV,
    [SENSOR_BATCH_TBL_ACC_LNR]      = SENSOR_BATCH_SIZE_ACC_LNR,
    [SENSOR_BATCH_TBL_ROT_VCTR]     = SENSOR_BATCH_SIZE_ROT_VCTR,
    [SENSOR_BATCH_TBL_GAME_ROT_VCTR]= SENSOR_BATCH_SIZE_GAME_ROT_VCTR,
    [SENSOR_BATCH_TBL_MAG_ROT_VCTR] = SENSOR_BATCH_SIZE_MAG_ROT_VCTR,
    [SENSOR_BATCH_TBL_ABS_STEP]     = SENSOR_BATCH_SIZE_ABS_STEP,
    [SENSOR_BATCH_TBL_REL_STEP_1]   = SENSOR_BATCH_SIZE_REL_STEP_1,
    [SENSOR_BATCH_TBL_REL_STEP_2]   = SENSOR_BATCH_SIZE_REL_STEP_2,
};

const uint8_t tbl_idx_to_smid[] =
{
    [SENSOR_BATCH_TBL_ACC]           = SHID_ACCELEROMETER,
    [SENSOR_BATCH_TBL_MAG]           = SHID_GEOMAGNETIC,
    [SENSOR_BATCH_TBL_GYRO]          = SHID_GYROSCOPE,
    [SENSOR_BATCH_TBL_MAG_UNCAL]     = SHID_MAGNETIC_FIELD_UNCALIBRATED,
    [SENSOR_BATCH_TBL_GYRO_UNCAL]    = SHID_GYROSCOPE_UNCALIBRATED,
    [SENSOR_BATCH_TBL_PRESSURE]      = SHID_PRESSURE,
    [SENSOR_BATCH_TBL_ORTN]          = SHID_ORIENTATION,
    [SENSOR_BATCH_TBL_GRV]           = SHID_GRAVITY,
    [SENSOR_BATCH_TBL_ACC_LNR]       = SHID_LINEAR_ACCELERATION,
    [SENSOR_BATCH_TBL_ROT_VCTR]      = SHID_ROTATION_VECTOR,
    [SENSOR_BATCH_TBL_GAME_ROT_VCTR] = SHID_GAME_ROTATION_VECTOR,
    [SENSOR_BATCH_TBL_MAG_ROT_VCTR]  = SHID_GEOMAGNETIC_ROTATION_VECTOR,
    [SENSOR_BATCH_TBL_REL_STEP_1]    = SHID_STEP_COUNTER,
    [SENSOR_BATCH_TBL_REL_STEP_2]    = SHID_STEP_COUNTER,
    [SENSOR_BATCH_TBL_ABS_STEP]      = SHID_STEP_COUNTER,
};

const uint8_t sensor_type_to_tbl_idx[] =
{
    [SENSOR_ACC]            = SENSOR_BATCH_TBL_ACC,
    [SENSOR_MAG]            = SENSOR_BATCH_TBL_MAG,
    [SENSOR_MAG_UNCAL]      = SENSOR_BATCH_TBL_MAG,
    [SENSOR_MAG_ROT_VCTR]   = SENSOR_BATCH_TBL_MAG_ROT_VCTR,
    [SENSOR_ACC_LNR]        = SENSOR_BATCH_TBL_ACC_LNR,
    [SENSOR_GRV]            = SENSOR_BATCH_TBL_GRV,
    [SENSOR_GYRO]           = SENSOR_BATCH_TBL_GYRO,
    [SENSOR_GYRO_UNCAL]     = SENSOR_BATCH_TBL_GYRO,
    [SENSOR_ORTN]           = SENSOR_BATCH_TBL_ORTN,
    [SENSOR_ROT_VCTR]       = SENSOR_BATCH_TBL_ROT_VCTR,
    [SENSOR_GAME_ROT_VCTR]  = SENSOR_BATCH_TBL_GAME_ROT_VCTR,
    [SENSOR_PRESSURE]       = SENSOR_BATCH_TBL_PRESSURE,
    [SENSOR_STEP_CNT]       = SENSOR_BATCH_TBL_REL_STEP_1,
    [SENSOR_STEP_DTC]       = SENSOR_BATCH_TBL_REL_STEP_1,
};

const uint8_t sensor_type_to_smid[] =
{
    [SENSOR_ACC]            = SHID_ACCELEROMETER,
    [SENSOR_MAG]            = SHID_GEOMAGNETIC,
    [SENSOR_MAG_UNCAL]      = SHID_MAGNETIC_FIELD_UNCALIBRATED,
    [SENSOR_MAG_ROT_VCTR]   = SHID_GEOMAGNETIC_ROTATION_VECTOR,
    [SENSOR_ACC_LNR]        = SHID_LINEAR_ACCELERATION,
    [SENSOR_GRV]            = SHID_GRAVITY,
    [SENSOR_GYRO]           = SHID_GYROSCOPE,
    [SENSOR_GYRO_UNCAL]     = SHID_GYROSCOPE_UNCALIBRATED,
    [SENSOR_ORTN]           = SHID_ORIENTATION,
    [SENSOR_ROT_VCTR]       = SHID_ROTATION_VECTOR,
    [SENSOR_GAME_ROT_VCTR]  = SHID_GAME_ROTATION_VECTOR,
    [SENSOR_PRESSURE]       = SHID_PRESSURE,
    [SENSOR_STEP_CNT]       = SHID_STEP_COUNTER,
    [SENSOR_STEP_DTC]       = SHID_STEP_DETECTOR,
};

static int u2dh_position = CONFIG_INPUT_ML610Q793_ACCELEROMETER_POSITION;
static const int u2dh_position_map[][3][3] = {
    { { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* top/upper-left */
    { {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* top/upper-right */
    { { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* top/lower-right */
    { { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* top/lower-left */
    { { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* bottom/upper-left */
    { { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* bottom/upper-right */
    { { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* bottom/lower-right */
    { {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* bottom/lower-right */
};

/* ------------------------------------------------
  Prototype Functions
------------------------------------------------ */
static uint8_t get_tbl_idx_to_size(uint8_t idx);
static uint8_t get_tbl_idx_to_smid(uint8_t idx);
static uint8_t get_sensor_type_to_tbl_idx(enum sensor_e_type type);
static uint8_t get_sensor_type_to_smid(enum sensor_e_type type);
static int16_t get_fastest_sensortask_val(void);
static int32_t get_batch_fifo_size(void);
static int32_t get_fw_version(uint8_t *fw_ver);
static void get_sns_value(enum sensor_e_type type, void *outdata);
static uint8_t get_miconshtdwn_status(void);
static int32_t dev_write(uint32_t adr, const uint8_t *data, uint8_t size);
static int32_t dev_read(uint32_t adr, uint8_t *data, uint16_t size);
static void unit_change(struct sensor_ctrl_param_output_str *dst);

static int32_t acc_read_data(struct acceleration *arg_Acc);
static int32_t mag_read_data(struct geomagnetic *arg_Mag);
static int32_t gyro_read_data(struct gyroscope *arg_Gyro);
static int32_t maguncalib_read_data(struct mag_uncalib *arg_Mag);
static int32_t gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro);
static int32_t gravity_read_data(struct gravity *arg_Gravity);
static int32_t linacc_read_data(struct linear_acceleration *arg_linacc);
static int32_t ori_read_data(struct orientation *arg_ori);
static int32_t rota_read_data(struct rotation_vector *arg_rota);
static int32_t gamerota_read_data(struct game_rotation_vector *arg_gamerota);
static int32_t magrota_read_data(struct geomagnetic_rotation_vector *arg_magrota);
static int32_t pressure_read_data(struct pressure *arg_pressure);

#if defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE)
static int32_t sns_mag_gyro_onoff(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new);
#endif /* defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE) */
static int32_t send_sensor_ctrl(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new,
    bool force);
static int32_t send_logging_state(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new);
static void set_flush(enum sensor_e_type type);
static void enable_irq_wake_irq(bool enable);
static void sns_read_timestamp_offsets(uint8_t *logging_header, int64_t *base_offsets, uint32_t *step_count);
static bool is_batch_id_valid(uint8_t batch_id);
static int32_t iio_report_events(uint8_t *batch_data, uint32_t len,
                                    int64_t *base_timestamps,
                                    unsigned long *batch_status);
static int32_t batch_mark_off(int32_t trigger, bool report);
static bool sns_is_needed_gyro_sync(void);
static int iio_report_event_now(enum sensor_e_type type);
static int32_t check_exist_sensor(void);
static int32_t initialize_snsdrv( void );
static int32_t sns_acc_set_auto_cal_offset_internal(struct acceleration offsets);
static void irq_proc(uint64_t dummyIRQfactorVal);
static int32_t get_sns_info_from_micon_dt(struct spi_device *client);
static int32_t acc_set_offsets(int32_t* offsets);
static int32_t acc_get_offsets(int32_t* offsets);
static int32_t acc_set_ac_offsets(struct acceleration offsets);
static int32_t acc_get_ac_offsets(struct acceleration* offsets);
static int32_t mag_set_offsets(struct geomagnetic offsets);
static int32_t mag_get_offsets(struct geomagnetic* offsets);
static int32_t mag_set_static_matrix(int32_t* smat);
static int32_t mag_get_static_matrix(int32_t* smat);
static int32_t gyro_set_offsets(struct gyroscope offsets);
static int32_t gyro_get_offsets(struct gyroscope* offsets);
static int32_t press_set_offsets(struct pressure offsets);
static int32_t press_get_offsets(struct pressure* offsets);
static int32_t sgnfcnt_enable(bool enable);
static int32_t set_dev_param(void);
static int32_t reset_save_and_taskoff(void);
static int32_t sns_initcmd_exe(void);
static int32_t enable_i2c_peri(bool enable);
static void set_batch_notify(bool enable);

static int32_t micon_app_clear(int32_t clear_req);
static int32_t dailys_send_param(bool enable, const struct pedo_param pp);
static int32_t dailys_enable(bool enable, const struct pedo_param pp);
static int32_t dailys_send_latency_option(enum sensor_e_type type, bool enable);
static int32_t dailys_get_pedo_data(struct pedometer *arg_Pedo);
static int32_t dailys_get_pedo_eb_data(struct pedometer *arg_Pedo);
static int32_t dailys_get_vehi_data(struct vehicle *arg_Vehi);
static int32_t event_mark_off(void);
static void dailys_vib_interlocking(int32_t mode);
static int32_t baro_enable(bool enable);
static int32_t vhdet_enable(bool enable);
static int32_t vhdet_get_data(struct vhdetect *arg_VHdetect);
static int32_t kcmot_wstart_enable(bool enable);
static int32_t kcmot_wstop_enable(bool enable);
static int32_t kcmot_train_enable(bool enable);
static int32_t kcmot_train_get_data(struct kc_motion_train *arg_Train);
static int32_t kcmot_other_vehi_enable(bool enable);
static int32_t kcmot_bringup_enable(bool enable);
static int32_t kcmot_get_bringup_data(struct kc_motion_bringup_data *arg_Bringup);
static int32_t uwater_enable(bool enable);
static int32_t vtrigger_enable(bool enable);


/* ------------------------------------------------
  Each Process
------------------------------------------------ */
static uint8_t get_tbl_idx_to_size(uint8_t idx)
{
    return tbl_idx_to_size[idx];
}

static uint8_t get_tbl_idx_to_smid(uint8_t idx)
{
    return tbl_idx_to_smid[idx];
}

static uint8_t get_sensor_type_to_tbl_idx(enum sensor_e_type type)
{
    return sensor_type_to_tbl_idx[type];
}

static uint8_t get_sensor_type_to_smid(enum sensor_e_type type)
{
    return sensor_type_to_smid[type];
}

static int32_t get_batch_fifo_size(void)
{
    return LOGGING_FIFO_SIZE;
}

static int16_t get_fastest_sensortask_val(void)
{
    return FASTEST_SENSTASK_PERIOD_MS;
}

static int32_t get_fw_version(uint8_t *fw_ver)
{
    return ml630q790_get_fw_version(fw_ver);
}

static inline bool sns_is_fw_ver_gteq(uint32_t compare_version)
{
    uint8_t fw_ver[4];
    int32_t ret;
    bool    result = false;
    SENSOR_N_LOG("start");

    result = (g_nFWVersion >= compare_version);

    if (!result) {
        SENSOR_N_LOG("re-check fw version directly");
        ret = ml630q790_get_fw_version(fw_ver);
        if (ret == SNS_RC_OK) {
            result = (ML630Q790_GET_FW_VER(fw_ver) >= compare_version);
        }
    }

    SENSOR_N_LOG("end result[%d]", result);
    return result;
}

static void get_sns_value(enum sensor_e_type type, void *outdata)
{
    struct acceleration                   last_acc_data       = {0};
    struct geomagnetic                    last_mag_data       = {0};
    struct gyroscope                      last_gyro_data      = {0};
    struct mag_uncalib                    last_maguc_data     = {0};
    struct gyro_uncalib                   last_gyrouc_data    = {0};
    struct orientation                    last_ori_data       = {0};
    struct gravity                        last_grv_data       = {0};
    struct linear_acceleration            last_lacc_data      = {0};
    struct rotation_vector                last_rv_data        = {0};
    struct geomagnetic_rotation_vector    last_georv_data     = {0};
    struct game_rotation_vector           last_gamrv_data     = {0};
    struct pressure                       last_press_data     = {0};

    SENSOR_D_LOG("start");
    switch (type){
        case SENSOR_ACC:
            acc_read_data(&last_acc_data);
            memcpy(outdata, &last_acc_data, sizeof(last_acc_data));
            break;
        case SENSOR_MAG:
            mag_read_data(&last_mag_data);
            memcpy(outdata, &last_mag_data, sizeof(last_mag_data));
            break;
        case SENSOR_GYRO:
            gyro_read_data(&last_gyro_data);
            memcpy(outdata, &last_gyro_data, sizeof(last_gyro_data));
            break;
        case SENSOR_MAG_UNCAL:
            maguncalib_read_data(&last_maguc_data);
            memcpy(outdata, &last_maguc_data, sizeof(last_maguc_data));
            break;
        case SENSOR_GYRO_UNCAL:
            gyrouncalib_read_data(&last_gyrouc_data);
            memcpy(outdata, &last_gyrouc_data, sizeof(last_gyrouc_data));
            break;
        case SENSOR_GRV:
            gravity_read_data(&last_grv_data);
            memcpy(outdata, &last_grv_data, sizeof(last_grv_data));
            break;
        case SENSOR_ACC_LNR:
            linacc_read_data(&last_lacc_data);
            memcpy(outdata, &last_lacc_data, sizeof(last_lacc_data));
            break;
        case SENSOR_ORTN:
            ori_read_data(&last_ori_data);
            memcpy(outdata, &last_ori_data, sizeof(last_ori_data));
            break;
        case SENSOR_ROT_VCTR:
            rota_read_data(&last_rv_data);
            memcpy(outdata, &last_rv_data, sizeof(last_rv_data));
            break;
        case SENSOR_GAME_ROT_VCTR:
            gamerota_read_data(&last_gamrv_data);
            memcpy(outdata, &last_gamrv_data, sizeof(last_gamrv_data));
            break;
        case SENSOR_MAG_ROT_VCTR:
            magrota_read_data(&last_georv_data);
            memcpy(outdata, &last_georv_data, sizeof(last_georv_data));
            break;
        case SENSOR_PRESSURE:
            pressure_read_data(&last_press_data);
            memcpy(outdata, &last_press_data, sizeof(last_press_data));
            break;
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
}
static uint8_t get_miconshtdwn_status(void)
{
    return ml630q790_get_shutdown_status();
}
static void set_sensordrv_status(enum sensor_drv_status_e_type current_status)
{
    atomic_set(&s_snsdrv_status, (int)current_status);
}
static int32_t dev_write(uint32_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    ret = ml630q790_device_write((uint8_t)adr, (void*)data, size);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t dev_read(uint32_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    ret = ml630q790_device_read((uint8_t)adr, (void*)data, size);
    SENSOR_D_LOG("end");
    return ret;
}

static void unit_change(struct sensor_ctrl_param_output_str *dst)
{
    SENSOR_D_LOG("start");
    dst->sensor.period_acc *= 10;
    dst->sensor.period_mag *= 10;
    dst->sensor.period_gyro *= 10;
    dst->sensor.period_pressure *= 10;
    dst->sensor.period_fusion_task *= 100;
    dst->sensor.period_fusion *= 10;
    dst->logging.period_acc *= 10;
    dst->logging.period_mag *= 10;
    dst->logging.period_gyro *= 10;
    dst->logging.period_pressure *= 10;
    dst->logging.period_fusion_task *= 100;
    dst->logging.period_fusion *= 10;
    if(dst->sensor.period_app_task >= 240) {
        dst->sensor.period_app_task = 0x5DBA;
    } else {
        dst->sensor.period_app_task *= 100;
    }
    if(dst->sensor.period_sensor_task != FASTEST_SENSTASK_PERIOD_MS){
        if(dst->sensor.period_app_task >= 240){
            dst->sensor.period_sensor_task = 0x5DBA;
        } else {
        dst->sensor.period_sensor_task *= 100;
        }
    }
    if(dst->logging.period_sensor_task != FASTEST_SENSTASK_PERIOD_MS){
        if(dst->sensor.period_app_task >= 240){
            dst->logging.period_sensor_task = 0x5DBA;
        } else {
        dst->sensor.period_sensor_task *= 100;
        }
    }
    if(dst->logging.batch_timeout_set < BATCH_TIMEOUT_MIN_ML630Q790){
        dst->logging.batch_timeout_set = BATCH_TIMEOUT_MIN_ML630Q790;
    }
    else if(dst->logging.batch_timeout_set >= BATCH_TIMEOUT_MAX_ML630Q790){
        dst->logging.batch_timeout_set = BATCH_TIMEOUT_MAX_ML630Q790;
    }

    SENSOR_D_LOG("end");
}

static int32_t acc_read_data(struct acceleration *arg_Acc)
{
    uint8_t ucBuff[MEASURE_DATA_SIZE];
    int32_t ret = SNS_RC_OK;
    int32_t raw[3];
    int32_t pos[3];
    int32_t xyz[3];
    int32_t temp;
    int i,j;

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT00, ucBuff, sizeof(ucBuff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)ucBuff[0] | (((int32_t)ucBuff[1] & 0x3F) << 8);
        raw[0] = ACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)ucBuff[2] | (((int32_t)ucBuff[3] & 0x3F) << 8);
        raw[1] = ACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)ucBuff[4] | (((int32_t)ucBuff[5] & 0x3F) << 8);
        raw[2] = ACCDATA_SIGN_COMVERT_14_32BIT(temp);

        SENSOR_N_LOG("reg - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     ucBuff[1],ucBuff[0],ucBuff[3],ucBuff[2],ucBuff[5],ucBuff[4]);
        SENSOR_N_LOG("raw - raw0[%04x] raw1[%04x] raw2[%04x]",
                     raw[0], raw[1], raw[2]);
    }

    for (i = 0; i < 3; i++) {
        xyz[i] = 0;
        for (j = 0; j < 3; j++){
            xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
        }
        pos[i] = xyz[i];
        xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    }

    arg_Acc->nX   = pos[0];
    arg_Acc->nY   = pos[1];
    arg_Acc->nZ   = pos[2];
    arg_Acc->outX = xyz[0];
    arg_Acc->outY = xyz[1];
    arg_Acc->outZ = xyz[2];

    SENSOR_N_LOG("arg_Acc - x[%04x] y[%04x] z[%04x]",
                     arg_Acc->nX, arg_Acc->nY, arg_Acc->nZ);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t mag_read_data(struct geomagnetic *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    uint8_t buff2[1];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT0C, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);
    if(SNS_RC_OK == ret){
        ret = dev_read(RSLT2D, buff2, sizeof(buff2));
        SENSOR_N_LOG("dev_read[%d]",ret);
    }

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Mag->x = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Mag->y = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Mag->z = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        arg_Mag->accuracy = buff2[0];

        SENSOR_N_LOG("mag - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%02x]",
                        buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff2[0]);
        SENSOR_N_LOG("arg_Mag - x[%04x] y[%04x] z[%04x] a[%x]",
                        arg_Mag->x,arg_Mag->y,arg_Mag->z,arg_Mag->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t gyro_read_data(struct gyroscope *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT06, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Gyro->x =GYRODATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Gyro->y = GYRODATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Gyro->z = GYRODATA_SIGN_COMVERT_12_32BIT(temp);

        SENSOR_N_LOG("gyro - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                        buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gyro - x[%04x] y[%04x] z[%04x]",
                        arg_Gyro->x,arg_Gyro->y,arg_Gyro->z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t maguncalib_read_data(struct mag_uncalib *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    struct geomagnetic saved_ofs = {0};
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT0C, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);


    if(SNS_RC_OK == ret){
        sns_load_offset_value(SENSOR_MAG, &saved_ofs);
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Mag->x =MAGDATA_SIGN_COMVERT_12_32BIT(temp) + saved_ofs.x;

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Mag->y = MAGDATA_SIGN_COMVERT_12_32BIT(temp) + saved_ofs.y;

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Mag->z = MAGDATA_SIGN_COMVERT_12_32BIT(temp) + saved_ofs.z;

        arg_Mag->cal_x = saved_ofs.x;
        arg_Mag->cal_y = saved_ofs.y;
        arg_Mag->cal_z = saved_ofs.z;

        SENSOR_N_LOG("mag - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                        buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Mag - x[%04x] y[%04x] z[%04x] calx[%04x] caly[%04x] calz[%04x]",
                        arg_Mag->x,arg_Mag->y,arg_Mag->z,arg_Mag->cal_x,arg_Mag->cal_y,arg_Mag->cal_z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;
    struct gyroscope g_ofs = {0};

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT06, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        sensor_load_offset_value(SENSOR_GYRO, &g_ofs);
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Gyro->x =GYRODATA_SIGN_COMVERT_12_32BIT(temp) + g_ofs.x;

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Gyro->y = GYRODATA_SIGN_COMVERT_12_32BIT(temp) + g_ofs.y;

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Gyro->z = GYRODATA_SIGN_COMVERT_12_32BIT(temp) + g_ofs.z;

        arg_Gyro->cal_x = g_ofs.x;
        arg_Gyro->cal_y = g_ofs.y;
        arg_Gyro->cal_z = g_ofs.z;

        SENSOR_N_LOG("gyro - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                    buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gyro - x[%04x] y[%04x] z[%04x] calx[%04x] caly[%04x] calz[%04x]",
                    arg_Gyro->x,arg_Gyro->y,arg_Gyro->z,arg_Gyro->cal_x,arg_Gyro->cal_y,arg_Gyro->cal_z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t gravity_read_data(struct gravity *arg_Gravity)
{
    int32_t ret = SNS_RC_OK;
    int32_t temp;
    int32_t raw[3];
    int32_t xyz[3];
    int i,j;
    uint8_t buff[6];

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT1A, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        raw[0] = GRADATA_SIGN_COMVERT_14_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        raw[1] = GRADATA_SIGN_COMVERT_14_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        raw[2] = GRADATA_SIGN_COMVERT_14_32BIT(temp);

        SENSOR_N_LOG("gravity - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                    buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gravity raw - x[%04x] y[%04x] z[%04x]",
                    raw[0],raw[1],raw[2]);
    }

    for (i = 0; i < 3; i++) {
        xyz[i] = 0;
        for (j = 0; j < 3; j++){
            xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
        }
        xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    }

    arg_Gravity->x = xyz[0];
    arg_Gravity->y = xyz[1];
    arg_Gravity->z = xyz[2];

    SENSOR_N_LOG("arg_Gravity - x[%04x] y[%04x] z[%04x]",
                xyz[0],xyz[1],xyz[2]);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t linacc_read_data(struct linear_acceleration *arg_linacc)
{
    int32_t ret = SNS_RC_OK;
    int32_t raw[3];
    int32_t xyz[3];
    int32_t temp;
    int i,j;
    uint8_t buff[6];

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT20, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0x3F) << 8);
        raw[0] = LINACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0x3F) << 8);
        raw[1] = LINACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0x3F) << 8);
        raw[2] = LINACCDATA_SIGN_COMVERT_14_32BIT(temp);

        SENSOR_N_LOG("reg - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                    buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("raw - raw0[%04x] raw1[%04x] raw2[%04x]",
                    raw[0], raw[1], raw[2]);
    }

    for (i = 0; i < 3; i++) {
        xyz[i] = 0;
        for (j = 0; j < 3; j++){
            xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
        }
        xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    }

    arg_linacc->x = xyz[0];
    arg_linacc->y = xyz[1];
    arg_linacc->z = xyz[2];

    SENSOR_N_LOG("arg_linacc - x[%04x] y[%04x] z[%04x]",
                    arg_linacc->x, arg_linacc->y, arg_linacc->z);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t ori_read_data(struct orientation *arg_ori)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    uint8_t buff2[1];

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT14, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);
    if(SNS_RC_OK == ret){
        ret = dev_read(RSLT2D, buff2, sizeof(buff2));
        SENSOR_N_LOG("dev_read[%d]",ret);
    }

    if(SNS_RC_OK == ret){
        arg_ori->pitch = (int16_t)(buff[0] | (buff[1] << 8));
        arg_ori->roll =  (int16_t)(buff[2] | (buff[3] << 8));
        arg_ori->yaw =   (int16_t)(buff[4] | (buff[5] << 8));
        arg_ori->accuracy = buff2[0];

        SENSOR_N_LOG("orientation - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%02x]",
                    buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff2[0]);
        SENSOR_N_LOG("arg_ori - x[%04x] y[%04x] z[%04x] a[%x]",
                    arg_ori->pitch,arg_ori->roll,arg_ori->yaw,arg_ori->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t rota_read_data(struct rotation_vector *arg_rota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[7];

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT26, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_rota->x = (int16_t)(buff[0] | (buff[1] << 8));
        arg_rota->y = (int16_t)(buff[2] | (buff[3] << 8));
        arg_rota->z = (int16_t)(buff[4] | (buff[5] << 8));
        arg_rota->accuracy = buff[6];

        SENSOR_N_LOG("rotation_vector - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%x]",
                    buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff[6]);
        SENSOR_N_LOG("arg_rota - x[%04x] y[%04x] z[%04x] a[%x]",
                    arg_rota->x,arg_rota->y,arg_rota->z,arg_rota->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t gamerota_read_data(struct game_rotation_vector *arg_gamerota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[8];

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT36, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_gamerota->x = (int16_t)(buff[0] | (buff[1] << 8));
        arg_gamerota->y = (int16_t)(buff[2] | (buff[3] << 8));
        arg_gamerota->z = (int16_t)(buff[4] | (buff[5] << 8));
        arg_gamerota->s = (int16_t)(buff[6] | (buff[7] << 8));

        SENSOR_N_LOG("game_rotation_vector - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] s[%02x][%02x]",
                    buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff[7],buff[6]);
        SENSOR_N_LOG("arg_gamerota - x[%04x] y[%04x] z[%04x] s[%04x]",
                    arg_gamerota->x,arg_gamerota->y,arg_gamerota->z,arg_gamerota->s);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t magrota_read_data(struct geomagnetic_rotation_vector *arg_magrota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[8];

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT2E, buff, sizeof(buff));
    SENSOR_N_LOG("dev_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_magrota->x = (int16_t)(buff[0] | (buff[1] << 8));
        arg_magrota->y = (int16_t)(buff[2] | (buff[3] << 8));
        arg_magrota->z = (int16_t)(buff[4] | (buff[5] << 8));
        arg_magrota->s = (int16_t)(buff[6] | (buff[7] << 8));

        SENSOR_N_LOG("mag_rotation_vector - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] s[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff[7],buff[6]);
        arg_magrota->accuracy = 3;

        SENSOR_N_LOG("arg_magrota - x[%04x] y[%04x] z[%04x] s[%04x] a[%x]",
                     arg_magrota->x,arg_magrota->y,arg_magrota->z,arg_magrota->s,arg_magrota->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t pressure_read_data(struct pressure *arg_pressure)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[3];
    int32_t offset = PRESSURE_DEFAULT_OFFSET;

    SENSOR_N_LOG("start");

    ret = dev_read(RSLT2C, buff, 1);
    ret = dev_read(RSLT12, buff+1, 2);
    SENSOR_N_LOG("dev_read[%d]",ret);
    SENSOR_N_LOG("pressure - [0x%02x][0x%02x][0x%02x]",buff[2],buff[1],buff[0]);

    if(SNS_RC_OK == ret){
        arg_pressure->pressure = (int32_t)((buff[0] & 0x01) | (buff[1] << 1) | (buff[2] << 9));
        arg_pressure->pressure += offset;

        SENSOR_N_LOG("arg_pressure - pressure[%d] offset[%d]",
                    arg_pressure->pressure, offset);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

#if defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE)
static int32_t sns_mag_gyro_onoff(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    bool mag_uncal_enable;
    bool gyro_uncal_enable;
    bool fusion_off;
    struct geomagnetic* mag_ofs;
    struct gyroscope* gyro_ofs;
    DECLARE_BITMAP(fusion_sns_mask, SENSOR_MAX);
    DECLARE_BITMAP(tmp_status, SENSOR_MAX);

    SENSOR_N_LOG("start");

    mag_uncal_enable = IS_MAG_UNCAL_EN(new->sensor.enable);
    gyro_uncal_enable = IS_GYRO_UNCAL_EN(new->sensor.enable);
    bitmap_andnot(tmp_status, old->enable_sns, new->enable_sns, SENSOR_MAX);
    get_fusion_sns_mask(fusion_sns_mask, SENSOR_MAX);
    bitmap_and(tmp_status, tmp_status, fusion_sns_mask, SENSOR_MAX);
    fusion_off = !bitmap_empty(tmp_status, SENSOR_MAX);

    if (mag_uncal_enable) {
        cmd.cmd.udata16 = HC_MAG_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        cmd.prm.ub_prm[2] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            mag_ofs->accuracy = res.res.ub_res[0];
            mag_ofs->x = (int32_t)(res.res.ub_res[1] | (res.res.ub_res[2] << 8) |
                                    (res.res.ub_res[3] << 16) | (res.res.ub_res[4] << 24)) / 100;
            mag_ofs->y = (int32_t)(res.res.ub_res[5] | (res.res.ub_res[6] << 8) |
                                    (res.res.ub_res[7] << 16) | (res.res.ub_res[8] << 24)) / 100;
            mag_ofs->z = (int32_t)(res.res.ub_res[9] | (res.res.ub_res[10] << 8) |
                                    (res.res.ub_res[11] << 16) | (res.res.ub_res[12] << 24)) / 100;

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d] a[%d]",mag_ofs->x,mag_ofs->y,mag_ofs->z,mag_ofs->accuracy);
            sns_save_offset_value(SENSOR_MAG, mag_ofs);
        }
    } else {
        cmd.cmd.udata16 = HC_MAG_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if (gyro_uncal_enable) {
        cmd.cmd.udata16 = HC_GYRO_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        cmd.prm.ub_prm[2] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            gyro_ofs->x = res.res.sw_res[0];
            gyro_ofs->y = res.res.sw_res[1];
            gyro_ofs->z = res.res.sw_res[2];
            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",gyro_ofs->x,gyro_ofs->y,gyro_ofs->z);
            sensor_save_offset_value(SENSOR_GYRO, gyro_ofs);
        }
    } else {
        cmd.cmd.udata16 = HC_GYRO_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if (fusion_off) {
        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            mag_ofs->accuracy = res.res.ub_res[0];
            mag_ofs->x = (int32_t)(res.res.ub_res[1] | (res.res.ub_res[2] << 8) |
                (res.res.ub_res[3] << 16) | (res.res.ub_res[4] << 24)) / 100;
            mag_ofs->y = (int32_t)(res.res.ub_res[5] | (res.res.ub_res[6] << 8) |
                (res.res.ub_res[7] << 16) | (res.res.ub_res[8] << 24)) / 100;
            mag_ofs->z = (int32_t)(res.res.ub_res[9] | (res.res.ub_res[10] << 8) |
                (res.res.ub_res[11] << 16) | (res.res.ub_res[12] << 24)) / 100;

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d] a[%d]",mag_ofs->x,mag_ofs->y,mag_ofs->z,mag_ofs->accuracy);
            sensor_save_offset_value(SENSOR_MAG, mag_ofs);
        }

        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            gyro_ofs->x = res.res.sw_res[0];
            gyro_ofs->y = res.res.sw_res[1];
            gyro_ofs->z = res.res.sw_res[2];

            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",x,gyro_ofs->y,gyro_ofs->z);
        }
    }

    SENSOR_N_LOG("end - SNS_RC_OK");
    return SNS_RC_OK;
}
#endif /* defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE) */
static int32_t send_sensor_ctrl(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new,
    bool force)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    SENSOR_N_LOG("start");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

#if defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE)
    if(g_mag_available || g_gyro_available) {
        ret = sns_mag_gyro_onoff(old, new);
        if(SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end sns_mag_gyro_onoff err ret[%d]", ret);
            return ret;
        }
    }
#endif /* defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE) */

    cmd.cmd.udata16 = HC_MUL_SET_SENSOR_PERIOD;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->sensor.period_sensor_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->sensor.period_sensor_task >> 8);
    cmd.prm.ub_prm[3]  = (uint8_t)(new->sensor.period_sensor_task >> 16);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->sensor.period_acc >> 0);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->sensor.period_acc >> 8);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->sensor.period_acc >> 16);
    if(g_pres_available) {
        cmd.prm.ub_prm[7]  = (uint8_t)(new->sensor.period_pressure >> 0);
        cmd.prm.ub_prm[8]  = (uint8_t)(new->sensor.period_pressure >> 8);
        cmd.prm.ub_prm[9]  = (uint8_t)(new->sensor.period_pressure >> 16);
    } else {
        cmd.prm.ub_prm[7]  = 0x00;
        cmd.prm.ub_prm[8]  = 0x00;
        cmd.prm.ub_prm[9]  = 0x00;
    }
    if(g_mag_available) {
        cmd.prm.ub_prm[10]  = (uint8_t)(new->sensor.period_mag >> 0);
        cmd.prm.ub_prm[11]  = (uint8_t)(new->sensor.period_mag >> 8);
        cmd.prm.ub_prm[12]  = (uint8_t)(new->sensor.period_mag >> 16);
    } else {
        cmd.prm.ub_prm[10]  = 0x00;
        cmd.prm.ub_prm[11]  = 0x00;
        cmd.prm.ub_prm[12]  = 0x00;
    }
    if(g_gyro_available) {
        cmd.prm.ub_prm[13] = (uint8_t)(new->sensor.period_gyro >> 0);
        cmd.prm.ub_prm[14] = (uint8_t)(new->sensor.period_gyro >> 8);
        cmd.prm.ub_prm[15] = (uint8_t)(new->sensor.period_gyro >> 16);
    } else {
        cmd.prm.ub_prm[13] = 0x00;
        cmd.prm.ub_prm[14] = 0x00;
        cmd.prm.ub_prm[15] = 0x00;
    }
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_SENSOR_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    cmd.cmd.udata16 = HC_MUL_SET_FUSION_PERIOD;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->sensor.period_fusion_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->sensor.period_fusion_task >> 8);
    cmd.prm.ub_prm[3]  = (uint8_t)(new->sensor.period_fusion_task >> 16);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[7]  = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[9]  = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[10]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[11] = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[12] = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[13] = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[14] = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[15] = (uint8_t)(new->sensor.period_fusion >> 8);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    cmd.cmd.udata16 = HC_MCU_SET_APP_TASK_PERIOD;
    cmd.prm.ub_prm[0]  = (uint8_t)(new->sensor.period_app_task >> 0);
    cmd.prm.ub_prm[1]  = (uint8_t)(new->sensor.period_app_task >> 8);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->sensor.period_app_task >> 16);
    cmd.prm.ub_prm[3]  = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_APP_TASK_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#ifdef CONFIG_USE_SENSOR_LSM6DS3
    cmd.cmd.udata16 = HC_ACC_SET_PARAM_LSM6DS3;
    cmd.prm.ub_prm[0]  = new->sensor.acc_param[0];
    cmd.prm.ub_prm[1]  = new->sensor.acc_param[1];
    cmd.prm.ub_prm[2]  = new->sensor.acc_param[2];
    cmd.prm.ub_prm[3]  = new->sensor.acc_param[3];
    cmd.prm.ub_prm[4]  = new->sensor.acc_param[4];
    cmd.prm.ub_prm[5]  = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM_LSM6DS3 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#else
    cmd.cmd.udata16 = HC_ACC_SET_PARAM_LIS2DH;
    cmd.prm.ub_prm[0]  = new->sensor.acc_param[0];
    cmd.prm.ub_prm[1]  = new->sensor.acc_param[1];
    cmd.prm.ub_prm[2]  = new->sensor.acc_param[2];
    cmd.prm.ub_prm[3]  = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM_LIS2DH err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
    if(g_gyro_available) {
        cmd.cmd.udata16 = HC_GYRO_SET_PARAM;
        cmd.prm.ub_prm[0]  = new->sensor.gyro_param[0];
        cmd.prm.ub_prm[1]  = new->sensor.gyro_param[1];
        cmd.prm.ub_prm[2]  = new->sensor.gyro_param[2];
        cmd.prm.ub_prm[3]  = new->sensor.gyro_param[3];
        cmd.prm.ub_prm[4]  = new->sensor.gyro_param[4];
        cmd.prm.ub_prm[5]  = new->sensor.gyro_param[5];
        cmd.prm.ub_prm[6]  = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_PARAM err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    if(g_mag_available) {
#ifdef CONFIG_INPUT_SENSOR_YAS_MAG
        if (force ||
            (memcmp(old->sensor.mag_param, new->sensor.mag_param, sizeof(old->sensor.mag_param)) != 0)) {
            cmd.cmd.udata16 = HC_MAG_SET_PERIOD;
            cmd.prm.ub_prm[0]  = new->sensor.mag_param[0];
            cmd.prm.ub_prm[1]  = new->sensor.mag_param[1];
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_MAG_SET_PERIOD err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
            cmd.cmd.udata16 = HC_MAG_SET_FILTER;
            cmd.prm.ub_prm[0]  = new->sensor.mag_param[2];
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_MAG_SET_FILTER err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
        }
#elif defined(CONFIG_INPUT_SENSOR_HSCDTD008A_MAG)
        if (force ||
            (memcmp(old->sensor.mag_param, new->sensor.mag_param, sizeof(old->sensor.mag_param)) != 0)) {
            cmd.cmd.udata16 = HC_MAG_SET_PARAM_HSCDTD008A;
            cmd.prm.ub_prm[0]  = new->sensor.mag_param[0];
            cmd.prm.ub_prm[1]  = new->sensor.mag_param[1];
            cmd.prm.ub_prm[2]  = new->sensor.mag_param[2];
            cmd.prm.ub_prm[3]  = 0x00;
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_MAG_SET_PARAM_HSCDTD008A err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
        }
#endif
    }

    if(g_pres_available) {
        cmd.cmd.udata16 = HC_PRE_SET_PARAM;
        cmd.prm.ub_prm[0]  = new->sensor.pressure_param[0];
        cmd.prm.ub_prm[1]  = new->sensor.pressure_param[1];
        cmd.prm.ub_prm[2]  = new->sensor.pressure_param[2];
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_PRE_SET_PARAM err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    cmd.cmd.udata16 = HC_MUL_SET_ENABLE;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = (uint8_t)(new->sensor.enable >> 0);
    cmd.prm.ub_prm[2] = (uint8_t)(new->sensor.enable >> 8);
    cmd.prm.ub_prm[3] = (uint8_t)(new->sensor.enable >> 16);
    cmd.prm.ub_prm[4] = (uint8_t)(new->sensor.enable >> 24);
    cmd.prm.ub_prm[5]  = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0]  = new->sensor.task_exec[0];
    cmd.prm.ub_prm[1]  = new->sensor.task_exec[1];
    cmd.prm.ub_prm[2]  = new->sensor.task_exec[2];
    cmd.prm.ub_prm[3]  = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld on%d\n",__func__,enter_us,exit_us,func_time_us,cmd.prm.ub_prm[0]);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    SENSOR_N_LOG("end");
    return SNS_RC_OK;
}

static int32_t send_logging_state(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    bool batch_on = false;
    uint8_t gyro_calib_notify = 0x00;
    uint8_t mag_calib_notify = 0x00;
    bool step_counter_enabled = false;

#ifndef CONFIG_SENSOR_MICON_LOGGING
    return SNS_RC_OK;
#endif
    SENSOR_N_LOG("start");

    batch_on = (new->logging.batch_enable != 0);
    step_counter_enabled = new->logging.batch_step_cnt_enable && !old->logging.batch_step_cnt_enable;

    gyro_calib_notify = 0x00;
    mag_calib_notify = 0x00;
    if (!batch_on) {
        if (IS_MAG_UNCAL_EN(new->sensor.enable))
            mag_calib_notify = 0x01;
        if (IS_GYRO_UNCAL_EN(new->sensor.enable))
            gyro_calib_notify = 0x01;
    }

    cmd.cmd.udata16 = HC_GYRO_SET_CALIB;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = gyro_calib_notify;
    cmd.prm.ub_prm[2] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_CALIB err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_CALIB;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = mag_calib_notify;
    cmd.prm.ub_prm[2] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_CALIB err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_SENSOR_PERIOD;
    cmd.prm.ub_prm[0]  = 0x01;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->logging.period_sensor_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->logging.period_sensor_task >> 8);
    cmd.prm.ub_prm[3]  = (uint8_t)(new->logging.period_sensor_task >> 16);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->logging.period_acc >> 0);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->logging.period_acc >> 8);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->logging.period_acc >> 16);
    cmd.prm.ub_prm[7]  = (uint8_t)(new->logging.period_pressure >> 0);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->logging.period_pressure >> 8);
    cmd.prm.ub_prm[9]  = (uint8_t)(new->logging.period_pressure >> 16);
    cmd.prm.ub_prm[10]  = (uint8_t)(new->logging.period_mag >> 0);
    cmd.prm.ub_prm[11]  = (uint8_t)(new->logging.period_mag >> 8);
    cmd.prm.ub_prm[12]  = (uint8_t)(new->logging.period_mag >> 16);
    cmd.prm.ub_prm[13]  = (uint8_t)(new->logging.period_gyro >> 0);
    cmd.prm.ub_prm[14] = (uint8_t)(new->logging.period_gyro >> 8);
    cmd.prm.ub_prm[15] = (uint8_t)(new->logging.period_gyro >> 16);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_SENSOR_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_PERIOD;
    cmd.prm.ub_prm[0]  = 0x01;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->logging.period_fusion_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->logging.period_fusion_task >> 8);
    cmd.prm.ub_prm[3]  = (uint8_t)(new->logging.period_fusion_task >> 16);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[7]  = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[9]  = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[10]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[11] = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[12] = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[13] = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[14] = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[15] = (uint8_t)(new->logging.period_fusion >> 8);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_LOGGING_TIMEOUT;
    cmd.prm.ub_prm[0] = (uint8_t)(new->logging.batch_timeout_set & 0xFF);
    cmd.prm.ub_prm[1] = (uint8_t)((new->logging.batch_timeout_set >> 8) & 0xFF);
    cmd.prm.ub_prm[2] = (uint8_t)((new->logging.batch_timeout_set >> 16) & 0xFF);
    cmd.prm.ub_prm[3] = (uint8_t)((new->logging.batch_timeout_set >> 24) & 0xFF);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOGGING_TIMEOUT err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_LOGGING_NOTIFY;
    cmd.prm.ub_prm[0] = batch_on ? 0x01 : 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOGGING_NOTIFY err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_SET_ANDROID_ENABLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = new->logging.batch_step_cnt_enable;
    cmd.prm.ub_prm[2] = new->logging.batch_step_dtc_enable;
    cmd.prm.ub_prm[3] = 0x00;
    if (step_counter_enabled) {
        cmd.prm.ub_prm[4] = 0x01;
    } else {
        cmd.prm.ub_prm[4] = 0x00;
    }
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_ANDROID_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_ENABLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (uint8_t)(new->logging.enable >> 0);
    cmd.prm.ub_prm[2] = (uint8_t)(new->logging.enable >> 8);
    cmd.prm.ub_prm[3] = (uint8_t)(new->logging.enable >> 16);
    cmd.prm.ub_prm[4] = (uint8_t)(new->logging.enable >> 24);
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    sns_logging_setup_basetime(new->logging.enable_sns, old->logging.enable_sns);

    if (step_counter_enabled) {
        msleep(50);
    }

    if(SNS_RC_ERR_INVALID_ID == batch_mark_off(LOGGING_TRIGGER_BATCH, 1)){
        /* Execute logging punctuation steps again for recovery.
           In this case logging data should not be reported. */
        batch_mark_off(LOGGING_TRIGGER_BATCH, 0);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static void set_flush(enum sensor_e_type type)
{
    SENSOR_D_LOG("start type[%d]", type);
    ml630q790_sns_set_flush((int32_t)type);
    SENSOR_D_LOG("end");
}

static void enable_irq_wake_irq(bool enable)
{
	SENSOR_D_LOG("start");
	ml630q790_ope_wake_irq(enable);
	SENSOR_D_LOG("end");
}

static void sns_read_timestamp_offsets(uint8_t *logging_header, int64_t *base_offsets, uint32_t *step_count)
{
    uint32_t time_stamp_acc = 0;
    uint32_t time_stamp_mag = 0;
    uint32_t time_stamp_gyro = 0;
    uint32_t time_stamp_ori = 0;
    uint32_t time_stamp_gravity = 0;
    uint32_t time_stamp_linacc = 0;
    uint32_t time_stamp_rota = 0;
    uint32_t time_stamp_gamerota = 0;
    uint32_t time_stamp_magrota = 0;
    int32_t  time_stamp_step1 = 0;
    int32_t  time_stamp_step2 = 0;
    int32_t  time_stamp_stepcount = 0;
    uint32_t time_stamp_pressure = 0;
    SENSOR_N_LOG("start");

    time_stamp_acc = ((logging_header[5] << 24) | (logging_header[4] << 16) |
                      (logging_header[3] << 8) | logging_header[2]);
    SENSOR_N_LOG("time_stamp_acc[%x]",time_stamp_acc);
    base_offsets[SENSOR_BATCH_TBL_ACC] = time_stamp_acc;

    time_stamp_mag = ((logging_header[9] << 24) | (logging_header[8] << 16) |
                      (logging_header[7] << 8) | logging_header[6]);
    SENSOR_N_LOG("time_stamp_mag[%x]",time_stamp_mag);
    base_offsets[SENSOR_BATCH_TBL_MAG] = time_stamp_mag;
    base_offsets[SENSOR_BATCH_TBL_MAG_UNCAL] = time_stamp_mag;

    time_stamp_gyro = ((logging_header[13] << 24) | (logging_header[12] << 16) |
                       (logging_header[11] << 8) | logging_header[10]);
    SENSOR_N_LOG("time_stamp_gyro[%x]",time_stamp_gyro);
    base_offsets[SENSOR_BATCH_TBL_GYRO] = time_stamp_gyro;
    base_offsets[SENSOR_BATCH_TBL_GYRO_UNCAL] = time_stamp_gyro;

    time_stamp_ori = ((logging_header[17] << 24) | (logging_header[16] << 16) |
                      (logging_header[15] << 8) | logging_header[14]);
    SENSOR_N_LOG("time_stamp_ori[%x]",time_stamp_ori);
    base_offsets[SENSOR_BATCH_TBL_ORTN] = time_stamp_ori;

    time_stamp_gravity = ((logging_header[21] << 24) | (logging_header[20] << 16) |
                      (logging_header[19] << 8) | logging_header[18]);
    SENSOR_N_LOG("time_stamp_gravity[%x]",time_stamp_gravity);
    base_offsets[SENSOR_BATCH_TBL_GRV] = time_stamp_gravity;

    time_stamp_linacc = ((logging_header[25] << 24) | (logging_header[24] << 16) |
                      (logging_header[23] << 8) | logging_header[22]);
    SENSOR_N_LOG("time_stamp_linacc[%x]",time_stamp_linacc);
    base_offsets[SENSOR_BATCH_TBL_ACC_LNR] = time_stamp_linacc;

    time_stamp_rota = ((logging_header[29] << 24) | (logging_header[28] << 16) |
                      (logging_header[27] << 8) | logging_header[26]);
    SENSOR_N_LOG("time_stamp_rota[%x]",time_stamp_rota);
    base_offsets[SENSOR_BATCH_TBL_ROT_VCTR] = time_stamp_rota;

    time_stamp_gamerota = ((logging_header[33] << 24) | (logging_header[32] << 16) |
                      (logging_header[31] << 8) | logging_header[30]);
    SENSOR_N_LOG("time_stamp_gamerota[%x]",time_stamp_gamerota);
    base_offsets[SENSOR_BATCH_TBL_GAME_ROT_VCTR] = time_stamp_gamerota;

    time_stamp_magrota = ((logging_header[37] << 24) | (logging_header[36] << 16) |
                      (logging_header[35] << 8) | logging_header[34]);
    SENSOR_N_LOG("time_stamp_magrota[%x]",time_stamp_magrota);
    base_offsets[SENSOR_BATCH_TBL_MAG_ROT_VCTR] = time_stamp_magrota;

    time_stamp_step1 = ((logging_header[41] << 24) | (logging_header[40] << 16) |
                      (logging_header[39] << 8) | logging_header[38]);
    SENSOR_N_LOG("time_stamp_step1[%x]",time_stamp_step1);
    base_offsets[SENSOR_BATCH_TBL_REL_STEP_1] = time_stamp_step1;

    time_stamp_step2 = ((logging_header[45] << 24) | (logging_header[44] << 16) |
                      (logging_header[43] << 8) | logging_header[42]);
    SENSOR_N_LOG("time_stamp_step2[%x]",time_stamp_step2);
    base_offsets[SENSOR_BATCH_TBL_REL_STEP_2] = time_stamp_step2;

    time_stamp_stepcount = ((logging_header[49] << 24) | (logging_header[48] << 16) |
                      (logging_header[47] << 8) | logging_header[46]);
    SENSOR_N_LOG("time_stamp_stepcount[%x]",time_stamp_stepcount);
    base_offsets[SENSOR_BATCH_TBL_ABS_STEP] = time_stamp_stepcount;

    *step_count = ((logging_header[53] << 24) | (logging_header[52] << 16) |
                      (logging_header[51] << 8) | logging_header[50]);
    SENSOR_N_LOG("stepcount_data[%x]",*step_count);

    time_stamp_pressure = ((logging_header[57] << 24) | (logging_header[56] << 16) |
                      (logging_header[55] << 8) | logging_header[54]);
    SENSOR_N_LOG("time_stamp_pressure[%x]",time_stamp_pressure);
    base_offsets[SENSOR_BATCH_TBL_PRESSURE] = time_stamp_pressure;
    SENSOR_N_LOG("end");
}

static bool is_batch_id_valid(uint8_t batch_id)
{
    bool ret = false;
    SENSOR_N_LOG("start id[%d]", batch_id);

    switch (batch_id) {
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    case SENSOR_BATCH_ID_ACC:
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    case SENSOR_BATCH_ID_MAG:
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    case SENSOR_BATCH_ID_GYRO:
#endif
#ifdef CONFIG_INPUT_SENSOR_UNCAL_MAGNETOMETER
    case SENSOR_BATCH_ID_MAG_UNCAL:
#endif
#ifdef CONFIG_INPUT_SENSOR_UNCAL_GYROSCOPE
    case SENSOR_BATCH_ID_GYRO_UNCAL:
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    case SENSOR_BATCH_ID_PRESSURE:
#endif
#ifdef CONFIG_INPUT_SENSOR_ORIENTATION
    case SENSOR_BATCH_ID_ORTN:
#endif
#ifdef CONFIG_INPUT_SENSOR_GRAVITY
    case SENSOR_BATCH_ID_GRV:
#endif
#ifdef CONFIG_INPUT_SENSOR_LINEAR_ACCELERATION
    case SENSOR_BATCH_ID_ACC_LNR:
#endif
#ifdef CONFIG_INPUT_SENSOR_ROTATION_VECTOR
    case SENSOR_BATCH_ID_ROT_VCTR:
#endif
#ifdef CONFIG_INPUT_SENSOR_GAME_ROTATION_VECTOR
    case SENSOR_BATCH_ID_GAME_ROT_VCTR:
#endif
#ifdef CONFIG_INPUT_SENSOR_MAG_ROTATION_VECTOR
    case SENSOR_BATCH_ID_MAG_ROT_VCTR:
#endif
#if defined(CONFIG_INPUT_SENSOR_STEP_COUNTER)||defined(CONFIG_INPUT_SENSOR_STEP_DETECTOR)
    case SENSOR_BATCH_ID_REL_STEP_1:
    case SENSOR_BATCH_ID_REL_STEP_2:
    case SENSOR_BATCH_ID_ABS_STEP:
#endif
        ret = true;
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end ret[%d]", ret);
    return ret;
}

static int32_t iio_report_events(uint8_t *batch_data, uint32_t len, int64_t *base_timestamps, unsigned long *batch_status)
{
    uint32_t stepcount_data = 0;
    int64_t  timestamp_offsets[SENSOR_BATCH_TBL_MAX];
    uint8_t  *buf = batch_data + LOGGING_RESPONSE_HEADER;
    int32_t  remain = len - LOGGING_RESPONSE_HEADER;
    struct   pressure press_cal_ofs = {0};
    //int32_t  press_cal_ofs = sns_pressure_get_cal_ofs();
    uint8_t  *data;
    uint8_t  batch_id;
    uint8_t  tbl_idx;
    int64_t  timestamp;
    int32_t  smid;
    int32_t  size;
    int64_t  *offset;
    uint8_t  tmp_event[EVENT_DATA_SIZE];
    int32_t  work;
    static uint16_t gyro_uncal_offset[3] = {0};
    static uint16_t mag_uncal_offset[3] = {0};
    static uint32_t step_count = 0;
    int32_t  event_cnt = 0;
    bool     first_step = true;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");
    // initialize timestamp
    sns_read_timestamp_offsets(batch_data, timestamp_offsets, &stepcount_data);
    step_count += stepcount_data;

    while (remain > 0)
    {
        batch_id = *buf;
        if (!is_batch_id_valid(batch_id)) {
            SENSOR_ERR_LOG("invalid batch_id [%d] remain[%d]", batch_id, remain);
            ret = SNS_RC_ERR_INVALID_ID;
            break;
        }
        data = buf+1;

        tbl_idx = batch_id_to_tbl_idx[batch_id];
        smid = tbl_idx_to_smid[tbl_idx];
        size = tbl_idx_to_size[tbl_idx];
        timestamp = base_timestamps[tbl_idx];
        offset = &timestamp_offsets[tbl_idx];
        SENSOR_N_LOG("ev[%d] idx[%d] smid[%d] size[%d] ts[%lld] ofs[%lld]", event_cnt, tbl_idx, smid, size, timestamp, *offset);
        switch (tbl_idx) {
        case SENSOR_BATCH_TBL_ACC:
            *offset += data[0];
            sns_iio_report_event(smid, &data[1], size-1, timestamp, *offset);
            break;
        case SENSOR_BATCH_TBL_PRESSURE:
            sns_load_offset_value(SENSOR_PRESSURE, &press_cal_ofs);
            *offset += data[0];
            work = ((int32_t)data[1] | (((int32_t)data[2]) << 8) | (((int32_t)data[3]) << 16))
                + press_cal_ofs.pressure + PRESSURE_DEFAULT_OFFSET;
            sns_iio_report_event(smid, (uint8_t*)&work, sizeof(work), timestamp, *offset);
            break;
        case SENSOR_BATCH_TBL_MAG:
            *offset += data[0];
            if(test_bit(SENSOR_MAG, batch_status)){
                sns_iio_report_event(SHID_GEOMAGNETIC, &data[1],
                    size-1, timestamp, *offset);
            }
            if(test_bit(SENSOR_MAG_UNCAL, batch_status)){
                memcpy(&tmp_event[0], &data[1], size-1);
                memcpy(&tmp_event[size-1], mag_uncal_offset, sizeof(mag_uncal_offset));
                sns_iio_report_event(SHID_MAGNETIC_FIELD_UNCALIBRATED, &tmp_event[0],
                    size-1+sizeof(mag_uncal_offset), timestamp, *offset);
            }
            break;
        case SENSOR_BATCH_TBL_GYRO:
            *offset += data[0];
            if(test_bit(SENSOR_GYRO, batch_status)){
                sns_iio_report_event(SHID_GYROSCOPE, &data[1],
                    size-1, timestamp, *offset);
            }
            if(test_bit(SENSOR_GYRO_UNCAL, batch_status)){
                memcpy(&tmp_event[0], &data[1], size-1);
                memcpy(&tmp_event[size-1], gyro_uncal_offset, sizeof(gyro_uncal_offset));
                sns_iio_report_event(SHID_GYROSCOPE_UNCALIBRATED, &tmp_event[0],
                    size-1+sizeof(gyro_uncal_offset), timestamp, *offset);
            }
            break;
        case SENSOR_BATCH_TBL_MAG_UNCAL:
            mag_uncal_offset[0] = *(uint16_t *)&data[0];
            mag_uncal_offset[1] = *(uint16_t *)&data[2];
            mag_uncal_offset[2] = *(uint16_t *)&data[4];
            break;
        case SENSOR_BATCH_TBL_GYRO_UNCAL:
            gyro_uncal_offset[0] = *(uint16_t *)&data[0];
            gyro_uncal_offset[1] = *(uint16_t *)&data[2];
            gyro_uncal_offset[2] = *(uint16_t *)&data[4];
            break;
        case SENSOR_BATCH_TBL_ORTN:
        case SENSOR_BATCH_TBL_GRV:
        case SENSOR_BATCH_TBL_ACC_LNR:
        case SENSOR_BATCH_TBL_ROT_VCTR:
        case SENSOR_BATCH_TBL_GAME_ROT_VCTR:
        case SENSOR_BATCH_TBL_MAG_ROT_VCTR:
            *offset += data[0];
            sns_iio_report_event(smid, &data[1], size-1, timestamp, *offset);
            break;
        case SENSOR_BATCH_TBL_REL_STEP_1:
            if (first_step) {
                first_step = false;
            }
            *offset += *(int16_t *)&data[0];
            step_count++;
            SENSOR_N_LOG("ev:%d REL_STEP_1 offset:%d total_offset:%lld step:%d", event_cnt, *(int16_t *)&data[0], *offset, step_count);
            sns_iio_report_event(SHID_STEP_COUNTER, (uint8_t *)&step_count, sizeof(step_count), timestamp, *offset);
            sns_iio_report_event(SHID_STEP_DETECTOR, (uint8_t *)&step_count, 0, timestamp, *offset);
            break;
        case SENSOR_BATCH_TBL_REL_STEP_2:
            if (first_step) {
                first_step = false;
                timestamp_offsets[SENSOR_BATCH_TBL_REL_STEP_1] = timestamp_offsets[SENSOR_BATCH_TBL_REL_STEP_2];
            }
            timestamp = base_timestamps[SENSOR_BATCH_TBL_REL_STEP_1];
            offset = &timestamp_offsets[SENSOR_BATCH_TBL_REL_STEP_1];
            *offset += *(int8_t *)&data[0];
            step_count++;
            SENSOR_N_LOG("ev:%d REL_STEP_2 offset:%d total_offset:%lld step:%d", event_cnt, *(int8_t *)&data[0], *offset, step_count);
            sns_iio_report_event(SHID_STEP_COUNTER, (uint8_t *)&step_count, sizeof(step_count), timestamp, *offset);
            sns_iio_report_event(SHID_STEP_DETECTOR, (uint8_t *)&step_count, 0, timestamp, *offset);
            break;
        case SENSOR_BATCH_TBL_ABS_STEP:
            if (first_step) {
                first_step = false;
            }
            timestamp = base_timestamps[SENSOR_BATCH_TBL_REL_STEP_1];
            offset = &timestamp_offsets[SENSOR_BATCH_TBL_REL_STEP_1];
            *offset = *(int32_t *)&data[0] + timestamp_offsets[SENSOR_BATCH_TBL_ABS_STEP];
            step_count = *(uint32_t *)&data[4];
            SENSOR_N_LOG("ev:%d ABS_STEP   offset:%d total_offset:%lld step:%d", event_cnt, *(int32_t *)&data[0], *offset, step_count);
            sns_iio_report_event(SHID_STEP_COUNTER, (uint8_t *)&step_count, sizeof(step_count), timestamp, *offset);
            break;
        default:
            SENSOR_ERR_LOG("invalid batch_id [%d] remain[%d]", batch_id, remain);
            remain = 0;
            break;
        }
        remain -= (size + 1);
        buf += (size + 1);
        event_cnt++;
    }
    SENSOR_N_LOG("end");
    return ret;
}

static int32_t batch_mark_off(int32_t trigger, bool report)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    int64_t  base_timestamps[SENSOR_BATCH_TBL_MAX];
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    static DECLARE_BITMAP(latest_batch_status, SENSOR_MAX);
    DECLARE_BITMAP(base_batch_status, SENSOR_MAX);
    uint16_t logging_data_size = 0;
    uint16_t read_data_size = 0;
    uint8_t  fifo_size_res[2];
    uint16_t fifo_data_size = 0;
    uint8_t clr_dailys_param;
    uint8_t *buf;
    uint8_t is_stepcnt_dtc_called = now_ctrl_param.output_param.logging.batch_step_cnt_enable | \
                                    now_ctrl_param.output_param.logging.batch_step_dtc_enable;
    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);

    sns_logging_proceed_time(base_timestamps);
    bitmap_copy(base_batch_status, latest_batch_status, SENSOR_MAX);
    sensor_get_batch_status(latest_batch_status);

    cmd.cmd.udata16 = HC_MUL_GET_LOG_PUNCTUATION;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MUL_GET_LOG_PUNCTUATION[%x]", res.err.udata16);
        mutex_unlock(&micon_hcres_mutex);
        return SNS_RC_ERR;
    }

    ret = dev_read(RSLT3E, fifo_size_res, 2);
    SENSOR_D_LOG("dev_read-ret[%d] res1[%x] res2[%x]",ret,fifo_size_res[0],fifo_size_res[1]);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("dev_read[%d]", ret);
        mutex_unlock(&micon_hcres_mutex);
        return ret;
    }
    fifo_data_size = ((fifo_size_res[1] << 8) | fifo_size_res[0]);
    SENSOR_D_LOG("dev_read-fifo_data_size[%x]",fifo_data_size);

    clr_dailys_param = 0x00;
    if (is_stepcnt_dtc_called)
        clr_dailys_param |= 0x40;
    if (LOGGING_TRIGGER_TIMEOUT != trigger)
        clr_dailys_param |= 0x80;

    if (clr_dailys_param) {
        cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        cmd.prm.ub_prm[5] = 0x00;
        cmd.prm.ub_prm[6] = 0x00;
        cmd.prm.ub_prm[7] = 0x00;
        cmd.prm.ub_prm[8] = 0x00;
        cmd.prm.ub_prm[9] = clr_dailys_param;
        cmd.prm.ub_prm[10] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_CLR_DAILYS err[%x]",res.err.udata16);
            mutex_unlock(&micon_hcres_mutex);
            return SNS_RC_ERR;
        }
    }

    if(LOGGING_RESPONSE_HEADER > fifo_data_size || fifo_data_size > sizeof(res.res.ub_res)) {
        SENSOR_ERR_LOG("fifo_data_size[%d]", fifo_data_size);
        mutex_unlock(&micon_hcres_mutex);
        return SNS_RC_ERR;
    }

    ret |= dev_read(FIFO, res.res.ub_res, fifo_data_size);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("dev_read[%d]", ret);
        mutex_unlock(&micon_hcres_mutex);
        return ret;
    }
    logging_data_size = ((res.res.ub_res[1] << 8) | res.res.ub_res[0]);
    SENSOR_D_LOG("logging_data_size[%x]",logging_data_size);

    if(logging_data_size == 0) {
        mutex_unlock(&micon_hcres_mutex);
        SENSOR_N_LOG("end - return[%d]",ret);
        return ret;
    }
    memcpy(g_logging_data, res.res.ub_res, fifo_data_size);

    buf = g_logging_data + LOGGING_RESPONSE_HEADER;
    SENSOR_D_LOG("data[%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x...]",
        buf[0], buf[1], buf[2], buf[3],
        buf[4], buf[5], buf[6], buf[7],
        buf[8], buf[9], buf[10], buf[11],
        buf[12], buf[13], buf[14], buf[15],
        buf[16], buf[17], buf[18], buf[19]);

    read_data_size = fifo_data_size;
    logging_data_size -= (fifo_data_size - LOGGING_RESPONSE_HEADER);
    SENSOR_D_LOG("read_data_size[%d]-logging_data_size[%d]",read_data_size,logging_data_size);

    if(logging_data_size > sizeof(g_logging_data) - read_data_size) {
        mutex_unlock(&micon_hcres_mutex);
        SENSOR_ERR_LOG("invalid buffer size. logging_data_size[%d] fifo_data_size[%d]",
            logging_data_size, fifo_data_size);
        return SNS_RC_ERR;
    }

    while(logging_data_size > 0) {
        if((0 < logging_data_size) && (logging_data_size <= 512)) {
            cmd.cmd.udata16 = HC_MUL_GET_LOGGING_DATA;
            cmd.prm.ub_prm[0] = 0x00;
            cmd.prm.ub_prm[1] = (uint8_t)logging_data_size;
            cmd.prm.ub_prm[2] = (uint8_t)(logging_data_size>>8);
            ret = sns_hostcmd(&cmd, &res, logging_data_size, EXE_HOST_ALL, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_MUL_GET_LOGGING_DATA[%x]", res.err.udata16);
                mutex_unlock(&micon_hcres_mutex);
                return SNS_RC_ERR;
            }

            if(g_logging_data != NULL){
                memcpy((g_logging_data + read_data_size), &res.res.ub_res, logging_data_size);
                read_data_size += logging_data_size;
                logging_data_size = 0;
            } else {
                SENSOR_ERR_LOG("g_logging_data:NULL!!!");
                break;
            }
        } else if(512 < logging_data_size) {
            cmd.cmd.udata16 = HC_MUL_GET_LOGGING_DATA;
            cmd.prm.ub_prm[0] = 0x00;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x02;
            ret = sns_hostcmd(&cmd, &res, 512, EXE_HOST_ALL, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_MUL_GET_LOGGING_DATA[%x]", res.err.udata16);
                mutex_unlock(&micon_hcres_mutex);
                return SNS_RC_ERR;
            }

            if(g_logging_data != NULL){
                memcpy((g_logging_data + read_data_size), &res.res.ub_res, 512);
                read_data_size += 512;
                logging_data_size -= 512;
            } else {
                SENSOR_ERR_LOG("g_logging_data:NULL!!!");
                break;
            }
        }
    }

    if(report){
        ret = iio_report_events(g_logging_data, read_data_size, base_timestamps, base_batch_status);
        if(SNS_RC_ERR_INVALID_ID == ret){
            SENSOR_ERR_LOG("Buffer all clear for recovery!!");
            /* If logging data is invalid and report is faild,
               clear all logging buffer for recovery.*/
            cmd.cmd.udata16 = HC_MUL_GET_LOGGING_DATA;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x00;
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_MUL_GET_LOGGING_DATA[%x] (Buffer All Clear Command)", res.err.udata16);
                mutex_unlock(&micon_hcres_mutex);
                return SNS_RC_ERR;
            }
            ret = SNS_RC_ERR_INVALID_ID;
        }
    }

    mutex_unlock(&micon_hcres_mutex);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static bool sns_is_needed_gyro_sync(void)
{
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    return now_ctrl_param.input_param.inputs[SENSOR_GYRO].sampling_period_ms ==
                now_ctrl_param.input_param.inputs[SENSOR_GYRO_UNCAL].sampling_period_ms &&
                test_bit(SENSOR_GYRO, now_ctrl_param.output_param.polling.enable_sns) &&
                test_bit(SENSOR_GYRO_UNCAL, now_ctrl_param.output_param.polling.enable_sns);
}

static int iio_report_event_now(enum sensor_e_type type)
{
    int64_t now = GET_CURRENT_TIMESTAMP_NS();
    uint8_t data[EVENT_DATA_SIZE] = {0};
    struct pressure press_cal_ofs = {0};
    struct geomagnetic mag_cal_ofs = {0};
    struct gyroscope gyro_cal_ofs = {0};
    int32_t work;

    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start type[%d]", type);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    SENSOR_ERR_LOG("[IT_TEST] start type[%d]", type);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    switch (type) {
    case SENSOR_ACC:
        ret = dev_read(RSLT00, data, 6);
        break;

    case SENSOR_MAG:
        ret = dev_read(RSLT0C, data, 6);
        SENSOR_N_LOG("dev_read[%d]",ret);
        if (SNS_RC_OK == ret) {
            ret = dev_read(RSLT2D, data+6, 1);
            SENSOR_N_LOG("dev_read[%d]",ret);
        }
        break;

    case SENSOR_MAG_UNCAL:
        ret = dev_read(RSLT0C, data, 6);
        if (SNS_RC_OK == ret) {
            ret = dev_read(RSLT2D, data+6, 1);
            sensor_load_offset_value(SENSOR_MAG, &mag_cal_ofs);
            if (SNS_RC_OK == ret) {
                data[7] = (uint8_t)(mag_cal_ofs.x & 0x000000ff);
                data[8] = (uint8_t)((mag_cal_ofs.x >> 8) & 0x000000ff);
                data[9] = (uint8_t)(mag_cal_ofs.y & 0x000000ff);
                data[10] = (uint8_t)((mag_cal_ofs.y >> 8) & 0x000000ff);
                data[11] = (uint8_t)(mag_cal_ofs.z & 0x000000ff);
                data[12] = (uint8_t)((mag_cal_ofs.z >> 8) & 0x000000ff);
            }
        }
        break;

    case SENSOR_GYRO:
        if (sns_is_needed_gyro_sync()) {
            ret = SNS_RC_ERR;
        }
        else {
            ret = dev_read(RSLT06, data, 6);
        }
        break;

    case SENSOR_GYRO_UNCAL:
        ret = dev_read(RSLT06, data, 6);
        if (SNS_RC_OK == ret) {
            if (sns_is_needed_gyro_sync()) {
                sns_iio_report_event(
                        sensor_type_to_smid[SENSOR_GYRO],
                        data,
                        EVENT_DATA_SIZE,
                        now,
                        0);
            }
            sensor_load_offset_value(SENSOR_GYRO, &gyro_cal_ofs);
            data[6] = (uint8_t)(gyro_cal_ofs.x & 0x000000ff);
            data[7] = (uint8_t)((gyro_cal_ofs.x >> 8) & 0x000000ff);
            data[8] = (uint8_t)(gyro_cal_ofs.y & 0x000000ff);
            data[9] = (uint8_t)((gyro_cal_ofs.y >> 8) & 0x000000ff);
            data[10] = (uint8_t)(gyro_cal_ofs.z & 0x000000ff);
            data[11] = (uint8_t)((gyro_cal_ofs.z >> 8) & 0x000000ff);
        }
        break;

    case SENSOR_PRESSURE:
        sns_load_offset_value(SENSOR_PRESSURE, &press_cal_ofs);
        ret = dev_read(RSLT2C, data, 1);
        if (SNS_RC_OK == ret) {
            ret = dev_read(RSLT12, data+1, 2);
            SENSOR_N_LOG("dev_read[%d]",ret);
            if (SNS_RC_OK == ret) {
                work = (int32_t)((data[0] & 0x01) | (data[1] << 1) | (data[2] << 9));
                work += press_cal_ofs.pressure + PRESSURE_DEFAULT_OFFSET;
                *(int32_t*)data = work;
            }
        }
        break;

    case SENSOR_ORTN:
        ret = dev_read(RSLT14, data, 6);
        SENSOR_N_LOG("dev_read[%d]",ret);
        if (SNS_RC_OK == ret) {
            ret = dev_read(RSLT2D, data+6, 1);
            SENSOR_N_LOG("dev_read[%d]",ret);
        }
        break;

    case SENSOR_GRV:
        ret = dev_read(RSLT1A, data, 6);
        break;

    case SENSOR_ACC_LNR:
        ret = dev_read(RSLT20, data, 6);
        break;

    case SENSOR_ROT_VCTR:
        ret = dev_read(RSLT26, data, 6);
        break;

    case SENSOR_GAME_ROT_VCTR:
        ret = dev_read(RSLT36, data, 8);
        break;

    case SENSOR_MAG_ROT_VCTR:
        ret = dev_read(RSLT2E, data, 8);
        break;

    default:
        ret = SNS_RC_ERR;
        SENSOR_ERR_LOG("invalid type:%d",type);
        break;
    }
    if (SNS_RC_OK == ret) {
        sns_iio_report_event(
                sensor_type_to_smid[type],
                data,
                EVENT_DATA_SIZE,
                now,
                0);
    }

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t check_exist_sensor(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);

    cmd.cmd.udata16 = HC_MCU_GET_EX_SENSOR;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("CMD Error <HC_MCU_GET_EX_SENSOR>[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    if( g_acc_available ){
        if( !(res.res.ub_res[0] & 0x01) ){
            SENSOR_ERR_LOG("Acc Sensor Not found[%x]",res.res.ub_res[0]);
            g_acc_available = 0;
            ret = SNS_RC_ERR;
        }
    }
    if( g_pres_available ){
        if( !(res.res.ub_res[0] & 0x20) ){
            SENSOR_ERR_LOG("Pressure Sensor Not found[%x]",res.res.ub_res[0]);
            g_pres_available = 0;
            ret = SNS_RC_ERR;
        }
    }
    if( g_mag_available ){
        if( !(res.res.ub_res[0] & 0x40) ){
            SENSOR_ERR_LOG("Mag Sensor Not found[%x]",res.res.ub_res[0]);
            g_mag_available = 0;
            ret = SNS_RC_ERR;
        }
    }
    if( g_gyro_available ){
        if( !(res.res.ub_res[0] & 0x80) ){
            SENSOR_ERR_LOG("Gyro Sensor Not found[%x]",res.res.ub_res[0]);
            g_gyro_available = 0;
            ret = SNS_RC_ERR;
        }
    }
    sns_invalid_setting(g_acc_available, g_mag_available, g_gyro_available, g_pres_available);

    SENSOR_D_LOG("end - ret %d", ret);

    return ret;
}

static void arg_Init(void)
{
    g_bDevIF_Error = false;
    g_micon_error = false;
    wake_lock_init(&g_pedo_wakelock, WAKE_LOCK_SUSPEND, SENSOR_MICON_DRIVER_NAME);
}

static int32_t initialize_snsdrv( void )
{
    uint8_t fw_ver[4];
    int32_t ret = SNS_RC_OK;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    SENSOR_N_LOG("start");

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    arg_Init();
    if(ml630q790_initialize_micondrv()){
        SENSOR_ERR_LOG("Failed ml630q790 initialize.");
        return SNS_RC_ERR;
    }

    ret = check_exist_sensor();
    if(ret != 0) {
        SENSOR_ERR_LOG("check_exist_sensor[%d]",ret);
    }

    ret = ml630q790_get_fw_version(fw_ver);
    g_nFWVersion = ML630Q790_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK ){
        g_nFWVersion = ML630Q790_FW_VER_NONE;
        SENSOR_ERR_LOG("ml630q790_get_fw_version[%d]",ret);
    }

    SENSOR_N_LOG("end - SNS_RC_OK");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld cnt:[%d]\n",__func__,enter_us,exit_us,func_time_us,cnt);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    return SNS_RC_OK;
}

static int32_t sns_acc_set_auto_cal_offset_internal(struct acceleration offsets)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_AUTO_CALIB_SET_DATA;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = 0x01;
    cmd.prm.ub_prm[3] = 0x01;
    cmd.prm.ub_prm[4] = (offsets.nX & 0x000000ff);
    cmd.prm.ub_prm[5] = ((offsets.nX >> 8) & 0x000000ff);
    cmd.prm.ub_prm[6] = (offsets.nY & 0x000000ff);
    cmd.prm.ub_prm[7] = ((offsets.nY >> 8) & 0x000000ff);
    cmd.prm.ub_prm[8] = (offsets.nZ & 0x000000ff);
    cmd.prm.ub_prm[9] = ((offsets.nZ >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_AUTO_CALIB_SET_DATA err[%x]",res.err.udata16);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t get_sns_info_from_micon_dt(struct spi_device *client)
{
    uint8_t tmp_acc_conv_axis = 0x00;
    uint8_t tmp_gyro_conv_axis = 0x00;
    uint8_t tmp_mag_conv_axis = 0x00;
    uint32_t tmp_mag_static_matrix[9] ={0};
    uint tmp_timestamp_adjust_sensor = 0;
    uint tmp_timestamp_adjust_app = 0;
    uint tmp_timestamp_adjust_dailys = 0;
    int ret = SNS_RC_OK;
    struct device_node *np = client->dev.of_node;
    int i;

    SENSOR_D_LOG("start");

    if(g_acc_available){
        ret = of_property_read_u8(np, "micon-acc-conv_axis", &tmp_acc_conv_axis);
        if(ret){
            SENSOR_ERR_LOG("micon-acc-conv_axis read err[%x]", ret);
            return ret;
        }
        g_acc_conv_axis = tmp_acc_conv_axis;
    }

    if(g_gyro_available){
        ret = of_property_read_u8(np, "micon-gyro-conv_axis", &tmp_gyro_conv_axis);
        if(ret){
            SENSOR_ERR_LOG("micon-gyro-conv_axis read err[%x]", ret);
            return ret;
        }
        g_gyro_conv_axis = tmp_gyro_conv_axis;
    }

    if(g_mag_available) {
        ret = of_property_read_u8(np, "micon-mag-conv_axis", &tmp_mag_conv_axis);
        if(ret){
            SENSOR_ERR_LOG("micon-mag-conv_axis read err[%x]", ret);
            return ret;
        }
        g_mag_conv_axis = tmp_mag_conv_axis;
    }

    if(g_mag_available) {
        ret = of_property_read_u32_array(np, "micon-mag-static_matrix", tmp_mag_static_matrix, 9);
        if(ret){
            SENSOR_ERR_LOG("micon-mag-static_matrix read err[%x]", ret);
            return ret;
        }
        for(i = 0; i < sizeof(g_mag_static_matrix) / sizeof(g_mag_static_matrix[0]); i++){
            g_mag_static_matrix[i] = tmp_mag_static_matrix[i];
        }
    }

    ret = of_property_read_u32(np, "micon-timestamp-adjust-sensor", &tmp_timestamp_adjust_sensor);
    if(ret){
        SENSOR_ERR_LOG("micon-timestamp-adjust-sensor read err[%x]", ret);
        return ret;
    }
    timestamp_adjust_sensor = tmp_timestamp_adjust_sensor; /* module_param */

    ret = of_property_read_u32(np, "micon-timestamp-adjust-app", &tmp_timestamp_adjust_app);
    if(ret){
        SENSOR_ERR_LOG("micon-timestamp-adjust-app read err[%x]", ret);
        return ret;
    }
    timestamp_adjust_app = tmp_timestamp_adjust_app; /* module_param */

    ret = of_property_read_u32(np, "micon-timestamp-adjust-dailys", &tmp_timestamp_adjust_dailys);
    if(ret){
        SENSOR_ERR_LOG("micon-timestamp-adjust-dailys read err[%x]", ret);
        return ret;
    }
    timestamp_adjust_dailys = tmp_timestamp_adjust_dailys; /* module_param */

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t acc_set_offsets(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    int32_t temp;

    SENSOR_N_LOG("start");

    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[0]);
    atomic_set(&g_nCalX, temp);
    SENSOR_D_LOG("set offset X[%d]",temp);

    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[1]);
    atomic_set(&g_nCalY, temp);
    SENSOR_D_LOG("set offset Y[%d]",temp);

    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[2]);
    atomic_set(&g_nCalZ, temp);
    SENSOR_D_LOG("set offset Z[%d]",temp);

    cmd.cmd.udata16 = HC_ACC_SET_CALIB;
    cmd.prm.ub_prm[0] = (uint8_t)(atomic_read(&g_nCalX) & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((atomic_read(&g_nCalX) >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(atomic_read(&g_nCalY) & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((atomic_read(&g_nCalY) >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(atomic_read(&g_nCalZ) & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((atomic_read(&g_nCalZ) >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_CALIB err[%x]",res.err.udata16);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}


static int32_t acc_get_offsets(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_ACC_GET_CALIB;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_GET_CALIB err[%x]",res.err.udata16);
    }

    atomic_set(&g_nCalX, res.res.sw_res[0]);
    atomic_set(&g_nCalY, res.res.sw_res[1]);
    atomic_set(&g_nCalZ, res.res.sw_res[2]);

    offsets[0] = atomic_read(&g_nCalX);
    offsets[1] = atomic_read(&g_nCalY);
    offsets[2] = atomic_read(&g_nCalZ);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t acc_set_ac_offsets(struct acceleration offsets)
{
    int32_t ret;

    SENSOR_N_LOG("start offset [X:%d Y:%d Z:%d]", offsets->nX, offsets->nY, offsets->nZ);
    ret = sns_acc_set_auto_cal_offset_internal(offsets);
    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t acc_get_ac_offsets(struct acceleration* offsets)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&acc_auto_cal_mutex);
    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_AUTO_CALIB_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_AUTO_CALIB_GET_DATA err[%x]",res.err.udata16);
    }else{
        offsets->nX = res.res.sw_res[0];
        offsets->nY = res.res.sw_res[1];
        offsets->nZ = res.res.sw_res[2];
    }
    mutex_unlock(&acc_auto_cal_mutex);

    SENSOR_N_LOG("end - return[%d] offset [X:%d Y:%d Z:%d]",ret, offsets->nX, offsets->nY, offsets->nZ);

    return ret;
}

static int32_t mag_set_offsets(struct geomagnetic offsets)
{
    int32_t ret = SNS_RC_OK;
    int32_t    temp;
    struct geomagnetic m_ofs = {0};
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_D_LOG("start");
    m_ofs.accuracy = offsets.accuracy;
    m_ofs.x = MAGDATA_SIGN_COMVERT_12_32BIT(offsets.x);
    m_ofs.y = MAGDATA_SIGN_COMVERT_12_32BIT(offsets.y);
    m_ofs.z = MAGDATA_SIGN_COMVERT_12_32BIT(offsets.z);
    SENSOR_N_LOG("set offset accuracy[%d] X[%d] Y[%d] Z[%d]",
                    m_ofs.accuracy, m_ofs.x, m_ofs.y, m_ofs.z);

    cmd.cmd.udata16 = HC_MAG_SET_DATA;
    cmd.prm.ub_prm[0] = m_ofs.accuracy;
    temp = m_ofs.x * 100;
    cmd.prm.ub_prm[1] = (uint8_t)(temp & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)((temp >> 8) & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((temp >> 16) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)((temp >> 24) & 0x000000ff);
    temp = m_ofs.y * 100;
    cmd.prm.ub_prm[5] = (uint8_t)(temp & 0x000000ff);
    cmd.prm.ub_prm[6] = (uint8_t)((temp >> 8) & 0x000000ff);
    cmd.prm.ub_prm[7] = (uint8_t)((temp >> 16) & 0x000000ff);
    cmd.prm.ub_prm[8] = (uint8_t)((temp >> 24) & 0x000000ff);
    temp = m_ofs.z * 100;
    cmd.prm.ub_prm[9] = (uint8_t)(temp & 0x000000ff);
    cmd.prm.ub_prm[10] = (uint8_t)((temp >> 8) & 0x000000ff);
    cmd.prm.ub_prm[11] = (uint8_t)((temp >> 16) & 0x000000ff);
    cmd.prm.ub_prm[12] = (uint8_t)((temp >> 24) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t mag_get_offsets(struct geomagnetic* offsets)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_D_LOG("start");
    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MAG_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    offsets->accuracy = res.res.ub_res[0];
    offsets->x = (int32_t)(res.res.ub_res[1] | (res.res.ub_res[2] << 8) |
        (res.res.ub_res[3] << 16) | (res.res.ub_res[4] << 24)) / 100;
    offsets->y = (int32_t)(res.res.ub_res[5] | (res.res.ub_res[6] << 8) |
        (res.res.ub_res[7] << 16) | (res.res.ub_res[8] << 24)) / 100;
    offsets->z = (int32_t)(res.res.ub_res[9] | (res.res.ub_res[10] << 8) |
        (res.res.ub_res[11] << 16) | (res.res.ub_res[12] << 24)) / 100;

    SENSOR_N_LOG("offsets [%d] [%d] [%d] [%d]",
            offsets->x,offsets->y,offsets->z,offsets->accuracy);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t mag_set_static_matrix(int32_t* smat)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_D_LOG("start");
    SENSOR_D_LOG("static_matrix[%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                    smat[0],smat[1],smat[2],
                    smat[3],smat[4],smat[5],
                    smat[6],smat[7],smat[8]);
    cmd.cmd.udata16    = HC_MAG_SET_STATIC_MATRIX;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = (uint8_t)(smat[0] & 0x000000ff);
    cmd.prm.ub_prm[2]  = (uint8_t)(((smat[0] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[3]  = (uint8_t)(smat[1] & 0x000000ff);
    cmd.prm.ub_prm[4]  = (uint8_t)(((smat[1] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[5]  = (uint8_t)(smat[2] & 0x000000ff);
    cmd.prm.ub_prm[6]  = (uint8_t)(((smat[2] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[7]  = (uint8_t)(smat[3] & 0x000000ff);
    cmd.prm.ub_prm[8]  = (uint8_t)(((smat[3] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[9]  = (uint8_t)(smat[4] & 0x000000ff);
    cmd.prm.ub_prm[10] = (uint8_t)(((smat[4] >> 8) & 0x000000ff));
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (uint8_t)(smat[5] & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(((smat[5] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[3] = (uint8_t)(smat[6] & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(((smat[6] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[5] = (uint8_t)(smat[7] & 0x000000ff);
    cmd.prm.ub_prm[6] = (uint8_t)(((smat[7] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[7] = (uint8_t)(smat[8] & 0x000000ff);
    cmd.prm.ub_prm[8] = (uint8_t)(((smat[8] >> 8) & 0x000000ff));
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    SENSOR_D_LOG("end - return[%d]", ret);

    return ret;
}
static int32_t mag_get_static_matrix(int32_t* smat)
{
    int32_t ret = SNS_RC_OK;
    int16_t tmp_matrix[9] = {0,0,0,0,0,0,0,0,0};
    int i;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_D_LOG("start");

    cmd.cmd.udata16 = HC_MAG_GET_STATIC_MATRIX;
    mutex_lock(&micon_hcres_mutex);
    ret = sns_hostcmd(&cmd, &res, 18, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_GET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    tmp_matrix[0] = (int16_t)(res.res.ub_res[0]  | (res.res.ub_res[1] << 8 ));
    tmp_matrix[1] = (int16_t)(res.res.ub_res[2]  | (res.res.ub_res[3] << 8 ));
    tmp_matrix[2] = (int16_t)(res.res.ub_res[4]  | (res.res.ub_res[5] << 8 ));
    tmp_matrix[3] = (int16_t)(res.res.ub_res[6]  | (res.res.ub_res[7] << 8 ));
    tmp_matrix[4] = (int16_t)(res.res.ub_res[8]  | (res.res.ub_res[9] << 8 ));
    tmp_matrix[5] = (int16_t)(res.res.ub_res[10] | (res.res.ub_res[11] << 8));
    tmp_matrix[6] = (int16_t)(res.res.ub_res[12] | (res.res.ub_res[13] << 8));
    tmp_matrix[7] = (int16_t)(res.res.ub_res[14] | (res.res.ub_res[15] << 8));
    tmp_matrix[8] = (int16_t)(res.res.ub_res[16] | (res.res.ub_res[17] << 8));
    for(i = 0; i < sizeof(tmp_matrix) / sizeof(tmp_matrix[0]); i++){
        smat[i] = (int32_t)tmp_matrix[i];
    }

    SENSOR_D_LOG("end - return[%d]", ret);

    return ret;
}

static int32_t gyro_set_offsets(struct gyroscope offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    struct gyroscope gyro;

    SENSOR_N_LOG("start");

    gyro.x = GYRODATA_SIGN_COMVERT_12_32BIT(offsets.x);
    SENSOR_N_LOG("set offset X[%d]",gyro.x);

    gyro.y = GYRODATA_SIGN_COMVERT_12_32BIT(offsets.y);
    SENSOR_N_LOG("set offset Y[%d]",gyro.y);

    gyro.z = GYRODATA_SIGN_COMVERT_12_32BIT(offsets.z);
    SENSOR_N_LOG("set offset Z[%d]",gyro.z);

    gyro.x *= 100;
    gyro.y *= 100;
    gyro.z *= 100;

    cmd.cmd.udata16 = HC_GYRO_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(gyro.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((gyro.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(gyro.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((gyro.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(gyro.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((gyro.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t gyro_get_offsets(struct gyroscope* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_GYRO_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    offsets->x = res.res.sw_res[0];
    offsets->y = res.res.sw_res[1];
    offsets->z = res.res.sw_res[2];

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t press_set_offsets(struct pressure offsets)
{
    DO_NOT_NEED_RUN;
    return 0;
}

static int32_t press_get_offsets(struct pressure* offsets)
{
    DO_NOT_NEED_RUN;
    return 0;
}

static int32_t sgnfcnt_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    uint8_t    param;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_EXEC_MOTION;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_MOTION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t set_dev_param(void)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
    struct acceleration acc_ofs = {0};
    struct geomagnetic mag_ofs = {0};
    struct gyroscope gyro_ofs = {0};
    if (g_acc_available) {
        cmd.cmd.udata16 = HC_ACC_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = g_acc_conv_axis;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    if (g_gyro_available) {
        cmd.cmd.udata16 = HC_GYRO_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = g_gyro_conv_axis;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    if (g_mag_available) {
        cmd.cmd.udata16 = HC_EC_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = g_mag_conv_axis;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_EC_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    if (g_acc_available) {
        cmd.cmd.udata16 = HC_ACC_SET_AUTO_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_AUTO_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    sns_load_offset_value(SENSOR_ACC, &acc_ofs);
    cmd.cmd.udata16 = HC_ACC_SET_CALIB;
    cmd.prm.ub_prm[0] = (uint8_t)(acc_ofs.nX & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((acc_ofs.nX  >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(acc_ofs.nY & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((acc_ofs.nY >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(acc_ofs.nZ & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((acc_ofs.nZ >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_CALIB err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if(g_mag_available) {
        sns_load_offset_value(SENSOR_MAG, &mag_ofs);
        mag_set_offsets(mag_ofs);
    }

    if(g_gyro_available) {
        sns_load_offset_value(SENSOR_GYRO, &gyro_ofs);
        cmd.cmd.udata16 = HC_GYRO_SET_DATA;
        cmd.prm.ub_prm[0] = (uint8_t)(gyro_ofs.x & 0x000000ff);
        cmd.prm.ub_prm[1] = (uint8_t)((gyro_ofs.x >> 8) & 0x000000ff);
        cmd.prm.ub_prm[2] = (uint8_t)(gyro_ofs.y & 0x000000ff);
        cmd.prm.ub_prm[3] = (uint8_t)((gyro_ofs.y >> 8) & 0x000000ff);
        cmd.prm.ub_prm[4] = (uint8_t)(gyro_ofs.z & 0x000000ff);
        cmd.prm.ub_prm[5] = (uint8_t)((gyro_ofs.z >> 8) & 0x000000ff);
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    ret = sns_acc_set_auto_cal_offset_internal(acc_ofs);

    if(g_mag_available) {
        cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
        cmd.prm.ub_prm[0]  = 0x00;
        cmd.prm.ub_prm[1]  = (uint8_t)(g_mag_static_matrix[0] & 0x000000ff);
        cmd.prm.ub_prm[2]  = (uint8_t)(((g_mag_static_matrix[0] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[3]  = (uint8_t)(g_mag_static_matrix[1] & 0x000000ff);
        cmd.prm.ub_prm[4]  = (uint8_t)(((g_mag_static_matrix[1] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[5]  = (uint8_t)(g_mag_static_matrix[2] & 0x000000ff);
        cmd.prm.ub_prm[6]  = (uint8_t)(((g_mag_static_matrix[2] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[7]  = (uint8_t)(g_mag_static_matrix[3] & 0x000000ff);
        cmd.prm.ub_prm[8]  = (uint8_t)(((g_mag_static_matrix[3] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[9]  = (uint8_t)(g_mag_static_matrix[4] & 0x000000ff);
        cmd.prm.ub_prm[10] = (uint8_t)(((g_mag_static_matrix[4] >> 8) & 0x000000ff));
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = (uint8_t)(g_mag_static_matrix[5] & 0x000000ff);
        cmd.prm.ub_prm[2] = (uint8_t)(((g_mag_static_matrix[5] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[3] = (uint8_t)(g_mag_static_matrix[6] & 0x000000ff);
        cmd.prm.ub_prm[4] = (uint8_t)(((g_mag_static_matrix[6] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[5] = (uint8_t)(g_mag_static_matrix[7] & 0x000000ff);
        cmd.prm.ub_prm[6] = (uint8_t)(((g_mag_static_matrix[7] >> 8) & 0x000000ff));
        cmd.prm.ub_prm[7] = (uint8_t)(g_mag_static_matrix[8] & 0x000000ff);
        cmd.prm.ub_prm[8] = (uint8_t)(((g_mag_static_matrix[8] >> 8) & 0x000000ff));
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if(g_acc_available) {
        cmd.cmd.udata16 = HC_ACC_SET_SIMPLE_FILTER;
        cmd.prm.ub_prm[0] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_SIMPLE_FILTER err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if(g_mag_available) {
        cmd.cmd.udata16 = HC_EC_SET_SIMPLE_FILTER;
        cmd.prm.ub_prm[0]  = 0x02;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_EC_SET_SIMPLE_FILTER err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if(g_gyro_available) {
        cmd.cmd.udata16 = HC_GYRO_SET_SIMPLE_FILTER;
        cmd.prm.ub_prm[0] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_SIMPLE_FILTER err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static int32_t reset_save_and_taskoff(void)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();

#ifdef CONFIG_SENSOR_MICON_LOGGING
    cmd.cmd.udata16 = HC_MUL_SET_ENABLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif

    cmd.cmd.udata16 = HC_MUL_SET_ENABLE;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = now_ctrl_param.output_param.sensor.task_exec[1];
    cmd.prm.ub_prm[2]  = now_ctrl_param.output_param.sensor.task_exec[2];
    cmd.prm.ub_prm[3]  = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    return SNS_RC_OK;
}

static int32_t sns_initcmd_exe(void)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MCU_SENSOR_INIT;
    cmd.prm.ub_prm[0] = 0x00;
    if(g_acc_available)
    {
        cmd.prm.ub_prm[0] |= 0x01;
    }
    if(g_pres_available)
    {
        cmd.prm.ub_prm[0] |= 0x20;
    }
    if(g_mag_available)
    {
        cmd.prm.ub_prm[0] |= 0x40;
    }
    if(g_gyro_available)
    {
        cmd.prm.ub_prm[0] |= 0x80;
    }
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MCU_SENSOR_INIT(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if(g_acc_available)
    {
        if((res.res.ub_res[0] & 0x01) != 0)
        {
            SENSOR_ERR_LOG("Acc Sensor Init Error[%x]",res.res.uw_res[0]);
            ret = SNS_RC_ERR;
        }
    }
    if(g_pres_available)
    {
        if((res.res.ub_res[0] & 0x20) != 0)
        {
            SENSOR_ERR_LOG("Pressure Sensor Init Error[%x]",res.res.uw_res[0]);
            ret = SNS_RC_ERR;
        }
    }
    if(g_mag_available)
    {
        if((res.res.ub_res[0] & 0x40) != 0)
        {
            SENSOR_ERR_LOG("Mag Sensor Init Error[%x]",res.res.uw_res[0]);
            ret = SNS_RC_ERR;
        }
    }
    if(g_gyro_available)
    {
        if((res.res.ub_res[0] & 0x80) != 0)
        {
            SENSOR_ERR_LOG("Gyro Sensor Init Error[%x]",res.res.uw_res[0]);
            ret = SNS_RC_ERR;
        }
    }
    SENSOR_N_LOG("end");
    return ret;
}

static int32_t enable_i2c_peri(bool enable)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    if(atomic_read(&g_FWUpdateStatus) || get_miconshtdwn_status() == true){
        SENSOR_ERR_LOG("FW Update or Recovery Now");
        return 0;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MCU_SET_PERI;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (enable == false) ? 0x01:0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 4, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MCU_SET_PERI(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    if((enable == true) && res.res.ub_res[1] == 0x01)
    {
        SENSOR_ERR_LOG("I2C Valid Error RST01[%x]",res.res.ub_res[1]);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end");
    return ret;
}

static void set_batch_notify(bool enable)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start-Enable[%d]",enable);

    cmd.cmd.udata16 = HC_MUL_SET_LOGGING_NOTIFY;
    cmd.prm.ub_prm[0] = enable;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOGGING_NOTIFY err[%x]",res.err.udata16);
        return;
    }

    SENSOR_N_LOG("end");
}

static int32_t micon_app_clear(int32_t clear_req)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;
    int32_t tmp = 0;
    int32_t tmpA = 0;

    SENSOR_N_LOG("start");

    tmp  |= (clear_req & DAILYS_CLEAR_SHOCK_STATE)      ? 0x01:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_STOP_STATE)       ? 0x02:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_OUT_STATE)        ? 0x04:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_TRAIN_STATE)      ? 0x08:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_MOTION_STATE)     ? 0x10:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_STEP_COUNTER)     ? 0x20:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_STEP_TIMESTMP)    ? 0x40:0x00;
    tmp  |= (clear_req & DAILYS_CLEAR_BATCH_TIMER)      ? 0x80:0x00;
    tmpA |= (clear_req & DAILYS_CLEAR_VH_STATE)         ? 0x01:0x00;
    tmpA |= (clear_req & DAILYS_CLEAR_EB_ALL)           ? 0x02:0x00;
    tmpA |= (clear_req & DAILYS_CLEAR_DAILYS_DAY)       ? 0x04:0x00;

    cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (clear_req & DAILYS_CLEAR_PEDO_DATA)  ? 0x01:0x00;
    cmd.prm.ub_prm[2] = (clear_req & DAILYS_CLEAR_PEDO_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[3] = (clear_req & DAILYS_CLEAR_VEHI_DATA)  ? 0x01:0x00;
    cmd.prm.ub_prm[4] = (clear_req & DAILYS_CLEAR_VEHI_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[5] = (clear_req & DAILYS_CLEAR_WIFI_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[6] = (clear_req & DAILYS_CLEAR_HEIGHT_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[7] = (clear_req & DAILYS_CLEAR_HEIGHT_ALL) ? 0x01:0x00;
    cmd.prm.ub_prm[8] = (clear_req & DAILYS_CLEAR_RT_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[9] = tmp;
    cmd.prm.ub_prm[10] = tmpA;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end Pedometer Data Clear Error[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    if (clear_req & DAILYS_CLEAR_DAILYS_DAY) {
        g_eventbuf_cleartime = GET_CURRENT_TIMESTAMP_NS();
        SENSOR_N_LOG("update eventbuf cleartime [%lld]", g_eventbuf_cleartime);
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t dailys_send_param(bool enable, const struct pedo_param pp)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    uint8_t    param_pressure;

    SENSOR_N_LOG("start enable[%d]", enable);

    if (enable) {
        param = HC_VALID;
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
        param_pressure = HC_VALID;
#else
        param_pressure = HC_INVALID;
#endif
    } else {
        param = HC_INVALID;
        param_pressure = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_SET_DAILYS;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = pp.step_wide;
    cmd.prm.ub_prm[3] = (pp.weight & 0xFF);
    cmd.prm.ub_prm[4] = ((pp.weight >> 8) & 0xFF);
    cmd.prm.ub_prm[5] = pp.vehi_type;
    cmd.prm.ub_prm[6] = param;
    cmd.prm.ub_prm[7] = param;
    cmd.prm.ub_prm[8] = param_pressure;
    cmd.prm.ub_prm[9] = 0x01;
    cmd.prm.ub_prm[10] = 0x00;
    cmd.prm.ub_prm[11] = param_pressure;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if ((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_DAILYS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end ret[%d]", ret);

    return ret;
}

static int32_t dailys_enable(bool enable, const struct pedo_param pp)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start enable[%d]", enable);

    param = enable ? HC_VALID : HC_INVALID;

    if (enable) {
        ret = dailys_send_param(enable, pp);
        if (SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end sns_dailys_send_ope_param err ret[%d]",ret);
            return SNS_RC_ERR;
        }

        ret = dailys_send_latency_option(SENSOR_EXT_PEDO, enable);
        if (SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end dailys_send_latency_option err ret[%d]",ret);
            return SNS_RC_ERR;
        }

        if (g_pres_available) {
            cmd.cmd.udata16 = HC_DST_SET_EB_PARAM;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = 0x9A;
            cmd.prm.ub_prm[2] = 0x17;
            cmd.prm.ub_prm[3] = 0x00;
            cmd.prm.ub_prm[4] = 0x00;
            cmd.prm.ub_prm[5] = 0x46;
            cmd.prm.ub_prm[6] = 0x9A;
            cmd.prm.ub_prm[7] = 0x17;
            cmd.prm.ub_prm[8] = 0x00;
            cmd.prm.ub_prm[9] = 0x00;
            cmd.prm.ub_prm[10] = 0x46;

            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_DST_SET_EB_PARAM err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
        }
        cmd.cmd.udata16 = HC_DST_SET_EB_SET;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        if (g_pres_available) {
            cmd.prm.ub_prm[5] = 0x01;
        } else {
            cmd.prm.ub_prm[5] = 0x00;
        }
        cmd.prm.ub_prm[6] = 0x01;
        cmd.prm.ub_prm[7] = 0x01;
        cmd.prm.ub_prm[8] = 0x01;

        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_SET_EB_SET err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        if (!g_micon_pedo_status && !g_micon_baro_status) {
            g_eventbuf_cleartime = GET_CURRENT_TIMESTAMP_NS();
            SENSOR_N_LOG("update eventbuf cleartime [%lld]", g_eventbuf_cleartime);
        }
    } else {
        cmd.cmd.udata16 = HC_DST_SET_EB_SET;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        cmd.prm.ub_prm[5] = 0x00;
        cmd.prm.ub_prm[6] = 0x01;
        cmd.prm.ub_prm[7] = 0x01;
        cmd.prm.ub_prm[8] = 0x01;

        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_SET_EB_SET err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        msleep(35);

        ret = dailys_send_param(enable, pp);
        if (SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end sns_dailys_send_ope_param err ret[%d]",ret);
            return SNS_RC_ERR;
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t dailys_send_latency_option(enum sensor_e_type type, bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    bool       enable_pedo = now_ctrl_param.input_param.inputs[SENSOR_EXT_PEDO].enable;
    bool       enable_baro = now_ctrl_param.input_param.inputs[SENSOR_EXT_BARO].enable;
    int32_t    period_pedo = now_ctrl_param.input_param.inputs[SENSOR_EXT_PEDO].sampling_period_ms/30;
    int32_t    period_baro = now_ctrl_param.input_param.inputs[SENSOR_EXT_BARO].sampling_period_ms/30;
    int32_t    period = 0x7FFFFFFF;

    SENSOR_N_LOG("start type[%d] enable[%d] enable_pedo[%d] enable_baro[%d] period_pedo[%d] period_baro[%d]", type, enable,
        enable_pedo, enable_baro, period_pedo, period_baro);

    if (type == SENSOR_EXT_PEDO)
        enable_pedo = enable;
    else if (type == SENSOR_EXT_BARO)
        enable_baro = enable;

    if (enable_pedo && period_pedo >= 0)
        period = min(period, period_pedo);
    if (enable_baro && period_baro >= 0)
        period = min(period, period_baro);

    cmd.cmd.udata16 = HC_DST_SET_DAILYS_OPT;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = (period >>  0) & 0xFF;
    cmd.prm.ub_prm[3] = (period >>  8) & 0xFF;
    cmd.prm.ub_prm[4] = (period >> 16) & 0xFF;
    cmd.prm.ub_prm[5] = (period >> 24) & 0xFF;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_DAILYS_OPT err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end ret[%d]", ret);

    return ret;
}

static int32_t dailys_get_pedo_data(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_PEDO1;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usStepCnt  = res.res.ud_res[0];
        arg_Pedo->usWalkTime = res.res.ud_res[1];
        arg_Pedo->usCal      = res.res.ud_res[2];
        arg_Pedo->usRTState  = res.res.ub_res[12];

        SENSOR_N_LOG("Step Count         [%u]",arg_Pedo->usStepCnt);
        SENSOR_N_LOG("Walking Time(10ms) [%u]",arg_Pedo->usWalkTime);
        SENSOR_N_LOG("Calorie(kcal)      [%u]",arg_Pedo->usCal);
        SENSOR_N_LOG("RealTime State     [%u]",arg_Pedo->usRTState);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_PEDO1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_PEDO2;
    ret = sns_hostcmd(&cmd, &res, 11, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usBodyFat  = res.res.ud_res[0];
        arg_Pedo->usExercise = res.res.ud_res[1];
        arg_Pedo->usMets     = res.res.ub_res[8];
        arg_Pedo->usSpeed    = ((res.res.ub_res[10] << 8 ) | res.res.ub_res[9]);

        SENSOR_N_LOG("Body Fat(100mg)    [%u]",arg_Pedo->usBodyFat);
        SENSOR_N_LOG("Exercise(0.1Ex)    [%u]",arg_Pedo->usExercise);
        SENSOR_N_LOG("Mets               [%d]",arg_Pedo->usMets);
        SENSOR_N_LOG("Speedometer(cm/sec)[%u]",arg_Pedo->usSpeed);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_PEDO2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_PEDO5;
    ret = sns_hostcmd(&cmd, &res, 12, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usSyncRunStepCnt = res.res.ud_res[0];
        arg_Pedo->usSyncRunTime = res.res.ud_res[1];
        arg_Pedo->usSyncRunCal = res.res.ud_res[2];

        SENSOR_N_LOG("SyncRunStepCnt     [%u]",arg_Pedo->usSyncRunStepCnt);
        SENSOR_N_LOG("SyncRunTime(10ms)  [%u]",arg_Pedo->usSyncRunTime);
        SENSOR_N_LOG("SyncRunCal(kcal)   [%u]",arg_Pedo->usSyncRunCal);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_PEDO5 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_RUN1;
    ret = sns_hostcmd(&cmd, &res, 9, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunStatus  = res.res.ub_res[0];
        arg_Pedo->usRunStepCnt = ((res.res.ub_res[4] << 24) | (res.res.uw_res[1] << 8) | res.res.ub_res[1]);
        arg_Pedo->usRunTime    = ((res.res.ub_res[8] << 24) | (res.res.uw_res[3] << 8) | res.res.ub_res[5]);

        SENSOR_N_LOG("Run Status         [%u]",arg_Pedo->usRunStatus);
        SENSOR_N_LOG("Run StepCnt        [%u]",arg_Pedo->usRunStepCnt);
        SENSOR_N_LOG("Run Time(10ms)     [%u]",arg_Pedo->usRunTime);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_RUN1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_RUN2;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunCal      = res.res.ud_res[0];
        arg_Pedo->usRunExercise = res.res.ud_res[1];

        SENSOR_N_LOG("Run Calorie(kcal)  [%u]",arg_Pedo->usRunCal);
        SENSOR_N_LOG("Run Exercise(0.1Ex)[%u]",arg_Pedo->usRunExercise);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_RUN2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    ret = dailys_get_pedo_eb_data(arg_Pedo);

    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_get_pedo_data_ext ret[%x]",ret);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t dailys_get_pedo_eb_data(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO1;
    ret = sns_hostcmd(&cmd, &res, 2, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->LogTimeStamp3sec  = res.res.uw_res[0];

        SENSOR_N_LOG("LogTimeStamp3sec   [%u]",arg_Pedo->LogTimeStamp3sec);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO2;
    ret = sns_hostcmd(&cmd, &res, 9, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->PD_Log_DetState    = res.res.ub_res[0];
        arg_Pedo->PD_Log_SectionStep =
            ((res.res.ub_res[2] << 8 ) | res.res.ub_res[1]);
        arg_Pedo->PD_Log_SectionCal  =
            ((res.res.ub_res[4] << 8 ) | res.res.ub_res[3]);
        arg_Pedo->RN_Log_SectionStep =
            ((res.res.ub_res[6] << 8 ) | res.res.ub_res[5]);
        arg_Pedo->RN_Log_SectionCal  =
            ((res.res.ub_res[8] << 8 ) | res.res.ub_res[7]);

        SENSOR_N_LOG("PD_Log_DetState    [%u]",arg_Pedo->PD_Log_DetState);
        SENSOR_N_LOG("PD_Log_SectionStep [%u]",arg_Pedo->PD_Log_SectionStep);
        SENSOR_N_LOG("PD_Log_SectionCal  [%u]",arg_Pedo->PD_Log_SectionCal);
        SENSOR_N_LOG("RN_Log_SectionStep [%u]",arg_Pedo->RN_Log_SectionStep);
        SENSOR_N_LOG("RN_Log_SectionCal  [%u]",arg_Pedo->RN_Log_SectionCal);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO3;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->PD_Log_SectionUpH     = res.res.uw_res[0];
        arg_Pedo->PD_Log_SectionDnH     = res.res.uw_res[1];
        arg_Pedo->PD_Log_SectionUpDnCal = res.res.uw_res[2];

        SENSOR_N_LOG("PD_Log_SectionUpH    [%u]",arg_Pedo->PD_Log_SectionUpH);
        SENSOR_N_LOG("PD_Log_SectionDnH    [%u]",arg_Pedo->PD_Log_SectionDnH);
        SENSOR_N_LOG("PD_Log_SectionUpDnCal[%u]",arg_Pedo->PD_Log_SectionUpDnCal);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO3 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO4;
    ret = sns_hostcmd(&cmd, &res, 3, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->VC_Log_DetState    = res.res.ub_res[0];
        arg_Pedo->VC_Log_SectionCal  =
            ((res.res.ub_res[2] << 8 ) | res.res.ub_res[1]);

        SENSOR_N_LOG("VC_Log_DetState    [%u]",arg_Pedo->VC_Log_DetState);
        SENSOR_N_LOG("VC_Log_SectionCal  [%u]",arg_Pedo->VC_Log_SectionCal);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO4 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO5;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->VC_Log_SectionUpH     = res.res.uw_res[0];
        arg_Pedo->VC_Log_SectionDnH     = res.res.uw_res[1];
        arg_Pedo->VC_Log_SectionUpDnCal = res.res.uw_res[2];

        SENSOR_N_LOG("VC_Log_SectionUpH    [%u]",arg_Pedo->VC_Log_SectionUpH);
        SENSOR_N_LOG("VC_Log_SectionDnH    [%u]",arg_Pedo->VC_Log_SectionDnH);
        SENSOR_N_LOG("VC_Log_SectionUpDnCal[%u]",arg_Pedo->VC_Log_SectionUpDnCal);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO5 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO6;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->PS_SectionPress        = res.res.sw_res[0];
        arg_Pedo->PS_SectionHeight       = res.res.sw_res[1];
        arg_Pedo->PS_SectionActiveHeight = res.res.sw_res[2];

        SENSOR_N_LOG("PS_SectionPress       [%d]",arg_Pedo->PS_SectionPress);
        SENSOR_N_LOG("PS_SectionHeight      [%d]",arg_Pedo->PS_SectionHeight);
        SENSOR_N_LOG("PS_SectionActiveHeight[%d]",arg_Pedo->PS_SectionActiveHeight);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO6 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t dailys_get_vehi_data(struct vehicle *arg_Vehi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_TRANS1;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Vehi->usVehiStatus     = res.res.ub_res[0];
        arg_Vehi->usVehiKind       = res.res.ub_res[1];
        arg_Vehi->usVehiDetectTime = ((res.res.uw_res[2] << 16) | res.res.uw_res[1]);
        arg_Vehi->usVehiRideTime   = ((res.res.uw_res[4] << 16) | res.res.uw_res[3]);

        SENSOR_N_LOG("Vehi Status          [%u]",arg_Vehi->usVehiStatus);
        SENSOR_N_LOG("Vehi Kind            [%u]",arg_Vehi->usVehiKind);
        SENSOR_N_LOG("Vehi DetectTime(10ms)[%u]",arg_Vehi->usVehiDetectTime);
        SENSOR_N_LOG("Vehi RideTime(10ms)  [%u]",arg_Vehi->usVehiRideTime);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_TRANS1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_TRANS2;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Vehi->usVehiRideCal  = res.res.ud_res[0];
        arg_Vehi->usVehiBodyFat  = res.res.ud_res[1];
        arg_Vehi->usVehiExercise = res.res.ud_res[2];
        arg_Vehi->usVehiMets     = res.res.ub_res[12];

        SENSOR_N_LOG("Vehi RideCal(kcal)   [%u]",arg_Vehi->usVehiRideCal);
        SENSOR_N_LOG("Vehi BodyFat(100mg)  [%d]",arg_Vehi->usVehiBodyFat);
        SENSOR_N_LOG("Vehi Exercise(Ex)    [%u]",arg_Vehi->usVehiExercise);
        SENSOR_N_LOG("Vehi Mets            [%u]",arg_Vehi->usVehiMets);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_TRANS2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_TRANS6;
    ret = sns_hostcmd(&cmd, &res, 12, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Vehi->usVehiOtherRideTime = res.res.ud_res[0];
        arg_Vehi->usVehiBicycRideTime = res.res.ud_res[1];
        arg_Vehi->usVehiTrainRideTime = res.res.ud_res[2];

        SENSOR_N_LOG("Vehi OtherRideTime(10msec)[%u]",arg_Vehi->usVehiOtherRideTime);
        SENSOR_N_LOG("Vehi BicycRideTime(10msec)[%u]",arg_Vehi->usVehiBicycRideTime);
        SENSOR_N_LOG("Vehi TrainRideTime(10msec)[%u]",arg_Vehi->usVehiTrainRideTime);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_TRANS6 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t event_mark_off(void)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    uint16_t eventbuf_data_size = 0;
    uint16_t read_data_size = 0;
    uint8_t  fifo_size_res[2];
    uint16_t fifo_data_size = 0;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);

    wake_lock_timeout(&g_pedo_wakelock, PEDOMETER_WAKELOCK_TIME);
    cmd.cmd.udata16 = HC_DST_GET_EB_PUNCTUATION;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_DST_EB_PUNCTUATION[%x]", res.err.udata16);
        mutex_unlock(&micon_hcres_mutex);
        return SNS_RC_ERR;
    }

    ret = dev_read(RSLT3E, fifo_size_res, 2);
    SENSOR_N_LOG("dev_read-ret[%d] res1[%x] res2[%x]",ret,fifo_size_res[0],fifo_size_res[1]);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("dev_read[%d]", ret);
        mutex_unlock(&micon_hcres_mutex);
        return ret;
    }
    fifo_data_size = ((fifo_size_res[1] << 8) | fifo_size_res[0]);
    SENSOR_N_LOG("dev_read-fifo_data_size[%x]",fifo_data_size);

    if(EVENTBUF_RESPONSE_HEADER > fifo_data_size || fifo_data_size > sizeof(res.res.ub_res)) {
        SENSOR_ERR_LOG("fifo_data_size[%d]", fifo_data_size);
        mutex_unlock(&micon_hcres_mutex);
        return SNS_RC_ERR;
    }

    ret = dev_read(FIFO, res.res.ub_res, fifo_data_size);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("dev_read[%d]", ret);
        mutex_unlock(&micon_hcres_mutex);
        return ret;
    }
    eventbuf_data_size = ((res.res.ub_res[1] << 8) | res.res.ub_res[0]);
    SENSOR_N_LOG("eventbuf_data_size[%x]",eventbuf_data_size);

    if(eventbuf_data_size == 0) {
        goto done;
    }

    memcpy(g_eventbuf_data, &res.res.ub_res[EVENTBUF_RESPONSE_HEADER], fifo_data_size - EVENTBUF_RESPONSE_HEADER);

    SENSOR_D_LOG("total_size=%d data[%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x...]",
        eventbuf_data_size,
        g_eventbuf_data[0], g_eventbuf_data[1], g_eventbuf_data[2], g_eventbuf_data[3],
        g_eventbuf_data[4], g_eventbuf_data[5], g_eventbuf_data[6], g_eventbuf_data[7],
        g_eventbuf_data[8], g_eventbuf_data[9], g_eventbuf_data[10], g_eventbuf_data[11],
        g_eventbuf_data[12], g_eventbuf_data[13], g_eventbuf_data[14], g_eventbuf_data[15],
        g_eventbuf_data[16], g_eventbuf_data[17], g_eventbuf_data[18], g_eventbuf_data[19]);

    read_data_size = (fifo_data_size - EVENTBUF_RESPONSE_HEADER);
    eventbuf_data_size -= (fifo_data_size - EVENTBUF_RESPONSE_HEADER);
    SENSOR_D_LOG("read_data_size[%d]-eventbuf_data_size[%d]",read_data_size,eventbuf_data_size);

    if(eventbuf_data_size > sizeof(g_eventbuf_data) - read_data_size) {
        mutex_unlock(&micon_hcres_mutex);
        SENSOR_ERR_LOG("invalid buffer size.  eventbuf_data_size[%d] fifo_data_size[%d]",
            eventbuf_data_size, fifo_data_size);
        return SNS_RC_ERR;
    }

    while (eventbuf_data_size > 0) {
        if ((0 < eventbuf_data_size) && (eventbuf_data_size <= 512)) {
            cmd.cmd.udata16 = HC_DST_GET_EB_DATA;
            cmd.prm.ub_prm[0] = 0x00;
            cmd.prm.ub_prm[1] = (uint8_t)eventbuf_data_size;
            cmd.prm.ub_prm[2] = (uint8_t)(eventbuf_data_size>>8);
            ret = sns_hostcmd(&cmd, &res, eventbuf_data_size, EXE_HOST_ALL, READ_FIFO);
            if ((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_DST_GET_EB_DATA[%x]", res.err.udata16);
                mutex_unlock(&micon_hcres_mutex);
                return SNS_RC_ERR;
            }

            if (g_eventbuf_data != NULL) {
                memcpy((g_eventbuf_data + read_data_size), &res.res.ub_res, eventbuf_data_size);
                read_data_size += eventbuf_data_size;
                eventbuf_data_size = 0;
            } else {
                SENSOR_ERR_LOG("g_eventbuf_data:NULL!!!");
                break;
            }
        } else if (512 < eventbuf_data_size) {
            cmd.cmd.udata16 = HC_DST_GET_EB_DATA;
            cmd.prm.ub_prm[0] = 0x00;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x02;
            ret = sns_hostcmd(&cmd, &res, 512, EXE_HOST_ALL, READ_FIFO);
            if ((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_DST_GET_EB_DATA[%x]", res.err.udata16);
                mutex_unlock(&micon_hcres_mutex);
                return SNS_RC_ERR;
            }

            if (g_eventbuf_data != NULL) {
                memcpy((g_eventbuf_data + read_data_size), &res.res.ub_res, 512);
                read_data_size += 512;
                eventbuf_data_size -= 512;
            } else {
                SENSOR_ERR_LOG("g_eventbuf_data:NULL!!!");
                break;
            }
        }
    }

    sensor_report_event(SENSOR_EXT_PEDO, g_eventbuf_cleartime, (void*)g_eventbuf_data, read_data_size);

done:
    cmd.cmd.udata16 = HC_DST_RCV_IBUF_DONE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_RCV_IBUF_DONE err[%x]",res.err.udata16);
        mutex_unlock(&micon_hcres_mutex);
        return SNS_RC_ERR;
    }

    mutex_unlock(&micon_hcres_mutex);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static void dailys_vib_interlocking(int32_t mode)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;
    int8_t mode_vib = INTERLOCKING_OFF;
    int8_t mode_spk = INTERLOCKING_OFF;
    int8_t interlocking = INTERLOCKING_OFF;
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    struct sensor_ctrl_param_output_str param = now_ctrl_param.output_param;

    SENSOR_D_LOG("start");

    if(mode & VIB_INTERLOCKING_ON)      mode_vib = INTERLOCKING_ON;
    if(mode & SPEAKER_INTERLOCKING_ON)  mode_spk = INTERLOCKING_ON;
    interlocking = mode_vib | mode_spk;
#ifdef CONFIG_USE_SENSOR_LSM6DS3
    if (g_interlocking != mode && !(IS_ANDROID_EN(param.enable_type.acc_en) || IS_FUSION_EN(param.enable_type.acc_en))) {
        /* By limitation of device, when Hi performance mode is changed, *
         * ODR selection must be chenge to other setting.                *
         * So, it set 416MHz once and set expected ODR selection after.  */
        cmd.cmd.udata16 = HC_ACC_SET_PARAM_LSM6DS3;
        cmd.prm.ub_prm[0] = param.sensor.acc_param[0];
        cmd.prm.ub_prm[1] = 0x05; /* 416MHz */
        cmd.prm.ub_prm[2] = param.sensor.acc_param[2];
        cmd.prm.ub_prm[3] = param.sensor.acc_param[3];
        cmd.prm.ub_prm[4] = interlocking;
        cmd.prm.ub_prm[5]  = 0x00;

        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_PARAM_LSM6DS3 err[%x]",res.err.udata16);
            return;
        }

        cmd.cmd.udata16 = HC_ACC_SET_PARAM_LSM6DS3;
        cmd.prm.ub_prm[0] = param.sensor.acc_param[0];
        cmd.prm.ub_prm[1] = param.sensor.acc_param[1];
        cmd.prm.ub_prm[2] = param.sensor.acc_param[2];
        cmd.prm.ub_prm[3] = param.sensor.acc_param[3];
        cmd.prm.ub_prm[4] = interlocking;
        cmd.prm.ub_prm[5]  = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        g_interlocking = mode;
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_PARAM_LSM6DS3 err[%x]",res.err.udata16);
        }
    }
#else
    if (g_interlocking != mode ) {
        cmd.cmd.udata16 = HC_DST_SET_VIB_INTERLOCKING;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = mode_vib;
        cmd.prm.ub_prm[2] = mode_spk;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        g_interlocking = mode;
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_SET_VIB_INTERLOCKING err[%x]",res.err.udata16);
            return;
        }
    }
#endif
    SENSOR_D_LOG("end - return[%d]",ret);
}

static int32_t baro_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start enable[%d]", enable);

    param = enable ? HC_VALID : HC_INVALID;

    if (enable) {
        cmd.cmd.udata16 = HC_BRM_SET_EB_PARAM;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x05;
        cmd.prm.ub_prm[2] = 0x76;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        cmd.prm.ub_prm[5] = 0x0E;

        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_BRM_SET_EB_PARAM err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    ret = dailys_send_latency_option(SENSOR_EXT_BARO, enable);
    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end dailys_send_latency_option err ret[%d]",ret);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_BRM_SET_EB_SET;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_BRM_SET_EB_SET err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if (enable) {
        if (!g_micon_pedo_status && !g_micon_baro_status) {
            g_eventbuf_cleartime = GET_CURRENT_TIMESTAMP_NS();
            SENSOR_N_LOG("update eventbuf cleartime [%lld]", g_eventbuf_cleartime);
        }
    }
    g_micon_baro_status = enable;

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t vhdet_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_EXEC_VH;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x00;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_VH err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t vhdet_get_data(struct vhdetect *arg_VHdetect)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_VH;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_VHdetect->usVhStatus = res.res.ub_res[0];

        SENSOR_N_LOG("VH Status [%d]",arg_VHdetect->usVhStatus);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_VH err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_wstart_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_WALK_START;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_WALK_START err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_wstop_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_WALK_STOP;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_WALK_STOP err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_train_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_TRAIN;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x01;
    cmd.prm.ub_prm[3] = 0x00;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_TRAIN err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_train_get_data(struct kc_motion_train *arg_Train)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MOT_GET_TRAIN_INFO;
    ret = sns_hostcmd(&cmd, &res, 11, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Train->usTrFstDetect = res.res.ub_res[0];
        arg_Train->usTrOtherFstDetect = res.res.ub_res[1];
        arg_Train->usTrRes = res.res.ub_res[2];
        arg_Train->TrDtctTime = ((res.res.ub_res[3]      ) & 0x000000FF)
                              + ((res.res.ub_res[4] <<  8) & 0x0000FF00)
                              + ((res.res.ub_res[5] << 16) & 0x00FF0000)
                              + ((res.res.ub_res[6] << 24) & 0xFF000000);
        arg_Train->TrDtctTimeFix = ((res.res.ub_res[7]       ) & 0x000000FF)
                                 + ((res.res.ub_res[8]  <<  8) & 0x0000FF00)
                                 + ((res.res.ub_res[9]  << 16) & 0x00FF0000)
                                 + ((res.res.ub_res[10] << 24) & 0xFF000000);

        SENSOR_N_LOG("First Detected       [%u]",arg_Train->usTrFstDetect);
        SENSOR_N_LOG("First Other Detected [%u]",arg_Train->usTrOtherFstDetect);
        SENSOR_N_LOG("Total Detected       [%u]",arg_Train->usTrRes);
        SENSOR_N_LOG("Detect Time          [%d]",arg_Train->TrDtctTime);
        SENSOR_N_LOG("Fix Detect Time      [%d]",arg_Train->TrDtctTimeFix);
    }else{
        SENSOR_ERR_LOG("end HC_MOT_GET_TRAIN_INFO err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_other_vehi_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_VEHICLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x01;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_VEHICLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_bringup_enable(bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_BRINGUP;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x05;
    cmd.prm.ub_prm[3] = 0x03;
    cmd.prm.ub_prm[4] = 0x0C;
    cmd.prm.ub_prm[5] = 0x06;
    cmd.prm.uw_prm[3] = 0x00B6;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_BRINGUP err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_get_bringup_data(struct kc_motion_bringup_data *arg_Bringup)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MOT_GET_BRINGUP_INFO;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Bringup->status = res.res.ub_res[0];

        SENSOR_N_LOG("Bringup status [%u]",arg_Bringup->status);
    }else{
        SENSOR_ERR_LOG("end HC_MOT_GET_BRINGUP_INFO err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t uwater_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;

    HostCmd cmd;
    HostCmdRes res;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_UWD_SET_EXEC;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = param;
    cmd.prm.ub_prm[3] = HC_INVALID;
    cmd.prm.ub_prm[4] = HC_INVALID;
    cmd.prm.ub_prm[5] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end %s err[%x]", __func__ ,res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t vtrigger_enable(bool enable)
{
    SENSOR_N_LOG("start");
    UNAVAILABLE_FEATURE;
    SENSOR_N_LOG("end");
    return SNS_RC_OK;
}

static void irq_proc(uint64_t dummyIRQfactorVal)
{
    int32_t            ret = SNS_RC_OK;
    HostCmd            cmd;
    HostCmdRes         res;
    uint8_t            pedo_detect = 0;
    uint8_t            vehi_detect = 0;
    uint8_t            ps_detect = 0;
    uint8_t            wifi_detect = 0;
    uint8_t            apl_detect = 0;
    uint8_t            apl_detect2 = 0;
    uint8_t            apl_detect3 = 0;
    uint8_t            eb_punctuation_done = false;
    uint8_t drv_status = (uint8_t)atomic_read(&s_snsdrv_status);
    Word               log_size;
    uint8_t            count_10, count, offset, i, j;
    char               strLog[50], temp[5];
    struct geomagnetic* m_ofs;
    struct gyroscope* g_ofs;

    SENSOR_D_LOG("start");
    if(drv_status == (uint8_t)SENSOR_SHUTDOWN){
        SENSOR_D_LOG("end -skip due to shutdown sequence running.");
        return;
    }
    wake_lock_timeout(&g_pedo_wakelock, PEDOMETER_WAKELOCK_TIME);
    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_DST_GET_INT_DETAIL;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("###  Error HC_DST_GET_INT_DETAIL");
        goto done;
    }

    SENSOR_D_LOG("HC_DST_GET_INT_DETAIL 0:[%x] 1:[%x] 2:[%x] 3:[%x] 4:[%x] 5:[%x] 6:[%x] 7:[%x] 8:[%x] 9:[%x]\n",
               res.res.ub_res[0],res.res.ub_res[1],res.res.ub_res[2],res.res.ub_res[3],
               res.res.ub_res[4],res.res.ub_res[5],res.res.ub_res[6],res.res.ub_res[7],res.res.ub_res[8],res.res.ub_res[9]);

    if(res.res.ub_res[0] == 0x01){
        pedo_detect = 0x01;
        SENSOR_N_LOG("### Step Detected !!");
        sensor_flush_event_buffer();
        eb_punctuation_done = true;
        sns_notify(SENSOR_EXT_PEDO,0);
    }

    if(res.res.ub_res[1] == 0x01){
        vehi_detect = 0x01;
        SENSOR_N_LOG("### Vehicle Detected !!");
        if(!eb_punctuation_done)
            sensor_flush_event_buffer();
        eb_punctuation_done = true;
        sns_notify(SENSOR_EXT_PEDO,0);
    }

    if(res.res.ub_res[2] == 0x01){
        ps_detect = 0x01;
        SENSOR_N_LOG("### Pressure Detected !!");
        if(!eb_punctuation_done)
            sensor_flush_event_buffer();
        eb_punctuation_done = true;
    }

    if(res.res.ub_res[3] == 0x01){
        wifi_detect = 0x01;
        SENSOR_N_LOG("### Intelli Wifi Detected !!");
        sns_notify(SENSOR_EXT_IWIFI,0);
    }

    if(res.res.ub_res[7] & 0x02){
        apl_detect |= 0x02;
        SENSOR_N_LOG("### Walk Stop Detected !!");
        sns_notify(SENSOR_KC_MOTION_WALK_STOP,0);
    }

    if(res.res.ub_res[7] & 0x04){
        apl_detect |= 0x04;
        SENSOR_N_LOG("### Bringup Detected !!");
        sns_notify(SENSOR_KC_MOTION_BRINGUP,0);
    }

    if(res.res.ub_res[7] & 0x08){
        apl_detect |= 0x08;
        SENSOR_N_LOG("### Train Detected !!");
        sns_notify(SENSOR_KC_MOTION_TRAIN,0);
    }

    if(res.res.ub_res[7] & 0x10){
        apl_detect |= 0x10;
        SENSOR_N_LOG("### First Train Detected !!");
        sns_notify(SENSOR_KC_MOTION_TRAIN,0);
    }

    if(res.res.ub_res[7] & 0x20){
        apl_detect |= 0x20;
        SENSOR_N_LOG("### Motion Detected !!");
        sns_notify(SENSOR_SGNFCNT_MTN,0);
    }

    if(res.res.ub_res[7] & 0x80){
        apl_detect |= 0x80;
        SENSOR_N_LOG("### VH Detected !!");
        g_AppVhFirstReportDone = true;
        sns_notify(SENSOR_EXT_VH,0);
        sns_notify(SENSOR_DEVICE_ORIENTATION,0);
    }

    if(res.res.ub_res[8] & 0x01){
        apl_detect2 |= 0x01;
        SENSOR_N_LOG("### Walk Start Detected !!");
        sns_notify(SENSOR_KC_MOTION_WALK_START,0);
    }

    if(res.res.ub_res[8] & 0x02){
        apl_detect2 |= 0x02;
        SENSOR_N_LOG("### Vehicle Start Detected !!");
        sns_notify(SENSOR_KC_MOTION_VEHICLE,0);
    }

    if(res.res.ub_res[9] & 0x01){
        apl_detect3 |= 0x01;
        SENSOR_N_LOG("### Acc AutoCalibration Detected !!");
        sns_notify(SENSOR_ACC_AUTO_CAL,0);
    }

    if(res.res.ub_res[9] & 0x02){
        apl_detect3 |= 0x02;
        SENSOR_N_LOG("### underwater Detected !!");
        sns_notify(SENSOR_UNDERWATER_DETECT,0);
    }

    if(res.res.ub_res[9] & 0x50){
        apl_detect3 |= (res.res.ub_res[9] & 0x50);
        SENSOR_N_LOG("### Event Buffer Int Detected!! res[9]=0x%02x", res.res.ub_res[9]);
        if(!eb_punctuation_done)
            sensor_flush_event_buffer();
        eb_punctuation_done = true;
    }

    if(res.res.ub_res[9] & 0x20){
        apl_detect3 |= 0x20;
        SENSOR_N_LOG("### Event Buffer Full Detected!!");
        sensor_ext_clear(DAILYS_CLEAR_DAILYS_DAY);
    }
#ifdef CONFIG_SENSOR_MICON_LOGGING
    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MCU_GET_INT_DETAIL;
    cmd.prm.ub_prm[0] = 0xc0;
    cmd.prm.ub_prm[1] = 0x40;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("###  Error HC_MCU_GET_INT_DETAIL");
        goto done;
    }

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    if(res.res.ub_res[0] == 0x01){
        SENSOR_N_LOG("### Geomagnetic Detected !!");
        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            goto done;
        } else {
            m_ofs->accuracy = res.res.ub_res[0];
            m_ofs->x = (int32_t)(res.res.ub_res[1] | (res.res.ub_res[2] << 8) |
                (res.res.ub_res[3] << 16) | (res.res.ub_res[4] << 24)) / 100;
            m_ofs->y = (int32_t)(res.res.ub_res[5] | (res.res.ub_res[6] << 8) |
                (res.res.ub_res[7] << 16) | (res.res.ub_res[8] << 24)) / 100;
            m_ofs->z = (int32_t)(res.res.ub_res[9] | (res.res.ub_res[10] << 8) |
                (res.res.ub_res[11] << 16) | (res.res.ub_res[12] << 24)) / 100;
            sns_save_offset_value(SENSOR_MAG, m_ofs);

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d] a[%d]",m_ofs->x,m_ofs->y,m_ofs->z,m_ofs->accuracy);
        }
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE

    if(res.res.ub_res[2] == 0x01){
        SENSOR_N_LOG("### Gyroscope Detected !!");
        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            goto done;
        } else {
            g_ofs->x = res.res.sw_res[0];
            g_ofs->y = res.res.sw_res[1];
            g_ofs->z = res.res.sw_res[2];
            sns_save_offset_value(SENSOR_GYRO, g_ofs);

            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",g_ofs->x,g_ofs->y,g_ofs->z);
        }
    }
#endif

#endif
    cmd.cmd.udata16 = HC_DST_CLR_INT_DETAIL;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = pedo_detect;
    cmd.prm.ub_prm[2] = vehi_detect;
    cmd.prm.ub_prm[3] = ps_detect;
    cmd.prm.ub_prm[4] = wifi_detect;
    cmd.prm.ub_prm[5] = 0x00;
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = apl_detect;
    cmd.prm.ub_prm[8] = apl_detect2;
    cmd.prm.ub_prm[9] = apl_detect3;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if(SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end HC_DST_CLR_INT_DETAIL err[%x]",res.err.udata16);
        goto done;
    }


    if(atomic_read(&g_MiconDebug) == true){
        SENSOR_ERR_LOG("###Micon Debug ");

        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = KC_LOG_READ;
        sns_hostcmd(&cmd, &res, KC_LOG_SIZE, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        log_size.udata16 = ( (res.res.ub_res[0]) | (res.res.ub_res[1] << 8));

        count_10 = log_size.udata16 / 10;
        count = log_size.udata16 % 10;
        offset = 0;

        SENSOR_ERR_LOG("###log_size:[%d] ", log_size.udata16);

        for(i = 0; i < count_10 ; i++)
        {
            SENSOR_ERR_LOG("###[%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x]",
                               res.res.ub_res[0+offset], res.res.ub_res[1+offset], res.res.ub_res[2+offset],
                               res.res.ub_res[3+offset], res.res.ub_res[4+offset], res.res.ub_res[5+offset],
                               res.res.ub_res[6+offset], res.res.ub_res[7+offset], res.res.ub_res[8+offset],
                               res.res.ub_res[9+offset] );
            offset += 10;
        }

        for(j = 0; j < count ; j++)
        {
            sprintf(temp, "[%02x]", res.res.ub_res[j + (10 * count_10)] );
            strcat(strLog, temp);
        }
        SENSOR_ERR_LOG("###%s ", strLog);
    }
done:
    SENSOR_D_LOG("end");
    return;
}

void func_register_ml630q790(struct sensor_micon_if_funcs *f)
{
    SENSOR_D_LOG("start");
    f->get_tblidx2size              = get_tbl_idx_to_size;
    f->get_tblidx2smid              = get_tbl_idx_to_smid;
    f->get_snstype2tblidx           = get_sensor_type_to_tbl_idx;
    f->get_snstype2smid             = get_sensor_type_to_smid;
    f->get_fastest_snstask_val      = get_fastest_sensortask_val;
    f->get_batch_fifo_size          = get_batch_fifo_size;
    f->get_fw_version               = get_fw_version;
    f->get_sns_value                = get_sns_value;
    f->get_miconshtdwn_status       = get_miconshtdwn_status;
    f->set_sensdrv_status           = set_sensordrv_status;
    f->write                        = dev_write;
    f->read                         = dev_read;
    f->unit_chg                     = unit_change;
    f->send_sensor_ctrl             = send_sensor_ctrl;
    f->send_logging_state           = send_logging_state;
    f->set_flush                    = set_flush;
    f->enable_irq_wake              = enable_irq_wake_irq;
    f->batch_mark_off               = batch_mark_off;
    f->iio_report_event_now         = iio_report_event_now;
    f->init_snsdriver               = initialize_snsdrv;
    f->get_micon_info               = get_sns_info_from_micon_dt;
    f->irq_proc                     = irq_proc;
    f->set_dev_param                = set_dev_param;
    f->reset_saveparam_and_taskoff  = reset_save_and_taskoff;
    f->set_sns_initcmd              = sns_initcmd_exe;
    f->enable_i2c_peripheral        = enable_i2c_peri;
    f->set_batch_notify             = set_batch_notify;
    f->acc_func.set_offsets         = acc_set_offsets;
    f->acc_func.get_offsets         = acc_get_offsets;
    f->acc_func.set_ac_offsets      = acc_set_ac_offsets;
    f->acc_func.get_ac_offsets      = acc_get_ac_offsets;
    f->mag_func.set_offsets         = mag_set_offsets;
    f->mag_func.get_offsets         = mag_get_offsets;
    f->mag_func.set_static_matrix   = mag_set_static_matrix;
    f->mag_func.get_static_matrix   = mag_get_static_matrix;
    f->gyro_func.set_offsets        = gyro_set_offsets;
    f->gyro_func.get_offsets        = gyro_get_offsets;
    f->press_func.set_offsets       = press_set_offsets;
    f->press_func.get_offsets       = press_get_offsets;
    f->sgnfcnt_func.enable          = sgnfcnt_enable;
    f->app_cmn.clear                = micon_app_clear;
    f->ds_func.send_ope_param       = dailys_send_param;
    f->ds_func.enable               = dailys_enable;
    f->ds_func.send_latency_option  = dailys_send_latency_option;
    f->ds_func.get_pedo_data        = dailys_get_pedo_data;
    f->ds_func.get_vehi_data        = dailys_get_vehi_data;
    f->ds_func.event_mark_off       = event_mark_off;
    f->ds_func.vib_interlocking     = dailys_vib_interlocking;
    f->baro_func.enable             = baro_enable;
    f->vhdet_func.enable            = vhdet_enable;
    f->vhdet_func.get_data          = vhdet_get_data;
    f->kcmot_func.wstart_enable     = kcmot_wstart_enable;
    f->kcmot_func.wstop_enable      = kcmot_wstop_enable;
    f->kcmot_func.train_enable      = kcmot_train_enable;
    f->kcmot_func.train_get_data    = kcmot_train_get_data;
    f->kcmot_func.other_vehi_enable = kcmot_other_vehi_enable;
    f->kcmot_func.bringup_enable    = kcmot_bringup_enable;
    f->kcmot_func.bringup_get_data  = kcmot_get_bringup_data;
    f->uw_func.enable               = uwater_enable;
    f->vt_func.enable               = vtrigger_enable;
    SENSOR_D_LOG("end");
}
