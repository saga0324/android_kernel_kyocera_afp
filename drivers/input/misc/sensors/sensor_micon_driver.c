/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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
/* CwMcuSensor.c - driver file for HTC SensorHUB
 *
 * Copyright (C) 2014 HTC Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "sensor_micon_driver.h"
#include "sensor_com.h"
#include <linux/version.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/sensor_micon_common.h>

#ifdef CONFIG_INPUT_SENSOR_EOSS3
#include <linux/ql/qleos_s3.h>
#include <linux/ql/qlspi_linux.h>
#include <linux/ql/qlpower_ext.h>
#include "sensor_micon_eoss3.h"
#endif
#ifdef CONFIG_INPUT_SENSOR_ML630Q790
#include <linux/sensor_power.h>
#include "sensor_micon_ml630q790.h"
#endif

#define DEFAULT_WEIGHT                  (650)
#define DEFAULT_STEPWIDE                (76)
#define DEFAULT_VEHITYPE                (2)
#define MICON_I2C_ENABLE_RETRY_NUM      (5)
#define GYRO_POWER_ON_WAIT_MS           (100)
#define ACC_POWER_ON_WAIT_MS            (20)
#define SENSOR_MICON_DRIVER_NAME        "sensor_micon"

/* ------------------------------------------------
  Variables
------------------------------------------------ */
struct sensor_ctrl_config_str {
    const enum sensor_e_type            sns_type;
    const int32_t                       en_type;
    const int32_t                       bt_type;
    int32_t                             en_bit_measure;
    int32_t                             en_bit_logging;
    struct sensor_poll_info_str*        poll_info_p;
};

// -----------------------------------------------
// global variables
// -----------------------------------------------
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
uint8_t g_acc_available = 1;
#else
uint8_t g_acc_available = 0;
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
uint8_t g_gyro_available = 1;
#else
uint8_t g_gyro_available = 0;
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
uint8_t g_mag_available = 1;
#else
uint8_t g_mag_available = 0;
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
uint8_t g_pres_available = 1;
#else
uint8_t g_pres_available = 0;
#endif

int32_t g_interlocking = 0x00;
bool g_AppVhFirstReportDone;
bool g_micon_pedo_status;
bool g_micon_baro_status;
bool g_bDevIF_Error = false;
bool g_micon_error = false;
//struct wake_lock g_sns_wakelock;
struct spi_device *client_sns;

// -----------------------------------------------
// static variables
// -----------------------------------------------
static struct sensor_ctrl_config_str sns_ctrl_config[SENSOR_MAX] = {
                                      /*sns_type                      en_type             bt_type             en_bit_measure                      en_bit_logging          poll_info_p */
    [SENSOR_ACC]                    = { SENSOR_ACC,                   EN_TYPE_NORMAL,     BT_TYPE_NORMAL,     EN_BIT_ACC,                         EN_BIT_ACC,             NULL },
    [SENSOR_ACC_AUTO_CAL]           = { SENSOR_ACC_AUTO_CAL,          EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_MAG]                    = { SENSOR_MAG,                   EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M,                         EN_BIT_MAG,             NULL },
    [SENSOR_MAG_UNCAL]              = { SENSOR_MAG_UNCAL,             EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M  |EN_BIT_MAG_UNCAL,      EN_BIT_MAG_UNCAL,       NULL },
    [SENSOR_MAG_ROT_VCTR]           = { SENSOR_MAG_ROT_VCTR,          EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M  |EN_BIT_MAG_ROT_VCTR,   EN_BIT_MAG_ROT_VCTR,    NULL },
    [SENSOR_ACC_LNR]                = { SENSOR_ACC_LNR,               EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M_G|EN_BIT_ACC_LNR,        EN_BIT_ACC_LNR,         NULL },
    [SENSOR_GRV]                    = { SENSOR_GRV,                   EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M_G|EN_BIT_GRV,            EN_BIT_GRV,             NULL },
    [SENSOR_GYRO]                   = { SENSOR_GYRO,                  EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M_G,                       EN_BIT_GYRO,            NULL },
    [SENSOR_GYRO_UNCAL]             = { SENSOR_GYRO_UNCAL,            EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M_G|EN_BIT_GYRO_UNCAL,     EN_BIT_GYRO_UNCAL,      NULL },
    [SENSOR_ORTN]                   = { SENSOR_ORTN,                  EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M_G|EN_BIT_ORTN,           EN_BIT_ORTN,            NULL },
    [SENSOR_ROT_VCTR]               = { SENSOR_ROT_VCTR,              EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_M_G|EN_BIT_ROT_VCTR,       EN_BIT_ROT_VCTR,        NULL },
    [SENSOR_GAME_ROT_VCTR]          = { SENSOR_GAME_ROT_VCTR,         EN_TYPE_FUSION,     BT_TYPE_NORMAL,     EN_BIT_A_G  |EN_BIT_GAME_ROT_VCTR,  EN_BIT_GAME_ROT_VCTR,   NULL },
    [SENSOR_LIGHT]                  = { SENSOR_LIGHT,                 EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_PROX]                   = { SENSOR_PROX,                  EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
#if 0
    [SENSOR_TEMP]                   = { SENSOR_TEMP,                  EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_TEMP_AMBIENT]           = { SENSOR_TEMP_AMBIENT,          EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_RH]                     = { SENSOR_RH,                    EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
#endif
    [SENSOR_PRESSURE]               = { SENSOR_PRESSURE,              EN_TYPE_NORMAL,     BT_TYPE_NORMAL,     EN_BIT_PRESSURE,                    EN_BIT_PRESSURE,        NULL },
    [SENSOR_SGNFCNT_MTN]            = { SENSOR_SGNFCNT_MTN,           EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M|EN_BIT_SIGNFCNT_MTN,     EN_BIT_NONE,            NULL },
    [SENSOR_STEP_CNT]               = { SENSOR_STEP_CNT,              EN_TYPE_APP,        BT_TYPE_STEP_CNT,   EN_BIT_ACC|EN_BIT_STEP_CNT,         EN_BIT_STEP_CNT,        NULL },
    [SENSOR_STEP_DTC]               = { SENSOR_STEP_DTC,              EN_TYPE_APP,        BT_TYPE_STEP_DTC,   EN_BIT_ACC|EN_BIT_STEP_DTC,         EN_BIT_STEP_DTC,        NULL },
    [SENSOR_DEVICE_ORIENTATION]     = { SENSOR_DEVICE_ORIENTATION,    EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_STATIONARY_DETECT]      = { SENSOR_STATIONARY_DETECT,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_MOTION_DETECT]          = { SENSOR_MOTION_DETECT,         EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_EXT_PEDO]               = { SENSOR_EXT_PEDO,              EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M_P,                       EN_BIT_NONE,            NULL },
    [SENSOR_EXT_VEHI]               = { SENSOR_EXT_VEHI,              EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M_P,                       EN_BIT_NONE,            NULL },
    [SENSOR_EXT_BARO]               = { SENSOR_EXT_BARO,              EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_PRESSURE,                    EN_BIT_NONE,            NULL },
    [SENSOR_EXT_IWIFI]              = { SENSOR_EXT_IWIFI,             EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M_P,                       EN_BIT_NONE,            NULL },
    [SENSOR_EXT_VH]                 = { SENSOR_EXT_VH,                EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_WALK_START]   = { SENSOR_KC_MOTION_WALK_START,  EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_WALK_STOP]    = { SENSOR_KC_MOTION_WALK_STOP,   EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_TRAIN]        = { SENSOR_KC_MOTION_TRAIN,       EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_VEHICLE]      = { SENSOR_KC_MOTION_VEHICLE,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_BRINGUP]      = { SENSOR_KC_MOTION_BRINGUP,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_UNDERWATER_DETECT]      = { SENSOR_UNDERWATER_DETECT,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_PRESSURE,                    EN_BIT_NONE,            NULL },
    [SENSOR_PIEZO_PRESS]            = { SENSOR_PIEZO_PRESS,           EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_PIEZO_WAKEUP]           = { SENSOR_PIEZO_WAKEUP,          EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_VOICE_TRIGGER]          = { SENSOR_VOICE_TRIGGER,         EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL },
    [SENSOR_COM]                    = { SENSOR_COM,                   EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL }
    /* SENSOR_MAX */
};

#ifdef CONFIG_INPUT_SENSOR_EOSS3
static struct qlpower_callback sns_pre_power_cb;
#endif
#ifdef CONFIG_INPUT_SENSOR_ML630Q790
static struct sensor_power_callback sns_pre_power_cb;
#endif
static struct iio_dev * g_iio_dev = NULL;
static struct sensor_micon_if_funcs sns_micon_func;
static struct sensor_ctrl_param_str sns_ctrl_param;
static struct pedo_param sns_pedo_param;
static struct hrtimer micon_poll_timer;
static struct workqueue_struct* micon_poll_wq;
static struct work_struct micon_work;
static uint8_t g_reset_param_status;
static int64_t g_logging_base_timestamps[SENSOR_BATCH_TBL_MAX] = {0};

static atomic_t g_pressure_cal_offset;
static atomic_t g_ResetStatus = ATOMIC_INIT(false);

static atomic_t g_fastest_poll_p = ATOMIC_INIT(PERIOD_ACC_MAX);
static struct mutex sensor_fifo_mutex;

static DEFINE_MUTEX(g_iio_mutex);
static DECLARE_BITMAP(fusion_sns_mask, SENSOR_MAX);

static int32_t save_status = SNS_RC_OK;

/* ------------------------------------------------
  Prototype Functions
------------------------------------------------ */
static inline int32_t sns_adjust_value_by_range_enable(int32_t value, bool enable, int32_t max, int32_t min, bool default_is_max);
static inline int32_t sns_adjust_value_by_range(int32_t value, int32_t max, int32_t min);
static int32_t sns_make_sensor_ctrl_info(struct sensor_ctrl_param_input_str* src, struct sensor_ctrl_param_output_str *dst);
static int32_t sns_judge_sensor_ctrl_change_type(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new
);
static int32_t sns_ctrl_polling(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new
);
static void sns_confirm_polling_stop(void);
static int32_t sns_update_micon_control_if_needed(bool force);
static int32_t sns_adjust_sampling_period(
    enum sensor_e_type type,
    int32_t period
);
static bool sns_set_input_param_batch(
    enum sensor_e_type type,
    struct sensor_batch_info_str *info
);
static bool sns_set_input_param_enable(
    enum sensor_e_type type,
    bool enable
);
static int32_t sns_get_micon_info_dt( struct spi_device *client );
static bool sns_devif_error_check(void);
static void sns_pre_power_on(void);
static void sns_pre_power_off(void);
static void sns_micon_init(void);
static void sns_micon_func_register(void);
static int32_t sns_reset_save_param(void);
static int32_t sns_reset_restore_param(void);

static void __sensor_exec_flush(enum sensor_e_type type);
static void __sensor_irq_proc(uint64_t result);
static int32_t __sensor_dev_resume( struct device *dev );
static int32_t __sensor_dev_suspend( struct device *dev );
static void __sensor_dev_shutdown( struct spi_device *client );
static int32_t __sensor_dev_remove( struct spi_device *client );
static int32_t __sensor_dev_init( struct spi_device *client );
static int32_t __sensor_dev_save_param(void);
static int32_t __sensor_dev_load_param(void);
static int sns_iio_report_flush(enum sensor_e_type type, int64_t timestamp);
static int sns_iio_probe_buffer(struct iio_dev *iio_dev);
static void sns_iio_remove_buffer(struct iio_dev *indio_dev);
static int sns_iio_init(void);
static int sns_iio_exit(void);

extern int ql_spi_power_reset(void);

void sns_poll_init(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info )
{
    int i;

    SENSOR_N_LOG("start");

    for(i = 0; i < ARRAY_SIZE(sns_ctrl_config); i++) {
        if (sns_ctrl_config[i].sns_type == type) {
            sns_ctrl_config[i].poll_info_p = poll_info;
            break;
        }
    }

    SENSOR_N_LOG("end");

    return;
}

#define PERIOD_ACC_MAX          (480)
#define PERIOD_ACC_MIN          (5)
#define PERIOD_MAG_MAX          (480)
#define PERIOD_MAG_MIN          (10)
#define PERIOD_GYRO_MAX         (480)
#define PERIOD_GYRO_MIN         (5)
#define PERIOD_PRESSURE_MAX     (480)
#define PERIOD_PRESSURE_MIN     (50)
#define PERIOD_FUSION_TASK_MAX  (100)
#define PERIOD_FUSION_TASK_MIN  (10)
#define PERIOD_FUSION_MAX       (100)
#define PERIOD_FUSION_MIN       (10)
#define PERIOD_APP_TASK_FAST    (30)
#define PERIOD_APP_TASK_SLOW    (240)
#define PERIOD_FUSION_BATCH_MAX (0x7FFF)
#define BATCH_TIMEOUT_MAX       (0x7FFFFFFF)
#define BATCH_TIMEOUT_MIN       (4)
int32_t sns_device_write(uint8_t adr, const uint8_t *data, uint8_t size)
{
	int32_t ret = SNS_RC_OK;
	ret = sns_micon_func.write(adr, data, size);
	return ret;
}
int32_t sns_device_read(uint32_t adr, uint8_t *data, uint16_t size)
{
	int32_t ret = SNS_RC_OK;
	ret = sns_micon_func.read(adr, data, size);
	return ret;
}
struct sensor_ctrl_param_str get_sns_ctrl_param(void)
{
    return sns_ctrl_param;
}
void get_fusion_sns_mask(unsigned long *dst, unsigned int nbits)
{
    bitmap_copy(dst, fusion_sns_mask, nbits);
}

static inline int32_t sns_adjust_value_by_range_enable(int32_t value, bool enable, int32_t max, int32_t min, bool default_is_max)
{
    int32_t ret;
    SENSOR_N_LOG("start");
    if (enable) {
        ret = max(min, value);
        ret = min(ret, max);
    } else {
        if (default_is_max)
            ret = max;
        else
            ret = min;
    }
    SENSOR_N_LOG("end");
    return ret;
}

static inline int32_t sns_adjust_value_by_range(int32_t value, int32_t max, int32_t min)
{
    int32_t ret;

    SENSOR_N_LOG("start");
    if (value < min) {
        ret = min;
    } else if (value > max) {
        ret = max;
    } else {
        ret = value;
    }
    SENSOR_N_LOG("end value[%d] ret[%d]", value, ret);
    return ret;
}

static int32_t sns_make_sensor_ctrl_info(struct sensor_ctrl_param_input_str* src, struct sensor_ctrl_param_output_str *dst)
{
    struct sensor_ctrl_param_input_one_str* input;
    const struct sensor_ctrl_config_str* config;
    int32_t period_acc = PERIOD_ACC_MAX;
    int32_t period_mag = PERIOD_MAG_MAX;
    int32_t period_gyro = PERIOD_GYRO_MAX;
    int32_t period_pressure = PERIOD_PRESSURE_MAX;
    int32_t period_acc_logging = PERIOD_ACC_MAX;
    int32_t period_mag_logging = PERIOD_MAG_MAX;
    int32_t period_gyro_logging = PERIOD_GYRO_MAX;
    int32_t period_pressure_logging = PERIOD_PRESSURE_MAX;
    int32_t period_fusion = PERIOD_FUSION_MAX;
    int32_t period_fusion_task = PERIOD_FUSION_TASK_MAX;
    int32_t period_sensor_task;
    int32_t period_app_task = PERIOD_APP_TASK_SLOW;
    int32_t batch_timeout = BATCH_TIMEOUT_MAX;
    int32_t batch_timeout_step = BATCH_TIMEOUT_MAX;
    int32_t period_R_logging = PERIOD_FUSION_BATCH_MAX;
    int32_t period_GeR_logging = PERIOD_FUSION_BATCH_MAX;
    int32_t period_GaR_logging = PERIOD_FUSION_BATCH_MAX;
    int32_t period_O_logging = PERIOD_FUSION_BATCH_MAX;
    int32_t period_Grv_logging = PERIOD_FUSION_BATCH_MAX;
    int32_t period_LA_logging = PERIOD_FUSION_BATCH_MAX;
    DECLARE_BITMAP(sensor_enable_sns, SENSOR_MAX);
    uint32_t measure_enable = 0;
    uint32_t logging_enable = 0;
    struct sensor_en_type en_type = {0};
    DECLARE_BITMAP(logging_enable_sns, SENSOR_MAX);
    uint8_t dd_control = 0;
    uint8_t acc_param[5] = {0};
    uint8_t gyro_param[6] = {0};
    uint8_t mag_param[3] = {0};
    uint8_t pressure_param[3] = {0};
    uint8_t task_exec[3] = {0};
    uint8_t batch_normal_enable = 0;
    uint8_t batch_step_cnt_enable = 0;
    uint8_t batch_step_dtc_enable = 0;
    uint8_t batch_enable = 0;
    uint32_t period_FFETimer = 0;
    DECLARE_BITMAP(polling_enable_sns, SENSOR_MAX);
    bool fusion_enable = false;
    bool app_enable = false;
    bool android_sensor_enable = false;
    int i;
    bool can_batch_in_resume;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    /* Do Nothing. */
#else
    char buf[SENSOR_TYPE_STATUS_LOG_SIZE] = {0};
#endif /* LINUX_VERSION_CODE */
    SENSOR_N_LOG("start");

    bitmap_zero(sensor_enable_sns, SENSOR_MAX);
    bitmap_zero(logging_enable_sns, SENSOR_MAX);
    bitmap_zero(polling_enable_sns, SENSOR_MAX);
    // enable/timeout
    for (i = 0; i < ARRAY_SIZE(src->inputs); i++) {
        config = &sns_ctrl_config[i];
        input = &src->inputs[i];

        if (input->enable) {
            set_bit(config->sns_type, sensor_enable_sns);
            // type check
            if (config->en_type == EN_TYPE_NORMAL) {
                android_sensor_enable = true;
                if (config->en_bit_measure & EN_BIT_ACC)
                    period_acc = min(period_acc, input->sampling_period_ms);
                if (config->en_bit_measure & EN_BIT_PRESSURE)
                    period_pressure = min(period_pressure, input->sampling_period_ms);

                if (config->en_bit_logging & EN_BIT_ACC)
                    period_acc_logging = min(period_acc_logging, input->sampling_period_ms);
                if (config->en_bit_logging & EN_BIT_PRESSURE)
                    period_pressure_logging = min(period_pressure_logging, input->sampling_period_ms);
            } else if (config->en_type == EN_TYPE_FUSION) {
                fusion_enable = true;
                period_fusion = min(period_fusion, input->sampling_period_ms);
                if (config->en_bit_measure & EN_BIT_ACC)
                    period_acc = min(period_acc, input->sampling_period_ms);
                if (config->en_bit_measure & EN_BIT_MAG)
                    period_mag = min(period_mag, input->sampling_period_ms);
                if (config->en_bit_measure & EN_BIT_GYRO)
                    period_gyro = min(period_gyro, input->sampling_period_ms);

                if (config->en_bit_logging & (EN_BIT_MAG | EN_BIT_MAG_UNCAL))
                    period_mag_logging = min(period_mag_logging, input->sampling_period_ms);
                if (config->en_bit_logging & (EN_BIT_GYRO | EN_BIT_GYRO_UNCAL))
                    period_gyro_logging = min(period_gyro_logging, input->sampling_period_ms);

                if(config->en_bit_logging & EN_BIT_ROT_VCTR)
                    period_R_logging = min(period_R_logging, input->sampling_period_ms);
                if(config->en_bit_logging & EN_BIT_MAG_ROT_VCTR)
                    period_GeR_logging = min(period_GeR_logging, input->sampling_period_ms);
                if(config->en_bit_logging & EN_BIT_GAME_ROT_VCTR)
                    period_GaR_logging = min(period_GeR_logging, input->sampling_period_ms);
                if(config->en_bit_logging & EN_BIT_ORTN)
                    period_O_logging = min(period_O_logging, input->sampling_period_ms);
                if(config->en_bit_logging & EN_BIT_GRV)
                    period_Grv_logging = min(period_Grv_logging, input->sampling_period_ms);
                if(config->en_bit_logging & EN_BIT_ACC_LNR)
                    period_LA_logging = min(period_LA_logging, input->sampling_period_ms);
            } else if (config->en_type == EN_TYPE_APP) {
                app_enable = true;
                if (config->en_bit_measure & EN_BIT_ACC) {
                    if (config->sns_type == SENSOR_DEVICE_ORIENTATION ||
                        config->sns_type == SENSOR_EXT_VH ||
                        config->sns_type == SENSOR_KC_MOTION_BRINGUP)
                        period_acc = min(period_acc, 30);
                    else
                        period_acc = min(period_acc, 240);
                }
                if (config->en_bit_measure & EN_BIT_MAG)
                    period_mag = min(period_mag, 240);
                if (config->en_bit_measure & EN_BIT_PRESSURE) {
                    if (config->sns_type == SENSOR_UNDERWATER_DETECT)
                        period_pressure = min(period_pressure, 50);
                    else
                        period_pressure = min(period_pressure, 480);
                }
            } else {
                // nop
                continue;
            }

            if ((config->en_bit_measure | config->en_bit_logging) & EN_BIT_ACC)
                en_type.acc_en |= 1 << config->en_type;
            if ((config->en_bit_measure | config->en_bit_logging) & EN_BIT_MAG)
                en_type.mag_en |= 1 << config->en_type;
            if ((config->en_bit_measure | config->en_bit_logging) & EN_BIT_GYRO)
                en_type.gyro_en |= 1 << config->en_type;
            if ((config->en_bit_measure | config->en_bit_logging) & EN_BIT_PRESSURE)
                en_type.press_en |= 1 << config->en_type;

#ifdef CONFIG_SENSOR_MICON_LOGGING
            // batch
            can_batch_in_resume = true;
            if (config->bt_type == BT_TYPE_NORMAL && (input->max_report_latency_ms >= BATCH_TIMEOUT_MIN || src->in_suspend)) {
                batch_enable = 0x01;
                batch_normal_enable = 0x01;
                batch_timeout = min(batch_timeout, input->max_report_latency_ms);
            } else if (config->bt_type == BT_TYPE_STEP_CNT) {
                batch_enable = 0x01;
                batch_step_cnt_enable = 0x01;
                batch_timeout_step = min(batch_timeout_step, input->max_report_latency_ms);
            } else if (config->bt_type == BT_TYPE_STEP_DTC) {
                batch_enable = 0x01;
                batch_step_dtc_enable = 0x01;
                batch_timeout_step = min(batch_timeout_step, input->max_report_latency_ms);
            } else {
                can_batch_in_resume = false;
            }

            // enable
            measure_enable |= config->en_bit_measure;
            if (src->in_suspend) {
                logging_enable |= config->en_bit_logging;
                set_bit(config->sns_type, logging_enable_sns);
            } else {
                if (can_batch_in_resume) {
                    logging_enable |= config->en_bit_logging;
                    set_bit(config->sns_type, logging_enable_sns);
                } else if (config->poll_info_p != NULL) {
                    set_bit(config->sns_type, polling_enable_sns);
                } else {
                    // nop
                }
            }
#else
            (void)can_batch_in_resume;
            // enable
            measure_enable |= config->en_bit_measure;
            if (!src->in_suspend && config->poll_info_p != NULL) {
                set_bit(config->sns_type, polling_enable_sns);
            }
#endif
        }
    }
    period_fusion_task = period_fusion;

    // adjust period or timeout
    if (!fusion_enable && app_enable && !android_sensor_enable) {
        if (test_bit(SENSOR_EXT_VH, sensor_enable_sns) ||
            test_bit(SENSOR_KC_MOTION_BRINGUP, sensor_enable_sns) ||
            test_bit(SENSOR_DEVICE_ORIENTATION, sensor_enable_sns) ||
            test_bit(SENSOR_UNDERWATER_DETECT, sensor_enable_sns))
                period_sensor_task = PERIOD_APP_TASK_FAST;
        else
                period_sensor_task = PERIOD_APP_TASK_SLOW;
    } else
        period_sensor_task = sns_micon_func.get_fastest_snstask_val();

    period_FFETimer = period_sensor_task;

    if (app_enable){
        if (test_bit(SENSOR_EXT_VH, sensor_enable_sns) ||
            test_bit(SENSOR_KC_MOTION_BRINGUP, sensor_enable_sns) ||
            test_bit(SENSOR_DEVICE_ORIENTATION, sensor_enable_sns) ||
            test_bit(SENSOR_UNDERWATER_DETECT, sensor_enable_sns))
            period_app_task = PERIOD_APP_TASK_FAST;
    }

    period_acc = sns_adjust_value_by_range_enable(period_acc,
        IS_ACC_EN(measure_enable), PERIOD_ACC_MAX, PERIOD_ACC_MIN, true);
    period_mag = sns_adjust_value_by_range_enable(period_mag,
        IS_MAG_EN(measure_enable), PERIOD_MAG_MAX, PERIOD_MAG_MIN, true);
    period_gyro = sns_adjust_value_by_range_enable(period_gyro,
        IS_GYRO_EN(measure_enable), PERIOD_GYRO_MAX, PERIOD_GYRO_MIN, true);
    period_pressure = sns_adjust_value_by_range_enable(period_pressure,
        IS_PRESSURE_EN(measure_enable), PERIOD_PRESSURE_MAX, PERIOD_PRESSURE_MIN, true);

    period_acc_logging = sns_adjust_value_by_range_enable(period_acc_logging,
        IS_ACC_EN(logging_enable), PERIOD_ACC_MAX, PERIOD_ACC_MIN, false);
    period_mag_logging = sns_adjust_value_by_range_enable(period_mag_logging,
        IS_MAG_EN(logging_enable)||IS_MAG_UNCAL_EN(logging_enable),
        PERIOD_MAG_MAX, PERIOD_MAG_MIN, false);
    period_gyro_logging = sns_adjust_value_by_range_enable(period_gyro_logging,
        IS_GYRO_EN(logging_enable)||IS_GYRO_UNCAL_EN(logging_enable),
        PERIOD_GYRO_MAX, PERIOD_GYRO_MIN, false);
    period_pressure_logging = sns_adjust_value_by_range_enable(period_pressure_logging,
        IS_PRESSURE_EN(logging_enable), PERIOD_PRESSURE_MAX, PERIOD_PRESSURE_MIN, false);

    period_fusion = sns_adjust_value_by_range_enable(period_fusion,
        fusion_enable, PERIOD_FUSION_MAX, PERIOD_FUSION_MIN, true);
    period_fusion_task = sns_adjust_value_by_range_enable(period_fusion_task,
        fusion_enable, PERIOD_FUSION_TASK_MAX, PERIOD_FUSION_TASK_MIN, true);
    period_R_logging = sns_adjust_value_by_range_enable(period_R_logging,
        IS_ROT_VCTR_EN(logging_enable), PERIOD_FUSION_BATCH_MAX, PERIOD_FUSION_TASK_MIN, true);
    period_GeR_logging = sns_adjust_value_by_range_enable(period_GeR_logging,
        IS_MAG_ROT_VCTR_EN(logging_enable), PERIOD_FUSION_BATCH_MAX, PERIOD_FUSION_TASK_MIN, true);
    period_GaR_logging = sns_adjust_value_by_range_enable(period_GaR_logging,
        IS_GAME_ROT_VCTR_EN(logging_enable), PERIOD_FUSION_BATCH_MAX, PERIOD_FUSION_TASK_MIN, true);
    period_O_logging = sns_adjust_value_by_range_enable(period_O_logging,
        IS_ORTN_EN(logging_enable), PERIOD_FUSION_BATCH_MAX, PERIOD_FUSION_TASK_MIN, true);
    period_Grv_logging = sns_adjust_value_by_range_enable(period_Grv_logging,
        IS_GRV_EN(logging_enable), PERIOD_FUSION_BATCH_MAX, PERIOD_FUSION_TASK_MIN, true);
    period_LA_logging = sns_adjust_value_by_range_enable(period_LA_logging,
        IS_ACC_LNR_EN(logging_enable), PERIOD_FUSION_BATCH_MAX, PERIOD_FUSION_TASK_MIN, true);
    batch_timeout = sns_adjust_value_by_range_enable(batch_timeout,
        batch_normal_enable, BATCH_TIMEOUT_MAX, BATCH_TIMEOUT_MIN, true);
    batch_timeout_step = sns_adjust_value_by_range_enable(batch_timeout_step,
        batch_step_cnt_enable || batch_step_dtc_enable, BATCH_TIMEOUT_MAX, BATCH_TIMEOUT_MIN, true);

#ifdef CONFIG_USE_SENSOR_LSM6DS3
    //acc param
    acc_param[0] = 0x01;
    if (IS_ANDROID_EN(en_type.acc_en) || IS_FUSION_EN(en_type.acc_en) || period_acc < 20)
        acc_param[1] = 0x04;
    else if (period_acc < 80)
        acc_param[1] = 0x02;
    else
        acc_param[1] = 0x00;
    acc_param[2] = 0x00;
    acc_param[3] = 0x01;
    acc_param[4] = (IS_ANDROID_EN(en_type.acc_en) || IS_FUSION_EN(en_type.acc_en)) ? 0x01 : (uint8_t)!!g_interlocking;
#else
    //acc param
    acc_param[0] = 0x02;
    if (period_acc < 30)
        acc_param[1] = 0x05;
    else
        acc_param[1] = 0x04;
    acc_param[2] = 0x01;
#endif
    //gyro param
    gyro_param[0] = 0x04;
#ifdef CONFIG_USE_SENSOR_LSM6DS3
    gyro_param[1] = 0x04;
#else
    if (period_gyro < 10)
        gyro_param[1] = 0x06;
    else if(period_gyro < 50)
        gyro_param[1] = 0x04;
    else
        gyro_param[1] = 0x02;
#endif
    gyro_param[2] = 0x00;
    gyro_param[3] = 0x00;
    gyro_param[4] = (80 + period_gyro - 1) / period_gyro;
#ifdef CONFIG_USE_SENSOR_LSM6DS3
    gyro_param[5] = 0x00;
#else
    gyro_param[5] = 0x01;
#endif

    //mag param
#ifdef CONFIG_INPUT_SENSOR_YAS_MAG
    if (period_mag < 50) {
        mag_param[0] = 0x0F;
        mag_param[1] = 0x00;
    } else if (period_mag < 240) {
        mag_param[0] = 0x32;
        mag_param[1] = 0x00;
    } else {
        mag_param[0] = 0xF0;
        mag_param[1] = 0x00;
    }
    mag_param[2] = 0x00;
#elif defined(CONFIG_INPUT_SENSOR_HSCDTD008A_MAG)
    if(period_mag < 50) {
        mag_param[0] = 0x03;
    } else if (period_mag < 100) {
        mag_param[0] = 0x02;
    } else {
        mag_param[0] = 0x01;
    }
    mag_param[1] = 0x00;
    mag_param[2] = 0x01;
#endif
    //pressure param
    pressure_param[0] = 0x01;
    pressure_param[1] = 0x00;
    pressure_param[2] = 0x01;

    // task_exec
    if (android_sensor_enable) {
        task_exec[0] = 0x01;
    }
    if (fusion_enable) {
        task_exec[0] = 0x01;
        task_exec[2] = 0x01;
    }
    if (batch_enable || app_enable) {
        task_exec[0] = 0x01;
        task_exec[1] = 0x01;
    }

    //dynamic_device_control
    if (!fusion_enable && app_enable && !android_sensor_enable)
        dd_control = (test_bit(SENSOR_EXT_VH, sensor_enable_sns) || test_bit(SENSOR_KC_MOTION_BRINGUP, sensor_enable_sns) ||  test_bit(SENSOR_DEVICE_ORIENTATION, sensor_enable_sns))? 0x00 : 0x01;
    else
        dd_control = 0x00;

    // update dst
    memset(dst, 0x00, sizeof(*dst));

    dst->sensor.enable = measure_enable;
    dst->sensor.period_sensor_task = period_sensor_task;
    dst->sensor.period_acc = period_acc;
    dst->sensor.period_mag = period_mag;
    dst->sensor.period_gyro = period_gyro;
    dst->sensor.period_pressure = period_pressure;
    dst->sensor.period_fusion_task = period_fusion_task;
    dst->sensor.period_fusion = period_fusion;
    dst->sensor.period_app_task = period_app_task;
    dst->sensor.dd_control = dd_control;
    memcpy(dst->sensor.acc_param, acc_param, sizeof(dst->sensor.acc_param));
    memcpy(dst->sensor.gyro_param, gyro_param, sizeof(dst->sensor.gyro_param));
    memcpy(dst->sensor.mag_param, mag_param, sizeof(dst->sensor.mag_param));
    memcpy(dst->sensor.pressure_param, pressure_param, sizeof(dst->sensor.pressure_param));
    memcpy(dst->sensor.task_exec, task_exec, sizeof(dst->sensor.task_exec));

#ifdef CONFIG_SENSOR_MICON_LOGGING
    dst->logging.enable = logging_enable;
    bitmap_copy(dst->logging.enable_sns, logging_enable_sns, SENSOR_MAX);
    dst->logging.period_sensor_task = period_sensor_task;
    dst->logging.period_acc = period_acc_logging;
    dst->logging.period_mag = period_mag_logging;
    dst->logging.period_gyro = period_gyro_logging;
    dst->logging.period_pressure = period_pressure_logging;
    dst->logging.period_fusion_task = period_fusion_task;
    dst->logging.period_fusion = period_fusion;
    dst->logging.period_R = period_R_logging;
    dst->logging.period_GeR = period_GeR_logging;
    dst->logging.period_GaR = period_GaR_logging;
    dst->logging.period_O = period_O_logging;
    dst->logging.period_Grv = period_Grv_logging;
    dst->logging.period_LA = period_LA_logging;

    if (src->in_suspend) {
        dst->logging.batch_timeout = BATCH_TIMEOUT_MAX;
        dst->logging.batch_timeout_step = BATCH_TIMEOUT_MAX;
    } else {
        dst->logging.batch_timeout = batch_timeout;
        dst->logging.batch_timeout_step = batch_timeout_step;
    }
    dst->logging.batch_timeout_set = (uint32_t)min(dst->logging.batch_timeout, dst->logging.batch_timeout_step);

    dst->logging.batch_step_cnt_enable = batch_step_cnt_enable;
    dst->logging.batch_step_dtc_enable = batch_step_dtc_enable;
    dst->logging.batch_enable = batch_enable;
#endif 
    dst->period_FFETimer = period_FFETimer;


    bitmap_copy(dst->polling.enable_sns, polling_enable_sns, SENSOR_MAX);
    bitmap_copy(dst->enable_sns, sensor_enable_sns, SENSOR_MAX);

    dst->enable_type = en_type;
    //Unit adjustment of micon differences
    sns_micon_func.unit_chg(dst);

    SENSOR_D_LOG("available acc=%d gyro=%d mag=%d pres=%d", g_acc_available, g_gyro_available, g_mag_available, g_pres_available);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    SENSOR_D_LOG("param sensor enable_sns=%*pb", SENSOR_MAX, dst->enable_sns);
    SENSOR_D_LOG("param logging enable_bit=%08x enable_sns=%*pb", dst->logging.enable, SENSOR_MAX, dst->logging.enable_sns);
    SENSOR_D_LOG("param polling enable_sns=%*pb", SENSOR_MAX, dst->polling.enable_sns);
#else
    bitmap_scnprintf(buf, SENSOR_TYPE_STATUS_LOG_SIZE, dst->enable_sns, SENSOR_MAX);
    SENSOR_D_LOG("param sensor enable_sns=%s", buf);
    bitmap_scnprintf(buf, SENSOR_TYPE_STATUS_LOG_SIZE, dst->logging.enable_sns, SENSOR_MAX);
    SENSOR_D_LOG("param logging enable_bit=%08x enable_sns=%s", dst->logging.enable, buf);
    bitmap_scnprintf(buf, SENSOR_TYPE_STATUS_LOG_SIZE, dst->polling.enable_sns, SENSOR_MAX);
    SENSOR_D_LOG("param polling enable_sns=%s", buf);
#endif /* LINUX_VERSION_CODE */
    SENSOR_D_LOG("param measure enable_bit=%08x", dst->sensor.enable);
    SENSOR_N_LOG("end");
    return 0;
}

#define CHANGE_TYPE_NONE            (0x00000000)
#define CHANGE_TYPE_SENSOR          (0x00000001)
#define CHANGE_TYPE_LOGGING         (0x00000002)
#define CHANGE_TYPE_POLLING         (0x00000004)
#define CHANGE_TYPE_SENSOR_OFF      (0x00000010)
#define CHANGE_TYPE_POLLING_OFF     (0x00000040)

static int32_t sns_judge_sensor_ctrl_change_type(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new)
{
    int32_t change_type = CHANGE_TYPE_NONE;
    SENSOR_D_LOG("start");

    if (!new)
        return change_type;

    if (!old) {
        change_type |= CHANGE_TYPE_SENSOR;
        change_type |= CHANGE_TYPE_LOGGING;
        change_type |= CHANGE_TYPE_POLLING;
        if (new->sensor.enable == 0)
            change_type |= CHANGE_TYPE_SENSOR_OFF;
    } else {
        if (old->sensor.enable || new->sensor.enable) {
            if (memcmp(&old->sensor, &new->sensor, sizeof(old->sensor)) != 0) {
                change_type |= CHANGE_TYPE_SENSOR;

                if (old->sensor.enable != new->sensor.enable &&
                    new->sensor.enable == 0)
                    change_type |= CHANGE_TYPE_SENSOR_OFF;
            }

            if (memcmp(&old->logging, &new->logging, sizeof(old->logging)) != 0)
                change_type |= CHANGE_TYPE_LOGGING;

            if (memcmp(&old->polling, &new->polling, sizeof(old->polling)) != 0) {
                change_type |= CHANGE_TYPE_POLLING;

                if (!bitmap_equal(old->polling.enable_sns, new->polling.enable_sns, SENSOR_MAX) &&
                    bitmap_empty(new->polling.enable_sns, SENSOR_MAX))
                    change_type |= CHANGE_TYPE_POLLING_OFF;
            }
        }
    }

    SENSOR_D_LOG("end - return[%d]",change_type);
    return change_type;
}

static int32_t reculc_micon_polling(unsigned long *polling_enable_sns)
{
    struct sensor_poll_info_str* poll_info_p;
    int32_t i;
    int32_t period = S32_MAX;
    SENSOR_D_LOG("start");

    for(i = 0; i < SENSOR_MAX; i++){
        poll_info_p = sns_ctrl_config[i].poll_info_p;
        if(test_bit(i, polling_enable_sns) && poll_info_p != NULL){
            period = min(period, atomic_read(&(poll_info_p->poll_time)));
        }
    }
    SENSOR_D_LOG("start");
    return period;
}

static int32_t sns_ctrl_polling(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new)
{
    struct sensor_poll_info_str* poll_info_p;
    int32_t i;
    int32_t power_on_wait_ms;
    int32_t period_fastest = PERIOD_ACC_MAX;
    
    DECLARE_BITMAP(changed_bit, SENSOR_MAX);
    SENSOR_N_LOG("start");

    bitmap_xor(changed_bit, old->polling.enable_sns, new->polling.enable_sns, SENSOR_MAX);
    for (i = 0; i < SENSOR_MAX; i++) {
        poll_info_p = sns_ctrl_config[i].poll_info_p;
        if ((test_bit(i, changed_bit)) && (poll_info_p != NULL)) {
            if (test_bit(i, new->polling.enable_sns)) {
                SENSOR_N_LOG("queue work type[%d]", i);
                period_fastest = min(period_fastest, atomic_read(&(poll_info_p->poll_time)));
                if (IS_GYRO_EN(sns_ctrl_config[i].en_bit_measure)) {
                    power_on_wait_ms = GYRO_POWER_ON_WAIT_MS;
                } else if (IS_ACC_EN(sns_ctrl_config[i].en_bit_measure)) {
                    power_on_wait_ms = ACC_POWER_ON_WAIT_MS;
                } else {
                    power_on_wait_ms = 0;
                }
                if (sns_ctrl_config[i].sns_type == SENSOR_ACC) {
                    sns_micon_func.acc_func.clear_filt_coff();
                }
                if (sns_ctrl_config[i].sns_type == SENSOR_GYRO) {
                    sns_micon_func.gyro_func.clear_filt_coff();
                }
                if (sns_ctrl_config[i].sns_type == SENSOR_GYRO_UNCAL) {
                    sns_micon_func.gyrouc_func.clear_filt_coff();
                }
                sensor_starttimer(&poll_info_p->timer, atomic_read(&(poll_info_p->poll_time))
                  + power_on_wait_ms);
            } else {
                SENSOR_N_LOG("cancel work type[%d]", i);
                sensor_stoptimer(&poll_info_p->timer);
                sensor_com_mutex_unlock();
                cancel_work_sync(&(poll_info_p->work));
                sensor_com_mutex_lock();
            }
        }
    }
    if(!bitmap_empty(new->polling.enable_sns, SENSOR_MAX)){
        period_fastest = reculc_micon_polling(new->polling.enable_sns);
        atomic_set(&g_fastest_poll_p, period_fastest);
        hrtimer_start(&micon_poll_timer, ktime_set(0, 0), HRTIMER_MODE_REL);
    } else {
        //no polling sensor.
        hrtimer_cancel(&micon_poll_timer);
        cancel_work_sync(&micon_work);
    }
    SENSOR_N_LOG("end");
    return SNS_RC_OK;
}

static void sns_confirm_polling_stop(void)
{
    int32_t i;

    for (i = 0; i < SENSOR_MAX; i++) {
        if (sns_ctrl_param.input_param.inputs[i].enable &&
            sns_ctrl_config[i].poll_info_p != NULL) {
            sensor_stoptimer(&(sns_ctrl_config[i].poll_info_p->timer));
            sensor_com_mutex_unlock();
            cancel_work_sync(&(sns_ctrl_config[i].poll_info_p->work));
            sensor_com_mutex_lock();
        }
    }
    if(!bitmap_empty(sns_ctrl_param.output_param.polling.enable_sns, SENSOR_MAX)){
        //read all sensor data polling stop.
        hrtimer_cancel(&micon_poll_timer);
        cancel_work_sync(&micon_work);
    }
}

static int32_t sns_update_micon_control_if_needed(bool force)
{
    struct sensor_ctrl_param_output_str output_old;
    struct sensor_ctrl_param_output_str output_new;
    int32_t ret = SNS_RC_OK;
    int32_t change_type;

    SENSOR_N_LOG("start");

    // save old output
    memcpy(&output_old, &sns_ctrl_param.output_param, sizeof(output_old));

    // make new output
    sns_make_sensor_ctrl_info(&sns_ctrl_param.input_param, &output_new);

    // update output_param
    memcpy(&sns_ctrl_param.output_param, &output_new, sizeof(sns_ctrl_param.output_param));

    // judge change type
    change_type = sns_judge_sensor_ctrl_change_type(
        (force ? NULL : &output_old), &output_new);

    if (change_type != CHANGE_TYPE_NONE) {
        if (change_type & CHANGE_TYPE_SENSOR_OFF) {
            if (change_type & CHANGE_TYPE_POLLING)
                ret = sns_ctrl_polling(&output_old, &output_new);
            if (change_type & CHANGE_TYPE_LOGGING)
                ret |= sns_micon_func.send_logging_state(&output_old, &output_new);
            if (change_type & CHANGE_TYPE_SENSOR)
                ret |= sns_micon_func.send_sensor_ctrl(&output_old, &output_new, force);
        } else {
            if (change_type & CHANGE_TYPE_SENSOR)
                ret = sns_micon_func.send_sensor_ctrl(&output_old, &output_new, force);
            if (change_type & CHANGE_TYPE_POLLING_OFF) {
                if (change_type & CHANGE_TYPE_LOGGING)
                    ret |= sns_micon_func.send_logging_state(&output_old, &output_new);
                if (change_type & CHANGE_TYPE_POLLING)
                    ret |= sns_ctrl_polling(&output_old, &output_new);
            } else {
                if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_POLLING))
                    ret = sns_ctrl_polling(&output_old, &output_new);
                if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_LOGGING))
                    ret = sns_micon_func.send_logging_state(&output_old, &output_new);
            }
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t sns_adjust_sampling_period(
    enum sensor_e_type type,
    int32_t period)
{
    int32_t ret;
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_ACC:
        ret = sns_adjust_value_by_range(period, PERIOD_ACC_MAX, PERIOD_ACC_MIN);
        break;
    case SENSOR_MAG:
    case SENSOR_MAG_UNCAL:
        ret = sns_adjust_value_by_range(period, PERIOD_MAG_MAX, PERIOD_MAG_MIN);
        break;
    case SENSOR_GYRO:
    case SENSOR_GYRO_UNCAL:
        ret = sns_adjust_value_by_range(period, PERIOD_GYRO_MAX, PERIOD_GYRO_MIN);
        break;
    case SENSOR_PRESSURE:
        ret = sns_adjust_value_by_range(period, PERIOD_PRESSURE_MAX, PERIOD_PRESSURE_MIN);
        break;
    case SENSOR_MAG_ROT_VCTR:
    case SENSOR_ACC_LNR:
    case SENSOR_GRV:
    case SENSOR_ORTN:
    case SENSOR_ROT_VCTR:
    case SENSOR_GAME_ROT_VCTR:
        ret = sns_adjust_value_by_range(period, PERIOD_FUSION_MAX, PERIOD_FUSION_MIN);
        break;
    default:
        ret = period;
        SENSOR_D_LOG("invalid type:%d ret:%d",type,ret);
        break;
    }

    SENSOR_N_LOG("end ret[%d]", ret);
    return ret;
}

static bool sns_set_input_param_batch(
    enum sensor_e_type type,
    struct sensor_batch_info_str *info)
{
    struct sensor_ctrl_param_input_one_str *dst =
        &sns_ctrl_param.input_param.inputs[type];
    struct sensor_ctrl_config_str *config =
        &sns_ctrl_config[type];
    bool ret;
    int32_t period;
    SENSOR_N_LOG("start");

    period = sns_adjust_sampling_period(type, info->period_ns);
    SENSOR_N_LOG("period[%d], info->period_ns[%d]", period, info->period_ns);

    if (dst->sampling_period_ms != period ||
        dst->max_report_latency_ms != info->timeout) {

        dst->sampling_period_ms = period;
        dst->max_report_latency_ms = info->timeout;
        ret = true;

        if (config->poll_info_p)
            atomic_set(&(config->poll_info_p->poll_time), period);
    } else {
        ret = false;
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static bool sns_set_input_param_enable(
    enum sensor_e_type type,
    bool enable)
{
    struct sensor_ctrl_param_input_one_str *dst =
        &sns_ctrl_param.input_param.inputs[type];
    bool ret;
    SENSOR_N_LOG("start");

    if (dst->enable != enable) {
        dst->enable = enable;
        ret = true;
    } else {
        ret = false;
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_set_batch(
    enum sensor_e_type type,
    struct sensor_batch_info_str *info)
{
    int32_t ret = 0;
    bool changed;
    SENSOR_D_LOG("start type[%d] period[%d] timeout[%d]", type, info->period_ns, info->timeout);

    changed = sns_set_input_param_batch(type, info);
    if (changed) {
        ret = sns_update_micon_control_if_needed(false);

        if ((ret == SNS_RC_OK) &&
            ((type == SENSOR_EXT_PEDO && g_micon_pedo_status) ||
             (type == SENSOR_EXT_BARO && g_micon_baro_status))) {
            ret = sns_micon_func.ds_func.send_latency_option(type, true);
        }
    }

    SENSOR_D_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_set_enable(enum sensor_e_type type, bool enable)
{
    int32_t ret = 0;
    bool changed;
    SENSOR_D_LOG("start type[%d] enable[%d]", type, enable);

    sns_micon_func.waiting_for_access_chk();
    changed = sns_set_input_param_enable(type, enable);
    if (changed) {
        ret = sns_update_micon_control_if_needed(false);
    }

    SENSOR_D_LOG("end - return[%d]",ret);
    return ret;
}

void sns_set_flush(enum sensor_e_type type, struct input_dev* dev)
{
    SENSOR_D_LOG("start type[%d]", type);
    sns_micon_func.set_flush(type);
    SENSOR_D_LOG("end");
}

void sns_get_logging_enable(unsigned long *logging_enable_status)
{
    bitmap_copy(logging_enable_status, sns_ctrl_param.output_param.logging.enable_sns, SENSOR_MAX);
    return;
}

int32_t sns_suspend()
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    sns_ctrl_param.input_param.in_suspend = true;
    ret = sns_update_micon_control_if_needed(false);

    sns_confirm_polling_stop();

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_resume(void)
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    sns_ctrl_param.input_param.in_suspend = false;
    ret = sns_update_micon_control_if_needed(false);

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_shutdown(void)
{
    bool changed = false;
    int32_t ret = SNS_RC_OK;
    int32_t i;
    SENSOR_N_LOG("start");

    for (i = 0; i < SENSOR_MAX; i++) {
        changed |= sns_set_input_param_enable(i, false);
    }
    if (changed) {
        ret = sns_update_micon_control_if_needed(false);
    }

    SENSOR_N_LOG("end - changed[%d] return[%d]",changed, ret);
    return ret;
}

int32_t sns_flush_event_buffer(void)
{
    return sns_micon_func.ds_func.event_mark_off();
}

int32_t sns_ts_set_nrfilt_param(struct nr_filt_prm prm)
{
    return sns_micon_func.app_cmn.ts_set_nrfilt_prm(prm);
}

int32_t sns_ts_get_nrfilt_param(struct nr_filt_prm *prm)
{
    return sns_micon_func.app_cmn.ts_get_nrfilt_prm(prm);
}

int32_t sns_acc_read_data(struct acceleration *arg_Acc)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_ACC, arg_Acc);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_acc_set_offset(int32_t* offsets)
{
    return sns_micon_func.acc_func.set_offsets(offsets);
}

int32_t sns_acc_get_offset(int32_t* offsets)
{
    return sns_micon_func.acc_func.get_offsets(offsets);
}

int32_t sns_acc_set_auto_cal_offset(struct acceleration offsets)
{
    SENSOR_D_LOG("offset_x:%d, offset_y:%d, offset_z:%d",
                    offsets.nX, offsets.nY, offsets.nZ);
    return sns_micon_func.acc_func.set_ac_offsets(offsets);
}

int32_t sns_acc_get_auto_cal_offset(struct acceleration* offsets)
{
    int32_t ret = SNS_RC_OK;
    ret = sns_micon_func.acc_func.get_ac_offsets(offsets);
    SENSOR_D_LOG("offset_x:%d, offset_y:%d, offset_z:%d",
                    offsets->nX, offsets->nY, offsets->nZ);
    return ret;
}

int32_t sns_acc_set_nrfilt_param(struct nr_filt_prm prm)
{
    return sns_micon_func.acc_func.set_nrfilt_prm(prm);
}

int32_t sns_acc_get_nrfilt_param(struct nr_filt_prm *prm)
{
    return sns_micon_func.acc_func.get_nrfilt_prm(prm);
}

void sns_acc_set_ts_nrfilt_baseinfo(int32_t base_val)
{
    sns_micon_func.acc_func.set_ts_nrfilt_info(base_val);
}

uint8_t sns_acc_get_deviceid(void)
{
    int8_t ret;
    ret = sns_micon_func.acc_func.get_devid();
    SENSOR_D_LOG("accelerometer device id[0x%02x]", ret);
    return ret;
}

int32_t sns_acc_set_dynamiccalib(bool enable)
{
    return sns_micon_func.acc_func.set_dcalib(enable);
}

int8_t sns_acc_set_selfchk_config(enum selfchk_test_e_type type)
{
    int8_t ret;
    if (type == SELFCHK_PRE){
        ret = sns_micon_func.acc_func.set_selfchk_precfg();
    } else {
        ret = sns_micon_func.acc_func.set_selfchk_cfg((uint8_t)type);
    }
    return ret;
}

void sns_dailys_set_vib_interlocking(int32_t mode)
{
    int8_t mode_vib = INTERLOCKING_OFF;
    int8_t mode_spk = INTERLOCKING_OFF;
    int8_t interlocking = INTERLOCKING_OFF;
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    struct sensor_ctrl_param_output_str param = now_ctrl_param.output_param;

    SENSOR_D_LOG("start");

    SENSOR_D_LOG("mode : %d", mode);
    if (mode & VIB_INTERLOCKING_ON)      mode_vib = INTERLOCKING_ON;
    if (mode & SPEAKER_INTERLOCKING_ON)  mode_spk = INTERLOCKING_ON;
    interlocking = mode_vib | mode_spk;

    if (g_interlocking != mode && !(IS_ANDROID_EN(param.enable_type.acc_en) || IS_FUSION_EN(param.enable_type.acc_en))) {
        sns_micon_func.ds_func.vib_interlocking(mode, interlocking);
        g_interlocking = mode;
    }
    SENSOR_D_LOG("end");
}

void sns_dailys_get_event_markoff_timestamps(int64_t *cur_kernel_time, int32_t *cur_micontask_time)
{
    sns_micon_func.ds_func.get_mark_off_times(cur_kernel_time, cur_micontask_time);
}

void sns_dailys_get_event_markoff_clrinfo(uint8_t *is_clr_exist, int32_t *clr_miconts)
{
    sns_micon_func.ds_func.get_mark_off_clrinfo(is_clr_exist, clr_miconts);
}

int32_t sns_mag_read_data(struct geomagnetic *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_MAG, arg_Mag);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_mag_set_offset(struct geomagnetic offsets)
{
    SENSOR_D_LOG("offset_x:%d, offset_y:%d, offset_z:%d, accuracy:%d",
                    offsets.x, offsets.y, offsets.z, offsets.accuracy);
    return sns_micon_func.mag_func.set_offsets(offsets);
}

int32_t sns_mag_get_offset(struct geomagnetic* offsets)
{
    int32_t ret = SNS_RC_OK;
    ret = sns_micon_func.mag_func.get_offsets(offsets);
    SENSOR_D_LOG("offset_x:%d, offset_y:%d, offset_z:%d, accuracy:%d",
                    offsets->x, offsets->y, offsets->z, offsets->accuracy);
    return ret;
}

int32_t sns_mag_set_accuracy(int8_t accuracy)
{
    int32_t    ret = SNS_RC_OK;
    struct geomagnetic offsets = {0};

    SENSOR_N_LOG("start");

    ret = sns_mag_get_offset(&offsets);
    offsets.accuracy = accuracy;
    sns_mag_set_offset(offsets);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_get_accuracy(int8_t* accuracy)
{
    int32_t ret = SNS_RC_OK;
    struct geomagnetic offsets;
    ret = sns_mag_get_offset(&offsets);
    *accuracy = offsets.accuracy;
    return ret;

}

int32_t sns_mag_get_static_matrix(int32_t* static_matrix)
{
    return sns_micon_func.mag_func.get_static_matrix(static_matrix);
}

int32_t sns_mag_set_static_matrix(int32_t* static_matrix)
{
    return sns_micon_func.mag_func.set_static_matrix(static_matrix);
}

uint8_t sns_mag_get_deviceid(void)
{
    uint8_t ret;
    ret = sns_micon_func.mag_func.get_devid();
    SENSOR_D_LOG("magnetometer device id[0x%02x]", ret);
    return ret;
}

int32_t sns_mag_set_dynamiccalib(bool enable)
{
    return sns_micon_func.mag_func.set_dcalib(enable);
}

int32_t sns_gyro_read_data(struct gyroscope *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_GYRO, arg_Gyro);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_gyro_set_offset(struct gyroscope offsets)
{
    SENSOR_D_LOG("offset_x:%d, offset_y:%d, offset_z:%d",
                    offsets.x, offsets.y, offsets.z);
    return sns_micon_func.gyro_func.set_offsets(offsets);
}

int32_t sns_gyro_get_offset(struct gyroscope* offsets)
{
    int32_t ret = SNS_RC_OK;
    ret = sns_micon_func.gyro_func.get_offsets(offsets);
    SENSOR_ERR_LOG("offset_x:%d, offset_y:%d, offset_z:%d",
                    offsets->x, offsets->y, offsets->z);
    return ret;
}

int32_t sns_gyro_set_nrfilt_param(struct nr_filt_prm prm)
{
    return sns_micon_func.gyro_func.set_nrfilt_prm(prm);
}

int32_t sns_gyro_get_nrfilt_param(struct nr_filt_prm *prm)
{
    return sns_micon_func.gyro_func.get_nrfilt_prm(prm);
}

void sns_gyro_set_ts_nrfilt_baseinfo(int32_t base_val)
{
    sns_micon_func.gyro_func.set_ts_nrfilt_info(base_val);
}

void sns_gyrouc_set_ts_nrfilt_baseinfo(int32_t base_val)
{
    sns_micon_func.gyrouc_func.set_ts_nrfilt_info(base_val);
}

uint8_t sns_gyro_get_deviceid(void)
{
    uint8_t ret;
    ret = sns_micon_func.gyro_func.get_devid();
    SENSOR_D_LOG("gyroscope device id[0x%02x]", ret);
    return ret;
}

int32_t sns_gyro_set_dynamiccalib(bool enable)
{
    return sns_micon_func.gyro_func.set_dcalib(enable);
}

int8_t sns_gyro_set_selfchk_config(enum selfchk_test_e_type type)
{
    int8_t ret;
    if(type == SELFCHK_PRE){
        ret = sns_micon_func.gyro_func.set_selfchk_precfg();
    } else {
        ret = sns_micon_func.gyro_func.set_selfchk_cfg((uint8_t)type);
    }
    return ret;
}

int8_t sns_gyro_start_manual_cal(uint8_t *cal_prm)
{
    return sns_micon_func.gyro_func.start_man_calib(cal_prm);
}

int8_t sns_gyro_wait_cal_result(void)
{
    return sns_micon_func.gyro_func.wait_man_calib();
}

int32_t sns_maguncalib_read_data(struct mag_uncalib *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_MAG_UNCAL, arg_Mag);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_GYRO_UNCAL, arg_Gyro);
    SENSOR_D_LOG("end");
    return ret;
}

void sns_mag_offset_read_data(struct geomagnetic *arg_Mag)
{

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end - x[%04x] y[%04x] z[%04x]",arg_Mag->x ,arg_Mag->y, arg_Mag->z);
}

void sns_gyro_offset_read_data(struct gyroscope *arg_Gyro)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end - x[%04x] y[%04x] z[%04x]",arg_Gyro->x ,arg_Gyro->y, arg_Gyro->z);
}

int32_t sns_gravity_read_data(struct gravity *arg_Gravity)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_GRV, arg_Gravity);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_linacc_read_data(struct linear_acceleration *arg_linacc)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_ACC_LNR, arg_linacc);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_ori_read_data(struct orientation *arg_ori)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_ORTN, arg_ori);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_rota_read_data(struct rotation_vector *arg_rota)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_ROT_VCTR, arg_rota);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_gamerota_read_data(struct game_rotation_vector *arg_gamerota)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_GAME_ROT_VCTR, arg_gamerota);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_magrota_read_data(struct geomagnetic_rotation_vector *arg_magrota)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_MAG_ROT_VCTR, arg_magrota);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_pressure_read_data(struct pressure *arg_pressure)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    sns_micon_func.get_sns_value(SENSOR_PRESSURE, arg_pressure);
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sns_pressure_base_val(bool req, DailysSetBaseParam *DailysBaseParam)
{
    return SNS_RC_OK;
}

int32_t sns_pressure_set_param(struct pressure *arg_pressure)
{
    return SNS_RC_OK;
}
int32_t sns_pressure_get_param(struct pressure *arg_pressure)
{
    return SNS_RC_OK;
}

int32_t sns_pressure_set_cal_ofs(struct pressure offset)
{
    atomic_set(&g_pressure_cal_offset, offset.pressure);
    return SNS_RC_OK;
}

uint8_t sns_pressure_get_deviceid(void)
{
    uint8_t ret;
    ret = sns_micon_func.press_func.get_devid();
    SENSOR_D_LOG("gyroscope device id[0x%02x]", ret);
    return ret;

}

int32_t sns_pressure_get_cal_ofs(struct pressure *offset)
{
    offset->pressure = atomic_read(&g_pressure_cal_offset);
    return SNS_RC_OK;
}

int32_t sns_motion_start(bool arg_iStart)
{
    return sns_micon_func.sgnfcnt_func.enable(arg_iStart);
}

void sns_set_pedo_param(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iVehiType)
{
    SENSOR_D_LOG("start");

    sns_pedo_param.weight       = arg_iWeight;
    sns_pedo_param.step_wide    = arg_iStepWide;
    sns_pedo_param.vehi_type    = arg_iVehiType;

    if (g_micon_pedo_status) {
        sns_micon_func.ds_func.send_ope_param(true, sns_pedo_param);
    }
    SENSOR_D_LOG("end - Weight[%d] StepWide[%d] VehiType[%d]",
                    sns_pedo_param.weight,
                    sns_pedo_param.step_wide,
                    sns_pedo_param.vehi_type);
}

int32_t sns_dailys_start(bool arg_iStart)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_D_LOG("start arg_iStart[%d]", arg_iStart);
    ret = sns_micon_func.ds_func.enable(arg_iStart, sns_pedo_param);
    g_micon_pedo_status = arg_iStart;
    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_baro_start(bool arg_iStart)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_D_LOG("start arg_iStart[%d]", arg_iStart);
    ret = sns_micon_func.baro_func.enable(arg_iStart);
    g_micon_baro_status = arg_iStart;
    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_pedo_data(struct pedometer *arg_Pedo)
{
    return sns_micon_func.ds_func.get_pedo_data(arg_Pedo);
}

int32_t sns_get_vehi_data(struct vehicle *arg_Vehi)
{
    return sns_micon_func.ds_func.get_vehi_data(arg_Vehi);
}

int32_t sns_iwifi_start(bool arg_iStart)
{
    int32_t    ret = SNS_RC_OK;

    return ret;
}

int32_t sns_get_iwifi_data(struct iwifi *arg_IWifi)
{
    int32_t ret = SNS_RC_OK;

    return ret;
}

int32_t sns_iwifi_set_info(bool req, DailysSetIWifiParam *DailysIWifiParam)
{
    return SNS_RC_OK;
}

int32_t sns_pedom_clear(int32_t clear_req)
{
    return sns_micon_func.app_cmn.clear(clear_req);
}

int32_t sns_vhdetect_start(bool arg_iStart)
{
    int32_t ret = SNS_RC_OK;
    if(arg_iStart == false){
        g_AppVhFirstReportDone = false;
    }
    ret = sns_micon_func.vhdet_func.enable(arg_iStart);
    return ret;
}

int32_t sns_get_vhdetect_data(struct vhdetect *arg_VHdetect)
{
    return sns_micon_func.vhdet_func.get_data(arg_VHdetect);
}

bool sns_is_vhdetect_first_report_done(void)
{
    return g_AppVhFirstReportDone;
}

int32_t sns_motion_detect_start(bool arg_iStart)
{
    int32_t    ret = SNS_RC_OK;

    return ret;
}

int32_t sns_stationary_detect_start(bool arg_iStart)
{
    int32_t    ret = SNS_RC_OK;

    return ret;
}

int32_t sns_kc_motion_walk_start_start(bool arg_iStart)
{
    return sns_micon_func.kcmot_func.wstart_enable(arg_iStart);
}

int32_t sns_kc_motion_walk_stop_start(bool arg_iStart)
{
    return sns_micon_func.kcmot_func.wstop_enable(arg_iStart);
}

int32_t sns_get_kc_motion_walk_data(struct kc_motion_walk_data *arg_Walk)
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    if( arg_Walk ) {
        arg_Walk->status = 0;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_train_start(bool arg_iStart)
{
    return sns_micon_func.kcmot_func.train_enable(arg_iStart);
}

int32_t sns_get_kc_motion_train_data(struct kc_motion_train *arg_Train)
{
    return sns_micon_func.kcmot_func.train_get_data(arg_Train);
}

int32_t sns_kc_motion_vehicle_start(bool arg_iStart)
{
    return sns_micon_func.kcmot_func.other_vehi_enable(arg_iStart);
}

int32_t sns_kc_motion_bringup_start(bool arg_iStart)
{
    return sns_micon_func.kcmot_func.bringup_enable(arg_iStart);
}

int32_t sns_get_kc_motion_bringup_data(struct kc_motion_bringup_data *arg_Bringup)
{
    return sns_micon_func.kcmot_func.bringup_get_data(arg_Bringup);
}


int32_t sns_uwater_start(bool arg_iStart)
{
    return sns_micon_func.uw_func.enable(arg_iStart);
}

int32_t sns_get_uwater_data(struct uwater_detect_info *arg_uwdInfo)
{
    int32_t ret = SNS_RC_OK;

    return ret;
}

int32_t sns_uwater_clear(void)
{
    int32_t ret = SNS_RC_OK;

    return ret;
}

int32_t sns_voice_trigger_start(bool arg_iStart)
{
	return sns_micon_func.vt_func.enable(arg_iStart);
}

int32_t sns_allapp_restart(void)
{
    int32_t status = 0;
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");


#ifdef CONFIG_INPUT_SENSOR_EXT
    status |= sensor_get_status(SENSOR_EXT_PEDO);
    status |= sensor_get_status(SENSOR_EXT_VEHI);
    if(status){
        SENSOR_ERR_LOG("Restart DailyStep");
        ret = sns_dailys_start(true);
        sensor_restore_status(SENSOR_EXT_PEDO, NULL);
    }
    status = 0;

    status = sensor_get_status(SENSOR_EXT_BARO);
    if(status){
        SENSOR_ERR_LOG("Restart Barometer app");
        ret = sns_baro_start(true);
    }
    status = 0;
#endif

#ifdef CONFIG_INPUT_SENSOR_SIGNIFICANT_MOTION
    status = sensor_get_status(SENSOR_SGNFCNT_MTN);
    if(status){
        SENSOR_ERR_LOG("Restart SignificantMotion");
        ret = sns_motion_start(true);
    }
    status = 0;
#endif

#if defined(CONFIG_INPUT_SENSOR_DEVICE_ORIENTATION) || defined(CONFIG_INPUT_SENSOR_EXT_VH)
    status |= sensor_get_status(SENSOR_EXT_VH);
    status |= sensor_get_status(SENSOR_DEVICE_ORIENTATION);
    if(status){
        SENSOR_ERR_LOG("Restart Device Orientation (vhdet)");
        ret = sns_vhdetect_start(true);
    }
    status = 0;
#endif

#ifdef CONFIG_INPUT_SENSOR_MOTION_DETECT
    status = sensor_get_status(SENSOR_MOTION_DETECT);
    if(status){
        SENSOR_ERR_LOG("Restart MotionDetect");
        ret = sns_motion_detect_start(true);
    }
    status = 0;
#endif

#ifdef CONFIG_INPUT_SENSOR_STATIONARY_DETECT
    status = sensor_get_status(SENSOR_STATIONARY_DETECT);
    if(status){
        SENSOR_ERR_LOG("Restart StationaryDetect");
        ret = sns_stationary_detect_start(true);
    }
    status = 0;
#endif

#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
    status = sensor_get_status(SENSOR_KC_MOTION_WALK_START);
    if(status){
        SENSOR_ERR_LOG("Restart KCMotionWalkStart");
        ret = sns_kc_motion_walk_start_start(true);
    }
    status = 0;

    status = sensor_get_status(SENSOR_KC_MOTION_WALK_STOP);
    if(status){
        SENSOR_ERR_LOG("Restart KCMotionWalkStop");
        ret = sns_kc_motion_walk_stop_start(true);
    }
    status = 0;

    status = sensor_get_status(SENSOR_KC_MOTION_TRAIN);
    if(status){
        SENSOR_ERR_LOG("Restart KCMotionWalkTrain");
        ret = sns_kc_motion_train_start(true);
    }
    status = 0;

    status = sensor_get_status(SENSOR_KC_MOTION_VEHICLE);
    if(status){
        SENSOR_ERR_LOG("Restart KCMotionWalkOtherVehicle");
        ret = sns_kc_motion_vehicle_start(true);
    }
    status = 0;

    status = sensor_get_status(SENSOR_KC_MOTION_BRINGUP);
    if(status){
        SENSOR_ERR_LOG("Restart KCMotionBringUp");
        ret = sns_kc_motion_bringup_start(true);
    }
    status = 0;
#endif

#ifdef CONFIG_INPUT_SENSOR_UNDERWATER_DETECT
    status = sensor_get_status(SENSOR_UNDERWATER_DETECT);
    if(status){
        SENSOR_ERR_LOG("Restart UnderWaterDetect");
        ret = sns_uwater_start(true);
    }
    status = 0;
#endif

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t sns_androidsensor_restart(void)
{
    int32_t ret;
    SENSOR_D_LOG("start");
    sns_micon_func.clr_batch_condition();
    ret = sns_update_micon_control_if_needed(true);
    SENSOR_D_LOG("end");
    return ret;
}

void dig_mic_on(void)
{
#ifdef FUNCTION_VT_ENABLE
    gpio_set_value(34, 1);
    usleep_range(3000,5000);
#endif
}

int32_t sns_set_dev_param(void)
{
    return sns_micon_func.set_dev_param();
}

static int32_t sns_get_micon_info_dt( struct spi_device *client )
{
    return sns_micon_func.get_micon_info(client);
}

void sns_invalid_setting(uint8_t acc_available, uint8_t mag_availble,
                         uint8_t gyro_available, uint8_t press_available)
{
    int32_t i = 0;
    SENSOR_D_LOG("start");
    for(i = 0; i < ARRAY_SIZE(sns_ctrl_config); i++) {
        if( !acc_available ){
            sns_ctrl_config[i].en_bit_measure &= ~EN_BIT_ACC;
            sns_ctrl_config[i].en_bit_logging &= ~EN_BIT_ACC;
        }
        if( !press_available ){
            sns_ctrl_config[i].en_bit_measure &= ~EN_BIT_PRESSURE;
            sns_ctrl_config[i].en_bit_logging &= ~EN_BIT_PRESSURE;
        }
        if( !mag_availble ){
            sns_ctrl_config[i].en_bit_measure &= ~EN_BIT_MAG;
            sns_ctrl_config[i].en_bit_logging &= ~EN_BIT_MAG;
        }
        if( !gyro_available ){
            sns_ctrl_config[i].en_bit_measure &= ~EN_BIT_GYRO;
            sns_ctrl_config[i].en_bit_logging &= ~EN_BIT_GYRO;
        }
    }
    SENSOR_D_LOG("end");
}

int32_t sns_initialize( void )
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    g_bDevIF_Error = false;
    g_micon_error = false;

    ret = sns_micon_func.init_snsdriver();
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Failed init_snsdriver[%d]",ret);
        ret = sns_micon_func.init_snsdriver();
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("Failed init_snsdriver[%d]",ret);
            g_micon_error = true;
            return ret;
        }
    }

    SENSOR_N_LOG("end - %d", ret);

    return ret;
}

static bool sns_devif_error_check(void)
{
    if(g_bDevIF_Error){
        SENSOR_ERR_LOG("g_bDevIF_Error[%d]",g_bDevIF_Error);
    }
    else{
        SENSOR_N_LOG("g_bDevIF_Error[%d]",g_bDevIF_Error);
    }
    return g_bDevIF_Error;
}

static void sns_pre_power_on(void)
{
    int32_t ret = SNS_RC_OK;
    int32_t cnt = 0;

    SENSOR_ERR_LOG("start");

    if(sns_devif_error_check() == false){
        SENSOR_ERR_LOG("Other Sensor Error");

        while(1){
            ret = sns_micon_func.enable_i2c_peripheral(true);
            cnt++;
            if(ret == SNS_RC_OK || cnt >= MICON_I2C_ENABLE_RETRY_NUM){
                SENSOR_ERR_LOG("i2c enable cnt[%d]",cnt);
                break;
            }
            msleep(10);
        }

        ret |= sns_micon_func.set_sns_initcmd();
        ret |= sns_micon_func.set_dev_param();
    } else {
        SENSOR_ERR_LOG("Micon SPI Error");
        ret |= sns_initialize();
        if(ret != SNS_RC_OK){
            SENSOR_ERR_LOG("sns_initialize ret[%d]",ret);
            atomic_set(&g_ResetStatus,false);
            return;
        }
        ret |= sns_micon_func.set_dev_param();
    }

    if( true == g_reset_param_status) {
        ret = sns_reset_restore_param();
        sensor_reset_resume();
        g_reset_param_status = false;
    }

    atomic_set(&g_ResetStatus,false);

    SENSOR_ERR_LOG("end");
}

static void sns_pre_power_off(void)
{
    SENSOR_N_LOG("start");

    if(sns_micon_func.get_miconshtdwn_status() == false){
        atomic_set(&g_ResetStatus,true);
        g_reset_param_status = false;
        if(sns_devif_error_check() == false){
            sensor_reset();
            sns_reset_save_param();
        }
        g_reset_param_status = true;
        if(sns_devif_error_check() == false){
            SENSOR_ERR_LOG("Other Sensor Error");
            sns_micon_func.enable_i2c_peripheral(false);
        } else {
            SENSOR_ERR_LOG("Micon SPI Error");
        }
    }

    SENSOR_N_LOG("end");
}

static void micon_poll_work_func(struct work_struct *work)
{
    SENSOR_D_LOG("start");
    sns_micon_func.read_all_sns_val();
    SENSOR_D_LOG("end");
}

static int get_fastest_polling_period(void)
{
    return atomic_read(&g_fastest_poll_p);
}

enum hrtimer_restart micon_poll(struct hrtimer *micon_timer)
{
    int delay_ms = get_fastest_polling_period();
    SENSOR_D_LOG("start");
    queue_work(micon_poll_wq, &micon_work);
    hrtimer_forward(micon_timer, hrtimer_get_expires(micon_timer), ms_to_ktime(delay_ms));
    SENSOR_D_LOG("end");
    return HRTIMER_RESTART;
}

static void sns_micon_init(void)
{
    SENSOR_D_LOG("start");
    //initialization of atomic variables
    atomic_set(&g_pressure_cal_offset,0);

    //initialization of mutex variables
    mutex_init(&sensor_fifo_mutex);

    //initialization of global variables
    g_micon_pedo_status = false;
    g_micon_baro_status = false;
    g_interlocking = 0x00;
    g_AppVhFirstReportDone = false;
    sns_pedo_param.weight = DEFAULT_WEIGHT;
    sns_pedo_param.step_wide = DEFAULT_STEPWIDE;
    sns_pedo_param.vehi_type = DEFAULT_VEHITYPE;

    //initialization of hrtimer
    SENSOR_D_LOG("hrtimer init setting start");
    hrtimer_init(&micon_poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    micon_poll_timer.function = micon_poll;
    micon_poll_wq = create_workqueue("micon_poll_wq");
    INIT_WORK(&micon_work, micon_poll_work_func);
    SENSOR_D_LOG("hrtimer init setting end");

    SENSOR_D_LOG("end");
}

static void sns_micon_func_register(void)
{
    SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ML630Q790
    func_register_ml630q790(&sns_micon_func);
#endif
#ifdef CONFIG_INPUT_SENSOR_EOSS3
    SENSOR_D_LOG("register functions for EOSS3");
    func_register_eoss3(&sns_micon_func);
#endif
    SENSOR_D_LOG("end");
}

static int32_t sns_reset_save_param(void)
{
    return sns_micon_func.reset_saveparam_and_taskoff();
}

static int32_t sns_reset_restore_param(void)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_ERR_LOG("restart Android sensor.");
    ret = sns_androidsensor_restart();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("restore failure. Android sensor restart.");
        goto exit_restore;
    }

    SENSOR_ERR_LOG("restart sensor app.");
    ret = sns_allapp_restart();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("restore failure. sensor application restart.");
    }
exit_restore:
    return ret;
}

int32_t sns_err_check(void)
{
    int ret_val = 0;

    if ((sns_devif_error_check() != false) && (atomic_read(&g_ResetStatus) == false) &&
        g_micon_error == false) {
        SENSOR_ERR_LOG("power reset!");
#ifdef CONFIG_INPUT_SENSOR_EOSS3
        ql_spi_power_reset();
#endif
#ifdef CONFIG_INPUT_SENSOR_ML630Q790
        sensor_power_reset(SENSOR_INDEX_ACC);
#endif
        ret_val = -ECOMM;
    } else {
        ret_val = 0;
    }

    return ret_val;
}

int32_t sns_get_reset_status(void)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return atomic_read(&g_ResetStatus);
}

void sns_enable_irq_wake_irq(bool arg_iEnable)
{
    SENSOR_N_LOG("start");
    sns_micon_func.enable_irq_wake(arg_iEnable);
    SENSOR_N_LOG("end");
}

void sns_set_buff_int(bool arg_iEnable)
{
    sns_micon_func.set_batch_notify(arg_iEnable);
}

void sns_load_offset_value(enum sensor_e_type type, void *outdata)
{
    sensor_load_offset_value(type, outdata);
}

void sns_save_offset_value(enum sensor_e_type type, void *savedata)
{
    sensor_save_offset_value(type, savedata);
}


void sns_notify(enum sensor_e_type type, uint32_t data)
{
    sensor_notify(type, data);
}

void sns_pm_wakeup_event(uint32_t timeout_msec)
{
    SENSOR_D_LOG("start [timeout:%d]", timeout_msec);
    pm_wakeup_event(&client_sns->dev, timeout_msec);
    SENSOR_D_LOG("end");
}


static void __sensor_exec_flush(enum sensor_e_type type)
{
	SENSOR_D_LOG("start");
	sns_logging_punctuation(LOGGING_TRIGGER_FLUSH);
	sns_iio_report_flush(type, 0);
    SENSOR_D_LOG("end");
}

static void __sensor_irq_proc(uint64_t result)
{
    SENSOR_D_LOG("start result = 0x%016llx\n", result);
    sns_micon_func.irq_proc(result);
    SENSOR_D_LOG("end");
}

static int32_t __sensor_dev_resume( struct device *dev )
{
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_N_LOG("start");

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    sensor_resume();
    sns_micon_func.set_sensdrv_status(SENSOR_RESUME);

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_N_LOG("end");
    return 0;
}

static int32_t __sensor_dev_suspend( struct device *dev )
{
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_N_LOG("start");

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    sensor_suspend();
    sns_micon_func.set_sensdrv_status(SENSOR_SUSPEND);

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_N_LOG("end");
    return 0;
}

static void __sensor_dev_shutdown( struct spi_device *client )
{
    SENSOR_N_LOG("start");
    sensor_shutdown();
    sns_micon_func.set_sensdrv_status(SENSOR_SHUTDOWN);
    device_init_wakeup(&client->dev, 0);
    SENSOR_N_LOG("end");
    return;
}

static int32_t __sensor_dev_remove( struct spi_device *client )
{
    SENSOR_N_LOG("start");
    device_init_wakeup(&client->dev, 0);
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t __sensor_dev_init( struct spi_device *client )
{
    int32_t ret = SNS_RC_OK;

    SENSOR_ERR_LOG("start");

    client_sns = client;
    ret = sns_initialize();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Failed sensor initialize[%d]",ret);
    }

    ret = sns_get_micon_info_dt(client);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Failed sns_get_micon_info_dt[%d]",ret);
        return -ENODEV;
    }
    dig_mic_on();
    ret = sns_set_dev_param();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Failed sns_set_dev_param[%d]",ret);
    }

    sns_pre_power_cb.power_on  = sns_pre_power_on;
    sns_pre_power_cb.power_off = sns_pre_power_off;
#ifdef CONFIG_INPUT_SENSOR_EOSS3
    qlpower_reg_cbfunc(&sns_pre_power_cb);
#endif
#ifdef CONFIG_INPUT_SENSOR_ML630Q790
    sensor_power_reg_cbfunc(&sns_pre_power_cb);
#endif
    sns_micon_func.set_sensdrv_status(SENSOR_RESUME);
    sensor_drv_set_status(SENSOR_RESUME);
    device_init_wakeup(&client->dev, 1);
    SENSOR_ERR_LOG("end");

    return 0;
}

static int32_t __sensor_dev_save_param( void )
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");
    ret = sns_reset_save_param();
    if (ret != SNS_RC_OK) {
        save_status = ret;
        SENSOR_ERR_LOG("Failed to save data.[%d]", ret);
    }
    SENSOR_N_LOG("end");
    return ret;
}

static int32_t __sensor_dev_load_param( void )
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");
    if (save_status == SNS_RC_OK) {
        ret = sns_reset_restore_param();
    } else {
        ret = SNS_RC_ERR;
        SENSOR_ERR_LOG("Save data is invalid.");
    }
    SENSOR_N_LOG("end");
    return ret;
}

void sensor_exec_flush(int32_t type)
{
	__sensor_exec_flush(type);
}
int sensor_dev_init(struct spi_device *client)
{
	return __sensor_dev_init(client);
}
int sensor_dev_remove(struct spi_device *client)
{
	return __sensor_dev_remove(client);
}
int sensor_dev_suspend(struct device *dev)
{
	return __sensor_dev_suspend(dev);
}
int sensor_dev_resume(struct device *dev)
{
	return __sensor_dev_resume(dev);
}
void sensor_dev_shutdown(struct spi_device *client)
{
	__sensor_dev_shutdown(client);
}
int32_t sensor_dev_save_param(void)
{
	return __sensor_dev_save_param();
}
int32_t sensor_dev_load_param(void)
{
	return __sensor_dev_load_param();
}
void sensor_irq_proc(uint64_t result){
    __sensor_irq_proc(result);
}

int32_t sensor_micon_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    sns_micon_func_register();
    ret = sns_iio_init();
    if(ret != 0){
        SENSOR_ERR_LOG("fail:sns_iio_init");
        goto IIO_INIT_ERR;
    }
    sns_micon_init();
    bitmap_zero(fusion_sns_mask, SENSOR_MAX);

    set_bit(SENSOR_MAG, fusion_sns_mask);
    set_bit(SENSOR_MAG_UNCAL, fusion_sns_mask);
    set_bit(SENSOR_MAG_ROT_VCTR, fusion_sns_mask);
    set_bit(SENSOR_ACC_LNR, fusion_sns_mask);
    set_bit(SENSOR_GRV, fusion_sns_mask);
    set_bit(SENSOR_GYRO, fusion_sns_mask);
    set_bit(SENSOR_GYRO_UNCAL, fusion_sns_mask);
    set_bit(SENSOR_ORTN, fusion_sns_mask);
    set_bit(SENSOR_ROT_VCTR, fusion_sns_mask);
    set_bit(SENSOR_GAME_ROT_VCTR, fusion_sns_mask);

    SENSOR_N_LOG("end");
    return SNS_RC_OK;

IIO_INIT_ERR:
#ifdef CONFIG_INPUT_SENSOR_ML630Q790
    //spi_unregister_driver(&sensor_micon_driver);
#endif /* CONFIG_INPUT_SENSOR_ML630Q790 */

    return SNS_RC_ERR;
}
void sensor_micon_exit(void)
{
    SENSOR_N_LOG("start");
    sns_iio_exit();
    SENSOR_N_LOG("end");
    return;
}


int sns_iio_report_event(uint8_t id, uint8_t *data, uint32_t len, int64_t timestamp, int64_t offset)
{
    u8 event[1+EVENT_DATA_SIZE+16];/* Sensor HAL uses fixed 32 bytes */
    SENSOR_N_LOG("start id[%d] len[%d] time[%lld] ofs[%lld]", id, len, timestamp, offset);

    if (!g_iio_dev || len > EVENT_DATA_SIZE) {
        SENSOR_ERR_LOG("param error id[%d] len[%d] time[%lld]", id, len, timestamp);
        return 0;
    }

    event[0] = id;
    memcpy(&event[1], data, len);
    memset(&event[1+len], 0, EVENT_DATA_SIZE-len);
    memcpy(&event[EVENT_DATA_SIZE+1], &timestamp, sizeof(int64_t));
    memcpy(&event[EVENT_DATA_SIZE+1+sizeof(int64_t)], &offset, sizeof(int64_t));

    mutex_lock(&g_iio_mutex);
    iio_push_to_buffers(g_iio_dev, event);
    mutex_unlock(&g_iio_mutex);
    SENSOR_N_LOG("end");

    return 0;
}

bool sns_is_needed_gyro_sync(void)
{
    return sns_ctrl_param.input_param.inputs[SENSOR_GYRO].sampling_period_ms ==
                sns_ctrl_param.input_param.inputs[SENSOR_GYRO_UNCAL].sampling_period_ms &&
                test_bit(SENSOR_GYRO, sns_ctrl_param.output_param.polling.enable_sns) &&
                test_bit(SENSOR_GYRO_UNCAL, sns_ctrl_param.output_param.polling.enable_sns);
}

int sns_iio_report_event_now(enum sensor_e_type type)
{
    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start type[%d]", type);

    ret = sns_micon_func.iio_report_event_now(type);

    SENSOR_D_LOG("end");
    return ret;
}

static int sns_iio_report_flush(enum sensor_e_type type, int64_t timestamp)
{
    int ret;
    uint8_t id = sns_micon_func.get_snstype2smid(type);
    SENSOR_N_LOG("start");
    ret = sns_iio_report_event(SHID_META_DATA, &id, 1, timestamp, 0);
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

void sns_logging_setup_basetime(unsigned long *logging_new, unsigned long *logging_old)
{
    int64_t now = GET_CURRENT_TIMESTAMP_NS();
    DECLARE_BITMAP(sensors_started, SENSOR_MAX);
    int i;
    uint8_t idx;

    SENSOR_N_LOG("start");

    bitmap_andnot(sensors_started, logging_new, logging_old, SENSOR_MAX);
    if (test_bit(SENSOR_MAG, logging_old) || test_bit(SENSOR_MAG_UNCAL, logging_old)) {
        clear_bit(SENSOR_MAG, sensors_started);
        clear_bit(SENSOR_MAG_UNCAL, sensors_started);
    }
    if (test_bit(SENSOR_GYRO, logging_old) || test_bit(SENSOR_GYRO_UNCAL, logging_old)) {
        clear_bit(SENSOR_GYRO, sensors_started);
        clear_bit(SENSOR_GYRO_UNCAL, sensors_started);
    }
    if (test_bit(SENSOR_STEP_CNT, logging_old) || test_bit(SENSOR_STEP_DTC, logging_old)) {
        clear_bit(SENSOR_STEP_CNT, sensors_started);
        clear_bit(SENSOR_STEP_DTC, sensors_started);
    }

    if (bitmap_empty(sensors_started, SENSOR_MAX)) {
        SENSOR_D_LOG("exit bitmap_empty");
        return;
    }

    for (i = 0; i < SENSOR_MAX; i++) {
        if (test_bit(i, sensors_started)) {
            switch (i) {
            case SENSOR_ACC:
            case SENSOR_MAG:
            case SENSOR_MAG_UNCAL:
            case SENSOR_MAG_ROT_VCTR:
            case SENSOR_ACC_LNR:
            case SENSOR_GRV:
            case SENSOR_GYRO:
            case SENSOR_GYRO_UNCAL:
            case SENSOR_ORTN:
            case SENSOR_ROT_VCTR:
            case SENSOR_GAME_ROT_VCTR:
            case SENSOR_PRESSURE:
            case SENSOR_STEP_CNT:
            case SENSOR_STEP_DTC:
                idx = sns_micon_func.get_snstype2tblidx(i);
                g_logging_base_timestamps[idx] = now;
                SENSOR_N_LOG("set base timestamp type:%d idx:%d time:%lld", i, idx, now);
                break;
            default:
                SENSOR_ERR_LOG("invalid sensor:%d",i);
                break;
            }
        }
    }

    SENSOR_N_LOG("end");
}

void sns_logging_proceed_time(int64_t *timestamps)
{
    int64_t now = GET_CURRENT_TIMESTAMP_NS();
    int i;
    SENSOR_N_LOG("start");

    memcpy(timestamps, g_logging_base_timestamps, sizeof(g_logging_base_timestamps));
    for (i = 0; i < ARRAY_SIZE(g_logging_base_timestamps); i++) {
        g_logging_base_timestamps[i] = now;
    }

    SENSOR_N_LOG("end");
}

int32_t sns_logging_punctuation(int32_t trigger)
{
    int32_t ret;
#ifndef CONFIG_SENSOR_MICON_LOGGING
    return SNS_RC_OK;
#endif

    SENSOR_N_LOG("start");

    sensor_com_mutex_lock();

    ret = sns_micon_func.batch_mark_off(trigger, 1);
    if(SNS_RC_ERR_INVALID_ID == ret){
        /* Execute logging punctuation steps again for recovery.
           In this case logging data should not be reported. */
        sns_micon_func.batch_mark_off(trigger, 0);
    }

    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int sns_preenable(struct iio_dev *indio_dev){ return 0;}

enum ABS_status {
    SH_SCAN_DUMMY1 = 0,
    SH_SCAN_DUMMY2,
};
#define IIO_ST(si, rb, sb, sh) \
{ .sign = si, .realbits = rb, .storagebits = sb, .shift = sh }

#define SH_CHANNEL(axis)            \
{                       \
    .type = IIO_ACCEL,          \
    .modified = 1,              \
    .channel2 = axis+1,         \
    .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),  \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),  \
    .scan_index = axis,         \
    .scan_type = IIO_ST('u', 128, 128, 0)   \
}

static const struct iio_chan_spec sh_channels[] = {
    SH_CHANNEL(SH_SCAN_DUMMY1),
    SH_CHANNEL(SH_SCAN_DUMMY2),
};

static const struct iio_buffer_setup_ops sns_iio_buffer_setup_ops = {
    .preenable = &sns_preenable,
    //.preenable = &iio_sw_buffer_preenable,
};

static int sns_iio_probe_buffer(struct iio_dev *iio_dev)
{
    int ret;
    struct iio_buffer *buffer;

    SENSOR_N_LOG("start");

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    buffer = iio_kfifo_allocate();
#else
    buffer = iio_kfifo_allocate(iio_dev);
#endif /* LINUX_VERSION_CODE */
    if (!buffer) {
        SENSOR_ERR_LOG("iio_kfifo_allocate failed");
        ret = -ENOMEM;
        goto error_ret;
    }

    buffer->length = sns_micon_func.get_batch_fifo_size();
    iio_dev->buffer = buffer;
    iio_dev->setup_ops = &sns_iio_buffer_setup_ops;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    /* Do Nothing. */
#else
    ret = iio_buffer_register(iio_dev, iio_dev->channels,
                  iio_dev->num_channels);
    if (ret) {
        SENSOR_ERR_LOG("iio_buffer_register failed");
        goto error_free_buf;
    }

    iio_scan_mask_set(iio_dev, iio_dev->buffer, SH_SCAN_DUMMY1);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, SH_SCAN_DUMMY2);
#endif /* LINUX_VERSION_CODE */

    SENSOR_N_LOG("end");

    return 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    /* Do Nothing. */
#else
error_free_buf:
    iio_kfifo_free(iio_dev->buffer);
#endif
error_ret:
    return ret;
}

static void sns_iio_remove_buffer(struct iio_dev *indio_dev)
{
    SENSOR_N_LOG("start");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    /* Do Nothing. */
#else
    iio_buffer_unregister(indio_dev);
#endif /* LINUX_VERSION_CODE */
    iio_kfifo_free(indio_dev->buffer);
    SENSOR_N_LOG("end");
}

static int sns_iio_read_raw(struct iio_dev *indio_dev,
    struct iio_chan_spec const *chan,
    int *val,
    int *val2,
    long m)
{
    return -EINVAL;
}

static const struct iio_info sh_info = {
    .driver_module = THIS_MODULE,
    .read_raw = &sns_iio_read_raw,
};

static int sns_iio_init(void)
{
    struct iio_dev *indio_dev;
    int error;

    //SENSOR_N_LOG("start");

    indio_dev = iio_device_alloc(0);
    if (!indio_dev) {
        SENSOR_ERR_LOG("iio_device_alloc failed");
        return -ENOMEM;
    }

    indio_dev->name = "sensor_hub";
    indio_dev->info = &sh_info;
    indio_dev->channels = sh_channels;
    indio_dev->num_channels = ARRAY_SIZE(sh_channels);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    indio_dev->modes |= INDIO_BUFFER_SOFTWARE;
#else
    indio_dev->modes |= INDIO_BUFFER_HARDWARE;
#endif /* LINUX_VERSION_CODE */
    g_iio_dev = indio_dev;

    error = sns_iio_probe_buffer(indio_dev);
    if (error) {
        SENSOR_ERR_LOG("sns_iio_probe_buffer failed");
        goto error_free_dev;
    }
    error = iio_device_register(indio_dev);
    if (error) {
        SENSOR_ERR_LOG("iio_device_register failed");
        goto error_remove_buffer;
    }

    SENSOR_N_LOG("end");

    return 0;

error_remove_buffer:
    if (indio_dev)
        sns_iio_remove_buffer(indio_dev);
error_free_dev:
    g_iio_dev = NULL;

    return error;
}

static int sns_iio_exit(void)
{
    struct iio_dev *indio_dev = g_iio_dev;
    SENSOR_N_LOG("start");

    if (indio_dev)
        iio_device_unregister(indio_dev);
    if (indio_dev)
        sns_iio_remove_buffer(indio_dev);

    g_iio_dev = NULL;

    SENSOR_N_LOG("end");
    return 0;
}
