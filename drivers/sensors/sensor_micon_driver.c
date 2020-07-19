/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"
#include <linux/spi/spi.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/of_gpio.h>

#include <linux/sensor_power.h>
#include <linux/pinctrl/consumer.h>
#include <linux/wakelock.h>

#define SENSOR_MICON_DRIVER_NAME    "sensor_micon"
#define HC_MCU_GET_VERSION           (0x0001u)
#define HC_MCU_GET_EX_SENSOR         (0x0003u)
#define HC_MCU_SET_PCON              (0x0004u)
#define HC_MCU_GET_INT_DETAIL        (0x0006u)
#define HC_MCU_SELF_CHK_FW           (0x000Au)
#define HC_MCU_SENSOR_INIT           (0x000Bu)
#define HC_MCU_I2C_COM               (0x000Cu)
#define HC_MCU_SET_PERI              (0x000Du)
#define HC_MCU_EXEC_TASK             (0x000Fu)
#define HC_MCU_SET_TASK              (0x0011u)
#define HC_MCU_SET_PDIR              (0x0013u)
#define HC_MCU_SET_APP_TASK_PERIOD   (0x0015u)
#define HC_MCU_FUP_START             (0x0101u)
#define HC_MCU_FUP_ERASE             (0x0102u)
#define HC_MCU_FUP_END               (0x0104u)
#define HC_MCU_FUP_WRITE_FIFO        (0x0105u)
#define HC_ACC_SET_CALIB             (0x1009u)
#define HC_ACC_GET_CALIB             (0x100au)
#define HC_ACC_SET_CONV_AXIS         (0x105Eu)
#define HC_ACC_SET_PARAM             (0x1078u)
#define HC_DST_SET_DAILYS            (0x1080u)
#define HC_DST_CLR_DAILYS            (0x1082u)
#define HC_DST_CLR_INT_DETAIL        (0x1083u)
#define HC_DST_GET_INT_DETAIL        (0x1084u)
#define HC_DST_GET_PEDO1             (0x1085u)
#define HC_DST_GET_PEDO2             (0x1086u)
#define HC_DST_GET_RUN1              (0x1087u)
#define HC_DST_GET_TRANS1            (0x1088u)
#define HC_DST_GET_TRANS2            (0x1089u)
#define HC_DST_GET_INTELLI_WIFI      (0x108Au)
#define HC_DST_SET_IWIFI_INFO        (0x108Bu)
#define HC_DST_GET_RUN2              (0x1092u)
#define HC_DST_GET_PEDO5             (0x1093u)
#define HC_DST_EXEC_IWIFI            (0x1094u)
#define HC_MOT_EXEC_WALK_STOP        (0x1096u)
#define HC_MOT_EXEC_TRAIN            (0x1097u)
#define HC_MOT_GET_TRAIN_INFO        (0x1098u)
#define HC_DST_GET_TRANS6            (0x1099u)
#define HC_MOT_EXEC_WALK_START       (0x109Au)
#define HC_MOT_EXEC_VEHICLE          (0x109Bu)
#define HC_DST_SET_DAILYS_OPT        (0x109Cu)
#define HC_MOT_EXEC_BRINGUP          (0x10B3u)
#define HC_MOT_GET_BRINGUP_INFO      (0x10B4u)
#define HC_DST_EXEC_VH               (0x10B5u)
#define HC_DST_GET_VH                (0x10B6u)
#define HC_DST_SET_EB_SET            (0x10B7u)
#define HC_DST_SET_EB_PARAM          (0x10BBu)
#define HC_DST_GET_EB_PUNCTUATION    (0x10BDu)
#define HC_DST_GET_EB_DATA           (0x10BEu)
#define HC_DST_RCV_IBUF_DONE         (0x10BFu)
#define HC_DST_SET_ANDROID_ENABLE    (0x10C0u)
#define HC_DST_SET_ANDROID_PARAM     (0x10C2u)
#define HC_DST_EXEC_MOTION           (0x10C3u)
#define HC_DST_GET_EB_INFO1          (0x10C4u)
#define HC_DST_GET_EB_INFO2          (0x10C5u)
#define HC_DST_GET_EB_INFO3          (0x10C6u)
#define HC_DST_GET_EB_INFO4          (0x10C7u)
#define HC_DST_GET_EB_INFO5          (0x10C8u)
#define HC_DST_GET_EB_INFO6          (0x10C9u)
#define HC_BRM_SET_EB_PARAM          (0x10CAu)
#define HC_BRM_SET_EB_SET            (0x10CCu)
#define HC_DYNAMIC_DEVICE_CONTROL    (0x10D3u)
#define HC_ACC_SET_PARAM_LSM6DS3     (0x10F0u)
#define HC_ACC_SET_PARAM_LIS2DH      (0x10F0u)
#define HC_ACC_SET_AUTO_CALIB        (0x10F2u)
#define HC_DST_SET_VIB_INTERLOCKING  (0x10D1u)
#define HC_AUTO_CALIB_SET_DATA       (0x1e00u)
#define HC_AUTO_CALIB_GET_DATA       (0x1e01u)
#define HC_PRE_SET_OFFSET            (0x6003u)
#define HC_PRE_GET_OFFSET            (0x6004u)
#define HC_PRE_SET_BASE_VAL          (0x6080u)
#define HC_PRE_GET_BASE_VAL          (0x6081u)
#define HC_PRE_SET_PARAM             (0x60F0u)
#define HC_UWD_SET_EXEC              (0x6084u)
#define HC_UWD_GET_EXEC              (0x6085u)
#define HC_UWD_GET_WATER_INFO        (0x6088u)
#define HC_MAG_SET_DATA              (0x7005u)
#define HC_MAG_GET_DATA              (0x7006u)
#define HC_MAG_SET_CALIB             (0x7007u)
#define HC_EC_SET_CONV_AXIS          (0x7009u)
#define HC_MAG_SET_STATIC_MATRIX     (0x700Du)
#define HC_MAG_GET_STATIC_MATRIX     (0x700Eu)
#define HC_MAG_SET_DISPERTION_THRESH (0x700Fu)
#define HC_MAG_SET_OFFSET            (0x70F0u)
#define HC_MAG_GET_OFFSET            (0x70F1u)
#define HC_MAG_SET_PERIOD            (0x70F4u)
#define HC_MAG_SET_FILTER            (0x70F6u)
#define HC_MAG_SET_OFFSET_SOFT       (0x70F8u)
#define HC_GYRO_SET_DATA             (0x8002u)
#define HC_GYRO_GET_DATA             (0x8003u)
#define HC_GYRO_SET_CALIB            (0x8004u)
#define HC_GYRO_SET_CONV_AXIS        (0x8009u)
#define HC_GYRO_SET_PARAM            (0x80F0u)
#define HC_MUL_SET_LOG_PARAM_SENSOR  (0xF001u)
#define HC_MUL_GET_LOG_PARAM_SENSOR  (0xF002u)
#define HC_MUL_SET_LOG_DELAY_SENSOR  (0xF003u)
#define HC_MUL_SET_LOG_PARAM_FUSION  (0xF005u)
#define HC_MUL_SET_LOG_DELAY_FUSION  (0xF007u)
#define HC_MUL_GET_LOGGING_DATA      (0xF009u)
#define HC_MUL_GET_LOG_PUNCTUATION   (0xF00Au)
#define HC_MUL_SET_ANDROID           (0xF00Bu)
#define HC_MUL_GET_ANDROID           (0xF00Cu)
#define HC_MUL_SET_ANDROID_PERIOD    (0xF00Du)
#define HC_MUL_SET_FUSION            (0xF015u)
#define HC_MUL_SET_FUSION_STATUS     (0xF018u)
#define HC_MUL_GET_FUSION_STATUS     (0xF019u)
#define HC_MUL_SET_ENABLE            (0xF01Du)
#define HC_MUL_GET_ENABLE            (0xF01Eu)
#define HC_MUL_SET_SENSOR_PERIOD     (0xF01Fu)
#define HC_MUL_GET_SENSOR_PERIOD     (0xF020u)
#define HC_MUL_SET_FUSION_PERIOD     (0xF021u)
#define HC_MUL_GET_FUSION_PERIOD     (0xF022u)

#define HC_MAG_SET_BUFF_INT          (0xF01Bu)

#define KC_LOG_READ                  (0x1F01u)
#define KC_LOG_SIZE                  (2 + 256)


#define DAILYS_CLEAR_PEDO_DATA     0x00001
#define DAILYS_CLEAR_PEDO_STATE    0x00002
#define DAILYS_CLEAR_VEHI_DATA     0x00004
#define DAILYS_CLEAR_VEHI_STATE    0x00008
#define DAILYS_CLEAR_WIFI_STATE    0x00010
#define DAILYS_CLEAR_HEIGHT_STATE  0x00020
#define DAILYS_CLEAR_HEIGHT_ALL    0x00040
#define DAILYS_CLEAR_RT_STATE      0x00080
#define DAILYS_CLEAR_SHOCK_STATE   0x00100
#define DAILYS_CLEAR_STOP_STATE    0x00200
#define DAILYS_CLEAR_OUT_STATE     0x00400
#define DAILYS_CLEAR_TRAIN_STATE   0x00800
#define DAILYS_CLEAR_MOTION_STATE  0x01000
#define DAILYS_CLEAR_STEP_COUNTER  0x02000
#define DAILYS_CLEAR_STEP_TIMESTMP 0x04000
#define DAILYS_CLEAR_BATCH_TIMER   0x08000
#define DAILYS_CLEAR_VH_STATE      0x10000
#define DAILYS_CLEAR_EB_ALL        0x20000
#define DAILYS_CLEAR_DAILYS_DAY    0x40000
#define    DAILYS_CLEAR_ALL (DAILYS_CLEAR_PEDO_DATA | DAILYS_CLEAR_PEDO_STATE | DAILYS_CLEAR_VEHI_DATA | DAILYS_CLEAR_VEHI_STATE | DAILYS_CLEAR_WIFI_STATE | DAILYS_CLEAR_HEIGHT_ALL | DAILYS_CLEAR_RT_STATE)

#define OTHER_SENSOR                 (0x00u)
#define GEOMAGNETIC                  (0x01u)
#define GEOMAGNETIC_UNCALIB          (0x02u)
#define GYROSCOPE                    (0x04u)
#define GYROSCOPE_UNCALIB            (0x08u)
#define STEP_COUNTER                 (0x10u)
#define STEP_DETECTOR                (0x20u)

#define LOG_PARAM_SNS_NON            (0x00u)
#define LOG_PARAM_SNS_ACC            (0x01u)
#define LOG_PARAM_SNS_MAG            (0x40u)
#define LOG_PARAM_SNS_GYRO           (0x80u)

#define LOG_PARAM_FUS_NON            (0x00u)
#define LOG_PARAM_FUS_ORI            (0x01u)
#define LOG_PARAM_FUS_GRV            (0x02u)
#define LOG_PARAM_FUS_LACC           (0x04u)
#define LOG_PARAM_FUS_ROT            (0x08u)
#define LOG_PARAM_FUS_MROT           (0x20u)
#define LOG_PARAM_FUS_GROT           (0x10u)

#define ERROR_FUP_CERTIFICATION      (0x0013u)
#define FUP_MAX_RAMSIZE              (512)
#define MEASURE_DATA_SIZE            (18)
#define LOGGING_FIFO_SIZE            (3072)
#define EVENTBUF_FIFO_SIZE           (3072)

#define SNS_BATCH_PRID_ACC_MAX       (479400)
#define SNS_BATCH_PRID_ACC_MIN       (7520)
#define SNS_BATCH_PRID_ACC_UNIT      (1800)

#define SNS_BATCH_PRID_GYRO_MAX      (479400)
#define SNS_BATCH_PRID_GYRO_MIN      (5640)
#define SNS_BATCH_PRID_GYRO_UNIT     (1800)

#define SNS_BATCH_PRID_MAG_MAX       (479400)
#define SNS_BATCH_PRID_MAG_MIN       (15040)
#define SNS_BATCH_PRID_MAG_UNIT      (1800)

#define SNS_BATCH_PRID_FUSION_MAX    (2550000)
#define SNS_BATCH_PRID_FUSION_MIN    (10000)
#define SNS_BATCH_PRID_FUSION_UNIT   (10000)

#define SNS_BATCH_PRID_STEP_MAX      (7650000)
#define SNS_BATCH_PRID_STEP_MIN      (30000)
#define SNS_BATCH_PRID_STEP_UNIT     (30000)

#define SNS_BATCH_TIMEOUT_MAX        (0xFFFFFFFF)
#define SNS_BATCH_TIMEOUT_MIN        (30)
#define SNS_BATCH_TIMEOUT_UNIT       (30)

#define DEFAULT_WEIGHT               (650)
#define DEFAULT_PEDOMETER            (76)
#define DEFAULT_VEHITYPE             (2)
#define STATUS_READ_RETRY_NUM  98
#define STATUS_READ_RETRY_NUM_2  10
#define RESET_RETRY_NUM 5
#define SNS_WORK_QUEUE_NUM   230
#define MICON_I2C_ENABLE_RETRY_NUM  5
#define GRAVITY_EARTH        9806550
#define ABSMAX_2G            (GRAVITY_EARTH * 2)
#define ABSMIN_2G            (-GRAVITY_EARTH * 2)
#define ABSMAX_GYRO          (32000)
#define ABSMIN_GYRO          (-32000)
#define ABSMAX_MAG          (32767)
#define ABSMIN_MAG          (-32767)
#define ACCDATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define GYRODATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00008000 ? (((~(data) & 0x0000FFFF) + 1) * (-1)): (data))
#define MAGDATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00008000 ? (((~(data) & 0x0000FFFF) + 1) * (-1)): (data))
#define GRADATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define LINACCDATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define EVENTBUF_RESPONSE_HEADER 22
#define LOGGING_RESPONSE_HEADER 58
enum {
    LOGGING_TRIGGER_NONE = 0,
    LOGGING_TRIGGER_FLUSH,
    LOGGING_TRIGGER_BATCH,
    LOGGING_TRIGGER_TIMEOUT,
    LOGGING_TRIGGER_BUF512,
    LOGGING_TRIGGER_BUFFULL,
    LOGGING_TRIGGER_MAX
};

static DEFINE_SPINLOCK(acc_lock);

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

#ifdef CONFIG_USE_GPIOMUX
static struct gpiomux_setting spi_sens_config = {
    .func = GPIOMUX_FUNC_1,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_spi_sens_configs[] = {
    {
        .gpio      = 8,         /* BLSP3 QUP SPI_DATA_MOSI */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
    {
        .gpio      = 9,         /* BLSP3 QUP SPI_DATA_MISO */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
    {
        .gpio      = 10,         /* BLSP3 QUP SPI_SENS_CS */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
    {
        .gpio      = 11,         /* BLSP3 QUP SPI_CLK */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
};
#else
static struct pinctrl *s_sns_pinctrl;
static struct pinctrl_state *s_gpio_state_active;
static struct pinctrl_state *s_gpio_state_suspend;
#endif /* CONFIG_USE_GPIOMUX */

#define WORK_OPT_NONE       0
#define WORK_OPT_INT        1
#define WORK_OPT_BATCH      2
#define WORK_OPT_FLUSH      3

typedef struct t_SNS_WorkQueue {
    struct work_struct  work;
    bool                status;
    int32_t             option;
    enum sensor_e_type  type;
} SNS_WorkQueue;

static struct sensor_power_callback sns_pre_power_cb;
static SNS_WorkQueue  s_tSnsWork[SNS_WORK_QUEUE_NUM];
static struct workqueue_struct *sns_wq_int;
static struct workqueue_struct *sns_wq;
static int32_t g_nSnsWorkCnt;
static int32_t g_nFWVersion;
static int32_t g_nIntIrqNo;
static int32_t g_FusionDelay;
static uint8_t g_dailys_status;
static bool g_micon_pedo_status;
static bool g_micon_baro_status;
static uint8_t g_step_status;
static uint8_t g_reset_param_status;
static HostCmdRes diag_res;
static uint8_t g_logging_data[LOGGING_FIFO_SIZE+LOGGING_RESPONSE_HEADER];
static uint8_t g_eventbuf_data[EVENTBUF_FIFO_SIZE];
static int64_t g_eventbuf_cleartime;
static int64_t g_logging_base_timestamps[SENSOR_BATCH_TBL_MAX] = {0};
static atomic_t g_FusionState;
static atomic_t g_SnsTaskState;
static atomic_t g_AppTaskState;
static atomic_t g_FusTaskState;
static atomic_t g_nCalX;
static atomic_t g_nCalY;
static atomic_t g_nCalZ;
static atomic_t g_LogParamSns;
static atomic_t g_LogParamFus;
static atomic_t g_Step_start;
static atomic_t g_MiconDebug;
static atomic_t g_sensors_on_status;
static atomic_t g_pressure_cal_offset;
static atomic_t g_ShutdownFlg;
static struct acceleration g_acc_auto_cal_offset = {0};
static struct wake_lock g_pedo_wakelock;

atomic_t g_nStepWide;
atomic_t g_nWeight;
atomic_t g_nVehiType;
atomic_t g_FWUpdateStatus;

static struct geomagnetic s_MagData;
static struct gyroscope s_GyroData;
static int32_t g_workqueue_used = 0;

static int32_t g_vib_interlocking = 0x00;

static struct mutex sensor_fifo_mutex;

static DEFINE_MUTEX(acc_auto_cal_mutex);
static DEFINE_MUTEX(sensor_state_irq_mutex);
static DEFINE_MUTEX(sensor_state_app_mutex);

/* enable type */
#define EN_TYPE_NONE                (0)
#define EN_TYPE_NORMAL              (1)
#define EN_TYPE_APP                 (2)
#define EN_TYPE_FUSION              (3)

/* batch type */
#define BT_TYPE_NONE                (0)
#define BT_TYPE_NORMAL              (1)
#define BT_TYPE_STEP_CNT            (2)
#define BT_TYPE_STEP_DTC            (3)

/* enable bit */
#define EN_BIT_NONE                 (0x00000000)
#define EN_BIT_ACC                  (0x00000001)
#define EN_BIT_PRESSURE             (0x00000020)
#define EN_BIT_MAG_UNCAL            (0x00000040)
#define EN_BIT_GYRO_UNCAL           (0x00000080)
#define EN_BIT_MAG                  (0x00000100)
#define EN_BIT_GYRO                 (0x00000200)
#define EN_BIT_ORTN                 (0x00010000)
#define EN_BIT_GRV                  (0x00020000)
#define EN_BIT_ACC_LNR              (0x00040000)
#define EN_BIT_ROT_VCTR             (0x00080000)
#define EN_BIT_GAME_ROT_VCTR        (0x00100000)
#define EN_BIT_MAG_ROT_VCTR         (0x00200000)

#define EN_BIT_A_M                  (EN_BIT_ACC|EN_BIT_MAG)
#define EN_BIT_A_M_G                (EN_BIT_ACC|EN_BIT_MAG|EN_BIT_GYRO)
#define EN_BIT_A_G                  (EN_BIT_ACC|EN_BIT_GYRO)
#define EN_BIT_A_M_P                (EN_BIT_ACC|EN_BIT_MAG|EN_BIT_PRESSURE)

#define IS_ACC_EN(enable_bits)          (((enable_bits) & EN_BIT_ACC) != 0)
#define IS_MAG_EN(enable_bits)          (((enable_bits) & EN_BIT_MAG) != 0)
#define IS_GYRO_EN(enable_bits)         (((enable_bits) & EN_BIT_GYRO) != 0)
#define IS_MAG_UNCAL_EN(enable_bits)    (((enable_bits) & EN_BIT_MAG_UNCAL) != 0)
#define IS_GYRO_UNCAL_EN(enable_bits)   (((enable_bits) & EN_BIT_GYRO_UNCAL) != 0)
#define IS_PRESSURE_EN(enable_bits)     (((enable_bits) & EN_BIT_PRESSURE) != 0)

#define FUSION_SNS_MASK             ((1<<SENSOR_MAG)                \
                                    |(1<<SENSOR_MAG_UNCAL)          \
                                    |(1<<SENSOR_MAG_ROT_VCTR)       \
                                    |(1<<SENSOR_ACC_LNR)            \
                                    |(1<<SENSOR_GRV)                \
                                    |(1<<SENSOR_GYRO)               \
                                    |(1<<SENSOR_GYRO_UNCAL)         \
                                    |(1<<SENSOR_ORTN)               \
                                    |(1<<SENSOR_ROT_VCTR)           \
                                    |(1<<SENSOR_GAME_ROT_VCTR))

struct sensor_ctrl_config_str {
    const enum sensor_e_type            sns_type;
    const int32_t                       en_type;
    const int32_t                       bt_type;
    const int32_t                       en_bit_measure;
    const int32_t                       en_bit_logging;
    struct sensor_poll_info_str*        poll_info_p;
};

struct sensor_ctrl_param_input_one_str {
    bool        enable;
    int32_t     sampling_period_ms;
    int32_t     max_report_latency_ms;
};

struct sensor_ctrl_param_input_str {
    struct sensor_ctrl_param_input_one_str inputs[SENSOR_MAX];
    bool in_suspend;
};

struct sensor_ctrl_param_output_str {
    uint32_t enable_sns;
    struct {
        uint32_t enable;
        uint16_t period_sensor_task;
        uint16_t period_acc;
        uint16_t period_pressure;
        uint16_t period_mag;
        uint16_t period_gyro;
        uint16_t period_fusion_task;
        uint16_t period_fusion;
        uint8_t acc_param[4];
        uint8_t gyro_param[6];
        uint8_t mag_param[3];
        uint8_t pressure_param[3];
        uint8_t task_exec[3];
        uint8_t dd_control;
    } sensor;
    struct {
        uint32_t enable;
        uint32_t enable_sns;
        uint16_t period_sensor_task;
        uint16_t period_acc;
        uint16_t period_pressure;
        uint16_t period_mag;
        uint16_t period_gyro;
        uint16_t period_fusion_task;
        uint16_t period_fusion;
        uint32_t batch_timeout;
        uint32_t batch_timeout_step;
        uint8_t batch_step_cnt_enable;
        uint8_t batch_step_dtc_enable;
        uint8_t batch_enable;
    } logging;
    struct {
        uint32_t enable_sns;
    } polling;
};

struct sensor_ctrl_param_str {
    struct sensor_ctrl_param_input_str  input_param;
    struct sensor_ctrl_param_output_str output_param;
};

struct sensor_ctrl_config_str sns_ctrl_config[SENSOR_MAX] = {
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
    [SENSOR_SGNFCNT_MTN]            = { SENSOR_SGNFCNT_MTN,           EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M,                         EN_BIT_NONE,            NULL },
    [SENSOR_STEP_CNT]               = { SENSOR_STEP_CNT,              EN_TYPE_APP,        BT_TYPE_STEP_CNT,   EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_STEP_DTC]               = { SENSOR_STEP_DTC,              EN_TYPE_APP,        BT_TYPE_STEP_DTC,   EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_EXT_PEDO]               = { SENSOR_EXT_PEDO,              EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                       EN_BIT_NONE,            NULL },
    [SENSOR_EXT_VEHI]               = { SENSOR_EXT_VEHI,              EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M_P,                       EN_BIT_NONE,            NULL },
    [SENSOR_EXT_BARO]               = { SENSOR_EXT_BARO,              EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_PRESSURE,                    EN_BIT_NONE,            NULL },
    [SENSOR_EXT_IWIFI]              = { SENSOR_EXT_IWIFI,             EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M,                         EN_BIT_NONE,            NULL },
    [SENSOR_EXT_VH]                 = { SENSOR_EXT_VH,                EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_WALK_START]   = { SENSOR_KC_MOTION_WALK_START,  EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_WALK_STOP]    = { SENSOR_KC_MOTION_WALK_STOP,   EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_TRAIN]        = { SENSOR_KC_MOTION_TRAIN,       EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_A_M,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_VEHICLE]      = { SENSOR_KC_MOTION_VEHICLE,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_KC_MOTION_BRINGUP]      = { SENSOR_KC_MOTION_BRINGUP,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_ACC,                         EN_BIT_NONE,            NULL },
    [SENSOR_UNDERWATER_DETECT]      = { SENSOR_UNDERWATER_DETECT,     EN_TYPE_APP,        BT_TYPE_NONE,       EN_BIT_PRESSURE,                    EN_BIT_NONE,            NULL },
    [SENSOR_COM]                    = { SENSOR_COM,                   EN_TYPE_NONE,       BT_TYPE_NONE,       EN_BIT_NONE,                        EN_BIT_NONE,            NULL }
    /* SENSOR_MAX */
};

static struct sensor_ctrl_param_str sns_ctrl_param;

#if defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE)
static int32_t sns_mag_gyro_onoff(struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new);
#endif
static int32_t sns_dailys_send_latency_option(enum sensor_e_type type, bool enable);
static int32_t sns_dailys_send_ope_param(bool);
static int32_t sns_logging_punctuation_locked(int32_t trigger);
static int32_t sns_logging_punctuation(int32_t trigger);
static bool sns_devif_error_check(void);
static void sns_pre_power_on(void);
static void sns_pre_power_off(void);
#if 0
static int32_t sns_micon_i2c_enable(bool arg_iEnable);
#endif
static int32_t sns_micon_initcmd(void);
static int32_t sns_reset_restore_param(void);
static int32_t sns_reset_save_param(void);

static irqreturn_t sns_irq_handler(int32_t irq, void *dev_id);
static void sns_int_work_func(struct work_struct *work);
static void sns_int_app_work_func(struct work_struct *work);
static int32_t sns_get_gpio_info( struct spi_device *client );
static int32_t sns_gpio_init( struct spi_device *client );
static void sns_FW_BRMP_ctrl(void);
static int32_t sns_update_fw_exe(bool boot, uint8_t *arg_iData, uint32_t arg_iLen);
static void sns_workqueue_init(void);
static int32_t sns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *), int32_t option, enum sensor_e_type type );
static void sns_workqueue_delete(struct work_struct *work);
void sns_set_buff_int(bool arg_iEnable);


static int32_t sensor_micon_resume( struct spi_device *client );
static int32_t sensor_micon_suspend( struct spi_device *client, pm_message_t mesg );
static void sensor_micon_shutdown( struct spi_device *client );
static int32_t sensor_micon_remove( struct spi_device *client );
static int32_t sensor_micon_probe( struct spi_device *client );
static int sns_iio_report_event(uint8_t id, uint8_t *data, uint32_t len, int64_t timestamp, int64_t offset);
static int sns_iio_report_events(uint8_t *batch_data, uint32_t len, int64_t *timestamps, uint32_t batch_status);
static int sns_iio_report_flush(enum sensor_e_type type, int64_t timestamp);
static void sns_logging_setup_basetime(uint32_t logging_new, uint32_t logging_old);
static int sns_iio_init(void);
static int sns_iio_exit(void);

#define ENABLE_IRQ {                                                         \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == false)){   \
        atomic_set(&g_bIsIntIrqEnable,true);                                 \
        enable_irq(g_nIntIrqNo);                                             \
    }                                                                        \
}
#define DISABLE_IRQ {                                                        \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == true)){    \
        disable_irq_nosync(g_nIntIrqNo);                                     \
        atomic_set(&g_bIsIntIrqEnable,false);                                \
    }                                                                        \
}

#define GET_CURRENT_TIMESTAMP_NS()   (ktime_to_ns(ktime_get_boottime()))

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

#define PERIOD_ACC_MAX          (200)
#define PERIOD_ACC_MIN          (10)
#define PERIOD_MAG_MAX          (400)
#define PERIOD_MAG_MIN          (20)
#define PERIOD_GYRO_MAX         (400)
#define PERIOD_GYRO_MIN         (10)
#define PERIOD_PRESSURE_MAX     (400)
#define PERIOD_PRESSURE_MIN     (50)
#define PERIOD_FUSION_TASK_MAX  (100)
#define PERIOD_FUSION_TASK_MIN  (10)
#define PERIOD_FUSION_MAX       (100)
#define PERIOD_FUSION_MIN       (10)
#define BATCH_TIMEOUT_MAX       (0x7FFFFFFF)
#define BATCH_TIMEOUT_MIN       (30)

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

static inline bool sns_is_fw_ver_gteq(uint32_t compare_version)
{
    return false;
#if 0 //Barometer
    uint8_t fw_ver[4];
    int32_t ret;
    bool    result = false;
    SENSOR_N_LOG("start");

    result = (g_nFWVersion >= compare_version);

    if (!result) {
        SENSOR_N_LOG("re-check fw version directly");
        ret = sns_get_fw_version(fw_ver);
        if (ret == SNS_RC_OK) {
            result = (SNESOR_COM_GET_FW_VER(fw_ver) >= compare_version);
        }
    }

    SENSOR_N_LOG("end result[%d]", result);
    return result;
#endif
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
    int32_t batch_timeout = BATCH_TIMEOUT_MAX;
    int32_t batch_timeout_step = BATCH_TIMEOUT_MAX;
    uint32_t sensor_enable_sns = 0;
    uint32_t measure_enable = 0;
    uint32_t logging_enable = 0;
    uint32_t logging_enable_sns = 0;
    uint8_t dd_control = 0;
    uint8_t acc_param[4] = {0};
    uint8_t gyro_param[6] = {0};
    uint8_t mag_param[3] = {0};
    uint8_t pressure_param[3] = {0};
    uint8_t task_exec[3] = {0};
    uint8_t batch_normal_enable = 0;
    uint8_t batch_step_cnt_enable = 0;
    uint8_t batch_step_dtc_enable = 0;
    uint8_t batch_enable = 0;
    uint32_t polling_enable_sns = 0;
    bool fusion_enable = false;
    bool app_enable = false;
    bool android_sensor_enable = false;
    int i;
    bool can_batch_in_resume;
    bool is_fw_gteq_0x27000000 = sns_is_fw_ver_gteq(0x27000000u);
    SENSOR_N_LOG("start");

    // enable/timeout
    for (i = 0; i < ARRAY_SIZE(src->inputs); i++) {
        config = &sns_ctrl_config[i];
        input = &src->inputs[i];

        if (input->enable) {
            sensor_enable_sns |= (1 << config->sns_type);
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
            } else if (config->en_type == EN_TYPE_APP) {
                app_enable = true;
                if (config->en_bit_measure & EN_BIT_ACC)
                    period_acc = min(period_acc, 30);
                if (config->en_bit_measure & EN_BIT_MAG)
                    period_mag = min(period_mag, 240);
                if (config->en_bit_measure & EN_BIT_PRESSURE)
                    period_pressure = min(period_pressure, 50);
            } else {
                // nop
                continue;
            }

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
                logging_enable_sns |= (1 << config->sns_type);
            } else {
                if (can_batch_in_resume) {
                    logging_enable |= config->en_bit_logging;
                    logging_enable_sns |= (1 << config->sns_type);
                } else if (config->poll_info_p != NULL) {
                    polling_enable_sns |= (1 << config->sns_type);
                } else {
                    // nop
                }
            }
#else
            (void)can_batch_in_resume;
            // enable
            measure_enable |= config->en_bit_measure;
            if (!src->in_suspend && config->poll_info_p != NULL) {
                polling_enable_sns |= (1 << config->sns_type);
            }
#endif
        }
    }
    period_fusion_task = period_fusion;

    // adjust period or timeout

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
        fusion_enable, PERIOD_FUSION_MAX, PERIOD_FUSION_MIN, false);
    period_fusion_task = sns_adjust_value_by_range_enable(period_fusion_task,
        fusion_enable, PERIOD_FUSION_TASK_MAX, PERIOD_FUSION_TASK_MIN, false);
    batch_timeout = sns_adjust_value_by_range_enable(batch_timeout,
        batch_normal_enable, BATCH_TIMEOUT_MAX, BATCH_TIMEOUT_MIN, true);
    batch_timeout_step = sns_adjust_value_by_range_enable(batch_timeout_step,
        batch_step_cnt_enable || batch_step_dtc_enable, BATCH_TIMEOUT_MAX, BATCH_TIMEOUT_MIN, true);

    if (period_acc < 30) {
        period_acc = 10;
        period_sensor_task = 0x00BCu;
    } else {
        period_acc = 30;
        period_sensor_task = 0x0BB8u;
    }

#ifdef CONFIG_USE_SENSOR_LSM6DS3
    //acc param
    acc_param[0] = 0x01;
    if (period_acc < 30)
        acc_param[1] = 0x04;
    else
        acc_param[1] = 0x02;
    acc_param[2] = 0x02;
    acc_param[3] = 0x01;
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
    if (period_gyro < 10)
        gyro_param[1] = 0x06;
    else if(period_gyro < 50)
        gyro_param[1] = 0x04;
    else
        gyro_param[1] = 0x02;
    gyro_param[2] = 0x00;
    gyro_param[3] = 0x00;
    gyro_param[4] = (80 + period_gyro - 1) / period_gyro;
    gyro_param[5] = 0x00;

    //mag param
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

    //pressure param
    if (is_fw_gteq_0x27000000) {
        pressure_param[0] = 0x01;
    } else {
        if (period_pressure < 100)
            pressure_param[0] = 0x02;
        else if (period_pressure < 200)
            pressure_param[0] = 0x03;
        else
            pressure_param[0] = 0x04;
    }
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
    if (measure_enable)
        dd_control = (sensor_enable_sns & (1 << SENSOR_ACC))? 0x00 : 0x01;
    else
        dd_control = 0x00;

    // update dst
    memset(dst, 0x00, sizeof(*dst));

    dst->sensor.enable = measure_enable;
    dst->sensor.period_sensor_task = period_sensor_task;
    dst->sensor.period_acc = period_acc * 10;
    dst->sensor.period_mag = period_mag * 10;
    dst->sensor.period_gyro = period_gyro * 10;
    dst->sensor.period_pressure = period_pressure * 10;
    dst->sensor.period_fusion_task = period_fusion_task * 100;
    dst->sensor.period_fusion = period_fusion * 10;
    dst->sensor.dd_control = dd_control;
    memcpy(dst->sensor.acc_param, acc_param, sizeof(dst->sensor.acc_param));
    memcpy(dst->sensor.gyro_param, gyro_param, sizeof(dst->sensor.gyro_param));
    memcpy(dst->sensor.mag_param, mag_param, sizeof(dst->sensor.mag_param));
    memcpy(dst->sensor.pressure_param, pressure_param, sizeof(dst->sensor.pressure_param));
    memcpy(dst->sensor.task_exec, task_exec, sizeof(dst->sensor.task_exec));

    dst->logging.enable = logging_enable;
    dst->logging.enable_sns = logging_enable_sns;
    dst->logging.period_sensor_task = period_sensor_task;
    dst->logging.period_acc = period_acc_logging * 10;
    dst->logging.period_mag = period_mag_logging * 10;
    dst->logging.period_gyro = period_gyro_logging * 10;
    dst->logging.period_pressure = period_pressure_logging * 10;
    dst->logging.period_fusion_task = period_fusion_task * 100;
    dst->logging.period_fusion = period_fusion * 10;

    if (src->in_suspend) {
        dst->logging.batch_timeout = 0xFFFFFFFF;
        dst->logging.batch_timeout_step = 0xFFFFFFFF;
    } else {
        dst->logging.batch_timeout = batch_timeout/30;
        dst->logging.batch_timeout_step = batch_timeout_step/30;
    }

    dst->logging.batch_step_cnt_enable = batch_step_cnt_enable;
    dst->logging.batch_step_dtc_enable = batch_step_dtc_enable;
    dst->logging.batch_enable = batch_enable;

    dst->polling.enable_sns = polling_enable_sns;
    dst->enable_sns = sensor_enable_sns;

    SENSOR_D_LOG("param sensor enable_sns=%08x", dst->enable_sns);
    SENSOR_D_LOG("param measure enable_bit=%08x", dst->sensor.enable);
    SENSOR_D_LOG("param logging enable_bit=%08x enable_sns=%08x", dst->logging.enable, dst->logging.enable_sns);
    SENSOR_D_LOG("param polling enable_sns=%08x", dst->polling.enable_sns);
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
    SENSOR_N_LOG("start");

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

                if ((old->polling.enable_sns != new->polling.enable_sns) &&
                    (new->polling.enable_sns == 0))
                    change_type |= CHANGE_TYPE_POLLING_OFF;
            }
        }
    }

    SENSOR_N_LOG("end - return[%d]",change_type);
    return change_type;
}

static int32_t sns_send_sensor_ctrl(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new,
    bool force)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    SENSOR_N_LOG("start");
#if defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE)
    ret = sns_mag_gyro_onoff(old, new);
    if(SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_mag_gyro_onoff err ret[%d]", ret);
        return ret;
    }
#endif

    cmd.cmd.udata16 = HC_DYNAMIC_DEVICE_CONTROL;
    cmd.prm.ub_prm[0]  = 0x01;
    cmd.prm.ub_prm[1]  = new->sensor.dd_control;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DYNAMIC_DEVICE_CONTROL err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_SENSOR_PERIOD;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->sensor.period_sensor_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->sensor.period_sensor_task >> 8);
    cmd.prm.ub_prm[3]  = 0x00;
    cmd.prm.ub_prm[4]  = (uint8_t)(new->sensor.period_acc >> 0);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->sensor.period_acc >> 8);
    cmd.prm.ub_prm[6]  = 0x00;
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    cmd.prm.ub_prm[7]  = (uint8_t)(new->sensor.period_pressure >> 0);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->sensor.period_pressure >> 8);
    cmd.prm.ub_prm[9]  = 0x00;
#else
    cmd.prm.ub_prm[7]  = 0x00;
    cmd.prm.ub_prm[8]  = 0x00;
    cmd.prm.ub_prm[9]  = 0x00;
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    cmd.prm.ub_prm[10]  = (uint8_t)(new->sensor.period_mag >> 0);
    cmd.prm.ub_prm[11]  = (uint8_t)(new->sensor.period_mag >> 8);
    cmd.prm.ub_prm[12]  = 0x00;
#else
    cmd.prm.ub_prm[10]  = 0x00;
    cmd.prm.ub_prm[11]  = 0x00;
    cmd.prm.ub_prm[12]  = 0x00;
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	cmd.prm.ub_prm[13] = (uint8_t)(new->sensor.period_gyro >> 0);
    cmd.prm.ub_prm[14] = (uint8_t)(new->sensor.period_gyro >> 8);
    cmd.prm.ub_prm[15] = 0x00;
#else
	cmd.prm.ub_prm[13] = 0x00;
    cmd.prm.ub_prm[14] = 0x00;
    cmd.prm.ub_prm[15] = 0x00;
#endif
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_SENSOR_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#if 0 //Fusion
    cmd.cmd.udata16 = HC_MUL_SET_FUSION_PERIOD;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->sensor.period_fusion_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->sensor.period_fusion_task >> 8);
    cmd.prm.ub_prm[3]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[7]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[9]  = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[10] = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[11] = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[12] = (uint8_t)(new->sensor.period_fusion >> 8);
    cmd.prm.ub_prm[13] = (uint8_t)(new->sensor.period_fusion >> 0);
    cmd.prm.ub_prm[14] = (uint8_t)(new->sensor.period_fusion >> 8);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
    cmd.cmd.udata16 = HC_MCU_SET_APP_TASK_PERIOD;
    cmd.prm.ub_prm[0]  = (uint8_t)(0x0BB8u >> 0);
    cmd.prm.ub_prm[1]  = (uint8_t)(0x0BB8u >> 8);
    cmd.prm.ub_prm[2]  = 0x00;
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
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM_LSM6DS3 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    cmd.cmd.udata16 = HC_GYRO_SET_PARAM;
    cmd.prm.ub_prm[0]  = new->sensor.gyro_param[0];
    cmd.prm.ub_prm[1]  = new->sensor.gyro_param[1];
    cmd.prm.ub_prm[2]  = new->sensor.gyro_param[2];
    cmd.prm.ub_prm[3]  = new->sensor.gyro_param[3];
    cmd.prm.ub_prm[4]  = new->sensor.gyro_param[4];
    cmd.prm.ub_prm[5]  = new->sensor.gyro_param[5];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
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
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    cmd.cmd.udata16 = HC_PRE_SET_PARAM;
    cmd.prm.ub_prm[0]  = new->sensor.pressure_param[0];
    cmd.prm.ub_prm[1]  = new->sensor.pressure_param[1];
    cmd.prm.ub_prm[2]  = new->sensor.pressure_param[2];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_PRE_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
    cmd.cmd.udata16 = HC_MUL_SET_ENABLE;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = (uint8_t)(new->sensor.enable >> 0);
    cmd.prm.ub_prm[2] = (uint8_t)(new->sensor.enable >> 8);
    cmd.prm.ub_prm[3] = (uint8_t)(new->sensor.enable >> 16);
    cmd.prm.ub_prm[4] = (uint8_t)(new->sensor.enable >> 24);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0]  = new->sensor.task_exec[0];
    cmd.prm.ub_prm[1]  = new->sensor.task_exec[1];
    cmd.prm.ub_prm[2]  = new->sensor.task_exec[2];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end");
    return SNS_RC_OK;
}

static int32_t sns_send_logging_state(
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
    cmd.prm.ub_prm[3]  = (uint8_t)(new->logging.period_acc >> 0);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->logging.period_acc >> 8);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->logging.period_pressure >> 0);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->logging.period_pressure >> 8);
    cmd.prm.ub_prm[7]  = (uint8_t)(new->logging.period_mag >> 0);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->logging.period_mag >> 8);
    cmd.prm.ub_prm[9]  = (uint8_t)(new->logging.period_gyro >> 0);
    cmd.prm.ub_prm[10] = (uint8_t)(new->logging.period_gyro >> 8);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_SENSOR_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_PERIOD;
    cmd.prm.ub_prm[0]  = 0x01;
    cmd.prm.ub_prm[1]  = (uint8_t)(new->logging.period_fusion_task >> 0);
    cmd.prm.ub_prm[2]  = (uint8_t)(new->logging.period_fusion_task >> 8);
    cmd.prm.ub_prm[3]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[4]  = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[5]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[6]  = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[7]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[8]  = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[9]  = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[10] = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[11] = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[12] = (uint8_t)(new->logging.period_fusion >> 8);
    cmd.prm.ub_prm[13] = (uint8_t)(new->logging.period_fusion >> 0);
    cmd.prm.ub_prm[14] = (uint8_t)(new->logging.period_fusion >> 8);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_SET_ANDROID_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (uint8_t)(new->logging.batch_timeout & 0xFF);
    cmd.prm.ub_prm[2] = (uint8_t)((new->logging.batch_timeout >> 8) & 0xFF);
    cmd.prm.ub_prm[3] = (uint8_t)((new->logging.batch_timeout >> 16) & 0xFF);
    cmd.prm.ub_prm[4] = (uint8_t)((new->logging.batch_timeout >> 24) & 0xFF);
    cmd.prm.ub_prm[5] = (uint8_t)(new->logging.batch_timeout_step & 0xFF);
    cmd.prm.ub_prm[6] = (uint8_t)((new->logging.batch_timeout_step >> 8) & 0xFF);
    cmd.prm.ub_prm[7] = (uint8_t)((new->logging.batch_timeout_step >> 16) & 0xFF);
    cmd.prm.ub_prm[8] = (uint8_t)((new->logging.batch_timeout_step >> 24) & 0xFF);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_ANDROID_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_BUFF_INT;
    cmd.prm.ub_prm[0] = batch_on ? 0x01 : 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_BUFF_INT err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_SET_ANDROID_ENABLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = new->logging.batch_step_cnt_enable;
    cmd.prm.ub_prm[2] = new->logging.batch_step_dtc_enable;
    cmd.prm.ub_prm[3] = new->logging.batch_enable;
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
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    sns_logging_setup_basetime(new->logging.enable_sns, old->logging.enable_sns);

    if (step_counter_enabled) {
        msleep(50);
    }

    sns_logging_punctuation_locked(LOGGING_TRIGGER_BATCH);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t sns_ctrl_polling(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new)
{
    struct sensor_poll_info_str* poll_info_p;
    int32_t i;
    uint32_t check_bit;
    uint32_t changed_bit = old->polling.enable_sns ^ new->polling.enable_sns;
    SENSOR_N_LOG("start changed_bit[0x%08x]", changed_bit);

    for (i = 0; i < SENSOR_MAX; i++) {
        check_bit = 1 << i;
        poll_info_p = sns_ctrl_config[i].poll_info_p;
        if ((changed_bit & check_bit) && (poll_info_p != NULL)) {
            if (new->polling.enable_sns & check_bit) {
                SENSOR_N_LOG("queue work type[%d]", i);
                queue_delayed_work((poll_info_p->poll_wq),
                                   &(poll_info_p->poll_work),
                                   msecs_to_jiffies(atomic_read(&(poll_info_p->poll_time))));
            } else {
                SENSOR_N_LOG("cancel work type[%d]", i);
                sensor_com_mutex_unlock();
                cancel_delayed_work_sync(&(poll_info_p->poll_work));
                sensor_com_mutex_lock();
            }
        }
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
            sensor_com_mutex_unlock();
            cancel_delayed_work_sync(&(sns_ctrl_config[i].poll_info_p->poll_work));
            sensor_com_mutex_lock();
        }
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
            if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_LOGGING))
                ret = sns_send_logging_state(&output_old, &output_new);
            if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_SENSOR))
                ret = sns_send_sensor_ctrl(&output_old, &output_new, force);
        } else {
            if (change_type & CHANGE_TYPE_SENSOR)
                ret = sns_send_sensor_ctrl(&output_old, &output_new, force);
            if (change_type & CHANGE_TYPE_POLLING_OFF) {
                if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_POLLING))
                    ret = sns_ctrl_polling(&output_old, &output_new);
                if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_LOGGING))
                    ret = sns_send_logging_state(&output_old, &output_new);
            } else {
                if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_LOGGING))
                    ret = sns_send_logging_state(&output_old, &output_new);
                if (ret == SNS_RC_OK && (change_type & CHANGE_TYPE_POLLING))
                    ret = sns_ctrl_polling(&output_old, &output_new);
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
            ret = sns_dailys_send_latency_option(type, true);
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

    sns_workqueue_create(sns_wq, sns_int_app_work_func, WORK_OPT_FLUSH, type);

    SENSOR_D_LOG("end");
}

uint32_t sns_get_logging_enable(void)
{
    return sns_ctrl_param.output_param.logging.enable_sns;
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

    SENSOR_N_LOG("start");

    mag_uncal_enable = IS_MAG_UNCAL_EN(new->sensor.enable);
    gyro_uncal_enable = IS_GYRO_UNCAL_EN(new->sensor.enable);
    fusion_off = (old->enable_sns & ~new->enable_sns & FUSION_SNS_MASK) != 0;

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

        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 7, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_MagData.x = res.res.sw_res[0];
            s_MagData.y = res.res.sw_res[1];
            s_MagData.z = res.res.sw_res[2];
            s_MagData.accuracy = res.res.ub_res[6];

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d] a[%d]",s_MagData.x,s_MagData.y,s_MagData.z,s_MagData.accuracy);
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

        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_GyroData.x = res.res.sw_res[0];
            s_GyroData.y = res.res.sw_res[1];
            s_GyroData.z = res.res.sw_res[2];

            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",s_GyroData.x,s_GyroData.y,s_GyroData.z);
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
        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 7, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_MagData.x = res.res.sw_res[0];
            s_MagData.y = res.res.sw_res[1];
            s_MagData.z = res.res.sw_res[2];
            s_MagData.accuracy = res.res.ub_res[6];

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d] a[%d]",s_MagData.x,s_MagData.y,s_MagData.z,s_MagData.accuracy);
        }

        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_GyroData.x = res.res.sw_res[0];
            s_GyroData.y = res.res.sw_res[1];
            s_GyroData.z = res.res.sw_res[2];

            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",s_GyroData.x,s_GyroData.y,s_GyroData.z);
        }
    }

    SENSOR_N_LOG("end - SNS_RC_OK");
    return SNS_RC_OK;
}
#endif /* defined(CONFIG_INPUT_SENSOR_MAGNETOMETER)||defined(CONFIG_INPUT_SENSOR_GYROSCOPE) */

#define PEDOMETER_WAKELOCK_TIME (msecs_to_jiffies(5000))
int32_t sns_flush_event_buffer(void)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    uint16_t eventbuf_data_size = 0;
    uint16_t read_data_size = 0;
    uint8_t  fifo_size_res[2];
    uint16_t fifo_data_size = 0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);

    wake_lock_timeout(&g_pedo_wakelock, PEDOMETER_WAKELOCK_TIME);
    cmd.cmd.udata16 = HC_DST_GET_EB_PUNCTUATION;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_DST_EB_PUNCTUATION[%x]", res.err.udata16);
        mutex_unlock(&sensor_fifo_mutex);
        return SNS_RC_ERR;
    }

    ret = sns_device_read(RSLT3E, fifo_size_res, 2);
    SENSOR_N_LOG("sns_device_read-ret[%d] res1[%x] res2[%x]",ret,fifo_size_res[0],fifo_size_res[1]);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("sns_device_read[%d]", ret);
        mutex_unlock(&sensor_fifo_mutex);
        return ret;
    }
    fifo_data_size = ((fifo_size_res[1] << 8) | fifo_size_res[0]);
    SENSOR_N_LOG("sns_device_read-fifo_data_size[%x]",fifo_data_size);

    if(EVENTBUF_RESPONSE_HEADER > fifo_data_size || fifo_data_size > sizeof(res.res.ub_res)) {
        SENSOR_ERR_LOG("fifo_data_size[%d]", fifo_data_size);
        mutex_unlock(&sensor_fifo_mutex);
        return SNS_RC_ERR;
    }

    ret = sns_device_read(FIFO, res.res.ub_res, fifo_data_size);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("sns_device_read[%d]", ret);
        mutex_unlock(&sensor_fifo_mutex);
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
        mutex_unlock(&sensor_fifo_mutex);
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
                mutex_unlock(&sensor_fifo_mutex);
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
                mutex_unlock(&sensor_fifo_mutex);
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
        mutex_unlock(&sensor_fifo_mutex);
        return SNS_RC_ERR;
    }

    mutex_unlock(&sensor_fifo_mutex);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_read_data(struct acceleration *arg_Acc)
{
    uint8_t ucBuff[MEASURE_DATA_SIZE];
    int32_t ret = SNS_RC_OK;
    int32_t raw[3];
    int32_t pos[3];
    int32_t xyz[3];
    int32_t temp;
    int i,j;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT00, ucBuff, sizeof(ucBuff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

int32_t sns_acc_set_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    int32_t temp;

    SENSOR_N_LOG("start");

    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[0]);
    atomic_set(&g_nCalX, temp);
    SENSOR_N_LOG("set offset X[%d]",temp);
    
    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[1]);
    atomic_set(&g_nCalY, temp);
    SENSOR_N_LOG("set offset Y[%d]",temp);
    
    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[2]);
    atomic_set(&g_nCalZ, temp);
    SENSOR_N_LOG("set offset Z[%d]",temp);

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

int32_t sns_acc_get_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_ACC_GET_CALIB;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

static int32_t sns_acc_set_auto_cal_offset_internal(void)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    struct acceleration offsets;

    SENSOR_N_LOG("start");

    mutex_lock(&acc_auto_cal_mutex);
    offsets.nX = g_acc_auto_cal_offset.nX;
    offsets.nY = g_acc_auto_cal_offset.nY;
    offsets.nZ = g_acc_auto_cal_offset.nZ;
    mutex_unlock(&acc_auto_cal_mutex);

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
int32_t sns_acc_set_auto_cal_offset(struct acceleration* offsets)
{
    int32_t ret;

    SENSOR_N_LOG("start offset [X:%d Y:%d Z:%d]", offsets->nX, offsets->nY, offsets->nZ);

    mutex_lock(&acc_auto_cal_mutex);
    g_acc_auto_cal_offset.nX = offsets->nX;
    g_acc_auto_cal_offset.nY = offsets->nY;
    g_acc_auto_cal_offset.nZ = offsets->nZ;
    mutex_unlock(&acc_auto_cal_mutex);

    ret = sns_acc_set_auto_cal_offset_internal();

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_get_auto_cal_offset(struct acceleration* offsets)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&acc_auto_cal_mutex);
    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_AUTO_CALIB_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_GET_CALIB err[%x]",res.err.udata16);
    }else{
        g_acc_auto_cal_offset.nX = res.res.sw_res[0];
        g_acc_auto_cal_offset.nY = res.res.sw_res[1];
        g_acc_auto_cal_offset.nZ = res.res.sw_res[2];
    }
    offsets->nX = g_acc_auto_cal_offset.nX;
    offsets->nY = g_acc_auto_cal_offset.nY;
    offsets->nZ = g_acc_auto_cal_offset.nZ;
    mutex_unlock(&acc_auto_cal_mutex);

    SENSOR_N_LOG("end - return[%d] offset [X:%d Y:%d Z:%d]",ret, offsets->nX, offsets->nY, offsets->nZ);

    return ret;
}

int32_t sns_acc_set_host_cmd(int32_t* req_cmd, int32_t* req_param)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    uint32_t i;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = (uint16_t)((req_cmd[0] << 8) | req_cmd[1]);
    for(i=0;i<16;i++){
        cmd.prm.ub_prm[i] = (uint8_t)req_param[i];
    }

    ret = sns_hostcmd(&cmd, &res, 128, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("sns_acc_set_host_cmd[%x] err[%x]", cmd.cmd.udata16, res.err.udata16);
    }

    memcpy(&diag_res, &res, sizeof(HostCmdRes));

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

void sns_acc_get_host_cmd(uint32_t* res)
{
    uint32_t i;

    SENSOR_N_LOG("start");

    for(i=0;i<128;i++){
        res[i] = (uint32_t)diag_res.res.ub_res[i];
    }

    SENSOR_N_LOG("end");
}

void sns_dailys_set_vib_interlocking(int32_t mode)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    if (g_vib_interlocking != mode) {
        cmd.cmd.udata16 = HC_DST_SET_VIB_INTERLOCKING;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = mode;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        g_vib_interlocking = mode;
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_SET_VIB_INTERLOCKING err[%x]",res.err.udata16);
            return;
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);
}

static int32_t sns_send_mag_set_offset_soft(void)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    struct geomagnetic mag;

    SENSOR_N_LOG("start");

    if (!sns_is_fw_ver_gteq(0x27000000u)) {
        SENSOR_N_LOG("skip");
        return ret;
    }

    mag.x = s_MagData.x*100;
    mag.y = s_MagData.y*100;
    mag.z = s_MagData.z*100;
    mag.accuracy = s_MagData.accuracy;

    cmd.cmd.udata16 = HC_MAG_SET_OFFSET_SOFT;
    cmd.prm.ub_prm[0] = mag.accuracy;
    cmd.prm.ub_prm[1] = (uint8_t)(mag.x & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)((mag.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((mag.x >> 16) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)((mag.x >> 24) & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)(mag.y & 0x000000ff);
    cmd.prm.ub_prm[6] = (uint8_t)((mag.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[7] = (uint8_t)((mag.y >> 16) & 0x000000ff);
    cmd.prm.ub_prm[8] = (uint8_t)((mag.y >> 24) & 0x000000ff);
    cmd.prm.ub_prm[9] = (uint8_t)(mag.z & 0x000000ff);
    cmd.prm.ub_prm[10] = (uint8_t)((mag.z >> 8) & 0x000000ff);
    cmd.prm.ub_prm[11] = (uint8_t)((mag.z >> 16) & 0x000000ff);
    cmd.prm.ub_prm[12] = (uint8_t)((mag.z >> 24) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_OFFSET_SOFT err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_read_data(struct geomagnetic *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    uint8_t buff2[1];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT0C, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);
    if(SNS_RC_OK == ret){
        ret = sns_device_read(RSLT2D, buff2, sizeof(buff2));
        SENSOR_N_LOG("sns_device_read[%d]",ret);
    }

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Mag->x =MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Mag->y = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Mag->z = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        arg_Mag->accuracy = buff2[0];
        s_MagData.accuracy = arg_Mag->accuracy;

        SENSOR_N_LOG("mag - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff2[0]);
        SENSOR_N_LOG("arg_Mag - x[%04x] y[%04x] z[%04x] a[%x]",
                     arg_Mag->x,arg_Mag->y,arg_Mag->z,arg_Mag->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_set_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    s_MagData.x = MAGDATA_SIGN_COMVERT_12_32BIT(offsets[0]);
    SENSOR_N_LOG("set offset X[%d]",s_MagData.x);
    
    s_MagData.y = MAGDATA_SIGN_COMVERT_12_32BIT(offsets[1]);
    SENSOR_N_LOG("set offset Y[%d]",s_MagData.y);
    
    s_MagData.z = MAGDATA_SIGN_COMVERT_12_32BIT(offsets[2]);
    SENSOR_N_LOG("set offset Z[%d]",s_MagData.z);

    cmd.cmd.udata16 = HC_MAG_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_MagData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_MagData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_MagData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_MagData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_MagData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_MagData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_get_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MAG_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 7, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    s_MagData.x = res.res.sw_res[0];
    s_MagData.y = res.res.sw_res[1];
    s_MagData.z = res.res.sw_res[2];
    s_MagData.accuracy = res.res.ub_res[6];

    offsets[0] = s_MagData.x;
    offsets[1] = s_MagData.y;
    offsets[2] = s_MagData.z;

    SENSOR_N_LOG("offsets [%d] [%d] [%d] [%d]",
            offsets[0],offsets[1],offsets[2],s_MagData.accuracy);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_set_accuracy(int8_t accuracy)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    s_MagData.accuracy = accuracy;
    ret = sns_send_mag_set_offset_soft();

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_get_accuracy(int8_t* accuracy)
{
    int32_t ret = SNS_RC_OK;
    int32_t offsets[3];
    ret = sns_mag_get_offset(offsets);
    *accuracy = s_MagData.accuracy;
    return ret;

}

int32_t sns_mag_get_static_matrix(int32_t* static_matrix)
{
    int32_t ret = SNS_RC_OK;
    int16_t tmp_matrix[9];
    int i;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MAG_GET_STATIC_MATRIX;
    mutex_lock(&sensor_fifo_mutex);
    ret = sns_hostcmd(&cmd, &res, 19, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_GET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    tmp_matrix[0] = (int16_t)(res.res.ub_res[1]  | (res.res.ub_res[2] << 8 ));
    tmp_matrix[1] = (int16_t)(res.res.ub_res[3]  | (res.res.ub_res[4] << 8 ));
    tmp_matrix[2] = (int16_t)(res.res.ub_res[5]  | (res.res.ub_res[6] << 8 ));
    tmp_matrix[3] = (int16_t)(res.res.ub_res[7]  | (res.res.ub_res[8] << 8 ));
    tmp_matrix[4] = (int16_t)(res.res.ub_res[9]  | (res.res.ub_res[10] << 8));
    tmp_matrix[5] = (int16_t)(res.res.ub_res[11] | (res.res.ub_res[12] << 8));
    tmp_matrix[6] = (int16_t)(res.res.ub_res[13] | (res.res.ub_res[14] << 8));
    tmp_matrix[7] = (int16_t)(res.res.ub_res[15] | (res.res.ub_res[16] << 8));
    tmp_matrix[8] = (int16_t)(res.res.ub_res[17] | (res.res.ub_res[18] << 8));

    for(i = 0; i < sizeof(tmp_matrix) / sizeof(tmp_matrix[0]); i++){
        static_matrix[i] = (int32_t)tmp_matrix[i];
    }
    SENSOR_N_LOG("end - return[%d]", ret);

    return ret;

}

int32_t sns_mag_set_static_matrix(int32_t* static_matrix)
{
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16    = HC_MAG_SET_STATIC_MATRIX;
    cmd.prm.ub_prm[0]  = 0x01;
    cmd.prm.ub_prm[1]  = 0x00;
    cmd.prm.ub_prm[2]  = (int8_t)(static_matrix[0] & 0x000000ff);
    cmd.prm.ub_prm[3]  = (int8_t)(((static_matrix[0] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[4]  = (int8_t)(static_matrix[1] & 0x000000ff);
    cmd.prm.ub_prm[5]  = (int8_t)(((static_matrix[1] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[6]  = (int8_t)(static_matrix[2] & 0x000000ff);
    cmd.prm.ub_prm[7]  = (int8_t)(((static_matrix[2] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[8]  = (int8_t)(static_matrix[3] & 0x000000ff);
    cmd.prm.ub_prm[9]  = (int8_t)(((static_matrix[3] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[10] = (int8_t)(static_matrix[4] & 0x000000ff);
    cmd.prm.ub_prm[11] = (int8_t)(((static_matrix[4] >> 8) & 0x000000ff));
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = (int8_t)(static_matrix[5] & 0x000000ff);
    cmd.prm.ub_prm[3] = (int8_t)(((static_matrix[5] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[4] = (int8_t)(static_matrix[6] & 0x000000ff);
    cmd.prm.ub_prm[5] = (int8_t)(((static_matrix[6] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[6] = (int8_t)(static_matrix[7] & 0x000000ff);
    cmd.prm.ub_prm[7] = (int8_t)(((static_matrix[7] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[8] = (int8_t)(static_matrix[8] & 0x000000ff);
    cmd.prm.ub_prm[9] = (int8_t)(((static_matrix[8] >> 8) & 0x000000ff));
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]", ret);

    return ret;
}

int32_t sns_gyro_read_data(struct gyroscope *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT06, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

int32_t sns_gyro_set_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    struct gyroscope gyro;

    SENSOR_N_LOG("start");

    s_GyroData.x = GYRODATA_SIGN_COMVERT_12_32BIT(offsets[0]);
    SENSOR_N_LOG("set offset X[%d]",s_GyroData.x);
    
    s_GyroData.y = GYRODATA_SIGN_COMVERT_12_32BIT(offsets[1]);
    SENSOR_N_LOG("set offset Y[%d]",s_GyroData.y);
    
    s_GyroData.z = GYRODATA_SIGN_COMVERT_12_32BIT(offsets[2]);
    SENSOR_N_LOG("set offset Z[%d]",s_GyroData.z);

    gyro.x = s_GyroData.x*100;
    gyro.y = s_GyroData.y*100;
    gyro.z = s_GyroData.z*100;

    cmd.cmd.udata16 = HC_GYRO_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_GyroData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_GyroData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_GyroData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_GyroData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_GyroData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_GyroData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyro_get_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_GYRO_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    s_GyroData.x = res.res.sw_res[0];
    s_GyroData.y = res.res.sw_res[1];
    s_GyroData.z = res.res.sw_res[2];

    offsets[0] = s_GyroData.x;
    offsets[1] = s_GyroData.y;
    offsets[2] = s_GyroData.z;

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_maguncalib_read_data(struct mag_uncalib *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT0C, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Mag->x =MAGDATA_SIGN_COMVERT_12_32BIT(temp) + s_MagData.x;

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Mag->y = MAGDATA_SIGN_COMVERT_12_32BIT(temp) + s_MagData.y;

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Mag->z = MAGDATA_SIGN_COMVERT_12_32BIT(temp) + s_MagData.z;

        arg_Mag->cal_x = s_MagData.x;
        arg_Mag->cal_y = s_MagData.y;
        arg_Mag->cal_z = s_MagData.z;

        SENSOR_N_LOG("mag - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Mag - x[%04x] y[%04x] z[%04x] calx[%04x] caly[%04x] calz[%04x]",
                     arg_Mag->x,arg_Mag->y,arg_Mag->z,arg_Mag->cal_x,arg_Mag->cal_y,arg_Mag->cal_z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT06, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Gyro->x =GYRODATA_SIGN_COMVERT_12_32BIT(temp) + s_GyroData.x;

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Gyro->y = GYRODATA_SIGN_COMVERT_12_32BIT(temp) + s_GyroData.y;

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Gyro->z = GYRODATA_SIGN_COMVERT_12_32BIT(temp) + s_GyroData.z;

        arg_Gyro->cal_x = s_GyroData.x;
        arg_Gyro->cal_y = s_GyroData.y;
        arg_Gyro->cal_z = s_GyroData.z;

        SENSOR_N_LOG("gyro - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gyro - x[%04x] y[%04x] z[%04x] calx[%04x] caly[%04x] calz[%04x]",
                     arg_Gyro->x,arg_Gyro->y,arg_Gyro->z,arg_Gyro->cal_x,arg_Gyro->cal_y,arg_Gyro->cal_z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

void sns_mag_offset_read_data(struct geomagnetic *arg_Mag)
{

    SENSOR_N_LOG("start");

    arg_Mag->x = s_MagData.x;
    arg_Mag->y = s_MagData.y;
    arg_Mag->z = s_MagData.z;

    SENSOR_N_LOG("end - x[%04x] y[%04x] z[%04x]",arg_Mag->x ,arg_Mag->y, arg_Mag->z);
}

void sns_gyro_offset_read_data(struct gyroscope *arg_Gyro)
{
    SENSOR_N_LOG("start");

    arg_Gyro->x = s_GyroData.x;
    arg_Gyro->y = s_GyroData.y;
    arg_Gyro->z = s_GyroData.z;

    SENSOR_N_LOG("end - x[%04x] y[%04x] z[%04x]",arg_Gyro->x ,arg_Gyro->y, arg_Gyro->z);
}

int32_t sns_gravity_read_data(struct gravity *arg_Gravity)
{
    int32_t ret = SNS_RC_OK;
    int32_t temp;
    int32_t raw[3];
    int32_t xyz[3];
    int i,j;
    uint8_t buff[6];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT1A, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

int32_t sns_linacc_read_data(struct linear_acceleration *arg_linacc)
{
    int32_t ret = SNS_RC_OK;
    int32_t raw[3];
    int32_t xyz[3];
    int32_t temp;
    int i,j;
    uint8_t buff[6];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT20, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

int32_t sns_ori_read_data(struct orientation *arg_ori)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    uint8_t buff2[1];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT14, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);
    if(SNS_RC_OK == ret){
        ret = sns_device_read(RSLT2D, buff2, sizeof(buff2));
        SENSOR_N_LOG("sns_device_read[%d]",ret);
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

int32_t sns_rota_read_data(struct rotation_vector *arg_rota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[7];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT26, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

int32_t sns_gamerota_read_data(struct game_rotation_vector *arg_gamerota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[8];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT36, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

int32_t sns_magrota_read_data(struct geomagnetic_rotation_vector *arg_magrota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[8];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT2E, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

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

#define PRESSURE_DEFAULT_OFFSET (30000u)
int32_t sns_pressure_read_data(struct pressure *arg_pressure)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[3];
    int32_t offset = PRESSURE_DEFAULT_OFFSET;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT2C, buff, 1);
    ret = sns_device_read(RSLT12, buff+1, 2);
    SENSOR_N_LOG("sns_device_read[%d]",ret);
    SENSOR_N_LOG("pressure - [%02x][%02x][%02x]",buff[2],buff[1],buff[0]);

    if(SNS_RC_OK == ret){
        arg_pressure->pressure = (int32_t)((buff[0] & 0x01) | (buff[1] << 1) | (buff[2] << 9));
        arg_pressure->pressure += offset;

        SENSOR_N_LOG("arg_pressure - pressure[%d] offset[%d]",
                     arg_pressure->pressure, offset);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_pressure_base_val(bool req, DailysSetBaseParam *DailysBaseParam)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
    SENSOR_N_LOG("start");

    if(req == true){
        cmd.cmd.udata16 = HC_PRE_SET_BASE_VAL;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = DailysBaseParam->m_SetMode;
        cmd.prm.ub_prm[2] = (DailysBaseParam->m_BasePress & 0xFF);
        cmd.prm.ub_prm[3] = ((DailysBaseParam->m_BasePress >> 8) & 0xFF);
        cmd.prm.ub_prm[4] = ((DailysBaseParam->m_BasePress >> 16) & 0xFF);
        cmd.prm.ub_prm[5] = (DailysBaseParam->m_BaseHeight & 0xFF);
        cmd.prm.ub_prm[6] = ((DailysBaseParam->m_BaseHeight >> 8) & 0xFF);
        cmd.prm.ub_prm[7] = ((DailysBaseParam->m_BaseHeight >> 16) & 0xFF);
        cmd.prm.ub_prm[8] = ((DailysBaseParam->m_BaseHeight >> 24) & 0xFF);

        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_ALL, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)){
            SENSOR_ERR_LOG("end HC_PRE_SET_BASE_VAL err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }else{
        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_PRE_GET_BASE_VAL;
        ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if((SNS_RC_OK == ret) && (0 == res.err.udata16)){
            DailysBaseParam->m_BasePress =  (uint32_t)((res.res.ub_res[4] << 16) | res.res.uw_res[1]);
            DailysBaseParam->m_BaseHeight = (uint32_t)((res.res.ub_res[8] << 24) | (res.res.uw_res[3] << 8) | res.res.ub_res[5]);
            SENSOR_N_LOG("HC_PRE_GET_BASE_VAL get, m_BasePress:%d, m_BaseHeight:%d",DailysBaseParam->m_BasePress,DailysBaseParam->m_BaseHeight);
        }else{
            SENSOR_ERR_LOG("end HC_PRE_GET_BASE_VAL err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    SENSOR_N_LOG("end");

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

void sns_pressure_set_cal_ofs(int32_t offset)
{
    atomic_set(&g_pressure_cal_offset, offset);
}

int32_t sns_motion_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

void sns_set_pedo_param(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iVehiType)
{
    SENSOR_N_LOG("start");

    atomic_set(&g_nWeight,arg_iWeight);
    atomic_set(&g_nStepWide,arg_iStepWide);
    atomic_set(&g_nVehiType,arg_iVehiType);

    if (g_micon_pedo_status) {
        sns_dailys_send_ope_param(true);
    }
    SENSOR_N_LOG("end - Weight[%d] StepWide[%d] VehiType[%d]",
                  atomic_read(&g_nStepWide), atomic_read(&g_nWeight), atomic_read(&g_nVehiType));
}

static int32_t sns_dailys_send_ope_param(bool enable)
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
    cmd.prm.ub_prm[2] = atomic_read(&g_nStepWide);
    cmd.prm.ub_prm[3] = (atomic_read(&g_nWeight) & 0xFF);
    cmd.prm.ub_prm[4] = ((atomic_read(&g_nWeight) >> 8) & 0xFF);
    cmd.prm.ub_prm[5] = 0x00;
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

static int32_t sns_dailys_send_latency_option(enum sensor_e_type type, bool enable)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    bool       enable_pedo = sns_ctrl_param.input_param.inputs[SENSOR_EXT_PEDO].enable;
    bool       enable_baro = sns_ctrl_param.input_param.inputs[SENSOR_EXT_BARO].enable;
    int32_t    period_pedo = sns_ctrl_param.input_param.inputs[SENSOR_EXT_PEDO].sampling_period_ms/30;
    int32_t    period_baro = sns_ctrl_param.input_param.inputs[SENSOR_EXT_BARO].sampling_period_ms/30;
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

int32_t sns_dailys_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start arg_iStart[%d]", arg_iStart);

    param = arg_iStart ? HC_VALID : HC_INVALID;

    ret = sns_dailys_send_ope_param(arg_iStart);
    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_dailys_send_ope_param err ret[%d]",ret);
        return SNS_RC_ERR;
    }

    ret = sns_dailys_send_latency_option(SENSOR_EXT_PEDO, arg_iStart);
    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_dailys_send_latency_option err ret[%d]",ret);
        return SNS_RC_ERR;
    }

    if (arg_iStart) {
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
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
#endif
        cmd.cmd.udata16 = HC_DST_SET_EB_SET;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
        cmd.prm.ub_prm[5] = 0x01;
#else
        cmd.prm.ub_prm[5] = 0x00;
#endif
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
        cmd.prm.ub_prm[6] = 0x00;
        cmd.prm.ub_prm[7] = 0x00;
        cmd.prm.ub_prm[8] = 0x00;

        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_SET_EB_SET err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        msleep(35);
    }

    g_micon_pedo_status = arg_iStart;

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_baro_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start arg_iStart[%d]", arg_iStart);

    param = arg_iStart ? HC_VALID : HC_INVALID;

    if (arg_iStart) {
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

    ret = sns_dailys_send_latency_option(SENSOR_EXT_BARO, arg_iStart);
    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_dailys_send_latency_option err ret[%d]",ret);
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

    if (arg_iStart) {
        if (!g_micon_pedo_status && !g_micon_baro_status) {
            g_eventbuf_cleartime = GET_CURRENT_TIMESTAMP_NS();
            SENSOR_N_LOG("update eventbuf cleartime [%lld]", g_eventbuf_cleartime);
        }
    }
    g_micon_baro_status = arg_iStart;

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t sns_get_pedo_eb_data(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO1;
    ret = sns_hostcmd(&cmd, &res, 2, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->LogTimeStamp3sec  = res.res.uw_res[0];

        SENSOR_N_LOG("LogTimeStamp3sec   [%u]",arg_Pedo->LogTimeStamp3sec);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_EB_INFO1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO2;
    ret = sns_hostcmd(&cmd, &res, 9, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO3;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO4;
    ret = sns_hostcmd(&cmd, &res, 3, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO5;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_EB_INFO6;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

int32_t sns_get_pedo_data(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_PEDO1;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_PEDO2;
    ret = sns_hostcmd(&cmd, &res, 11, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_PEDO5;
    ret = sns_hostcmd(&cmd, &res, 12, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_RUN1;
    ret = sns_hostcmd(&cmd, &res, 9, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_RUN2;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunCal      = res.res.ud_res[0];
        arg_Pedo->usRunExercise = res.res.ud_res[1];

        SENSOR_N_LOG("Run Calorie(kcal)  [%u]",arg_Pedo->usRunCal);
        SENSOR_N_LOG("Run Exercise(0.1Ex)[%u]",arg_Pedo->usRunExercise);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_RUN2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    ret = sns_get_pedo_eb_data(arg_Pedo);

    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_get_pedo_data_ext ret[%x]",ret);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_vehi_data(struct vehicle *arg_Vehi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_TRANS1;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_TRANS2;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_TRANS6;
    ret = sns_hostcmd(&cmd, &res, 12, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

int32_t sns_iwifi_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_EXEC_IWIFI;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_IWIFI err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_iwifi_data(struct iwifi *arg_IWifi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_INTELLI_WIFI;
    ret = sns_hostcmd(&cmd, &res, 2, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_IWifi->usPedoStatus = res.res.ub_res[0];
        arg_IWifi->usVehiStatus = res.res.ub_res[1];

        SENSOR_N_LOG("Pedo Status [%u]",arg_IWifi->usPedoStatus);
        SENSOR_N_LOG("Vehi Status [%u]",arg_IWifi->usVehiStatus);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_INTELLI_WIFI err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_iwifi_set_info(bool req, DailysSetIWifiParam *DailysIWifiParam)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_DST_SET_IWIFI_INFO;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (DailysIWifiParam->m_nPedoStartStep & 0xFF);
    cmd.prm.ub_prm[2] = (DailysIWifiParam->m_nPedoEndTime & 0xFF);
    cmd.prm.ub_prm[3] = ((DailysIWifiParam->m_nPedoEndTime >> 8) & 0xFF);
    cmd.prm.ub_prm[4] = (DailysIWifiParam->m_nVehiStartTime & 0xFF);
    cmd.prm.ub_prm[5] = ((DailysIWifiParam->m_nVehiStartTime >> 8) & 0xFF);
    cmd.prm.ub_prm[6] = (DailysIWifiParam->m_nVehiEndTime & 0xFF);
    cmd.prm.ub_prm[7] = ((DailysIWifiParam->m_nVehiEndTime >> 8) & 0xFF);

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_IWIFI_INFO err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return SNS_RC_OK;
}

int32_t sns_pedom_clear(int32_t clear_req)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;
    int32_t tmp = 0;
    int32_t tmpA = 0;

    SENSOR_N_LOG("start");

    tmp |= (clear_req & DAILYS_CLEAR_SHOCK_STATE)   ? 0x01:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_STOP_STATE)    ? 0x02:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_OUT_STATE)     ? 0x04:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_TRAIN_STATE)   ? 0x08:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_MOTION_STATE)  ? 0x10:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_STEP_COUNTER)  ? 0x20:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_STEP_TIMESTMP) ? 0x40:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_BATCH_TIMER)   ? 0x80:0x00;
    tmpA |= (clear_req & DAILYS_CLEAR_VH_STATE)     ? 0x01:0x00;
    tmpA |= (clear_req & DAILYS_CLEAR_EB_ALL)       ? 0x02:0x00;
    tmpA |= (clear_req & DAILYS_CLEAR_DAILYS_DAY)   ? 0x04:0x00;

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

int32_t sns_vhdetect_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

int32_t sns_get_vhdetect_data(struct vhdetect *arg_VHdetect)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_DST_GET_VH;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

int32_t sns_kc_motion_walk_start_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

int32_t sns_kc_motion_walk_stop_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

int32_t sns_get_kc_motion_train_data(struct kc_motion_train *arg_Train)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MOT_GET_TRAIN_INFO;
    ret = sns_hostcmd(&cmd, &res, 11, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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

int32_t sns_kc_motion_vehicle_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

int32_t sns_kc_motion_bringup_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

int32_t sns_get_kc_motion_bringup_data(struct kc_motion_bringup_data *arg_Bringup)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MOT_GET_BRINGUP_INFO;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
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


int32_t sns_uwater_start(bool arg_iStart)
{
    int32_t    ret = SNS_RC_OK;

    HostCmd cmd;
    HostCmdRes res;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
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

int32_t sns_get_uwater_data(struct uwater_detect_info *arg_uwdInfo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_UWD_GET_WATER_INFO;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_uwdInfo->uwater_detected = res.res.ub_res[0];
        SENSOR_N_LOG("uwater_detected [%u]",arg_uwdInfo->uwater_detected);
    }else{
        SENSOR_ERR_LOG("end %s err[%x]", __func__, res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_uwater_clear(void)
{
    int32_t ret = SNS_RC_OK;

    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_UWD_GET_EXEC;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end sns_uwater_clear err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_UWD_SET_EXEC;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = res.res.ub_res[1];
    cmd.prm.ub_prm[2] = res.res.ub_res[2];
    cmd.prm.ub_prm[3] = res.res.ub_res[3];
    cmd.prm.ub_prm[4] = res.res.ub_res[4];
    cmd.prm.ub_prm[5] = HC_VALID;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end sns_uwater_clear err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}



int32_t sns_set_dev_param(void)
{
    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    const int16_t static_matrix[9] = {
        9334,-1158,53,118,10536,-193,-1196,-614,10174
    };
#endif

#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    cmd.cmd.udata16 = HC_ACC_SET_CONV_AXIS;
    cmd.prm.ub_prm[0] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_CONV_AXIS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    cmd.cmd.udata16 = HC_ACC_SET_AUTO_CALIB;
    cmd.prm.ub_prm[0] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_AUTO_CALIB err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    cmd.cmd.udata16 = HC_GYRO_SET_CONV_AXIS;
    cmd.prm.ub_prm[0] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_CONV_AXIS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    cmd.cmd.udata16 = HC_EC_SET_CONV_AXIS;
    cmd.prm.ub_prm[0] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_EC_SET_CONV_AXIS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif

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
        return SNS_RC_ERR;
    }

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    cmd.cmd.udata16 = HC_MAG_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_MagData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_MagData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_MagData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_MagData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_MagData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_MagData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    sns_send_mag_set_offset_soft();
#endif

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    cmd.cmd.udata16 = HC_GYRO_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_GyroData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_GyroData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_GyroData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_GyroData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_GyroData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_GyroData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif

    ret = sns_acc_set_auto_cal_offset_internal();

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
    cmd.prm.ub_prm[0]  = 0x01;
    cmd.prm.ub_prm[1]  = 0x00;
    cmd.prm.ub_prm[2]  = (uint8_t)(static_matrix[0] & 0x000000ff);
    cmd.prm.ub_prm[3]  = (uint8_t)(((static_matrix[0] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[4]  = (uint8_t)(static_matrix[1] & 0x000000ff);
    cmd.prm.ub_prm[5]  = (uint8_t)(((static_matrix[1] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[6]  = (uint8_t)(static_matrix[2] & 0x000000ff);
    cmd.prm.ub_prm[7]  = (uint8_t)(((static_matrix[2] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[8]  = (uint8_t)(static_matrix[3] & 0x000000ff);
    cmd.prm.ub_prm[9]  = (uint8_t)(((static_matrix[3] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[10] = (uint8_t)(static_matrix[4] & 0x000000ff);
    cmd.prm.ub_prm[11] = (uint8_t)(((static_matrix[4] >> 8) & 0x000000ff));
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = (uint8_t)(static_matrix[5] & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)(((static_matrix[5] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[4] = (uint8_t)(static_matrix[6] & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)(((static_matrix[6] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[6] = (uint8_t)(static_matrix[7] & 0x000000ff);
    cmd.prm.ub_prm[7] = (uint8_t)(((static_matrix[7] >> 8) & 0x000000ff));
    cmd.prm.ub_prm[8] = (uint8_t)(static_matrix[8] & 0x000000ff);
    cmd.prm.ub_prm[9] = (uint8_t)(((static_matrix[8] >> 8) & 0x000000ff));
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */
    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

#ifndef CONFIG_USE_GPIOMUX
static int sns_pinctrl_select(bool on)
{

    SENSOR_N_LOG("start pins_state[%s]",
                on ? "sensor_micon_active" : "sensor_micon_sleep");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sns_pinctrl_init(struct spi_device *client)
{

    SENSOR_N_LOG("start");

    s_sns_pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(s_sns_pinctrl)) {
        SENSOR_ERR_LOG("Target does not use pinctrl. [%ld]", PTR_ERR(s_sns_pinctrl));
        s_sns_pinctrl = NULL;
        return -ENODEV;
    }

    s_gpio_state_active = pinctrl_lookup_state(s_sns_pinctrl, "sensor_micon_active");
    if (IS_ERR_OR_NULL(s_gpio_state_active)) {
        SENSOR_ERR_LOG("Can not get sns active pinstate. [%ld]", PTR_ERR(s_gpio_state_active));
        s_sns_pinctrl = NULL;
        return -ENODEV;
    }

    s_gpio_state_suspend = pinctrl_lookup_state(s_sns_pinctrl, "sensor_micon_sleep");
    if (IS_ERR_OR_NULL(s_gpio_state_suspend)) {
        SENSOR_ERR_LOG("Can not get sns sleep pinstate. [%ld]", PTR_ERR(s_gpio_state_suspend));
        s_sns_pinctrl = NULL;
        return -ENODEV;
    }

    SENSOR_N_LOG("end");
    return 0;
}
#endif /* CONFIG_USE_GPIOMUX */

static int32_t sns_initialize_exe( void )
{
    uint8_t fw_ver[4];
    uint8_t reg;
    int32_t cnt;
    int32_t cnt_hw_reset = 0;
    bool  status_ok = false;
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    DISABLE_IRQ;

    sns_workqueue_init();

    atomic_set(&g_FusionState,SNS_OFF);
    atomic_set(&g_SnsTaskState,SNS_OFF);
    atomic_set(&g_AppTaskState,SNS_OFF);
    atomic_set(&g_FusTaskState,SNS_OFF);
    g_micon_pedo_status = false;
    g_micon_baro_status = false;
    g_vib_interlocking = 0x00;

    g_bDevIF_Error = false;
    g_micon_error = false;

    if (!gpio_is_valid(g_micon_bdata.rst_gpio)) {
        SENSOR_ERR_LOG("gpio is invalid");
        return SNS_RC_ERR;
    }

    while (1) {
        gpio_set_value(g_micon_bdata.rst_gpio, 0);
        udelay(500);
        gpio_set_value(g_micon_bdata.rst_gpio, 1);

        msleep(20);

#ifdef CONFIG_USE_GPIOMUX
        msm_gpiomux_install(msm_spi_sens_configs, ARRAY_SIZE(msm_spi_sens_configs));
#else
        sns_pinctrl_select(true); //cannnot operate pinctrl setting from client.
#endif
        reg = 0xFF;
        cnt = 0;
        while (1) {
            sns_device_read(STATUS, &reg, sizeof(reg));
            if (reg == 0x00) {
                status_ok = true;
                break;
            }
            if (reg == 0xFD && cnt >= STATUS_READ_RETRY_NUM_2) {
                break;
            }
            if (cnt >= STATUS_READ_RETRY_NUM) {
                SENSOR_ERR_LOG("STATUS read TimeOut[%x]",reg);
                return SNS_RC_ERR_TIMEOUT;
            }

            ++cnt;
            msleep(10);
        }

        if (!status_ok) {
            cnt_hw_reset++;
            if (cnt_hw_reset > RESET_RETRY_NUM) {
                SENSOR_ERR_LOG("STATUS read retry count over[%d]",cnt_hw_reset);
                return SNS_RC_ERR_TIMEOUT;
            }
            SENSOR_ERR_LOG("STATUS read fail reg[%x] 10ms_wait_cnt[%d] hwreset_cnt[%d]",reg,cnt,cnt_hw_reset);
            continue;
        }

        break;
    }

    reg = 0x04;
    sns_device_write(CFG, &reg, sizeof(reg));
    {
        uint8_t data = 0;
        sns_device_read(CFG, &data, 1);
        if (data != reg) {
            SENSOR_ERR_LOG("CFG unable to write [%x]",data);
            return SNS_RC_ERR;
        }
        SENSOR_N_LOG("CFG read [%x]",data);
    }

    reg = 0x00;
    sns_device_write(INTMASK0, &reg, sizeof(reg));
    sns_device_write(INTMASK1, &reg, sizeof(reg));

    ENABLE_IRQ;

    cmd.cmd.udata16 = HC_MCU_SET_PDIR;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = 0x01;
    cmd.prm.ub_prm[3] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_PDIR err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x11;
    cmd.prm.ub_prm[1] = 0x11;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_PCON err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    ret = sns_check_sensor();
    if(ret != 0) {
        SENSOR_ERR_LOG("sns_check_sensor[%d]",ret);
    }

    ret = sns_get_fw_version(fw_ver);
    g_nFWVersion = SNESOR_COM_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK ){
        g_nFWVersion = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("sns_get_fw_version[%d]",ret);
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

int32_t sns_initialize( void )
{
    int32_t ret;

    SENSOR_N_LOG("start");

    ret = sns_initialize_exe();
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Failed sns_initialize_exe[%d]",ret);
        ret = sns_initialize_exe();
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("Failed sns_initialize_exe[%d]",ret);
            g_micon_error = true;
            return ret;
        }
    }

    SENSOR_N_LOG("end - %d", ret);

    return ret;
}

static irqreturn_t sns_irq_handler(int32_t irq, void *dev_id)
{

    SENSOR_N_LOG("start");

    if( irq != g_nIntIrqNo ){
        return IRQ_NONE;
    }

    DISABLE_IRQ;
    if( sns_workqueue_create(sns_wq_int, sns_int_work_func, WORK_OPT_NONE, SENSOR_MAX) != SNS_RC_OK){
        ENABLE_IRQ;
    }else{
        SENSOR_N_LOG("### --> s_tWork_Int");
    }

    SENSOR_N_LOG("end - IRQ_HANDLED");

    return IRQ_HANDLED;
}

static void sns_int_work_func(struct work_struct *work)
{
    Long lreg;

    u_int8_t sreg30, sreg31, sreg32, sreg33;
    u_int8_t sreg34, sreg35, sreg36, sreg37;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_state_irq_mutex);

    sns_device_read(ERROR0, lreg.udata8, 4);

    if(lreg.udata16[1] == 0){
        sns_workqueue_delete(work) ;
        ENABLE_IRQ;
        goto done;
    }

    SENSOR_N_LOG("### INTREQ0/1[%x],ERROR0/1[%x]",
                                    lreg.udata16[0],lreg.udata16[1]);

    g_lastCmdError = lreg.udata16[0];

    if(lreg.udata16[1] == 0xFFFF){
        SENSOR_ERR_LOG("### sns_int_work_func MiconWDT Error[%x][%x]",
                                    lreg.udata16[0],lreg.udata16[1]);
        g_bDevIF_Error = true;
        sns_workqueue_delete(work);

        DISABLE_IRQ;
        goto done;
    }

    if(lreg.udata16[1] == INTREQ_ERROR){
        SENSOR_ERR_LOG("### sns_int_work_func Error[%x][%x]",
                                    lreg.udata16[0],lreg.udata16[1]);
        mutex_lock(&s_tDataMutex);
        g_nIntIrqFlg |= INTREQ_ERROR;
        mutex_unlock(&s_tDataMutex);

        wake_up_interruptible(&s_tWaitInt);

        sns_workqueue_delete(work);
        ENABLE_IRQ;
        goto done;
    }

    if(lreg.udata16[1] & INTREQ_HOST_CMD){
        if(!(g_nIntIrqFlg & INTREQ_HOST_CMD)){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg |= INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            wake_up_interruptible(&s_tWaitInt);
        }
        SENSOR_N_LOG("### INTREQ_HOST_CMD[%x]",g_nIntIrqFlg);
    }

    if(lreg.udata16[1] & (INTREQ_ACC | INTREQ_MAG | INTREQ_GYRO | INTREQ_FUSION | INTREQ_KCLOG)){
        SENSOR_N_LOG("### INTREQ_ACC | INTREQ_MAG | INTREQ_GYRO | INTREQ_FUSION | INTREQ_KCLOG");
        if(lreg.udata16[1] & INTREQ_KCLOG){
            atomic_set(&g_MiconDebug, true);
        }
        sns_workqueue_create(sns_wq, sns_int_app_work_func, WORK_OPT_INT, SENSOR_MAX);
    }


    if(lreg.udata16[1]  & INTREQ_NMI){
        SENSOR_ERR_LOG("### INTREQ_NMI");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));
        memset( &sreg35, 0x00, sizeof(sreg35));
        memset( &sreg36, 0x00, sizeof(sreg36));
        memset( &sreg37, 0x00, sizeof(sreg37));

        sns_device_read(RSLT30, &sreg30, 1);
        sns_device_read(RSLT31, &sreg31, 1);
        sns_device_read(RSLT32, &sreg32, 1);
        sns_device_read(RSLT33, &sreg33, 1);
        sns_device_read(RSLT34, &sreg34, 1);
        sns_device_read(RSLT35, &sreg35, 1);
        sns_device_read(RSLT36, &sreg36, 1);
        sns_device_read(RSLT37, &sreg37, 1);



        SENSOR_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        SENSOR_ERR_LOG("### result reg 34:[%x], 35:[%x], 36:[%x], 37:[%x]"
                                         ,sreg34, sreg35, sreg36, sreg37);
    }

    if(lreg.udata16[1]  & INTREQ_EXCP){
        SENSOR_ERR_LOG("### INTREQ_EXCP");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));

        sns_device_read(RSLT30, &sreg30, 1);
        sns_device_read(RSLT31, &sreg31, 1);
        sns_device_read(RSLT32, &sreg32, 1);
        sns_device_read(RSLT33, &sreg33, 1);
        sns_device_read(RSLT34, &sreg34, 1);

        SENSOR_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        SENSOR_ERR_LOG("### result reg 34:[%02x]",sreg34);
    }


    if(lreg.udata16[1]  & INTREQ_KCLOG){
        SENSOR_ERR_LOG("### INTREQ_KCLOG");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));

        sns_device_read(RSLT30, &sreg30, 1);
        sns_device_read(RSLT31, &sreg31, 1);
        sns_device_read(RSLT32, &sreg32, 1);
        sns_device_read(RSLT33, &sreg33, 1);
        sns_device_read(RSLT34, &sreg34, 1);
 
        SENSOR_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        SENSOR_ERR_LOG("### result reg 34:[%02x]",sreg34);


    }
    sns_workqueue_delete(work) ;
    ENABLE_IRQ;

done:
    mutex_unlock(&sensor_state_irq_mutex);
    SENSOR_N_LOG("end");

    return;
}

static void sns_int_app_work_func(struct work_struct *work)
{
    HostCmd            cmd;
    HostCmdRes         res;
    int32_t            ret = SNS_RC_OK;
    uint8_t            pedo_detect = 0;
    uint8_t            vehi_detect = 0;
    uint8_t            ps_detect = 0;
    uint8_t            wifi_detect = 0;
    uint8_t            apl_detect = 0;
    uint8_t            apl_detect2 = 0;
    uint8_t            apl_detect3 = 0;
    uint8_t            eb_punctuation_done = false;
    Word               log_size;
    uint8_t            count_10, count, offset, i, j;
    char               strLog[50], temp[4];
    SNS_WorkQueue      *sns_work = container_of(work, SNS_WorkQueue, work);
    int32_t            option = sns_work->option;
    enum sensor_e_type type = sns_work->type;

    SENSOR_N_LOG("start");
    if(atomic_read(&g_ShutdownFlg) == true){
        SENSOR_ERR_LOG("### Shutdown return !!");
        return ;
    }
    mutex_lock(&sensor_state_app_mutex);

    if(option == WORK_OPT_INT) {
        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_DST_GET_INT_DETAIL;
        ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("###  Error HC_DST_GET_INT_DETAIL");
            sns_workqueue_delete(work) ;
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
            sensor_interrupt(SENSOR_EXT_PEDO,0);
        }

        if(res.res.ub_res[1] == 0x01){
            vehi_detect = 0x01;
            SENSOR_N_LOG("### Vehicle Detected !!");
            if(!eb_punctuation_done)
                sensor_flush_event_buffer();
            eb_punctuation_done = true;
            sensor_interrupt(SENSOR_EXT_PEDO,0);
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
            sensor_interrupt(SENSOR_EXT_IWIFI,0);
        }

        if(res.res.ub_res[7] & 0x02){
            apl_detect |= 0x02;
            SENSOR_N_LOG("### Walk Stop Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_WALK_STOP,0);
        }

        if(res.res.ub_res[7] & 0x04){
            apl_detect |= 0x04;
            SENSOR_N_LOG("### Bringup Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_BRINGUP,0);
        }

        if(res.res.ub_res[7] & 0x08){
            apl_detect |= 0x08;
            SENSOR_N_LOG("### Train Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_TRAIN,0);
        }

        if(res.res.ub_res[7] & 0x10){
            apl_detect |= 0x10;
            SENSOR_N_LOG("### First Train Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_TRAIN,0);
        }

        if(res.res.ub_res[7] & 0x20){
            apl_detect |= 0x20;
            SENSOR_N_LOG("### Motion Detected !!");
            sensor_interrupt(SENSOR_SGNFCNT_MTN,0);
        }

        if(res.res.ub_res[7] & 0x40){
            apl_detect |= 0x40;
            SENSOR_N_LOG("### Batch Timer !!");
            sns_logging_punctuation(LOGGING_TRIGGER_TIMEOUT);
        }

        if(res.res.ub_res[7] & 0x80){
            apl_detect |= 0x80;
            SENSOR_N_LOG("### VH Detected !!");
            sensor_interrupt(SENSOR_EXT_VH,0);
        }

        if(res.res.ub_res[8] & 0x01){
            apl_detect2 |= 0x01;
            SENSOR_N_LOG("### Walk Start Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_WALK_START,0);
        }

        if(res.res.ub_res[8] & 0x02){
            apl_detect2 |= 0x02;
            SENSOR_N_LOG("### Vehicle Start Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_VEHICLE,0);
        }

        if(res.res.ub_res[9] & 0x01){
            apl_detect3 |= 0x01;
            SENSOR_N_LOG("### Acc AutoCalibration Detected !!");
            sensor_interrupt(SENSOR_ACC_AUTO_CAL,0);
        }

        if(res.res.ub_res[9] & 0x02){
            apl_detect3 |= 0x02;
            SENSOR_N_LOG("### underwater Detected !!");
            sensor_interrupt(SENSOR_UNDERWATER_DETECT,0);
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
        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_MCU_GET_INT_DETAIL;
        cmd.prm.ub_prm[0] = 0xc0;
        cmd.prm.ub_prm[1] = 0x40;
        ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("###  Error HC_MCU_GET_INT_DETAIL");
            sns_workqueue_delete(work) ;
            goto done;
        }

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
        if(res.res.ub_res[0] == 0x01){
            SENSOR_N_LOG("### Geomagnetic Detected !!");
            mutex_lock(&sensor_fifo_mutex);
            cmd.cmd.udata16 = HC_MAG_GET_DATA;
            ret = sns_hostcmd(&cmd, &res, 7, EXE_HOST_ALL, READ_FIFO);
            mutex_unlock(&sensor_fifo_mutex);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
                sns_workqueue_delete(work);
                goto done;
            } else {
                s_MagData.x = res.res.sw_res[0];
                s_MagData.y = res.res.sw_res[1];
                s_MagData.z = res.res.sw_res[2];
                s_MagData.accuracy = res.res.ub_res[6];

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d] a[%d]",s_MagData.x,s_MagData.y,s_MagData.z,s_MagData.accuracy);
            }
        }
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE

        if(res.res.ub_res[2] == 0x01){
            SENSOR_N_LOG("### Gyroscope Detected !!");
            mutex_lock(&sensor_fifo_mutex);
            cmd.cmd.udata16 = HC_GYRO_GET_DATA;
            ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
            mutex_unlock(&sensor_fifo_mutex);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
                sns_workqueue_delete(work);
                goto done;
            } else {
                s_GyroData.x = res.res.sw_res[0];
                s_GyroData.y = res.res.sw_res[1];
                s_GyroData.z = res.res.sw_res[2];

                SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",s_GyroData.x,s_GyroData.y,s_GyroData.z);
            }
        }
#endif
        if(res.res.ub_res[4] == 0x01){
            SENSOR_N_LOG("### Buffer 512Byte !!");
            sns_logging_punctuation(LOGGING_TRIGGER_BUF512);
        }

        if(res.res.ub_res[4] == 0x02){
            SENSOR_N_LOG("### Buffer Full !!");
            sns_logging_punctuation(LOGGING_TRIGGER_BUFFULL);
        }
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
            sns_workqueue_delete(work) ;
            goto done;
        }


        if(atomic_read(&g_MiconDebug) == true){
            SENSOR_ERR_LOG("###Micon Debug ");

            mutex_lock(&sensor_fifo_mutex);
            cmd.cmd.udata16 = KC_LOG_READ;
            sns_hostcmd(&cmd, &res, KC_LOG_SIZE, EXE_HOST_ALL, READ_FIFO);
            mutex_unlock(&sensor_fifo_mutex);
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

            sns_workqueue_delete(work);
            goto done;
        }
    } else if (option == WORK_OPT_FLUSH) {
        sns_logging_punctuation(LOGGING_TRIGGER_FLUSH);
        sns_iio_report_flush(type, 0);
    }

    sns_workqueue_delete(work) ;

done:
    mutex_unlock(&sensor_state_app_mutex);
    SENSOR_N_LOG("end");

    return;
}

static int32_t sns_get_gpio_info( struct spi_device *client )
{
	int value = 0;
	int ret = SNS_RC_OK;
	struct device_node *np = client->dev.of_node;
	
    SENSOR_N_LOG("start");

	value = of_get_named_gpio_flags(np, "LAPIS,rst-gpio", 0, NULL);
	if(value < 0) {
		SENSOR_ERR_LOG("LAPIS,rst-gpio property not found");
		ret = SNS_RC_ERR;
	}
	g_micon_bdata.rst_gpio = value;
    SENSOR_N_LOG("rst-gpio[%d]", g_micon_bdata.rst_gpio);

	value = of_get_named_gpio_flags(np, "LAPIS,int-gpio", 0, NULL);
	if(value < 0) {
		SENSOR_ERR_LOG("LAPIS,int-gpio property not found");
		ret = SNS_RC_ERR;
	}
	g_micon_bdata.int_gpio = value;
    SENSOR_N_LOG("int-gpio[%d]", g_micon_bdata.int_gpio);

	value = of_get_named_gpio_flags(np, "LAPIS,brmp-gpio", 0, NULL);
	if(value < 0) {
		SENSOR_ERR_LOG("LAPIS,brmp-gpio property not found");
		ret = SNS_RC_ERR;
	}
	g_micon_bdata.brmp_gpio = value;
    SENSOR_N_LOG("brmp-gpio[%d]", g_micon_bdata.brmp_gpio);
	
    SENSOR_N_LOG("end");

	return ret;
}

static int32_t sns_gpio_init( struct spi_device *client )
{
    int32_t ret;

    SENSOR_N_LOG("start");
	
	ret = sns_get_gpio_info(client);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to get_gpio_info [%d]",ret);
        return ret;
    }

	ret = gpio_request(g_micon_bdata.rst_gpio, GPIO_RESET_NAME);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_request g_micon_bdata.rst_gpio[%d]",ret);
        return ret;
    }

    ret = gpio_direction_output(g_micon_bdata.rst_gpio, 1);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_direction_output g_micon_bdata.rst_gpio[%d]",ret);
        goto ERROR_RST;
    }

    ret = gpio_request(g_micon_bdata.brmp_gpio, GPIO_BRMP_NAME);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_request g_micon_bdata.brmp_gpio[%d]",ret);
        goto ERROR_RST;
    }

    ret = gpio_direction_output(g_micon_bdata.brmp_gpio, 0);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_direction_output g_micon_bdata.brmp_gpio[%d]",ret);
        goto ERROR_BRMP;
    }

    g_nIntIrqNo = gpio_to_irq(g_micon_bdata.int_gpio);
    atomic_set(&g_bIsIntIrqEnable, true);
    ret = gpio_request(g_micon_bdata.int_gpio, GPIO_INT_NAME);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_request g_micon_bdata.int_gpio[%d]",ret);
        goto ERROR_BRMP;
    }

    ret = gpio_direction_input(g_micon_bdata.int_gpio);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_direction_output g_micon_bdata.int_gpio[%d]",ret);
        goto ERROR_INT;
    }

    ret = request_any_context_irq(g_nIntIrqNo, sns_irq_handler, IRQF_TRIGGER_LOW, GPIO_INT_NAME, NULL);
    if(ret < 0) {
        SENSOR_ERR_LOG("Failed request_any_context_irq[%d]",ret);
        goto ERROR_INT;
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;

ERROR_INT:
    gpio_free(g_micon_bdata.int_gpio);
ERROR_BRMP:
    gpio_free(g_micon_bdata.brmp_gpio);
ERROR_RST:
    gpio_free(g_micon_bdata.rst_gpio);
    return SNS_RC_ERR;
}

int32_t sns_check_sensor(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MCU_GET_EX_SENSOR;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("CMD Error <HC_MCU_GET_EX_SENSOR>[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    if( !(res.res.ub_res[0] & 0x01) ){
        SENSOR_ERR_LOG("Acc Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    if( !(res.res.ub_res[0] & 0x20) ){
        SENSOR_ERR_LOG("Pressure Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    if( !(res.res.ub_res[0] & 0x40) ){
        SENSOR_ERR_LOG("Mag Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    if( !(res.res.ub_res[0] & 0x80) ){
        SENSOR_ERR_LOG("Gyro Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }
#endif

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

int32_t sns_get_fw_version(uint8_t *arg_iData)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;

    ret = sns_hostcmd(&cmd, &res, 8, (EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER), READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("FW Version Get Error[%d][%x]",cmd.prm.ub_prm[0], res.err.udata16);
        return SNS_RC_ERR;
    }

    arg_iData[0] = res.res.ub_res[0];
    arg_iData[1] = res.res.ub_res[1];
    arg_iData[2] = res.res.ub_res[2];
    arg_iData[3] = res.res.ub_res[3];

    SENSOR_N_LOG("FW Version[%02x][%02x][%02x][%02x]",arg_iData[0], arg_iData[1], arg_iData[2], arg_iData[3]);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

void sns_BRMP_direction(void)
{
    int32_t    ret;

    SENSOR_N_LOG("start");

    if (!gpio_is_valid(g_micon_bdata.brmp_gpio)) {
        SENSOR_ERR_LOG("gpio is invalid");
        return;
    }
    ret = gpio_direction_input(g_micon_bdata.brmp_gpio);

    if(ret < 0 ){
        SENSOR_ERR_LOG("Error BRMP CTL");
    }

    SENSOR_N_LOG("end");
}

static void sns_FW_BRMP_ctrl(void)
{
    SENSOR_N_LOG("start");

    if (!gpio_is_valid(g_micon_bdata.brmp_gpio) || !gpio_is_valid(g_micon_bdata.rst_gpio)) {
        SENSOR_ERR_LOG("gpio is invalid");
        return;
    }

    gpio_direction_output(g_micon_bdata.brmp_gpio, 1);

    gpio_direction_output(g_micon_bdata.rst_gpio, 0);

    udelay(510);

    gpio_set_value(g_micon_bdata.rst_gpio, 1);

    msleep(20);

    gpio_set_value(g_micon_bdata.brmp_gpio, 0);

    g_micon_error = false;

    SENSOR_N_LOG("end");
}

int32_t sns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    fw_ver[4];
    HostCmd    cmd;
    HostCmdRes res;
    int32_t    i;

    SENSOR_D_LOG("start");

    atomic_set(&g_FWUpdateStatus,true);
    
    ret = sns_get_fw_version(fw_ver);
    g_nFWVersion = SNESOR_COM_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Get Version Error!!");
        g_nFWVersion = SNESOR_COM_FW_VER_NONE;
    }

    SENSOR_N_LOG("Now[%x] Base[%x]",g_nFWVersion, SNESOR_COM_FW_VER_DATA);

    if(g_nFWVersion != SNESOR_COM_FW_VER_DATA){
        SENSOR_D_LOG("Need to update F/W Version");

        mutex_lock(&sensor_fifo_mutex);
        cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
        ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER, READ_FIFO);
        mutex_unlock(&sensor_fifo_mutex);
        SENSOR_N_LOG("HC_MCU_SELF_CHK_FW[%d] err res[%x] err[%x]", ret, res.res.ub_res[0], res.err.udata16);
        if(1) {
            SENSOR_N_LOG("F/W Update Start...");
            DISABLE_IRQ;
            for(i = 0; i < SNS_SPI_RETRY_NUM; i++){
                sns_FW_BRMP_ctrl();
                ret = sns_update_fw_exe(true, arg_iData, arg_iLen);
                if(ret != SNS_RC_ERR_RAMWRITE)
                    break;
            }
        }else if(0x00 == res.res.ub_res[0]){
            SENSOR_N_LOG("HC_MCU_SELF_CHK_FW(-) OK!!");
        }
        SENSOR_N_LOG("F/W Initialize Start...");
        ret |= sns_initialize();
        ret |= sns_set_dev_param();
    }else{
        SENSOR_N_LOG("None update F/W Version");
    }
    SENSOR_N_LOG("F/W Update Check Completed...");

    if (gpio_is_valid(g_micon_bdata.brmp_gpio)) {
        ret |= gpio_direction_input(g_micon_bdata.brmp_gpio);
    }

    atomic_set(&g_FWUpdateStatus,false);

    SENSOR_D_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_update_fw(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = SNS_RC_OK;
    uint32_t   i;

    SENSOR_D_LOG("start");

    atomic_set(&g_FWUpdateStatus,true);

    SENSOR_N_LOG("F/W Update Start...");
    DISABLE_IRQ;
    for(i = 0; i < SNS_SPI_RETRY_NUM; i++){
        sns_FW_BRMP_ctrl();
        ret = sns_update_fw_exe(true, arg_iData, arg_iLen);
        if(ret != SNS_RC_ERR_RAMWRITE)
            break;
    }
    SENSOR_N_LOG("F/W Initialize Start...");
    ret |= sns_initialize();
    ret |= sns_set_dev_param();
    if (gpio_is_valid(g_micon_bdata.brmp_gpio)) {
        ret |= gpio_direction_input(g_micon_bdata.brmp_gpio);
    }

    atomic_set(&g_FWUpdateStatus,false);

    if(ret != 0){
        SENSOR_ERR_LOG("error(sns_update_fw_exe) : fw_update");
        return SNS_RC_ERR;
    }

    SENSOR_D_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_update_fw_exe(bool boot, uint8_t *arg_iData, uint32_t arg_iLen)
{
    uint8_t reg = 0xFF;
    int32_t i;
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;
    int32_t size;
    int32_t send_num = 0;
    uint32_t chksum = 0;
    uint8_t chksum_data[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t    fw_ver[4];
    int cnt = 170;

    SENSOR_N_LOG("start");

    memset(&res, 0x00, sizeof(HostCmdRes));

    SENSOR_N_LOG("boot[%d] Data[%x] Len[%d]", boot, (int)arg_iData, arg_iLen);

    if((arg_iData == NULL) || (arg_iLen == 0)){
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    ENABLE_IRQ;
    if(!boot){
        cmd.cmd.udata16 = HC_MCU_FUP_START;
        cmd.prm.ub_prm[0] = 0x55;
        cmd.prm.ub_prm[1] = 0xAA;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("Communication Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
            SENSOR_ERR_LOG("Certification Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR;
        }
    }

    msleep(30);

    reg = 0x04;
    sns_device_write(CFG, &reg, sizeof(reg));

    reg = 0x00;
    sns_device_write(INTMASK0, &reg, sizeof(reg));
    sns_device_write(INTMASK1, &reg, sizeof(reg));

    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x11;
    cmd.prm.ub_prm[1] = 0x11;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("PortSettint err[%x]", res.err.udata16);
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("Check Firmware Mode.");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_RSLT);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    if(res.res.ub_res[2] != 0x01){
        SENSOR_ERR_LOG("Version check Error!");
        SENSOR_N_LOG("FW Version[%02x] [%02x] [%02x] [%02x]",
                      res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3]);
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("FW Version[%02x] [%02x] [%02x] [%02x]",
                  res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3]);

    SENSOR_N_LOG("Flash Clear.");
    cmd.cmd.udata16 = HC_MCU_FUP_ERASE;
    cmd.prm.ub_prm[0] = 0xAA;
    cmd.prm.ub_prm[1] = 0x55;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
        SENSOR_ERR_LOG("Certification Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    while(cnt > 0) {
        sns_device_read(STATUS, &reg, sizeof(reg));
        if(reg == 0x00) {
            SENSOR_N_LOG("STATUS OK!!");
            break;
        } else {
            msleep(10);
            cnt--;
        }
    }

    if(cnt <= 0){
        SENSOR_ERR_LOG("Flash Clear STATUS Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    send_num = arg_iLen / FUP_MAX_RAMSIZE;
    send_num = (arg_iLen % FUP_MAX_RAMSIZE) ? send_num+1 : send_num;

    for(i=0; i < arg_iLen; i++){
        chksum += (uint32_t)arg_iData[i];
    }
    for(i=0; i < send_num; i++){
        if((arg_iLen - (FUP_MAX_RAMSIZE * i)) >= FUP_MAX_RAMSIZE){
            size = FUP_MAX_RAMSIZE;
        } else {
            size = arg_iLen % FUP_MAX_RAMSIZE;
        }
        ret = sns_spi_ram_write_proc(FIFO, &arg_iData[i * FUP_MAX_RAMSIZE], size);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("RAM Write Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR_RAMWRITE;
        }
        cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
        if((ret != SNS_RC_OK) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("Communication Error! err[%x]",res.err.udata16);
            DISABLE_IRQ;
            return SNS_RC_ERR;
        }
    }

    SENSOR_N_LOG("Self Check.");
    chksum_data[3] = (uint8_t)((chksum >> 24) & 0x000000ff);
    chksum_data[2] = (uint8_t)((chksum >> 16) & 0x000000ff);
    chksum_data[1] = (uint8_t)((chksum >> 8) & 0x000000ff);
    chksum_data[0] = (uint8_t)(chksum & 0x000000ff);
    ret = sns_device_write(FIFO, chksum_data, sizeof(chksum_data));
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("chksum FIFO Write Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR_RAMWRITE;
    }
    cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }
    cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER, READ_RSLT);
    SENSOR_N_LOG("HC_MCU_SELF_CHK_FW[%d] err res[%x] err[%x]",
                  ret, res.res.ub_res[0], res.err.udata16);
    SENSOR_N_LOG("End Firmware Update.");

    cmd.cmd.udata16 = HC_MCU_FUP_END;
    sns_hostcmd(&cmd, &res, 0, EXE_HOST_ERR, READ_RSLT);
    msleep(300);

    reg = 0x04;
    sns_device_write(CFG, &reg, sizeof(reg));

    reg = 0x00;
    sns_device_write(INTMASK0, &reg, sizeof(reg));
    sns_device_write(INTMASK1, &reg, sizeof(reg));

    SENSOR_N_LOG("Check User program mode.");
    ret = sns_get_fw_version(fw_ver);
    g_nFWVersion = SNESOR_COM_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK ){
        g_nFWVersion = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("Version not get.");
    }
    SENSOR_N_LOG("Sensor FW Version.[%08x]",g_nFWVersion);
    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x00){
        SENSOR_ERR_LOG("Version check Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }
    DISABLE_IRQ;
    SENSOR_N_LOG("end");

    return SNS_RC_OK;
}

static void sns_workqueue_init(void)
{
    int32_t i;

    SENSOR_N_LOG("start");

    for(i=0; i<SNS_WORK_QUEUE_NUM; i++){
        cancel_work_sync(&s_tSnsWork[i].work);
        s_tSnsWork[i].status = false;
    }
    g_nSnsWorkCnt = 0;

    SENSOR_N_LOG("end");
}

static int32_t sns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *), int32_t option, enum sensor_e_type type )
{
    int32_t ret = SNS_RC_ERR;
    int32_t i;
    unsigned long flags;

    SENSOR_N_LOG("start queue:%p func:%pf option:%d type:%d", queue, func, option, type);

    if((queue == NULL) || (func == NULL)){
        return SNS_RC_ERR;
    }

    spin_lock_irqsave(&acc_lock, flags);

    SENSOR_N_LOG("g_nSnsWorkCnt[%d]status[%x]",g_nSnsWorkCnt, s_tSnsWork[g_nSnsWorkCnt].status);

    if(s_tSnsWork[g_nSnsWorkCnt].status == false){

        INIT_WORK( &s_tSnsWork[g_nSnsWorkCnt].work, func );
        s_tSnsWork[g_nSnsWorkCnt].option = option;
        s_tSnsWork[g_nSnsWorkCnt].type = type;

        ret = queue_work( queue, &s_tSnsWork[g_nSnsWorkCnt].work );

        if (ret == 1) {
            s_tSnsWork[g_nSnsWorkCnt].status = true;

            if(++g_nSnsWorkCnt >= SNS_WORK_QUEUE_NUM){
                g_nSnsWorkCnt = 0;
            }
            ret = SNS_RC_OK;
        }else{
            SENSOR_ERR_LOG("queue_work Non Create[%d]",ret);
        }
        g_workqueue_used = 0;
    }else{
        if(g_workqueue_used < SNS_WORK_QUEUE_NUM){
            SENSOR_ERR_LOG("SNS queue_work[%d] used!!",g_nSnsWorkCnt);
            for(i=0; i<SNS_WORK_QUEUE_NUM; i++){
                SENSOR_N_LOG("g_nSnsWorkCnt[%d]status[%x]",i,s_tSnsWork[i].status);
            }
            if(++g_nSnsWorkCnt >= SNS_WORK_QUEUE_NUM){
                g_nSnsWorkCnt = 0;
            }
            g_workqueue_used++;
        }
    }

    spin_unlock_irqrestore(&acc_lock, flags);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static void sns_workqueue_delete(struct work_struct *work)
{
    int32_t i;
    unsigned long flags;

    SENSOR_N_LOG("start");

    spin_lock_irqsave(&acc_lock, flags);

    for(i=0; i<SNS_WORK_QUEUE_NUM; i++){
        if(&s_tSnsWork[i].work == work){
            s_tSnsWork[i].status = false;
            SENSOR_N_LOG("hit delete queue[%d]! work:0x[%x] 0x[%x] ",
                        i, (int)&s_tSnsWork[i].work, (int)work);
            break;
        }
    }

    spin_unlock_irqrestore(&acc_lock, flags);

    SENSOR_N_LOG("end");

    return ;
}

static bool sns_devif_error_check(void)
{
    SENSOR_N_LOG("g_bDevIF_Error[%d]",g_bDevIF_Error);
    return g_bDevIF_Error;
}

static void sns_pre_power_on(void)
{
    int32_t ret = SNS_RC_OK;
#if 0
    int32_t cnt = 0;
#endif

    SENSOR_N_LOG("start");

    if(sns_devif_error_check() == false){
        SENSOR_N_LOG("Other Sensor Error");
#if 0
        while(1){
            ret = sns_micon_i2c_enable(true);
            cnt++;
            if(ret == SNS_RC_OK || cnt >= MICON_I2C_ENABLE_RETRY_NUM){
                SENSOR_N_LOG("i2c enable cnt[%d]",cnt);
                break;
            }
            msleep(10);
        }
#endif
        ret |= sns_micon_initcmd();
        ret |= sns_set_dev_param();
    } else {
        SENSOR_N_LOG("Micon SPI Error");
        ret |= sns_initialize();
        if(ret != SNS_RC_OK){
            SENSOR_N_LOG("sns_initialize ret[%d]",ret);
            atomic_set(&g_ResetStatus,false);
            return;
        }
        ret |= sns_set_dev_param();
    }

    if( true == g_reset_param_status) {
        ret = sns_reset_restore_param();
        sensor_reset_resume();
        g_reset_param_status = false;
    }

    atomic_set(&g_ResetStatus,false);

    SENSOR_N_LOG("end");
}

static void sns_pre_power_off(void)
{
    SENSOR_N_LOG("start");

    if(atomic_read(&g_ShutdownStatus) == false){
        atomic_set(&g_ResetStatus,true);
        g_reset_param_status = false;
        if(sns_devif_error_check() == false){
            sensor_reset();
            sns_reset_save_param();
        }
        g_reset_param_status = true;
        if(sns_devif_error_check() == false){
            SENSOR_N_LOG("Other Sensor Error");
#if 0
            sns_micon_i2c_enable(false);
#endif
        } else {
            SENSOR_N_LOG("Micon SPI Error");
        }
    }

    SENSOR_N_LOG("end");
}

#if 0
static int32_t sns_micon_i2c_enable(bool arg_iEnable)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    if(atomic_read(&g_FWUpdateStatus) || atomic_read(&g_ShutdownStatus) == true){
        SENSOR_ERR_LOG("FW Update or Recovery Now");
        return 0;
    }

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MCU_SET_PERI;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (arg_iEnable == false) ? 0x01:0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 4, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MCU_SET_PERI(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    if((arg_iEnable == true) && res.res.ub_res[1] == 0x01)
    {
        SENSOR_ERR_LOG("I2C Valid Error RST01[%x]",res.res.ub_res[1]);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end");
    return ret;
}
#endif

static int32_t sns_micon_initcmd(void)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);
    cmd.cmd.udata16 = HC_MCU_SENSOR_INIT;
    cmd.prm.ub_prm[0] = 0x00;
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    cmd.prm.ub_prm[0] |= 0x01;
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    cmd.prm.ub_prm[0] |= 0x20;
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    cmd.prm.ub_prm[0] |= 0x40;
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    cmd.prm.ub_prm[0] |= 0x80;
#endif
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&sensor_fifo_mutex);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MCU_SENSOR_INIT(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    if((res.res.ub_res[0] & 0x01) != 0)
    {
        SENSOR_ERR_LOG("Acc Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    if((res.res.ub_res[0] & 0x20) != 0)
    {
        SENSOR_ERR_LOG("Pressure Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    if((res.res.ub_res[0] & 0x40) != 0)
    {
        SENSOR_ERR_LOG("Mag Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    if((res.res.ub_res[0] & 0x80) != 0)
    {
        SENSOR_ERR_LOG("Gyro Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }
#endif
    SENSOR_N_LOG("end");
    return ret;
}

static int32_t sns_reset_save_param(void)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;

#ifdef CONFIG_SENSOR_MICON_LOGGING
    cmd.cmd.udata16 = HC_MUL_SET_ENABLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
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
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0]  = 0x00;
    cmd.prm.ub_prm[1]  = sns_ctrl_param.output_param.sensor.task_exec[1];
    cmd.prm.ub_prm[2]  = sns_ctrl_param.output_param.sensor.task_exec[2];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    return SNS_RC_OK;
}

static int32_t sns_reset_restore_param(void)
{
    return sns_update_micon_control_if_needed(true);
}

int32_t sns_err_check(void)
{
    int ret_val = 0;

    if ((sns_devif_error_check() != false) && (atomic_read(&g_ResetStatus) == false) &&
        g_micon_error == false) {
        sensor_power_reset(SENSOR_INDEX_ACC);
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

    if(arg_iEnable == true){
        enable_irq_wake(g_nIntIrqNo);
        SENSOR_N_LOG("enable_irq_wake");
    } else {
        disable_irq_wake(g_nIntIrqNo);
        SENSOR_N_LOG("disable_irq_wake");
    }

    SENSOR_N_LOG("end");
}

void sns_set_buff_int(bool arg_iEnable)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start-Enable[%d]",arg_iEnable);

    cmd.cmd.udata16 = HC_MAG_SET_BUFF_INT;
    cmd.prm.ub_prm[0] = arg_iEnable;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_BUFF_INT err[%x]",res.err.udata16);
        return;
    }

    SENSOR_N_LOG("end");
}

static struct of_device_id sensor_micon_match_table[] = {
    { .compatible = SENSOR_MICON_DRIVER_NAME,},
    { },
};

static struct spi_driver sensor_micon_driver = {
    .probe       = sensor_micon_probe,
    .driver = {
        .name    = SENSOR_MICON_DRIVER_NAME,
        .bus     = &spi_bus_type,
        .owner   = THIS_MODULE,
        .of_match_table = sensor_micon_match_table,
    },
    .remove      = sensor_micon_remove,
    .suspend     = sensor_micon_suspend,
    .resume      = sensor_micon_resume,
    .shutdown    = sensor_micon_shutdown,
};

static int32_t sensor_micon_resume( struct spi_device *client )
{

    SENSOR_N_LOG("start");

    atomic_set(&g_bIsResume,true);

    sensor_resume();

    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_micon_suspend( struct spi_device *client, pm_message_t mesg )
{

    SENSOR_N_LOG("start");

    sensor_suspend();

    if(sns_wq_int != NULL){
        SENSOR_N_LOG("sns_wq_int");
        flush_workqueue(sns_wq_int);
    }

    if(sns_wq != NULL){
        SENSOR_N_LOG("sns_wq");
        flush_workqueue(sns_wq);
    }

    atomic_set(&g_bIsResume,false);

    SENSOR_N_LOG("end");
    return 0;
}

static void sensor_micon_shutdown( struct spi_device *client )
{
    SENSOR_N_LOG("start");

    atomic_set(&g_ShutdownFlg, true);
    if(atomic_read(&g_ShutdownStatus) == false){

        sensor_shutdown();

        mutex_lock(&sensor_state_app_mutex);
        mutex_lock(&sensor_state_irq_mutex);

        DISABLE_IRQ;
        sns_workqueue_init();

        if(gpio_is_valid(g_micon_bdata.int_gpio)) {
            gpio_free(g_micon_bdata.int_gpio);
        }

        if(gpio_is_valid(g_micon_bdata.rst_gpio)) {
            gpio_free(g_micon_bdata.rst_gpio);
        }

        if(sns_wq_int != NULL){
            SENSOR_N_LOG("sns_wq_int");
            destroy_workqueue(sns_wq_int);
            sns_wq_int = NULL;
        }

        if(sns_wq != NULL){
            SENSOR_N_LOG("sns_wq");
            destroy_workqueue(sns_wq);
            sns_wq = NULL;
        }
        wake_lock_destroy(&g_pedo_wakelock);

        mutex_unlock(&sensor_state_irq_mutex);
        mutex_unlock(&sensor_state_app_mutex);
    }

    atomic_set(&g_ShutdownStatus,true);

    SENSOR_N_LOG("end");
    return;
}

static int32_t sensor_micon_remove( struct spi_device *client )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_micon_probe( struct spi_device *client )
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    client_sns = client;
    client_sns->bits_per_word = 8;

    g_nIntIrqNo = -1;
    g_nIntIrqFlg = 0;
    g_bDevIF_Error = false;
    g_micon_error = false;
    atomic_set(&g_bIsResume,true);
    atomic_set(&g_ShutdownStatus,false);
    atomic_set(&g_ShutdownFlg,false);

    atomic_set(&g_nWeight,DEFAULT_WEIGHT);
    atomic_set(&g_nStepWide,DEFAULT_PEDOMETER);
    atomic_set(&g_nVehiType,DEFAULT_VEHITYPE);
    atomic_set(&g_FWUpdateStatus,false);
    atomic_set(&g_nCalX,0);
    atomic_set(&g_nCalY,0);
    atomic_set(&g_nCalZ,0);
    atomic_set(&g_LogParamSns,0);
    atomic_set(&g_LogParamFus,0);
    atomic_set(&g_Step_start,0);
    atomic_set(&g_MiconDebug,0);
    atomic_set(&g_sensors_on_status,0);
    atomic_set(&g_pressure_cal_offset,0);


    g_FusionDelay = 0;
    g_dailys_status = SNS_OFF;
    g_micon_pedo_status = false;
    g_micon_baro_status = false;
    g_step_status = OTHER_SENSOR;
    g_vib_interlocking = 0x00;

    init_waitqueue_head(&s_tWaitInt);

    sns_workqueue_init();
    mutex_init(&s_tDataMutex);
    mutex_init(&s_tSpiMutex);
    mutex_init(&sensor_fifo_mutex);

    mutex_lock(&s_tDataMutex);
    memset(&s_MagData, 0x00, sizeof(s_MagData));
    memset(&s_GyroData, 0x00, sizeof(s_GyroData));
    mutex_unlock(&s_tDataMutex);

    wake_lock_init(&g_pedo_wakelock, WAKE_LOCK_SUSPEND, SENSOR_MICON_DRIVER_NAME);

    if(g_nIntIrqNo == -1){
        ret = sns_gpio_init(client);
        if(ret != SNS_RC_OK){
            SENSOR_ERR_LOG("Failed sns_gpio_init[%d]",ret);
            return -ENODEV;
        }
        DISABLE_IRQ;
    }

    ret = sns_pinctrl_init(client);
    if (ret < 0) {
        SENSOR_ERR_LOG("Failed sns_pinctrl_init[%d]",ret);
        return -ENODEV;
    }

    ret = sns_initialize();
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Failed sns_initialize[%d]",ret);
    }

    ret = sns_set_dev_param();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Failed sns_set_dev_param[%d]",ret);
    }

    sns_pre_power_cb.power_on  = sns_pre_power_on;
    sns_pre_power_cb.power_off = sns_pre_power_off;
    sensor_power_reg_cbfunc(&sns_pre_power_cb);

    SENSOR_N_LOG("end");
    return 0;
}

void sensor_micon_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    sema_init(&s_tSnsrSema, 1);

    sns_wq_int = create_singlethread_workqueue("sns_wq_int");
    if(!sns_wq_int)
    {
        SENSOR_ERR_LOG("can't create interrupt queue-sns_wq_int");
        return;
    }

    sns_wq = create_singlethread_workqueue("sns_wq");
    if(!sns_wq)
    {
        SENSOR_ERR_LOG("can't create interrupt queue-sns_wq");
        goto REGIST_ERR;
    }

    ret = spi_register_driver(&sensor_micon_driver);
    if(ret != 0){
        SENSOR_ERR_LOG("fail:spi_register_driver()->ret[%d]",ret);
        goto REGIST_ERR;
    }

    sns_iio_init();

    SENSOR_N_LOG("end");
    return;

REGIST_ERR:
    if(sns_wq_int != NULL){
        flush_workqueue(sns_wq_int);
        destroy_workqueue(sns_wq_int);
        sns_wq_int = NULL;
    }

    if(sns_wq != NULL){
        flush_workqueue(sns_wq);
        destroy_workqueue(sns_wq);
        sns_wq = NULL;
    }

    return;
}

void sensor_micon_exit(void)
{
    SENSOR_N_LOG("start");
    sns_iio_exit();
    SENSOR_N_LOG("end");
    return;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

static struct iio_dev * g_iio_dev = NULL;
static DEFINE_MUTEX(g_iio_mutex);

// Must match HAL
typedef enum {
    SHID_ACCELEROMETER = 0,
    SHID_GEOMAGNETIC,
    SHID_ORIENTATION,
    SHID_GYROSCOPE,
    SHID_PRESSURE,
    SHID_GRAVITY,
    SHID_LINEAR_ACCELERATION,
    SHID_ROTATION_VECTOR,
    SHID_MAGNETIC_FIELD_UNCALIBRATED,
    SHID_GAME_ROTATION_VECTOR,
    SHID_GYROSCOPE_UNCALIBRATED,
    SHID_STEP_DETECTOR,
    SHID_STEP_COUNTER,
    SHID_GEOMAGNETIC_ROTATION_VECTOR,
    SHID_META_DATA,
    SHID_MAX,
} SENSOR_HUB_EVENT_ID;

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

static bool sns_is_batch_id_valid(uint8_t batch_id)
{
    bool ret = false;
    SENSOR_N_LOG("start id[%d]", batch_id);

    switch (batch_id) {
    case SENSOR_BATCH_ID_ACC:
    case SENSOR_BATCH_ID_MAG:
    case SENSOR_BATCH_ID_GYRO:
    case SENSOR_BATCH_ID_MAG_UNCAL:
    case SENSOR_BATCH_ID_GYRO_UNCAL:
    case SENSOR_BATCH_ID_PRESSURE:
    case SENSOR_BATCH_ID_ORTN:
    case SENSOR_BATCH_ID_GRV:
    case SENSOR_BATCH_ID_ACC_LNR:
    case SENSOR_BATCH_ID_ROT_VCTR:
    case SENSOR_BATCH_ID_GAME_ROT_VCTR:
    case SENSOR_BATCH_ID_MAG_ROT_VCTR:
    case SENSOR_BATCH_ID_REL_STEP_1:
    case SENSOR_BATCH_ID_REL_STEP_2:
    case SENSOR_BATCH_ID_ABS_STEP:
        ret = true;
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end ret[%d]", ret);
    return ret;
}

#define EVENT_DATA_SIZE 15

static int sns_iio_report_event(uint8_t id, uint8_t *data, uint32_t len, int64_t timestamp, int64_t offset)
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

static bool sns_is_needed_gyro_sync(void)
{
    return sns_ctrl_param.input_param.inputs[SENSOR_GYRO].sampling_period_ms ==
                sns_ctrl_param.input_param.inputs[SENSOR_GYRO_UNCAL].sampling_period_ms &&
                (sns_ctrl_param.output_param.polling.enable_sns & (1 << SENSOR_GYRO)) &&
                (sns_ctrl_param.output_param.polling.enable_sns & (1 << SENSOR_GYRO_UNCAL));
}

int sns_iio_report_event_now(enum sensor_e_type type)
{
    int64_t now = GET_CURRENT_TIMESTAMP_NS();
    uint8_t data[EVENT_DATA_SIZE] = {0};
    int32_t work;

    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start type[%d]", type);

    switch (type) {
    case SENSOR_ACC:
        ret = sns_device_read(RSLT00, data, 6);
        break;

    case SENSOR_MAG:
        ret = sns_device_read(RSLT0C, data, 6);
        SENSOR_N_LOG("sns_device_read[%d]",ret);
        if (SNS_RC_OK == ret) {
            ret = sns_device_read(RSLT2D, data+6, 1);
            SENSOR_N_LOG("sns_device_read[%d]",ret);
        }
        break;

    case SENSOR_MAG_UNCAL:
        ret = sns_device_read(RSLT0C, data, 6);
        if (SNS_RC_OK == ret) {
            ret = sns_device_read(RSLT2D, data+6, 1);
            if (SNS_RC_OK == ret) {
                data[7] = (uint8_t)(s_MagData.x & 0x000000ff);
                data[8] = (uint8_t)((s_MagData.x >> 8) & 0x000000ff);
                data[9] = (uint8_t)(s_MagData.y & 0x000000ff);
                data[10] = (uint8_t)((s_MagData.y >> 8) & 0x000000ff);
                data[11] = (uint8_t)(s_MagData.z & 0x000000ff);
                data[12] = (uint8_t)((s_MagData.z >> 8) & 0x000000ff);
            }
        }
        break;

    case SENSOR_GYRO:
        if (sns_is_needed_gyro_sync()) {
            ret = SNS_RC_ERR;
        }
        else {
            ret = sns_device_read(RSLT06, data, 6);
        }
        break;

    case SENSOR_GYRO_UNCAL:
        ret = sns_device_read(RSLT06, data, 6);
        if (SNS_RC_OK == ret) {
            if (sns_is_needed_gyro_sync()) {
                sns_iio_report_event(
                        sensor_type_to_smid[SENSOR_GYRO],
                        data,
                        EVENT_DATA_SIZE,
                        now,
                        0);
            }
            data[6] = (uint8_t)(s_GyroData.x & 0x000000ff);
            data[7] = (uint8_t)((s_GyroData.x >> 8) & 0x000000ff);
            data[8] = (uint8_t)(s_GyroData.y & 0x000000ff);
            data[9] = (uint8_t)((s_GyroData.y >> 8) & 0x000000ff);
            data[10] = (uint8_t)(s_GyroData.z & 0x000000ff);
            data[11] = (uint8_t)((s_GyroData.z >> 8) & 0x000000ff);
        }
        break;

    case SENSOR_PRESSURE:
        ret = sns_device_read(RSLT2C, data, 1);
        if (SNS_RC_OK == ret) {
            ret = sns_device_read(RSLT12, data+1, 2);
            SENSOR_N_LOG("sns_device_read[%d]",ret);
            if (SNS_RC_OK == ret) {
                work = (int32_t)((data[0] & 0x01) | (data[1] << 1) | (data[2] << 9));
                work += atomic_read(&g_pressure_cal_offset) + PRESSURE_DEFAULT_OFFSET;
                *(int32_t*)data = work;
            }
        }
        break;

    case SENSOR_ORTN:
        ret = sns_device_read(RSLT14, data, 6);
        SENSOR_N_LOG("sns_device_read[%d]",ret);
        if (SNS_RC_OK == ret) {
            ret = sns_device_read(RSLT2D, data+6, 1);
            SENSOR_N_LOG("sns_device_read[%d]",ret);
        }
        break;

    case SENSOR_GRV:
        ret = sns_device_read(RSLT1A, data, 6);
        break;

    case SENSOR_ACC_LNR:
        ret = sns_device_read(RSLT20, data, 6);
        break;

    case SENSOR_ROT_VCTR:
        ret = sns_device_read(RSLT26, data, 6);
        break;

    case SENSOR_GAME_ROT_VCTR:
        ret = sns_device_read(RSLT36, data, 8);
        break;

    case SENSOR_MAG_ROT_VCTR:
        ret = sns_device_read(RSLT2E, data, 8);
        break;

    default:
        ret = SNS_RC_ERR;
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

static int sns_iio_report_events(uint8_t *batch_data, uint32_t len, int64_t *base_timestamps, uint32_t batch_status)
{
    uint32_t stepcount_data = 0;
    int64_t  timestamp_offsets[SENSOR_BATCH_TBL_MAX];
    uint8_t  *buf = batch_data + LOGGING_RESPONSE_HEADER;
    int32_t  remain = len - LOGGING_RESPONSE_HEADER;
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
    SENSOR_N_LOG("start");

    // initialize timestamp
    sns_read_timestamp_offsets(batch_data, timestamp_offsets, &stepcount_data);
    step_count += stepcount_data;

    while (remain > 0)
    {
        batch_id = *buf;
        if (!sns_is_batch_id_valid(batch_id)) {
            SENSOR_ERR_LOG("invalid batch_id [%d] remain[%d]", batch_id, remain);
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
            *offset += data[0];
            work = ((int32_t)data[1] | (((int32_t)data[2]) << 8) | (((int32_t)data[3]) << 16))
                + atomic_read(&g_pressure_cal_offset) + PRESSURE_DEFAULT_OFFSET;
            sns_iio_report_event(smid, (uint8_t*)&work, sizeof(work), timestamp, *offset);
            break;
        case SENSOR_BATCH_TBL_MAG:
            *offset += data[0];
            if(batch_status & (BATCH_ON << SENSOR_MAG)){
                sns_iio_report_event(SHID_GEOMAGNETIC, &data[1],
                    size-1, timestamp, *offset);
            }
            if(batch_status & (BATCH_ON << SENSOR_MAG_UNCAL)){
                memcpy(&tmp_event[0], &data[1], size-1);
                memcpy(&tmp_event[size-1], mag_uncal_offset, sizeof(mag_uncal_offset));
                sns_iio_report_event(SHID_MAGNETIC_FIELD_UNCALIBRATED, &tmp_event[0],
                    size-1+sizeof(mag_uncal_offset), timestamp, *offset);
            }
            break;
        case SENSOR_BATCH_TBL_GYRO:
            *offset += data[0];
            if(batch_status & (BATCH_ON << SENSOR_GYRO)){
                sns_iio_report_event(SHID_GYROSCOPE, &data[1],
                    size-1, timestamp, *offset);
            }
            if(batch_status & (BATCH_ON << SENSOR_GYRO_UNCAL)){
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
    return 0;
}

static int sns_iio_report_flush(enum sensor_e_type type, int64_t timestamp)
{
    int ret;
    uint8_t id = sensor_type_to_smid[type];
    SENSOR_N_LOG("start");
    ret = sns_iio_report_event(SHID_META_DATA, &id, 1, timestamp, 0);
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static void sns_logging_setup_basetime(uint32_t logging_new, uint32_t logging_old)
{
    int64_t now = GET_CURRENT_TIMESTAMP_NS();
    uint32_t sensors_started;
    int i;

    SENSOR_N_LOG("start");

    sensors_started = logging_new & ~logging_old;
    if (logging_old & ((1 << SENSOR_MAG)|(1 << SENSOR_MAG_UNCAL))) {
        sensors_started &= ~((1 << SENSOR_MAG)|(1 << SENSOR_MAG_UNCAL));
    }
    if (logging_old & ((1 << SENSOR_GYRO)|(1 << SENSOR_GYRO_UNCAL))) {
        sensors_started &= ~((1 << SENSOR_GYRO)|(1 << SENSOR_GYRO_UNCAL));
    }
    if (logging_old & ((1 << SENSOR_STEP_CNT)|(1 << SENSOR_STEP_DTC))) {
        sensors_started &= ~((1 << SENSOR_STEP_CNT)|(1 << SENSOR_STEP_DTC));
    }

    if (!sensors_started) {
        return;
    }

    for (i = 0; i < SENSOR_MAX; i++) {
        if (sensors_started & (1 << i)) {
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
                g_logging_base_timestamps[sensor_type_to_tbl_idx[i]] = now;
                SENSOR_N_LOG("set base timestamp type:%d idx:%d time:%lld", i, sensor_type_to_tbl_idx[i], now);
                break;
            default:
                break;
            }
        }
    }

    SENSOR_N_LOG("end");
}

static void sns_logging_proceed_time(int64_t *timestamps)
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

static int32_t sns_logging_punctuation_locked(int32_t trigger)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    int64_t  base_timestamps[SENSOR_BATCH_TBL_MAX];
    static uint32_t latest_batch_status = 0;
    uint32_t base_batch_status;
    uint16_t logging_data_size = 0;
    uint16_t read_data_size = 0;
    uint8_t  fifo_size_res[2];
    uint16_t fifo_data_size = 0;
    uint8_t clr_dailys_param;
    uint8_t *buf;
    SENSOR_N_LOG("start");

    mutex_lock(&sensor_fifo_mutex);

    sns_logging_proceed_time(base_timestamps);
    base_batch_status = latest_batch_status;
    latest_batch_status = sensor_get_batch_status();

    cmd.cmd.udata16 = HC_MUL_GET_LOG_PUNCTUATION;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MUL_GET_LOG_PUNCTUATION[%x]", res.err.udata16);
        mutex_unlock(&sensor_fifo_mutex);
        return SNS_RC_ERR;
    }

    ret = sns_device_read(RSLT3E, fifo_size_res, 2);
    SENSOR_N_LOG("sns_device_read-ret[%d] res1[%x] res2[%x]",ret,fifo_size_res[0],fifo_size_res[1]);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("sns_device_read[%d]", ret);
        mutex_unlock(&sensor_fifo_mutex);
        return ret;
    }
    fifo_data_size = ((fifo_size_res[1] << 8) | fifo_size_res[0]);
    SENSOR_N_LOG("sns_device_read-fifo_data_size[%x]",fifo_data_size);

    clr_dailys_param = 0x00;
    if (sns_ctrl_param.output_param.logging.batch_step_cnt_enable ||
        sns_ctrl_param.output_param.logging.batch_step_dtc_enable)
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
            mutex_unlock(&sensor_fifo_mutex);
            return SNS_RC_ERR;
        }
    }

    if(LOGGING_RESPONSE_HEADER > fifo_data_size || fifo_data_size > sizeof(res.res.ub_res)) {
        SENSOR_ERR_LOG("fifo_data_size[%d]", fifo_data_size);
        mutex_unlock(&sensor_fifo_mutex);
        return SNS_RC_ERR;
    }

    ret |= sns_device_read(FIFO, res.res.ub_res, fifo_data_size);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("sns_device_read[%d]", ret);
        mutex_unlock(&sensor_fifo_mutex);
        return ret;
    }
    logging_data_size = ((res.res.ub_res[1] << 8) | res.res.ub_res[0]);
    SENSOR_D_LOG("logging_data_size[%x]",logging_data_size);

    if(logging_data_size == 0) {
        mutex_unlock(&sensor_fifo_mutex);
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
    SENSOR_N_LOG("read_data_size[%d]-logging_data_size[%d]",read_data_size,logging_data_size);

    if(logging_data_size > sizeof(g_logging_data) - read_data_size) {
        mutex_unlock(&sensor_fifo_mutex);
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
                mutex_unlock(&sensor_fifo_mutex);
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
                mutex_unlock(&sensor_fifo_mutex);
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

    sns_iio_report_events(g_logging_data, read_data_size, base_timestamps, base_batch_status);

    mutex_unlock(&sensor_fifo_mutex);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t sns_logging_punctuation(int32_t trigger)
{
    int32_t ret;
#ifndef CONFIG_SENSOR_MICON_LOGGING
    return SNS_RC_OK;
#endif

    SENSOR_N_LOG("start");

    sensor_com_mutex_lock();

    ret = sns_logging_punctuation_locked(trigger);

    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

enum ABS_status {
    SH_SCAN_DUMMY1 = 0,
    SH_SCAN_DUMMY2,
};

#define SH_CHANNEL(axis)            \
{                       \
    .type = IIO_ACCEL,          \
    .modified = 1,              \
    .channel2 = axis+1,         \
    .info_mask = BIT(IIO_CHAN_INFO_SCALE),  \
    .scan_index = axis,         \
    .scan_type = IIO_ST('u', 128, 128, 0)   \
}

static const struct iio_chan_spec sh_channels[] = {
    SH_CHANNEL(SH_SCAN_DUMMY1),
    SH_CHANNEL(SH_SCAN_DUMMY2),
};

static const struct iio_buffer_setup_ops sns_iio_buffer_setup_ops = {
    .preenable = &iio_sw_buffer_preenable,
};


static int sns_iio_probe_buffer(struct iio_dev *iio_dev)
{
    int ret;
    struct iio_buffer *buffer;

    SENSOR_N_LOG("start");

    buffer = iio_kfifo_allocate(iio_dev);
    if (!buffer) {
        SENSOR_ERR_LOG("iio_kfifo_allocate failed");
        ret = -ENOMEM;
        goto error_ret;
    }

    buffer->length = 4096;
    iio_dev->buffer = buffer;
    iio_dev->setup_ops = &sns_iio_buffer_setup_ops;
    ret = iio_buffer_register(iio_dev, iio_dev->channels,
                  iio_dev->num_channels);
    if (ret) {
        SENSOR_ERR_LOG("iio_buffer_register failed");
        goto error_free_buf;
    }

    iio_scan_mask_set(iio_dev, iio_dev->buffer, SH_SCAN_DUMMY1);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, SH_SCAN_DUMMY2);

    SENSOR_N_LOG("end");

    return 0;

error_free_buf:
    iio_kfifo_free(iio_dev->buffer);
error_ret:
    return ret;
}

static void sns_iio_remove_buffer(struct iio_dev *indio_dev)
{
    SENSOR_N_LOG("start");

    iio_buffer_unregister(indio_dev);
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

    SENSOR_N_LOG("start");

    indio_dev = iio_device_alloc(0);
    if (!indio_dev) {
        SENSOR_ERR_LOG("iio_device_alloc failed");
        return -ENOMEM;
    }

    indio_dev->name = "sensor_hub";
    indio_dev->info = &sh_info;
    indio_dev->channels = sh_channels;
    indio_dev->num_channels = ARRAY_SIZE(sh_channels);
    indio_dev->modes |= INDIO_BUFFER_HARDWARE;

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
    if (indio_dev)
        iio_device_free(indio_dev);
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
    if (indio_dev)
        iio_device_free(indio_dev);

    g_iio_dev = NULL;

    SENSOR_N_LOG("end");
    return 0;
}
