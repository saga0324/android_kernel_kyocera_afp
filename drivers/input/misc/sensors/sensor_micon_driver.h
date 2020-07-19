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
#ifndef SENSOR_MICOM_H
#define SENSOR_MICOM_H

#include "sensor_driver.h"
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <asm/gpio.h>
//#include <mach/irqs.h>
#include <linux/irq.h>


#define SNS_RC_OK                 (0)
#define SNS_RC_OK_NACK            (1)
#define SNS_RC_ERR                (-1)
#define SNS_RC_ERR_TIMEOUT        (-2)
#define SNS_RC_ERR_RAMWRITE       (-3)
#define SNS_RC_ERR_INVALID_ID     (-4)

#define SNS_OFF                   (0)
#define SNS_ON                    (1)

#define HC_INVALID                   (0u)
#define HC_VALID                     (1u)

#define EXE_HOST_WAIT                (1)
#define EXE_HOST_RES                 (2)
#define EXE_HOST_ERR                 (4)
#define EXE_HOST_ALL                 (EXE_HOST_WAIT|EXE_HOST_RES|EXE_HOST_ERR)
#define EXE_HOST_EX_NO_RECOVER       (16)
#define EXE_HOST_NO_RES              (EXE_HOST_WAIT|EXE_HOST_ERR)
#define GPIO_INT_NAME            "sns_irq"
#define GPIO_RESET_NAME          "sns_reset"
#define GPIO_BRMP_NAME           "sns_brmp"
//#define SNS_GPIO_INT              (31)
//#define SNS_GPIO_BRMP             (32)
//#define SNS_GPIO_RST              (1)

#define SNS_WORK_QUEUE_NUM   230
#define SNS_SPI_RETRY_NUM    5
#define SNS_SPI_RESUME_RETRY_NUM 300
#define U2DH_RESOLUTION      1024
#define U2DH_GRAVITY_EARTH   9806550

/* limitation */
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
#define EN_BIT_STEP_CNT             (0x00400000)
#define EN_BIT_STEP_DTC             (0x00800000)
#define EN_BIT_SIGNFCNT_MTN         (0x01000000)

#define EN_BIT_A_M                  (EN_BIT_ACC|EN_BIT_MAG)
#define EN_BIT_A_M_G                (EN_BIT_ACC|EN_BIT_MAG|EN_BIT_GYRO)
#define EN_BIT_A_G                  (EN_BIT_ACC|EN_BIT_GYRO)
#define EN_BIT_A_M_P                (EN_BIT_ACC|EN_BIT_MAG|EN_BIT_PRESSURE)

#define IS_ACC_EN(enable_bits)              (((enable_bits) & EN_BIT_ACC) != 0)
#define IS_MAG_EN(enable_bits)              (((enable_bits) & EN_BIT_MAG) != 0)
#define IS_GYRO_EN(enable_bits)             (((enable_bits) & EN_BIT_GYRO) != 0)
#define IS_MAG_UNCAL_EN(enable_bits)        (((enable_bits) & EN_BIT_MAG_UNCAL) != 0)
#define IS_GYRO_UNCAL_EN(enable_bits)       (((enable_bits) & EN_BIT_GYRO_UNCAL) != 0)
#define IS_PRESSURE_EN(enable_bits)         (((enable_bits) & EN_BIT_PRESSURE) != 0)
#define IS_ORTN_EN(enable_bits)             (((enable_bits) & EN_BIT_ORTN) != 0)
#define IS_GRV_EN(enable_bits)              (((enable_bits) & EN_BIT_GRV) != 0)
#define IS_ACC_LNR_EN(enable_bits)          (((enable_bits) & EN_BIT_ACC_LNR) != 0)
#define IS_ROT_VCTR_EN(enable_bits)         (((enable_bits) & EN_BIT_ROT_VCTR) != 0)
#define IS_GAME_ROT_VCTR_EN(enable_bits)    (((enable_bits) & EN_BIT_GAME_ROT_VCTR) != 0)
#define IS_MAG_ROT_VCTR_EN(enable_bits)     (((enable_bits) & EN_BIT_MAG_ROT_VCTR) != 0)
#define IS_STEP_CNT_EN(enable_bits)         (((enable_bits) & EN_BIT_STEP_CNT) != 0)
#define IS_STEP_DTC_EN(enable_bits)         (((enable_bits) & EN_BIT_STEP_DTC) != 0)
#define IS_SIGNFCNT_MTN_EN(enable_bits)     (((enable_bits) & EN_BIT_SIGNFCNT_MTN) != 0)
#define IS_FUSION_EN(enable_bits)       (((enable_bits) & (1 << EN_TYPE_FUSION)) != 0)
#define IS_APP_EN(enable_bits)          (((enable_bits) & (1 << EN_TYPE_APP)) != 0)
#define IS_ANDROID_EN(enable_bits)      (((enable_bits) & (1 << EN_TYPE_NORMAL)) != 0)

#define GET_CURRENT_TIMESTAMP_NS()   (ktime_to_ns(ktime_get_boottime()))
#define PEDOMETER_WAKELOCK_TIME (msecs_to_jiffies(200))

#define EVENT_DATA_SIZE 15

#define UNAVAILABLE_FEATURE \
    pr_err("This feature can't use with this micon")
#define DO_NOT_NEED_RUN     \
    pr_debug("[SENSOR][%s][D](%d): This process does not need to run with this microcontroller\n", __func__, __LINE__)

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

#define ACCDATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define GYRODATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00008000 ? (((~(data) & 0x0000FFFF) + 1) * (-1)): (data))
#define MAGDATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00008000 ? (((~(data) & 0x0000FFFF) + 1) * (-1)): (data))
#define GRADATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define LINACCDATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))

struct sensor_en_type {
    uint32_t acc_en;
    uint32_t mag_en;
    uint32_t gyro_en;
    uint32_t press_en;
};
struct sensor_ctrl_param_output_str {
    DECLARE_BITMAP(enable_sns, SENSOR_MAX);
    struct {
        uint32_t enable;
        uint16_t period_sensor_task;
        uint16_t period_acc;
        uint16_t period_pressure;
        uint16_t period_mag;
        uint16_t period_gyro;
        uint16_t period_fusion_task;
        uint16_t period_fusion;
        uint16_t period_app_task;
        uint8_t acc_param[5];
        uint8_t gyro_param[6];
        uint8_t mag_param[3];
        uint8_t pressure_param[3];
        uint8_t task_exec[3];
        uint8_t dd_control;
    } sensor;
    struct {
        uint32_t enable;
        DECLARE_BITMAP(enable_sns, SENSOR_MAX);
        uint16_t period_sensor_task;
        uint16_t period_acc;
        uint16_t period_pressure;
        uint16_t period_mag;
        uint16_t period_gyro;
        uint16_t period_fusion_task;
        uint16_t period_fusion;
        uint16_t period_R;
        uint16_t period_GeR;
        uint16_t period_GaR;
        uint16_t period_O;
        uint16_t period_Grv;
        uint16_t period_LA;
        uint32_t batch_timeout;
        uint32_t batch_timeout_step;
        uint32_t batch_timeout_set;
        uint8_t batch_step_cnt_enable;
        uint8_t batch_step_dtc_enable;
        uint8_t batch_enable;
    } logging;
    struct {
        DECLARE_BITMAP(enable_sns, SENSOR_MAX);
    } polling;
    struct sensor_en_type enable_type;
    uint32_t period_FFETimer;
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

struct sensor_ctrl_param_str {
    struct sensor_ctrl_param_input_str  input_param;
    struct sensor_ctrl_param_output_str output_param;
};

struct acceleration_func {
    int32_t (*set_offsets)(int32_t* offsets);
    int32_t (*get_offsets)(int32_t* offsets);
    int32_t (*set_ac_offsets)(struct acceleration offsets);
    int32_t (*get_ac_offsets)(struct acceleration* offsets);
    int32_t (*set_nrfilt_prm)(struct nr_filt_prm prm);
    int32_t (*get_nrfilt_prm)(struct nr_filt_prm *prm);
    void (*clear_filt_coff)(void);
    void (*set_ts_nrfilt_info)(int32_t ts_base_val);
    uint8_t (*get_devid)(void);
    int8_t  (*set_dcalib)(bool enable);
    int8_t  (*set_selfchk_precfg)(void);
    int8_t  (*set_selfchk_cfg)(uint8_t type);
};

struct magnetometer_func {
    int32_t (*set_offsets)(struct geomagnetic offsets);
    int32_t (*get_offsets)(struct geomagnetic* offsets);
    int32_t (*set_static_matrix)(int32_t* smat);
    int32_t (*get_static_matrix)(int32_t* smat);
    uint8_t (*get_devid)(void);
    int8_t  (*set_dcalib)(bool enable);
};

struct gyroscope_func {
    int32_t (*set_offsets)(struct gyroscope offsets);
    int32_t (*get_offsets)(struct gyroscope* offsets);
    int32_t (*set_nrfilt_prm)(struct nr_filt_prm prm);
    int32_t (*get_nrfilt_prm)(struct nr_filt_prm *prm);
    void (*clear_filt_coff)(void);
    void (*set_ts_nrfilt_info)(int32_t ts_base_val);
    uint8_t (*get_devid)(void);
    int8_t  (*set_dcalib)(bool enable);
    int8_t  (*set_selfchk_precfg)(void);
    int8_t  (*set_selfchk_cfg)(uint8_t type);
    int8_t  (*start_man_calib)(uint8_t *calib_prm);
    int8_t  (*wait_man_calib)(void);
};

struct gyrouc_func {
    void (*clear_filt_coff)(void);
    void (*set_ts_nrfilt_info)(int32_t ts_base_val);
};

struct pressure_func {
    int32_t (*set_offsets)(struct pressure offsets);
    int32_t (*get_offsets)(struct pressure* offsets);
    uint8_t (*get_devid)(void);
};

struct sgnfcntmtn_func {
    int32_t (*enable)(bool enable);
};

struct micon_app_cmn {
    int32_t     (*clear)(int32_t clr_req);
    int32_t     (*ts_set_nrfilt_prm)(struct nr_filt_prm prm);
    int32_t     (*ts_get_nrfilt_prm)(struct nr_filt_prm *prm);
};

struct dailys_func {
    int32_t (*enable)(bool enable, const struct pedo_param pp);
    int32_t (*send_ope_param)(bool enable, const struct pedo_param pp);
    int32_t (*send_latency_option)(enum sensor_e_type type, bool enable);
    int32_t (*get_pedo_data)(struct pedometer *arg_Pedo);
    int32_t (*get_vehi_data)(struct vehicle *arg_Vehi);
    int32_t (*event_mark_off)(void);
    void    (*get_mark_off_times)(int64_t *cur_kernel_time, int32_t *cur_micontask_time);
    void    (*get_mark_off_clrinfo)(uint8_t *is_clr_exist, int32_t *clr_miconts);
    void    (*vib_interlocking)(int32_t mode, int8_t interlocking);
};

struct barometer_func {
    int32_t (*enable)(bool enable);
};

struct vhdetect_func {
    int32_t (*enable)(bool enable);
    int32_t (*get_data)(struct vhdetect *arg_VHdetect);
};

struct kc_motion_det_func {
    int32_t (*wstart_enable)(bool enable);
    int32_t (*wstop_enable)(bool enable);
    int32_t (*train_enable)(bool enable);
    int32_t (*train_get_data)(struct kc_motion_train *arg_Train);
    int32_t (*other_vehi_enable)(bool enable);
    int32_t (*bringup_enable)(bool enable);
    int32_t (*bringup_get_data)(struct kc_motion_bringup_data *arg_Bringup);
};

struct under_water_det_func {
    int32_t (*enable)(bool enable);
};

struct vtrigger_func {
    int32_t (*enable)(bool enable);
};

struct sensor_micon_if_funcs {
    uint8_t (*get_tblidx2size)(uint8_t idx);
    uint8_t (*get_tblidx2smid)(uint8_t idx);
    uint8_t (*get_snstype2tblidx)(enum sensor_e_type type);
    uint8_t (*get_snstype2smid)(enum sensor_e_type type);
    int16_t (*get_fastest_snstask_val)(void);
    int32_t (*get_batch_fifo_size)(void);
    int32_t (*get_fw_version)(uint8_t *fw_ver);
    int32_t (*get_micon_info)(struct spi_device *client);
    void    (*get_sns_value)(enum sensor_e_type type, void *outdata);
    void    (*read_all_sns_val)(void);
    uint8_t (*get_miconshtdwn_status)(void);
    void    (*set_sensdrv_status)(enum sensor_drv_status_e_type current_status);
    int32_t (*write)(uint32_t adr, const uint8_t *data, uint8_t size);
    int32_t (*read)(uint32_t adr, uint8_t *data, uint16_t size);
    void    (*unit_chg)(struct sensor_ctrl_param_output_str *dst);
    int32_t (*send_sensor_ctrl)(struct sensor_ctrl_param_output_str *old,
                                struct sensor_ctrl_param_output_str *new,
                                bool force);
    int32_t (*send_logging_state)(struct sensor_ctrl_param_output_str *old,
                                     struct sensor_ctrl_param_output_str *new);
    void    (*set_flush)(enum sensor_e_type type);
    void    (*enable_irq_wake)(bool enable);
    void    (*waiting_for_access_chk)(void);
    int32_t (*batch_mark_off)(int32_t trigger, bool report);
    int32_t (*iio_report_event_now)(enum sensor_e_type type);
    int32_t (*init_snsdriver)(void);
    void    (*irq_proc)(uint64_t result);
    int32_t (*set_dev_param)(void);
    int32_t (*reset_saveparam_and_taskoff)(void);
    int32_t (*set_sns_initcmd)(void);
    int32_t (*enable_i2c_peripheral)(bool enable);
    void    (*set_batch_notify)(bool enable);
    void    (*clr_batch_condition)(void);
    struct acceleration_func acc_func;
    struct magnetometer_func mag_func;
    struct gyroscope_func gyro_func;
    struct gyrouc_func gyrouc_func;
    struct pressure_func press_func;
    struct sgnfcntmtn_func sgnfcnt_func;
    struct micon_app_cmn app_cmn;
    struct dailys_func ds_func;
    struct barometer_func baro_func;
    struct vhdetect_func vhdet_func;
    struct kc_motion_det_func kcmot_func;
    struct under_water_det_func uw_func;
    struct vtrigger_func vt_func;
};

struct sensor_batch_str {
    char* buffer_p;
    enum sensor_batch_report_e_type repo_type;
};

struct sensor_batch_enable_param_str {
    uint8_t step_cnt;
    uint8_t step_det;
    uint8_t timer_enable;
    uint8_t new_step_cnt;
    uint8_t sns_enable;
    uint8_t fusion_enable;
};

struct sensor_batch_set_param_str {
    uint8_t  period;
    uint32_t timeout;
};

//extern struct mutex s_tSpiMutex;
extern bool g_bDevIF_Error;
extern bool g_micon_error;
//extern wait_queue_head_t s_tWaitInt;
extern atomic_t g_bIsResume;
extern atomic_t g_bIsIntIrqEnable;
//extern atomic_t g_FWUpdateStatus;
extern uint16_t g_lastCmdError;
extern bool g_micon_pedo_status;
extern bool g_micon_baro_status;
extern int32_t g_interlocking;
extern bool g_AppVhFirstReportDone;
extern uint8_t g_acc_available;
extern uint8_t g_gyro_available;
extern uint8_t g_mag_available;
extern uint8_t g_pres_available;

/* function ml630q790.c */
//int32_t sns_spi_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size);
int32_t sns_device_write(uint8_t adr, const uint8_t *data, uint8_t size);
int32_t sns_device_read(uint32_t adr, uint8_t *data, uint16_t size);
/* function sensor_micon_driver.c */
int32_t sns_logging_state(
    uint32_t type,
    uint32_t arg_timeout_ms,
    uint32_t arg_period_ms
);
int32_t sns_micon_get_batch_data(struct sensor_batch_str* batch_p);
int32_t sns_ts_set_nrfilt_param(struct nr_filt_prm prm);
int32_t sns_ts_get_nrfilt_param(struct nr_filt_prm *prm);
int32_t sns_acc_activate(bool arg_iEnable);
int32_t sns_acc_read_data(struct acceleration *arg_Acc);
int32_t sns_acc_set_offset(int32_t* offsets);
int32_t sns_acc_get_offset(int32_t* offsets);
int32_t sns_acc_set_auto_cal_offset(struct acceleration offsets);
int32_t sns_acc_get_auto_cal_offset(struct acceleration *offsets);
int32_t sns_acc_set_nrfilt_param(struct nr_filt_prm prm);
int32_t sns_acc_get_nrfilt_param(struct nr_filt_prm *prm);
void sns_acc_set_ts_nrfilt_baseinfo(int32_t ts_base_val);
uint8_t sns_acc_get_deviceid(void);
int32_t sns_acc_set_dynamiccalib(bool enable);
int8_t sns_acc_set_selfchk_config(enum selfchk_test_e_type type);
int32_t sns_acc_set_host_cmd(int32_t* req_cmd, int32_t* req_param);
void sns_acc_get_host_cmd(uint32_t* res);
void sns_dailys_set_vib_interlocking(int32_t mode);
void sns_dailys_get_event_markoff_timestamps(int64_t *cur_kernel_time,
                                                int32_t *cur_micontask_time);
void sns_dailys_get_event_markoff_clrinfo(uint8_t *is_clr_exist, int32_t *clr_miconts);
int32_t sns_mag_activate(bool arg_iEnable);
int32_t sns_mag_read_data(struct geomagnetic *arg_Mag);
int32_t sns_mag_set_offset(struct geomagnetic offsets);
int32_t sns_mag_get_offset(struct geomagnetic *offsets);
int32_t sns_mag_set_accuracy(int8_t accuracy);
int32_t sns_mag_get_accuracy(int8_t* accuracy);
int32_t sns_mag_get_static_matrix(int32_t* static_matrix);
int32_t sns_mag_set_static_matrix(int32_t* static_matrix);
uint8_t sns_mag_get_deviceid(void);
int32_t sns_mag_set_dynamiccalib(bool enable);
int32_t sns_gyro_activate(bool arg_iEnable);
int32_t sns_gyro_read_data(struct gyroscope *arg_Gyro);
int32_t sns_gyro_set_offset(struct gyroscope offsets);
int32_t sns_gyro_get_offset(struct gyroscope *offsets);
int32_t sns_gyro_set_nrfilt_param(struct nr_filt_prm prm);
int32_t sns_gyro_get_nrfilt_param(struct nr_filt_prm *prm);
void sns_gyro_set_ts_nrfilt_baseinfo(int32_t ts_base_val);
void sns_gyrouc_set_ts_nrfilt_baseinfo(int32_t ts_base_val);
uint8_t sns_gyro_get_deviceid(void);
int32_t sns_gyro_set_dynamiccalib(bool enable);
int8_t  sns_gyro_set_selfchk_config(enum selfchk_test_e_type selfchk_type);
int8_t  sns_gyro_start_manual_cal(uint8_t *cal_prm);
int8_t  sns_gyro_wait_cal_result(void);
int32_t sns_maguncalib_activate(bool arg_iEnable);
int32_t sns_maguncalib_read_data(struct mag_uncalib *arg_Mag);
void sns_gyrouc_set_ts_nrfilt_baseinfo(int32_t ts_base_val);
int32_t sns_gyrouncalib_activate(bool arg_iEnable);
int32_t sns_gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro);
void    sns_mag_offset_read_data(struct geomagnetic *arg_Mag);
void    sns_gyro_offset_read_data(struct gyroscope *arg_Gyro);
int32_t sns_gravity_activate(bool arg_iEnable);
int32_t sns_gravity_read_data(struct gravity *arg_Gravity);
int32_t sns_linacc_activate(bool arg_iEnable);
int32_t sns_linacc_read_data(struct linear_acceleration *arg_linacc);
int32_t sns_ori_activate(bool arg_iEnable);
int32_t sns_ori_read_data(struct orientation *arg_ori);
int32_t sns_rota_activate(bool arg_iEnable);
int32_t sns_rota_read_data(struct rotation_vector *arg_rota);
int32_t sns_gamerota_activate(bool arg_iEnable);
int32_t sns_gamerota_read_data(struct game_rotation_vector *arg_gamerota);
int32_t sns_magrota_activate(bool arg_iEnable);
int32_t sns_magrota_read_data(struct geomagnetic_rotation_vector *arg_magrota);
int32_t sns_pressure_activate(bool arg_iEnable);
int32_t sns_pressure_read_data(struct pressure *arg_magrota);
int32_t sns_pressure_base_val(bool req, DailysSetBaseParam *DailysBaseParam);
int32_t sns_pressure_set_param(struct pressure *arg_pressure);
int32_t sns_pressure_get_param(struct pressure *arg_pressure);
int32_t sns_pressure_set_cal_ofs(struct pressure offset);
uint8_t sns_pressure_get_deviceid(void);
int32_t sns_motion_activate(bool arg_iEnable);
int32_t sns_motion_start(bool arg_iStart);
int32_t sns_scount_activate(bool arg_iEnable);
int32_t sns_sdetect_activate(bool arg_iEnable);
int32_t sns_dailys_activate(bool arg_iEnable);
int32_t sns_dailys_start(bool arg_iStart);
void    sns_set_pedo_param(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iVehiType);
int32_t sns_get_pedo_data(struct pedometer *arg_Pedo);
int32_t sns_get_vehi_data(struct vehicle *arg_Vehi);
int32_t sns_baro_start(bool arg_iStart);
int32_t sns_iwifi_activate(bool arg_iEnable);
int32_t sns_iwifi_start(bool arg_iStart);
int32_t sns_get_iwifi_data(struct iwifi *arg_Vehi);
int32_t sns_iwifi_set_info(bool req, DailysSetIWifiParam *DailysIWifiParam);
int32_t sns_pedom_clear(int32_t clear_req);
int32_t sns_vhdetect_activate(bool arg_iEnable);
int32_t sns_vhdetect_start(bool arg_iStart);
int32_t sns_get_vhdetect_data(struct vhdetect *arg_VHdetect);
bool    sns_is_vhdetect_first_report_done(void);
int32_t sns_motion_detect_start(bool arg_iStart);
int32_t sns_stationary_detect_start(bool arg_iStart);
int32_t sns_flush_event_buffer(void);

int32_t sns_kc_motion_walk_start_activate(bool arg_iEnable);
int32_t sns_kc_motion_walk_start_start(bool arg_iStart);
int32_t sns_kc_motion_walk_stop_activate(bool arg_iEnable);
int32_t sns_kc_motion_walk_stop_start(bool arg_iStart);
int32_t sns_get_kc_motion_walk_data(struct kc_motion_walk_data *arg_Walk);
int32_t sns_kc_motion_train_activate(bool arg_iEnable);
int32_t sns_kc_motion_train_start(bool arg_iStart);
int32_t sns_get_kc_motion_train_data(struct kc_motion_train *arg_Train);
int32_t sns_kc_motion_vehicle_activate(bool arg_iEnable);
int32_t sns_kc_motion_vehicle_start(bool arg_iStart);
int32_t sns_kc_motion_bringup_activate(bool arg_iEnable);
int32_t sns_kc_motion_bringup_start(bool arg_iStart);
int32_t sns_get_kc_motion_bringup_data(struct kc_motion_bringup_data *arg_Bringup);

int32_t sns_uwater_start(bool arg_iStart);
int32_t sns_get_uwater_data(struct uwater_detect_info *arg_uwdInfo);
int32_t sns_uwater_clear(void);
int32_t sns_voice_trigger_start(bool arg_iStart);
int32_t sns_set_dev_param(void);
int32_t sns_initialize( void );
int32_t sns_get_fw_version(uint8_t *arg_iData);
void sns_BRMP_direction(void);
int32_t sns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen);
int32_t sns_update_fw(uint8_t *arg_iData, uint32_t arg_iLen);
int32_t sns_err_check(void);
int32_t sns_get_reset_status(void);
void sns_enable_irq_wake_irq(bool arg_iEnable);
void sns_set_buff_int(bool arg_iEnable);

enum sensor_e_type;
struct sensor_batch_info_str;
struct sensor_poll_info_str;
void sns_poll_init(enum sensor_e_type type, struct sensor_poll_info_str* poll_info);
int32_t sns_set_enable(enum sensor_e_type type, bool enable);
int32_t sns_set_batch(enum sensor_e_type type, struct sensor_batch_info_str *info);
void sns_set_flush(enum sensor_e_type type , struct input_dev* dev);
void sns_get_logging_enable(unsigned long *logging_enable_status);
int32_t sns_suspend(void);
int32_t sns_resume(void);
int32_t sns_shutdown(void);
void sns_set_period(uint32_t type,int32_t polltime, uint8_t flag);
void sns_set_app_task( uint32_t type ,uint8_t onoff);
void sns_set_app_task_batch( uint32_t type );
int sns_iio_report_event_now(enum sensor_e_type type);

int32_t sensor_micon_init(void);
void sensor_micon_exit(void);

void sns_load_offset_value(enum sensor_e_type type, void *outdata);
void sns_save_offset_value(enum sensor_e_type type, void *savedata);
void sns_notify(enum sensor_e_type type, uint32_t data);
int32_t sns_logging_punctuation(int32_t trigger);
void get_fusion_sns_mask(unsigned long *dst, unsigned int nbits);
struct sensor_ctrl_param_str get_sns_ctrl_param(void);
int32_t sns_pressure_get_cal_ofs(struct pressure *offset);
void sns_logging_setup_basetime(unsigned long *logging_new, unsigned long *logging_old);
bool sns_is_needed_gyro_sync(void);
int sns_iio_report_event(uint8_t id, uint8_t *data, uint32_t len, int64_t timestamp, int64_t offset);
void sns_logging_proceed_time(int64_t *timestamps);
int64_t get_logging_base_timestamp(uint8_t tbl_idx);
void sns_invalid_setting(uint8_t acc_available, uint8_t mag_availble,
                        uint8_t gyro_available, uint8_t press_available);
void sns_pm_wakeup_event(uint32_t timeout_msec);

#endif /* SENSOR_MICOM_H */
