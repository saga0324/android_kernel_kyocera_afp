/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
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
#ifndef SENSOR_DRVIVER_H
#define SENSOR_DRVIVER_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/version.h>
#include <linux/slab.h>

//#include <stdint.h>
//#include "gp2ap030a.h"
#ifdef CONFIG_INPUT_SENSOR_RPR0521
#include "rohm_rpr0521_i2c.h"
#endif
#ifdef CONFIG_INPUT_SENSOR_APDS9960
#include "sns_dd_apds9960_priv.h"
#endif
#ifdef CONFIG_INPUT_SENSOR_STK3338
#include "sensortek_stk3338_i2c.h"
#endif
#ifdef CONFIG_INPUT_TI_MSP430
#include "msp430.h"
#endif

#undef SENSOR_DRIVER_DEBUG

#ifdef SENSOR_DRIVER_DEBUG
#define SENSOR_D_LOG(msg, ...) \
    pr_notice("[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_V_LOG(msg, ...) \
    pr_notice("[SENSOR][%s][V](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define SENSOR_D_LOG(msg, ...) \
    pr_debug("[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_V_LOG(msg, ...) \
    pr_debug("[SENSOR][%s][V](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#ifdef SENSOR_DRIVER_DEBUG
#define SENSOR_D_DUMP(prefix_str, prefix_type, buf, len) \
    print_hex_dump(KERN_NOTICE, prefix_str, prefix_type, 16, 1, buf, len, false)
#else
#if defined(CONFIG_DYNAMIC_DEBUG)
#define SENSOR_D_DUMP(prefix_str, prefix_type, buf, len) \
    dynamic_hex_dump(prefix_str, prefix_type, 16, 1, buf, len, false)
#else
#define SENSOR_D_DUMP(prefix_str, prefix_type, buf, len)
#endif /* defined(CONFIG_DYNAMIC_DEBUG) */
#endif

#define SENSOR_TI_LOG(msg, ...) \
pr_debug("[SENSOR][%s][D_TI](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_N_LOG(msg, ...)
//printk(KERN_NOTICE "[SENSOR][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_A_LOG(msg, ...) 
//printk(KERN_NOTICE "[SENSOR][%s][A](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_ERR_LOG(msg, ...) \
printk(KERN_ERR "[SENSOR][%s][F](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
  #ifdef CONFIG_USE_SENSOR_LSM6DS3
    #define ACC_SLV_ADDR      (0x6A)
    #define ACC_DEV_ID_ADDR   (0x0F)
  #elif CONFIG_USE_SENSOR_LIS2DH12
    #define ACC_SLV_ADDR      (0x19)
    #define ACC_DEV_ID_ADDR   (0x0F)
  #endif
#else
    #define ACC_SLV_ADDR      (0x00)
    #define ACC_DEV_ID_ADDR   (0x00)
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
  #ifdef CONFIG_INPUT_SENSOR_YAS_MAG
    #define MAG_SLV_ADDR      (0x2E)
    #define MAG_DEV_ID_ADDR   (0x80)
  #elif CONFIG_INPUT_SENSOR_HSCDTD008A_MAG
    #define MAG_SLV_ADDR      (0x0C)
    #define MAG_DEV_ID_ADDR   (0x0F)
  #endif
#else
  #define MAG_SLV_ADDR      (0x00)
  #define MAG_DEV_ID_ADDR   (0x00)
#endif /* CONFIG_INPUT_SENSOR_GEOMAGNETIC */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
  #ifdef CONFIG_USE_SENSOR_LSM6DS3
    #define GYRO_SLV_ADDR      (0x6A)
    #define GYRO_DEV_ID_ADDR   (0x0F)
  #endif
#else
  #define GYRO_SLV_ADDR         (0x00)
  #define GYRO_DEV_ID_ADDR      (0x00)
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    #define PRESS_SLV_ADDR     (0x5D)
    #define PRESS_DEV_ID_ADDR  (0x10)
#else
    #define PRESS_SLV_ADDR     (0x00)
    #define PRESS_DEV_ID_ADDR  (0x00)
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */

#define ABS_WAKE             (ABS_MISC)
#define ABS_STATUS           (ABS_BRAKE)

#define INPUT_REPORT_NUM (36)
#define SENSOR_BATCH_REPORT_SIZE_MAG_UNCAL  (14)
#define SENSOR_BATCH_REPORT_SIZE_GYRO_UNCAL (13)

#define INTERLOCKING_ON                     (0x01)
#define INTERLOCKING_OFF                    (0x00)
#define VIB_INTERLOCKING_ON                 0x01
#define VIB_INTERLOCKING_OFF                ~VIB_INTERLOCKING_ON
#define SPEAKER_INTERLOCKING_ON             0x02
#define SPEAKER_INTERLOCKING_OFF            ~SPEAKER_INTERLOCKING_ON

#define OFS_A_L_LIMIT      (-819)
#define OFS_A_U_LIMIT      (819)
#define OFS_M_L_LIMIT      (-32768)
#define OFS_M_U_LIMIT      (32768)
#define OFS_G_L_LIMIT      (-32768)
#define OFS_G_U_LIMIT      (32768)
#define OFS_TMP_L_LIMIT     (-32767)
#define OFS_TMP_U_LIMIT     (32767)
#define SENSOR_TYPE_STATUS_LOG_SIZE         (((SENSOR_MAX + (BITS_PER_LONG - 1)) / BITS_PER_LONG) * 9)

#define SOFT_VERSION    0x0F
#define SOFT_BUILD_DATE 20190828

enum sensor_type_status_e_type{
    SENSOR_OFF = 0,
    SENSOR_ON  = 1,
};

enum sensor_batch_status_e_type{
    BATCH_OFF = 0,
    BATCH_ON  = 1,
};

enum sensor_drv_status_e_type{
    SENSOR_POWER_OFF = 0,
    SENSOR_FW_UPDATE,
    SENSOR_RESUME,
    SENSOR_SUSPEND,
    SENSOR_FAILURE_FW_UPDATE,
    SENSOR_FAILURE_ACCESS,
    SENSOR_SHUTDOWN,
    SENSOR_RESET,
};

enum sensor_chk_status_e_type{
    SENSOR_CHK_NORMAL = 0,
    SENSOR_CHK_SKIP_CTL,
    SENSOR_CHK_SKIP_ALL,
    SENSOR_CHK_POLL_CTL,
    SENSOR_CHK_MICON_CTL,
};

enum sensor_e_type{
    SENSOR_ACC = 0,
    SENSOR_ACC_AUTO_CAL,
    SENSOR_MAG,
    SENSOR_MAG_UNCAL,
    SENSOR_MAG_ROT_VCTR,
    SENSOR_ACC_LNR,
    SENSOR_GRV,
    SENSOR_GYRO,
    SENSOR_GYRO_UNCAL,
    SENSOR_ORTN,
    SENSOR_ROT_VCTR,
    SENSOR_GAME_ROT_VCTR,
    SENSOR_LIGHT,
    SENSOR_PROX,
    SENSOR_GESTURE,
#if 0
    SENSOR_TEMP,
    SENSOR_TEMP_AMBIENT,
    SENSOR_RH,
#endif
    SENSOR_PRESSURE,
    SENSOR_SGNFCNT_MTN,
    SENSOR_STEP_CNT,
    SENSOR_STEP_DTC,
    SENSOR_DEVICE_ORIENTATION,
    SENSOR_STATIONARY_DETECT,
    SENSOR_MOTION_DETECT,
    SENSOR_EXT_PEDO,
    SENSOR_EXT_VEHI,
    SENSOR_EXT_BARO,
    SENSOR_EXT_IWIFI,
    SENSOR_EXT_VH,
    SENSOR_KC_MOTION_WALK_START,
    SENSOR_KC_MOTION_WALK_STOP,
    SENSOR_KC_MOTION_TRAIN,
    SENSOR_KC_MOTION_VEHICLE,
    SENSOR_KC_MOTION_BRINGUP,
    SENSOR_UNDERWATER_DETECT,
    SENSOR_COM,
    SENSOR_PIEZO_PRESS,
    SENSOR_PIEZO_WAKEUP,
    SENSOR_VOICE_TRIGGER,
    SENSOR_MAX
};
#ifdef CONFIG_INPUT_SENSOR_VT_WAKEUP
#define VT_WAKEUP (SENSOR_VOICE_TRIGGER+1)
#endif
enum sensor_batch_report_e_type{
    SENSOR_COMP_BATCH = 0,
    SENSOR_COMP_FLUSH = 1,
    SENSOR_COMP_TIME  = 2,
    SENSOR_COMP_ABS   = 3,
    SENSOR_COMP_FLUSH_BF = 4
};

enum noizereduction_e_type{
    NR_ACC  = 0,
    NR_GYRO,
    NR_TIME,
    NR_TYPE_MAX
};

struct base_reliability {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct base_value {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct base_hysteresis_value {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct nr_filt_prm {
    union {
        struct {
            int32_t th_l;
            int32_t th_h;
            int32_t coff;
            int32_t rel_th;
            int32_t base_filt_coff;
            int32_t base_filt_acoff;
            int32_t tolerance;
            int32_t base_filt_hys;
            int32_t filt_lmt_th;
        };
        struct {
            int32_t ts_th_us;
            int32_t ts_tol_us;
            int32_t ts_rel_th;
            int32_t ts_base_filt_coff;
            int32_t ts_base_filt_acoff;
            int32_t ts_filt_coff;
            int32_t ts_ng_th;
            int32_t ts_unaccept_num;
            int32_t ts_reserved1;
            int32_t ts_reserved2;
        };
    };
};

struct timestamp_filter_val {
    int16_t base_rel;
    int64_t base_val;
    int64_t base_set_val;
    int16_t ng_cnt;
    int16_t luck_sample_num;
    int64_t prv_ts;
    int64_t prv_raw_ts;
    int16_t init_done_flg;
};

struct ts_filt_prm {
    int32_t th_l;
    int32_t th_h;
    int32_t coff;
    int32_t rel_th;
    int32_t base_filt_coff;
    int32_t base_filt_acoff;
    int32_t tolerance;
    int32_t base_filt_hys;
};

enum selfchk_test_e_type {
    SELFCHK_PRE = 0,
    SELFCHK_PLUS,
    SELFCHK_MINUS,
    SELFCHK_OFF,
};

enum manual_calib_e_type {
    SETTING = 0,
    WAIT,
    DONE
};

struct acceleration {
    int32_t nX;
    int32_t nY;
    int32_t nZ;
    int32_t outX;
    int32_t outY;
    int32_t outZ;
};

struct geomagnetic {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t accuracy;
};

struct gyroscope {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct mag_uncalib {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t cal_x;
    int32_t cal_y;
    int32_t cal_z;
};

struct gyro_uncalib {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t cal_x;
    int32_t cal_y;
    int32_t cal_z;
};

struct orientation {
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
    uint8_t accuracy;
};

struct gravity {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct linear_acceleration {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct rotation_vector {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t accuracy;
};

struct geomagnetic_rotation_vector {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t s;
    uint8_t accuracy;
};

struct game_rotation_vector {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t s;
};

struct step_counter {
    uint16_t step;
};

struct pressure {
    int32_t pressure;
};

struct sens_ofs {
    int32_t x;
    int32_t y;
    int32_t z;
    uint8_t accuracy;
};

struct pedometer {
    int32_t usStepCnt;
    int32_t usWalkTime;
    int32_t usCal;
    int32_t usRTState;
    int32_t usBodyFat;
    int32_t usExercise;
    int32_t usMets;
    int32_t usSpeed;
    int32_t usRunStatus;
    int32_t usRunStepCnt;
    int32_t usRunTime;
    int32_t usStExercise;
    int32_t usStCal;
    int32_t usStBodyFat;
    int32_t usSportExercise;
    int32_t usSyncRunStepCnt;
    int32_t usSyncRunTime;
    int32_t usSyncRunCal;
    int32_t usRunCal;
    int32_t usRunExercise;
    int32_t usPS_InitBaseHeight;
    int32_t usPS_ActHeight;
    int32_t usPS_ActHeightMin;
    int32_t usPS_ActHeightAve;
    int32_t usPS_ActHeightMax;

    int32_t LogTimeStamp3sec;
    int32_t PD_Log_DetState;
    int32_t PD_Log_SectionStep;
    int32_t PD_Log_SectionCal;
    int32_t RN_Log_SectionStep;
    int32_t RN_Log_SectionCal;
    int32_t PD_Log_SectionUpH;
    int32_t PD_Log_SectionDnH;
    int32_t PD_Log_SectionUpDnCal;
    int32_t VC_Log_DetState;
    int32_t VC_Log_SectionCal;
    int32_t VC_Log_SectionUpH;
    int32_t VC_Log_SectionDnH;
    int32_t VC_Log_SectionUpDnCal;
    int32_t PS_SectionPress;
    int32_t PS_SectionHeight;
    int32_t PS_SectionActiveHeight;
};

struct vehicle {
    int32_t usVehiStatus;
    int32_t usVehiKind;
    int32_t usVehiDetectTime;
    int32_t usVehiRideTime;
    int32_t usVehiRideCal;
    int32_t usVehiBodyFat;
    int32_t usVehiExercise;
    int32_t usVehiMets;
    int32_t usVehiStExercise;
    int32_t usVehiStRideCal;
    int32_t usVehiStBodyFat;
    int32_t usVehiBiExercise;
    int32_t usVehiBiRideCal;
    int32_t usVehiBiBodyFat;
    int32_t usVehiSportExercise;
    int32_t usVehiOtherRideTime;
    int32_t usVehiBicycRideTime;
    int32_t usVehiTrainRideTime;
};

struct iwifi{
    int32_t usPedoStatus;
    int32_t usVehiStatus;
};

struct vhdetect{
    int32_t usVhStatus;
};

struct pedo_param{
    int32_t weight;
    int32_t step_wide;
    int32_t vehi_type;
};

typedef struct {
  uint32_t m_nPedoStartStep;
  uint32_t m_nPedoEndTime;
  uint32_t m_nVehiStartTime;
  uint32_t m_nVehiEndTime;
}DailysSetIWifiParam;

typedef struct {
  uint32_t m_SetMode;
  uint32_t m_BasePress;
  uint32_t m_BaseHeight;
}DailysSetBaseParam;

struct kc_motion_train {
    uint8_t usTrFstDetect;
    uint8_t usTrOtherFstDetect;
    uint8_t usTrRes;
    uint32_t TrDtctTime;
    uint32_t TrDtctTimeFix;
};

struct kc_motion_walk_data {
    uint32_t status;
};

struct kc_motion_vehicle_data {
    uint32_t status;
    uint32_t kind;
};

struct kc_motion_bringup_data {
    uint32_t status;
};

struct uwater_detect_info {
    bool uwater_detected;
};


typedef enum{
    PS_CAL_RESULT_DUMMY=-5,
    PS_CAL_ERR_OTHER,
    PS_CAL_ERR_NEARLIGHT,
    PS_CAL_ERR_SUNLIGHT,
    PS_CAL_ERR_NEAROBJ,
    PS_CAL_OK,
    PS_CALIBRATING,
} PS_CAL_RESULT;

union sensor_read_data_u {
    struct acceleration acc_data;
    struct gyroscope gyro_data;
    struct geomagnetic mag_data;
    struct gyro_uncalib gyro_uncal_data;
    struct mag_uncalib mag_uncal_data;
    struct gravity grv_data;
    struct linear_acceleration acc_lnr_data;
    struct orientation ortn_data;
    struct rotation_vector rot_vctr_data;
    struct game_rotation_vector game_rot_vctr_data;
    struct geomagnetic_rotation_vector mag_rot_vctr_data;
    struct step_counter step_cnt_data;
    struct pressure pressure_data;
};

union sensor_ext_read_data_u {
    struct pedometer pedm_data;
    struct vehicle   vehi_data;
    struct iwifi     iwifi_data;
    struct vhdetect  vh_data;
};

union sensor_motion_read_data_u {
    struct kc_motion_walk_data walk_data;
    struct kc_motion_vehicle_data vehicle_data;
    struct kc_motion_bringup_data bringup_data;
};

typedef struct DailysSetIWifiParam sensor_ext_param_str;

typedef struct uwater_detect_info uwater_detect_info_t;

struct sensor_input_info_str {
    struct input_dev* dev;
    void (*param_func)( struct input_dev *dev );
    struct attribute_group* attr_grp;
};

struct sensor_poll_info_str {
    const char * name;
    atomic_t poll_time;
    struct workqueue_struct* poll_wq;
    void (*poll_func)(struct work_struct *work);
    struct work_struct work;
    struct hrtimer timer;
};

struct sensor_batch_data_str {
    int32_t payload_size;
    int32_t recode_num;
    void*  payload_p;
};
struct sensor_batch_info_str {
    int32_t flags;
    int32_t period_ns;
    int32_t timeout;
};
struct sensor_batch_timeout_str {
    enum sensor_e_type type;
    int32_t timeout;
};
struct sensor_batch_data_info_str {
    uint32_t id;
    uint32_t size;
    uint8_t* tmp_buff_p;
    struct sensor_batch_data_str data_info;
};

struct sensor_poll_ctl_str {
    enum sensor_e_type type;
    struct sensor_poll_info_str* poll_info_p;
};

struct sensor_ext_pedom_param_str {
    uint32_t weight;
    uint32_t step_wide;
    uint32_t vehi_type;
};

int sensor_input_init( struct sensor_input_info_str* info );

uint32_t sensor_get_status(enum sensor_e_type type);
void sensor_set_status(
    enum sensor_e_type type,
    enum sensor_type_status_e_type on
);

void sensor_poll_init(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info );
void sensor_enable(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info,
    bool enable
);
int32_t sensor_type_get_data(
    enum sensor_e_type type,
    union sensor_read_data_u* read_data );

void sensor_set_poll_time(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info,
    int32_t polltime
);
enum sensor_micon_polling_e_type sensor_get_poll_status(void);
void sensor_report_abs (
    struct input_dev* dev,
    int32_t x, int32_t y, int32_t z,
    int32_t last_x, int32_t last_y, int32_t last_z,
    int* rudder_cnt
);
void sensor_report_ac_abs (
    struct input_dev* dev,
    int32_t x, int32_t y, int32_t z,
    int32_t last_x, int32_t last_y, int32_t last_z,
    uint8_t accuracy,
    int* rudder_cnt
);
void sensor_report_abs_current_time(struct input_dev *dev);
void sensor_report_abs_time(struct input_dev *dev, s64 time_ns);
void sensor_report_event( enum sensor_e_type type, int64_t timestamp, void *event, uint32_t len );

uint32_t sensor_set_batch(
    enum sensor_e_type type,
    struct sensor_batch_info_str info);
uint32_t sensor_set_flush( 
    enum sensor_e_type type,
    struct input_dev* dev
    );
static inline void sensor_report_batch(
    struct input_dev* dev,
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data ) {}
void sensor_get_batch_status(unsigned long *logging_enable_status);
int32_t sensor_get_initialize_state(enum sensor_e_type type);

void sensor_ext_update_ofs_val(void);
int32_t sensor_ext_get_data(
    enum sensor_e_type type,
    union sensor_ext_read_data_u* read_data );
int32_t sensor_ext_set_param(
    enum sensor_e_type type,
    DailysSetIWifiParam* iwifi_param,
    struct sensor_ext_pedom_param_str* pedom_param );
void sensor_get_event_markoff_timestamps(int64_t *kts, int32_t *micontaskts);
void sensor_get_event_markoff_clrinfo(uint8_t *is_clr_exist, int32_t *clr_miconts);
int32_t sensor_ext_clear( uint32_t clear_req );
int32_t sensor_uwater_get_data(uwater_detect_info_t* read_data );
int32_t sensor_uwater_clear( void );
void sensor_flush_event_buffer(void);
void sensor_com_mutex_lock(void);
void sensor_com_mutex_unlock(void);
enum sensor_drv_status_e_type sensor_drv_get_status( void );
enum sensor_micon_polling_e_type sensor_get_poll_status_current(void);
void sensor_drv_set_status( enum sensor_drv_status_e_type next_status );
int sensor_set_ps_threshold( void );
void sensor_notify( enum sensor_e_type type, uint32_t data);
void sensor_report_data( enum sensor_e_type type,  uint32_t data );
void sensor_load_offset_value(enum sensor_e_type type, void *outdata);
void sensor_save_offset_value(enum sensor_e_type type, void *savedata);
//int32_t sensor_set_offset_value(enum sensor_e_type type, void *setdata);
int32_t sensor_set_offset_value(enum sensor_e_type type, struct sens_ofs set_ofs);
int32_t sensor_get_offset_value(enum sensor_e_type type, void* output);
int32_t sensor_set_nrfilt_param(enum noizereduction_e_type type, struct nr_filt_prm prm);
int32_t sensor_get_nrfilt_param(enum noizereduction_e_type type, struct nr_filt_prm *prm);
void sensor_restore_status(enum sensor_e_type type, void *restore_param);
uint8_t sensor_get_device_id(enum sensor_e_type type);
int8_t sensor_set_selfchk_config(enum sensor_e_type sns_type,
                                enum selfchk_test_e_type selfchk_type);
int8_t sensor_set_dynamiccalib_enable(enum sensor_e_type type,
                                    bool enable);
int8_t sensor_start_manual_calib(enum sensor_e_type sns_type,
                                uint8_t *cal_prm,
                                enum manual_calib_e_type cal_phase);
void sensor_suspend( void );
void sensor_resume( void );
void sensor_shutdown( void );
void sensor_reset( void );
void sensor_reset_resume( void );
void sensor_starttimer(struct hrtimer *timer, int delay_ms);
void sensor_stoptimer(struct hrtimer *timer);
enum hrtimer_restart sensor_poll(struct hrtimer *timer);

#endif /* SENSOR_DRVIVER_H */
