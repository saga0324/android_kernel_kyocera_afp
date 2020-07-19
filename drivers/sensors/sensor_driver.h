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
#ifndef SENSOR_DRVIVER_H
#define SENSOR_DRVIVER_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include "sensor_micon_driver.h"

#undef SENSOR_DRIVER_DEBUG

#ifdef SENSOR_DRIVER_DEBUG
#define SENSOR_D_LOG(msg, ...) \
    pr_notice("[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define SENSOR_D_LOG(msg, ...) \
    pr_debug("[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#ifdef SENSOR_DRIVER_DEBUG
#define SENSOR_D_DUMP(prefix_str, prefix_type, buf, len) \
    print_hex_dump(KERN_NOTICE, prefix_str, prefix_type, 16, 1, buf, len, false)
#else
#define SENSOR_D_DUMP(prefix_str, prefix_type, buf, len) \
    dynamic_hex_dump(prefix_str, prefix_type, 16, 1, buf, len, false)
#endif

#define SENSOR_N_LOG(msg, ...)
//printk(KERN_NOTICE "[SENSOR][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_A_LOG(msg, ...) 
//printk(KERN_NOTICE "[SENSOR][%s][A](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_ERR_LOG(msg, ...) \
printk(KERN_ERR "[SENSOR][%s][F](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define ABS_WAKE             (ABS_MISC)
#define ABS_STATUS           (ABS_BRAKE)

#define INPUT_REPORT_NUM (36)
#define SENSOR_BATCH_REPORT_SIZE_MAG_UNCAL  (14)
#define SENSOR_BATCH_REPORT_SIZE_GYRO_UNCAL (13)

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
#if 0
    SENSOR_TEMP,
    SENSOR_TEMP_AMBIENT,
    SENSOR_RH,
#endif
    SENSOR_PRESSURE,
    SENSOR_SGNFCNT_MTN,
    SENSOR_STEP_CNT,
    SENSOR_STEP_DTC,
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
    SENSOR_MAX
};

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
    struct delayed_work poll_work;
    void (*poll_func)(struct work_struct *work);
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
    enum sensor_micon_butch_id_e_type id;
    enum sensor_micon_butch_size_e_type size;
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
static inline int32_t sensor_copy_batch_data(
   enum sensor_micon_batch_tbl_num_e_type type,
   struct sensor_batch_data_str batch_data,
   uint8_t* hal_addr) { return 0; }
static inline int32_t sensor_copy_batch_uncal_data(
   enum sensor_micon_batch_tbl_num_e_type type,
   struct sensor_batch_data_str batch_data,
   struct sensor_batch_data_str batch_data_cal,
   uint8_t* hal_addr) { return 0; }
static inline void sensor_report_batch(
    struct input_dev* dev,
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data ) {}
uint32_t sensor_get_batch_status(void);
static inline int32_t sensor_timestamp_report(void) { return 0; }

int32_t sensor_ext_get_data(
    enum sensor_e_type type,
    union sensor_ext_read_data_u* read_data );
int32_t sensor_ext_set_param(
    enum sensor_e_type type,
    DailysSetIWifiParam* iwifi_param,
    struct sensor_ext_pedom_param_str* pedom_param );
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
void sensor_interrupt( enum sensor_e_type type, uint32_t data);
void sensor_report_data( enum sensor_e_type type,  uint32_t data );
void sensor_suspend( void );
void sensor_resume( void );
void sensor_shutdown( void );
void sensor_reset( void );
void sensor_reset_resume( void );

#endif /* SENSOR_DRVIVER_H */
