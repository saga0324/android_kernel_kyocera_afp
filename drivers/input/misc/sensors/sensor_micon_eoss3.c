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


#include <linux/ql/qleos_s3.h>
#include <linux/ql/qlspi_linux.h>
#include "sensor_micon_eoss3.h"
#include <linux/leds.h>
#include <asm/div64.h>

/* ------------------------------------------------
    Define
------------------------------------------------ */
#define BATCH_MARGIN_SIZE                   (512)
#define LED_GREEN_ON                        (0x0000FF00)
#define LED_RED_ON                          (0x00FF0000)
#define SNS_INIT_RETRY_NUM                  (3)
#define FASTEST_SENSTASK_PERIOD_MS          (0x00000001)
#define LOWEST_MAGCAL_PERIOD_MS             (0x00000064)
#define BATCH_DATA_SIZE_PER_ID              (12)
#define SENSOR_MICON_WAKELOCK_TIME          (500)
#define SENSOR_MICON_WAKELOCK_STATUS        (SIGN_MOTION_DETECTED | PD_STEP | VC_DETECTED | \
                                             EVENT_MARGIN_INTR | EVENT_MAX_LAT_INTR | \
                                             SP_CNG | BW_CNG | BU_CNG | TR_DETECTED | \
                                             TR_FST_DETECTED | PD_FST_OTH_DETECTED | UWD_WATER_DETECTED)
#define BATCH_MAX_READ_SIZE                 (2040)
#define UNMENTIONED                         (0x00000000)
#define EOS_CHECK_MAX_RETLY_NUM             (40)
/* ------------------------------------------------
    Variables
------------------------------------------------ */
static DEFINE_MUTEX(micon_hcres_mutex);
static DEFINE_MUTEX(acc_auto_cal_mutex);
static DEFINE_MUTEX(batch_markoff_mutex);
static DEFINE_MUTEX(acc_report_event_now_mutex);
static DEFINE_MUTEX(gyro_report_event_now_mutex);
static DEFINE_MUTEX(gyrouc_report_event_now_mutex);
static int64_t current_batch_kts = 0;
static int64_t previous_batch_kts = 0;
static uint32_t current_s3_ts[BATCH_TS_TYPE_MAX] = {0};
static uint32_t previous_s3_ts[BATCH_TS_TYPE_MAX] = {0};
static int32_t curent_ev_markoff_s3ts = 0;
static int64_t current_ev_markoff_kts = 0;
static uint8_t clr_exist_info = 0;
static int32_t clr_micon_timestamp = 0;
static uint16_t mark_off_buf_idx = 0;
static uint16_t mark_off_data_bytes = 0;
static uint8_t mark_off_buf[64*1024] = {0};
//static uint16_t ev_buf_idx = 0;
static int64_t eventbuf_cleartime;
static uint8_t event_mark_off_buf[4*1024] = {0};
static atomic_t s_snsdrv_status = ATOMIC_INIT((int)SENSOR_POWER_OFF);
static atomic_t is_1stBatch_proc = ATOMIC_INIT(true);
static struct acceleration                   last_acc_data       = {0};
static struct geomagnetic                    last_mag_data       = {0};
static struct gyroscope                      last_gyro_data      = {0};
static struct mag_uncalib                    last_maguc_data     = {0};
static struct gyro_uncalib                   last_gyrouc_data    = {0};
static struct orientation                    last_ori_data       = {0};
static struct gravity                        last_grv_data       = {0};
static struct linear_acceleration            last_lacc_data      = {0};
static struct rotation_vector                last_rv_data        = {0};
static struct geomagnetic_rotation_vector    last_georv_data     = {0};
static struct game_rotation_vector           last_gamrv_data     = {0};
static struct pressure                       last_press_data     = {0};
static int16_t no_noize_reduction_filt_counter = 0;

static struct acceleration prv_acc               = {0};
static struct base_reliability acc_rel           = {0};
static struct base_value acc_val                 = {0};
static struct base_hysteresis_value acc_hys_val  = {0};
static struct gyroscope prv_gyro                 = {0};
static struct base_reliability gyro_rel          = {0};
static struct base_value gyro_val                = {0};
static struct base_hysteresis_value gyro_hys_val = {0};
static struct timestamp_filter_val acc_ts        = {0};
static struct timestamp_filter_val gyro_ts       = {0};
static struct timestamp_filter_val gyrouc_ts     = {0};
static bool   acc_val_clear_flg                  = false;
static bool   gyro_val_clear_flg                 = false;
static bool   gyrouc_val_clear_flg               = false;
static int64_t acc_base_set_val_rcv              = 0;
static int64_t gyro_base_set_val_rcv             = 0;
static int64_t gyrouc_base_set_val_rcv           = 0;

static struct nr_filt_prm acc_nrfilt_param;
static struct nr_filt_prm gyro_nrfilt_param;
static struct nr_filt_prm ts_nrfilt_param;

static int16_t gyro_static_offset[3] = {0};
static int16_t gyro_dynamic_offset[3] = {0};

//static atomic_t g_nCalX = ATOMIC_INIT(0);
//static atomic_t g_nCalY = ATOMIC_INIT(0);
//static atomic_t g_nCalZ = ATOMIC_INIT(0);
static uint8_t g_acc_conv_axis = 0x00;
static uint8_t g_gyro_conv_axis = 0x00;
static uint8_t g_mag_conv_axis = 0x00;
static int16_t g_mag_static_matrix[9] ={10000, 0, 0, 0, 10000, 0, 0, 0, 10000};

static bool s_RedledOnstate = false;
static bool s_GreenledOnstate = false;
static int32_t s_LEDparam = 0x00000000;

static const uint8_t batch_id_to_tbl_idx[] =
{
    [SENSOR_BATCH_ID_ACC]           = SENSOR_BATCH_TBL_ACC,
    [SENSOR_BATCH_ID_MAG]           = SENSOR_BATCH_TBL_MAG,
    [SENSOR_BATCH_ID_GYRO]          = SENSOR_BATCH_TBL_GYRO,
    [SENSOR_BATCH_ID_MAG_UNCAL_RAW] = SENSOR_BATCH_TBL_MAG_UNCAL_RAW,
    [SENSOR_BATCH_ID_MAG_UNCAL_OFS] = SENSOR_BATCH_TBL_MAG_UNCAL_OFS,
    [SENSOR_BATCH_ID_GYRO_UNCAL_RAW] = SENSOR_BATCH_TBL_GYRO_UNCAL_RAW,
    [SENSOR_BATCH_ID_GYRO_UNCAL_OFS] = SENSOR_BATCH_TBL_GYRO_UNCAL_OFS,
    [SENSOR_BATCH_ID_PRESSURE]      = SENSOR_BATCH_TBL_PRESSURE,
    [SENSOR_BATCH_ID_ORTN]          = SENSOR_BATCH_TBL_ORTN,
    [SENSOR_BATCH_ID_GRV]           = SENSOR_BATCH_TBL_GRV,
    [SENSOR_BATCH_ID_ACC_LNR]       = SENSOR_BATCH_TBL_ACC_LNR,
    [SENSOR_BATCH_ID_ROT_VCTR]      = SENSOR_BATCH_TBL_ROT_VCTR,
    [SENSOR_BATCH_ID_GAME_ROT_VCTR] = SENSOR_BATCH_TBL_GAME_ROT_VCTR,
    [SENSOR_BATCH_ID_MAG_ROT_VCTR]  = SENSOR_BATCH_TBL_MAG_ROT_VCTR,
    [SENSOR_BATCH_ID_STEPCOUNTER]    = SENSOR_BATCH_TBL_STEPCOUNTER,
    [SENSOR_BATCH_ID_STEPDETECTOR]    = SENSOR_BATCH_TBL_STEPDETECTOR,
};

static const uint8_t tbl_idx_to_size[] =
{
    [SENSOR_BATCH_TBL_ACC]          = SENSOR_BATCH_SIZE_ACC,
    [SENSOR_BATCH_TBL_MAG_UNCAL_RAW]  = SENSOR_BATCH_SIZE_MAG_UNCAL_RAW,
    [SENSOR_BATCH_TBL_MAG_UNCAL_OFS]  = SENSOR_BATCH_SIZE_MAG_UNCAL_OFS,
    [SENSOR_BATCH_TBL_GYRO_UNCAL_RAW] = SENSOR_BATCH_SIZE_GYRO_UNCAL_RAW,
    [SENSOR_BATCH_TBL_GYRO_UNCAL_OFS] = SENSOR_BATCH_SIZE_GYRO_UNCAL_OFS,
    [SENSOR_BATCH_TBL_MAG]          = SENSOR_BATCH_SIZE_MAG,
    [SENSOR_BATCH_TBL_GYRO]         = SENSOR_BATCH_SIZE_GYRO,
    [SENSOR_BATCH_TBL_PRESSURE]     = SENSOR_BATCH_SIZE_PRESSURE,
    [SENSOR_BATCH_TBL_ORTN]         = SENSOR_BATCH_SIZE_ORTN,
    [SENSOR_BATCH_TBL_GRV]          = SENSOR_BATCH_SIZE_GRV,
    [SENSOR_BATCH_TBL_ACC_LNR]      = SENSOR_BATCH_SIZE_ACC_LNR,
    [SENSOR_BATCH_TBL_ROT_VCTR]     = SENSOR_BATCH_SIZE_ROT_VCTR,
    [SENSOR_BATCH_TBL_GAME_ROT_VCTR]= SENSOR_BATCH_SIZE_GAME_ROT_VCTR,
    [SENSOR_BATCH_TBL_MAG_ROT_VCTR] = SENSOR_BATCH_SIZE_MAG_ROT_VCTR,
    [SENSOR_BATCH_TBL_STEPCOUNTER]     = SENSOR_BATCH_SIZE_STEPCOUNTER,
    [SENSOR_BATCH_TBL_STEPDETECTOR]   = SENSOR_BATCH_SIZE_STEPDETECTOR,
};

static const uint8_t tbl_idx_to_nousesize[] =
{
    [SENSOR_BATCH_TBL_ACC]          = SENSOR_BATCH_NOUSE_SIZE_ACC,
    [SENSOR_BATCH_TBL_MAG_UNCAL_RAW]  = SENSOR_BATCH_NOUSE_SIZE_MAG_UNCAL_RAW,
    [SENSOR_BATCH_TBL_MAG_UNCAL_OFS]  = SENSOR_BATCH_NOUSE_SIZE_MAG_UNCAL_OFS,
    [SENSOR_BATCH_TBL_GYRO_UNCAL_RAW] = SENSOR_BATCH_NOUSE_SIZE_GYRO_UNCAL_RAW,
    [SENSOR_BATCH_TBL_GYRO_UNCAL_OFS] = SENSOR_BATCH_NOUSE_SIZE_GYRO_UNCAL_OFS,
    [SENSOR_BATCH_TBL_MAG]          = SENSOR_BATCH_NOUSE_SIZE_MAG,
    [SENSOR_BATCH_TBL_GYRO]         = SENSOR_BATCH_NOUSE_SIZE_GYRO,
    [SENSOR_BATCH_TBL_PRESSURE]     = SENSOR_BATCH_NOUSE_SIZE_PRESSURE,
    [SENSOR_BATCH_TBL_ORTN]         = SENSOR_BATCH_NOUSE_SIZE_ORTN,
    [SENSOR_BATCH_TBL_GRV]          = SENSOR_BATCH_NOUSE_SIZE_GRV,
    [SENSOR_BATCH_TBL_ACC_LNR]      = SENSOR_BATCH_NOUSE_SIZE_ACC_LNR,
    [SENSOR_BATCH_TBL_ROT_VCTR]     = SENSOR_BATCH_NOUSE_SIZE_ROT_VCTR,
    [SENSOR_BATCH_TBL_GAME_ROT_VCTR]= SENSOR_BATCH_NOUSE_SIZE_GAME_ROT_VCTR,
    [SENSOR_BATCH_TBL_MAG_ROT_VCTR] = SENSOR_BATCH_NOUSE_SIZE_MAG_ROT_VCTR,
    [SENSOR_BATCH_TBL_STEPCOUNTER]   = SENSOR_BATCH_NOUSE_SIZE_STEPCOUNTER,
    [SENSOR_BATCH_TBL_STEPDETECTOR]   = SENSOR_BATCH_NOUSE_SIZE_STEPDETECTOR,
};

static const uint8_t tbl_idx_to_smid[] =
{
    [SENSOR_BATCH_TBL_ACC]           = SHID_ACCELEROMETER,
    [SENSOR_BATCH_TBL_MAG]           = SHID_GEOMAGNETIC,
    [SENSOR_BATCH_TBL_GYRO]          = SHID_GYROSCOPE,
    [SENSOR_BATCH_TBL_MAG_UNCAL_RAW] = SHID_MAGNETIC_FIELD_UNCALIBRATED,
    [SENSOR_BATCH_TBL_MAG_UNCAL_OFS] = SHID_MAGNETIC_FIELD_UNCALIBRATED,
    [SENSOR_BATCH_TBL_GYRO_UNCAL_RAW]= SHID_GYROSCOPE_UNCALIBRATED,
    [SENSOR_BATCH_TBL_GYRO_UNCAL_OFS]= SHID_GYROSCOPE_UNCALIBRATED,
    [SENSOR_BATCH_TBL_PRESSURE]      = SHID_PRESSURE,
    [SENSOR_BATCH_TBL_ORTN]          = SHID_ORIENTATION,
    [SENSOR_BATCH_TBL_GRV]           = SHID_GRAVITY,
    [SENSOR_BATCH_TBL_ACC_LNR]       = SHID_LINEAR_ACCELERATION,
    [SENSOR_BATCH_TBL_ROT_VCTR]      = SHID_ROTATION_VECTOR,
    [SENSOR_BATCH_TBL_GAME_ROT_VCTR] = SHID_GAME_ROTATION_VECTOR,
    [SENSOR_BATCH_TBL_MAG_ROT_VCTR]  = SHID_GEOMAGNETIC_ROTATION_VECTOR,
    [SENSOR_BATCH_TBL_STEPCOUNTER]      = SHID_STEP_COUNTER,
    [SENSOR_BATCH_TBL_STEPDETECTOR]    = SHID_STEP_DETECTOR,
};

static const uint8_t sensor_type_to_tbl_idx[] =
{
    [SENSOR_ACC]            = SENSOR_BATCH_TBL_ACC,
    [SENSOR_MAG]            = SENSOR_BATCH_TBL_MAG,
    [SENSOR_MAG_UNCAL]      = SENSOR_BATCH_TBL_MAG_UNCAL_OFS,
    [SENSOR_MAG_ROT_VCTR]   = SENSOR_BATCH_TBL_MAG_ROT_VCTR,
    [SENSOR_ACC_LNR]        = SENSOR_BATCH_TBL_ACC_LNR,
    [SENSOR_GRV]            = SENSOR_BATCH_TBL_GRV,
    [SENSOR_GYRO]           = SENSOR_BATCH_TBL_GYRO,
    [SENSOR_GYRO_UNCAL]     = SENSOR_BATCH_TBL_GYRO_UNCAL_OFS,
    [SENSOR_ORTN]           = SENSOR_BATCH_TBL_ORTN,
    [SENSOR_ROT_VCTR]       = SENSOR_BATCH_TBL_ROT_VCTR,
    [SENSOR_GAME_ROT_VCTR]  = SENSOR_BATCH_TBL_GAME_ROT_VCTR,
    [SENSOR_PRESSURE]       = SENSOR_BATCH_TBL_PRESSURE,
    [SENSOR_STEP_CNT]       = SENSOR_BATCH_TBL_STEPCOUNTER,
    [SENSOR_STEP_DTC]       = SENSOR_BATCH_TBL_STEPDETECTOR,
};

static const uint8_t sensor_type_to_smid[] =
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

/* Lower Limit of offset value ACCELEROMETER, MAGNETOMETER, GYROSCOPE, BAROMETER */
static const int16_t sns_ofs_llimit[PHYSICAL_SENS_MAX] = {OFS_A_L_LIMIT, OFS_TMP_L_LIMIT, OFS_TMP_L_LIMIT, OFS_TMP_L_LIMIT};
/* Upper Limit of offset value ACCELEROMETER, MAGNETOMETER, GYROSCOPE, BAROMETER */
static const int16_t sns_ofs_ulimit[PHYSICAL_SENS_MAX] = {OFS_A_U_LIMIT, OFS_TMP_U_LIMIT, OFS_TMP_U_LIMIT, OFS_TMP_U_LIMIT};

/* ------------------------------------------------
    Prototype Functions
------------------------------------------------ */
extern void kclights_led_set_test(enum led_brightness value);

static uint8_t get_tbl_idx_to_size(uint8_t idx);
static uint8_t get_tbl_idx_to_smid(uint8_t idx);
static uint8_t get_sensor_type_to_tbl_idx(enum sensor_e_type type);
static uint8_t get_sensor_type_to_smid(enum sensor_e_type type);
static int16_t get_fastest_sensortask_val(void);
static int32_t get_batch_fifo_size(void);
static int32_t get_fw_version(uint8_t *fw_ver);
static void get_sns_value(enum sensor_e_type type, void *outdata);
static void read_all_sensor_value(void);
static void clr_sns_ofs_val(const uint8_t snstype);
static void chk_sns_ofs_val(int16_t *ofsval, const uint8_t snstype);
static uint8_t get_miconshtdwn_status(void);
static void set_sensordrv_status(enum sensor_drv_status_e_type current_status);
static bool is_sns_connected(const enum ql_sensor_type snstype);
static void dev_access_check(void);
static int32_t dev_write(uint32_t adr, const uint8_t *data, uint8_t size);
static int32_t dev_read(uint32_t adr, uint8_t *data, uint16_t size);
static void unit_change(struct sensor_ctrl_param_output_str *dst);
static void output_sns_param(int32_t *out, bool condition, int8_t sns_bit_point);
static int32_t get_calc_processing_param(uint32_t sns_enable);
static int32_t get_sensor_processing_param(uint32_t sns_enable);
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int32_t get_calc_batch_processing_param(uint32_t logging_enable);
static int32_t get_sensor_batch_processing_param(uint32_t sns_enable);
#endif

static int8_t set_FFETimer_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_sensor_task_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_sensor_proc_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_FFEapp_task_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_M4App_task_period(struct sensor_ctrl_param_output_str *new);
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t set_sensor_calc_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_FFEfusion_task_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_M4fusion_task_period(struct sensor_ctrl_param_output_str *new);
#endif
static void set_odr(uint16_t period_ms, const enum ql_sensor_type sensortype);
static void set_sensors_ODR(struct sensor_ctrl_param_output_str *new);
static int8_t enable_sensor_processing(struct sensor_ctrl_param_output_str *new);
static int8_t enable_calc_processing(struct sensor_ctrl_param_output_str *new);
static int8_t enable_sensor_task(struct sensor_ctrl_param_output_str *new);
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t enable_FFEFusion_task(struct sensor_ctrl_param_output_str *new);
#endif
static int8_t enable_FFEapp_task(struct sensor_ctrl_param_output_str *new);
static int8_t enable_FFETimer(struct sensor_ctrl_param_output_str *new);
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t enable_M4Fusion_Task(struct sensor_ctrl_param_output_str *new);
#endif
static int8_t enable_M4app_task(struct sensor_ctrl_param_output_str *new);
static int32_t send_sensor_ctrl(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new,
    bool force);
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t enable_batch_full_interrupt(uint8_t enable);
static int8_t enable_batch_interrupt(uint8_t enable, int16_t margin_buf_size);
static int8_t set_sensor_batch_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_fusion_batch_period(struct sensor_ctrl_param_output_str *new);
static int8_t set_enable_steplike_sensor(struct sensor_ctrl_param_output_str *new,
                                        int8_t sc_1st_en);
static int8_t set_batch_maxlatency(struct sensor_ctrl_param_output_str *new);
static int8_t set_enable_batch_sensor_processing(struct sensor_ctrl_param_output_str *new);
static int8_t set_enable_batch_fusion_processing(struct sensor_ctrl_param_output_str *new);
#endif
static int32_t send_logging_state(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new);
static void set_flush(enum sensor_e_type type);
static void enable_irq_wake_irq(bool enable);

static int64_t calc_elapsed_time(uint32_t start_time, uint32_t current_time);
static int64_t convert_host_sensortimestamp(uint32_t sensor_ts, enum batch_ts_type type);
static bool is_batch_id_valid(uint8_t batch_id);
static int32_t iio_report_events(uint8_t *batch_data, uint32_t len);
static void all_clr_batching_data(void);
static bool check_can_mark_off(const uint32_t now_addr, const uint32_t l_limit, const uint32_t u_limit);
static bool is_buf_circulated(struct get_batchbuf_info info, uint32_t r_size);
static int8_t read_batchdata(uint32_t s_ptr, int16_t size);
static int8_t read_batchdata_circulated(struct get_batchbuf_info info, uint32_t size, uint32_t *next_ptr);
static int8_t get_batch_buffer_data(struct get_batchbuf_info bd_info,
                                    int16_t *next_remain, uint32_t *next_ptr);
static void record_timestamp_data(uint32_t cur_ts_stask, uint32_t cur_ts_atask);
static void record_kernel_timestamp(int64_t current_ts);
static int8_t make_batch_markoff_param(enum batch_markoff_timing_type timing,
                                        const uint32_t cur_m4_rd_ptr,
                                        const uint32_t cur_ffe_rd_ptr,
                                        struct batch_markoff_param *out);
static int8_t run_batch_markoff_cmd(enum batch_markoff_timing_type timing,
                                    const uint32_t cur_m4_rd_ptr,
                                    const uint32_t cur_ffe_rd_ptr,
                                    struct batch_markoff_resp *resp);
static int32_t batch_mark_off_exec(bool is_discard);
static int32_t batch_mark_off(const int32_t trigger, bool report);
static int verify_sensor_data(const uint8_t h_val, const uint8_t t_val);
static int iio_report_event_now(enum sensor_e_type type);

static int32_t vtrigger_enable(bool enable);
static int32_t uwater_enable(bool enable);
static int32_t kcmot_get_bringup_data(struct kc_motion_bringup_data *arg_Bringup);
static int32_t kcmot_bringup_enable(bool enable);
static int32_t kcmot_other_vehi_enable(bool enable);
static int32_t kcmot_train_get_data(struct kc_motion_train *arg_Train);
static int32_t kcmot_train_enable(bool enable);
static int32_t kcmot_wstop_enable(bool enable);
static int32_t kcmot_wstart_enable(bool enable);
static int32_t vhdet_get_data(struct vhdetect *arg_VHdetect);
static int32_t vhdet_enable(bool enable);
static int32_t baro_enable(bool enable);
static void dailys_vib_interlocking(int32_t mode, int8_t interlocking);
static void record_evmarkoff_kernel_timestamp(int64_t current_ts);
static void record_evmarkoff_s3_timestamp(int32_t current_ts);
static void clear_evbuf_markoff_base_timestamps(void);
static int32_t dailys_event_mark_off(void);
static int32_t dailys_event_mark_off_exec(void);
static void get_event_mark_off_times(int64_t *cur_ktime, int32_t *cur_micontime);
static void get_event_mark_off_clrinfo(uint8_t *is_clr_exist, int32_t *clr_micontime);
static int32_t dailys_get_vehi_data(struct vehicle *arg_Vehi);
static int32_t dailys_get_pedo_data(struct pedometer *arg_Pedo);
static int32_t dailys_send_latency_option(enum sensor_e_type type, bool enable);
static int32_t dailys_send_param(bool enable, const struct pedo_param pp);
static int32_t dailys_enable(bool enable, const struct pedo_param pp);
static int32_t micon_app_clear(int32_t clr_req);

static void set_batch_notify(bool enable);
static int32_t enable_i2c_peri(bool enable);
static void clr_batch_condition(void);
static int32_t sns_initcmd_exe(void);
static int32_t reset_save_and_taskoff(void);
static void SetBaroEventBufferParam(const uint8_t *prm);
static void SetDailyStepEventBufferSetting(const uint8_t *prm);
static void SetDailyStepEventBufferHeightParam(const uint8_t *prm);
static void InitSetDailyStepBaroSetting(void);
static void M4AppTask_Buf_Size_FFE_to_M4(const uint16_t setBufSize);
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static void FFEFusion_Buf_Size_FFE_to_M4(const uint16_t setBufSize);
static void FFEFusion_set_init_wait_time(const uint16_t iwt_ms);
#endif
static void FFEAppTask_set_init_wait_time(const uint16_t iwt_ms);
static void get_Mag_SoftIronVal(int16_t *mat);
static void set_Mag_SoftIronVal(int16_t *mat);
static int32_t set_LPF_Coeff(const int16_t coeff, const enum ql_sensor_type sensortype);
static int32_t set_all_LPF_Coeff(void);
static int32_t set_Enable_LPFilter(const uint8_t enable, const enum ql_sensor_type sensortype);
static int32_t set_all_Enable_LPFilter(void);
static int32_t set_enable_updt_calval(const uint8_t enable, const enum ql_sensor_type sensortype);
static int32_t set_all_Enable_Update_Of_CalVal(void);
static int32_t set_enable_calib(const uint8_t enable, const enum ql_sensor_type sensortype);
static int32_t set_enable_cal_interrupt(const uint8_t enable, const uint8_t sensortype);
static int32_t set_all_Enable_Cal_Interrupt(void);
static int32_t set_calib_val(const int16_t *c_val, const uint8_t accuracy, const enum ql_sensor_type sensortype);
static int32_t set_all_calib_value(void);
static int32_t set_init_wait_time(const uint16_t iwt_ms, const enum ql_sensor_type sensortype);
static int32_t set_all_IWT(void);
static int32_t set_axis(const uint8_t axis, const enum ql_sensor_type sensortype);
static int32_t set_all_axis(void);
static int32_t InitSetting(void);
static int32_t set_ts_nrfilter_param(struct nr_filt_prm prm);
static int32_t get_ts_nrfilter_param(struct nr_filt_prm* prm);
static int32_t acc_set_auto_cal_offset_internal(struct acceleration offsets);
static int32_t acc_set_offsets(int32_t* offsets);
static int32_t acc_get_offsets(int32_t* offsets);
static int32_t acc_set_ac_offsets(struct acceleration offsets);
static int32_t acc_get_ac_offsets(struct acceleration* offsets);
static int32_t acc_set_nrfilter_param(struct nr_filt_prm prm);
static int32_t acc_get_nrfilter_param(struct nr_filt_prm* prm);
static uint8_t acc_get_device_id(void);
static int8_t  acc_set_dynamic_calib(bool enable);
static int8_t  acc_set_selfchk_presetting(void);
static int8_t  acc_set_selfchk_cfg(uint8_t type);
static int32_t mag_set_offsets(struct geomagnetic offsets);
static int32_t mag_get_offsets(struct geomagnetic* offsets);
static int32_t mag_set_static_matrix(int32_t* static_matrix);
static int32_t mag_get_static_matrix(int32_t* static_matrix);
static uint8_t mag_get_device_id(void);
static int8_t  mag_set_dynamic_calib(bool enable);
static int32_t gyro_set_offsets(struct gyroscope offsets);
static int32_t gyro_get_offsets(struct gyroscope* offsets);
static int32_t gyro_set_nrfilter_param(struct nr_filt_prm prm);
static int32_t gyro_get_nrfilter_param(struct nr_filt_prm* prm);
static uint8_t gyro_get_device_id(void);
static int8_t  gyro_set_dynamic_calib(bool enable);
static int8_t  gyro_set_selfchk_presetting(void);
static int8_t  gyro_set_selfchk_cfg(uint8_t type);
static int8_t  gyro_start_man_calib(uint8_t *calprm);
static int8_t  gyro_wait_man_calib(void);
static int32_t press_set_offsets(struct pressure offset);
static int32_t press_get_offsets(struct pressure* offset);
static uint8_t press_get_device_id(void);
static int32_t sgnfcnt_enable(bool enable);
static int32_t set_dev_param(void);
static int32_t set_measurement_mode(const uint8_t mode, const uint8_t sensortype);
static int32_t set_all_measurement_mode(void);
static int32_t set_dynamic_range(const uint8_t val, const uint8_t sensortype);
static int32_t set_all_dynamic_range(void);
static int32_t init_exec(const uint8_t sensortype);
static int8_t sensor_init(const uint8_t sensortype);
static int32_t all_init(void);
static int32_t InitDevice(void);
static int32_t get_micon_info_dt(struct spi_device *client);
static int32_t initialize_snsdrv(void);
static void irq_proc(uint64_t result);

/* ------------------------------------------------
    Each Process
------------------------------------------------ */
static inline int16_t msec_to_hz(int16_t time_ms) {
    int16_t res;
    if(likely(time_ms > 0)){
        res = 1000 / time_ms;
    } else {
        res = 200; //Max Frequency that EOSS3 can be set.
    }
    return res;
};


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

static int16_t get_fastest_sensortask_val(void)
{
    return FASTEST_SENSTASK_PERIOD_MS;
}

static int32_t get_batch_fifo_size(void)
{
    int32_t latest_m4_fifo_size = 0;
#ifdef BATCHING_DYNAMIC_MEMORY
    latest_m4_fifo_size = 8192; //8KB tempolaly
#else
    latest_m4_fifo_size = M4_BATCH_DATA_END_ADDR - M4_BATCH_DATA_START_ADDR;
#endif
    return latest_m4_fifo_size;
}

static int32_t get_fw_version(uint8_t *fw_ver)
{
    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    mutex_unlock(&micon_hcres_mutex);

    fw_ver[0] = 0x01;
    fw_ver[1] = 0x23;
    fw_ver[2] = 0x45;
    fw_ver[3] = 0x67;

    SENSOR_D_LOG("FW Version[%02x][%02x][%02x][%02x]",fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);

    SENSOR_D_LOG("end - SNS_RC_OK");

    return ret;
}

static void get_sns_value(enum sensor_e_type type, void *outdata)
{
    SENSOR_D_LOG("start");
    switch (type){
        case SENSOR_ACC:
            memcpy(outdata, &last_acc_data, sizeof(last_acc_data));
            break;
        case SENSOR_MAG:
            memcpy(outdata, &last_mag_data, sizeof(last_mag_data));
            break;
        case SENSOR_GYRO:
            memcpy(outdata, &last_gyro_data, sizeof(last_gyro_data));
            break;
        case SENSOR_MAG_UNCAL:
            memcpy(outdata, &last_maguc_data, sizeof(last_maguc_data));
            break;
        case SENSOR_GYRO_UNCAL:
            memcpy(outdata, &last_gyrouc_data, sizeof(last_gyrouc_data));
            break;
        case SENSOR_GRV:
            memcpy(outdata, &last_grv_data, sizeof(last_grv_data));
            break;
        case SENSOR_ACC_LNR:
            memcpy(outdata, &last_lacc_data, sizeof(last_lacc_data));
            break;
        case SENSOR_ORTN:
            memcpy(outdata, &last_ori_data, sizeof(last_ori_data));
            break;
        case SENSOR_ROT_VCTR:
            memcpy(outdata, &last_rv_data, sizeof(last_rv_data));
            break;
        case SENSOR_GAME_ROT_VCTR:
            memcpy(outdata, &last_gamrv_data, sizeof(last_gamrv_data));
            break;
        case SENSOR_MAG_ROT_VCTR:
            memcpy(outdata, &last_georv_data, sizeof(last_georv_data));
            break;
        case SENSOR_PRESSURE:
            memcpy(outdata, &last_press_data, sizeof(last_press_data));
            break;
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
}

static void backup_sns_value(void)
{
    SENSOR_D_LOG("start");
    prv_acc.outX = last_acc_data.outX;
    prv_acc.outY = last_acc_data.outY;
    prv_acc.outZ = last_acc_data.outZ;
    prv_gyro.x = last_gyro_data.x;
    prv_gyro.y = last_gyro_data.y;
    prv_gyro.z = last_gyro_data.z;
    SENSOR_D_LOG("end");
}

static void read_all_sensor_value(void)
{
	struct SensorDataType s_data;
	//int64_t prv_ts = latest_ts;
	char buf[256] = {0};
	int32_t Pl, Ph;
	int16_t x,y,z,A;
	int16_t o_x,o_y,o_z;
	uint8_t verify_head, verify_tail;
	int i;
  int8_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");
  ret = dev_read(ANDROID_SENSOR_DATA_ADDR, buf, sizeof(buf));
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("read error[%d].",ret);
		return;
	}
	memcpy(&s_data, buf, 160);
	verify_head = buf[VERIFY_DATA_HEAD_OFS];
	verify_tail = buf[VERIFY_DATA_TAIL_OFS];
	for(i = 0; i < 160; i+=16){
		SENSOR_D_LOG("data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x",
						i, buf[i], i+1, buf[i+1], i+2, buf[i+2], i+3, buf[i+3], i+4, buf[i+4], i+5, buf[i+5], i+6, buf[i+6], i+7, buf[i+7]);
		SENSOR_D_LOG("data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x, data[%02x]=0x%02x",
						i+8, buf[i+8], i+9, buf[i+9], i+10, buf[i+10], i+11, buf[i+11], i+12, buf[i+12], i+13, buf[i+13], i+14, buf[i+14], i+15, buf[i+15]);
	}
    if(verify_sensor_data(verify_head, verify_tail) != VERIFY_OK){
        SENSOR_ERR_LOG("Verify error. EOSS3 recovery. head:[0x%02x] tail:[0x%02x]",
                        verify_head, verify_tail);
        qleoss3_request_hw_reboot("sensor");
        return;
    }
    // SENSOR_D_LOG("timestamp(now)=%lld, timestamp(prv)=%lld, diff= %lld",
                    // now_ts, prv_ts, (now_ts-prv_ts));
    //latest_ts = now_ts;

    backup_sns_value();
    //Accelerometer
    x = s_data.accel_xy;
    y = s_data.accel_xy >> 16;
    z = s_data.accel_z & 0xFFFF;
    last_acc_data.outX = (int32_t)x;
    last_acc_data.outY = (int32_t)y;
    last_acc_data.outZ = (int32_t)z;

    //Magnetometer
    x = s_data.mag_xy;
    y = s_data.mag_xy >> 16;
    z = s_data.mag_zA;
    A = s_data.mag_zA >> 16;
    last_mag_data.x = (int32_t)x;
    last_mag_data.y = (int32_t)y;
    last_mag_data.z = (int32_t)z;
    last_mag_data.accuracy = (uint8_t)(A & 0x00ff);

    //Gyroscope
    x = s_data.gyro_xy;
    y = s_data.gyro_xy >> 16;
    z = s_data.gyro_z;
    last_gyro_data.x = (int32_t)x;
    last_gyro_data.y = (int32_t)y;
    last_gyro_data.z = (int32_t)z;

    //Pressure
    Pl = s_data.press_Pa_L;
    Ph = s_data.press_Pa_H;
    last_press_data.pressure = Pl;

    //Orientation
    x = s_data.orientation_pr;
    y = s_data.orientation_pr >> 16;
    z = s_data.orientation_y;
    A = s_data.mag_zA >> 16;
    last_ori_data.pitch   = (int32_t)x;
    last_ori_data.roll    = (int32_t)y;
    last_ori_data.yaw     = (int32_t)z;
    last_ori_data.accuracy = (uint8_t)(A & 0x00ff);

    //Gravity
    x = s_data.gravity_xy;
    y = s_data.gravity_xy >> 16;
    z = s_data.gravity_z;
    last_grv_data.x = (int32_t)x;
    last_grv_data.y = (int32_t)y;
    last_grv_data.z = (int32_t)z;

    //Linear Acceleration
    x = s_data.linear_accel_xy;
    y = s_data.linear_accel_xy >> 16;
    z = s_data.linear_accel_z;
    last_lacc_data.x = (int32_t)x;
    last_lacc_data.y = (int32_t)y;
    last_lacc_data.z = (int32_t)z;

    //RotationVector
    x = s_data.rotation_vec_q01;
    y = s_data.rotation_vec_q01 >> 16;
    z = s_data.rotation_vec_q2;
    last_rv_data.x = (int32_t)x;
    last_rv_data.y = (int32_t)y;
    last_rv_data.z = (int32_t)z;

    //GeomagRotationVector
    x = s_data.geomag_rotation_vec_q01;
    y = s_data.geomag_rotation_vec_q01 >> 16;
    z = s_data.geomag_rotation_vec_q2A;
    A = s_data.geomag_rotation_vec_q2A >> 16;
    last_georv_data.x = (int32_t)x;
    last_georv_data.y = (int32_t)y;
    last_georv_data.z = (int32_t)z;
    last_georv_data.accuracy = (uint8_t)( (A >> 8) & 0x00ff);

    //GameRotationVector
    x = s_data.game_rotation_vec_q01;
    y = s_data.game_rotation_vec_q01 >> 16;
    z = s_data.game_rotation_vec_q2;
    last_gamrv_data.x = (int32_t)x;
    last_gamrv_data.y = (int32_t)y;
    last_gamrv_data.z = (int32_t)z;

    //MagneticFieldUncalibrated
    x = s_data.mag_xy;
    y = s_data.mag_xy >> 16;
    z = s_data.mag_zA;
    A = s_data.mag_zA >> 16;
    o_x = s_data.mag_calib_offs_xy;
    o_y = s_data.mag_calib_offs_xy >> 16;
    o_z = s_data.mag_calib_offs_z;
    last_maguc_data.x       = (int32_t)x + o_x;
    last_maguc_data.y       = (int32_t)y + o_y;
    last_maguc_data.z       = (int32_t)z + o_z;
    last_maguc_data.cal_x   = (int32_t)o_x;
    last_maguc_data.cal_y   = (int32_t)o_y;
    last_maguc_data.cal_z   = (int32_t)o_z;

    //GyroscopeUncalibrated
    x = s_data.gyro_xy;
    y = s_data.gyro_xy >> 16;
    z = s_data.gyro_z;
    o_x = s_data.gyro_calib_offs_xy;
    o_y = s_data.gyro_calib_offs_xy >> 16;
    o_z = s_data.gyro_calib_offs_z;
    last_gyrouc_data.x       = (int32_t)x + gyro_dynamic_offset[0];
    last_gyrouc_data.y       = (int32_t)y + gyro_dynamic_offset[1];
    last_gyrouc_data.z       = (int32_t)z + gyro_dynamic_offset[2];
    last_gyrouc_data.cal_x   = (int32_t)gyro_dynamic_offset[0];
    last_gyrouc_data.cal_y   = (int32_t)gyro_dynamic_offset[1];
    last_gyrouc_data.cal_z   = (int32_t)gyro_dynamic_offset[2];

    SENSOR_D_LOG("end");
}


static void clr_sns_ofs_val(const uint8_t snstype)
{
    int16_t clr_ofsval[3] = {0};
    uint8_t accuracy = ((snstype == MAGNETOMETER) ? 0 : 3);
    SENSOR_D_LOG("start");
    set_calib_val(clr_ofsval, accuracy, snstype);
    SENSOR_D_LOG("end");
}

static void chk_sns_ofs_val(int16_t *ofsval, const uint8_t snstype)
{
    int16_t l_limit = sns_ofs_llimit[snstype];
    int16_t u_limit = sns_ofs_ulimit[snstype];
    uint8_t i;

    SENSOR_D_LOG("start");
    if( (snstype == MAGNETOMETER)||
        (snstype == GYROSCOPE)||
        (snstype == BAROMETER)){
        SENSOR_D_LOG("end");
        return;
    }

    for(i = 0; i < 3; i++){
        if(((int32_t)ofsval[i] < l_limit) || (u_limit < (int32_t)ofsval[i]) ){
            SENSOR_ERR_LOG("Invalid offset value[%d] type[%d]", ofsval[i], snstype);
            SENSOR_ERR_LOG("clear sensor offset values");
            clr_sns_ofs_val(snstype);
            ofsval[0] = 0;
            ofsval[1] = 0;
            ofsval[2] = 0;
            break;
        }
    }
    SENSOR_D_LOG("end");
}

static uint8_t get_miconshtdwn_status(void)
{
    return qleoss3_get_shutdown_status();
}

static void set_sensordrv_status(enum sensor_drv_status_e_type current_status)
{
    atomic_set(&s_snsdrv_status, (int)current_status);
}
static bool is_sns_connected(const enum ql_sensor_type snstype)
{
    bool result = true;
    SENSOR_D_LOG("start");
    switch(snstype){
        case ACCELEROMETER:
            if(!g_acc_available)  result = false;
            break;
        case MAGNETOMETER:
            if(!g_mag_available)  result = false;
            break;
        case GYROSCOPE:
            if(!g_gyro_available) result = false;
            break;
        case BAROMETER:
            if(!g_pres_available) result = false;
            break;
        default:
            result = false;
            SENSOR_ERR_LOG("Invalid sns type[%d]", snstype);
            break;
    }
    SENSOR_D_LOG("end -result[%d]", result);
    return result;
}

void get_nr_filter_param(int16_t *th_l, int16_t *th_h,
                        int16_t *coeff, const enum ql_sensor_type sns_type)
{
    SENSOR_D_LOG("start");
    switch(sns_type){
        case ACCELEROMETER:
            *th_l  = acc_nrfilt_param.th_l;
            *th_h  = acc_nrfilt_param.th_h;
            *coeff = acc_nrfilt_param.coff;
            break;
        case GYROSCOPE:
            *th_l  = gyro_nrfilt_param.th_l;
            *th_h  = gyro_nrfilt_param.th_h;
            *coeff = gyro_nrfilt_param.coff;
            break;
        default :
            *th_l  = 0;
            *th_h  = 0;
            *coeff = 0;
            SENSOR_ERR_LOG("INVALID SensorType [0x%02x]", sns_type);
            break;
    }
    SENSOR_D_LOG("end");
}

void get_nr_filter_ext_param(int16_t *th_h, int16_t *th_rel,
                        int16_t *b_coff,int16_t *a_coff, int16_t *tol, int16_t *hys, const enum ql_sensor_type sns_type)
{
    SENSOR_D_LOG("start");
    switch(sns_type){
        case ACCELEROMETER:
            *th_h   = acc_nrfilt_param.th_h;
            *th_rel = acc_nrfilt_param.rel_th;
            *b_coff = acc_nrfilt_param.base_filt_coff;
            *a_coff = acc_nrfilt_param.base_filt_acoff;
            *tol    = acc_nrfilt_param.tolerance;
            *hys    = acc_nrfilt_param.base_filt_hys;
            break;
        case GYROSCOPE:
            *th_h   = gyro_nrfilt_param.th_h;
            *th_rel = gyro_nrfilt_param.rel_th;
            *b_coff = gyro_nrfilt_param.base_filt_coff;
            *a_coff = gyro_nrfilt_param.base_filt_acoff;
            *tol    = gyro_nrfilt_param.tolerance;
            *hys    = gyro_nrfilt_param.base_filt_hys;
            break;
        default :
            *th_h   = 0;
            *th_rel = 0;
            *b_coff = 0;
            *a_coff = 0;
            *tol    = 0;
            *hys    = 0;
            SENSOR_ERR_LOG("INVALID SensorType [0x%02x]", sns_type);
            break;
    }
    SENSOR_D_LOG("end");
}

static void acc_clear_filt_coff(void){
	acc_val_clear_flg = true;
}
static void acc_clear_filt_coff_exe(bool flg){
	if (flg || (acc_ts.base_set_val != acc_base_set_val_rcv)) {
	    memset(&prv_acc,0, sizeof(prv_acc));
	    memset(&acc_rel,0, sizeof(acc_rel));
	    memset(&acc_val,0, sizeof(acc_val));
	    memset(&acc_hys_val,0, sizeof(acc_hys_val));
	    memset(&acc_ts,0, sizeof(acc_ts));
	    SENSOR_D_LOG("Accel_filt_coff_clear_done!");
        acc_val_clear_flg = false;
	}
}
static void acc_part_clear_filt_coff(void){
    memset(&acc_rel,0, sizeof(acc_rel));
    memset(&acc_val,0, sizeof(acc_val));
    memset(&acc_hys_val,0, sizeof(acc_hys_val));
    SENSOR_D_LOG("Accel_filt_part_coff_clear!");
}


static void gyro_clear_filt_coff(void){
	gyro_val_clear_flg = true;
}
static void gyro_clear_filt_coff_exe(bool flg){
	if (flg || (gyro_ts.base_set_val != gyro_base_set_val_rcv)) {
	    memset(&gyro_ts,0, sizeof(gyro_ts));
	    SENSOR_D_LOG("Gyro_filt_coff_clear_done!");
        gyro_val_clear_flg = false;
	}
}

static void gyrouc_clear_filt_coff(void){
	gyrouc_val_clear_flg = true;
}
static void gyrouc_clear_filt_coff_exe(bool flg){
	if (flg || (gyrouc_ts.base_set_val != gyrouc_base_set_val_rcv)) {
	    memset(&prv_gyro,0, sizeof(prv_gyro));
	    memset(&gyro_rel,0, sizeof(gyro_rel));
	    memset(&gyro_val,0, sizeof(gyro_val));
	    memset(&gyro_hys_val,0, sizeof(gyro_hys_val));
	    memset(&gyrouc_ts,0, sizeof(gyrouc_ts));
	    SENSOR_D_LOG("Gyrouc_filt_coff_clear!");
        gyrouc_val_clear_flg = false;
	}
}
static void gyrouc_part_clear_filt_coff(void){
    memset(&gyro_rel,0, sizeof(gyro_rel));
    memset(&gyro_val,0, sizeof(gyro_val));
    memset(&gyro_hys_val,0, sizeof(gyro_hys_val));
    SENSOR_D_LOG("Gyro_filt_part_coff_clear!");
}

static int64_t apply_ts_noize_reduction_filter(int64_t cur_val, struct timestamp_filter_val *ts, int64_t base_sv)
{
    int64_t filtered_val = cur_val;
    int64_t th_ts, tol_ts;
    int16_t th_Rel, base_coeff, ts_coeff, ng_th, unaccept_num;
    int64_t TmpDelta = 0;
    int64_t add_coeff = 0;
    int64_t tmp64 = 0;
    //int16_t luck_sample_num = 0;

    th_ts = ts_nrfilt_param.ts_th_us * 1000;
    tol_ts = ts_nrfilt_param.ts_tol_us * 1000;
    th_Rel = ts_nrfilt_param.ts_rel_th;
    base_coeff = ts_nrfilt_param.ts_base_filt_coff;
    ts_coeff = ts_nrfilt_param.ts_filt_coff;
    ng_th = ts_nrfilt_param.ts_ng_th;
    unaccept_num = ts_nrfilt_param.ts_unaccept_num;

    ts->luck_sample_num = 0;

	if (ts->init_done_flg == 0) {
		ts->base_set_val = base_sv;
        ts->base_val = base_sv;
		ts->prv_ts = cur_val - base_sv;
		ts->prv_raw_ts = cur_val - base_sv;
        ts->base_rel = th_Rel;
        ts->ng_cnt = 0;
        SENSOR_D_LOG("ts init prc done.");
		ts->init_done_flg = 1;
	}

    if (ts->base_rel >= th_Rel){
        add_coeff = ts_nrfilt_param.ts_base_filt_acoff;
    } else {
        add_coeff = 0;
    }

    TmpDelta = cur_val - ts->prv_raw_ts;
    if(ts->ng_cnt < ng_th){
        if (((TmpDelta - ts->base_val) <= th_ts) && ((TmpDelta - ts->base_val) >= (-1 * th_ts))){
            if (ts->base_rel < th_Rel) {
                ts->base_rel += 1;
            }
            ts->base_val = ts->base_val + ((TmpDelta - ts->base_val) >> (base_coeff + add_coeff));
            if (ts->base_val > ts->base_set_val + tol_ts) {
                ts->base_val = ts->base_set_val + tol_ts;
            }
            if (ts->base_val < ts->base_set_val - tol_ts) {
                ts->base_val = ts->base_set_val - tol_ts;
            }
            ts->ng_cnt = 0;
        } else {
            ts->ng_cnt = ts->ng_cnt + 1;
        }
    } else {
        ts->base_val = TmpDelta;
        ts->base_rel = 0;
        ts->ng_cnt = 0;
    }

    TmpDelta = cur_val - (ts->prv_ts + ts->base_val);
    if (ts->base_rel >= th_Rel){
        filtered_val -= (TmpDelta - (TmpDelta >> ts_coeff));
        TmpDelta = filtered_val - cur_val;
        if(TmpDelta > ts->base_val) {
        	filtered_val -= (tol_ts >> 1);
        	ts->base_val -= (tol_ts >> 1);
            SENSOR_D_LOG("fast cd:%lld", TmpDelta);
        }else if (TmpDelta  <= (-1 * ts->base_val)) {
            if (TmpDelta  <= (ts->base_val * (-1 * unaccept_num))) {
                ts->luck_sample_num = unaccept_num;
                SENSOR_D_LOG("luck_sample_adjusted[%d]", ts->luck_sample_num);
            } else {
                tmp64 = abs64(TmpDelta);
                do_div(tmp64, ts->base_val);
                ts->luck_sample_num = tmp64;
                SENSOR_D_LOG("luck_sample_available[%d]", ts->luck_sample_num);
            }
            if ((cur_val < (filtered_val + ts->base_val * ts->luck_sample_num)) && (ts->luck_sample_num > 0)){
                ts->luck_sample_num -= 1;
                SENSOR_D_LOG("replay_num decrement[%d]", ts->luck_sample_num);
            }
            SENSOR_D_LOG("slow cd:%lld", TmpDelta);
        }
    }

    TmpDelta = filtered_val;
    if (ts->base_rel >= th_Rel){
        filtered_val -= ts->base_val;
        if(filtered_val < ts->prv_ts){
            filtered_val = ts->prv_ts;
        }
    } else {
        ts->base_rel = th_Rel;
    }
	if (cur_val < filtered_val) {
		filtered_val = cur_val;
	}

    ts->prv_ts = TmpDelta + ts->base_val * ts->luck_sample_num;
    ts->prv_raw_ts = cur_val;
    SENSOR_D_LOG("[end]cur_val[%lld], ts->prv_ts[%lld], ts->base_val[%lld], ts->base_rel[%d], ts->ng_cnt[%d], TmpDelta[%lld], filtered_val[%lld]",
                    cur_val, ts->prv_ts, ts->base_val, ts->base_rel, ts->ng_cnt, TmpDelta, filtered_val);
    return filtered_val;
}

static int16_t get_no_nr_thresh(enum sensor_e_type snstype)
{
    int16_t th = 0;
    SENSOR_D_LOG("start");
    switch (snstype) {
        case SENSOR_ACC:
            th = acc_nrfilt_param.filt_lmt_th;
            break;
        case SENSOR_GYRO_UNCAL:
            th = gyro_nrfilt_param.filt_lmt_th;
            break;
        default:
            SENSOR_ERR_LOG("INVALID Sensor Type[%d]", snstype);
            break;
    }
    SENSOR_D_LOG("end");
    return th;
}

static bool no_noize_reduction_filter(int16_t cmp_x, int16_t cmp_y, int16_t cmp_z,
                                        enum sensor_e_type snstype)
{
    bool no_filt = false;
    int16_t thresh = get_no_nr_thresh(snstype);

    SENSOR_D_LOG("start");
	if(abs(cmp_x) >= thresh){
		no_filt = true;
	}
	else if(abs(cmp_y) >= thresh){
		no_filt = true;
	}
	else if(abs(cmp_z) >= thresh){
		no_filt = true;
	}
    SENSOR_D_LOG("end");

	return no_filt;
}

static int16_t apply_noize_reduction_filter(int16_t cur_val, int16_t prv_val, const enum ql_sensor_type sns_type)
{
    int16_t filtered_val = cur_val;
    int16_t th_L, th_H, coeff;
    int16_t TmpDelta = 0;

    SENSOR_D_LOG("start");
    SENSOR_D_LOG("type[%d], current value[%d], previous value[%d]",
                    sns_type, cur_val, prv_val);

    TmpDelta = cur_val - prv_val;
    get_nr_filter_param(&th_L, &th_H, &coeff, sns_type);

    if( (th_L <= TmpDelta) && (TmpDelta <= th_H) ){
        TmpDelta = TmpDelta >> coeff;
        if(TmpDelta <= th_L){
            TmpDelta = th_L;
        }
        filtered_val = prv_val + TmpDelta;
    }
    else if( ((-1 * th_H) <= TmpDelta) && (TmpDelta <= (-1 * th_L)) ){
        TmpDelta = TmpDelta >> coeff;
        if(TmpDelta >= (-1 * th_L)){
            TmpDelta = (-1 * th_L);
        }
        filtered_val = prv_val + TmpDelta;
    }

    SENSOR_D_LOG("type[0x%02x] FilteredVal[%d]", sns_type, filtered_val);

    SENSOR_D_LOG("end");

    return filtered_val;
}

static int16_t apply_noize_reduction_filter_ext(int16_t cur_val, int16_t prv_val, int32_t *base_val, int32_t *base_hys, int16_t *base_rel, const enum ql_sensor_type sns_type)
{
    int16_t filtered_val = cur_val;
    int16_t th_H;
    int16_t th_Rel, base_coeff, base_a_coeff, tolerance, th_hys;
    int16_t TmpDelta = 0;

    SENSOR_D_LOG("ext_start");

    get_nr_filter_ext_param(&th_H, &th_Rel, &base_coeff, &base_a_coeff, &tolerance, &th_hys, sns_type);
    if(*base_rel >= th_Rel){
        base_coeff = base_a_coeff;
    }

    TmpDelta = cur_val - prv_val;
    if ((TmpDelta <= th_H) && (TmpDelta >= (-1 * th_H))){
        if (*base_rel < th_Rel) {
            *base_rel += 1;
        }
        *base_val = *base_val + ((((int32_t)cur_val << 16) - *base_val) >> base_coeff);
    } else {
        *base_val = (int32_t)cur_val << 16;
        *base_rel = 0;
    }
	if((((*base_hys >> 8) - (*base_val >> 8)) >> 8) >= th_hys){
		*base_hys = *base_val;
	} else if(((*base_val >> 16) - (*base_hys >> 16)) >= th_hys){
		*base_hys = *base_val;
	}

    TmpDelta = cur_val - (*base_hys >> 16);
    if (*base_rel >= th_Rel){
        if (TmpDelta > tolerance){
            filtered_val = (*base_hys >> 16) + tolerance;
        } else if (TmpDelta < (-1 * tolerance)) {
            filtered_val = (*base_hys >> 16) - tolerance;
        }
    }

    SENSOR_D_LOG("ext_end");

    return filtered_val;
}

static void dev_access_check(void)
{
    int state;
    int retly_num = 0;
    SENSOR_D_LOG("start");
    while(1){
        state = (int)ql_spi_get_eoss3_status();
        if((state == DEAD) || (state == NORMAL)){
            break;
        } else {
            if(retly_num >= EOS_CHECK_MAX_RETLY_NUM){
                SENSOR_ERR_LOG("Expired retly num.");
                break;
            }
            SENSOR_ERR_LOG("EOS state is not good condition. retly.");
            msleep(100);
            retly_num++;
        }
    }
    SENSOR_D_LOG("end");
}

static int32_t dev_write(uint32_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    ret = micon_dev_write(adr, (void*)data, size);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t dev_read(uint32_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    ret = micon_dev_read(adr, (void*)data, size);
    SENSOR_D_LOG("end");
    return ret;
}

static void unit_change(struct sensor_ctrl_param_output_str *dst)
{
    SENSOR_D_LOG("Need not.");
}
static void output_sns_param(int32_t *out, bool condition, int8_t sns_bit_point)
{
    if (condition){
        *out |= (1 << sns_bit_point);
    } else {
        *out &= ~(1 << sns_bit_point);
    }
}

static int32_t get_calc_processing_param(uint32_t sns_enable)
{
    int32_t param = 0x00000000;

    SENSOR_D_LOG("start");

    output_sns_param(&param, IS_ROT_VCTR_EN(sns_enable),        ASC_ROTATION_VECT_CALC      );
    output_sns_param(&param, IS_MAG_ROT_VCTR_EN(sns_enable),    ASC_GEO_ROTATION_VECT_CALC  );
    output_sns_param(&param, IS_GAME_ROT_VCTR_EN(sns_enable),   ASC_GAME_ROTATION_VECT_CALC );
    output_sns_param(&param, IS_ORTN_EN(sns_enable),            ASC_ORIENTATION_CALC        );
    output_sns_param(&param, IS_GRV_EN(sns_enable),             ASC_GRAVITY_CALC            );
    output_sns_param(&param, IS_ACC_LNR_EN(sns_enable),         ASC_LINEAR_ACCEL_CALC       );
    output_sns_param(&param, IS_MAG_EN(sns_enable),             ASC_MAGNETOMETER_CALIB      );
    output_sns_param(&param, IS_STEP_CNT_EN(sns_enable),        ASC_STEP_COUNTER_CALC       );
    output_sns_param(&param, IS_STEP_DTC_EN(sns_enable),        ASC_STEP_DETECTOR_CALC      );
    output_sns_param(&param, IS_SIGNFCNT_MTN_EN(sns_enable),    ASC_SIGNIFICANT_MOTION      );

    SENSOR_D_LOG("end");

    return param;
}

static int32_t get_sensor_processing_param(uint32_t sns_enable)
{
    int32_t param = 0x00000000;
    SENSOR_D_LOG("start");

    output_sns_param(&param, IS_ACC_EN(sns_enable),         STC_ACCEL_PROCESSING    );
    output_sns_param(&param, IS_MAG_EN(sns_enable),         STC_MAGNET_PROCESSING   );
    output_sns_param(&param, IS_GYRO_EN(sns_enable),        STC_GYRO_PROCESSING     );
    output_sns_param(&param, IS_PRESSURE_EN(sns_enable),    STC_BARO_PROCESSING     );

    SENSOR_D_LOG("end");
    return param;
}

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int32_t get_calc_batch_processing_param(uint32_t logging_enable)
{
    int32_t param = 0x00000000;

    SENSOR_D_LOG("start");

    output_sns_param(&param, IS_ROT_VCTR_EN(logging_enable),        ASC_ROTATION_VECT_BATCH         );
    output_sns_param(&param, IS_MAG_ROT_VCTR_EN(logging_enable),    ASC_GEO_ROTATION_VECT_BATCH     );
    output_sns_param(&param, IS_GAME_ROT_VCTR_EN(logging_enable),   ASC_GAME_ROTATION_VECT_BATCH    );
    output_sns_param(&param, IS_ORTN_EN(logging_enable),            ASC_ORIENTATION_BATCH           );
    output_sns_param(&param, IS_GRV_EN(logging_enable),             ASC_GRAVITY_BATCH               );
    output_sns_param(&param, IS_ACC_LNR_EN(logging_enable),         ASC_LINEAR_ACCEL_BATCH          );
    output_sns_param(&param, IS_STEP_CNT_EN(logging_enable),        ASC_STEP_COUNTER_BATCH          );
    output_sns_param(&param, IS_STEP_DTC_EN(logging_enable),        ASC_STEP_DETECTOR_BATCH         );

    SENSOR_D_LOG("end");

    return param;
}

static int32_t get_sensor_batch_processing_param(uint32_t logging_enable)
{
    int32_t param = 0x00000000;
    SENSOR_D_LOG("start");
    output_sns_param(&param, IS_ACC_EN(logging_enable),         STC_ACCEL_BATCHING              );
    output_sns_param(&param, IS_MAG_EN(logging_enable),         STC_MAGNETIC_FEILD_BATCHING     );
    output_sns_param(&param, IS_MAG_UNCAL_EN(logging_enable),   STC_MF_UNCALIBRATED_BATCHING    );
    output_sns_param(&param, IS_GYRO_EN(logging_enable),        STC_GYRO_BATCHING               );
    output_sns_param(&param, IS_GYRO_UNCAL_EN(logging_enable),  STC_GYRO_UNCALIBRATED_BATCHING  );
    output_sns_param(&param, IS_PRESSURE_EN(logging_enable),    STC_PRESSURE_BATCHING           );
    SENSOR_D_LOG("end");
    return param;
}
#endif

static uint8_t get_device_id(uint8_t snstype)
{
    uint8_t id;
    uint8_t slv_addr;
    uint8_t devid_addr;
    struct hc_info hc = {0};
	SENSOR_D_LOG("start");
    if( is_sns_connected(snstype) ){
        switch(snstype){
            case ACCELEROMETER:
                slv_addr    = ACC_SLV_ADDR;
                devid_addr  = ACC_DEV_ID_ADDR;
                break;
            case MAGNETOMETER:
                slv_addr    = MAG_SLV_ADDR;
                devid_addr  = MAG_DEV_ID_ADDR;
                break;
            case GYROSCOPE:
                slv_addr    = GYRO_SLV_ADDR;
                devid_addr  = GYRO_DEV_ID_ADDR;
                break;
            case BAROMETER:
                slv_addr    = PRESS_SLV_ADDR;
                devid_addr  = PRESS_DEV_ID_ADDR;
                break;
            default:
                SENSOR_ERR_LOG("Invalid sns type[%d]", snstype);
                return SNS_RC_ERR;
        }

        hc.command = prepare_host_command(SYSTEM,
                SYS_I2C, SYS_EXEC_ARBITARY_READ_PROCESS);
        hc.response_len = 1;
        hc.param_data_len = 4;
        hc.param_data[0] = slv_addr
                         | (0x01 << 8)
                         | (0x01 << 16)
                         | (devid_addr << 24);
        if(qleoss3_hostcmd(&hc) == QL_STATUS_OK){
            id = hc.response[0];
            SENSOR_D_LOG("type[0x%02x], device id[0x%02x]", snstype, id);
        } else {
            id = SNS_RC_ERR;
            SENSOR_ERR_LOG("hostcommand error.");
        }
	} else {
		SENSOR_D_LOG("sensor[0x%02x] is not connected. this process was skipped.", snstype);
        id = SNS_RC_ERR;
	}
	SENSOR_D_LOG("end");
    return id;
}

static int8_t set_FFETimer_period(struct sensor_ctrl_param_output_str *new)
{
    int8_t ret = SNS_RC_OK;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(FFE_TIMER_CONTROL,
        TIMER, FTC_SET_FFE_TIMER_PERIOD);
    hc.param_data_len = 4;
    hc.param_data[0] = new->period_FFETimer;
    hc.response_len = 0;
    SENSOR_D_LOG("SetFFETimerPeriod FFETimerPeriod[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t set_sensor_task_period(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int32_t param;
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    param = new->sensor.period_sensor_task;
    hc.command = prepare_host_command(SENSOR_TASK_CONTROL,
            TASK, STC_SET_SENSOR_TASK_PERIOD);
    hc.param_data_len = 4;
    hc.param_data[0] = param;
    hc.response_len = 0;
    SENSOR_D_LOG("SetSensorTaskPeriod SensorTaskPeriod[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t set_FFEapp_task_period(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(FFE_APP_TASK,
            TASK, FAT_SET_FFE_APP_TASK_PERIOD);
    hc.param_data_len = 4;
    hc.param_data[0] = (int32_t)new->sensor.period_app_task;
    hc.response_len = 0;
    SENSOR_D_LOG("SetFFEAppTaskPeriod[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t set_sensor_proc_period(struct sensor_ctrl_param_output_str *new)
{
    int32_t cmd_param[4] = {0};
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    cmd_param[0]  = (int32_t)new->sensor.period_acc;
    cmd_param[1]  = (int32_t)new->sensor.period_mag;
    cmd_param[2]  = (int32_t)new->sensor.period_gyro;
    cmd_param[3]  = (int32_t)new->sensor.period_pressure;
    hc.command = prepare_host_command(SENSOR_TASK_CONTROL,
            STC_INDIVIDUAL_MEAS, STC_SET_SENSOR_PERIOD);
    hc.param_data_len = 16;
    hc.param_data[0] = cmd_param[0];
    hc.param_data[1] = cmd_param[1];
    hc.param_data[2] = cmd_param[2];
    hc.param_data[3] = cmd_param[3];
    hc.response_len = 0;
    SENSOR_D_LOG("SetSensorPeriod ACC[0x%08x] / MAG[0x%08x] / GYRO[0x%08x] / PRESS[0x%08x]",
                    hc.param_data[0],hc.param_data[1],hc.param_data[2],hc.param_data[3]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t set_M4App_task_period(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(M4_APP_TASK,
        TASK, M4AT_SET_M4_APP_TASK_PERIOD);
    hc.param_data_len = 4;
    hc.param_data[0] = (int32_t)new->sensor.period_app_task;
    hc.response_len = 0;
    SENSOR_D_LOG("SetM4AppTaskPeriod[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t set_sensor_calc_period(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int32_t param = (int32_t)new->sensor.period_fusion;
    int32_t current_magp = (int32_t)new->sensor.period_mag;
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");
    hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
            ASC_INDIVIDUAL_CALC, ASC_SET_INDIVIDUAL_PERIOD);
    hc.param_data_len = 28;
    hc.param_data[0] = param;
    hc.param_data[1] = param;
    hc.param_data[2] = param;
    hc.param_data[3] = param;
    hc.param_data[4] = param;
    hc.param_data[5] = param;
    hc.param_data[6] = min(current_magp, LOWEST_MAGCAL_PERIOD_MS);
    hc.response_len = 0;
    SENSOR_D_LOG("SetSensorCalc(fusion)Period[All fusion period is same] period[0x%08x]", param);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t set_FFEfusion_task_period(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
        TASK, ASC_SET_FFE_FUSION_PERIOD);
    hc.param_data_len = 4;
    hc.param_data[0] = (int32_t)new->sensor.period_fusion_task;
    hc.response_len = 0;
    SENSOR_D_LOG("SetFusionTaskPeriod Period(FFE FusionTask)[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t set_M4fusion_task_period(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
        TASK, ASC_SET_M4_FUSION_TASK_PERIOD);
    hc.param_data_len = 4;
    hc.param_data[0] = (int32_t)new->sensor.period_fusion_task;
    hc.response_len = 0;
    SENSOR_D_LOG("SetFusionTaskPeriod Period(M4 FusionTask)[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}
#endif

static void set_odr(uint16_t period_ms, const enum ql_sensor_type sensortype)
{
    struct hc_info hc = {0};
    int32_t odr = (int32_t)msec_to_hz(period_ms);
    SENSOR_D_LOG("start snstype[0x%02x]", sensortype);

    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
    }
    if( is_sns_connected(sensortype) ){
        hc.command = prepare_host_command(SENSOR_DEVICE_CONTROL,
                sensortype, SDC_SET_ODR);
        hc.param_data[0] = odr;
        hc.param_data_len = 2;
        hc.response_len = 0;
        SENSOR_D_LOG("sens[0x%02x], odr[0x%08x]", sensortype, hc.param_data[0]);
        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }
    SENSOR_D_LOG("end");
}

static void set_sensors_ODR(struct sensor_ctrl_param_output_str *new)
{
    uint16_t period;
    SENSOR_D_LOG("start");

    //Set ACC ODR
    period = ((new->sensor.period_acc == PERIOD_ACC_MIN) ? 0 : new->sensor.period_acc);
    set_odr(period, ACCELEROMETER);
    //Set MAG ODR
    period = new->sensor.period_mag;
    set_odr(period, MAGNETOMETER);
    //Set GYRO ODR
    period = ((new->sensor.period_acc == PERIOD_GYRO_MIN) ? 0 : new->sensor.period_acc);
    //period = new->sensor.period_gyro;
    set_odr(period, GYROSCOPE);
    //Set BARO ODR
    period = new->sensor.period_pressure;
    set_odr(period, BAROMETER);

    SENSOR_D_LOG("end");
}

static int8_t enable_sensor_processing(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int32_t param;
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    SENSOR_D_LOG("Start now sensorEnableState[0x%08x]", new->sensor.enable);
    param = get_sensor_processing_param(new->sensor.enable);
    hc.command = prepare_host_command(SENSOR_TASK_CONTROL,
            STC_INDIVIDUAL_MEAS, STC_ENABLE_SENSOR_PROCESSING);
    hc.param_data_len = 4;
    hc.param_data[0] = param;
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnable SensorProcessing[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t enable_calc_processing(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int32_t param;
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    memset(&hc,0, sizeof(hc));
    SENSOR_D_LOG("Start now sensorEnableState[0x%08x]", new->sensor.enable);
    param = get_calc_processing_param(new->sensor.enable);
    hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
            ASC_INDIVIDUAL_CALC, ASC_ENABLE_INDIVIDUAL_CAL);
    hc.param_data_len = 4;
    hc.param_data[0] = param;
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnable CalcProcessing[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t enable_M4Fusion_Task(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    memset(&hc,0, sizeof(hc));
    hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
            TASK, ASC_ENABLE_M4_FUSION_TASK);
    hc.param_data_len = 1;
    hc.param_data[0] = (int32_t)new->sensor.task_exec[2];
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnableM4FusionTask TaskEnable[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}
#endif

static int8_t enable_sensor_task(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(SENSOR_TASK_CONTROL,
            TASK, STC_ENABLE_SENSOR_TASK);
    hc.param_data_len = 1;
    hc.param_data[0] = (int32_t)new->sensor.task_exec[0];
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnableSensorTask SensorTaskEnable[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t enable_FFEFusion_task(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
            TASK, ASC_ENABLE_FFE_FUSION_TASK);
    hc.param_data_len = 1;
    hc.param_data[0] = (int32_t)new->sensor.task_exec[2];
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnableFFEFusionTask TaskEnable[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}
#endif

static int8_t enable_FFEapp_task(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(FFE_APP_TASK,
            TASK, FAT_ENABLE_FFE_APPLICATION);
    hc.param_data_len = 1;
    hc.param_data[0] = (int32_t)new->sensor.task_exec[1];
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnableFFEapptask(FFE App Task) Enable[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t enable_FFETimer(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t param;
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    param = (!!new->sensor.enable) ? 1 : 0;
    hc.command = prepare_host_command(FFE_TIMER_CONTROL,
            TIMER, FTC_ENABLE_FFE_TIMER);
    hc.param_data_len = 1;
    hc.param_data[0] = (int32_t)param;
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnableFFETimer Enable[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t enable_M4app_task(struct sensor_ctrl_param_output_str *new)
{
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    hc.command = prepare_host_command(M4_APP_TASK,
            TASK, M4AT_ENABLE_M4_APP_TASK);
    hc.param_data_len = 1;
    hc.param_data[0] = (int32_t)new->sensor.task_exec[1];
    hc.response_len = 0;
    SENSOR_D_LOG("SetEnableFFEAppTask Enable[0x%08x]", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t send_sensor_ctrl(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new,
    bool force)
{
    int32_t ret;
    uint8_t m_mode;

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_D_LOG("start");

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    //Set FFE Timer period
    ret = set_FFETimer_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_FFETimer_period");
        goto exit;
    }

    //Set sensor task period
    ret = set_sensor_task_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_sensor_task_period");
        goto exit;
    }

    //Set sensor processing period
    ret = set_sensor_proc_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_sensor_proc_period(don't skip)[%d]", ret);
    }

    //Set FFE app task period
    ret = set_FFEapp_task_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_FFEapp_task_period");
        goto exit;
    }

    //Set M4 App task period
    ret = set_M4App_task_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_M4App_task_period");
        goto exit;
    }

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
    //Set android sensor calculation period
    ret = set_sensor_calc_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_sensor_calc_period(don't skip)[%d]", ret);
    }

    //[FFE Fusion Task] Set Fusion Task period
    ret = set_FFEfusion_task_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_FFEfusion_task_period");
        goto exit;
    }

    //[M4 Fusion Task] Set Fusion Task period
    ret = set_M4fusion_task_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_M4fusion_task_period");
        goto exit;
    }
#endif

    //Set ODR[AS/MS/GS/PS]
    set_sensors_ODR(new);

    //Acc-Set Meas Mode
    m_mode = new->sensor.acc_param[4];
    SENSOR_D_LOG("acc measurement mode[0x%02x]", m_mode);
    set_measurement_mode(m_mode, ACCELEROMETER);

    //Enable/Disable Sensor Processing
    ret = enable_sensor_processing(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_sensor_processing(don't skip)[%d]", ret);
    }

    //Enable/Disable android sensor calculation[Fusion]
    ret = enable_calc_processing(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_calc_processing(don't skip)[%d]", ret);
    }

    //Enable/Disable sensor task
    ret = enable_sensor_task(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_sensor_task");
        goto exit;
    }

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
    //[FFE Fusion Task-Enable/Disable Fusion task]
    ret = enable_FFEFusion_task(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_FFEFusion_task");
        goto exit;
    }
#endif

    //[FFE App TaskEnable/Disable FFE app task]
    ret = enable_FFEapp_task(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_FFEapp_task");
        goto exit;
    }

    //Enable/Disable FFE timer
    ret = enable_FFETimer(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_FFETimer");
        goto exit;
    }

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
    //Enable/Disable M4 fusion task
    ret = enable_M4Fusion_Task(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_M4Fusion_Task");
        goto exit;
    }
#endif

    //Enable/Disable M4 app task
    ret = enable_M4app_task(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_M4app_task");
    }

exit:

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static int8_t enable_batch_full_interrupt(uint8_t enable)
{
	struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");

	hc.command = prepare_host_command(BATCH_MODE_CONTROL,
		BATCH, BMC_ENABLE_BUF_FULL_INTERRUPT);
	hc.param_data_len = 1;
	hc.param_data[0] = (int32_t)enable;
	hc.response_len = 0;
	SENSOR_D_LOG("SetEnableBufferFullInterrupt Enable[0x%08x]", hc.param_data[0]);
	ret = qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

static int8_t enable_batch_interrupt(uint8_t enable, int16_t margin_buf_size)
{
    int32_t param = 0x00000000;
    struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    param = (int32_t)(enable | (( (int32_t)margin_buf_size & 0x0000ffff )<< 8));
    hc.command = prepare_host_command(BATCH_MODE_CONTROL,
		BATCH, BMC_ENABLE_BATCH_INTERRUPT);
	hc.param_data_len = 3;
    hc.param_data[0] = param;
	hc.response_len = 0;
    SENSOR_D_LOG("SetEnableBufferInterrupt Enable_Margin[0x%08x]", hc.param_data[0]);
	ret = qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

static int8_t set_sensor_batch_period(struct sensor_ctrl_param_output_str *new)
{
	uint8_t i = 0;
	int32_t param[7] = {0x00000000};
	struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");
	param[0] = new->logging.period_acc;
	param[1] = new->logging.period_mag;
	param[2] = new->logging.period_mag;
	param[3] = new->logging.period_gyro;
	param[4] = new->logging.period_gyro;
	param[5] = new->logging.period_pressure;
	param[6] = new->logging.period_acc;
	hc.command = prepare_host_command(SENSOR_TASK_CONTROL,
	STC_INDIVIDUAL_BATCH, STC_SET_SENSOR_BATCH_PERIOD);
	hc.param_data_len = 28;
	for(i = 0; i < ARRAY_SIZE(param); i++){
	    hc.param_data[i] = param[i];
	}
	hc.response_len = 0;
	SENSOR_D_LOG("Batch SetSensorPeriod ACC[0x%08x] / MAG[0x%08x] / MAG Uncal[0x%08x]",
					hc.param_data[0],hc.param_data[1],hc.param_data[2]);
	SENSOR_D_LOG("Batch SetSensorPeriod Gyro[0x%08x] / Gyro Uncal[0x%08x] / Press[0x%08x]",
					hc.param_data[3],hc.param_data[4],hc.param_data[5]);
	ret = qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

static int8_t set_fusion_batch_period(struct sensor_ctrl_param_output_str *new)
{
	uint8_t i = 0;
	int32_t param[6] = {0x00000000};
	struct hc_info hc= {0};
    int8_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");
	param[0] = new->logging.period_R;
	param[1] = new->logging.period_GeR;
	param[2] = new->logging.period_GaR;
	param[3] = new->logging.period_O;
	param[4] = new->logging.period_Grv;
	param[5] = new->logging.period_LA;
	hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
	ASC_INDIVIDUAL_BATCH, ASC_SET_BATCH_PERIOD);
	hc.param_data_len = 24;
	for(i = 0; i < 6; i++){
		hc.param_data[i] = param[i];
	}
	hc.response_len = 0;
	SENSOR_D_LOG("Batch SetFusionPeriod RotVec[0x%08x] / MAGRotVec[0x%08x] / GAMERotVec[0x%08x]",
					hc.param_data[0],hc.param_data[1],hc.param_data[2]);
	SENSOR_D_LOG("Batch SetFusionPeriod Ori[0x%08x] / Gravity[0x%08x] / LACC[0x%08x]",
					hc.param_data[3],hc.param_data[4],hc.param_data[5]);
	ret = qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

static int8_t set_enable_steplike_sensor(struct sensor_ctrl_param_output_str *new,
                                        int8_t sc_1st_en)
{
	struct hc_info hc = {0};
    uint8_t sc_enable = new->logging.batch_step_cnt_enable;
    uint8_t sd_enable = new->logging.batch_step_dtc_enable;
    int8_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");
	hc.command = prepare_host_command(APP, APP_AR, APP_AR__AR_StepOnOff);
	hc.param_data[0] = (int32_t)((0x00000001)|
	                            (((int32_t)sc_enable << 8 ) & 0x0000FF00)|
                                (((int32_t)sd_enable << 16) & 0x00FF0000));
	hc.param_data[1] = (int32_t)sc_1st_en;
    hc.param_data_len = 5;
    hc.response_len = 0;
	SENSOR_D_LOG("Enable StepLike Sensor param1[0x%08x],param2[0x%08x]",
					hc.param_data[0],hc.param_data[1]);
	ret = qleoss3_hostcmd(&hc);
	SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

static int8_t set_batch_maxlatency(struct sensor_ctrl_param_output_str *new)
{
	uint8_t i = 0;
	int32_t param[2] = {0};
	struct hc_info hc = {0};
    int8_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");

	param[0] = new->logging.batch_timeout_set;
	param[1] = 0x00000000;
	hc.command = prepare_host_command(BATCH_MODE_CONTROL,
			BATCH, BMC_SET_MAX_LATENCY);
	hc.param_data_len = 8;
	for(i = 0; i < 2; i++){
		hc.param_data[i] = param[i];
	}
	hc.response_len = 0;
	SENSOR_D_LOG("SetBatchMaxReportLatency MaxReportLatency[0x%08x%08x]", hc.param_data[1], hc.param_data[0]);
	ret = qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end[%d]", ret);
    return ret;
}

static int8_t set_enable_batch_sensor_processing(struct sensor_ctrl_param_output_str *new)
{
	int32_t param = 0;
    int ret = SNS_RC_OK;
	struct hc_info hc = {0};

	SENSOR_D_LOG("start");
	param = get_sensor_batch_processing_param(new->logging.enable);
	hc.command = prepare_host_command(SENSOR_TASK_CONTROL,
			STC_INDIVIDUAL_BATCH, STC_ENABLE_SENSOR_BATCHING);
	hc.param_data_len = 4;
	hc.param_data[0] = param;
	hc.response_len = 0;
	SENSOR_D_LOG("SetEnable SensorBatchProcessing[0x%08x]", hc.param_data[0]);
	ret = qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end ret[%d]", ret);
    return ret;
}

static int8_t set_enable_batch_fusion_processing(struct sensor_ctrl_param_output_str *new)
{
    int8_t ret = SNS_RC_OK;
	int32_t param = 0;
	struct hc_info hc = {0};

	SENSOR_D_LOG("start");
	param = get_calc_batch_processing_param(new->logging.enable);
	hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
			ASC_INDIVIDUAL_BATCH, ASC_ENABLE_INDIVIDUAL_BATCH);
	hc.param_data_len = 4;
	hc.param_data[0] = param;
	hc.response_len = 0;
	SENSOR_D_LOG("SetEnable FusionBatchProcessing[0x%08x]", hc.param_data[0]);
	ret = qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end ret[%d]", ret);
    return ret;
}
#endif

static int32_t send_logging_state(
    struct sensor_ctrl_param_output_str *old,
    struct sensor_ctrl_param_output_str *new)
{
    int32_t ret = SNS_RC_OK;
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
    int batch_onoff = !!new->logging.batch_enable;
    int sc1st_en = new->logging.batch_step_cnt_enable && !old->logging.batch_step_cnt_enable;
    SENSOR_D_LOG("start");

    //Setting every sensor's cal interrupt
    //AccCal interrupt setting
    ret = set_enable_cal_interrupt(!batch_onoff, ACCELEROMETER);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_enable_cal_interrupt[ACC] (don't skip)");
    }
    //MagCal interrupt setting
    ret = set_enable_cal_interrupt(!batch_onoff, MAGNETOMETER);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_enable_cal_interrupt[MAG] (don't skip)");
    }
    //GyroCal interrupt setting
    ret = set_enable_cal_interrupt(!batch_onoff, GYROSCOPE);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_enable_cal_interrupt[GYRO] (don't skip)");
    }
    //Setting interrupt about batch
    //Full interrupt setting
	ret = enable_batch_full_interrupt(batch_onoff);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_batch_full_interrupt");
        goto exit;
    }
    //Margin interrupt setting
	ret = enable_batch_interrupt(batch_onoff, BATCH_MARGIN_SIZE);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:enable_batch_interrupt");
        goto exit;
    }

    //Setting batch period(sampling frequency for storing data to batch-buffer)
	ret = set_sensor_batch_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_sensor_batch_period (don't skip)");
    }

	ret = set_fusion_batch_period(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_fusion_batch_period (don't skip)");
    }

    //Setting maxlatency
	ret = set_batch_maxlatency(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_batch_maxlatency");
        goto exit;
    }

    //Setting each sensors batch processing
	//[A,M,MU,G,GU,P]Enable sensor batch
	ret = set_enable_batch_sensor_processing(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_enable_batch_sensor_processing (don't skip)");
    }
	//[R,GeR,GaR,O,Grv,LA]Enable android sensor batch
	ret = set_enable_batch_fusion_processing(new);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_enable_batch_fusion_processing (don't skip)");
    }

    //Enable Android KC Sensor
    ret = set_enable_steplike_sensor(new, sc1st_en);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error:set_enable_steplike_sensor");
        goto exit;
    }

    batch_mark_off(LOGGING_TRIGGER_CHANGE_SENSOR_STATE, (bool)batch_onoff);
    if(!batch_onoff){
        //Clear T[n] value for batch.
        atomic_set(&is_1stBatch_proc, true);
    }

exit:
#endif
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static void set_flush(enum sensor_e_type type)
{
	SENSOR_D_LOG("start");
    qleoss3_set_flush(type);
	SENSOR_D_LOG("end");
}

static void enable_irq_wake_irq(bool enable)
{
	SENSOR_D_LOG("start");
	qlspi_ope_wake_irq(enable);
	SENSOR_D_LOG("end");
}

#define MSEC2NSEC_UNIT (1000000)
static int64_t culc_ts_Cal(int64_t numerator, int64_t denominator)
{
    int64_t result;
    int64_t tmp;
    SENSOR_D_LOG("start");
    if(atomic_read(&is_1stBatch_proc)){
        atomic_set(&is_1stBatch_proc, false);
        result = MSEC2NSEC_UNIT;
    } else {
        tmp = numerator;
        do_div(tmp, denominator);
        result = tmp;
    }
    SENSOR_D_LOG("end");
    return result;
}

static int64_t calc_elapsed_time(uint32_t start_time, uint32_t current_time)
{
    int64_t elapsed_time;
    SENSOR_D_LOG("start");
    if (start_time <= current_time) {
        elapsed_time = (int64_t)(current_time - start_time);
    } else {
        elapsed_time = (int64_t)((U32_MAX - start_time) + current_time) + 1;
        SENSOR_D_LOG("Overflowed. start_time[%u] current_time[%u]", start_time, current_time);
    }
    SENSOR_D_LOG("end");
    return elapsed_time;
}

static int64_t convert_host_sensortimestamp(uint32_t sensor_ts, enum batch_ts_type type)
{
    int64_t ts_cal = 0;
    int64_t numerator;
    int64_t denominator;
    int64_t timestamp = 0;

    SENSOR_D_LOG("start batch ts type[%d]", type);
    if( (type != SENSOR_TASK) && (type != APP_TASK) ){
        SENSOR_ERR_LOG("Invalid batch_ts_type[%d]", type);
        return -1;
    }
    SENSOR_D_LOG("current_s3_ts[%u] previous_s3_ts[%u] current_ts[%lld] prev_ts[%lld] sensor_ts[%u] ",
                 current_s3_ts[type], previous_s3_ts[type], current_batch_kts, previous_batch_kts, sensor_ts);
    numerator = current_batch_kts - previous_batch_kts;
    denominator = (int64_t) calc_elapsed_time(previous_s3_ts[type], current_s3_ts[type]);
    ts_cal = culc_ts_Cal(numerator, denominator);
    timestamp = current_batch_kts - ((int64_t) calc_elapsed_time(sensor_ts, current_s3_ts[type]) * ts_cal);
    SENSOR_D_LOG("ts_Cal[%lld], timestamp[%lld]", ts_cal, timestamp);
    SENSOR_D_LOG("end");

    return timestamp;
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
    case SENSOR_BATCH_ID_MAG_UNCAL_RAW:
    case SENSOR_BATCH_ID_MAG_UNCAL_OFS:
#endif
#ifdef CONFIG_INPUT_SENSOR_UNCAL_GYROSCOPE
    case SENSOR_BATCH_ID_GYRO_UNCAL_RAW:
    case SENSOR_BATCH_ID_GYRO_UNCAL_OFS:
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
#ifdef CONFIG_INPUT_SENSOR_STEP_DETECTOR
    case SENSOR_BATCH_ID_STEPDETECTOR:
#endif
#ifdef CONFIG_INPUT_SENSOR_STEP_COUNTER
    case SENSOR_BATCH_ID_STEPCOUNTER:
#endif
    case SENSOR_BATCH_ID_SENSORTASK_TSOF:
    case SENSOR_BATCH_ID_APPTASK_TSOF:
        ret = true;
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end ret[%d]", ret);
    return ret;
}

static int32_t iio_report_events(uint8_t *batch_data, uint32_t len)
{
    uint8_t  *buf = batch_data;
    int32_t  remain = len;
    uint8_t  data[EVENT_DATA_SIZE] = {0};
    uint8_t  tsof_cnt[BATCH_TS_TYPE_MAX] = {0};
    uint8_t  batch_id;
    uint8_t  tbl_idx;
    int64_t  timestamp = 0;
    int32_t  smid;
    int32_t  size;
    uint8_t  nousesize = 0;
    uint8_t  gyro_uncal_raw[6] = {0}; /* X,Y,Z */
    uint8_t  mag_uncal_raw[7] = {0};  /* X,Y,Z,Accuracy */
    uint32_t step_count = 0;
    int32_t  event_cnt = 0;
    union timestamp sensor_ts;
    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start remain[%d]", remain);

    while (remain > 0)
    {
        batch_id = *buf;
        if (!is_batch_id_valid(batch_id)) {
            SENSOR_ERR_LOG("invalid batch_id [%d] remain[%d]", batch_id, remain);
            ret = SNS_RC_ERR_INVALID_ID;
            break;
        }

        tbl_idx = batch_id_to_tbl_idx[batch_id];
        smid = tbl_idx_to_smid[tbl_idx];
        size = tbl_idx_to_size[tbl_idx];
        nousesize = tbl_idx_to_nousesize[tbl_idx];
        memcpy(sensor_ts.data, buf+8, sizeof(sensor_ts.data));
        SENSOR_D_LOG("remain[%d] ev[%d] idx[%d] smid[%d] size[%d]", remain, event_cnt, tbl_idx, smid, size);
        SENSOR_D_LOG("data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                     *buf, *(buf+1), *(buf+2), *(buf+3), *(buf+4), *(buf+5), *(buf+6), *(buf+7), *(buf+8), *(buf+9), *(buf+10), *(buf+11));

        switch (tbl_idx) {
        case SENSOR_BATCH_TBL_ACC:
            data[0] = *(buf+nousesize+1); //X_L
            data[1] = *(buf+nousesize+2); //X_H
            data[2] = *(buf+nousesize+3); //Y_L
            data[3] = *(buf+nousesize+4); //Y_H
            data[4] = *(buf+nousesize+5); //Z_L
            data[5] = *(buf+nousesize+6); //Z_H
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_MAG:
            data[0] = *(buf+nousesize+2); //X_L
            data[1] = *(buf+nousesize+3); //X_H
            data[2] = *(buf+nousesize+4); //Y_L
            data[3] = *(buf+nousesize+5); //Y_H
            data[4] = *(buf+nousesize+6); //Z_L
            data[5] = *(buf+nousesize+7); //Z_H
            data[6] = *(buf+nousesize+1); //Accuracy
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_GYRO:
            data[0] = *(buf+nousesize+1); //X_L
            data[1] = *(buf+nousesize+2); //X_H
            data[2] = *(buf+nousesize+3); //Y_L
            data[3] = *(buf+nousesize+4); //Y_H
            data[4] = *(buf+nousesize+5); //Z_L
            data[5] = *(buf+nousesize+6); //Z_H
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_MAG_UNCAL_RAW:
            mag_uncal_raw[0] = *(buf+nousesize+2); //X_L
            mag_uncal_raw[1] = *(buf+nousesize+3); //X_H
            mag_uncal_raw[2] = *(buf+nousesize+4); //Y_L
            mag_uncal_raw[3] = *(buf+nousesize+5); //Y_H
            mag_uncal_raw[4] = *(buf+nousesize+6); //Z_L
            mag_uncal_raw[5] = *(buf+nousesize+7); //Z_H
            mag_uncal_raw[6] = *(buf+nousesize+1); //Accuracy
            break;

        case SENSOR_BATCH_TBL_MAG_UNCAL_OFS:
            memcpy(data, mag_uncal_raw, sizeof(mag_uncal_raw));
            data[7]  = *(buf+nousesize+1); //X_L
            data[8]  = *(buf+nousesize+2); //X_H
            data[9]  = *(buf+nousesize+3); //Y_L
            data[10] = *(buf+nousesize+4); //Y_H
            data[11] = *(buf+nousesize+5); //Z_L
            data[12] = *(buf+nousesize+6); //Z_H
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, (size+sizeof(mag_uncal_raw)), timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_GYRO_UNCAL_RAW:
            gyro_uncal_raw[0] = *(buf+nousesize+1); //X_L
            gyro_uncal_raw[1] = *(buf+nousesize+2); //X_H
            gyro_uncal_raw[2] = *(buf+nousesize+3); //Y_L
            gyro_uncal_raw[3] = *(buf+nousesize+4); //Y_H
            gyro_uncal_raw[4] = *(buf+nousesize+5); //Z_L
            gyro_uncal_raw[5] = *(buf+nousesize+6); //Z_H
            break;

        case SENSOR_BATCH_TBL_GYRO_UNCAL_OFS:
            memcpy(data, gyro_uncal_raw, sizeof(gyro_uncal_raw));
            data[6]  = *(buf+nousesize+1); //X_L
            data[7]  = *(buf+nousesize+2); //X_H
            data[8]  = *(buf+nousesize+3); //Y_L
            data[9]  = *(buf+nousesize+4); //Y_H
            data[10] = *(buf+nousesize+5); //Z_L
            data[11] = *(buf+nousesize+6); //Z_H
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, (size+sizeof(gyro_uncal_raw)), timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_PRESSURE:
            data[0] = *(buf+nousesize+1); //X_L
            data[1] = *(buf+nousesize+2); //X_H
            data[2] = *(buf+nousesize+3); //Y_L
            data[3] = *(buf+nousesize+4); //Y_H
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_ORTN:
            data[0] = *(buf+nousesize+1); //yaw
            data[1] = *(buf+nousesize+2); //yaw
            data[2] = *(buf+nousesize+3); //roll
            data[3] = *(buf+nousesize+4); //roll
            data[4] = *(buf+nousesize+5); //pitch
            data[5] = *(buf+nousesize+6); //pitch
            data[6] = 3;                  //accuracy
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_GRV:
        case SENSOR_BATCH_TBL_ACC_LNR:
        case SENSOR_BATCH_TBL_ROT_VCTR:
        case SENSOR_BATCH_TBL_GAME_ROT_VCTR:
            data[0] = *(buf+nousesize+1); //X_L
            data[1] = *(buf+nousesize+2); //X_H
            data[2] = *(buf+nousesize+3); //Y_L
            data[3] = *(buf+nousesize+4); //Y_H
            data[4] = *(buf+nousesize+5); //Z_L
            data[5] = *(buf+nousesize+6); //Z_H
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_MAG_ROT_VCTR:
            data[0] = *(buf+nousesize+2); //X_L
            data[1] = *(buf+nousesize+3); //X_H
            data[2] = *(buf+nousesize+4); //Y_L
            data[3] = *(buf+nousesize+5); //Y_H
            data[4] = *(buf+nousesize+6); //Z_L
            data[5] = *(buf+nousesize+7); //Z_H
            data[6] = *(buf+nousesize+1); //Accuracy
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, SENSOR_TASK);
            sns_iio_report_event(smid, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_STEPCOUNTER:
            data[0] = *(buf+nousesize+1); //StepCount1
            data[1] = *(buf+nousesize+2); //StepCount2
            data[2] = *(buf+nousesize+3); //StepCount3
            data[3] = *(buf+nousesize+4); //StepCount4
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, APP_TASK);
            step_count = *(int32_t *)&data[0];
            SENSOR_D_LOG("step:%d timestamp:%lld", step_count, timestamp);
            sns_iio_report_event(SHID_STEP_COUNTER, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_STEPDETECTOR:
            data[0] = 0x01; //StepDetect1
            data[1] = 0x00; //StepDetect2
            data[2] = 0x00; //StepDetect3
            data[3] = 0x00; //StepDetect4
            timestamp = convert_host_sensortimestamp(sensor_ts.ts, APP_TASK);
            SENSOR_D_LOG("timestamp:%lld", timestamp);
            sns_iio_report_event(SHID_STEP_DETECTOR, data, size, timestamp, 0);
            break;

        case SENSOR_BATCH_TBL_SENSORTASK_TSOF:
            SENSOR_D_LOG("Sensor's Timestamp overflow");
            ++tsof_cnt[SENSOR_TASK];
            break;

        case SENSOR_BATCH_TBL_APPTASK_TSOF:
            SENSOR_D_LOG("Application's Timestamp overflow");
            ++tsof_cnt[APP_TASK];
            break;

        default:
            SENSOR_ERR_LOG("invalid batch_id [%d] remain[%d]", batch_id, remain);
            remain = 0;
            break;
        }
        remain -= BATCH_DATA_SIZE_PER_ID;
        buf += BATCH_DATA_SIZE_PER_ID;
        event_cnt++;
        memset(data, 0x00, sizeof(data));
    }
    SENSOR_D_LOG("end");
    return ret;
}

static void all_clr_batching_data(void)
{
    struct hc_info hc = {0};
    SENSOR_D_LOG("start");
    hc.command = prepare_host_command(BATCH_MODE_CONTROL,
            BATCH, BMC_ALL_CLEAR_BATCHING_DATA);
    hc.param_data_len = 4;
    hc.param_data[0] = 1;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);
    SENSOR_D_LOG("end");
}

static bool check_can_mark_off(const uint32_t now_addr, const uint32_t l_limit, const uint32_t u_limit)
{
    bool result = true;
    if(now_addr < l_limit){
        SENSOR_ERR_LOG("less batch buffer now_addr[0x%08x] lower_limit[0x%08x]",
                        now_addr, l_limit);
        result = false;
    }
    else if(u_limit < now_addr){
        SENSOR_ERR_LOG("over batch buffer now_addr[0x%08x] lower_limit[0x%08x]",
                        now_addr, u_limit);
        result = false;
    }
    return result;
}

static bool is_buf_circulated(struct get_batchbuf_info info, uint32_t r_size)
{
    return ((info.s_ptr+r_size) >= info.u_limit_addr ? true:false);
}

static int8_t read_batchdata(uint32_t s_ptr, int16_t size)
{
    int8_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start ptr[0x%08x] size[%d]", s_ptr, size);
    ret = dev_read(s_ptr, &mark_off_buf[mark_off_buf_idx], size);
    if(ret != SNS_RC_OK){
        SENSOR_D_LOG("end");
        return ret;
    }
    mark_off_buf_idx += size;
    SENSOR_D_LOG("end");
    return ret;
}

static int8_t read_batchdata_circulated(struct get_batchbuf_info info, uint32_t size, uint32_t *next_ptr)
{
    int8_t ret = SNS_RC_OK;
    uint32_t r_size;
    uint32_t new_s_ptr = info.s_ptr;

    SENSOR_D_LOG("start");
    r_size = info.u_limit_addr - info.s_ptr;
    ret = read_batchdata(new_s_ptr, r_size);
    if(ret != SNS_RC_OK){
        SENSOR_D_LOG("end");
        return ret;
    }
    new_s_ptr = info.l_limit_addr;
    r_size = size - r_size;
    if(r_size){
        ret = read_batchdata(new_s_ptr, r_size);
        if(ret != SNS_RC_OK){
            SENSOR_D_LOG("end");
            return ret;
        }
        new_s_ptr += r_size;
    }
    *next_ptr = new_s_ptr;

    SENSOR_D_LOG("end");
    return ret;
}

static int8_t get_batch_buffer_data(struct get_batchbuf_info bd_info,
                                    int16_t *next_remain, uint32_t *next_ptr)
{
    int8_t ret = SNS_RC_OK;
    uint16_t r_size;
    uint32_t new_s_ptr = 0x00000000;

    SENSOR_D_LOG("start");
    if(bd_info.remain_size < BATCH_MAX_READ_SIZE){
        r_size = bd_info.remain_size;
        if(is_buf_circulated(bd_info, r_size)){
            SENSOR_D_LOG("Ring buffer Circulation");
            ret = read_batchdata_circulated(bd_info, r_size, &new_s_ptr);
            if(ret != SNS_RC_OK){
                SENSOR_D_LOG("end");
                return ret;
            }
        } else {
            ret = read_batchdata(bd_info.s_ptr, r_size);
            if(ret != SNS_RC_OK){
                SENSOR_D_LOG("end");
                return ret;
            }
            new_s_ptr = bd_info.s_ptr + r_size;
        }
        *next_ptr = new_s_ptr;
        *next_remain = 0;
    } else {
        r_size = BATCH_MAX_READ_SIZE;
        if(is_buf_circulated(bd_info, r_size)){
            SENSOR_D_LOG("Ring buffer Circulation");
            ret = read_batchdata_circulated(bd_info, r_size, &new_s_ptr);
            if(ret != SNS_RC_OK){
                SENSOR_D_LOG("end");
                return ret;
            }
        } else {
            ret = read_batchdata(bd_info.s_ptr, r_size);
            if(ret != SNS_RC_OK){
                SENSOR_D_LOG("end");
                return ret;
            }
            new_s_ptr = bd_info.s_ptr + r_size;
        }
        *next_ptr = new_s_ptr;
        *next_remain = bd_info.remain_size - r_size;
    }
    SENSOR_D_LOG("end");
    return ret;
}

static void record_timestamp_data(uint32_t cur_ts_stask, uint32_t cur_ts_atask)
{
    int8_t i = 0;

    for(i = 0; i < BATCH_TS_TYPE_MAX; i++){
        previous_s3_ts[i] = current_s3_ts[i];
    }
    current_s3_ts[SENSOR_TASK] = cur_ts_stask;
    current_s3_ts[APP_TASK] = cur_ts_atask;
}

static void record_kernel_timestamp(int64_t current_ts)
{
    previous_batch_kts = current_batch_kts;
    current_batch_kts = current_ts;
}

static int8_t make_batch_markoff_param(enum batch_markoff_timing_type timing,
                                        const uint32_t cur_m4_rd_ptr,
                                        const uint32_t cur_ffe_rd_ptr,
                                        struct batch_markoff_param *out)
{
    int8_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    switch(timing){
        case START:
            SENSOR_D_LOG("timing[Batch MarkOff Start]");
            out->m4_buf_lock = 0x01;
            out->update_m4_rd_ptr = 0x00;
            out->cur_m4_rd_ptr = UNMENTIONED;
            out->ffe_buf_lock = 0x01;
            out->update_ffe_rd_ptr = 0x00;
            out->cur_ffe_rd_ptr = UNMENTIONED;
            break;
        case MIDSTREAM:
            SENSOR_D_LOG("timing[Batch MarkOff MiddleStream]");
            out->m4_buf_lock = 0x01;
            out->update_m4_rd_ptr = 0x01;
            out->cur_m4_rd_ptr = cur_m4_rd_ptr;
            out->ffe_buf_lock = 0x01;
            out->update_ffe_rd_ptr = 0x01;
            out->cur_ffe_rd_ptr = cur_ffe_rd_ptr;
            break;
        case END:
            SENSOR_D_LOG("timing[Batch MarkOff End]");
            out->m4_buf_lock = 0x00;
            out->update_m4_rd_ptr = 0x01;
            out->cur_m4_rd_ptr = cur_m4_rd_ptr;
            out->ffe_buf_lock = 0x00;
            out->update_ffe_rd_ptr = 0x01;
            out->cur_ffe_rd_ptr = cur_ffe_rd_ptr;
            break;
        default:
            SENSOR_ERR_LOG("INVALID timing value[%d]", timing);
            ret = SNS_RC_ERR;
            break;
    }
    SENSOR_D_LOG("end");
    return ret;
}

static int8_t run_batch_markoff_cmd(enum batch_markoff_timing_type timing,
                                    const uint32_t cur_m4_rd_ptr,
                                    const uint32_t cur_ffe_rd_ptr,
                                    struct batch_markoff_resp *resp)
{
    struct hc_info hc = {0};
    struct batch_markoff_param param;
    bool rec_timestamp_flg = false;
    int8_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    make_batch_markoff_param(timing, cur_m4_rd_ptr, cur_ffe_rd_ptr, &param);
    SENSOR_D_LOG("m4_buf_lock = %d " "update_m4_rd_ptr = %d "
        "cur_m4_rd_ptr = 0x%x " "ffe_buf_lock = %d "
        "update_ffe_rd_ptr = %d " "cur_ffe_rd_ptr = 0x%x ",
        param.m4_buf_lock, param.update_m4_rd_ptr,
        param.cur_m4_rd_ptr, param.ffe_buf_lock,
        param.update_ffe_rd_ptr, param.cur_ffe_rd_ptr);

    if(timing == START)
        rec_timestamp_flg = true;

    hc.command = prepare_host_command(BATCH_MODE_CONTROL,
                    BATCH, BMC_MARK_OFF_BATCHING_DATA);
    hc.param_data_len = sizeof(struct batch_markoff_param);
    memcpy(hc.param_data, &param, sizeof(struct batch_markoff_param));
    hc.response_len = sizeof(struct batch_markoff_resp);
    if(rec_timestamp_flg)
        record_kernel_timestamp(GET_CURRENT_TIMESTAMP_NS());
    ret = qleoss3_hostcmd(&hc);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("host cmd error");
        return ret;
    }
    memcpy(resp, &hc.response[0], sizeof(struct batch_markoff_resp));
    SENSOR_D_LOG("batch markoff response data");
    SENSOR_D_LOG("Valid m4 batch bytes = %u "
    "Valid ffe batch bytes = %u "
    "sens_task_ts = %u " "app_task_ts = %u "
    "cur_m4_rd_ptr = 0x%x " "cur_ffe_rd_ptr = 0x%x ",
    resp->m4_batch_bytes,
    resp->ffe_batch_bytes,
    resp->sens_task_ts,
    resp->app_task_ts,
    resp->cur_m4_rd_ptr,
    resp->cur_ffe_rd_ptr);

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t batch_mark_off_exec(bool is_discard)
{
    struct batch_markoff_param mark_off_param;
    struct batch_markoff_resp mark_off_resp;
    struct get_batchbuf_info m4_gbd_info;
    struct get_batchbuf_info ffe_gbd_info;
    //uint16_t index = 0;
    uint32_t m4_s_ptr = 0, m4_e_ptr = 0, ffe_s_ptr = 0, ffe_e_ptr = 0;
    uint32_t next_s_ptr = 0;
    uint16_t total_remain_bytes = 0;
    int16_t remain_bytes = 0;
	uint32_t m4_batch_buff_start_address;
	uint32_t m4_batch_buff_end_address;
	uint32_t ffe_batch_buff_start_address;
	uint32_t ffe_batch_buff_end_address;
    uint8_t timing = START;
#ifdef BATCHING_DYNAMIC_MEMORY
	struct batching_buff_adr batch_hdr;
#endif
    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");

    /* Mark off Batching Data */
    /* 1. in first markoff lock m4 and ffe rd ptrs
     * 2. change the read pointer and send mark off command with update
     * 3. */
    ret = run_batch_markoff_cmd(timing, UNMENTIONED, UNMENTIONED, &mark_off_resp);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("host cmd error");
        return ret;
    }
    record_timestamp_data(mark_off_resp.sens_task_ts, mark_off_resp.app_task_ts);
    SENSOR_D_LOG("sens_task_ts : previous[%u] current[%u]",
                previous_s3_ts[SENSOR_TASK], current_s3_ts[SENSOR_TASK]);
    SENSOR_D_LOG("app_task_ts  : previous[%u] current[%u]",
                previous_s3_ts[APP_TASK], current_s3_ts[APP_TASK]);

    if (!mark_off_resp.m4_batch_bytes && !mark_off_resp.ffe_batch_bytes) {
        SENSOR_D_LOG("No valid batch data\n");
        mark_off_data_bytes = 0;
        mark_off_param.cur_m4_rd_ptr = mark_off_resp.cur_m4_rd_ptr;
        mark_off_param.cur_ffe_rd_ptr = mark_off_resp.cur_ffe_rd_ptr;
        timing = END;
        run_batch_markoff_cmd(timing, mark_off_param.cur_m4_rd_ptr, mark_off_param.cur_ffe_rd_ptr, &mark_off_resp);
        SENSOR_D_LOG("end");
        return ret;
    }

#ifdef BATCHING_DYNAMIC_MEMORY
	//struct batching_buff_adr batch_hdr;
	QLSPI_Read_S3_Mem(BATCH_BUF_ADDRESS,
		(uint8_t*)&batch_hdr, sizeof(batch_hdr));
	m4_s_ptr = mark_off_resp.cur_m4_rd_ptr;
	m4_e_ptr = batch_hdr.m4_batch_end_addr;
	ffe_s_ptr = mark_off_resp.cur_ffe_rd_ptr;
	ffe_e_ptr = batch_hdr.ffe_batch_end_addr;

	m4_batch_buff_start_address = batch_hdr.m4_batch_start_addr;
	m4_batch_buff_end_address = batch_hdr.m4_batch_end_addr;
	ffe_batch_buff_start_address = batch_hdr.ffe_batch_start_addr;
	ffe_batch_buff_end_address = batch_hdr.ffe_batch_end_addr;
#else
	m4_s_ptr = mark_off_resp.cur_m4_rd_ptr;
	m4_e_ptr = M4_BATCH_DATA_END_ADDR;
	ffe_s_ptr = mark_off_resp.cur_ffe_rd_ptr;
	ffe_e_ptr = FFE_BATCH_DATA_END_ADDR;

	m4_batch_buff_start_address = M4_BATCH_DATA_START_ADDR;
	m4_batch_buff_end_address = M4_BATCH_DATA_END_ADDR;
	ffe_batch_buff_start_address = FFE_BATCH_DATA_START_ADDR;
	ffe_batch_buff_end_address = FFE_BATCH_DATA_END_ADDR;
#endif //BATCHING_DYNAMIC_MEMORY
	SENSOR_D_LOG("BATCH buffer M4 start address = 0x%X\t"
					"end address = 0x%X\t"
					"FFE start address = 0x%X\t"
					"End address = 0x%X\t\n",
					m4_batch_buff_start_address,
					m4_batch_buff_end_address,
					ffe_batch_buff_start_address,
					ffe_batch_buff_end_address);

    m4_gbd_info.s_ptr           = m4_s_ptr;
    m4_gbd_info.remain_size     = mark_off_resp.m4_batch_bytes;
    m4_gbd_info.l_limit_addr    = m4_batch_buff_start_address;
    m4_gbd_info.u_limit_addr    = m4_batch_buff_end_address;
    ffe_gbd_info.s_ptr          = ffe_s_ptr;
    ffe_gbd_info.remain_size    = mark_off_resp.ffe_batch_bytes;
    ffe_gbd_info.l_limit_addr   = ffe_batch_buff_start_address;
    ffe_gbd_info.u_limit_addr   = ffe_batch_buff_end_address;
    SENSOR_D_LOG("m4_data_byte=%d, FFE_data_byte=%d",
                    m4_gbd_info.remain_size, ffe_gbd_info.remain_size);
    mark_off_buf_idx = 0;

    while(1){
        if( (m4_gbd_info.remain_size < 0) || (ffe_gbd_info.remain_size < 0) ){
            SENSOR_ERR_LOG("Excess read M4[%d], FFE[%d]",
                            m4_gbd_info.remain_size, ffe_gbd_info.remain_size);
            goto exit;
        } else {
            SENSOR_D_LOG("Read Batch Buffer Data");
            if(m4_gbd_info.remain_size){
                SENSOR_D_LOG("get m4 batch buffer data");
                ret = get_batch_buffer_data(m4_gbd_info, &remain_bytes, &next_s_ptr);
                if(ret != SNS_RC_OK){
                    SENSOR_ERR_LOG("dev_read error");
                    goto exit;
                }
                m4_gbd_info.s_ptr = next_s_ptr;
                m4_gbd_info.remain_size = remain_bytes;
                if(!check_can_mark_off(m4_gbd_info.s_ptr, m4_batch_buff_start_address, m4_batch_buff_end_address)){
                    SENSOR_ERR_LOG("M4 Out of range error. cannot execute batch mark off.");
                    goto exit;
                }
            }
            if(ffe_gbd_info.remain_size){
                SENSOR_D_LOG("get ffe batch buffer data");
                ret = get_batch_buffer_data(ffe_gbd_info, &remain_bytes, &next_s_ptr);
                if(ret != SNS_RC_OK){
                    SENSOR_ERR_LOG("dev_read error");
                    goto exit;
                }
                ffe_gbd_info.s_ptr = next_s_ptr;
                ffe_gbd_info.remain_size = remain_bytes;
                if(!check_can_mark_off(ffe_gbd_info.s_ptr, ffe_batch_buff_start_address, ffe_batch_buff_end_address)){
                    SENSOR_ERR_LOG("FFE Out of range error. cannot execute batch mark off.");
                    goto exit;
                }
            }
            total_remain_bytes = m4_gbd_info.remain_size + ffe_gbd_info.remain_size;
            if(total_remain_bytes == 0){
                SENSOR_D_LOG("last batch_mark_off");
                timing = END;
            } else {
                timing = MIDSTREAM;
            }
        }
        ret = run_batch_markoff_cmd(timing, m4_gbd_info.s_ptr, ffe_gbd_info.s_ptr, &mark_off_resp);
        if(ret != SNS_RC_OK){
            SENSOR_ERR_LOG("host cmd error");
            break;
        }
        if(timing == END){
            mark_off_data_bytes = mark_off_buf_idx;
            break;
        }
    }
exit:
    SENSOR_D_LOG("get batch data size = %d", mark_off_data_bytes);
    mark_off_buf_idx = 0;
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t batch_mark_off(const int32_t trigger, bool report)
{
    int ret = SNS_RC_OK;
    bool is_discard = false;
    SENSOR_D_LOG("start report[%d]", report);

    switch(trigger){
        case LOGGING_TRIGGER_MAX_LATENCY:
        case LOGGING_TRIGGER_REMAIN_MARGIN:
        case LOGGING_TRIGGER_CHANGE_SENSOR_STATE:
        case LOGGING_TRIGGER_FLUSH:
            is_discard = false;
            break;
        case LOGGING_TRIGGER_M4_BUF_BLANK_FULL:
            is_discard = true;
            break;
        default:
            SENSOR_ERR_LOG("Invalid trigger[%d].", trigger);
            break;
    }

    mutex_lock(&batch_markoff_mutex);
    mutex_lock(&micon_hcres_mutex);
    ret = batch_mark_off_exec(is_discard);
    mutex_unlock(&micon_hcres_mutex);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("error cannot read batch data. data discard.");
        mark_off_data_bytes = 0;
    }
    else if(report){
        ret = iio_report_events(mark_off_buf, mark_off_data_bytes);
        mark_off_data_bytes = 0;
    }
    mutex_unlock(&batch_markoff_mutex);
    SENSOR_D_LOG("end");
    return ret;
}

static int verify_sensor_data(const uint8_t h_val, const uint8_t t_val)
{
    int result = VERIFY_NG;
    if((h_val == VERIFY_HEAD_VAL) && (t_val == VERIFY_TAIL_VAL)) {
        result = VERIFY_OK;
    }
    return result;
}

static int iio_report_event_now(enum sensor_e_type type)
{
	int ret = SNS_RC_OK;
    int64_t now = GET_CURRENT_TIMESTAMP_NS();
    uint8_t data[EVENT_DATA_SIZE] = {0};
	int16_t i = 0;
	int32_t Pl;
    int16_t x,y,z,A;
    int16_t o_x,o_y,o_z;
    int16_t diff_x, diff_y, diff_z;

    switch (type) {
    case SENSOR_ACC:
        x = (int16_t)last_acc_data.outX;
        y = (int16_t)last_acc_data.outY;
        z = (int16_t)last_acc_data.outZ;

        SENSOR_D_LOG("accel        %d\t%d\t%d", x, y, z);

        mutex_lock(&acc_report_event_now_mutex);
        acc_clear_filt_coff_exe(acc_val_clear_flg);

        diff_x = abs(x - prv_acc.outX);
        diff_y = abs(y - prv_acc.outY);
        diff_z = abs(z - prv_acc.outZ);
        if(no_noize_reduction_filter(diff_x, diff_y, diff_z, SENSOR_ACC)){
            no_noize_reduction_filt_counter = INIT_NO_NR_FILT_TERM;
        }
        if (no_noize_reduction_filt_counter == 0) {
            x = apply_noize_reduction_filter(x, prv_acc.outX, ACCELEROMETER);
            y = apply_noize_reduction_filter(y, prv_acc.outY, ACCELEROMETER);
            z = apply_noize_reduction_filter(z, prv_acc.outZ, ACCELEROMETER);
            x = apply_noize_reduction_filter_ext(x, prv_acc.outX, &acc_val.x, &acc_hys_val.x, &acc_rel.x, ACCELEROMETER);
            y = apply_noize_reduction_filter_ext(y, prv_acc.outY, &acc_val.y, &acc_hys_val.y, &acc_rel.y, ACCELEROMETER);
            z = apply_noize_reduction_filter_ext(z, prv_acc.outZ, &acc_val.z, &acc_hys_val.z, &acc_rel.z, ACCELEROMETER);
        }
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        SENSOR_D_LOG("ACC ts noize reduction.");
        now = apply_ts_noize_reduction_filter(now, &acc_ts, acc_base_set_val_rcv);
        break;
    case SENSOR_MAG:
        x = (int16_t)last_mag_data.x;
        y = (int16_t)last_mag_data.y;
        z = (int16_t)last_mag_data.z;
        A = (int16_t)last_mag_data.accuracy;
        SENSOR_D_LOG("mag          %d\t%d\t%d\t%d", x, y, z, A);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        data[6] = (uint8_t)(A & 0x00ff);
        //data[7] = (uint8_t)( (A >> 8) & 0x00ff);
        break;
    case SENSOR_MAG_UNCAL:
        x = (int16_t)(last_maguc_data.x - last_maguc_data.cal_x);
        y = (int16_t)(last_maguc_data.y - last_maguc_data.cal_y);
        z = (int16_t)(last_maguc_data.z - last_maguc_data.cal_z);
        A = (int16_t)last_mag_data.accuracy;
        SENSOR_D_LOG("mag uc          %d\t%d\t%d\t%d", x, y, z, A);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        data[6] = (uint8_t)(A & 0x00ff);
        //magnetic field uncalibrated offset
        o_x = (int16_t)last_maguc_data.cal_x;
        o_y = (int16_t)last_maguc_data.cal_y;
        o_z = (int16_t)last_maguc_data.cal_z;
        SENSOR_D_LOG("mag uc(ofs)        %d\t%d\t%d", o_x, o_y, o_z);
        data[7] = (uint8_t)(o_x & 0x000000ff);
        data[8] = (uint8_t)((o_x >> 8) & 0x000000ff);
        data[9] = (uint8_t)(o_y & 0x000000ff);
        data[10] = (uint8_t)((o_y >> 8) & 0x000000ff);
        data[11] = (uint8_t)(o_z & 0x000000ff);
        data[12] = (uint8_t)((o_z >> 8) & 0x000000ff);
        break;

    case SENSOR_GYRO:
        if(sns_is_needed_gyro_sync()){
            SENSOR_D_LOG("synchronization with uncal sensor");
            ret = SNS_RC_ERR;
        } else {
            x = (int16_t)last_gyro_data.x;
            y = (int16_t)last_gyro_data.y;
            z = (int16_t)last_gyro_data.z;
            data[0] = (uint8_t)(x & 0x00ff);
            data[1] = (uint8_t)( (x >> 8) & 0x00ff);
            data[2] = (uint8_t)(y & 0x00ff);
            data[3] = (uint8_t)( (y >> 8) & 0x00ff);
            data[4] = (uint8_t)(z & 0x00ff);
            data[5] = (uint8_t)( (z >> 8) & 0x00ff);

            mutex_lock(&gyro_report_event_now_mutex);

            gyro_clear_filt_coff_exe(gyro_val_clear_flg);

            now = apply_ts_noize_reduction_filter(now, &gyro_ts, gyro_base_set_val_rcv);


            SENSOR_D_LOG("gyro         %d\t%d\t%d", x, y, z);
        }
        break;
    case SENSOR_GYRO_UNCAL:
        x = (int16_t)(last_gyrouc_data.x - last_gyrouc_data.cal_x);
        y = (int16_t)(last_gyrouc_data.y - last_gyrouc_data.cal_y);
        z = (int16_t)(last_gyrouc_data.z - last_gyrouc_data.cal_z);
        SENSOR_D_LOG("gyro uc        %d\t%d\t%d", x, y, z);

        mutex_lock(&gyrouc_report_event_now_mutex);

        gyrouc_clear_filt_coff_exe(gyrouc_val_clear_flg);

        if(no_noize_reduction_filter(x, y, z, SENSOR_GYRO_UNCAL)){
            no_noize_reduction_filt_counter = INIT_NO_NR_FILT_TERM;
        }
        if (no_noize_reduction_filt_counter == 0) {
            x = apply_noize_reduction_filter(x, prv_gyro.x, GYROSCOPE);
            y = apply_noize_reduction_filter(y, prv_gyro.y, GYROSCOPE);
            z = apply_noize_reduction_filter(z, prv_gyro.z, GYROSCOPE);
            x = apply_noize_reduction_filter_ext(x, prv_gyro.x, &gyro_val.x, &gyro_hys_val.x, &gyro_rel.x, GYROSCOPE);
            y = apply_noize_reduction_filter_ext(y, prv_gyro.y, &gyro_val.y, &gyro_hys_val.y, &gyro_rel.y, GYROSCOPE);
            z = apply_noize_reduction_filter_ext(z, prv_gyro.z, &gyro_val.z, &gyro_hys_val.z, &gyro_rel.z, GYROSCOPE);
        } else {
            no_noize_reduction_filt_counter--;
        }
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        SENSOR_D_LOG("GYROUncal ts noize reduction.");
        now = apply_ts_noize_reduction_filter(now, &gyrouc_ts, gyrouc_base_set_val_rcv);

        if(sns_is_needed_gyro_sync()){
            sns_iio_report_event(
                    sensor_type_to_smid[SENSOR_GYRO],
                    data,
                    EVENT_DATA_SIZE,
                    now,
                    0);
            if (gyrouc_ts.luck_sample_num > 0) {
                for (i=0; i<gyrouc_ts.luck_sample_num; i++) {
                    if (SNS_RC_OK == ret) {
                        sns_iio_report_event(
                                sensor_type_to_smid[SENSOR_GYRO],
                                data,
                                EVENT_DATA_SIZE,
				                now + (gyrouc_ts.base_val * (i+1)),
                                0);
                    }
                }
            }
        }
        o_x = gyro_dynamic_offset[0];
        o_y = gyro_dynamic_offset[1];
        o_z = gyro_dynamic_offset[2];
        SENSOR_D_LOG("gyro uc(ofs)       %d\t%d\t%d", o_x, o_y, o_z);
        data[6] = (uint8_t)(o_x & 0x000000ff);
        data[7] = (uint8_t)((o_x >> 8) & 0x000000ff);
        data[8] = (uint8_t)(o_y & 0x000000ff);
        data[9] = (uint8_t)((o_y >> 8) & 0x000000ff);
        data[10] = (uint8_t)(o_z & 0x000000ff);
        data[11] = (uint8_t)((o_z >> 8) & 0x000000ff);

        break;

    case SENSOR_PRESSURE:
        Pl = last_press_data.pressure;
        SENSOR_D_LOG("pressure        %d", Pl);
        data[0] = (uint8_t)(Pl & 0x000000ff);
        data[1] = (uint8_t)((Pl >> 8) & 0x000000ff);
        data[2] = (uint8_t)((Pl >> 16) & 0x000000ff);
        data[3] = (uint8_t)((Pl >> 24) & 0x000000ff);

        //Do not use press_Pa_H.
        //data[4] = (uint8_t)(Ph & 0x000000ff);
        //data[5] = (uint8_t)((Ph >> 8) & 0x000000ff);
        //data[6] = (uint8_t)((Ph >> 16) & 0x000000ff);
        //data[7] = (uint8_t)((Ph >> 24) & 0x000000ff);
        break;

    case SENSOR_ORTN:
        x = (int16_t)last_ori_data.pitch;
        y = (int16_t)last_ori_data.roll;
        z = (int16_t)last_ori_data.yaw;
        A = (int16_t)last_ori_data.accuracy;
        SENSOR_D_LOG("orient       %d\t%d\t%d", x, y, z);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        data[6] = (uint8_t)(A & 0x00ff);
        break;

    case SENSOR_GRV:
        x = (int16_t)last_grv_data.x;
        y = (int16_t)last_grv_data.y;
        z = (int16_t)last_grv_data.z;
        SENSOR_D_LOG("gravity      %d\t%d\t%d", x, y, z);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        break;

    case SENSOR_ACC_LNR:
        x = (int16_t)last_lacc_data.x;
        y = (int16_t)last_lacc_data.y;
        z = (int16_t)last_lacc_data.z;
        SENSOR_D_LOG("linear accel %d\t%d\t%d", x, y, z);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        break;

    case SENSOR_ROT_VCTR:
        x = (int16_t)last_rv_data.x;
        y = (int16_t)last_rv_data.y;
        z = (int16_t)last_rv_data.z;
        SENSOR_D_LOG("rotation     %d\t%d\t%d", x, y, z);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        break;

    case SENSOR_GAME_ROT_VCTR:
        x = (int16_t)last_gamrv_data.x;
        y = (int16_t)last_gamrv_data.y;
        z = (int16_t)last_gamrv_data.z;
        SENSOR_D_LOG("game rot     %d\t%d\t%d", x, y, z);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        break;

    case SENSOR_MAG_ROT_VCTR:
        x = (int16_t)last_georv_data.x;
        y = (int16_t)last_georv_data.y;
        z = (int16_t)last_georv_data.z;
        A = last_georv_data.accuracy;
        SENSOR_D_LOG("geomag rot   %d\t%d\t%d\t%d", x, y, z, A);
        data[0] = (uint8_t)(x & 0x00ff);
        data[1] = (uint8_t)( (x >> 8) & 0x00ff);
        data[2] = (uint8_t)(y & 0x00ff);
        data[3] = (uint8_t)( (y >> 8) & 0x00ff);
        data[4] = (uint8_t)(z & 0x00ff);
        data[5] = (uint8_t)( (z >> 8) & 0x00ff);
        data[6] = (uint8_t)(A & 0x00ff);
        data[7] = (uint8_t)( (A >> 8) & 0x00ff);
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
    if (type == SENSOR_ACC){
        if (acc_ts.luck_sample_num > 0) {
            for (i=0; i<acc_ts.luck_sample_num; i++) {
                if (SNS_RC_OK == ret) {
                    sns_iio_report_event(
                            sensor_type_to_smid[type],
                            data,
                            EVENT_DATA_SIZE,
			                now + (acc_ts.base_val * (i+1)),
                            0);
                }
            }
        }
        if (no_noize_reduction_filt_counter != 0) {
            acc_part_clear_filt_coff();
        }
        mutex_unlock(&acc_report_event_now_mutex);
    }
    if (type == SENSOR_GYRO){
        if(!sns_is_needed_gyro_sync()){
            if (gyro_ts.luck_sample_num > 0) {
                for (i=0; i<gyro_ts.luck_sample_num; i++) {
                    if (SNS_RC_OK == ret) {
                        sns_iio_report_event(
                                sensor_type_to_smid[type],
                                data,
                                EVENT_DATA_SIZE,
				                now + (gyro_ts.base_val * (i+1)),
                                0);
                    }
                }
            }

            mutex_unlock(&gyro_report_event_now_mutex);
        }
    }
    if (type == SENSOR_GYRO_UNCAL){
        if (gyrouc_ts.luck_sample_num > 0) {
            for (i=0; i<gyrouc_ts.luck_sample_num; i++) {
                if (SNS_RC_OK == ret) {
                    sns_iio_report_event(
                            sensor_type_to_smid[type],
                            data,
                            EVENT_DATA_SIZE,
			                now + (gyrouc_ts.base_val * (i+1)),
                            0);
                }
            }
        }
        if (no_noize_reduction_filt_counter != 0) {
            gyrouc_part_clear_filt_coff();
        }
        mutex_unlock(&gyrouc_report_event_now_mutex);
    }
    SENSOR_D_LOG("end");
    return ret;
}


static int32_t uwater_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_UWD,
            APP_UWD__UWD_Setting);
    hc.param_data[0] = 0x01             // CTRL_UWD_Main_Change
                     | (param << 8)     // CTRL_UWD_Algo_En
                     | (0x00 << 16)     // CTRL_UWD_InitLoop
                     | (param << 24);   // CTRL_UWD_WaterDetect_INT_En
    hc.param_data[1] = param            // CTRL_UWD_TargetDepthDetect_INT_En
                     | (param << 8)     // CTRL_UWD_MaxDepthDetect_INT_En
                     | (0x0A << 16)     // CTRL_UWD_PressureChangeTh_L
                     | (0x00 << 24);    // CTRL_UWD_PressureChangeTh_H
    hc.param_data[2] = 0x64             // CTRL_UWD_JudgeWaitSmp_In_L
                     | (0x00 << 8)      // CTRL_UWD_JudgeWaitSmp_In_H
                     | (0xE8 << 16)     // CTRL_UWD_JudgeDelta_In_L
                     | (0x03 << 24);    // CTRL_UWD_JudgeDelta_In_H
    hc.param_data[3] = 0x64             // CTRL_UWD_JudgeWaitSmp_Out_L
                     | (0x00 << 8)      // CTRL_UWD_JudgeWaitSmp_Out_H
                     | (0x0A << 16)     // CTRL_UWD_JudgeDelta_Out_L
                     | (0x00 << 24);    // CTRL_UWD_JudgeDelta_Out_H
    hc.param_data[4] = 0x00             // CTRL_UWD_Param1_Change
                     | (0x96 << 8)      // CTRL_UWD_TargetWaterDepth
                     | (0x00 << 16)     // CTRL_UWD_Param2_Change
                     | (0x00 << 24);    // CTRL_UWD_MaxWaterDepth
    hc.param_data_len = 20;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t vtrigger_enable(bool enable)
{
    struct hc_info hc = {0};
    SENSOR_D_LOG("start enable[%d]", enable);
    hc.command = prepare_host_command(VOICE, KPD_DETECTION,
                                        VOICE_SET_KPD_ENABLE);
    hc.param_data[0] = (uint8_t)enable;
    hc.param_data_len = 6;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return SNS_RC_OK;
}

static int32_t kcmot_get_bringup_data(struct kc_motion_bringup_data *arg_Bringup)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DSPLUS,
            APP_FFEAPP__BU_GetInfo);
    hc.param_data[0] = 0x00;
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 0;
    hc.response_len = 1;
    mutex_lock(&micon_hcres_mutex);
    qleoss3_hostcmd(&hc);

    //CTRL_BU_State
    SENSOR_D_LOG("response 0[%d]",hc.response[0]);
    arg_Bringup->status = hc.response[0];
    mutex_unlock(&micon_hcres_mutex);

    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_bringup_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_FFEAPP,
            APP_FFEAPP__BU_OnOff);
    hc.param_data[0] = 0x01           // CTRL_BU_Main_Change
                     | (param << 8)   // CTRL_BU_En_Function
                     | (param << 16)  // CTRL_BU_En_Interrupt
                     | (0x05 << 24);  // CTRL_BU_MovingDtctLvSumThMin
    hc.param_data[1] = 0x03           // CTRL_BU_MovingDtctSumNum
                     | (0x0c << 8)    // CTRL_BU_StaticDtctLvSumThMax
                     | (0x06 << 16)   // CTRL_BU_StaticDtctSumNum
                     | (0xb6 << 24);  // CTRL_BU_StaticDtctPeriodNum
    hc.param_data[2] = 0x00           // CTRL_BU_StaticDtctPeriodNum
                     | (0x00 << 8)    // padding
                     | (0x00 << 16)   // padding
                     | (0x00 << 24);  // padding
    hc.param_data_len = 9;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;

}

static int32_t kcmot_other_vehi_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_N_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DSPLUS,
            APP_DSPLUS__TR_OnOff);

    hc.param_data[0] = 0x00           // CTRL_TR_Main_Change
                     | (0x00 << 8)    // CTRL_TR_En
                     | (0x00 << 16)   // CTRL_TR_DtctINT_En
                     | (0x00 << 24);  // CTRL_TR_Log_En
    hc.param_data[1] = 0x01           // CTRL_TR_Other_Main_Change
                     | (param << 8)   // CTRL_TR_OtherEn
                     | (param << 16)  // CTRL_TR_OtherDtctINT_En
                     | (0x00 << 24);  // padding
    hc.param_data_len = 7;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_train_get_data(struct kc_motion_train *arg_Train)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    /* Send command */
    hc.command = prepare_host_command(APP, APP_DSPLUS,
            APP_DSPLUS__TR_GetInfo);
    hc.param_data[0] = 0x00;
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 0;
    hc.response_len = 11;
    qleoss3_hostcmd(&hc);

    arg_Train->usTrFstDetect = hc.response[0];
    arg_Train->usTrOtherFstDetect = hc.response[1];
    arg_Train->usTrRes = hc.response[2];
    arg_Train->TrDtctTime = (((uint32_t)hc.response[3]      ) & 0x000000FF)
                          | (((uint32_t)hc.response[4] <<  8) & 0x0000FF00)
                          | (((uint32_t)hc.response[5] << 16) & 0x00FF0000)
                          | (((uint32_t)hc.response[6] << 24) & 0xFF000000);
    arg_Train->TrDtctTimeFix = (((uint32_t)hc.response[7]       ) & 0x000000FF)
                             | (((uint32_t)hc.response[8]  <<  8) & 0x0000FF00)
                             | (((uint32_t)hc.response[9]  << 16) & 0x00FF0000)
                             | (((uint32_t)hc.response[10] << 24) & 0xFF000000);
    mutex_unlock(&micon_hcres_mutex);

    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_train_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DSPLUS,
            APP_DSPLUS__TR_OnOff);

    hc.param_data[0] = 0x01           // CTRL_TR_Main_Change
                     | (param << 8)   // CTRL_TR_En
                     | (param << 16)  // CTRL_TR_DtctINT_En
                     | (0x00 << 24);  // CTRL_TR_Log_En
    hc.param_data[1] = 0x00           // CTRL_TR_Other_Main_Change
                     | (0x00 << 8)    // CTRL_TR_OtherEn
                     | (0x00 << 16)   // CTRL_TR_OtherDtctINT_En
                     | (0x00 << 24);  // padding
    hc.param_data_len = 7;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;

}

static int32_t kcmot_wstop_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DSPLUS,
            APP_DSPLUS__SP_OnOff);
    hc.param_data[0] = 0x01           // CTRL_SP_Main_Change
                     | (param << 8)   // CTRL_SP_En
                     | (0x00 << 16)   // padding
                     | (0x00 << 24);  // padding
    hc.param_data_len = 2;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t kcmot_wstart_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DSPLUS,
            APP_DSPLUS__BW_OnOff);
    hc.param_data[0] = 0x01           // CTRL_BW_Main_Change
                     | (param << 8)   // CTRL_BW_En
                     | (0x00 << 16)   // padding
                     | (0x00 << 24);  // padding
    hc.param_data_len = 2;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;

}

static int32_t vhdet_get_data(struct vhdetect *arg_VHdetect)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;
    uint32_t CTRL_VH_State = 0;

    SENSOR_N_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    /* Send command */
    hc.command = prepare_host_command(APP, APP_FFEAPP,
            APP_FFEAPP__VH_GetInfo);
    hc.param_data[0] = 0x00;
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 0;
    hc.response_len = 1;
    qleoss3_hostcmd(&hc);
    CTRL_VH_State = hc.response[0];
    arg_VHdetect->usVhStatus = CTRL_VH_State;
    mutex_unlock(&micon_hcres_mutex);
    SENSOR_D_LOG("CTRL_VH_State[%d]",CTRL_VH_State);


    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t vhdet_enable(bool enable)
{
    int32_t ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");
    if(enable == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_FFEAPP,
            APP_FFEAPP__VH_OnOff);
    hc.param_data[0] = 0x01           // CTRL_VH_Main_Change
                     | (param << 8)   // CTRL_VH_En_Function
                     | (param << 16)  // CTRL_VH_En_Interrupt
                     | (0x00 << 24);  // CTRL_VH_InitialState
    hc.param_data_len = 4;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_D_LOG("end");
    return ret;
}

int32_t baro_enable(bool enable)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    struct hc_info hc;

    SENSOR_N_LOG("start enable[%d]", enable);

    param = enable ? HC_VALID : HC_INVALID;

    if (enable) {

        /* Send command */
        hc.command = prepare_host_command(APP, APP_BR,
                APP_BR__BR_LogPrsSetting);
        hc.param_data[0] = 0x01                // CTRL_Log_PrsParam_Change
                         | (0x30 << 8)         // CTRL_PS_LogP_MinFreq
                         | (0x75 << 16)        // CTRL_PS_LogP_MinFreq
                         | (0x00 << 24);       // CTRL_PS_LogP_MinFreq
        hc.param_data[1] = 0x00                // CTRL_PS_LogP_MinFreq
                         | (0x0e << 8)         // CTRL_PS_LogP_ChangeTh
                         | (0x00 << 16)        // padding
                         | (0x00 << 24);       // padding
        hc.param_data[2] = 0x00;
        hc.param_data[3] = 0x00;
        hc.param_data[4] = 0x00;
        hc.param_data[5] = 0x00;
        hc.param_data[6] = 0x00;
        hc.param_data[7] = 0x00;
        hc.param_data_len = 6;
        hc.response_len = 0;
        qleoss3_hostcmd(&hc);

    }

    ret = dailys_send_latency_option(SENSOR_EXT_BARO, enable);
    if (SNS_RC_OK != ret) {
        SENSOR_ERR_LOG("end sns_dailys_send_latency_option err ret[%d]",ret);
        return SNS_RC_ERR;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_BR,
            APP_BR__BR_LogSetting);
    hc.param_data[0] = 0x01                 // CTRL_Log_BR_Main_Change
                     | (param << 8)         // CTRL_Log_BR_En
                     | (param << 16)        // CTRL_Log_P_ChangeEn
                     | (0x00 << 24);        // padding
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 3;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    if (enable) {
        clear_evbuf_markoff_base_timestamps();
    }
    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static void dailys_vib_interlocking(int32_t mode, int8_t interlocking)
{
    int ret = SNS_RC_OK;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    memset(&hc, 0, sizeof(hc));
    hc.command = prepare_host_command(SENSOR_DEVICE_CONTROL,
            ACCELEROMETER, SDC_SET_MEASUREMENT_MODE);
    hc.param_data_len = 1;
    hc.param_data[0] = interlocking;
    hc.response_len = 0;

    SENSOR_D_LOG("interlocking : %d", interlocking);

    ret = qleoss3_hostcmd(&hc);
    if (ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("host cmd error");
    }

    SENSOR_D_LOG("end");
}

static void record_evmarkoff_kernel_timestamp(int64_t current_ts)
{
    current_ev_markoff_kts = current_ts;
}

static void record_evmarkoff_s3_timestamp(int32_t current_ts)
{
    curent_ev_markoff_s3ts = current_ts;
}

static void get_event_mark_off_times(int64_t *cur_ktime, int32_t *cur_micontime)
{
    *cur_ktime = current_ev_markoff_kts;
    *cur_micontime = curent_ev_markoff_s3ts;
}

static void get_event_mark_off_clrinfo(uint8_t *is_clr_exist, int32_t *clr_micontime)
{
    *is_clr_exist   = clr_exist_info;
    *clr_micontime  = clr_micon_timestamp;
}

static void clear_evbuf_markoff_base_timestamps(void)
{
	SENSOR_D_LOG("start");
    dailys_event_mark_off();
    sensor_report_event(SENSOR_EXT_PEDO, 0, (void*)&event_mark_off_buf[0],0);
	SENSOR_D_LOG("end");
}

static int32_t dailys_event_mark_off(void)
{
    int32_t ret;
    mutex_lock(&batch_markoff_mutex);
    ret = dailys_event_mark_off_exec();
    mutex_unlock(&batch_markoff_mutex);
    return ret;
}

static int32_t dailys_event_mark_off_exec(void)
{
	struct hc_info hc = {0};
	struct event_batch_markoff_param mark_off_param;
	struct event_batch_markoff_resp mark_off_resp;

	uint32_t data_bytes, events, s_ptr = 0, e_ptr = 0, remaining_bytes = 0;
	uint16_t index = 0;
	uint32_t evt_buff_start_addr = 0;
	uint32_t evt_buff_end_addr = 0;
#ifdef BATCHING_DYNAMIC_MEMORY
	struct evt_batching_buff_adr evt_batch_hdr;
#endif
	uint16_t i = 0;
	uint32_t tmp_s_ptr = 0;
	int x_data_bytes = 0;
	uint32_t index_padding = 0;
    int32_t ret = SNS_RC_OK;
	int end_padding = 0;
    int64_t mark_off_kernel_time = 0;

	SENSOR_D_LOG("start");
	/*Mark off Event batching Data*/
	hc.command = prepare_host_command(APP, APP_EBUF,
				APP_EBUF__EventBufMarkOff);
	hc.param_data_len = sizeof(mark_off_param);

	mark_off_param.evt_buf_lock 		= 1;
	mark_off_param.update_evt_rd_ptr 	= 0;
	mark_off_param.cur_rd_ptr		= 0;

	memcpy(hc.param_data, &mark_off_param, sizeof(mark_off_param));
	hc.response_len = sizeof(mark_off_resp);
	ret = qleoss3_hostcmd(&hc);
    mark_off_kernel_time = get_ev_markoff_ktime();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("host cmd error");
		return -1;

    }
    record_evmarkoff_kernel_timestamp(mark_off_kernel_time);
	memcpy(&mark_off_resp, &hc.response[0], sizeof(mark_off_resp));

    record_evmarkoff_s3_timestamp(mark_off_resp.timestamp);
	data_bytes = mark_off_resp.valid_data_bytes;
	//event_mark_off_data_bytes = data_bytes;
	events = mark_off_resp.valid_events;

	if (!mark_off_resp.valid_data_bytes) {
		SENSOR_D_LOG("No valid event batch data\n");
	    mark_off_param.evt_buf_lock = 0;
		mark_off_param.update_evt_rd_ptr = 1;
		mark_off_param.cur_rd_ptr = mark_off_resp.cur_rd_ptr;
		hc.command = prepare_host_command(APP, APP_EBUF,
				APP_EBUF__EventBufMarkOff);
		hc.param_data_len = sizeof(mark_off_param);
		memcpy(hc.param_data, &mark_off_param, sizeof(mark_off_param));
		hc.response_len = 0;
		qleoss3_hostcmd(&hc);

		return 0;
	}

	SENSOR_D_LOG("Valid event batch bytes = %u "
		"Valid Events = %u "
		"Timestamp = %u "
		"cur_rd_ptr = 0x%x "
        "is_cleared = 0x%x "
        "clr_timestamp = %d",
		mark_off_resp.valid_data_bytes,
		mark_off_resp.valid_events,
		mark_off_resp.timestamp,
		mark_off_resp.cur_rd_ptr,
        mark_off_resp.is_cleared,
        mark_off_resp.clr_timestamp);

#ifdef BATCHING_DYNAMIC_MEMORY
	QLSPI_Read_S3_Mem(EVENT_BATCH_BUF_ADDRESS,
		(uint8_t*)&evt_batch_hdr, sizeof(evt_batch_hdr));
	s_ptr = mark_off_resp.cur_rd_ptr;
	e_ptr = evt_batch_hdr.evt_batch_end_addr;

	evt_buff_start_addr = evt_batch_hdr.evt_batch_start_addr;
	evt_buff_end_addr = evt_batch_hdr.evt_batch_end_addr;
	SENSOR_D_LOG("EVENT buffer start address = 0x%X "
					"End address = 0x%X",
					evt_buff_start_addr, evt_buff_end_addr);
#else
	s_ptr = mark_off_resp.cur_rd_ptr;
	e_ptr = EVENT_BATCH_DATA_END_ADDR;
	evt_buff_start_addr = EVENT_BATCH_DATA_START_ADDR;
	evt_buff_end_addr = EVENT_BATCH_DATA_END_ADDR;
#endif
    clr_exist_info = mark_off_resp.is_cleared;
    clr_micon_timestamp = mark_off_resp.clr_timestamp;

	if ((s_ptr & 0x3) ) {
		index_padding += (s_ptr & 0x3);
	}
	SENSOR_D_LOG("index_padding:%d",index_padding);

	while(data_bytes) {
		tmp_s_ptr = s_ptr;
		if ((s_ptr & 0x3) ) {
			data_bytes += (s_ptr & 0x3);
			s_ptr = s_ptr & 0xfffffffffffffffc;
		}
		SENSOR_D_LOG("s_ptr:0x%x",s_ptr);

        if (data_bytes) {
            if ((s_ptr + 2040) >= e_ptr) {
                x_data_bytes = e_ptr - s_ptr;

                if (data_bytes >= 2040) {
                    dev_read(s_ptr, &event_mark_off_buf[index],
                        x_data_bytes);
                    s_ptr = evt_buff_start_addr;
                    index += x_data_bytes;
                    data_bytes -= x_data_bytes;

                    x_data_bytes = 2040 - x_data_bytes;
                    if (x_data_bytes) {
                        dev_read(s_ptr, &event_mark_off_buf[index],
                            x_data_bytes);
                        s_ptr += x_data_bytes;
                        index += x_data_bytes;
                        data_bytes -= x_data_bytes;
                    }
                    mark_off_param.evt_buf_lock = 1;
                } else {
                    if (data_bytes >= x_data_bytes) {
                        dev_read(s_ptr, &event_mark_off_buf[index],
                            x_data_bytes);
                        s_ptr = evt_buff_start_addr;
                        index += x_data_bytes;
                        data_bytes -= x_data_bytes;

                        if (data_bytes) {
                            end_padding = 0;
                            if (data_bytes & 0x03) {
                                end_padding = (data_bytes & 0x03);
                                SENSOR_D_LOG("end_padding:%d",end_padding);
                            }
                            //dev_read(s_ptr, &event_mark_off_buf[index],
                            //    data_bytes);
                            dev_read(s_ptr, &event_mark_off_buf[index],
                                data_bytes+end_padding);
                            s_ptr += data_bytes;
                            index += data_bytes;
                            data_bytes = 0;
                        }
                    } else {
                        end_padding = 0;
                        if (data_bytes & 0x03) {
                            end_padding = (data_bytes & 0x03);
                            SENSOR_D_LOG("end_padding:%d",end_padding);
                        }
                        //dev_read(s_ptr, &event_mark_off_buf[index],
                        //    data_bytes);
                        dev_read(s_ptr, &event_mark_off_buf[index],
                            data_bytes+end_padding);
                        s_ptr += data_bytes;
                        index += data_bytes;
                        data_bytes = 0;
                    }
                    mark_off_param.evt_buf_lock = 0;
                }
            } else {
                if (data_bytes >= 2040) {
                    dev_read(s_ptr, &event_mark_off_buf[index],
                        2040);
                    s_ptr += 2040;
                    index += 2040;
                    data_bytes -= 2040;
                    mark_off_param.evt_buf_lock = 1;
                } else {
                    end_padding = 0;
                    if (data_bytes & 0x03) {
                        end_padding = (data_bytes & 0x03);
                        SENSOR_D_LOG("end_padding:%d\n",end_padding);
                    }
                    //dev_read(s_ptr, &event_mark_off_buf[index],
                    //    data_bytes);
                    SENSOR_D_LOG("startp:0x%x, read_bytes:%d, end_padding:%d",s_ptr, data_bytes, end_padding);
                    dev_read(s_ptr, &event_mark_off_buf[index],
                        data_bytes+end_padding);
                    s_ptr += data_bytes;
                    index += data_bytes;
                    data_bytes = 0;
                    mark_off_param.evt_buf_lock = 0;
                }
            }
            mark_off_param.update_evt_rd_ptr = 1;
            mark_off_param.cur_rd_ptr = s_ptr;
        } else {
            mark_off_param.evt_buf_lock = 0;
            mark_off_param.update_evt_rd_ptr = 0;
            mark_off_param.cur_rd_ptr = 0;
        }




		if(s_ptr == e_ptr) {
            s_ptr = evt_buff_start_addr;
		}

		mark_off_param.update_evt_rd_ptr = 1;
		mark_off_param.cur_rd_ptr = s_ptr;


		/* Send command */
		hc.command = prepare_host_command(APP, APP_EBUF,
				APP_EBUF__EventBufMarkOff);

		hc.param_data_len = sizeof(mark_off_param);
		memcpy(hc.param_data, &mark_off_param, sizeof(mark_off_param));

		SENSOR_D_LOG("evt_buf_lock = %d "
			"update_evt_rd_ptr = %d "
			"cur_rd_ptr = 0x%x ",
			mark_off_param.evt_buf_lock,
			mark_off_param.update_evt_rd_ptr,
			mark_off_param.cur_rd_ptr);

		hc.response_len = 0;
		qleoss3_hostcmd(&hc);
	}
		SENSOR_D_LOG("event_mark_off_buf index:%d remaining_bytes:%d",index,remaining_bytes);
		SENSOR_D_LOG("==========event_mark_off_buf==========");
		for (i=index_padding;i<index;i++) {
			SENSOR_D_LOG("event_mark_off_buf[%d] = 0x%02x",i,event_mark_off_buf[i]);
		}
		SENSOR_D_LOG("----------event_mark_off_buf----------");

	sensor_report_event(SENSOR_EXT_PEDO, eventbuf_cleartime, (void*)&event_mark_off_buf[index_padding], index-(index_padding));
    SENSOR_D_LOG("end");
	return 0;
}

static int32_t dailys_get_vehi_data(struct vehicle *arg_Vehi)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DS,
            APP_DS__VC_GetInfo);
    hc.param_data[0] = 0x00;
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 0;
    hc.response_len = 79;
    ret = qleoss3_hostcmd(&hc);
    if(ret != QL_STATUS_OK){
        SENSOR_ERR_LOG("host command error [0x%08x]", hc.command);
        goto err_exit;
    }

    arg_Vehi->usVehiStatus       = (((int32_t)hc.response[0]      ) & 0x000000FF);
    arg_Vehi->usVehiKind         = (((int32_t)hc.response[1]      ) & 0x000000FF);
    arg_Vehi->usVehiDetectTime   = (((int32_t)hc.response[2]      ) & 0x000000FF)
                                 | (((int32_t)hc.response[3] <<  8) & 0x0000FF00)
                                 | (((int32_t)hc.response[4] << 16) & 0x00FF0000)
                                 | (((int32_t)hc.response[5] << 24) & 0xFF000000);
    arg_Vehi->usVehiRideTime     = (((int32_t)hc.response[6]      ) & 0x000000FF)
                                 | (((int32_t)hc.response[7] <<  8) & 0x0000FF00)
                                 | (((int32_t)hc.response[8] << 16) & 0x00FF0000)
                                 | (((int32_t)hc.response[9] << 24) & 0xFF000000);
    SENSOR_N_LOG("Vehi Status          [%u]",arg_Vehi->usVehiStatus);
    SENSOR_N_LOG("Vehi Kind            [%u]",arg_Vehi->usVehiKind);
    SENSOR_N_LOG("Vehi DetectTime(10ms)[%u]",arg_Vehi->usVehiDetectTime);
    SENSOR_N_LOG("Vehi RideTime(10ms)  [%u]",arg_Vehi->usVehiRideTime);

    arg_Vehi->usVehiRideCal      = (((int32_t)hc.response[10]      ) & 0x000000FF)
                                 | (((int32_t)hc.response[11] <<  8) & 0x0000FF00)
                                 | (((int32_t)hc.response[12] << 16) & 0x00FF0000)
                                 | (((int32_t)hc.response[13] << 24) & 0xFF000000);
    arg_Vehi->usVehiBodyFat      = (((int32_t)hc.response[14]      ) & 0x000000FF)
                                 | (((int32_t)hc.response[15] <<  8) & 0x0000FF00)
                                 | (((int32_t)hc.response[16] << 16) & 0x00FF0000)
                                 | (((int32_t)hc.response[17] << 24) & 0xFF000000);
    arg_Vehi->usVehiExercise     = (((int32_t)hc.response[18]      ) & 0x000000FF)
                                 | (((int32_t)hc.response[19] <<  8) & 0x0000FF00)
                                 | (((int32_t)hc.response[20] << 16) & 0x00FF0000)
                                 | (((int32_t)hc.response[21] << 24) & 0xFF000000);
    arg_Vehi->usVehiMets         = (((int32_t)hc.response[22]      ) & 0x000000FF);
    SENSOR_N_LOG("Vehi RideCal(kcal)   [%u]",arg_Vehi->usVehiRideCal);
    SENSOR_N_LOG("Vehi BodyFat(100mg)  [%d]",arg_Vehi->usVehiBodyFat);
    SENSOR_N_LOG("Vehi Exercise(Ex)    [%u]",arg_Vehi->usVehiExercise);
    SENSOR_N_LOG("Vehi Mets            [%u]",arg_Vehi->usVehiMets);

    arg_Vehi->usVehiOtherRideTime  = (((int32_t)hc.response[51]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[52] <<  8) & 0x0000FF00)
                                   | (((int32_t)hc.response[53] << 16) & 0x00FF0000)
                                   | (((int32_t)hc.response[54] << 24) & 0xFF000000);
    arg_Vehi->usVehiBicycRideTime  = (((int32_t)hc.response[55]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[56] <<  8) & 0x0000FF00)
                                   | (((int32_t)hc.response[57] << 16) & 0x00FF0000)
                                   | (((int32_t)hc.response[58] << 24) & 0xFF000000);
    arg_Vehi->usVehiTrainRideTime  = (((int32_t)hc.response[59]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[60] <<  8) & 0x0000FF00)
                                   | (((int32_t)hc.response[61] << 16) & 0x00FF0000)
                                   | (((int32_t)hc.response[62] << 24) & 0xFF000000);
    SENSOR_N_LOG("Vehi OtherRideTime(10msec)[%u]",arg_Vehi->usVehiOtherRideTime);
    SENSOR_N_LOG("Vehi BicycRideTime(10msec)[%u]",arg_Vehi->usVehiBicycRideTime);
    SENSOR_N_LOG("Vehi TrainRideTime(10msec)[%u]",arg_Vehi->usVehiTrainRideTime);

err_exit:
    mutex_unlock(&micon_hcres_mutex);

    SENSOR_D_LOG("end");

    return ret;
}

static int32_t dailys_get_pedo_data(struct pedometer *arg_Pedo)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc = {0};

    SENSOR_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DS,
            APP_DS__PD_GetInfo);
    hc.param_data[0] = 0x00;
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 0;
    hc.response_len = 81;
    ret = qleoss3_hostcmd(&hc);
    if(ret != QL_STATUS_OK){
        SENSOR_ERR_LOG("host command error [0x%08x]", hc.command);
        goto err_exit;
    }
    arg_Pedo->usStepCnt   = (((int32_t)hc.response[0]      ) & 0x000000FF)
                          | (((int32_t)hc.response[1] <<  8) & 0x0000FF00)
                          | (((int32_t)hc.response[2] << 16) & 0x00FF0000)
                          | (((int32_t)hc.response[3] << 24) & 0xFF000000);
    arg_Pedo->usWalkTime  = (((int32_t)hc.response[4]      ) & 0x000000FF)
                          | (((int32_t)hc.response[5] <<  8) & 0x0000FF00)
                          | (((int32_t)hc.response[6] << 16) & 0x00FF0000)
                          | (((int32_t)hc.response[7] << 24) & 0xFF000000);
    arg_Pedo->usCal       = (((int32_t)hc.response[8]      ) & 0x000000FF)
                          | (((int32_t)hc.response[9] <<  8) & 0x0000FF00)
                          | (((int32_t)hc.response[10] << 16) & 0x00FF0000)
                          | (((int32_t)hc.response[11] << 24) & 0xFF000000);
    arg_Pedo->usRTState   = (((int32_t)hc.response[12]      ) & 0x000000FF);
    SENSOR_N_LOG("Step Count         [%u]",arg_Pedo->usStepCnt);
    SENSOR_N_LOG("Walking Time(10ms) [%u]",arg_Pedo->usWalkTime);
    SENSOR_N_LOG("Calorie(kcal)      [%u]",arg_Pedo->usCal);
    SENSOR_N_LOG("RealTime State     [%u]",arg_Pedo->usRTState);

    arg_Pedo->usBodyFat   = (((int32_t)hc.response[13]      ) & 0x000000FF)
                          | (((int32_t)hc.response[14] <<  8) & 0x0000FF00)
                          | (((int32_t)hc.response[15] << 16) & 0x00FF0000)
                          | (((int32_t)hc.response[16] << 24) & 0xFF000000);
    arg_Pedo->usExercise  = (((int32_t)hc.response[17]      ) & 0x000000FF);
    arg_Pedo->usMets      = (((int32_t)hc.response[18]      ) & 0x000000FF);
    arg_Pedo->usSpeed     = (((int32_t)hc.response[19]      ) & 0x000000FF)
                          | (((int32_t)hc.response[20] <<  8) & 0x0000FF00);
    SENSOR_N_LOG("Body Fat(100mg)    [%u]",arg_Pedo->usBodyFat);
    SENSOR_N_LOG("Exercise(0.1Ex)    [%u]",arg_Pedo->usExercise);
    SENSOR_N_LOG("Mets               [%d]",arg_Pedo->usMets);
    SENSOR_N_LOG("Speedometer(cm/sec)[%u]",arg_Pedo->usSpeed);

    arg_Pedo->usSyncRunStepCnt  = (((int32_t)hc.response[40]      ) & 0x000000FF)
                                | (((int32_t)hc.response[41] <<  8) & 0x0000FF00)
                                | (((int32_t)hc.response[42] << 16) & 0x00FF0000)
                                | (((int32_t)hc.response[43] << 24) & 0xFF000000);
    arg_Pedo->usSyncRunTime  = (((int32_t)hc.response[44]      ) & 0x000000FF)
                             | (((int32_t)hc.response[45] <<  8) & 0x0000FF00)
                             | (((int32_t)hc.response[46] << 16) & 0x00FF0000)
                             | (((int32_t)hc.response[47] << 24) & 0xFF000000);
    arg_Pedo->usSyncRunCal  = (((int32_t)hc.response[48]      ) & 0x000000FF)
                            | (((int32_t)hc.response[49] <<  8) & 0x0000FF00)
                            | (((int32_t)hc.response[50] << 16) & 0x00FF0000)
                            | (((int32_t)hc.response[51] << 24) & 0xFF000000);
    SENSOR_N_LOG("SyncRunStepCnt     [%u]",arg_Pedo->usSyncRunStepCnt);
    SENSOR_N_LOG("SyncRunTime(10ms)  [%u]",arg_Pedo->usSyncRunTime);
    SENSOR_N_LOG("SyncRunCal(kcal)   [%u]",arg_Pedo->usSyncRunCal);

    arg_Pedo->usRunStatus   = (((int32_t)hc.response[64]      ) & 0x000000FF);
    arg_Pedo->usRunStepCnt  = (((int32_t)hc.response[65]      ) & 0x000000FF)
                            | (((int32_t)hc.response[66] <<  8) & 0x0000FF00)
                            | (((int32_t)hc.response[67] << 16) & 0x00FF0000)
                            | (((int32_t)hc.response[68] << 24) & 0xFF000000);
    arg_Pedo->usRunTime   = (((int32_t)hc.response[69]      ) & 0x000000FF)
                          | (((int32_t)hc.response[70] <<  8) & 0x0000FF00)
                          | (((int32_t)hc.response[71] << 16) & 0x00FF0000)
                          | (((int32_t)hc.response[72] << 24) & 0xFF000000);
    SENSOR_N_LOG("Run Status         [%u]",arg_Pedo->usRunStatus);
    SENSOR_N_LOG("Run StepCnt        [%u]",arg_Pedo->usRunStepCnt);
    SENSOR_N_LOG("Run Time(10ms)     [%u]",arg_Pedo->usRunTime);

    arg_Pedo->usRunCal      = (((int32_t)hc.response[73]      ) & 0x000000FF)
                            | (((int32_t)hc.response[74] <<  8) & 0x0000FF00)
                            | (((int32_t)hc.response[75] << 16) & 0x00FF0000)
                            | (((int32_t)hc.response[76] << 24) & 0xFF000000);
    arg_Pedo->usRunExercise = (((int32_t)hc.response[77]      ) & 0x000000FF)
                            | (((int32_t)hc.response[78] <<  8) & 0x0000FF00)
                            | (((int32_t)hc.response[79] << 16) & 0x00FF0000)
                            | (((int32_t)hc.response[80] << 24) & 0xFF000000);
    SENSOR_N_LOG("Run Calorie(kcal)  [%u]",arg_Pedo->usRunCal);
    SENSOR_N_LOG("Run Exercise(0.1Ex)[%u]",arg_Pedo->usRunExercise);


    //sns_get_pedo_eb_data

    /* Send command */
    hc.command = prepare_host_command(APP, APP_EBUF,
            APP_EBUF__EventBufGetRtInfo);
    hc.param_data[0] = 0x00;
    hc.param_data[1] = 0x00;
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 0;
    hc.response_len = 32;
    ret = qleoss3_hostcmd(&hc);
    if(ret != QL_STATUS_OK){
        SENSOR_ERR_LOG("host command error [0x%08x]", hc.command);
        goto err_exit;
    }

    arg_Pedo->LogTimeStamp3sec     = (((int32_t)hc.response[0]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[1] <<  8) & 0x0000FF00);
    arg_Pedo->PD_Log_DetState      = (((int32_t)hc.response[2]      ) & 0x000000FF);
    arg_Pedo->PD_Log_SectionStep   = (((int32_t)hc.response[3]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[4] <<  8) & 0x0000FF00);
    arg_Pedo->PD_Log_SectionCal    = (((int32_t)hc.response[5]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[6] <<  8) & 0x0000FF00);
    arg_Pedo->RN_Log_SectionStep   = (((int32_t)hc.response[7]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[8] <<  8) & 0x0000FF00);
    arg_Pedo->RN_Log_SectionCal    = (((int32_t)hc.response[9]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[10] <<  8) & 0x0000FF00);
    arg_Pedo->PD_Log_SectionUpH    = (((int32_t)hc.response[11]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[12] <<  8) & 0x0000FF00);
    arg_Pedo->PD_Log_SectionDnH    = (((int32_t)hc.response[13]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[14] <<  8) & 0x0000FF00);
    arg_Pedo->PD_Log_SectionUpDnCal= (((int32_t)hc.response[15]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[16] <<  8) & 0x0000FF00);
    arg_Pedo->VC_Log_DetState      = (((int32_t)hc.response[17]      ) & 0x000000FF);
    arg_Pedo->VC_Log_SectionCal    = (((int32_t)hc.response[18]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[19] <<  8) & 0x0000FF00);
    arg_Pedo->VC_Log_SectionUpH    = (((int32_t)hc.response[20]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[21] <<  8) & 0x0000FF00);
    arg_Pedo->VC_Log_SectionDnH    = (((int32_t)hc.response[22]      ) & 0x000000FF)
                                   | (((int32_t)hc.response[23] <<  8) & 0x0000FF00);
    arg_Pedo->VC_Log_SectionUpDnCal = (((int32_t)hc.response[24]      ) & 0x000000FF)
                                    | (((int32_t)hc.response[25] <<  8) & 0x0000FF00);
    arg_Pedo->PS_SectionPress       = (((int32_t)hc.response[26]      ) & 0x000000FF)
                                    | (((int32_t)hc.response[27] <<  8) & 0x0000FF00);
    arg_Pedo->PS_SectionHeight      = (((int32_t)hc.response[28]      ) & 0x000000FF)
                                    | (((int32_t)hc.response[29] <<  8) & 0x0000FF00);
    arg_Pedo->PS_SectionActiveHeight = (((int32_t)hc.response[30]      ) & 0x000000FF)
                                     | (((int32_t)hc.response[31] <<  8) & 0x0000FF00);

    SENSOR_N_LOG("LogTimeStamp3sec   [%u]",arg_Pedo->LogTimeStamp3sec);
    SENSOR_N_LOG("PD_Log_DetState    [%u]",arg_Pedo->PD_Log_DetState);
    SENSOR_N_LOG("PD_Log_SectionStep [%u]",arg_Pedo->PD_Log_SectionStep);
    SENSOR_N_LOG("PD_Log_SectionCal  [%u]",arg_Pedo->PD_Log_SectionCal);
    SENSOR_N_LOG("RN_Log_SectionStep [%u]",arg_Pedo->RN_Log_SectionStep);
    SENSOR_N_LOG("RN_Log_SectionCal  [%u]",arg_Pedo->RN_Log_SectionCal);
    SENSOR_N_LOG("PD_Log_SectionUpH    [%u]",arg_Pedo->PD_Log_SectionUpH);
    SENSOR_N_LOG("PD_Log_SectionDnH    [%u]",arg_Pedo->PD_Log_SectionDnH);
    SENSOR_N_LOG("PD_Log_SectionUpDnCal[%u]",arg_Pedo->PD_Log_SectionUpDnCal);
    SENSOR_N_LOG("VC_Log_DetState    [%u]",arg_Pedo->VC_Log_DetState);
    SENSOR_N_LOG("VC_Log_SectionCal  [%u]",arg_Pedo->VC_Log_SectionCal);
    SENSOR_N_LOG("VC_Log_SectionUpH    [%u]",arg_Pedo->VC_Log_SectionUpH);
    SENSOR_N_LOG("VC_Log_SectionDnH    [%u]",arg_Pedo->VC_Log_SectionDnH);
    SENSOR_N_LOG("VC_Log_SectionUpDnCal[%u]",arg_Pedo->VC_Log_SectionUpDnCal);
    SENSOR_N_LOG("PS_SectionPress       [%d]",arg_Pedo->PS_SectionPress);
    SENSOR_N_LOG("PS_SectionHeight      [%d]",arg_Pedo->PS_SectionHeight);
    SENSOR_N_LOG("PS_SectionActiveHeight[%d]",arg_Pedo->PS_SectionActiveHeight);
err_exit:
    mutex_unlock(&micon_hcres_mutex);

    SENSOR_D_LOG("end");

    return ret;
}

static int32_t dailys_send_latency_option(enum sensor_e_type type, bool enable)
{
    int32_t    ret = SNS_RC_OK;
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    bool       enable_pedo = now_ctrl_param.input_param.inputs[SENSOR_EXT_PEDO].enable;
    bool       enable_baro = now_ctrl_param.input_param.inputs[SENSOR_EXT_BARO].enable;
    int32_t    period_pedo = now_ctrl_param.input_param.inputs[SENSOR_EXT_PEDO].sampling_period_ms/30;
    int32_t    period_baro = now_ctrl_param.input_param.inputs[SENSOR_EXT_BARO].sampling_period_ms/30;
    int32_t    period = 0x7FFFFFFF;
    struct hc_info hc;

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

    /* Send command */
    hc.command = prepare_host_command(APP, APP_EBUF,
            PP_EBUF__Log_OptSetting);
    hc.param_data[0] = 0x01                              // CTRL_DS_Option_Change
                     | (0x01 << 8)                       // CTRL_DS_INT_MaxDelayEn
                     | (((period >>  0) & 0xFF) << 16)   // CTRL_DS_INT_MaxDelay
                     | (((period >>  8) & 0xFF) << 24);  // CTRL_DS_INT_MaxDelay
    hc.param_data[1] = ((period >> 16) & 0xFF)           // CTRL_DS_INT_MaxDelay
                     | ((period >> 24) & 0xFF << 8)      // CTRL_DS_INT_MaxDelay
                     | (0x00 << 16)                      // padding
                     | (0x00 << 24);                     // padding
    hc.param_data[2] = 0x00;
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 6;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

    SENSOR_N_LOG("end ret[%d]", ret);

    return ret;
}

static int32_t dailys_send_param(bool enable, const struct pedo_param pp)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;
    uint8_t    param_pressure;
    struct hc_info hc;

    SENSOR_D_LOG("start enable[%d]", enable);
    if (enable) {
        param = HC_VALID;
        if (g_pres_available) {
            param_pressure = HC_VALID;
        } else {
            param_pressure = HC_INVALID;
        }
    } else {
        param = HC_INVALID;
        param_pressure = HC_INVALID;
    }

    /* Send command */
    hc.command = prepare_host_command(APP, APP_DS,
            APP_DS__DS_OnOff);
    hc.param_data[0] = 0x01                                            // CTRL_DS_Main_Change
                     | (param << 8)                                    // CTRL_DS_En
                     | ((pp.step_wide    & 0xFF) << 16)      // CTRL_PD_StepPitch
                     | ((pp.weight       & 0xFF) << 24);     // CTRL_PD_Weight
    hc.param_data[1] = ((pp.weight       & 0xFF00) >> 8)    // CTRL_PD_Weight
                     | ((pp.vehi_type    & 0xFF) << 8)       // CTRL_CY_Kind
                     | (param << 16)                                   // CTRL_PD_Step_INT_En
                     | (param << 24);                                  // CTRL_VC_Dtct_INT_En
    hc.param_data[2] = param_pressure           // CTRL_PS_GradChange_INT_En
                     | (0x01 << 8)              // CTRL_VC_Est_En
                     | (0x00 << 16)             // CTRL_PD_DetIntFreq
                     | (param_pressure << 24);  // CTRL_DS_PS_En
    hc.param_data[3] = 0x00;
    hc.param_data[4] = 0x00;
    hc.param_data[5] = 0x00;
    hc.param_data[6] = 0x00;
    hc.param_data[7] = 0x00;
    hc.param_data_len = 12;
    hc.response_len = 0;
    ret = qleoss3_hostcmd(&hc);
    SENSOR_D_LOG("end ret[%d]", ret);

    return ret;
}

static int32_t dailys_enable(bool enable, const struct pedo_param pp)
{
    int32_t ret = SNS_RC_OK;
    uint8_t param;
    struct hc_info hc;

    SENSOR_D_LOG("start enable[%d]", enable);
    param = enable ? HC_VALID : HC_INVALID;

    if(enable) {
        ret = dailys_send_param(enable, pp);
        if (SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end sns_dailys_send_param err ret[%d]",ret);
            return SNS_RC_ERR;
        }

        ret = dailys_send_latency_option(SENSOR_EXT_PEDO, enable);
        if (SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end sns_dailys_send_latency_option err ret[%d]",ret);
            return SNS_RC_ERR;
        }
        if(g_pres_available) {
            /* Send command */
            hc.command = prepare_host_command(APP, APP_EBUF,
                    APP_EBUF__Log_HeightSetting);
            hc.param_data[0] = 0x01                 // CTRL_Log_H_Param_Change
                             | (0x9A << 8)          // CTRL_PS_LogH_MinFreq
                             | (0x17 << 16)         // CTRL_PS_LogH_MinFreq
                             | (0x00 << 24);        // CTRL_PS_LogH_MinFreq
            hc.param_data[1] = 0x00                 // CTRL_PS_LogH_MinFreq
                             | (0x46 << 8)          // CTRL_PS_LogH_ChangeTh
                             | (0x9A << 16)         // CTRL_PS_Log_ActH_MinFreq
                             | (0x17 << 24);        // CTRL_PS_Log_ActH_MinFreq
            hc.param_data[2] = 0x00                 // CTRL_PS_Log_ActH_MinFreq
                             | (0x00 << 8)          // CTRL_PS_Log_ActH_MinFreq
                             | (0x46 << 16)         // CTRL_PS_Log_ActH_ChangeTh
                             | (0x00 << 24);        // padding
            hc.param_data[3] = 0x00;
            hc.param_data[4] = 0x00;
            hc.param_data[5] = 0x00;
            hc.param_data[6] = 0x00;
            hc.param_data[7] = 0x00;
            hc.param_data_len = 11;
            hc.response_len = 0;
            qleoss3_hostcmd(&hc);
        }

        /* Send command */
        hc.command = prepare_host_command(APP, APP_EBUF,
                APP_EBUF__Log_Setting);
        hc.param_data[0] = 0x01                                             // CTRL_Log_DS_Main_Change
                         | (0x01 << 8)                                      // CTRL_Log_DS_En
                         | (0x00 << 16)                                     // CTRL_Log_PD_Height_En
                         | (0x00 << 24);                                    // CTRL_Log_VC_Height_En
        hc.param_data[1] = 0x00                                             // CTRL_Log_P_ChangeEn
                         | (0x00 << 8)                                      // CTRL_Log_H_ChangeEn
                         | (g_pres_available ? (0x01 << 16):(0x00 << 16))   // CTRL_Log_ActH_ChangeEn
                         | (0x01 << 24);                                    // CTRL_Log_ThReach_INT_En
        hc.param_data[2] = 0x01                                             // CTRL_Log_Full_INT_En
                         | (0x01 << 8)                                      // CTRL_Log_Update_INT_En
                         | (0x00 << 16)                                     // CTRL_Log_Update_INT_Margin
                         | (0x02 << 24);                                    // CTRL_Log_Update_INT_Margin
        hc.param_data[3] = 0x00                                             // CTRL_Log_Update_INT_Margin
                         | (0x00 << 8)                                      // CTRL_Log_Update_INT_Margin
                         | (0x00 << 16)                                     // padding
                         | (0x00 << 24);                                    // padding
        hc.param_data[4] = 0x00;
        hc.param_data[5] = 0x00;
        hc.param_data[6] = 0x00;
        hc.param_data[7] = 0x00;
        hc.param_data_len = 14;
        hc.response_len = 0;
        qleoss3_hostcmd(&hc);

        clear_evbuf_markoff_base_timestamps();
    } else {
        /* Send command */
        hc.command = prepare_host_command(APP, APP_EBUF,
                APP_EBUF__Log_Setting);
        hc.param_data[0] = 0x01                // CTRL_Log_DS_Main_Change
                         | (0x00 << 8)         // CTRL_Log_DS_En
                         | (0x00 << 16)        // CTRL_Log_PD_Height_En
                         | (0x00 << 24);       // CTRL_Log_VC_Height_En
        hc.param_data[1] = 0x00                // CTRL_Log_P_ChangeEn
                         | (0x00 << 8)         // CTRL_Log_H_ChangeEn
                         | (0x00 << 16)        // CTRL_Log_ActH_ChangeEn
                         | (0x01 << 24);       // CTRL_Log_ThReach_INT_En
        hc.param_data[2] = 0x01                // CTRL_Log_Full_INT_En
                         | (0x01 << 8)         // CTRL_Log_Update_INT_En
                         | (0x00 << 16)        // CTRL_Log_Update_INT_Margin
                         | (0x02 << 24);       // CTRL_Log_Update_INT_Margin
        hc.param_data[3] = 0x00                // CTRL_Log_Update_INT_Margin
                         | (0x00 << 8)         // CTRL_Log_Update_INT_Margin
                         | (0x00 << 16)        // padding
                         | (0x00 << 24);       // padding
        hc.param_data[4] = 0x00;
        hc.param_data[5] = 0x00;
        hc.param_data[6] = 0x00;
        hc.param_data[7] = 0x00;
        hc.param_data_len = 14;
        hc.response_len = 0;
        qleoss3_hostcmd(&hc);

        ret = dailys_send_param(enable, pp);
        if (SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end dailys_send_param err ret[%d]",ret);
            return SNS_RC_ERR;
        }
        msleep(35);

    }
    SENSOR_D_LOG("end ret[%d]", ret);
    return ret;
}

static int32_t micon_app_clear(int32_t clr_req)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc = {0};
	SENSOR_D_LOG("start");
    SENSOR_D_LOG("clear_request:0x%08x",clr_req);
    hc.command = prepare_host_command(APP, APP_ALL,
            APP_ALL__AppInfoClear);
    hc.param_data[0] = 0x01                                                           // CTRL_DS_Clear_Change
                     | (((clr_req & DAILYS_CLEAR_PEDO_DATA)     ? 0x01:0x00) << 8)    // CTRL_PD_MainRegister_Clear
                     | (((clr_req & DAILYS_CLEAR_PEDO_STATE)    ? 0x01:0x00) << 16)   // CTRL_PD_State_Clear
                     | (((clr_req & DAILYS_CLEAR_VEHI_DATA)     ? 0x01:0x00) << 24);  // CTRL_VC_MainRegister_Clear
    hc.param_data[1] = (((clr_req & DAILYS_CLEAR_VEHI_STATE)    ? 0x01:0x00) << 0)    // CTRL_VC_State_Clear
                     | (((clr_req & DAILYS_CLEAR_WIFI_STATE)    ? 0x01:0x00) << 8)    // CTRL_WF_State_Clear
                     | (((clr_req & DAILYS_CLEAR_HEIGHT_STATE)  ? 0x01:0x00) << 16)   // CTRL_PS_ActiveHeightAggr_Clear
                     | (((clr_req & DAILYS_CLEAR_HEIGHT_ALL)    ? 0x01:0x00) << 24);  // CTRL_PS_ActiveHeightAll_Clear
    hc.param_data[2] = (((clr_req & DAILYS_CLEAR_RT_STATE)      ? 0x01:0x00) << 0)    // CTRL_RT_State_Clear
                     | (((clr_req & DAILYS_CLEAR_SHOCK_STATE)   ? 0x01:0x00) << 8)    // CTRL_SD_State_Clear
                     | (((clr_req & DAILYS_CLEAR_STOP_STATE)    ? 0x01:0x00) << 16)   // CTRL_SP_BW_State_Clear
                     | (((clr_req & DAILYS_CLEAR_OUT_STATE)     ? 0x01:0x00) << 24);  // CTRL_BU_State_Clear
    hc.param_data[3] = (((clr_req & DAILYS_CLEAR_TRAIN_STATE)   ? 0x01:0x00) << 0)    // CTRL_TR_State_Clear
                     | (((clr_req & DAILYS_CLEAR_MOTION_STATE)  ? 0x01:0x00) << 8)    // CTRL_AR_SigMot_State_Clear
                     | (((clr_req & DAILYS_CLEAR_STEP_COUNTER)  ? 0x01:0x00) << 16)   // CTRL_AR_StepCnt_MainRegister_Clear
                     | (((clr_req & DAILYS_CLEAR_STEP_TIMESTMP) ? 0x01:0x00) << 24);  // CTRL_AR_Step_TimeStamp_Clear
    hc.param_data[4] = (((clr_req & DAILYS_CLEAR_BATCH_TIMER)   ? 0x01:0x00) << 0)    // CTRL_AR_Batch_Timer_Clear
                     | (((clr_req & DAILYS_CLEAR_VH_STATE)      ? 0x01:0x00) << 8)    // CTRL_VH_State_Clear
                     | (((clr_req & DAILYS_CLEAR_EB_ALL)        ? 0x01:0x00) << 16)   // CTRL_Log_All_Clear
                     | (((clr_req & DAILYS_CLEAR_DAILYS_DAY)    ? 0x01:0x00) << 24);  // CTRL_Log_1day_Clear
    hc.param_data_len = 20;
    hc.response_len = 0;
    SENSOR_D_LOG("hc.param_data:0x%08x",hc.param_data[0]);
    SENSOR_D_LOG("hc.param_data:0x%08x",hc.param_data[1]);
    SENSOR_D_LOG("hc.param_data:0x%08x",hc.param_data[2]);
    SENSOR_D_LOG("hc.param_data:0x%08x",hc.param_data[3]);
    SENSOR_D_LOG("hc.param_data:0x%08x",hc.param_data[4]);
    ret = qleoss3_hostcmd(&hc);
    if(clr_req & DAILYS_CLEAR_DAILYS_DAY){
        eventbuf_cleartime = GET_CURRENT_TIMESTAMP_NS();
    }

    SENSOR_D_LOG("end ret[%d]", ret);
    return ret;
}

static void SetBaroEventBufferParam(const uint8_t *prm)
{
	struct hc_info hc = {0};
	SENSOR_D_LOG("start");
    hc.command = prepare_host_command(APP, APP_BR,
            APP_BR__BR_LogPrsSetting);
    hc.param_data[0] =  prm[0]                // CTRL_Log_PrsParam_Change
                     | (prm[1] << 8)         // CTRL_PS_LogP_MinFreq
                     | (prm[2] << 16)        // CTRL_PS_LogP_MinFreq
                     | (prm[3] << 24);       // CTRL_PS_LogP_MinFreq
    hc.param_data[1] =  prm[4]                // CTRL_PS_LogP_MinFreq
                     | (prm[5] << 8)         // CTRL_PS_LogP_ChangeTh
                     | (0x00 << 16)        // padding
                     | (0x00 << 24);       // padding
    hc.param_data_len = 6;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end");
}

static void SetDailyStepEventBufferSetting(const uint8_t *prm)
{
	struct hc_info hc = {0};
	SENSOR_D_LOG("start");
        hc.command = prepare_host_command(APP, APP_EBUF,
                APP_EBUF__Log_Setting);
        hc.param_data[0] =  prm[0]               // CTRL_Log_DS_Main_Change
                         | (prm[1] << 8)         // CTRL_Log_DS_En
                         | (prm[2] << 16)        // CTRL_Log_PD_Height_En
                         | (prm[3] << 24);       // CTRL_Log_VC_Height_En
        hc.param_data[1] =  prm[4]               // CTRL_Log_P_ChangeEn
                         | (prm[5] << 8)         // CTRL_Log_H_ChangeEn
                         | (prm[6] << 16)        // CTRL_Log_ActH_ChangeEn
                         | (prm[7] << 24);       // CTRL_Log_ThReach_INT_En
        hc.param_data[2] =  prm[8]               // CTRL_Log_Full_INT_En
                         | (prm[9] << 8)         // CTRL_Log_Update_INT_En
                         | (prm[10] << 16)       // CTRL_Log_Update_INT_Margin
                         | (prm[11] << 24);      // CTRL_Log_Update_INT_Margin
        hc.param_data[3] =  prm[12]              // CTRL_Log_Update_INT_Margin
                         | (prm[13] << 8)        // CTRL_Log_Update_INT_Margin
                         | (0x00 << 16)          // padding
                         | (0x00 << 24);         // padding
        hc.param_data_len = 14;
        hc.response_len = 0;
        qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end");
}

static void SetDailyStepEventBufferHeightParam(const uint8_t *prm)
{
	struct hc_info hc = {0};
	SENSOR_D_LOG("start");
    hc.command = prepare_host_command(APP, APP_EBUF,
            APP_EBUF__Log_HeightSetting);
    hc.param_data[0] =  prm[0]                  // CTRL_Log_H_Param_Change
                     | (prm[1] << 8)            // CTRL_PS_LogH_MinFreq
                     | (prm[2] << 16)           // CTRL_PS_LogH_MinFreq
                     | (prm[3] << 24);          // CTRL_PS_LogH_MinFreq
    hc.param_data[1] =  prm[4]                  // CTRL_PS_LogH_MinFreq
                     | (prm[5] << 8)            // CTRL_PS_LogH_ChangeTh
                     | (prm[6] << 16)           // CTRL_PS_Log_ActH_MinFreq
                     | (prm[7] << 24);          // CTRL_PS_Log_ActH_MinFreq
    hc.param_data[2] =  prm[8]                  // CTRL_PS_Log_ActH_MinFreq
                     | (prm[9] << 8)            // CTRL_PS_Log_ActH_MinFreq
                     | (prm[10] << 16)          // CTRL_PS_Log_ActH_ChangeTh
                     | (0x00 << 24);            // padding
    hc.param_data_len = 11;
    hc.response_len = 0;
    qleoss3_hostcmd(&hc);
	SENSOR_D_LOG("end");
}

static void InitSetDailyStepBaroSetting(void)
{
    uint8_t init_ds_eb_height_param[14] = { 0x01, 0x70, 0x17, 0x00,
                                            0x00, 0x46, 0x70, 0x17,
                                            0x00, 0x00, 0x46, 0x00,
                                            0x00, 0x00};
    uint8_t init_ds_eb_setting[14]      = { 0x01, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x01, 0x01,
                                            0x01, 0x01, 0x00, 0x02,
                                            0x00, 0x00};
    uint8_t init_baro_eb_setting[14]    = { 0x01, 0x30, 0x75, 0x00,
                                            0x00, 0x0E, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00};
	SENSOR_D_LOG("start");
    //Set DailyStep Event Buffer Param
    SetDailyStepEventBufferHeightParam(init_ds_eb_height_param);

    //Set DailyStep Event Buffer Setting
    SetDailyStepEventBufferSetting(init_ds_eb_setting);

    //Set Baro Event Buffer Param
    SetBaroEventBufferParam(init_baro_eb_setting);

	SENSOR_D_LOG("end");
}
static void M4AppTask_Buf_Size_FFE_to_M4(const uint16_t setBufSize)
{
	struct hc_info hc;

	SENSOR_D_LOG("start");
	memset(&hc, 0, sizeof(hc));
	hc.command = prepare_host_command(M4_APP_TASK,
			TASK, M4AT_SET_BUF_SIZE_FFE_TO_M4);
	hc.param_data_len = 2;
	hc.param_data[0] = (uint32_t)(setBufSize & 0x0000FFFF);
	hc.response_len = 0;
    SENSOR_D_LOG("M4AppTask_Buf_Size_FFE_to_M4 setBufSize[0x%08x]", hc.param_data[0]);

	qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end");
}

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
static void FFEFusion_Buf_Size_FFE_to_M4(const uint16_t setBufSize)
{
	struct hc_info hc;

	SENSOR_D_LOG("start");
	memset(&hc, 0, sizeof(hc));
	hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
			TASK, ASC_SET_BUF_SIZE_FFE_TO_M4);
	hc.param_data_len = 2;
	hc.param_data[0] = (uint32_t)(setBufSize & 0x0000FFFF);
	hc.response_len = 0;
    SENSOR_D_LOG("FFEFusion_Buf_Size_FFE_to_M4 setBufSize[0x%08x]", hc.param_data[0]);

	qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end");
}

static void FFEFusion_set_init_wait_time(const uint16_t iwt_ms)
{
	struct hc_info hc;

	SENSOR_D_LOG("start");
	memset(&hc, 0, sizeof(hc));
	hc.command = prepare_host_command(ANDROID_SENSOR_TASK_CONTROL,
			TASK, ASC_SET_FFE_FUSION_INIT_WAIT_TIME);
	hc.param_data_len = 2;
	hc.param_data[0] = (uint32_t)iwt_ms;
	hc.response_len = 0;
	SENSOR_D_LOG("FFE FusionTask InitWaitTime[0x%08x]", hc.param_data[0]);

	qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end");
}
#endif

static void FFEAppTask_set_init_wait_time(const uint16_t iwt_ms)
{
	struct hc_info hc;

	SENSOR_D_LOG("start");
	memset(&hc, 0, sizeof(hc));
	hc.command = prepare_host_command(FFE_APP_TASK,
			TASK, FAT_SET_FFE_INIT_WAIT_TIME);
	hc.param_data_len = 2;
	hc.param_data[0] = (uint32_t)iwt_ms;
	hc.response_len = 0;
	SENSOR_D_LOG("FFE AppTask InitWaitTime[0x%08x]", hc.param_data[0]);

	qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end");
}

static void get_Mag_SoftIronVal(int16_t *mat)
{
    struct hc_info hc = {0};
    SENSOR_D_LOG("Start");
    hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
            MAGNETOMETER, SPC_GET_SOFT_IRON_CAL_VALUE);
    hc.response_len = 18;

    mutex_lock(&micon_hcres_mutex);
    qleoss3_hostcmd(&hc);
    mutex_unlock(&micon_hcres_mutex);
    mat[0] = (int16_t)((hc.response[0]   | (hc.response[1] << 8 )));
    mat[1] = (int16_t)((hc.response[2]   | (hc.response[3] << 8 )));
    mat[2] = (int16_t)((hc.response[4]   | (hc.response[5] << 8 )));
    mat[3] = (int16_t)((hc.response[6]   | (hc.response[7] << 8 )));
    mat[4] = (int16_t)((hc.response[8]   | (hc.response[9] << 8 )));
    mat[5] = (int16_t)((hc.response[10]  | (hc.response[11] << 8 )));
    mat[6] = (int16_t)((hc.response[12]  | (hc.response[13] << 8 )));
    mat[7] = (int16_t)((hc.response[14]  | (hc.response[15] << 8 )));
    mat[8] = (int16_t)((hc.response[16]  | (hc.response[17] << 8 )));
    SENSOR_D_LOG("sMat_00_01[0x%08x], sMat_02_10[0x%08x], sMat_11_12[0x%08x], sMat_20_21[0x%08x], sMat_22[0x%08x]",
                    hc.response[0],hc.response[1],hc.response[2],hc.response[3],hc.response[4]);
    SENSOR_D_LOG("End");
}

static void set_Mag_SoftIronVal(int16_t *mat)
{
	struct hc_info hc = {0};
	uint16_t static_mat[3][3]   = { {mat[0], mat[1], mat[2]},
                                    {mat[3], mat[4], mat[5]},
                                    {mat[6], mat[7], mat[8]} };
	SENSOR_D_LOG("Start");

	hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
			MAGNETOMETER, SPC_SET_SOFT_IRON_CAL_VALUE);
	hc.param_data_len = 18;
	hc.param_data[0] = (uint32_t)(static_mat[0][0] | (static_mat[0][1] << 16));
	hc.param_data[1] = (uint32_t)(static_mat[0][2] | (static_mat[1][0] << 16));
	hc.param_data[2] = (uint32_t)(static_mat[1][1] | (static_mat[1][2] << 16));
	hc.param_data[3] = (uint32_t)(static_mat[2][0] | (static_mat[2][1] << 16));
	hc.param_data[4] = (uint32_t)static_mat[2][2];
	hc.response_len = 0;
	SENSOR_D_LOG("sMat_00_01[0x%08x], sMat_02_10[0x%08x], sMat_11_12[0x%08x], sMat_20_21[0x%08x], sMat_22[0x%08x]",
					hc.param_data[0],hc.param_data[1],hc.param_data[2],hc.param_data[3],hc.param_data[4]);

	qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("End");
}

static int32_t set_Enable_LPFilter(const uint8_t enable, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_ENABLE_LPF);
        hc.param_data_len = 1;
        hc.param_data[0] = enable;
        hc.response_len = 0;
        SENSOR_D_LOG("SensorType[0x%02x], enable[0x%08x]", sensortype, hc.param_data[0]);
        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }

    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_Enable_LPFilter(void)
{
	int32_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	ret = set_Enable_LPFilter(DISABLE, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer Enable LPF[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	ret = set_Enable_LPFilter(ENABLE, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer Enable LPF[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	ret = set_Enable_LPFilter(DISABLE, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Enable LPF[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
	ret = set_Enable_LPFilter(DISABLE, BAROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Pressure Enable LPF[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */

exit:
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static int32_t set_LPF_Coeff(const int16_t coeff, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_SET_LPF_COEFFICIENT);
        hc.param_data_len = 2;
        hc.param_data[0] = coeff;
        hc.response_len = 0;
        SENSOR_D_LOG("SensorType[0x%02x], coefficeint[0x%08x]", sensortype, hc.param_data[0]);

        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }
    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_LPF_Coeff(void)
{
	int32_t ret = SNS_RC_OK;
	int16_t coeff = 0;
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	coeff = 0x00;
	ret = set_LPF_Coeff(coeff, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer LPF Coefficient[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	coeff = 0x02;
	ret = set_LPF_Coeff(coeff, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer LPF Coefficient[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	coeff = 0x00;
	ret = set_LPF_Coeff(coeff, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope LPF Coefficient[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
	coeff = 0x00;
	ret = set_LPF_Coeff(coeff, BAROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Pressure LPF Coefficient[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */


exit:
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static int32_t set_enable_updt_calval(const uint8_t enable, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_ENABLE_UPDATE_OF_CAL_VALUE);
        hc.param_data_len = 1;
        hc.param_data[0] = enable;
        hc.response_len = 0;
        SENSOR_D_LOG("SensorType[0x%02x], enable[0x%08x]", sensortype, hc.param_data[0]);

        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }
    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_Enable_Update_Of_CalVal(void)
{
	int32_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	ret = set_enable_updt_calval(ENABLE, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer Enable Update Of CalVal[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	ret = set_enable_updt_calval(ENABLE, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer Enable Update Of CalVal[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	ret = set_enable_updt_calval(ENABLE, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Enable Update Of CalVal[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

exit:
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static int32_t set_enable_calib(const uint8_t enable, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_ENABLE_CALIBRATION);
        hc.param_data_len = 1;
        hc.param_data[0] = enable;
        hc.response_len = 0;
        SENSOR_D_LOG("SensorType[0x%02x], enable[0x%08x]", sensortype, hc.param_data[0]);

        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }

    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_Enable_Cal(void)
{
	int32_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	ret = set_enable_calib(ENABLE, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer Enable Calibration[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	ret = set_enable_calib(ENABLE, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer Enable Calibration[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	ret = set_enable_calib(ENABLE, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Enable Calibration[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
	ret = set_enable_calib(DISABLE, BAROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Pressure Enable Calibration[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */

exit:
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static int32_t set_enable_cal_interrupt(const uint8_t enable, const uint8_t sensortype)
{
	int32_t ret = SNS_RC_OK;
	struct hc_info hc;

	SENSOR_D_LOG("start");
	if(sensortype >= PHYSICAL_SENS_MAX){
		SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
		return -EINVAL;
	}
	memset(&hc, 0, sizeof(hc));
	hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
			sensortype, SPC_ENABLE_CAL_INTERRUPT);
	hc.param_data_len = 1;
	hc.param_data[0] = enable;
	hc.response_len = 0;
	SENSOR_D_LOG("SensorType[0x%02x], enable[0x%08x]", sensortype, hc.param_data[0]);

	qleoss3_hostcmd(&hc);

	SENSOR_D_LOG("end[%d]", ret);

	return ret;
}

static int32_t set_all_Enable_Cal_Interrupt(void)
{
	int32_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	ret = set_enable_cal_interrupt(ENABLE, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer Enable Calibration Interrupt[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	ret = set_enable_cal_interrupt(ENABLE, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer Enable Calibratin Interrupt[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	ret = set_enable_cal_interrupt(ENABLE, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Enable Calibratin Interrupt[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

exit:
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static int32_t set_calib_val(const int16_t *c_val, const uint8_t accuracy, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_SET_CALIBRATION_VALUE);
        if(sensortype == BAROMETER){
            hc.param_data_len = 2;
            hc.param_data[0] = (uint32_t)c_val[0];
            hc.response_len = 0;
            SENSOR_D_LOG("SensorType[0x%02x], cal_val[0x%08x]",
                        sensortype, hc.param_data[0]);
        }else{
            hc.param_data_len = 7;
            hc.param_data[0] = (uint32_t)( c_val[0] | c_val[1] << 16);
            hc.param_data[1] = (uint32_t)( c_val[2] | accuracy << 16);
            hc.response_len = 0;
            SENSOR_D_LOG("SensorType[0x%02x], cal_val(PRM00-03)[0x%08x], cal_val_AND_accuracy(PRM04-07)[0x%08x]",
                        sensortype, hc.param_data[0], hc.param_data[1]);
        }
        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }
    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_calib_value(void)
{
	int32_t ret = SNS_RC_OK;
	int16_t c_val[3] = {0, 0, 0};
	uint8_t accuracy = 0;
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    struct acceleration acc_ofs = {0};
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    struct geomagnetic mag_ofs = {0};
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    struct gyroscope gyro_ofs = {0};
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    struct pressure press_ofs = {0};
#endif
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    sns_load_offset_value(SENSOR_ACC, &acc_ofs);
    c_val[0] = (int16_t)acc_ofs.nX;
    c_val[1] = (int16_t)acc_ofs.nY;
    c_val[2] = (int16_t)acc_ofs.nZ;
	accuracy = 0x03;
	ret = set_calib_val(c_val, accuracy, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer Calibration Value[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    sns_load_offset_value(SENSOR_MAG, &mag_ofs);
	c_val[0] = (int16_t)mag_ofs.x;
	c_val[1] = (int16_t)mag_ofs.y;
	c_val[2] = (int16_t)mag_ofs.z;
	accuracy = mag_ofs.accuracy;
	ret = set_calib_val(c_val, accuracy, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer Calibration Value[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    sns_load_offset_value(SENSOR_GYRO, &gyro_ofs);
	c_val[0] = (int16_t)gyro_ofs.x;
	c_val[1] = (int16_t)gyro_ofs.y;
	c_val[2] = (int16_t)gyro_ofs.z;
	accuracy = 0x03;
	ret = set_calib_val(c_val, accuracy, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Calibration Value[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    sns_load_offset_value(SENSOR_PRESSURE, &press_ofs);
	c_val[0] = (int16_t)press_ofs.pressure;
	accuracy = 0x00;
	ret = set_calib_val(c_val, accuracy, BAROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Pressure Calibration Value[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */

exit:
	SENSOR_D_LOG("end[%d]", ret);
	return ret;
}

static int32_t set_init_wait_time(const uint16_t iwt_ms, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_SET_INITIAL_WAIT_TIME);
        hc.param_data_len = 2;
        hc.param_data[0] = (uint32_t)iwt_ms;
        hc.response_len = 0;
        SENSOR_D_LOG("SensorType[0x%02x], init wait time[0x%08x]", sensortype, hc.param_data[0]);
        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }

    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_IWT(void)
{
	int32_t ret = SNS_RC_OK;
	uint16_t iwt_ms = 0;
	SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	iwt_ms = 0x0000;
	ret = set_init_wait_time(iwt_ms, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer Initial Wait Time[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	iwt_ms = 0x0000;
	ret = set_init_wait_time(iwt_ms, MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer Initial Wait Time[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	iwt_ms = 0x0050;
	ret = set_init_wait_time(iwt_ms, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Initial Wait Time[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
	iwt_ms = 0x0000;
	ret = set_init_wait_time(iwt_ms, BAROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope Initial Wait Time[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */

exit:
	SENSOR_D_LOG("end[%d]", ret);

	return ret;
}

static int32_t set_axis(const uint8_t axis, const enum ql_sensor_type sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }
    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));
        hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
                sensortype, SPC_CHANGE_AXIS_DIRECTION);
        hc.param_data_len = 1;
        hc.param_data[0] = axis;
        hc.response_len = 0;
        SENSOR_D_LOG("SensorType[0x%02x], axis[0x%08x]", sensortype, hc.param_data[0]);
        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
    }

    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_axis(void)
{
	int32_t ret = SNS_RC_OK;
	int8_t axis_dir[3] = {0};

	SENSOR_D_LOG("start");
	axis_dir[ACCELEROMETER]	= g_acc_conv_axis;
	axis_dir[MAGNETOMETER]	= g_mag_conv_axis;
	axis_dir[GYROSCOPE]	= g_gyro_conv_axis;
	SENSOR_D_LOG("axis dir ACC[0x%02x], MAG[0x%02x], GYRO[0x%02x]",
					axis_dir[ACCELEROMETER], axis_dir[MAGNETOMETER], axis_dir[GYROSCOPE]);
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	ret = set_axis(axis_dir[ACCELEROMETER], ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Accelerometer axis[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
	ret = set_axis(axis_dir[MAGNETOMETER], MAGNETOMETER);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Magnetometer axis[%d]", ret);
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_MAGNETOMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	ret = set_axis(axis_dir[GYROSCOPE], GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_D_LOG("Failed set Gyroscope axis[%d]", ret);
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */

exit:
	SENSOR_D_LOG("end[%d]", ret);

	return ret;
}

static int32_t InitSetting(void)
{
	int32_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");

	//Setaxis
	ret = set_all_axis();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_axis[%d]", ret);
		return ret;
	}
	//SetInitialWaitTime
	ret = set_all_IWT();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_IWT[%d]", ret);
		return ret;
	}
	//Set calib value
	ret = set_all_calib_value();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_calib_value[%d]", ret);
		return ret;
	}
	//Set EnableCALInt
	ret = set_all_Enable_Cal_Interrupt();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_Enable_Cal_Interrupt[%d]", ret);
		return ret;
	}
	//Set EnableCAL
	ret = set_all_Enable_Cal();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_Enable_Cal[%d]", ret);
		return ret;
	}
	//Set EnableUpdateofCalVal
	ret = set_all_Enable_Update_Of_CalVal();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_Enable_Update_Of_CalVal[%d]", ret);
		return ret;
	}
	//Set LPF coeff
	ret = set_all_LPF_Coeff();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_LPF_Coeff[%d]", ret);
		return ret;
	}
	//Enable LPFilter
	ret = set_all_Enable_LPFilter();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Failed set_all_Enable_LPFilter[%d]", ret);
		return ret;
	}
	//Set soft iron calib val
	set_Mag_SoftIronVal(g_mag_static_matrix);

	//FFE AppTask Set Initial wait time
	FFEAppTask_set_init_wait_time(0);

#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
	//FFE Fusion Set Initial wait time
	FFEFusion_set_init_wait_time(0);

	//Set Fusion Buffer Size (224 ByteSet)
	FFEFusion_Buf_Size_FFE_to_M4(0x00E0);
#endif

	//Set App Buffer Size(256 ByteSet)
	M4AppTask_Buf_Size_FFE_to_M4(0x0100);

    //DailyStep and Baro Settings
    InitSetDailyStepBaroSetting();
	SENSOR_D_LOG("end - SNS_RC_OK");
	return SNS_RC_OK;
}

static int32_t set_ts_nrfilter_param(struct nr_filt_prm prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    ts_nrfilt_param.ts_th_us           = prm.ts_th_us;
    ts_nrfilt_param.ts_tol_us          = prm.ts_tol_us;
    ts_nrfilt_param.ts_rel_th          = prm.ts_rel_th;
    ts_nrfilt_param.ts_base_filt_coff  = prm.ts_base_filt_coff;
    ts_nrfilt_param.ts_base_filt_acoff = prm.ts_base_filt_acoff;
    ts_nrfilt_param.ts_filt_coff       = prm.ts_filt_coff;
    ts_nrfilt_param.ts_ng_th           = prm.ts_ng_th;
    ts_nrfilt_param.ts_unaccept_num    = prm.ts_unaccept_num;
    ts_nrfilt_param.ts_reserved1       = 0;
    ts_nrfilt_param.ts_reserved2       = 0;
    SENSOR_D_LOG("end");
    return ret;

}
static int32_t get_ts_nrfilter_param(struct nr_filt_prm* prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    prm->ts_th_us           = ts_nrfilt_param.ts_th_us;
    prm->ts_tol_us          = ts_nrfilt_param.ts_tol_us;
    prm->ts_rel_th          = ts_nrfilt_param.ts_rel_th;
    prm->ts_base_filt_coff  = ts_nrfilt_param.ts_base_filt_coff;
    prm->ts_base_filt_acoff = ts_nrfilt_param.ts_base_filt_acoff;
    prm->ts_filt_coff       = ts_nrfilt_param.ts_filt_coff;
    prm->ts_ng_th           = ts_nrfilt_param.ts_ng_th;
    prm->ts_unaccept_num    = ts_nrfilt_param.ts_unaccept_num;
    prm->ts_reserved1       = ts_nrfilt_param.ts_reserved1;
    prm->ts_reserved2       = ts_nrfilt_param.ts_reserved2;
    SENSOR_D_LOG("end");
    return ret;
}


static int32_t acc_set_auto_cal_offset_internal(struct acceleration offsets)
{
    int32_t ret = SNS_RC_OK;
    int16_t c_val[3] = {0};
    uint8_t accuracy;

    SENSOR_D_LOG("start");

    mutex_lock(&acc_auto_cal_mutex);
    c_val[0] = (int16_t)offsets.nX;
    c_val[1] = (int16_t)offsets.nY;
    c_val[2] = (int16_t)offsets.nZ;
    accuracy = 3;
    mutex_unlock(&acc_auto_cal_mutex);
    ret = set_calib_val(c_val, accuracy, ACCELEROMETER);
    SENSOR_D_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t acc_set_offsets(int32_t* offsets)
{
	int32_t ret = SNS_RC_OK;
    int16_t c_val[3] = {0};
    uint8_t accuracy = 3;

    SENSOR_D_LOG("start");

    c_val[0] = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[0]);
    //atomic_set(&g_nCalX, temp);
    SENSOR_D_LOG("set offset X[%d]",c_val[0]);

    c_val[1] = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[1]);
    //atomic_set(&g_nCalY, temp);
    SENSOR_D_LOG("set offset Y[%d]",c_val[1]);

    c_val[2] = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[2]);
    //atomic_set(&g_nCalZ, temp);
    SENSOR_D_LOG("set offset Z[%d]",c_val[2]);

    ret = set_calib_val(c_val, accuracy, ACCELEROMETER);

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t acc_get_offsets(int32_t* offsets)
{
	int32_t ret;
    struct hc_info hc = {0};
    int16_t aofs_s16[3] = {0};
	SENSOR_D_LOG("start");
	hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
			ACCELEROMETER, SPC_GET_CALIBRATION_VALUE);
	hc.response_len = 7;
	ret = qleoss3_hostcmd(&hc);
    if(ret == QL_STATUS_OK){
        aofs_s16[0] = (int16_t)(hc.response[0]  | (hc.response[1] << 8 ));
        aofs_s16[1] = (int16_t)(hc.response[2]  | (hc.response[3] << 8 ));
        aofs_s16[2] = (int16_t)(hc.response[4]  | (hc.response[5] << 8 ));
        chk_sns_ofs_val(aofs_s16, ACCELEROMETER);
        offsets[0] = (int32_t)aofs_s16[0];
        offsets[1] = (int32_t)aofs_s16[1];
        offsets[2] = (int32_t)aofs_s16[2];
    } else {
        SENSOR_ERR_LOG("hostcmd error cmdID[0x%08x]", hc.command);
        ret = SNS_RC_ERR;
    }
	SENSOR_D_LOG("end");
	return ret;
}

static int32_t acc_set_ac_offsets(struct acceleration offsets)
{
    int32_t ret;
    SENSOR_D_LOG("start offset [X:%d Y:%d Z:%d]", offsets.nX, offsets.nY, offsets.nZ);
    ret = acc_set_auto_cal_offset_internal(offsets);
    SENSOR_D_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t acc_get_ac_offsets(struct acceleration* offsets)
{
    int32_t ret = SNS_RC_OK;
    int32_t tmp_offsets[3];
	SENSOR_D_LOG("start");
    acc_get_offsets(tmp_offsets);
    offsets->nX = tmp_offsets[0];
    offsets->nY = tmp_offsets[1];
    offsets->nZ = tmp_offsets[2];
	SENSOR_D_LOG("end");
    return ret;
}

static int32_t acc_set_nrfilter_param(struct nr_filt_prm prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    acc_nrfilt_param.th_l            = prm.th_l;
    acc_nrfilt_param.th_h            = prm.th_h;
    acc_nrfilt_param.coff            = prm.coff;
    acc_nrfilt_param.rel_th          = prm.rel_th;
    acc_nrfilt_param.base_filt_coff  = prm.base_filt_coff;
    acc_nrfilt_param.base_filt_acoff = prm.base_filt_acoff;
    acc_nrfilt_param.tolerance       = prm.tolerance;
    acc_nrfilt_param.base_filt_hys   = prm.base_filt_hys;
    acc_nrfilt_param.filt_lmt_th     = prm.filt_lmt_th;
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t acc_get_nrfilter_param(struct nr_filt_prm *prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    prm->th_l            = acc_nrfilt_param.th_l;
    prm->th_h            = acc_nrfilt_param.th_h;
    prm->coff            = acc_nrfilt_param.coff;
    prm->rel_th          = acc_nrfilt_param.rel_th;
    prm->base_filt_coff  = acc_nrfilt_param.base_filt_coff;
    prm->base_filt_acoff = acc_nrfilt_param.base_filt_acoff;
    prm->tolerance       = acc_nrfilt_param.tolerance;
    prm->base_filt_hys   = acc_nrfilt_param.base_filt_hys;
    prm->filt_lmt_th     = acc_nrfilt_param.filt_lmt_th;
    SENSOR_D_LOG("end");
    return ret;
}

static void acc_set_ts_nrfilt_info(int32_t base_val)
{
    int64_t input_base_ns = base_val * 1000 * 1000;
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("now setting acc_ts.base_val[%lld], input_base_ns[%lld]",
                        acc_ts.base_val, input_base_ns);
    if(acc_ts.base_set_val != input_base_ns){
        SENSOR_D_LOG("change base val [%lld to %lld]",
                        acc_ts.base_val, input_base_ns);
    	acc_base_set_val_rcv = input_base_ns;
        acc_ts.init_done_flg = 0;
    }
    SENSOR_D_LOG("end");
}

static uint8_t acc_get_device_id(void)
{
    uint8_t id;
	SENSOR_D_LOG("start");
    id = get_device_id(ACCELEROMETER);
	SENSOR_D_LOG("end");
    return id;
}

static int8_t  acc_set_dynamic_calib(bool enable)
{
    int8_t ret;
    ret = set_enable_calib((uint8_t)enable, ACCELEROMETER);
    ret = set_enable_updt_calval((uint8_t)enable, ACCELEROMETER);
    return ret;
}

static int8_t acc_set_selfchk_presetting(void)
{
    int8_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");
    set_dynamic_range(0x00, ACCELEROMETER);
    SENSOR_D_LOG("end");
    return ret;
}

static int8_t acc_set_selfchk_cfg(uint8_t type)
{
    int8_t ret;
    struct hc_info hc   = {0};
    uint8_t slv_addr    = ACC_SLV_ADDR;
    uint8_t w_len       = 0x02;
    uint8_t reg_addr    = 0x14;
    uint8_t setprm;

	SENSOR_D_LOG("start");
    switch(type){
        case SELFCHK_PLUS:
            setprm = 0x01;
            break;
        case SELFCHK_MINUS:
            setprm = 0x02;
            break;
        case SELFCHK_OFF:
            setprm = 0x00;
            break;
    }

    hc.command = prepare_host_command(SYSTEM,
            SYS_I2C, SYS_EXEC_ARBITARY_WRITE_PROCESS);
    hc.response_len = 0;
    hc.param_data_len = 4;
    hc.param_data[0] = slv_addr
                    | (w_len << 8)
                    | (reg_addr << 16)
                    | (setprm << 24);
    ret = qleoss3_hostcmd(&hc);
	SENSOR_D_LOG("end");
    return ret;
}

static int32_t mag_set_offsets(struct geomagnetic offsets)
{
    int32_t ret = SNS_RC_OK;
    int16_t o_16[3] = {0};
    uint8_t accuracy = 0;
    SENSOR_D_LOG("start");
    o_16[0] = (int16_t)offsets.x;
    o_16[1] = (int16_t)offsets.y;
    o_16[2] = (int16_t)offsets.z;
    accuracy = offsets.accuracy;
    ret = set_calib_val(o_16, accuracy, MAGNETOMETER);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t mag_get_offsets(struct geomagnetic *offsets)
{
    int32_t ret;
    struct hc_info hc = {0};
    int16_t mofs_s16[3] = {0};

    SENSOR_D_LOG("start");
	hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
			MAGNETOMETER, SPC_GET_CALIBRATION_VALUE);
	hc.response_len = 7;
	ret = qleoss3_hostcmd(&hc);
    if(ret == QL_STATUS_OK){
        mofs_s16[0] = (int16_t)(hc.response[0]  | (hc.response[1] << 8 ));
        mofs_s16[1] = (int16_t)(hc.response[2]  | (hc.response[3] << 8 ));
        mofs_s16[2] = (int16_t)(hc.response[4]  | (hc.response[5] << 8 ));
        offsets->x = (int32_t)mofs_s16[0];
        offsets->y = (int32_t)mofs_s16[1];
        offsets->z = (int32_t)mofs_s16[2];
        offsets->accuracy = hc.response[6];
        ret = SNS_RC_OK;
    } else {
        SENSOR_ERR_LOG("hostcmd error cmdID[0x%08x]", hc.command);
        ret = SNS_RC_ERR;
    }
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t mag_set_static_matrix(int32_t* static_matrix)
{
    int32_t ret = SNS_RC_OK;
    int16_t sm_16[9];
    SENSOR_D_LOG("start");
    sm_16[0] = (int16_t)static_matrix[0];
    sm_16[1] = (int16_t)static_matrix[1];
    sm_16[2] = (int16_t)static_matrix[2];
    sm_16[3] = (int16_t)static_matrix[3];
    sm_16[4] = (int16_t)static_matrix[4];
    sm_16[5] = (int16_t)static_matrix[5];
    sm_16[6] = (int16_t)static_matrix[6];
    sm_16[7] = (int16_t)static_matrix[7];
    sm_16[8] = (int16_t)static_matrix[8];
    set_Mag_SoftIronVal(sm_16);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t mag_get_static_matrix(int32_t* static_matrix)
{
    int32_t ret = SNS_RC_OK;
    int16_t smat[9] = {0};
    uint8_t i = 0;
    SENSOR_D_LOG("start");
    get_Mag_SoftIronVal(smat);
    for(i = 0; i < ARRAY_SIZE(smat); i++){
        static_matrix[i]= (int32_t)smat[i];
    }
    SENSOR_D_LOG("end");
    return ret;
}

static void gyro_set_static_offsets(int16_t factory_ofs_x, int16_t factory_ofs_y, int16_t factory_ofs_z)
{
    SENSOR_D_LOG("start");
    gyro_static_offset[0] = factory_ofs_x;
    gyro_static_offset[1] = factory_ofs_y;
    gyro_static_offset[2] = factory_ofs_z;
    SENSOR_D_LOG("gyro static offset X[%d], Y[%d], Z[%d]",
                gyro_static_offset[0],gyro_static_offset[1],gyro_static_offset[2]);
    SENSOR_D_LOG("end");
}

static void gyro_set_dynamic_offsets(struct gyroscope *total_ofs)
{
    SENSOR_D_LOG("start");
    gyro_dynamic_offset[0] = total_ofs->x - gyro_static_offset[0];
    gyro_dynamic_offset[1] = total_ofs->y - gyro_static_offset[1];
    gyro_dynamic_offset[2] = total_ofs->z - gyro_static_offset[2];
    SENSOR_D_LOG("gyro dynamic offset X[%d], Y[%d], Z[%d]",
                gyro_dynamic_offset[0],gyro_dynamic_offset[1],gyro_dynamic_offset[2]);
    SENSOR_D_LOG("end");
}

static uint8_t mag_get_device_id(void)
{
    uint8_t id;
	SENSOR_D_LOG("start");
    id = get_device_id(MAGNETOMETER);
	SENSOR_D_LOG("end");
    return id;
}

static int8_t mag_set_dynamic_calib(bool enable)
{
    int8_t ret;
    ret = set_enable_calib((uint8_t)enable, MAGNETOMETER);
    ret = set_enable_updt_calval((uint8_t)enable, MAGNETOMETER);
    return ret;
}

static int32_t gyro_set_offsets(struct gyroscope offsets)
{
    int32_t ret = SNS_RC_OK;
    int16_t o_16[3] = {0};
    uint8_t accuracy;
    SENSOR_D_LOG("start");
    o_16[0] = (int16_t)offsets.x;
    o_16[1] = (int16_t)offsets.y;
    o_16[2] = (int16_t)offsets.z;
    accuracy = 0x03;
    ret = set_calib_val(o_16, accuracy, GYROSCOPE);
    gyro_set_static_offsets(o_16[0], o_16[1], o_16[2]);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t gyro_get_offsets(struct gyroscope* offsets)
{
    int32_t ret;
    int16_t gofs_s16[3] = {0};
    struct hc_info hc = {0};
    SENSOR_D_LOG("start");
	hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
			GYROSCOPE, SPC_GET_CALIBRATION_VALUE);
	hc.response_len = 7;
	ret = qleoss3_hostcmd(&hc);
    if(ret == QL_STATUS_OK){
        gofs_s16[0] = (int16_t)(hc.response[0]  | (hc.response[1] << 8 ));
        gofs_s16[1] = (int16_t)(hc.response[2]  | (hc.response[3] << 8 ));
        gofs_s16[2] = (int16_t)(hc.response[4]  | (hc.response[5] << 8 ));
        offsets->x = (int32_t)gofs_s16[0];
        offsets->y = (int32_t)gofs_s16[1];
        offsets->z = (int32_t)gofs_s16[2];
    } else {
        SENSOR_ERR_LOG("hostcmd error cmdID[0x%08x]", hc.command);
        ret = SNS_RC_ERR;
    }
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t gyro_set_nrfilter_param(struct nr_filt_prm prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    gyro_nrfilt_param.th_l            = prm.th_l;
    gyro_nrfilt_param.th_h            = prm.th_h;
    gyro_nrfilt_param.coff            = prm.coff;
    gyro_nrfilt_param.rel_th          = prm.rel_th;
    gyro_nrfilt_param.base_filt_coff  = prm.base_filt_coff;
    gyro_nrfilt_param.base_filt_acoff = prm.base_filt_acoff;
    gyro_nrfilt_param.tolerance       = prm.tolerance;
    gyro_nrfilt_param.base_filt_hys   = prm.base_filt_hys;
    gyro_nrfilt_param.filt_lmt_th     = prm.filt_lmt_th;
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t gyro_get_nrfilter_param(struct nr_filt_prm *prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    prm->th_l           = gyro_nrfilt_param.th_l;
    prm->th_h           = gyro_nrfilt_param.th_h;
    prm->coff           = gyro_nrfilt_param.coff;
    prm->rel_th         = gyro_nrfilt_param.rel_th;
    prm->base_filt_coff = gyro_nrfilt_param.base_filt_coff;
    prm->base_filt_acoff= gyro_nrfilt_param.base_filt_acoff;
    prm->tolerance      = gyro_nrfilt_param.tolerance;
    prm->base_filt_hys  = gyro_nrfilt_param.base_filt_hys;
    prm->filt_lmt_th    = gyro_nrfilt_param.filt_lmt_th;
    SENSOR_D_LOG("end");
    return ret;
}

static void gyro_set_ts_nrfilt_info(int32_t base_val)
{
    int64_t input_base_ns = base_val * 1000 * 1000;
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("now setting gyro_ts.base_val[%lld], input_base_ns[%lld]",
                        gyro_ts.base_val, input_base_ns);
    if(gyro_ts.base_set_val != input_base_ns){
        SENSOR_D_LOG("change base val [%lld to %lld]",
                        gyro_ts.base_val, input_base_ns);
    	gyro_base_set_val_rcv = input_base_ns;
        gyro_ts.init_done_flg = 0;
    }
    SENSOR_D_LOG("end");
}

static uint8_t gyro_get_device_id(void)
{
    uint8_t id;
	SENSOR_D_LOG("start");
    id = get_device_id(GYROSCOPE);
	SENSOR_D_LOG("end");
    return id;
}

static int8_t gyro_set_dynamic_calib(bool enable)
{
    int8_t ret;
    ret = set_enable_calib((uint8_t)enable, GYROSCOPE);
    ret = set_enable_updt_calval((uint8_t)enable, GYROSCOPE);
    return ret;
}

static int8_t gyro_set_selfchk_presetting(void)
{
    int8_t ret = SNS_RC_OK;
	SENSOR_D_LOG("start");
    set_dynamic_range(0x03, GYROSCOPE);
    SENSOR_D_LOG("end");
    return ret;
}

static int8_t gyro_set_selfchk_cfg(uint8_t type)
{
    int8_t ret;
    struct hc_info hc   = {0};
    uint8_t slv_addr    = GYRO_SLV_ADDR;
    uint8_t w_len       = 0x02;
    uint8_t reg_addr    = 0x14;
    uint8_t setprm;

	SENSOR_D_LOG("start");
    switch(type){
        case SELFCHK_PLUS:
            setprm = 0x04;
            break;
        case SELFCHK_MINUS:
            setprm = 0x0C;
            break;
        case SELFCHK_OFF:
            setprm = 0x00;
            break;
    }

    hc.command = prepare_host_command(SYSTEM,
            SYS_I2C, SYS_EXEC_ARBITARY_WRITE_PROCESS);
    hc.response_len = 0;
    hc.param_data_len = 4;
    hc.param_data[0] = slv_addr
                    | (w_len << 8)
                    | (reg_addr << 16)
                    | (setprm << 24);
    SENSOR_ERR_LOG("Set parameter => 0x%08x", hc.param_data[0]);
    ret = qleoss3_hostcmd(&hc);
	SENSOR_D_LOG("end");
    return ret;
}

static int8_t gyro_start_man_calib(uint8_t *calprm)
{
    int8_t ret;
    struct hc_info hc = {0};
    SENSOR_D_LOG("start");

    SENSOR_ERR_LOG("calprm[0]=0x%02x calprm[1]=0x%02x calprm[2]=0x%02x calprm[3]=0x%02x",
                    calprm[0],calprm[1],calprm[2],calprm[3]);
    SENSOR_ERR_LOG("calprm[4]=0x%02x calprm[5]=0x%02x calprm[6]=0x%02x calprm[7]=0x%02x",
                    calprm[4],calprm[5],calprm[6],calprm[7]);
    SENSOR_ERR_LOG("calprm[8]=0x%02x calprm[9]=0x%02x calprm[10]=0x%02x calprm[11]=0x%02x",
                    calprm[8],calprm[9],calprm[10],calprm[11]);
    SENSOR_ERR_LOG("calprm[12]=0x%02x calprm[13]=0x%02x",
                    calprm[12],calprm[13]);

    hc.command = prepare_host_command(APP, APP_GY,
                                    APP_GY__GyroCalParamSetting);
    hc.response_len = 0;
    hc.param_data_len = 14;
    hc.param_data[0] =  calprm[0]           // GY_PramChangeFlag
                     | (calprm[1] << 8)     // GY_CalDoneFlag
                     | (calprm[2] << 16)    // GY_MoveTh
                     | (calprm[3] << 24);   // GY_MoveTh
    hc.param_data[1] =  calprm[4]           // GY_MaxMinTh
                     | (calprm[5] << 8)     // GY_MaxMinTh
                     | (calprm[6] << 16)    // GY_StillTh
                     | (calprm[7] << 24);   // GY_StillTh
    hc.param_data[2] =  calprm[8]           // GY_DiffOffset
                     | (calprm[9] << 8)     // GY_DiffOffset
                     | (calprm[10] << 16)   // GY_DataNumStillFastest
                     | (calprm[11] << 24);  // GY_DataNumStillFastest
    hc.param_data[3] =  calprm[12]          // GY_DataNumMovingFastest
                     | (calprm[13] << 8)    // GY_DataNumMovingFastest
                     | (0x00 << 16)         // padding
                     | (0x00 << 24);        // padding
    ret = qleoss3_hostcmd(&hc);
	SENSOR_D_LOG("end");
    return ret;
}

static int8_t  gyro_wait_man_calib(void)
{
    int8_t ret;
    int8_t loopnum = 0;
    uint8_t wait_result;
    struct hc_info hc = {0};

	SENSOR_D_LOG("start");
    hc.command = prepare_host_command(APP, APP_GY,
                                    APP_GY__GetGyroCalParamSetting);
    hc.response_len = 14;
    hc.param_data_len = 0;
    while(1){
        msleep(100);
        ret = qleoss3_hostcmd(&hc);
        if(ret != QL_STATUS_OK){
            ret = -3;
            break;
        }
        wait_result = hc.response[1];
        SENSOR_ERR_LOG("calib result = 0x%02x", wait_result);
        if(wait_result & 0x01){
            SENSOR_D_LOG("Gyro Calibration Done.");
            ret = 0;
            break;
        } else if(loopnum >= 30){
            SENSOR_ERR_LOG("Gyro Calib loop num expired.");
            ret = -1;
            break;
        } else {
            SENSOR_D_LOG("Gyro Calibrating.");
            loopnum++;
        }
    }
	SENSOR_D_LOG("end");
    return ret;
}

static void gyrouc_set_ts_nrfilt_info(int32_t base_val)
{
    int64_t input_base_ns = base_val * 1000 * 1000;
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("now setting gyrouc_ts.base_val[%lld], input_base_ns[%lld]",
                        gyrouc_ts.base_val, input_base_ns);
    if(gyrouc_ts.base_set_val != input_base_ns){
        SENSOR_D_LOG("change base val [%lld to %lld]",
                        gyrouc_ts.base_val, input_base_ns);
    	gyrouc_base_set_val_rcv = input_base_ns;
        gyrouc_ts.init_done_flg = 0;
    }
    SENSOR_D_LOG("end");
}

static int32_t press_set_offsets(struct pressure offset)
{
    int32_t ret = SNS_RC_OK;
    int16_t o_16[1] = {0};
    uint8_t accuracy = 0;
    SENSOR_D_LOG("start");
    o_16[0] = (int16_t)offset.pressure;
    ret = set_calib_val(o_16, accuracy, BAROMETER);
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t press_get_offsets(struct pressure* offsets)
{
    int32_t ret;
    struct hc_info hc = {0};
    SENSOR_D_LOG("start");

	hc.command = prepare_host_command(SENSOR_PROCESS_CONTROL,
			BAROMETER, SPC_GET_CALIBRATION_VALUE);
	hc.response_len = 2;
	ret = qleoss3_hostcmd(&hc);
    if(ret == QL_STATUS_OK){
        offsets->pressure = (((int32_t)hc.response[0]      ) & 0x000000FF)
                            | (((int32_t)hc.response[1] <<  8) & 0x0000FF00);
        ret = SNS_RC_OK;
    } else {
        ret = SNS_RC_ERR;
    }
    SENSOR_D_LOG("end");
    return ret;
}

static uint8_t press_get_device_id(void)
{
    uint8_t id;
	SENSOR_D_LOG("start");
    id = get_device_id(BAROMETER);
	SENSOR_D_LOG("end");
    return id;
}


static int32_t sgnfcnt_enable(bool enable)
{
    int32_t ret = SNS_RC_OK;
    struct sensor_ctrl_param_str now_ctrl_param = get_sns_ctrl_param();
    SENSOR_D_LOG("start enable[%d]", enable);
    if(enable){
        now_ctrl_param.output_param.sensor.enable |= EN_BIT_SIGNFCNT_MTN;
    } else {
        now_ctrl_param.output_param.sensor.enable &= ~EN_BIT_SIGNFCNT_MTN;
    }
    ret = enable_calc_processing(&now_ctrl_param.output_param);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("enable_calc_processing error ret[%d]", ret);
    }
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t reset_save_and_taskoff(void)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return ret;
}

static void set_batch_notify(bool enable)
{
    SENSOR_D_LOG("start");
#ifndef CONFIG_INPUT_SENSOR_EOSS3_MEM_REDUCE_VER1
    enable_batch_full_interrupt(enable);
    enable_batch_interrupt(enable, BATCH_MARGIN_SIZE);
#endif
    SENSOR_D_LOG("end");
}

static int32_t enable_i2c_peri(bool enable)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return ret;
}

static void clr_batch_condition(void)
{
    SENSOR_D_LOG("start");
    atomic_set(&is_1stBatch_proc, true);
    current_s3_ts[SENSOR_TASK]  = 0;
    current_s3_ts[APP_TASK]     = 0;
    previous_s3_ts[SENSOR_TASK] = 0;
    previous_s3_ts[APP_TASK]    = 0;
    SENSOR_D_LOG("end");
}

static int32_t sns_initcmd_exe(void)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t set_dev_param(void)
{
	int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
	ret = InitSetting();
	if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Failed InitSetting[%d]",ret);
		return ret;
	}
    SENSOR_D_LOG("end");
    return SNS_RC_OK;
}


static int32_t set_measurement_mode(const uint8_t mode, const uint8_t sensortype)
{
	int32_t ret = SNS_RC_OK;
	struct hc_info hc;

	SENSOR_D_LOG("start");
	if(sensortype >= PHYSICAL_SENS_MAX){
		SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
		return -EINVAL;
	}

	if( is_sns_connected(sensortype) ){
		memset(&hc, 0, sizeof(hc));
		hc.command = prepare_host_command(SENSOR_DEVICE_CONTROL,
				sensortype, SDC_SET_MEASUREMENT_MODE);
		hc.param_data_len = 1;
		hc.param_data[0] = mode;
		hc.response_len = 0;

		qleoss3_hostcmd(&hc);
	} else {
		SENSOR_ERR_LOG("sensor[0x%02x] is not connected. this process was skipped.", sensortype);
	}
	SENSOR_D_LOG("end[%d]", ret);

	return ret;
}

static int32_t set_all_measurement_mode(void)
{
	int32_t ret = SNS_RC_OK;
	uint8_t m_mode = 0;

	SENSOR_D_LOG("start");

#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	m_mode = LP_MODE;
	ret = set_measurement_mode(m_mode, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Error : AccelerometerSensor Set Measurement Mode");
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	m_mode = LP_MODE;
	ret = set_measurement_mode(m_mode, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Error : GyroscopeSensor Set Measurement Mode");
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
	m_mode = LP_MODE;
	ret = set_measurement_mode(m_mode, BAROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Error : Barometer Set Measurement Mode");
	}
#endif /* CONFIG_INPUT_SENSOR_PRESSURE */
exit:
    SENSOR_D_LOG("end[%d]", ret);

	return ret;
}

static int32_t set_dynamic_range(const uint8_t val, const uint8_t sensortype)
{
    int32_t ret = SNS_RC_OK;
    struct hc_info hc;

    SENSOR_D_LOG("start");
    if(sensortype >= PHYSICAL_SENS_MAX){
        SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
        return -EINVAL;
    }

    if( is_sns_connected(sensortype) ){
        memset(&hc, 0, sizeof(hc));

        hc.command = prepare_host_command(SENSOR_DEVICE_CONTROL,
                sensortype, SDC_SET_DYNAMIC_RANGE);
        hc.param_data_len = 1;
        hc.param_data[0] = val;
        hc.response_len = 0;

        qleoss3_hostcmd(&hc);
    } else {
        SENSOR_ERR_LOG("skipped. sensor[0x%02x] is dis-connected.", sensortype);
    }
    SENSOR_D_LOG("end[%d]", ret);

    return ret;
}

static int32_t set_all_dynamic_range(void)
{
	uint8_t range_val = 0;
	int32_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");

#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
	range_val = 0x02;
	ret = set_dynamic_range(range_val, ACCELEROMETER);
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Error : AccelerometerSensor Set Dynamic Range");
		goto exit;
	}
#endif /* CONFIG_INPUT_SENSOR_ACCELEROMETER */

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
	range_val = 0x03;
	ret = set_dynamic_range(range_val, GYROSCOPE);
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("Error : GyroscopeSensor Set Dynamic Range");
	}
#endif /* CONFIG_INPUT_SENSOR_GYROSCOPE */
exit:
	SENSOR_D_LOG("end");
	return ret;
}

static int32_t init_exec(const uint8_t sensortype)
{
	int32_t ret = SNS_RC_OK;
	struct hc_info hc = {0};

    SENSOR_D_LOG("start senstype[0x%02d]", sensortype);
	if(sensortype >= PHYSICAL_SENS_MAX){
		SENSOR_ERR_LOG("INVALID SensorType[0x%02x]", sensortype);
		return -EINVAL;
	}
	memset(&hc, 0, sizeof(hc));

	hc.command = prepare_host_command(SENSOR_DEVICE_CONTROL,
			sensortype, SDC_EXECUTE_PROCESS_OF_INIT);
    hc.param_data_len = 0;
	hc.response_len = 8;

	qleoss3_hostcmd(&hc);
	if (hc.response[0] == 0) {
		SENSOR_ERR_LOG("Target device[0x%02x] not found", sensortype);
		return SNS_RC_ERR;
	}

    SENSOR_D_LOG("end");
	return ret;
}

static int8_t sensor_init(const uint8_t sensortype)
{
    int8_t ret = SNS_RC_OK;
    uint8_t retry_cnt = 0;

    SENSOR_D_LOG("start");
    while(1){
        ret = init_exec(sensortype);
        if(ret == SNS_RC_OK){
            SENSOR_D_LOG("OK : Initialize. sensortype[%d]", sensortype);
            break;
        } else if (retry_cnt < SNS_INIT_RETRY_NUM){
            ++retry_cnt;
        } else {
            ret = SNS_RC_ERR;
            break;
        }
    }
    SENSOR_D_LOG("end");
    return ret;
}

static int32_t all_init(void)
{
    int32_t ret = SNS_RC_OK;

    SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    SENSOR_D_LOG("AccelerometerSensor Initialize");
    ret = sensor_init(ACCELEROMETER);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error : AccelerometerSensor Initialize");
        g_acc_available = 0;
        ret = SNS_RC_ERR;
    }
#endif /*CONFIG_INPUT_SENSOR_ACCELEROMETER*/
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    SENSOR_D_LOG("MagnetometerSensor Initialize");
    ret = sensor_init(MAGNETOMETER);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error : MagnetometerSensor Initialize");
        g_mag_available = 0;
        ret = SNS_RC_ERR;
    }
#endif /*CONFIG_INPUT_SENSOR_MAGNETOMETER*/

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    SENSOR_D_LOG("GyroscopeSensor Initialize");
    ret = sensor_init(GYROSCOPE);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error : GyroscopeSensor Initialize");
        g_gyro_available = 0;
        ret = SNS_RC_ERR;
    }
#endif /*CONFIG_INPUT_SENSOR_GYROSCOPE*/

#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    SENSOR_D_LOG("Barometer Initialize");
    ret = sensor_init(BAROMETER);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error : Barometer Initialize");
        g_pres_available = 0;
        ret = SNS_RC_ERR;
    }
#endif /*CONFIG_INPUT_SENSOR_PRESSURE*/
    sns_invalid_setting(g_acc_available, g_mag_available, g_gyro_available, g_pres_available);
    SENSOR_D_LOG("end");

    return ret;
}

static void init_arg(void)
{
    SENSOR_D_LOG("start");
    acc_nrfilt_param.th_l             = INIT_ACC_NR_FILT_TH_L;
    acc_nrfilt_param.th_h             = INIT_ACC_NR_FILT_TH_H;
    acc_nrfilt_param.coff             = INIT_ACC_NR_FILT_COFF;
    acc_nrfilt_param.rel_th           = INIT_ACC_NR_REL_TH;
    acc_nrfilt_param.base_filt_coff   = INIT_ACC_NR_BASE_FILT_COFF;
    acc_nrfilt_param.base_filt_acoff  = INIT_ACC_NR_BASE_FILT_ACOFF;
    acc_nrfilt_param.tolerance        = INIT_ACC_NR_TOLERANCE;
    acc_nrfilt_param.base_filt_hys    = INIT_ACC_NR_BASE_FILT_HYS;
    acc_nrfilt_param.filt_lmt_th      = INIT_ACC_NR_FILT_LMT_TH;
    gyro_nrfilt_param.th_l            = INIT_GYR_NR_FILT_TH_L;
    gyro_nrfilt_param.th_h            = INIT_GYR_NR_FILT_TH_H;
    gyro_nrfilt_param.coff            = INIT_GYR_NR_FILT_COFF;
    gyro_nrfilt_param.rel_th          = INIT_GYR_NR_REL_TH;
    gyro_nrfilt_param.base_filt_coff  = INIT_GYR_NR_BASE_FILT_COFF;
    gyro_nrfilt_param.base_filt_acoff = INIT_GYR_NR_BASE_FILT_ACOFF;
    gyro_nrfilt_param.tolerance       = INIT_GYR_NR_TOLERANCE;
    gyro_nrfilt_param.base_filt_hys   = INIT_GYR_NR_BASE_FILT_HYS;
    gyro_nrfilt_param.filt_lmt_th     = INIT_GYR_NR_FILT_LMT_TH;
    ts_nrfilt_param.ts_th_us          = INIT_TS_NR_FILT_TH_US;
    ts_nrfilt_param.ts_tol_us         = INIT_TS_NR_FILT_TOLERANCE_US;
    ts_nrfilt_param.ts_rel_th         = INIT_TS_NR_REL_TH;
    ts_nrfilt_param.ts_base_filt_coff = INIT_TS_NR_BASE_FILT_COFF;
    ts_nrfilt_param.ts_base_filt_acoff= INIT_TS_NR_BASE_FILT_ACOFF;
    ts_nrfilt_param.ts_filt_coff      = INIT_TS_NR_FILT_COFF;
    ts_nrfilt_param.ts_ng_th          = INIT_TS_NR_NG_TH;
    ts_nrfilt_param.ts_unaccept_num   = INIT_TS_NR_UNACCEPT_NUM;
    ts_nrfilt_param.ts_reserved1      = 0;
    ts_nrfilt_param.ts_reserved2      = 0;
    SENSOR_D_LOG("end");
}
static int32_t InitDevice(void)
{
	int32_t ret = SNS_RC_OK;

	SENSOR_D_LOG("start");

	ret = all_init();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("all_init FAILED [%d]", ret);
	}
	ret = set_all_dynamic_range();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("set_all_dynamic_range FAILED [%d]", ret);
		goto exit;
	}
	ret = set_all_measurement_mode();
	if(ret != SNS_RC_OK){
		SENSOR_ERR_LOG("set_all_measurement_mode FAILED [%d]", ret);
	}
exit:
    SENSOR_D_LOG("end - SNS_RC_OK");
	return ret;
}

static int32_t get_micon_info_dt(struct spi_device *client)
{
    struct device_node *np = client->dev.of_node;
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    uint8_t tmp_acc_conv_axis = 0x00;
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    uint8_t tmp_gyro_conv_axis = 0x00;
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    uint8_t tmp_mag_conv_axis = 0x00;
    uint32_t tmp_mag_smat[9] ={0};
    uint8_t i = 0;
#endif
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    ret = of_property_read_u8(np, "micon-acc-conv_axis", &tmp_acc_conv_axis);
    if(ret){
        SENSOR_ERR_LOG("micon-acc-conv_axis read err[%x]", ret);
        return ret;
    }
    g_acc_conv_axis = tmp_acc_conv_axis;
#endif

#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    ret = of_property_read_u8(np, "micon-gyro-conv_axis", &tmp_gyro_conv_axis);
    if(ret){
        SENSOR_ERR_LOG("micon-gyro-conv_axis read err[%x]", ret);
        return ret;
    }
    g_gyro_conv_axis = tmp_gyro_conv_axis;
#endif

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    ret = of_property_read_u8(np, "micon-mag-conv_axis", &tmp_mag_conv_axis);
    if(ret){
        SENSOR_ERR_LOG("micon-mag-conv_axis read err[%x]", ret);
        return ret;
    }
    g_mag_conv_axis = tmp_mag_conv_axis;
#endif

#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    ret = of_property_read_u32_array(np, "micon-mag-static_matrix", tmp_mag_smat, 9);
    if(ret){
        SENSOR_ERR_LOG("micon-mag-static_matrix read err[%x]", ret);
        return ret;
    }
    for(i = 0; i < ARRAY_SIZE(g_mag_static_matrix); i++){
        g_mag_static_matrix[i] = (int16_t)tmp_mag_smat[i];
    }
#endif

    SENSOR_D_LOG("end");
    return ret;
}

static int32_t initialize_snsdrv(void)
{
    int32_t ret = SNS_RC_OK;

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_D_LOG("start");

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    init_arg();
    ret = InitDevice();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Error : Sensor Initialize[%d]", ret);
        return ret;
    }

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    SENSOR_D_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static void irq_proc_priority3(uint64_t result)
{
    uint64_t batch_buf_expired = (REMAIN_MARGIN_INTR
                                | M4_BUFFER_FULL_INTR
                                | FFE_BUFFER_FULL_INTR);
    uint64_t batch_buf_blank_full = (M4_BUF_BLANK_FULL | FFE_BUF_BLANK_FULL);
    SENSOR_D_LOG("start");
    if(result & IRQ_PROC_PRIORITY3){
        if (result & ROT_CHANGE_DETECTED) {
            SENSOR_D_LOG("Rotation change detected\n");
            sns_notify(SENSOR_EXT_VH,0);
            sns_notify(SENSOR_DEVICE_ORIENTATION,0);
        }

        if (result & MAX_LATENCY_INTR) {
            SENSOR_D_LOG("Batch latency expire intr\n");
            batch_mark_off(LOGGING_TRIGGER_MAX_LATENCY, true);
            SENSOR_D_LOG("mark_off end");
            return;
        }

        if (result & batch_buf_expired ) {
            SENSOR_D_LOG("Batch margin/full(M4 of FFE):factor[0x%016llx]", result);
            batch_mark_off(LOGGING_TRIGGER_REMAIN_MARGIN, true);
            SENSOR_D_LOG("mark_off end");
            return;
        }

        if (result & batch_buf_blank_full) {
            SENSOR_ERR_LOG("M4/FFE Buffer Blank Space full:factor[0x%016llx]", result);
            all_clr_batching_data();
            batch_mark_off(LOGGING_TRIGGER_M4_BUF_BLANK_FULL, false);
        }
    }
    SENSOR_D_LOG("end");
}

static void irq_proc_priority4(uint64_t result)
{
    SENSOR_D_LOG("start");
    if(result & IRQ_PROC_PRIORITY4){
    	if (result & KPT2_DETECTED) {
            SENSOR_D_LOG("KPT detected");
            SENSOR_D_LOG("KPT detected[OK A]");
            if(s_GreenledOnstate){
                s_LEDparam &= ~LED_GREEN_ON;
                s_GreenledOnstate = false;
            }else{
                s_LEDparam |= LED_GREEN_ON;
                s_GreenledOnstate = true;
            }
#ifdef CONFIG_INPUT_SENSOR_VT_WAKEUP
            sns_notify(VT_WAKEUP,0);
#endif
            sns_notify(SENSOR_VOICE_TRIGGER, (uint32_t)s_GreenledOnstate);
            //kclights_led_set_test(s_LEDparam);
        }

        if (result & KPT1_DETECTED) {
            SENSOR_D_LOG("KPT detected");
            SENSOR_D_LOG("VT detected! (but I'm not VT)");
            if(s_RedledOnstate){
                s_LEDparam &= ~LED_RED_ON;
                s_RedledOnstate = false;
            }else{
                s_LEDparam |= LED_RED_ON;
                s_RedledOnstate = true;
            }
#ifdef CONFIG_INPUT_SENSOR_VT_WAKEUP
            sns_notify(VT_WAKEUP,0);
#endif
            sns_notify(SENSOR_VOICE_TRIGGER, (uint32_t)s_RedledOnstate);
            //kclights_led_set_test(s_LEDparam);
        }

        if (result & VOICE_DATA_READY) {
            SENSOR_D_LOG("VOICE_DATA_READY detected");
            SENSOR_D_LOG("Audio streaming events detect.(Voice data block)");
        }

        if (result & VOICE_STREAM_END) {
            SENSOR_D_LOG("VOICE_STREAM_END detected");
            SENSOR_D_LOG("Audio streaming events detect.(Voice data block)");
        }
    }
    SENSOR_D_LOG("end");
}

static void irq_proc_priority5(uint64_t result)
{
    SENSOR_D_LOG("start");
    if(result & IRQ_PROC_PRIORITY5){
        if (result & EVENT_MARGIN_INTR) {
            SENSOR_D_LOG("Event Buffer Margin intr\n");
            //atomic_set(&event_margin, true);
            dailys_event_mark_off();
        }

        if (result & EVENT_MAX_LAT_INTR) {
            SENSOR_D_LOG("Event Buffer Max Latency intr\n");
            //atomic_set(&event_lat_exp, true);
            dailys_event_mark_off();
        }

        if (result & EVENT_BUFFER_FULL_INTR) {
            SENSOR_D_LOG("Event Buffer Full intr\n"); //If event buffer is full, this is a error case.
            //atomic_set(&event_buf_full, true);
            dailys_event_mark_off();
        }

        if (result & EVENT_BLANK_FULL_INTR) {
            SENSOR_D_LOG("Event Buffer Blank Space Full intr\n");
            //atomic_set(&event_buf_blank_full, true);
            dailys_event_mark_off();
        }
    }
    SENSOR_D_LOG("end");
}

static void irq_proc_priority6(uint64_t result)
{
    struct geomagnetic*     mag_ofs;
    struct gyroscope*       gyro_ofs;

    SENSOR_D_LOG("start");
    if(result & IRQ_PROC_PRIORITY6){
        if (result & ACCEL_CAL_DONE) {
            SENSOR_D_LOG("Accel cal done detected\n");
            sns_notify(SENSOR_ACC_AUTO_CAL,0);
        }

        if (result & MAG_CAL_DONE) {
            SENSOR_D_LOG("Mag cal done detected\n");
            mag_ofs = (struct geomagnetic*)kzalloc(sizeof(struct geomagnetic), GFP_KERNEL);
            mag_get_offsets(mag_ofs);
            sns_save_offset_value(SENSOR_MAG, mag_ofs);
            kfree(mag_ofs);
        }

        if (result & GYRO_CAL_DONE) {
            SENSOR_D_LOG("Gyro cal done detected\n");
            gyro_ofs = (struct gyroscope*)kzalloc(sizeof(struct gyroscope), GFP_KERNEL);
            gyro_get_offsets(gyro_ofs);
            gyro_set_dynamic_offsets(gyro_ofs);
            //sns_save_offset_value(SENSOR_GYRO, gyro_ofs);
            kfree(gyro_ofs);
        }

        if (result & SIGN_MOTION_DETECTED) {
            SENSOR_D_LOG("Significant motion detected\n");
            sns_notify(SENSOR_SGNFCNT_MTN, 0);
        }
        if (result & PD_STEP) {
            SENSOR_D_LOG("PD_STEP Detected\n");
            dailys_event_mark_off();
            sns_notify(SENSOR_EXT_PEDO,0);
        }

        if (result & VC_DETECTED) {
            SENSOR_D_LOG("VC_DETECTED\n");
            dailys_event_mark_off();
            sns_notify(SENSOR_EXT_PEDO,0);
        }

        if (result & PS_GRADCNG) {
            SENSOR_D_LOG("PS_GRADCNG\n");
            dailys_event_mark_off();
            sns_notify(SENSOR_EXT_PEDO,0);
        }

        if (result & SD_CNG) {
            SENSOR_D_LOG("SD_CNG\n");
            SENSOR_D_LOG("SD_CNG does not used.");
        }

        if (result & SP_CNG) {
            SENSOR_D_LOG("SP_CNG\n");
            sns_notify(SENSOR_KC_MOTION_WALK_STOP,0);
        }

        if (result & BW_CNG) {
            SENSOR_D_LOG("BW_CNG\n");
            sns_notify(SENSOR_KC_MOTION_WALK_START,0);
        }

        if (result & BU_CNG) {
            SENSOR_D_LOG("BU_CNG\n");
            sns_notify(SENSOR_KC_MOTION_BRINGUP,0);
        }

        if (result & TR_DETECTED) {
            SENSOR_D_LOG("TR_DETECTED\n");
            sns_notify(SENSOR_KC_MOTION_TRAIN,0);
        }

        if (result & TR_FST_DETECTED) {
            SENSOR_D_LOG("TR_FST_DETECTED\n");
            sns_notify(SENSOR_KC_MOTION_TRAIN,0);
        }

        if (result & PD_FST_OTH_DETECTED) {
            SENSOR_D_LOG("PD_FST_OTH_DETECTED\n");
            sns_notify(SENSOR_KC_MOTION_VEHICLE,0);
        }

	    if (result & UWD_WATER_DETECTED) {
		    SENSOR_D_LOG("UWD_WATER_DETECTED\n");
            sns_notify(SENSOR_UNDERWATER_DETECT, 0);
        }

    	if (result & UWD_TARGET_DEPTH_DETECTED) {
    		SENSOR_D_LOG("UWD_TARGET_DEPTH_DETECTED\n");
            //sns_notify(SENSOR_UNDERWATER_DETECT, 0);
    	}

    	if (result & UWD_MAX_DEPTH_DETECTED) {
    		SENSOR_D_LOG("UWD_MAX_DEPTH_DETECTED\n");
            //sns_notify(SENSOR_UNDERWATER_DETECT, 0);
    	}
    }
    SENSOR_D_LOG("end");
}

static void irq_proc(uint64_t result)
{
    uint8_t drv_status = (uint8_t)atomic_read(&s_snsdrv_status);
    SENSOR_D_LOG("start");

    if(drv_status == (uint8_t)SENSOR_SHUTDOWN){
        SENSOR_D_LOG("end -skip due to shutdown sequence running.");
        return;
    }
    if (result & SENSOR_MICON_WAKELOCK_STATUS) {
        sns_pm_wakeup_event(SENSOR_MICON_WAKELOCK_TIME);
    }

    SENSOR_D_LOG("irq result = 0x%016llx", result);
    irq_proc_priority3(result);
    irq_proc_priority4(result);
    irq_proc_priority5(result);
    irq_proc_priority6(result);

    SENSOR_D_LOG("end");
}
void func_register_eoss3(struct sensor_micon_if_funcs *f)
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
    f->read_all_sns_val             = read_all_sensor_value;
    f->get_miconshtdwn_status       = get_miconshtdwn_status;
    f->set_sensdrv_status           = set_sensordrv_status;
    f->waiting_for_access_chk       = dev_access_check;
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
    f->get_micon_info               = get_micon_info_dt;
    f->irq_proc                     = irq_proc;
    f->set_dev_param                = set_dev_param;
    f->reset_saveparam_and_taskoff  = reset_save_and_taskoff;
    f->set_sns_initcmd              = sns_initcmd_exe;
    f->enable_i2c_peripheral        = enable_i2c_peri;
    f->clr_batch_condition          = clr_batch_condition;
    f->set_batch_notify             = set_batch_notify;
    f->acc_func.set_offsets         = acc_set_offsets;
    f->acc_func.get_offsets         = acc_get_offsets;
    f->acc_func.set_ac_offsets      = acc_set_ac_offsets;
    f->acc_func.get_ac_offsets      = acc_get_ac_offsets;
    f->acc_func.set_nrfilt_prm      = acc_set_nrfilter_param;
    f->acc_func.get_nrfilt_prm      = acc_get_nrfilter_param;
    f->acc_func.clear_filt_coff     = acc_clear_filt_coff;
    f->acc_func.set_ts_nrfilt_info  = acc_set_ts_nrfilt_info;
    f->acc_func.get_devid           = acc_get_device_id;
    f->acc_func.set_dcalib          = acc_set_dynamic_calib;
    f->acc_func.set_selfchk_precfg  = acc_set_selfchk_presetting;
    f->acc_func.set_selfchk_cfg     = acc_set_selfchk_cfg;
    f->mag_func.set_offsets         = mag_set_offsets;
    f->mag_func.get_offsets         = mag_get_offsets;
    f->mag_func.set_static_matrix   = mag_set_static_matrix;
    f->mag_func.get_static_matrix   = mag_get_static_matrix;
    f->mag_func.get_devid           = mag_get_device_id;
    f->mag_func.set_dcalib          = mag_set_dynamic_calib;
    f->gyro_func.set_offsets        = gyro_set_offsets;
    f->gyro_func.get_offsets        = gyro_get_offsets;
    f->gyro_func.set_nrfilt_prm     = gyro_set_nrfilter_param;
    f->gyro_func.get_nrfilt_prm     = gyro_get_nrfilter_param;
    f->gyro_func.clear_filt_coff    = gyro_clear_filt_coff;
    f->gyro_func.set_ts_nrfilt_info = gyro_set_ts_nrfilt_info;
    f->gyro_func.get_devid          = gyro_get_device_id;
    f->gyro_func.set_dcalib         = gyro_set_dynamic_calib;
    f->gyro_func.set_selfchk_precfg = gyro_set_selfchk_presetting;
    f->gyro_func.set_selfchk_cfg    = gyro_set_selfchk_cfg;
    f->gyro_func.start_man_calib    = gyro_start_man_calib;
    f->gyro_func.wait_man_calib     = gyro_wait_man_calib;
    f->gyrouc_func.clear_filt_coff    = gyrouc_clear_filt_coff;
    f->gyrouc_func.set_ts_nrfilt_info = gyrouc_set_ts_nrfilt_info;
    f->press_func.set_offsets       = press_set_offsets;
    f->press_func.get_offsets       = press_get_offsets;
    f->press_func.get_devid         = press_get_device_id;
    f->sgnfcnt_func.enable          = sgnfcnt_enable;
    f->app_cmn.ts_set_nrfilt_prm    = set_ts_nrfilter_param;
    f->app_cmn.ts_get_nrfilt_prm    = get_ts_nrfilter_param;
    f->app_cmn.clear                = micon_app_clear;
    f->ds_func.send_ope_param       = dailys_send_param;
    f->ds_func.enable               = dailys_enable;
    f->ds_func.send_latency_option  = dailys_send_latency_option;
    f->ds_func.get_pedo_data        = dailys_get_pedo_data;
    f->ds_func.get_vehi_data        = dailys_get_vehi_data;
    f->ds_func.event_mark_off       = dailys_event_mark_off;
    f->ds_func.get_mark_off_times   = get_event_mark_off_times;
    f->ds_func.get_mark_off_clrinfo = get_event_mark_off_clrinfo;
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
