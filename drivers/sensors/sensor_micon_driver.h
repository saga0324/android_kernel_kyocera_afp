/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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
//#include <mach/gpio.h>
//#include <mach/gpiomux.h>

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

#define READ_RSLT                (1)
#define READ_FIFO                (0)

#define CFG                          (0x00u)
#define INTMASK0                     (0x02u)
#define INTMASK1                     (0x03u)
#define STATUS                       (0x09u)
#define INTREQ0                      (0x0cu)
#define ERROR0                       (0x0au)
#define CMD0                         (0x30u)
#define PRM0                         (0x2Fu)
#define PRM0F                        (0x20u)
#define RSLT00                       (0x40u) /* Acceleration */
#define RSLT06                       (0x46u) /* Gyroscope */
#define RSLT0C                       (0x4Cu) /* Geomagnetic */
#define RSLT12                       (0x52u) /* Pressure */
#define RSLT14                       (0x54u) /* Orientation */
#define RSLT1A                       (0x5Au) /* Gravity */
#define RSLT20                       (0x60u) /* LinearAcceleration */
#define RSLT26                       (0x66u) /* RotationVector */
#define RSLT2C                       (0x6Cu) /* Pressure */
#define RSLT2D                       (0x6Du) /* GeomagneticRotationVectorAccuracy */
#define RSLT2E                       (0x6Eu) /* GeomagneticRotationVector */
#define RSLT36                       (0x76u) /* GameRotationVector */
#define RSLT3E                       (0x7Eu)
#define FIFO                         (0x10u)

#define RSLT30                       (0x70u)
#define RSLT31                       (0x71u)
#define RSLT32                       (0x72u)
#define RSLT33                       (0x73u)
#define RSLT34                       (0x74u)
#define RSLT35                       (0x75u)
#define RSLT36                       (0x76u)
#define RSLT37                       (0x77u)



#define INTREQ_NONE                  (0x0000u)
#define INTREQ_HOST_CMD              (0x0001u)
#define INTREQ_ACC                   (0x0002u)
#define INTREQ_MAG                   (0x0080u)
#define INTREQ_GYRO                  (0x0100u)
#define INTREQ_NMI                   (0x0800u)
#define INTREQ_EXCP                  (0x1000u)
#define INTREQ_FUSION                (0x2000u)
#define INTREQ_LOGGING               (0x4000u)
#define INTREQ_KCLOG                 (0x8000u)
#define INTREQ_ERROR                 (0xFFFEu)

#define GPIO_INT_NAME            "sns_irq"
#define GPIO_RESET_NAME          "sns_reset"
#define GPIO_BRMP_NAME           "sns_brmp"
//#define SNS_GPIO_INT              (31)
//#define SNS_GPIO_BRMP             (32)
//#define SNS_GPIO_RST              (1)

#define SNESOR_COM_GET_FW_VER(data)  ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]))
#define SNESOR_COM_FW_VER_NONE       (0x00000000)
#define SNESOR_COM_FW_VER_DATA       (0x0A020006)

#define SNS_SPI_RETRY_NUM    5
#define SNS_SPI_RESUME_RETRY_NUM 300
#define U2DH_RESOLUTION      1024
#define U2DH_GRAVITY_EARTH   9806550

typedef union {
    uint16_t   udata16;
    int16_t    sdata16;
    uint8_t    udata8[2];
    int8_t     sdata8[2];
} Word;

typedef union {
    uint32_t   udata32;
    int32_t    sdata32;
    uint16_t   udata16[2];
    int16_t    sdata16[2];
    uint8_t    udata8[4];
    int8_t     sdata8[4];
} Long;

typedef union {
    uint8_t    ub_prm[16];
    int8_t     sb_prm[16];
    uint16_t   uw_prm[8];
    int16_t    sw_prm[8];
    uint32_t   ud_prm[4];
    int32_t    sd_prm[4];
} HCParam;

typedef union {
    uint8_t    ub_res[512];
    int8_t     sb_res[512];
    uint16_t   uw_res[256];
    int16_t    sw_res[256];
    uint32_t   ud_res[128];
    int32_t    sd_res[128];
} HCRes;

typedef struct {
    Word       cmd;
    HCParam    prm;
    uint8_t    ent;
} HostCmd;

typedef struct {
    HCRes      res;
    Word       err;
} HostCmdRes;

struct micon_bdata {
	int32_t rst_gpio;
	int32_t int_gpio;
	int32_t brmp_gpio;
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

struct temperature {
    int32_t temperature;
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

enum sensor_batch_report_e_type{
    SENSOR_COMP_BATCH = 0,
    SENSOR_COMP_FLUSH = 1,
    SENSOR_COMP_TIME  = 2,
    SENSOR_COMP_ABS   = 3,
    SENSOR_COMP_FLUSH_BF = 4
};
struct sensor_batch_str {
    char* buffer_p;
    enum sensor_batch_report_e_type repo_type;
};

enum sns_micon_batch_e_type{
    SENSOR_BATCH_TYPE_ACC = 0,
    SENSOR_BATCH_TYPE_MAG,
    SENSOR_BATCH_TYPE_GYRO,
    SENSOR_BATCH_TYPE_ORTN,
    SENSOR_BATCH_TYPE_GRV,
    SENSOR_BATCH_TYPE_ACC_LNR,
    SENSOR_BATCH_TYPE_ROT_VCTR,
    SENSOR_BATCH_TYPE_GAME_ROT_VCTR,
    SENSOR_BATCH_TYPE_MAG_ROT_VCTR,
    SENSOR_BATCH_TYPE_STEP_CNT,
    SENSOR_BATCH_TYPE_STEP_DET,
    SENSOR_BATCH_TYPE_MAX
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

extern struct mutex s_tDataMutex;
extern struct mutex s_tSpiMutex;
extern struct semaphore s_tSnsrSema;
extern struct spi_device *client_sns;
extern struct micon_bdata g_micon_bdata;
extern bool g_bDevIF_Error;
extern bool g_micon_error;
extern bool g_spi_error;
extern int32_t g_nIntIrqFlg;
extern wait_queue_head_t s_tWaitInt;
extern atomic_t g_bIsResume;
extern atomic_t g_ResetStatus;
extern atomic_t g_ShutdownStatus;
extern atomic_t g_bIsIntIrqEnable;
extern atomic_t g_nStepWide;
extern atomic_t g_nWeight;
extern atomic_t g_nVehiType;
extern atomic_t g_FWUpdateStatus;
extern uint16_t g_lastCmdError;
 

/* function ml630q790.c */
int32_t sns_spi_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size);
int32_t sns_device_write(uint8_t adr, const uint8_t *data, uint8_t size);
int32_t sns_device_read(uint8_t adr, uint8_t *data, uint16_t size);
int32_t sns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint32_t res_size, uint8_t mode, uint8_t is_fwud);
int32_t sns_spi_probe(void);

/* function sensor_micon_driver.c */
int32_t sns_logging_state(
    uint32_t type,
    uint32_t arg_timeout_ms,
    uint32_t arg_period_ms
);
int32_t sns_micon_get_batch_data(struct sensor_batch_str* batch_p);
int32_t sns_acc_activate(bool arg_iEnable);
int32_t sns_acc_read_data(struct acceleration *arg_Acc);
int32_t sns_acc_set_offset(int32_t* offsets);
int32_t sns_acc_get_offset(int32_t* offsets);
int32_t sns_acc_set_auto_cal_offset(struct acceleration* offsets);
int32_t sns_acc_get_auto_cal_offset(struct acceleration* offsets);
int32_t sns_acc_set_host_cmd(int32_t* req_cmd, int32_t* req_param);
void sns_acc_get_host_cmd(uint32_t* res);
void sns_dailys_set_vib_interlocking(int32_t mode);
int32_t sns_mag_activate(bool arg_iEnable);
int32_t sns_mag_read_data(struct geomagnetic *arg_Mag);
int32_t sns_mag_set_offset(int32_t* offsets);
int32_t sns_mag_get_offset(int32_t* offsets);
int32_t sns_mag_set_accuracy(int8_t accuracy);
int32_t sns_mag_get_accuracy(int8_t* accuracy);
int32_t sns_mag_get_static_matrix(int32_t* static_matrix);
int32_t sns_mag_set_static_matrix(int32_t* static_matrix);
int32_t sns_gyro_activate(bool arg_iEnable);
int32_t sns_gyro_read_data(struct gyroscope *arg_Gyro);
int32_t sns_gyro_set_offset(int32_t* offsets);
int32_t sns_gyro_get_offset(int32_t* offsets);
int32_t sns_maguncalib_activate(bool arg_iEnable);
int32_t sns_maguncalib_read_data(struct mag_uncalib *arg_Mag);
int32_t sns_gyrouncalib_activate(bool arg_iEnable);
int32_t sns_gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro);
void sns_mag_offset_read_data(struct geomagnetic *arg_Mag);
void sns_gyro_offset_read_data(struct gyroscope *arg_Gyro);
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
void sns_pressure_set_cal_ofs(int32_t offset);
int32_t sns_motion_activate(bool arg_iEnable);
int32_t sns_motion_start(bool arg_iStart);
int32_t sns_scount_activate(bool arg_iEnable);
int32_t sns_sdetect_activate(bool arg_iEnable);
int32_t sns_dailys_activate(bool arg_iEnable);
int32_t sns_dailys_start(bool arg_iStart);
void sns_set_pedo_param(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iVehiType);
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

int32_t sns_set_dev_param(void);
int32_t sns_initialize( void );
int32_t sns_check_sensor(void);
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
uint32_t sns_get_logging_enable(void);
int32_t sns_suspend(void);
int32_t sns_resume(void);
int32_t sns_shutdown(void);
void sns_set_period(uint32_t type,int32_t polltime, uint8_t flag);
void sns_set_app_task( uint32_t type ,uint8_t onoff);
void sns_set_app_task_batch( uint32_t type );
int sns_iio_report_event_now(enum sensor_e_type type);

void sensor_micon_init(void);
void sensor_micon_exit(void);

#endif /* SENSOR_MICOM_H */
