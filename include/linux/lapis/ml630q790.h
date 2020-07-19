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

#ifndef __ML630Q790_H_INCLUDED
#define __ML630Q790_H_INCLUDED

#include <linux/device.h>
#include <linux/kernel.h>

#undef ML630Q790_DRIVER_DEBUG

/* ******************************************************
  Log Macro
****************************************************** */
#ifdef ML630Q790_DRIVER_DEBUG
#define ML630Q790_I_LOG(msg, ...) \
    pr_notice("[ml630q790][%s][I](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ML630Q790_D_LOG(msg, ...) \
    pr_notice("[ml630q790][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define ML630Q790_I_LOG(msg, ...) \
    pr_info("[ml630q790][%s][I](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
    
#define ML630Q790_D_LOG(msg, ...) \
    pr_debug("[ml630q790][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif
#define ML630Q790_ERR_LOG(msg, ...)\
    pr_err("[ml630q790][%s][ERR](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#ifdef ML630Q790_DRIVER_DEBUG
#define ML630Q790_D_DUMP(prefix_str, prefix_type, buf, len) \
    print_hex_dump(KERN_NOTICE, prefix_str, prefix_type, 16, 1, buf, len, false)
#else
#if defined(CONFIG_DYNAMIC_DEBUG)
#define ML630Q790_D_DUMP(prefix_str, prefix_type, buf, len) \
    dynamic_hex_dump(prefix_str, prefix_type, 16, 1, buf, len, false)
#else
#define ML630Q790_D_DUMP(prefix_str, prefix_type, buf, len)
#endif /* defined(CONFIG_DYNAMIC_DEBUG) */
#endif

/* ******************************************************
  DEFINE
****************************************************** */
#define SENSOR_MICON_DRIVER_NAME    "sensor_micon"
#define ML630Q790_SYSFS_FOLDER      "ml630q790"
#define GPIO_INT_NAME               "sns_irq"
#define GPIO_RESET_NAME             "sns_reset"
#define GPIO_BRMP_NAME              "sns_brmp"

#define ML630Q790_GET_FW_VER(data)  ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]))
#define ML630Q790_FW_VER_NONE       (0x00000000)
#define ML630Q790_FW_VER_DATA       (0x0E020005)
#define ML630Q790_FW_VER_CHK_OK     (0x00)
#define ML630Q790_FW_VER_CHK_NG     (0xFF)
#define ERROR_FUP_CERTIFICATION     (0x0013u)

#define READ_RSLT                (1)
#define READ_FIFO                (0)

#define ML630Q790_RC_OK                 (0)
#define ML630Q790_RC_ERR                (-1)
#define ML630Q790_RC_ERR_TIMEOUT        (-2)
#define ML630Q790_RC_ERR_RAMWRITE       (-3)
#define ML630Q790_RC_ERR_INVALID_ID     (-4)

/*
 * ML630Q790 INTREQ FACTOR
 */
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
#define INTREQ_DUMMY                 (0x0000u)
/*
 * ML630Q790 Register Map
 */
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

/*
 * ML630Q790 HOST COMMAND ID
 */
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
#define HC_ACC_SET_SIMPLE_FILTER     (0x100Cu)
#define HC_ACC_GET_SIMPLE_FILTER     (0x100Du)
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
#define HC_EC_SET_SIMPLE_FILTER      (0x700Fu)
#ifdef CONFIG_INPUT_SENSOR_YAS_MAG
#define HC_MAG_SET_OFFSET            (0x70F0u)
#define HC_MAG_GET_OFFSET            (0x70F1u)
#define HC_MAG_SET_PERIOD            (0x70F4u)
#define HC_MAG_SET_FILTER            (0x70F6u)
#elif defined(CONFIG_INPUT_SENSOR_HSCDTD008A_MAG)
#define HC_MAG_SET_PARAM_HSCDTD008A  (0x70F0u)
#define HC_MAG_GET_PARAM_HSCDTD008A  (0x70F1u)
#endif
#define HC_GYRO_SET_DATA             (0x8002u)
#define HC_GYRO_GET_DATA             (0x8003u)
#define HC_GYRO_SET_CALIB            (0x8004u)
#define HC_GYRO_SET_CONV_AXIS        (0x8009u)
#define HC_GYRO_SET_SIMPLE_FILTER    (0x800Bu)
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
#define HC_MUL_SET_LOGGING_TIMEOUT   (0xF023u)
#define HC_MUL_SET_LOGGING_NOTIFY    (0xF01Bu)

#define EXE_HOST_WAIT                (1)
#define EXE_HOST_RES                 (2)
#define EXE_HOST_ERR                 (4)
#define EXE_HOST_ALL                 (EXE_HOST_WAIT|EXE_HOST_RES|EXE_HOST_ERR)
#define EXE_HOST_NO_RES              (EXE_HOST_WAIT|EXE_HOST_ERR)
#define EXE_HOST_EX_NO_RECOVER       (16)

#define KC_LOG_READ                  (0x1F01u)
#define KC_LOG_SIZE                  (2 + 256)

#define FUP_MAX_RAMSIZE              (512)
#define LOGGING_FIFO_SIZE            (4096)
#define LOGGING_RESPONSE_HEADER      (58)
#define EVENTBUF_FIFO_SIZE           (3072)
#define EVENTBUF_RESPONSE_HEADER     (22)
#define RESET_RETRY_NUM              (5)

#define WORK_OPT_NONE       0
#define WORK_OPT_INT        1
#define WORK_OPT_BATCH      2
#define WORK_OPT_FLUSH      3

enum ml630q790_drv_status_e_type{
    ML630Q790_POWER_OFF = 0,
    ML630Q790_FW_UPDATING,
    ML630Q790_NORMAL,
    ML630Q790_SUSPEND,
    ML630Q790_FAILURE_FW_UPDATE,
    ML630Q790_FAILURE_ACCESS,
    ML630Q790_SHUTDOWN,
    ML630Q790_RESET,
};

struct micon_bdata {
	int32_t rst_gpio;
	int32_t int_gpio;
	int32_t brmp_gpio;
};

typedef union {
    uint32_t   udata32;
    int32_t    sdata32;
    uint16_t   udata16[2];
    int16_t    sdata16[2];
    uint8_t    udata8[4];
    int8_t     sdata8[4];
} Long;

typedef union {
    uint8_t    ub_prm[64];
    int8_t     sb_prm[64];
    uint16_t   uw_prm[32];
    int16_t    sw_prm[32];
    uint32_t   ud_prm[16];
    int32_t    sd_prm[16];
} HCParam;

typedef union {
    uint8_t    ub_res[1024];
    int8_t     sb_res[1024];
    uint16_t   uw_res[512];
    int16_t    sw_res[512];
    uint32_t   ud_res[256];
    int32_t    sd_res[256];
} HCRes;

typedef union {
    uint16_t   udata16;
    int16_t    sdata16;
    uint8_t    udata8[2];
    int8_t     sdata8[2];
} Word;

typedef struct {
    Word       cmd;
    HCParam    prm;
    uint8_t    ent;
} HostCmd;

typedef struct {
    HCRes      res;
    Word       err;
} HostCmdRes;

extern struct mutex micon_hcres_mutex;
extern atomic_t g_FWUpdateStatus;
extern atomic_t g_MiconDebug;
extern int32_t g_nFWVersion;
extern bool g_bDevIF_Error;
extern bool g_micon_error;

/*
 * ML630Q790 Dedicated API
 */
void ml630q790_ope_wake_irq(bool enable);
void ml630q790_sns_set_flush(int32_t type);
/* function ml630q790.c */
int32_t sns_spi_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size);
int32_t ml630q790_device_write(uint8_t adr, const uint8_t *data, uint8_t size);
int32_t ml630q790_device_read(uint8_t adr, uint8_t *data, uint16_t size);
int32_t ml630q790_initialize_micondrv(void);
uint8_t ml630q790_get_shutdown_status(void);
int32_t ml630q790_drv_get_status(void);
int32_t sns_spi_probe(void);

//temporary...
int32_t sns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint32_t res_size, uint8_t mode, uint8_t is_fwud);
#endif		/* __ML630Q790_H_INCLUDED */
