/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#ifndef __QLEOS_S3_H
#define __QLEOS_S3_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/string.h>



#include<linux/ql/qlspi.h>
#include<linux/ql/qlspi_linux.h>
#include<linux/ql/qlmemorymap.h>

#define DRIVER_NAME	"qleoss3"
#define DEVICE_NAME	DRIVER_NAME

#define DRV_DMESG_TAG        " ql_eos_s3 : "

#undef QL_EOS_DBG

#ifdef QL_EOS_DBG
#define QLEOS_ERR(fmt, args...) \
	pr_err(DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, \
	__LINE__, ##args)

#define QLEOS_LOG(fmt, args...) \
	pr_info(DRV_DMESG_TAG"%s %d : "fmt, __FUNCTION__, \
	__LINE__, ##args)

//#define QLEOS_DBG(fmt, args...)
//	pr_debug(DRV_DMESG_TAG"[%d]Dbg:%s: "fmt, __LINE__,
//	__FUNCTION__, ##args)
#define QLEOS_DBG(fmt, args...) \
	pr_err(DRV_DMESG_TAG"[%d]Dbg:%s: "fmt, __LINE__, \
	__FUNCTION__, ##args)

#define QLEOS_IO(fmt, args...) \
	pr_info(DRV_DMESG_TAG"%s %d : "fmt "\n", __FUNCTION__, \
	__LINE__, ##args)

#define ENTERED()	QLEOS_IO("Entered\n")
#define EXITED()	QLEOS_IO("Exited\n")

#else
#define QLEOS_ERR(fmt, args...) \
	pr_err(DRV_DMESG_TAG"%s %d : "fmt "\n", __FUNCTION__, \
	__LINE__, ##args)

#define QLEOS_LOG(fmt, args...) \
	pr_debug(DRV_DMESG_TAG"%s %d : "fmt "\n", __FUNCTION__, \
	__LINE__, ##args)

#define QLEOS_DBG(fmt, args...) \
	pr_debug(DRV_DMESG_TAG"%s %d : "fmt "\n", __FUNCTION__, \
	__LINE__, ##args)

#define QLEOS_IO(fmt, args...) \
	pr_info(DRV_DMESG_TAG"%s %d : "fmt "\n", __FUNCTION__, \
	__LINE__, ##args)

#define ENTERED()	QLEOS_DBG("Entered")
#define EXITED()	QLEOS_DBG("Exited")

#endif



#define FFE_STATUS_ADDR		0x4004A110
#define FFE_BUSY		(0x4)

//#define SENSORHUB_MM_BASE_ADDR		(0x20060000)
//#define SENSORHUB_MM_BASE_ADDR		(0x2006d000)
#define SENSORHUB_MM_BASE_ADDR			(AP_COMM_COMMAND_RESPONSE_SECTION_ADDR)

//#define AP_COMM_COMMAND_SIZE		4
//#define AP_COMM_PARAMS_SIZE			64
//#define AP_COMM_RESULT_SIZE			8
//#define AP_COMM_RESPONSE_SIZE		128

/* Addresses */
#define QL_COMMAND_ADDR		(SENSORHUB_MM_BASE_ADDR)
#define CMD_PARAM_ADDR		(SENSORHUB_MM_BASE_ADDR + AP_COMM_COMMAND_SIZE)
#define SENSOR_RESULT_ADDR	(CMD_PARAM_ADDR + AP_COMM_PARAMS_SIZE)
#define SENSOR_RESPONSE_ADDR	(SENSOR_RESULT_ADDR + AP_COMM_RESULT_SIZE)

#define COMMAND_RESPONSE_SECTION_SIZE	(AP_COMM_COMMAND_SIZE + AP_COMM_PARAMS_SIZE + AP_COMM_RESULT_SIZE + AP_COMM_RESPONSE_SIZE)

/* SECTION 2: ANDROID SENSOR DATA */
#if 0		
#define ANDROID_SENSOR_DATA_SECTION_ADDR	(SENSORHUB_MM_BASE_ADDR + COMMAND_RESPONSE_SECTION_SIZE)

#define ANDROID_SENSOR_DATA_ADDR	(ANDROID_SENSOR_DATA_SECTION_ADDR)
#define ANDROID_SENSOR_DATA_ACCEL_ADDR	(ANDROID_SENSOR_DATA_ADDR + 0x00)
#define ANDROID_SENSOR_DATA_MAG_ADDR	(ANDROID_SENSOR_DATA_ADDR + 0x08)
#define ANDROID_SENSOR_DATA_GYRO_ADDR	(ANDROID_SENSOR_DATA_ADDR + 0x10)
#define ANDROID_SENSOR_DATA_BARO_ADDR	(ANDROID_SENSOR_DATA_ADDR + 0x18)
#define ANDROID_SENSOR_DATA_STEPCOUNTER_ADDR	(ANDROID_SENSOR_DATA_ADDR + 0x60)
#endif

#define INTR_CLEAR_ADDR		(0x40004850)

#define ACCEL_DATA_ADDR		(ANDROID_SENSOR_DATA_ACCEL_ADDR)

#define MARK_OFF_DATA		(SENSOR_RESPONSE_ADDR + 12)

#if 0
#define M4_BATCH_DATA_START_ADDR	0x200615B4
#define M4_BATCH_DATA_END_ADDR		0x200695AC
#define FFE_BATCH_DATA_START_ADDR	0x200695AC
#define FFE_BATCH_DATA_END_ADDR		0x200715A4
#endif
#if 0
#define M4_BATCH_DATA_START_ADDR	0x2006d000 + 0x15B4
#define M4_BATCH_DATA_END_ADDR		0x2006d000 + 0x95AC
#define FFE_BATCH_DATA_START_ADDR	0x2006d000 + 0x95AC
#define FFE_BATCH_DATA_END_ADDR		0x2006d000 + 0x115A4

#define EVENT_BATCH_DATA_START_ADDR	0x2007EDF8
#define EVENT_BATCH_DATA_END_ADDR	0x2007EDF8 + 0x1000 //4KB
#endif

/* Using this Macro we can dynamically change the Batch and Event batch buffer start and end address.
   If we enable this, start and end address has to be stored in fixed memory. 
   Disable this macro if the both start and end address are fixed. */
#define BATCHING_DYNAMIC_MEMORY

#ifdef BATCHING_DYNAMIC_MEMORY
#define BATCH_BUF_ADDRESS			(ANDROID_BATCH_BUFFER_START_ADDR_M4_STORE)
#define EVENT_BATCH_BUF_ADDRESS		(EVENT_BATCH_BUFFER_DATA_START_ADDR_STORE)

struct batching_buff_adr {
	uint32_t m4_batch_start_addr;
	uint32_t m4_batch_end_addr;
	uint32_t ffe_batch_start_addr;
	uint32_t ffe_batch_end_addr;
};
struct evt_batching_buff_adr {
	uint32_t evt_batch_start_addr;
	uint32_t evt_batch_end_addr;
};
#else
#define M4_BATCH_DATA_START_ADDR	ANDROID_BATCH_BUFFER_SECTION_ADDR_M4
#define M4_BATCH_DATA_END_ADDR		(ANDROID_BATCH_BUFFER_SECTION_ADDR_M4 + ANDROID_BATCH_BUFFER_SECTION_SIZE_M4 + ANDROID_BATCH_BUFFER_SECTION_BLANK_M4)

#define FFE_BATCH_DATA_START_ADDR	ANDROID_BATCH_BUFFER_SECTION_ADDR_FFE
#define FFE_BATCH_DATA_END_ADDR		(ANDROID_BATCH_BUFFER_SECTION_ADDR_FFE + ANDROID_BATCH_BUFFER_SECTION_SIZE_FFE + ANDROID_BATCH_BUFFER_SECTION_BLANK_FFE)

#define EVENT_BATCH_DATA_START_ADDR	EVENT_BATCH_BUFFER_DATA_ADDR
#define EVENT_BATCH_DATA_END_ADDR	(EVENT_BATCH_BUFFER_DATA_ADDR + EVENT_BATCH_BUFFER_SECTION_SIZE)
#endif

#define HOSTCMD_RESULT_SIZE		(8)
#define HOSTCMD_RSP_SIZE_MAX	(256)
#define HOSTCMD_RSP_BUFSIZE		(HOSTCMD_RESULT_SIZE + HOSTCMD_RSP_SIZE_MAX)

/* Sensor IDs */
#define QL_SENSOR_RESERVED	(0x0)
#define QL_SENSOR_ACCEL_ID	(0x1)
#define QL_SENSOR_GYRO_ID	(0x5)
#define QL_SENSOR_MAG_ID	(0x6)
#define QL_SENSOR_BARO_ID	(0x7)


/* For resetting driver */

#define QL_CURRENT_TASK			_IO('Q', 0x0A)
#define QL_BATCH_MODR_BM_MODE		_IOW('Q', 0x0B, uint8_t)
#define QL_BATCH_MODC_BM_MODE		_IOW('Q', 0x0C, uint8_t)
#define QL_BATCH_MODR_M4BF_MODE		_IOW('Q', 0x0D, uint8_t)
#define QL_BATCH_MODC_M4BF_MODE		_IOW('Q', 0x0E, uint8_t)
#define QL_BATCH_MODR_LE_MODE		_IOW('Q', 0x0F, uint8_t)
#define QL_BATCH_MODC_LE_MODE		_IOW('Q', 0x10, uint8_t)
#define QL_BATCH_MODR_AH_MODE		_IOW('Q', 0x11, uint8_t)
#define QL_BATCH_MODC_AH_MODE		_IOW('Q', 0x12, uint8_t)
#define QL_BATCH_MODR_FFEBF_MODE	_IOW('Q', 0x13, uint8_t)
#define QL_BATCH_MODC_FFEBF_MODE	_IOW('Q', 0x14, uint8_t)
#define QL_BATCH_MODR_BSM4BF_MODE	_IOW('Q', 0x15, uint8_t)
#define QL_BATCH_MODR_BSFFEBF_MODE	_IOW('Q', 0x16, uint8_t)

#define QL_EVENT_MODR_BM_MODE		_IOW('Q', 0x17, uint8_t)
#define QL_EVENT_MODR_BF_MODE		_IOW('Q', 0x18, uint8_t)
#define QL_EVENT_MODR_LE_MODE		_IOW('Q', 0x19, uint8_t)
#define QL_EVENT_MODR_BSBF_MODE		_IOW('Q', 0x1A, uint8_t)
#define QL_EVENT_MODR_AH_MODE		_IOW('Q', 0x1B, uint8_t)

#define QL_VOICE_STREAMING_MODE		_IOW('Q', 0x1C, uint8_t)

#define QL_COMMAND(cmd)		_IOWR('Q', cmd, struct sensor_info);

#define NUM_DEV		(1)
#define SIGEVT		44

#define WAITEVENT_TIMEOUT            (6000)

#define MAX_QWORK_COUNT		100

/* Interrrupt status */
/* 0-7*/
#define INTREQ_NONE		(0x00)
#define INTREQ_HOST_CMD		(1 << 0)
#define INTREQ_BOOT_DONE	(1 << 1)

#define ACCEL_CAL_DONE		(1 << 4)
#define MAG_CAL_DONE		(1 << 5)
#define GYRO_CAL_DONE		(1 << 6)
#define SENSOR_CAL_DONE		(ACCEL_CAL_DONE | MAG_CAL_DONE | \
				GYRO_CAL_DONE)

/* 8-15*/
#define SIGN_MOTION_DETECTED	(1 << 8)
#define ROT_CHANGE_DETECTED	(1 << 9)

/* 16-31*/
#define KPT2_DETECTED		(1 << 15)
#define KPT1_DETECTED		(1 << 16)

#define PD_STEP						(1 << 17)
#define VC_DETECTED					(1 << 18)
#define PS_GRADCNG					(1 << 19)
#define SD_CNG						(1 << 20)
#define SP_CNG						(1 << 21)
#define BW_CNG						(1 << 22)
#define BU_CNG						(1 << 23)
#define TR_DETECTED					(1 << 24)
#define TR_FST_DETECTED				(1 << 25)
#define PD_FST_OTH_DETECTED			(1 << 26)
#define UWD_WATER_DETECTED			(1 << 27)
#define UWD_TARGET_DEPTH_DETECTED	(1 << 28)
#define UWD_MAX_DEPTH_DETECTED		(1 << 29)

#define M4_APP_ALGO_EVENTS 	(PD_STEP | VC_DETECTED | PS_GRADCNG | SD_CNG  | \
				SP_CNG | BW_CNG | BU_CNG | SIGN_MOTION_DETECTED| TR_DETECTED |\
				TR_FST_DETECTED | PD_FST_OTH_DETECTED | UWD_WATER_DETECTED | \
				UWD_TARGET_DEPTH_DETECTED | UWD_MAX_DEPTH_DETECTED)


/*32-39*/
#define M4_BUFFER_FULL_INTR	(1ULL << 32)
#define REMAIN_MARGIN_INTR	(1ULL << 33)
#define MAX_LATENCY_INTR	(1ULL << 34)
#define M4_BUF_BLANK_FULL	(1ULL << 35)
#define FFE_BUFFER_FULL_INTR	(1ULL << 36)
#define FFE_BUF_BLANK_FULL	(1ULL << 37)
#define BATCH_EVENTS		(M4_BUFFER_FULL_INTR | REMAIN_MARGIN_INTR | \
				MAX_LATENCY_INTR | M4_BUF_BLANK_FULL | \
				FFE_BUFFER_FULL_INTR | FFE_BUF_BLANK_FULL)

/*40-43*/
#define EVENT_BUFFER_FULL_INTR	(1ULL << 40)
#define EVENT_MARGIN_INTR	(1ULL << 41)
#define EVENT_MAX_LAT_INTR	(1ULL << 42)
#define EVENT_BLANK_FULL_INTR	(1ULL << 43)
#define EVENT_BUF_EVENTS	(EVENT_BUFFER_FULL_INTR | EVENT_MARGIN_INTR | \
				EVENT_MAX_LAT_INTR | EVENT_BLANK_FULL_INTR)

/*46 - 47 */
#define VOICE_DATA_READY	(1ULL << 46)
#define VOICE_STREAM_END	(1ULL << 47)
#define AUDIO_STREAMING_EVENTS	(VOICE_DATA_READY | VOICE_STREAM_END)

/* 56-63 */
#define WDT_EXPIRED			(1ULL << 59)
#define HOST_CMD_ERR0		(1ULL << 60)
#define HOST_CMD_ERR1		(1ULL << 61)
#define HOST_CMD_ERR2		(1ULL << 62)

#define HOST_CMD_ERR		(HOST_CMD_ERR0 | HOST_CMD_ERR1 | HOST_CMD_ERR2)

#define INTREQ_ERROR		(1ULL << 63)

#define ALL_HOST_RESP		(INTREQ_HOST_CMD | INTREQ_BOOT_DONE | HOST_CMD_ERR | WDT_EXPIRED | INTREQ_ERROR)

#define KPT_DETECTED		(KPT1_DETECTED | KPT2_DETECTED)
#if 0
#define SENSOR_ALL_EVENTS	(SENSOR_CAL_DONE | SIGN_MOTION_DETECTED | \
				ROT_CHANGE_DETECTED | KPT_DETECTED | \
				BATCH_EVENTS | INTREQ_ERROR | INTREQ_BOOT_DONE | \
				EVENT_BUF_EVENTS)
#endif
#define SENSOR_ALL_EVENTS	(SENSOR_CAL_DONE | SIGN_MOTION_DETECTED | \
				ROT_CHANGE_DETECTED | KPT_DETECTED | \
				BATCH_EVENTS | EVENT_BUF_EVENTS | \
				M4_APP_ALGO_EVENTS | AUDIO_STREAMING_EVENTS)

/* Data offsets */

#define IRQ_PROC_PRIORITY1	(INTREQ_HOST_CMD)
#define IRQ_PROC_PRIORITY2	(INTREQ_BOOT_DONE | HOST_CMD_ERR | INTREQ_ERROR)
#define IRQ_PROC_PRIORITY3	(ROT_CHANGE_DETECTED | M4_BUFFER_FULL_INTR | \
							REMAIN_MARGIN_INTR | MAX_LATENCY_INTR | \
							M4_BUF_BLANK_FULL | FFE_BUFFER_FULL_INTR | \
							FFE_BUF_BLANK_FULL)
#define IRQ_PROC_PRIORITY4	(KPT_DETECTED | AUDIO_STREAMING_EVENTS)
#define IRQ_PROC_PRIORITY5	(EVENT_BUF_EVENTS)
#define IRQ_PROC_PRIORITY6	(SENSOR_CAL_DONE | M4_APP_ALGO_EVENTS)

#define HC_SCALE_CHANGE			0x00000900
#define HC_CRC_CHECK			0x00000300

/***************** 7-0 command bits ***************/
/* sensor device control commands */
#define SDC_EXECUTE_PROCESS_OF_INIT		0x00
#define SDC_SET_DYNAMIC_RANGE			0x01
#define SDC_SET_ODR				0x02
#define SDC_SET_MEASUREMENT_MODE		0x03

#define SDC_GET_DYNAMIC_RANGE			0x81
#define SDC_GET_ODR				0x82
#define SDC_GET_MEASUREMENT_MODE		0x83

/* Sensor processing control command */
#define SPC_CHANGE_AXIS_DIRECTION		0x00
#define SPC_SET_INITIAL_WAIT_TIME		0x01
#define SPC_ENABLE_CALIBRATION			0x02
#define SPC_ENABLE_UPDATE_OF_CAL_VALUE		0x03
#define SPC_ENABLE_CAL_INTERRUPT		0x04
#define SPC_SET_CALIBRATION_VALUE		0x05
#define SPC_ENABLE_LPF				0x06
#define SPC_SET_LPF_COEFFICIENT			0x07
#define SPC_SET_SOFT_IRON_CAL_VALUE		0x08
#define SPC_SET_PERIOD_FOR_UPDATE_CAL_VAL	0x09

#define SPC_GET_AXIS_DIRECTION			0x80
#define SPC_GET_INITIAL_WAIT_TIME		0x81
#define SPC_GET_ENABLE_CALIBRATION		0x82
#define SPC_GET_ENABLE_UPDATE_OF_CAL_VALUE	0x83
#define SPC_GET_ENABLE_CAL_INTERRUPT		0x84
#define SPC_GET_CALIBRATION_VALUE		0x85
#define SPC_GET_ENABLE_LOW_PASS_FILTER		0x86
#define SPC_GET_LPF_COEFFICIENT			0x87
#define SPC_GET_SOFT_IRON_CAL_VALUE		0x88
#define SPC_GET_PERIOD_FOR_UPDATE_CAL_VAL	0x89

/* Sensor task control commands */
/* Every sensor has specific bit in parameter */
#define STC_ENABLE_SENSOR_TASK			0x00
#define STC_SET_SENSOR_TASK_PERIOD		0x02
#define STC_ENABLE_SENSOR_PROCESSING		0x00
#define STC_SET_SENSOR_PERIOD			0x01
#define STC_ENABLE_SENSOR_BATCHING		0x00
#define STC_SET_SENSOR_BATCH_PERIOD		0x01

#define STC_GET_ENABLE_SENSOR_TASK		0x80
#define STC_GET_SENSOR_TASK_PERIOD		0x82
#define STC_GET_ENABLE_SENSOR_PROCESSING	0x80
#define STC_GET_SENSOR_PERIOD			0x81
#define STC_GET_ENABLE_SENSOR_BATCHING		0x80
#define STC_GET_SENSOR_BATCH_PERIOD		0x81

/* Android sensor control commands */
/* Every sensor has specific bit in parameter */
#define ASC_ENABLE_M4_FUSION_TASK			0x00
#define ASC_SET_INITIAL_WAIT_TIME			0x01
#define ASC_SET_M4_FUSION_TASK_PERIOD		0x02
#define ASC_ENABLE_FFE_FUSION_TASK			0x03
#define ASC_SET_FFE_FUSION_INIT_WAIT_TIME	0x04
#define ASC_SET_FFE_FUSION_PERIOD			0x05
#define ASC_SET_BUF_SIZE_FFE_TO_M4			0x06
#define ASC_CLEAR_FFE_FUSION_BUF			0x07

#define ASC_ENABLE_INDIVIDUAL_CAL		0x00
#define ASC_SET_INDIVIDUAL_PERIOD		0x01
#define ASC_ENABLE_INDIVIDUAL_BATCH		0x00
#define ASC_SET_BATCH_PERIOD			0x01

#define ASC_GET_ENABLE_M4_FUSION_TASK		0x80
#define ASC_GET_INITIAL_WAIT_TIME			0x81
#define ASC_GET_M4_FUSION_TASK_PERIOD		0x82
#define ASC_GET_ENABLE_FFE_FUSION_TASK		0x83
#define ASC_GET_FFE_FUSION_INIT_WAIT_TIME	0x84
#define ASC_GET_FFE_FUSION_PERIOD			0x85
#define ASC_GET_BUF_SIZE_FFE_TO_M4			0x86

#define ASC_GET_ENABLE_INDIVIDUAL_CAL		0x80
#define ASC_GET_INDIVIDUAL_PERIOD		0x81
#define ASC_GET_ENABLE_INDIVIDUAL_BATCH		0x80
#define ASC_GET_BATCH_PERIOD			0x81

/* System commands */
#define SYS_ERASE_INTERRUPT_FACTOR		0x00

#define SYS_EXEC_ARBITARY_WRITE_PROCESS		0x00
#define SYS_EXEC_ARBITARY_READ_PROCESS		0x01
#define SYS_GET_ERROR_FACTOR			0x00
#define SYS_EXEC_CRC_CHECK_PROCESS		0x00
#define SYS_GET_RESET_LOG			0x00
#define SYS_SET_SPI_SCALE_FREQ		0x00
#define SYS_GET_FW_VERSION		0x80

/* Batch mode control command */
#define BMC_MARK_OFF_BATCHING_DATA		0x00
#define BMC_GET_BATCHING_DATA			0x01
#define BMC_ALL_CLEAR_BATCHING_DATA		0x02
#define BMC_ENABLE_BUF_FULL_INTERRUPT		0x03
#define BMC_ENABLE_BATCH_INTERRUPT		0x04
#define BMC_SET_MAX_LATENCY			0x05
#define BMC_GET_ENABLE_BUF_FULL_INTERRUPT	0x83
#define BMC_GET_ENABLE_BATCH_INTERRUPT	0x84
#define BMC_GET_MAX_LATENCY			0x85

/* FFE Timer control commands */
#define FTC_ENABLE_FFE_TIMER			0x00
#define FTC_SET_FFE_TIMER_PERIOD		0x01

#define FTC_GET_ENABLE_FFE_TIMER		0x80
#define FTC_GET_FFE_TIMER_PERIOD		0x81

/* FFE application task commands */
#define FAT_ENABLE_FFE_APPLICATION		0x00
#define FAT_SET_FFE_INIT_WAIT_TIME		0x01
#define FAT_SET_FFE_APP_TASK_PERIOD		0x02

#define FAT_GET_ENABLE_FFE_APPLICATION		0x80
#define FAT_GET_FFE_INIT_WAIT_TIME		0x81
#define FAT_GET_FFE_APP_TASK_PERIOD		0x82

/* Application commands */
#define APP_PCG_ENABLE_FUNCTION			0x00
#define APP_PCG_SET_PARAMETER			0x01
#define APP_PCG_SET_CLEAR			0x02
#define APP_PCG_GET_ENABLE_FUNCTION		0x80
#define APP_PCG_GET_NUM_OF_STEPS		0x83

#define APP_DOUBLE_TAP_ENABLE_FUNC		0x00
#define APP_DOUBLE_TAP_GET_ENABLE_FUNC		0x80

/* Application Commands New */
#define APP_ALL					0x00
	#define APP_ALL__AppInfoClear			0x00
#define APP_DS					0x01
	#define APP_DS__DS_OnOff			0x00
	#define APP_DS__DS_GetOnOff			0x80
	#define APP_DS__PD_GetInfo			0x81 // 0x90 ?
	#define APP_DS__VC_GetInfo			0x82 // 0x91 ?
#define APP_DSPLUS				0x02
	#define APP_DSPLUS__WF_OnOff			0x00
	#define APP_DSPLUS__WF_Setting			0x01
	#define APP_DSPLUS__SP_OnOff			0x03 // 0x10 ?
	#define APP_DSPLUS__BW_OnOff			0x04 // 0x11 ?
	#define APP_DSPLUS__TR_OnOff			0x05 // 0x20 ?
	#define APP_DSPLUS__WF_GetOnOff			0x80
	#define APP_DSPLUS__WF_GetSetting		0x81
	#define APP_DSPLUS__WF_GetInfo			0x82
	#define APP_DSPLUS__SP_GetOnOff			0x83 // 0x90 ?
	#define APP_DSPLUS__BW_GetOnOff			0x84 // 0x91 ?
	#define APP_DSPLUS__TR_GetOnOff			0x85 // 0xA0 ?
	#define APP_DSPLUS__TR_GetInfo			0x86 // 0xA1 ?
#define APP_OTHER				0x03
	#define APP_OTHER__DS_VibOnOff			0x00
	#define APP_OTHER__DC_OnOff			0x01
	#define APP_OTHER__DS_GetVibOnOff		0x80
	#define APP_OTHER__DC_GetOnOff			0x81
#define APP_EBUF				0x04
	#define APP_EBUF__Log_Setting			0x00
	#define PP_EBUF__Log_OptSetting			0x01
	#define APP_EBUF__Log_PrsSetting		0x02
	#define APP_EBUF__Log_HeightSetting		0x03
	#define APP_EBUF__EventBufMarkOff		0x04
	#define APP_EBUF__Log_GetSetting		0x80
	#define APP_EBUF__Log_GetOptSetting		0x81
	#define APP_EBUF__Log_GetPrsSetting		0x82
	#define APP_EBUF__Log_GetHeightSetting		0x83
	#define APP_EBUF__EventBufGetRtInfo		0x85

#define APP_AR					0x05
	#define APP_AR__AR_StepOnOff			0x01
	#define APP_AR__AR_SigMotOnOff			0x02
	#define APP_AR__AR_GetStepOnOff			0x81
	#define APP_AR__AR_GetSigMotOnOff		0x82
#define APP_BR					0x06
	#define APP_BR__BR_LogSetting			0x00
	#define APP_BR__BR_LogPrsSetting		0x01
	#define APP_BR__BR_GetLogSetting		0x80
	#define APP_BR__BR_GetLogPrsSetting		0x81
#define APP_PS					0x07
	#define APP_PS__PS_HeightSetting		0x00
	#define APP_PS__PS_GetHeightSetting		0x80
	#define APP_PS__PS_GetInfo			0x81
#define APP_UWD					0x08
	#define APP_UWD__UWD_Setting			0x00
	#define APP_UWD__UWD_GetSetting			0x80
	#define APP_UWD__UWD_GetInfo			0x81
#define APP_GY					0x09
	#define APP_GY__GyroCalParamSetting		0x00
	#define APP_GY__GetGyroCalParamSetting		0x80
#define APP_FFEAPP				0x0A
	#define APP_FFEAPP__VH_OnOff			0x00
	#define APP_FFEAPP__SD_OnOff			0x02
	#define APP_FFEAPP__BU_OnOff			0x04
	#define APP_FFEAPP__VH_GetOnOff			0x80
	#define APP_FFEAPP__VH_GetInfo			0x81
	#define APP_FFEAPP__SD_GetOnOff			0x82
	#define APP_FFEAPP__SD_GetInfo			0x83
	#define APP_FFEAPP__BU_GetOnOff			0x84
	#define APP_FFEAPP__BU_GetInfo			0x85
#define VOICE_KPD				0x00
	#define SET_EN					0x00
	#define SET_MODEL_INFO				0x01
	#define GET_EN					0x80
	#define GET_MODEL_INFO				0x81
#define VOICE_CFG				0x01

#define VOICE_DATA				0x02
	#define START_AUDIO_STREAM			0x01
	#define AUDIO_MARK_OFF				0x02

/* M4 Application Task commands */
#define M4AT_ENABLE_M4_APP_TASK			0x00
//#define M4AT_SET_INIT_WAIT_TIME			0x01
//#define M4AT_SET_M4_APP_TASK_PERIOD		0x02
#define M4AT_SET_M4_APP_TASK_PERIOD		0x01
#define M4AT_SET_BUF_SIZE_FFE_TO_M4		0x02
#define M4AT_SET_BUF_SIZE_M4_TO_FFE		0x04

#define M4AT_GET_ENABLE_M4_APP_TASK		0x80
//#define M4AT_GET_INIT_WAIT_TIME			0x81
//#define M4AT_GET_M4_APP_TASK_PERIOD		0x82
#define M4AT_GET_M4_APP_TASK_PERIOD		0x81
#define M4AT_GET_BUF_SIZE_FFE_TO_M4		0x82
#define M4AT_GET_BUF_SIZE_M4_TO_FFE		0x84

#define VOICE_SET_KPD_ENABLE			0x00
#define VOICE_GET_KPD_ENABLE			0x80

/********************************************************/

#define ENABLE		(1)
#define DISABLE		(0)

/* */
#define STC_ACCEL_PROCESSING		(0)
#define STC_MAGNET_PROCESSING		(1)
#define STC_GYRO_PROCESSING		(2)
#define STC_BARO_PROCESSING		(3)

/* */
#define STC_ACCEL_BATCHING		(0)
#define STC_MAGNETIC_FEILD_BATCHING	(1)
#define STC_MF_UNCALIBRATED_BATCHING	(2)
#define STC_GYRO_BATCHING		(3)
#define STC_GYRO_UNCALIBRATED_BATCHING	(4)
#define STC_PRESSURE_BATCHING		(5)

/* */
#define ASC_ROTATION_VECT_CALC		(0)
#define ASC_GEO_ROTATION_VECT_CALC	(1)
#define ASC_GAME_ROTATION_VECT_CALC	(2)
#define ASC_ORIENTATION_CALC		(3)
#define ASC_GRAVITY_CALC		(4)
#define ASC_LINEAR_ACCEL_CALC		(5)
#define ASC_MAGNETOMETER_CALIB		(6)
#define ASC_STEP_COUNTER_CALC		(7)
#define ASC_STEP_DETECTOR_CALC		(8)
#define ASC_SIGNIFICANT_MOTION		(9)

#define ASC_ROTATION_VECT_BATCH		(0)
#define ASC_GEO_ROTATION_VECT_BATCH	(1)
#define ASC_GAME_ROTATION_VECT_BATCH	(2)
#define ASC_ORIENTATION_BATCH		(3)
#define ASC_GRAVITY_BATCH		(4)
#define ASC_LINEAR_ACCEL_BATCH		(5)
#define ASC_STEP_COUNTER_BATCH		(6)
#define ASC_STEP_DETECTOR_BATCH		(7)

enum sensor_measurement_mode {
	LP_MODE = 0x00,
	HP_MODE
};

/*************** 15-8 command bits *************/
enum ql_sensor_type {
	ACCELEROMETER	=	0x00,
	MAGNETOMETER,
	GYROSCOPE,
	BAROMETER,
	PHYSICAL_SENS_MAX
};

#define TASK				0x00
#define STC_INDIVIDUAL_MEAS		0x01
#define STC_INDIVIDUAL_BATCH		0x02
#define ASC_INDIVIDUAL_CALC		0x01
#define ASC_INDIVIDUAL_BATCH		0x02

#define SYS_INTERRUPT			0x00
#define SYS_I2C				0x01
#define SYS_ERROR			0x02
#define SYS_CRC_CHECK			0x03
#define SYS_RESET_LOG			0x04
#define SYS_SPI_SCALING			0x09
#define SYS_FW_VERSION			0x0F

#define TIMER				0x00
#define BATCH				0x00

#define PCG				0x00
#define DOUBLE_TAP			0x01

#define KPD_DETECTION	0x00
/**********************************************/

/************* 23-16 coomand bits *************/
enum sensor_host_cmd_purpose {
	SYSTEM = 0,
	SENSOR_DEVICE_CONTROL = 1,
	SENSOR_PROCESS_CONTROL,
	SENSOR_TASK_CONTROL,
	ANDROID_SENSOR_TASK_CONTROL,
	BATCH_MODE_CONTROL,
	FFE_TIMER_CONTROL,
	FFE_APP_TASK,
	APPLICATION,
	M4_APP_TASK,
	VOICE,
	APP
};

enum firmware_type {
    KC_M4_L = 0,    /* KC_M4_Version_Low */
    KC_M4_H,        /* KC_M4_Version_High */
    QL_M4_L,        /* QL_M4_Version_Low */
    QL_M4_H,        /* QL_M4_Version_High */
    KC_FFE_L,        /* KC_FFE_Version_Low */
    KC_FFE_H,        /* KC_FFE_Version_High */
    QL_FFE_L,        /* QL_FFE_Version_Low */
    QL_FFE_H,        /* QL_FFE_Version_High */
    KC_FABRIC_L,    /* KC_FABRIC_Version_Low */
    KC_FABRIC_H,    /* KC_FABRIC_Version_High */
    QL_FABRIC_L,    /* QL_FABRIC_Version_Low */
    QL_FABRIC_H,    /* QL_FABRIC_Version_High */
    FW_TYPE_MAX
};

/**********************************************/

typedef union {
	uint8_t ub_res[8];
	uint16_t uw_res[4];
	uint32_t ud_res[2];
}cmd_result;

struct cmd_result_res {
	cmd_result result;
	uint8_t *response;
};

union sensor_host_command {
	struct {
		unsigned long command:8;
		unsigned long type:8;
		unsigned long purpose:8;
		unsigned long reserved:8;
	}cmd_struct;
	uint32_t host_command;
};


struct sensor_period {
	uint32_t accel;
	uint32_t mag;
	uint32_t gyro;
	uint32_t baro;
};

struct sensor_calc_period {
	uint32_t rot_vec;
	uint32_t geo_rot_vec;
	uint32_t game_rot_vec;
	uint32_t orientation;
	uint32_t gravity;
	uint32_t lin_accel;
	uint32_t mag_cal;
};

struct sensor_batch_period {
	uint32_t accel_batch_period;
	uint32_t mf_batch_period;
	uint32_t mf_uncal_batch_period;
	uint32_t gyro_batch_period;
	uint32_t gyro_uncal_batch_period;
	uint32_t pressure_batch_period;
};

struct android_sens_batch_period {
	uint32_t rot_vec_batch;
	uint32_t geo_rot_vec_batch;
	uint32_t game_rot_vec_batch;
	uint32_t orient_batch;
	uint32_t gravity_batch;
	uint32_t lin_accel_batch;
};

struct batch_markoff_param {
	uint8_t m4_buf_lock;
	uint8_t update_m4_rd_ptr;
	uint32_t cur_m4_rd_ptr;
	uint8_t ffe_buf_lock;
	uint8_t update_ffe_rd_ptr;
	uint32_t cur_ffe_rd_ptr;
}__attribute__((__packed__));

struct batch_markoff_resp {
	uint32_t m4_batch_bytes;
	uint32_t ffe_batch_bytes;
	uint32_t sens_task_ts;
	uint32_t app_task_ts;
	uint32_t cur_m4_rd_ptr;
	uint32_t cur_ffe_rd_ptr;
};

struct event_batch_markoff_param {
	uint8_t evt_buf_lock;
	uint8_t update_evt_rd_ptr;
	uint32_t cur_rd_ptr;
}__attribute__((__packed__));

struct event_batch_markoff_resp {
	uint8_t evt_buf_lock;
	uint8_t update_evt_rd_ptr;
	uint32_t cur_rd_ptr;
	uint16_t valid_data_bytes;
	uint16_t valid_events;
	uint32_t timestamp;
	uint8_t pd_rn_state;
	uint32_t pd_rn_section_step_num;
	uint32_t pd_rn_section_step_cal;
	uint8_t vc_cy_state;
	uint32_t vc_cy_section_cal;
	uint16_t pd_weight;
	uint8_t is_cleared;
	uint16_t clr_timestamp;
}__attribute__((__packed__));
#if 0
struct hc_info {
	uint32_t command;
	uint32_t param_data[8];
	int param_data_len;
	uint8_t *response;
	int response_len;
};
#endif
struct hc_info {
	uint32_t command;
	uint32_t param_data[8];
	int param_data_len;
	uint8_t response[HOSTCMD_RSP_SIZE_MAX];
	int response_len;
};

enum calib_mode {
	MF_UNCALIB = 1,
	GYRO_UNCALIB
};

typedef struct t_qleoss3_workqueue {
    struct work_struct  work;
    bool                status;
    uint64_t result;
} s_workqueue;

enum reboot_proc_e_type {
	REBOOT_TYPE_INIT = 0,
	REBOOT_TYPE_SW,
	REBOOT_TYPE_HW,
	REBOOT_TYPE_UNEXPECT
};


/* Function Prototypes */
int get_boot_mode(void);
QL_Status mcu_fw_download(void);
int send_command_to_s3(struct hc_info *hc, uint32_t command);
int qleos_s3_create_sysfs(void);
void qleos_s3_remove_sysfs(void);
//int qleoss3_open(struct inode *inode, struct file *filp);
int boot_sequence(void);
uint32_t prepare_host_command(uint8_t purpose, uint8_t type, uint8_t cmd);
void reset_driver(void);
//void batch_config(int batch_mode);
int set_command(uint32_t command, int length, uint8_t *param, uint8_t *result);
int get_command(uint32_t command, int length, uint8_t *response);
int set_get_command(uint32_t command, int length, uint8_t *param, int res_len, uint8_t *reponse);
void sys_wait_for_intr(int intr_num);
int wait_for_response(uint64_t result_mask);
int wait_for_response_kpd(uint64_t result_mask_kpd);
int send_erase_interrupt_factor_command(const uint64_t ack);
int qleoss3_hostcmd(struct hc_info *ihc);
int qleoss3_set_get_hostcmd(struct hc_info *ihc, int mode);
uint8_t qleoss3_get_shutdown_status(void);
void qleoss3_set_flush(int32_t type);
//QL_Status write_s3_mem(uint32_t addr, void *data, uint32_t len);
//QL_Status read_s3_mem(uint32_t addr, void *data, uint32_t len);

int check_crc_error(void);
void hw_reboot_exec(void);
void qleoss3_request_hw_reboot(char *client_name);

int64_t get_ev_markoff_ktime(void);
void fwdl_sequence_when_recovering(void);

#endif	//__QLEOS_S3_H
