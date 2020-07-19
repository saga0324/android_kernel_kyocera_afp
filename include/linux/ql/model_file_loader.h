#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

#include "qlmcu.h"
#include "qltrace.h"

#include "qlspi_linux.h"

#include "qleos_s3.h"


#define QL_LDR_DRV_DMESG_TAG	" ql_spi : M_Loader "

#define L_DRIVER_NAME		"model_file_loader"
#define L_DEVICE_NAME		L_DRIVER_NAME
#define INIT_MODEL_LOADER	0
#define DONE_MODEL_LOADER	1

#define DEFAULT		0
#define NET_FILE_TURN	1
#define GRAM_FILE_TURN	2

#define NET_FILE_SIZE	128*1024
#define GRAM_FILE_SIZE	128*1024


#define M_LDR_DEBUG_EN

#ifdef M_LDR_DEBUG_EN
#define MLDR_ERR(fmt, args...) \
		pr_err(QL_LDR_DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, \
		__LINE__, ##args)
#define MLDR_LOG(fmt, args...) \
        pr_info(QL_LDR_DRV_DMESG_TAG"%s %d : "fmt, __FUNCTION__, \
		__LINE__, ##args)
#define MLDR_DBG(fmt, args...) \
        pr_info(QL_LDR_DRV_DMESG_TAG"[%d]Dbg:%s: "fmt, __LINE__, \
		__FUNCTION__, ##args)
#else
#define MLDR_ERR(fmt, args...) \
		pr_err(QL_LDR_DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, \
		__LINE__, ##args)
#define MLDR_LOG(fmt, args...) \
        pr_info(QL_LDR_DRV_DMESG_TAG"%s %d : "fmt, __FUNCTION__, \
		__LINE__, ##args)
#define MLDR_DBG(fmt, args...) \
        pr_debug(QL_LDR_DRV_DMESG_TAG"[%d]Dbg:%s: "fmt, __LINE__, \
		__FUNCTION__, ##args)
#endif

struct net_gram_resp {
	uint32_t net_addr;
	uint32_t net_size;
	uint32_t gram_addr;
	uint32_t gram_size;
};

int start_model_file_loader(void);
int load_fw_to_arr(uint8_t** fw_image_buf,uint32_t* fw_image_size,const char* name);
