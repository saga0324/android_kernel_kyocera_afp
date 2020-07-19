/*
 * drivers/staging/sensorhub/LINUX/Drivers/QLSENSORTEST/src/qlsensortest.c
 *
 * Copyright (C) 2017 QuickLogic Corp
 * Author: Ashwinee Dhakate
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */


#include <linux/ql/qleos_s3.h>
#include <linux/sched.h>
#include <linux/ql/qleos_common.h>
#include <linux/ktime.h>
#include <linux/ql/qlspi_linux.h>
#include "qlpower.h"

//#define DEBUG_GPIO

#ifdef DEBUG_GPIO
#include <linux/gpio.h>
#endif
/*
 * For /dev node creation
 */
static dev_t dev;
static struct cdev cdev;
struct class *s_class;

/*
 * host command sync variables
 */

static struct mutex data_mutex;
static DEFINE_MUTEX(hc_exec);
static DEFINE_MUTEX(hc_irq_mutual);
//struct mutex spi_mutex;
wait_queue_head_t wait_int;
spinlock_t queue_lock;
static struct mutex eos3_irq_mutex;
static struct mutex eose3_evt_mutex;
static struct mutex eose3_eif_mutex;
static atomic_t pending_hc_cmplt;
static atomic_t batch_margin;
static atomic_t m4_batch_buf_full;
static atomic_t batch_lat_exp;
static atomic_t m4_buf_blank_full;
static atomic_t ffe_batch_buf_full;
static atomic_t ffe_buf_blank_full;
static atomic_t wait_event_flag;
static atomic_t rqst_hw_rbt_already = ATOMIC_INIT(false);
static struct workqueue_struct *evt_wq;
static struct workqueue_struct *rbt_wq;
static s_workqueue s_work[MAX_QWORK_COUNT];
struct work_struct qleoss3_rbt_work;

static int work_cnt = 0;
static int g_workqueue_used = 0;

static uint64_t intr_irq_flag;
static uint64_t intr_hc_flag = 0;
static uint64_t intr_err_flag = 0;
static bool intr_isr_after_hc_done = false;

static atomic_t g_sns_flush_type;
static atomic_t g_is_flush_exec;

static int64_t markoff_ktime = 0;
/*
 * Completion is used to block the call untill M4 interrupts AP
 */
//struct mutex irq_lock;
volatile int command_state = 0;

struct hc_info hc;
static uint8_t sys_result[8] = {0};

uint8_t mark_off_buf[64*1024];
uint8_t result_response_buf[HOSTCMD_RSP_BUFSIZE];
uint8_t test_response_buf[HOSTCMD_RSP_BUFSIZE];
int16_t hc_response_len = 0;
uint16_t mark_off_data_bytes = 0;
int batching = 0;

/* Signaling to Application */
static struct task_struct *task = NULL;
//static int signum = 0;

static void qleoss3_int_event_work_func(struct work_struct *work);
static void qleoss3_cleanup_isrproc(void);
static int qleoss3_waitcmd(void);
static int get_error_factor(void);

/* KC customize */
#define M4_RESET_LOG_ADDR  M4_RESET_LOG_INFO_ADDR
#define M4_RESET_LOG_SIZE  M4_RESET_LOG_INFO_SIZE

static char reset_log_buf[M4_RESET_LOG_SIZE] = {0};
static struct delayed_work wfl_monitor_dwork;
static struct delayed_work rcvry_during_rcvry_dwork;
static int qleos_s3_chk_isr_retry(const uint64_t irqfactor);
static atomic_t wfl_S3toAP_status;
static atomic_t wfl_S3toAP_finish_flg;
#define WFL_MONITOR_TIME_US					(10)

#define STATUS_AP2S3HIGH_AND_NOHOSTCMDIRQ	(-4) //It is different from the error number of SPI_STATUS_XXX
#define HCWAIT_MAX_RETRY_NUM				(10)
#define RE_RECOVERY_KICK_TIME				(5*1000)

uint8_t qleoss3_get_shutdown_status(void)
{
    ENTERED();
    EXITED();
    return ql_spi_get_shutdown_status();
}

uint32_t prepare_host_command(uint8_t purpose, uint8_t type, uint8_t cmd)
{
	union sensor_host_command h_cmd;

	ENTERED();
	memset(&h_cmd,0, sizeof(h_cmd));

	h_cmd.cmd_struct.purpose	= purpose;
	h_cmd.cmd_struct.type		= type;
	h_cmd.cmd_struct.command	= cmd;

	QLEOS_DBG("0x%08x\n", h_cmd.host_command);
    EXITED();
	return h_cmd.host_command;
}

static void qleoss3_workqueue_init(void)
{
    int32_t i;

    ENTERED();
    for (i = 0; i < MAX_QWORK_COUNT; i++) {
        cancel_work_sync(&s_work[i].work);
        s_work[i].status = false;
    }
    work_cnt = 0;
    EXITED();
}

static int32_t qleoss3_workqueue_create(struct workqueue_struct *queue,
	void (*func)(struct work_struct *), uint64_t result)
{
	int32_t ret = 0;
	int32_t i;
	unsigned long flags;

	QLEOS_DBG("start queue:%p func:%pf result:%016llx", queue, func, result);

	if((queue == NULL) || (func == NULL)){
		QLEOS_ERR("neither queue nor func\n");
		return -1;
	}

	spin_lock_irqsave(&queue_lock, flags);

	QLEOS_DBG("work_cnt[%d]status[%x]",work_cnt, s_work[work_cnt].status);

	if(s_work[work_cnt].status == false){

		INIT_WORK(&s_work[work_cnt].work, func);
		s_work[work_cnt].result = result;

		ret = queue_work(queue, &s_work[work_cnt].work);

		if (ret == 1) {
			s_work[work_cnt].status = true;

			if(++work_cnt >= MAX_QWORK_COUNT){
				work_cnt = 0;
			}
			ret = 0;
		} else {
			QLEOS_ERR("queue_work Non Create[%d]",ret);
		}
		g_workqueue_used = 0;
	} else {
		if(g_workqueue_used < MAX_QWORK_COUNT){
			QLEOS_ERR("queue_work[%d] used!!",work_cnt);
			for(i = 0; i < 50; i++){
				QLEOS_DBG("work_cnt[%d]status[%x]",i,s_work[i].status);
			}
			if(++work_cnt >= MAX_QWORK_COUNT){
				work_cnt = 0;
			}
			g_workqueue_used++;
		}
	}

	spin_unlock_irqrestore(&queue_lock, flags);

	QLEOS_DBG("END\n");
	return ret;
}

static void qleoss3_workqueue_delete(struct work_struct *work)
{
    int32_t i;
    unsigned long flags;

    ENTERED();
    spin_lock_irqsave(&queue_lock, flags);

    for(i = 0; i < MAX_QWORK_COUNT; i++){
        if(&s_work[i].work == work){
            s_work[i].status = false;
            QLEOS_DBG("hit delete queue[%d]! work:0x[%p] 0x[%p] ",
                        i, &s_work[i].work, work);
            break;
        }
    }

    spin_unlock_irqrestore(&queue_lock, flags);

    EXITED();
    return ;
}



int set_command(uint32_t command, int length, uint8_t *param, uint8_t *result)
{
	//int j = 0;
	int ret = 0;
	struct hc_info ihc = {0};
	ENTERED();

	ihc.command = command;
	ihc.param_data_len = length;
	memcpy(ihc.param_data, param, length);
	ihc.response_len = 0;

	//for (j = 0; j < 2; j++)
	//	QLEOS_DBG("param[%d] 0x%x\n", j, ihc.param_data[j]);

	if (qleoss3_hostcmd(&ihc)) {
		QLEOS_ERR("Failed to send the command to s3\n");
		ret = -1;
	}
	memcpy(result, sys_result, 8);

	EXITED();
	return ret;
}

int get_command(uint32_t command, int length, uint8_t *response)
{
	int ret = 0;
	struct hc_info ihc = {0};
	ENTERED();

	ihc.command = command;
	ihc.param_data_len = 0;
	ihc.response_len = length;
	if (qleoss3_hostcmd(&ihc)) {
		QLEOS_ERR("Failed to send the command to s3\n");
		ret = -1;
	}

	memcpy(response, &result_response_buf[0], length + 8);
	EXITED();
	return ret;
}

int set_get_command(uint32_t command, int length, uint8_t *param,
	int res_len, uint8_t *response)
{
	// int j = 0;
	int ret = 0;
	struct hc_info ihc = {0};
	ENTERED();

	ihc.command = command;
	ihc.param_data_len = length;
	memcpy(ihc.param_data, param, length);
	ihc.response_len = res_len;

	// for (j = 0; j < 2; j++)
		// QLEOS_DBG("param[%d] 0x%x\n", j, ihc.param_data[j]);

	if (qleoss3_hostcmd(&ihc)) {
		QLEOS_ERR("Failed to send the command to s3\n");
		ret = -1;
	}

	memcpy(response, &result_response_buf[0], res_len + 8);
	EXITED();
	return ret;
}

int qleoss3_set_get_hostcmd(struct hc_info *ihc, int mode)
{
	int ret = 0;
	ENTERED();
	if (qleoss3_hostcmd(ihc)) {
		QLEOS_ERR("Failed to send the command to s3\n");
		ret = -1;
	}

	EXITED();
	return ret;
}

static void qleos_finish_wait_low_s3toap_polling(void)
{
	ENTERED();
	if(atomic_read(&wfl_S3toAP_status) == true){
		QLEOS_DBG("S3toAP low waiting finish. \n");
		atomic_set(&wfl_S3toAP_finish_flg, true);
		flush_delayed_work(&wfl_monitor_dwork);
	}
	EXITED();
}

static void qleos_wfl_monitor(struct work_struct *work)
{
	uint8_t S3toAPval;
	ENTERED();

	while(1){
		S3toAPval = get_S3toAP_Value();
		QLEOS_DBG("monitor condition S3toAP value:[%s]\n", ((S3toAPval==GPIO_HIGH) ? "HIGH":"LOW"));
		if(!atomic_read(&wfl_S3toAP_status)){
		    QLEOS_ERR("Be not waiting for S3toAP LOW status. Monitor End.");
			break;
		}
		else if((S3toAPval == GPIO_LOW) || atomic_read(&wfl_S3toAP_finish_flg)){
			QLEOS_DBG("Host command break block root.\n");
			mutex_lock(&data_mutex);
			intr_hc_flag |= INTREQ_HOST_CMD;
			mutex_unlock(&data_mutex);
			atomic_set(&wfl_S3toAP_status ,false);
			atomic_set(&wfl_S3toAP_finish_flg, false);
			wake_up_interruptible(&wait_int);
			break;
		} else {
			QLEOS_DBG("Monitor Retry.\n");
		    usleep_range(10, 20);
		}
    }
	EXITED();
}

static int qleos_s3_chk_isr_retry(const uint64_t irqfactor)
{
	int ret = 0;

	ENTERED();
	if(irqfactor & INTREQ_HOST_CMD){
		set_APtoS3_value(GPIO_LOW);
		mutex_lock(&data_mutex);
		intr_irq_flag |= (INTREQ_HOST_CMD);
		if(atomic_read(&pending_hc_cmplt)){
			atomic_set(&wfl_S3toAP_status ,true);
			intr_isr_after_hc_done = true;
		    QLEOS_DBG("wfl_S3toAP_status set true.");
		}
		mutex_unlock(&data_mutex);
		send_erase_interrupt_factor_command(irqfactor & INTREQ_HOST_CMD);
		schedule_delayed_work(&wfl_monitor_dwork, 0);
	} else {
		ret = STATUS_AP2S3HIGH_AND_NOHOSTCMDIRQ;
	}
	EXITED();
	return ret;
}


static void error_log_save(void)
{
	return;
}

#define FIRSTDL_RETRY_CNT_MAX	(3)
static void hw_reboot_initialize(void)
{
	ENTERED();
	/* Cancel timer wait. */
	reinit_completion(&power_done_wait);
	reinit_completion(&boot_done_wait);
	reinit_completion(&firstdl_done_wait);
	/* Cancel command wait. */
	if (atomic_read(&wait_event_flag)) {
		mutex_lock(&data_mutex);
		intr_err_flag |= INTREQ_ERROR;
		mutex_unlock(&data_mutex);
		wake_up_interruptible(&wait_int);
	}
	atomic_set(&wfl_S3toAP_status ,false);
	atomic_set(&wfl_S3toAP_finish_flg, false);
	atomic_set(&pending_hc_cmplt, false);
	atomic_set(&g_sns_flush_type,0);
	atomic_set(&g_is_flush_exec, false);
	intr_irq_flag = 0;
	intr_hc_flag = 0;
	intr_isr_after_hc_done = false;
	EXITED();
}

#define REBOOT_PROC_INTERVAL_TIME	(5 * 60 * 1000 * 1000)	/* 5minute */
#define REBOOT_PROC_MAX_CNT			(40)

static int hw_reboot_freq_monitor(void)
{
	static uint8_t reboot_cnt = 0;
	static ktime_t prev_time;
	ktime_t curr_time;
	s64 func_time_us = 0;

	ENTERED();
	reboot_cnt++;
	curr_time = ktime_get();
	if (reboot_cnt > 1) {
		func_time_us = ktime_to_us(ktime_sub(curr_time, prev_time));
		if ((func_time_us > REBOOT_PROC_INTERVAL_TIME) || (func_time_us < 0)) {
			reboot_cnt = 0;
		} else {
			if (reboot_cnt > REBOOT_PROC_MAX_CNT) {
				QLEOS_ERR("Maximum number of reboots exceeded.\n");
			    EXITED();
			    return -1;
			}
		}
	}
	prev_time = curr_time;
	QLEOS_ERR("func_time_us = %lld\n", func_time_us);
	QLEOS_ERR("reboot_cnt = %d\n", reboot_cnt);
	EXITED();
	return 0;
}

static void hw_reboot_event_work_func(struct work_struct *work)
{
	int i = 0;
    int cur_eoss3_status = ql_spi_get_eoss3_status();
	ENTERED();

	QLEOS_ERR("Micon EOSS3 recovery start.");
	QLEOS_ERR("Current EOS status[%d].", cur_eoss3_status);
	if(cur_eoss3_status != RECOVERING){
		QLEOS_ERR("skip. recovery already running.");
		return;
	}
	//flush workqueue s_work[work_cnt].work
	qleoss3_workqueue_init();
	qlspi_disable_irq();

	error_log_save();
	if (sensor_ql_micon_save_param() != 0) {
		QLEOS_ERR("micon parameter save error.\n");
	}
	if (QLSPI_Read_S3_Mem(M4_RESET_LOG_ADDR, &reset_log_buf[0], M4_RESET_LOG_SIZE) != QL_STATUS_OK) {
		QLEOS_ERR("QLSPI_Read_S3_Mem error.\n");
	} else {
		QLEOS_ERR("**************** ResetLog Dump Start ****************");
		for (i = 0; i < M4_RESET_LOG_SIZE; i++) {
			QLEOS_ERR("%02x %c", reset_log_buf[i], reset_log_buf[i]);
		}
		QLEOS_ERR("**************** ResetLog Dump End ******************");
	}

	if(hw_reboot_freq_monitor()){
		ql_spi_set_eoss3_status(DEAD);
		QLEOS_ERR("hw_reboot is occured High frequency. kill eoss3.");
		qleoss3_workqueue_delete(work);
		EXITED();
		return;
	}

	ql_spi_power_offon();
	hw_reboot_initialize();
	complete(&power_done_wait);
	fwdl_sequence_when_recovering();
	atomic_set(&rqst_hw_rbt_already, false);
	QLEOS_ERR("Micon EOSS3 recovery end.");

	EXITED();
}

static void re_rcvry_work(struct work_struct *work)
{
	ENTERED();
	QLEOS_ERR("Recoverying during recovery has occured. restart EOSS3 recovery.");
	hw_reboot_exec();
	EXITED();
}

void hw_reboot_exec(void)
{
	int prev_eoss3_status = ql_spi_get_eoss3_status();

	ENTERED();
	if ((prev_eoss3_status == DEAD)||(prev_eoss3_status == NOT_INITIALIZE)) {
		QLEOS_ERR("It is not possible to reboot because it is out of order.");
		goto exit;
	}
	else if(prev_eoss3_status != NORMAL){
		QLEOS_ERR("Since initialization / recovery is in progress, re-recovery.");
		schedule_delayed_work(&rcvry_during_rcvry_dwork, msecs_to_jiffies(RE_RECOVERY_KICK_TIME));
		goto exit;
	}

	ql_spi_set_eoss3_status(RECOVERING);

	QLEOS_ERR("EOS status : prev = %d, curr = %d", prev_eoss3_status, ql_spi_get_eoss3_status());

	ql_spi_micon_recovery_wakelock();
	queue_work(rbt_wq, &qleoss3_rbt_work);
exit:
	EXITED();
}

void qleoss3_request_hw_reboot(char *client_name)
{
	ENTERED();
	if((!atomic_read(&rqst_hw_rbt_already)) && (ql_spi_get_eoss3_status() == NORMAL)){
		atomic_set(&rqst_hw_rbt_already, true);
		QLEOS_ERR("Request hw_reboot. client[%s]", client_name);
		hw_reboot_exec();
	}
	EXITED();
}

QL_Status qleos_s3_isr(void * data)
{
	int length = 8;
	int ret = 0;
	static int loop_num = 0;
	uint64_t result;
	int8_t current_eos_status;
	const uint64_t other_factor = INTREQ_BOOT_DONE | WDT_EXPIRED | HOST_CMD_ERR | INTREQ_ERROR;
	uint8_t tmp_response_buf[HOSTCMD_RSP_BUFSIZE] = {0};

	ENTERED();
	mutex_lock(&hc_irq_mutual);
	mutex_lock(&eos3_irq_mutex);
	if (hc_response_len) {
		length += hc_response_len;
		QLEOS_DBG(" response length = %d\n", length);
		if(length > HOSTCMD_RSP_BUFSIZE){
			QLEOS_ERR("Invalid result-response length[%d]\n", length);
			mutex_unlock(&eos3_irq_mutex);
			mutex_unlock(&hc_irq_mutual);
			return -1;
		}
	}

	qleos_finish_wait_low_s3toap_polling();
	/* Read result and response in one go */
	if (QLSPI_Read_S3_Mem(SENSOR_RESULT_ADDR, tmp_response_buf, length) !=
		QL_STATUS_OK) {
		QLEOS_ERR("Reading result-response failed\n");
		mutex_unlock(&eos3_irq_mutex);
		mutex_unlock(&hc_irq_mutual);
		return -1;
	}
	memcpy(&result, tmp_response_buf, 8);
	mutex_lock(&data_mutex);
	intr_irq_flag |= result;
	mutex_unlock(&data_mutex);
	if(result & INTREQ_HOST_CMD){
		QLEOS_DBG("copy response data to shared response buf. size[%d]", length);
		memcpy(&result_response_buf[0], tmp_response_buf, length);
	}

	QLEOS_DBG("result: 0x%016llx address is 0x%x", result, SENSOR_RESULT_ADDR);

	if(atomic_read(&pending_hc_cmplt) && !intr_isr_after_hc_done){
		ret = qleos_s3_chk_isr_retry(result);
		if(ret == STATUS_AP2S3HIGH_AND_NOHOSTCMDIRQ){
			QLEOS_DBG("[Executing HostCMD] And [No HostCMD irq factor].retry!");
			loop_num++;
			ret = 0;
			if(loop_num >= HCWAIT_MAX_RETRY_NUM){
				QLEOS_ERR("expired loop num[HostCommand Complete waiting]");
				loop_num = 0;
				ret = -1;
			}
			mutex_unlock(&eos3_irq_mutex);
			mutex_unlock(&hc_irq_mutual);
			return ret;
		}
	} else {
		ret = send_erase_interrupt_factor_command(result);
		if( (ret == -1) || (ret == SPI_STATUS_ERR_AHB) || (ret == SPI_STATUS_ERR_DMA) ){
			QLEOS_ERR("EIF error. reboot hw.");
			loop_num = 0;
			mutex_unlock(&eos3_irq_mutex);
			mutex_unlock(&hc_irq_mutual);
			goto hw_reboot;
		}

	}
	mutex_unlock(&hc_irq_mutual);
	loop_num = 0;

	/* ignore spurious interrupt */
	if ((result & ~(ALL_HOST_RESP | SENSOR_ALL_EVENTS)) || (result == 0x0)){
		QLEOS_DBG("This is unexpected event... Ignore!!![0x%016llx]\n", result);
		mutex_unlock(&eos3_irq_mutex);
		EXITED();
		return 0;
	}

	if (result & other_factor) {
		QLEOS_DBG("result:other factor\n");

		mutex_lock(&data_mutex);
		if (result & HOST_CMD_ERR) {
			QLEOS_ERR("Host Command Error Occured\n");
			intr_irq_flag &= ~(result & HOST_CMD_ERR);
			//error process about Result bit60-62 are nonzero.
			mutex_unlock(&data_mutex);
			mutex_unlock(&eos3_irq_mutex);
			goto hw_reboot;
		}
		if (result & INTREQ_ERROR){
			QLEOS_ERR("INTREQ Error Occured\n");
			intr_irq_flag &= ~(result & INTREQ_ERROR);
			//error process as Result bit63 ==1
			mutex_unlock(&data_mutex);
			mutex_unlock(&eos3_irq_mutex);
			//get_error_factor();
			goto hw_reboot;
		}
		if (result & WDT_EXPIRED){
			QLEOS_ERR("WDT Expired Occured\n");
			intr_irq_flag &= ~(result & WDT_EXPIRED);
			//error process as Result bit59 ==1
			mutex_unlock(&data_mutex);
			mutex_unlock(&eos3_irq_mutex);
			goto hw_reboot;
		}
		if (result & INTREQ_BOOT_DONE) {
			current_eos_status = ql_spi_get_eoss3_status();
			intr_irq_flag &= ~INTREQ_BOOT_DONE;
			if (current_eos_status == FW_DOWNLOADING) {
				QLEOS_DBG("Boot done\n");
				complete(&firstdl_done_wait);
				qleoss3_workqueue_create(evt_wq, qleoss3_int_event_work_func, result);
			}
			else if(current_eos_status == DEAD) {
				QLEOS_ERR("device is dead.");
			} else {
				QLEOS_ERR("Unexpected BootDone error[%d]\n", ql_spi_get_eoss3_status());
				mutex_unlock(&data_mutex);
				mutex_unlock(&eos3_irq_mutex);
				goto hw_reboot;
			}
		}
		mutex_unlock(&data_mutex);
	}

	if (result & SENSOR_ALL_EVENTS) {
		QLEOS_DBG("result:ALL_EVENTS\n");
		mutex_lock(&data_mutex);
		intr_irq_flag &= ~(result & SENSOR_ALL_EVENTS);
		mutex_unlock(&data_mutex);
		/* create workqueue */
		qleoss3_workqueue_create(evt_wq, qleoss3_int_event_work_func, result);
	}

	mutex_unlock(&eos3_irq_mutex);
	EXITED();
	return 0;
hw_reboot:
	hw_reboot_exec();
	EXITED();
	return -1;
}

static void qleoss3_int_event_work_func(struct work_struct *work)
{
	//int ret = 0;
	s_workqueue *work_data = container_of(work, s_workqueue, work);
	uint64_t result = work_data->result;
    int8_t current_eos_status = ql_spi_get_eoss3_status();
	struct siginfo info;
	int32_t flush_type = atomic_read(&g_sns_flush_type);
	int32_t flush_opt = atomic_read(&g_is_flush_exec);
	QLEOS_DBG("result = 0x%016llx\n", result);

	atomic_set(&g_sns_flush_type,0);
	atomic_set(&g_is_flush_exec, false);

	if((current_eos_status == RECOVERING) || (current_eos_status == DEAD)){
		if(!(result & INTREQ_BOOT_DONE)){
			qleoss3_workqueue_delete(work) ;
			QLEOS_ERR("Force end.");
			return;
		}
	}

	mutex_lock(&eose3_evt_mutex);

	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIGEVT;
	info.si_code = SI_QUEUE;
	info.si_int = 0;

	if (result & INTREQ_BOOT_DONE) {
		QLEOS_DBG("Boot done detected\n");
		ql_spi_any_device_init();
	}

	if (result & SENSOR_ALL_EVENTS) {
		sensor_ql_micon_interrupt(result);
	}
	if (result & INTREQ_ERROR) {
		QLEOS_DBG("Error detected\n");
		info.si_int |= (1 << 30);
		get_error_factor();
	}
	if(flush_opt){
		QLEOS_DBG("Sensor Flush\n");
		sensor_ql_micon_flush(flush_type);
	}
	if (task != NULL) {
		if(send_sig_info(SIGEVT, &info, task) < 0) {
			QLEOS_ERR("Unable to send signal\n");
		}
	}
	qleoss3_workqueue_delete(work) ;

	mutex_unlock(&eose3_evt_mutex);

	EXITED();
	return;
}


void reset_driver(void)
{
}

#define HOSTCMD_EVENT_MARKOFF	(0x000b0404)
static void qleoss3_get_kerneltime_if_needed(const uint32_t hostcmdID)
{
	if(hostcmdID == HOSTCMD_EVENT_MARKOFF){
		QLEOS_DBG("record kernel time");
		markoff_ktime = ktime_to_ns(ktime_get_boottime());
	}
}

int64_t get_ev_markoff_ktime(void)
{
	return markoff_ktime;
}

static bool can_exec_hc(const int cur_status, const uint32_t hcID)
{
	if( (cur_status == NORMAL) || (cur_status == CLIENT_INITIALIZING) )
		return true;
	else if( (cur_status == FW_DOWNLOADING) && ( (hcID == HC_SCALE_CHANGE) || (hcID == HC_CRC_CHECK) ) )
		return true;
	else
		return false;
}

/* sending command to s3 with data and trigger s3 */
int qleoss3_hostcmd(struct hc_info *ihc)
{
	int cmd_param_len = 4;
	int ret = QL_STATUS_OK;

#ifdef CONFIG_USE_KC_MICON_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
#endif /* CONFIG_USE_KC_MICON_PERFORMANCE_MEASURE_LOG */

	ENTERED();

#ifdef CONFIG_USE_KC_MICON_PERFORMANCE_MEASURE_LOG
	enter = ktime_get();
#endif /* CONFIG_USE_KC_MICON_PERFORMANCE_MEASURE_LOG */

	mutex_lock(&hc_exec);
	mutex_lock(&hc_irq_mutual);
	if(!can_exec_hc(ql_spi_get_eoss3_status(), ihc->command)) {
		QLEOS_ERR("skip -command = 0x%08x\n", ihc->command);
		mutex_unlock(&hc_exec);
		mutex_unlock(&hc_irq_mutual);
		return -1;
	}

	atomic_set(&pending_hc_cmplt, true);
	QLEOS_DBG("HC = 0x%08x, prm = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		ihc->command,
		ihc->param_data[0],
		ihc->param_data[1],
		ihc->param_data[2],
		ihc->param_data[3],
		ihc->param_data[4],
		ihc->param_data[5],
		ihc->param_data[6],
		ihc->param_data[7]);

	/* If command has parameter, add parameter length to total length */
	if(ihc->param_data_len) {
		cmd_param_len += ihc->param_data_len;
	}

#ifdef SPI_SCALING_EN
	if(ihc->command == 0x00000900) {
		qlspi_change_freq(SPI_FREQ_LOW_ID);
	}
#endif

	mutex_lock(&data_mutex);
	intr_irq_flag = 0;
	intr_hc_flag = 0;
	intr_isr_after_hc_done = false;
	mutex_unlock(&data_mutex);

	if(ihc->response_len)
		hc_response_len = ihc->response_len;
	ret = QLSPI_Write_S3_Mem(QL_COMMAND_ADDR, (uint8_t*)ihc, cmd_param_len);
	/* Write command and param(if present) to SRAM */
	if (ret == QL_STATUS_OK) {
		QLEOS_DBG("Triggering interrupt to s3\n");
		qleoss3_get_kerneltime_if_needed(ihc->command);
		if((QLSPI_Trigger_Intr()) != QL_STATUS_OK) {
			QLEOS_ERR("Triggering interrupt to s3 failed\n");
		}
		mutex_unlock(&hc_irq_mutual);

		ret = qleoss3_waitcmd();
		if (ret != QL_STATUS_OK) {
			if (ret == -QL_STATUS_ERROR) {
				/*donno what to do*/
				mutex_unlock(&hc_exec);
				return -QL_STATUS_ERROR;
			} else if (ret == -2) {
				// as per KCFJ-104, change error recovery on AHB Write timeout to reload fw:
				QLEOS_ERR("Host command timeout\n");
				mutex_unlock(&hc_exec);
				hw_reboot_exec();
				return -QL_STATUS_ERROR;
			}
			mutex_unlock(&hc_exec);
			return -QL_STATUS_ERROR;
		}
		QLEOS_DBG("len = 0x%x, res = 0x%08x, rsp = 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
				ihc->response_len,
				(unsigned int)result_response_buf[0],     //res
				(unsigned int)result_response_buf[8],     //rsp[0]
				(unsigned int)result_response_buf[8+4],   //rsp[1]
				(unsigned int)result_response_buf[8+8],   //rsp[2]
				(unsigned int)result_response_buf[8+12],  //rsp[3]
				(unsigned int)result_response_buf[8+16],  //rsp[4]
				(unsigned int)result_response_buf[8+20]); //rsp[5]
		if (ihc->response_len) {
			memcpy(ihc->response, &result_response_buf[8],
				ihc->response_len);
			hc_response_len = 0;
		}
#ifdef SPI_SCALING_EN
		if (ihc->command == 0x00000900) {
			if(ihc->param_data[0] == 1) {
				qlspi_change_freq(SPI_FREQ_HIGH_ID);
			} else if (ihc->param_data[0] == 2) {
				qlspi_change_freq(SPI_FREQ_LOW_ID);
			}
		}
#endif
	} else {
		QLEOS_ERR(" Writing command to S3 failed\n");
		mutex_unlock(&hc_irq_mutual);
	}

	mutex_unlock(&hc_exec);

#ifdef CONFIG_USE_KC_MICON_PERFORMANCE_MEASURE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n", __func__, enter_us, exit_us, func_time_us);
#endif /* CONFIG_USE_KC_MICON_PERFORMANCE_MEASURE_LOG */

	EXITED();
	return ret;
}

static void qleoss3_cleanup_isrproc(void)
{
	ENTERED();
	mutex_lock(&data_mutex);
	atomic_set(&pending_hc_cmplt, false);
	atomic_set(&wfl_S3toAP_status, false);
	QLEOS_DBG("wfl_S3toAP_status set false.");
	intr_isr_after_hc_done = false;
	mutex_unlock(&data_mutex);
	EXITED();
}

static int qleoss3_waitcmd(void)
{
	int ret = -2;
	int result = 0;
	long timeout;
	int retry = 1;//300;
	//uint64_t ack = 0;

	ENTERED();
	timeout = msecs_to_jiffies(4000);

	while (retry) {
		//wait_event_interruptible(wait_int, intr_irq_flag & INTREQ_HOST_CMD);
		atomic_set(&wait_event_flag, true);
		result = wait_event_interruptible_timeout(wait_int,
			((intr_hc_flag & INTREQ_HOST_CMD) | (intr_err_flag & INTREQ_ERROR)), timeout);
		atomic_set(&wait_event_flag, false);
		if (result == -ERESTARTSYS) {
			QLEOS_DBG("Wait event signal received [%d]\n", retry);
			usleep_range(10000, 10100);
			continue;
		}
		if (result == 0) {
			QLEOS_ERR("Wait event timeout..[0x%016llx]\n",
				intr_irq_flag);
			ret = -2;
			break;
		}
		if (intr_hc_flag & INTREQ_HOST_CMD) {
			/* send erase interrupt factor */
			//send_erase_interrupt_factor_command(INTREQ_HOST_CMD);
			mutex_lock(&data_mutex);
			intr_irq_flag &= ~INTREQ_HOST_CMD;
			intr_hc_flag &= ~INTREQ_HOST_CMD;
			mutex_unlock(&data_mutex);
			QLEOS_DBG("HOST_CMD done\n");
			ret = 0;
			break;
		}
		if (intr_err_flag & INTREQ_ERROR) {
			mutex_lock(&data_mutex);
			intr_err_flag &= ~INTREQ_ERROR;
			mutex_unlock(&data_mutex);
			QLEOS_DBG("HOST_CMD cancel\n");
			ret = 0;
			break;
		}
		QLEOS_DBG("Release host command wait\n");
		// mutex_lock(&data_mutex);
		// if (intr_irq_flag & INTREQ_HOST_CMD) {
			// intr_irq_flag &= ~INTREQ_HOST_CMD;
			// mutex_unlock(&data_mutex);
			// ret = 0;
			// qlspi_enable_irq();
			// break;
		// }
		// mutex_unlock(&data_mutex);

		QLEOS_DBG("Retry = [%d]\n", retry);
		retry--;
	}
	QLEOS_DBG("while break\n");
	qleoss3_cleanup_isrproc();

	EXITED();
	return ret;
}

void qleoss3_set_flush(int32_t type)
{
	ENTERED();
	atomic_set(&g_sns_flush_type,type);
	atomic_set(&g_is_flush_exec, true);
	qleoss3_workqueue_create(evt_wq, qleoss3_int_event_work_func, 0);
	EXITED();
}

#ifdef DEBUG_DATA_READ_FROM_REGISTER
#define NORMAL  1
#define NOTGOOD 0
#define SW_INTR2_REG		(0x40004850)
void dbg_RESULT_N_SWINTR2_reg_read(uint8_t result)
{
	uint64_t irqfactor;
	uint32_t sw_intr2_val;
	uint8_t status;

	ENTERED();
	if(result){
		status = NOTGOOD;
	} else {
		status = NORMAL;
	}
	QLSPI_Read_S3_Mem(SENSOR_RESULT_ADDR, test_response_buf, 8);
	memcpy(&irqfactor, test_response_buf, 8);
	QLSPI_Read_S3_Mem(SW_INTR2_REG, test_response_buf, 4);
	memcpy(&sw_intr2_val, test_response_buf, 4);
	QLEOS_DBG("=== EIF status[%s] ===\n", (status == NORMAL) ? "OK":"NG");
	QLEOS_DBG("IRQ factor[0x%016llx], sw_intr2_val[0x%08x]",
			irqfactor, sw_intr2_val);
	QLEOS_DBG("======================\n");
	EXITED();
}
#endif /* DEBUG_DATA_READ_FROM_REGISTER */
int send_erase_interrupt_factor_command(const uint64_t ack)
{
	uint8_t cmd_param[4 + 8];
	int retval = 0;
	ENTERED();

	if(!ack){
		QLEOS_DBG("ack is zero. ignore EIF.");
		goto exit;
	}
	memset(cmd_param, 0, 12);

	memcpy(&cmd_param[4], &ack, 8);
	QLEOS_DBG("EIF cmd_param [0x%016llx]\n", *(uint64_t*)&cmd_param[4]);
	if(QLSPI_Write_S3_Mem(QL_COMMAND_ADDR, cmd_param, 12) != QL_STATUS_OK) {
		QLEOS_ERR("Writing to S3 failed\n");
		return retval;
	}

	retval = qlspi_is_spi_intr_set_zero();

	QLEOS_DBG("Triggering interrupt to s3 [EIF]\n");
	if((QLSPI_Trigger_Intr()) != QL_STATUS_OK) {
		udelay(10);
		QLEOS_ERR("Triggering interrupt to s3 failed\n");
		return retval;
	}
	udelay(10);
	set_APtoS3_value(GPIO_LOW);

	/* Check for GPIO is low */
	retval = qlspi_check_for_gpio_if_low();

#ifdef DEBUG_DATA_READ_FROM_REGISTER
	dbg_RESULT_N_SWINTR2_reg_read(retval);
#endif /* DEBUG_DATA_READ_FROM_REGISTER */

	if (retval) {
		QLEOS_ERR("GPIO timeout!!!\n");
		//hw_reboot_exec(REBOOT_TYPE_HW);
		return retval;
	}

	QLEOS_DBG("Interrupt Cleared\n");
exit:
	/* Enable irq */
	qlspi_enable_irq();
	EXITED();
    return QL_STATUS_OK;
}


/* 09_BathMarkOff */
void sensor_batch_mark_off(int is_clear)
{
	//struct hc_info hc;
	struct batch_markoff_param mark_off_param;
	struct batch_markoff_resp mark_off_resp;
	uint16_t index = 0;
	uint32_t m4_s_ptr = 0, m4_e_ptr = 0, ffe_s_ptr = 0, ffe_e_ptr = 0;
	int m4_data_bytes = 0, ffe_data_bytes = 0, x_data_bytes = 0;
	uint32_t m4_batch_buff_start_address;
	uint32_t m4_batch_buff_end_address;
	uint32_t ffe_batch_buff_start_address;
	uint32_t ffe_batch_buff_end_address;
#ifdef BATCHING_DYNAMIC_MEMORY
	struct batching_buff_adr batch_hdr;
#endif

	ENTERED();

	/* Mark off Batching Data */
	/* 1. in first markoff lock m4 and ffe rd ptrs
	 * 2. change the read pointer and send mark off command with update
	 * 3. */
	hc.command = prepare_host_command(BATCH_MODE_CONTROL,
			BATCH, BMC_MARK_OFF_BATCHING_DATA);
	hc.param_data_len = sizeof(mark_off_param);
	mark_off_param.m4_buf_lock = 1;
	mark_off_param.update_m4_rd_ptr = 0;
	mark_off_param.cur_m4_rd_ptr = 0;
	mark_off_param.ffe_buf_lock = 1;
	mark_off_param.update_ffe_rd_ptr = 0;
	mark_off_param.cur_ffe_rd_ptr = 0;

	memcpy(hc.param_data, &mark_off_param, sizeof(mark_off_param));

	QLEOS_DBG("m4_buf_lock = %d\t" "update_m4_rd_ptr = %d\t"
		"cur_m4_rd_ptr = 0x%x\t" "ffe_buf_lock = %d\t"
		"update_ffe_rd_ptr = %d\t" "cur_ffe_rd_ptr = 0x%x\t",
		mark_off_param.m4_buf_lock, mark_off_param.update_m4_rd_ptr,
		mark_off_param.cur_m4_rd_ptr, mark_off_param.ffe_buf_lock,
		mark_off_param.update_ffe_rd_ptr, mark_off_param.cur_ffe_rd_ptr);

	hc.response_len = sizeof(mark_off_resp);
	qleoss3_hostcmd(&hc);

	memcpy(&mark_off_resp, &result_response_buf[8], sizeof(mark_off_resp));

	if (!mark_off_resp.m4_batch_bytes && !mark_off_resp.ffe_batch_bytes) {
		QLEOS_DBG("No valid batch data\n");
		return;
	}

	m4_data_bytes = mark_off_resp.m4_batch_bytes;
	ffe_data_bytes = mark_off_resp.ffe_batch_bytes;


	QLEOS_DBG("Valid m4 batch bytes = %u\t"
		"Valid ffe batch bytes = %u\t"
		"sens_task_ts = %u\t" "app_task_ts = %u\t"
		"cur_m4_rd_ptr = 0x%x\t" "cur_ffe_rd_ptr = 0x%x\t",
		mark_off_resp.m4_batch_bytes,
		mark_off_resp.ffe_batch_bytes,
		mark_off_resp.sens_task_ts,
		mark_off_resp.app_task_ts,
		mark_off_resp.cur_m4_rd_ptr,
		mark_off_resp.cur_ffe_rd_ptr);
	//QLEOS_DBG("Valid ffe batch bytes = %d\n", ffe_data_bytes);
	/* clear mark off data */
	if (is_clear) {
		QLEOS_DBG("Clearing mark off data\n");
		hc.command = prepare_host_command(BATCH_MODE_CONTROL,
				BATCH, BMC_ALL_CLEAR_BATCHING_DATA);
		hc.param_data_len = 4;
		hc.param_data[0] = 1;
		hc.response_len = 0;
		qleoss3_hostcmd(&hc);

		return;
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

	QLEOS_DBG("BATCH buffer M4 start address = 0x%X\t"
					"end address = 0x%X\t"
					"FFE start address = 0x%X\t"
					"End address = 0x%X\t\n", 
					m4_batch_buff_start_address,
					m4_batch_buff_end_address,
					ffe_batch_buff_start_address,
					ffe_batch_buff_end_address);

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

	while (m4_data_bytes || ffe_data_bytes) {
		if (ffe_data_bytes) {
			if ((ffe_s_ptr + 2040) >= ffe_e_ptr) {
				x_data_bytes = ffe_e_ptr - ffe_s_ptr;

				if (ffe_data_bytes >= 2040) {
					QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
						x_data_bytes);
					ffe_s_ptr = ffe_batch_buff_start_address;
					index += x_data_bytes;
					ffe_data_bytes -= x_data_bytes;

					x_data_bytes = 2040 - x_data_bytes;
					if (x_data_bytes) {
						QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
							x_data_bytes);
						ffe_s_ptr += x_data_bytes;
						index += x_data_bytes;
						ffe_data_bytes -= x_data_bytes;
					}
					mark_off_param.ffe_buf_lock = 1;
				} else {
					if (ffe_data_bytes >= x_data_bytes) {
						QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
							x_data_bytes);
						ffe_s_ptr = ffe_batch_buff_start_address;
						index += x_data_bytes;
						ffe_data_bytes -= x_data_bytes;

						if (ffe_data_bytes) {
							QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
								ffe_data_bytes);
							ffe_s_ptr += ffe_data_bytes;
							index += ffe_data_bytes;
							ffe_data_bytes = 0;
						}
					} else {
						QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
							ffe_data_bytes);
						ffe_s_ptr += ffe_data_bytes;
						index += ffe_data_bytes;
						ffe_data_bytes = 0;
					}
					mark_off_param.ffe_buf_lock = 0;
				}
			} else {
				if (ffe_data_bytes >= 2040) {
					QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
						2040);
					ffe_s_ptr += 2040;
					index += 2040;
					ffe_data_bytes -= 2040;
					mark_off_param.ffe_buf_lock = 1;
				} else {
					QLSPI_Read_S3_Mem(ffe_s_ptr, &mark_off_buf[index],
						ffe_data_bytes);
					ffe_s_ptr += ffe_data_bytes;
					index += ffe_data_bytes;
					ffe_data_bytes = 0;
					mark_off_param.ffe_buf_lock = 0;
				}
				//mark_off_param.ffe_buf_lock = 1;
			}
			mark_off_param.update_ffe_rd_ptr = 1;
			mark_off_param.cur_ffe_rd_ptr = ffe_s_ptr;
		} else {
			mark_off_param.ffe_buf_lock = 0;
			mark_off_param.update_ffe_rd_ptr = 0;
			mark_off_param.cur_ffe_rd_ptr = 0;
		}

		if (m4_data_bytes) {
			if ((m4_s_ptr + 2040) >= m4_e_ptr) {
				x_data_bytes = m4_e_ptr - m4_s_ptr;

				if (m4_data_bytes >= 2040) {
					QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
						x_data_bytes);
					m4_s_ptr = m4_batch_buff_start_address;
					index += x_data_bytes;
					m4_data_bytes -= x_data_bytes;

					x_data_bytes = 2040 - x_data_bytes;
					if (x_data_bytes) {
						QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
							x_data_bytes);
						m4_s_ptr += x_data_bytes;
						index += x_data_bytes;
						m4_data_bytes -= x_data_bytes;
					}
					mark_off_param.m4_buf_lock = 1;
				} else {
					if (m4_data_bytes >= x_data_bytes) {
						QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
							x_data_bytes);
						m4_s_ptr = m4_batch_buff_start_address;
						index += x_data_bytes;
						m4_data_bytes -= x_data_bytes;

						if (m4_data_bytes) {
							QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
								m4_data_bytes);
							m4_s_ptr += m4_data_bytes;
							index += m4_data_bytes;
							m4_data_bytes = 0;
						}
					} else {
						QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
							m4_data_bytes);
						m4_s_ptr += m4_data_bytes;
						index += m4_data_bytes;
						m4_data_bytes = 0;
					}
					mark_off_param.m4_buf_lock = 0;
				}
			} else {
				if (m4_data_bytes >= 2040) {
					QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
						2040);
					m4_s_ptr += 2040;
					index += 2040;
					m4_data_bytes -= 2040;
					mark_off_param.m4_buf_lock = 1;
				} else {
					QLSPI_Read_S3_Mem(m4_s_ptr, &mark_off_buf[index],
						m4_data_bytes);
					m4_s_ptr += m4_data_bytes;
					index += m4_data_bytes;
					m4_data_bytes = 0;
					mark_off_param.m4_buf_lock = 0;
				}
			}
			mark_off_param.update_m4_rd_ptr = 1;
			mark_off_param.cur_m4_rd_ptr = m4_s_ptr;
		} else {
			mark_off_param.m4_buf_lock = 0;
			mark_off_param.update_m4_rd_ptr = 0;
			mark_off_param.cur_m4_rd_ptr = 0;
		}

		/* Send command */
		hc.command = prepare_host_command(BATCH_MODE_CONTROL,
			BATCH, BMC_MARK_OFF_BATCHING_DATA);

		hc.param_data_len = sizeof(mark_off_param);
		memcpy(hc.param_data, &mark_off_param, sizeof(mark_off_param));

		QLEOS_DBG("m4_buf_lock = %d\t" "update_m4_rd_ptr = %d\t"
		"cur_m4_rd_ptr = 0x%x\t" "ffe_buf_lock = %d\t"
		"update_ffe_rd_ptr = %d\t" "cur_ffe_rd_ptr = 0x%x\t",
		mark_off_param.m4_buf_lock, mark_off_param.update_m4_rd_ptr,
		mark_off_param.cur_m4_rd_ptr, mark_off_param.ffe_buf_lock,
		mark_off_param.update_ffe_rd_ptr, mark_off_param.cur_ffe_rd_ptr);

		hc.response_len = 0;
		qleoss3_hostcmd(&hc);
	}

	EXITED();

}


#if 1
static int get_error_factor(void)
{
	struct hc_info ihc = {0};

	ENTERED();

	ihc.command = prepare_host_command(SYSTEM, SYS_ERROR,
				SYS_GET_ERROR_FACTOR);

	ihc.param_data_len = 0;
	ihc.response_len = 16;

	qleoss3_hostcmd(&ihc);
	QLEOS_DBG("Error factor : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
		"0x%02x 0x%02x\n", ihc.response[0], ihc.response[1],
		ihc.response[2], ihc.response[3], ihc.response[4],
		ihc.response[5], ihc.response[6], ihc.response[7],
		ihc.response[8], ihc.response[9], ihc.response[10],
		ihc.response[11], ihc.response[12], ihc.response[13],
		ihc.response[14], ihc.response[15]);
	EXITED();

	return 0;
}
#endif

int check_crc_error(void)
{
	uint8_t result1[8];
	struct hc_info ihc = {0};

	ENTERED();
	//memset(&hc,0, sizeof(hc));

	ihc.command = prepare_host_command(SYSTEM, SYS_CRC_CHECK,
				SYS_EXEC_CRC_CHECK_PROCESS);

	ihc.param_data[0] = 0;
	ihc.param_data_len = 4;
	ihc.response_len = 4;

	qleoss3_hostcmd(&ihc);

	if (result_response_buf[8] & 0x7F) {
		QLEOS_ERR("Got CRC error 0x%x\n", result_response_buf[0]);
		return -1;
	}
	memcpy(result1, &result_response_buf[0], 8);

	// Get Error Factor is done in response to an EVENT
	// with the INTREQ_ERROR bit set, should not be done
	// here, because the error may be unrelated.
	//if (result1[7] & INTREQ_ERROR) {
	//	/* Get error factor */
	//	QLEOS_ERR("CRC command error\n");
	//	get_error_factor();
	//}


	EXITED();
	return 0;

}

int qleoss3_open(struct inode *inode, struct file *filp)
{

	ENTERED();
	EXITED();
	return 0;
}

int qleoss3_release(struct inode *inode, struct file *filp)
{
	ENTERED();
	EXITED();
	return 0;
}

static ssize_t qleoss3_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	unsigned long bytes_not_read  = 0;
	uint8_t *data;

	ENTERED();

	if (!batching) {
		data = kmalloc(GFP_KERNEL, count);

		QLSPI_Read_S3_Mem(ANDROID_SENSOR_DATA_ADDR, data, count);
		bytes_not_read = copy_to_user(buf, data, count);
		if (bytes_not_read) {
			QLEOS_ERR("Could not read %lu bytes data \n",
				bytes_not_read);
			return (count - bytes_not_read);
		}
		kfree(data);
		EXITED();
		return count;
	} else {
		batching = 0;
		QLEOS_DBG("Reading markoff data\n");
		bytes_not_read = copy_to_user(buf, mark_off_buf,
			mark_off_data_bytes);
		if (bytes_not_read) {
			QLEOS_ERR("Could not read %lu bytes data \n",
				bytes_not_read);
			return (mark_off_data_bytes - bytes_not_read);
		}
		return mark_off_data_bytes;
	}
}


static ssize_t qleoss3_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	ENTERED();
	EXITED();
	return count;
}


/* This covers 11_AppCng except significant motion */
static long qleoss3_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int retval = 0;
#if 0
	struct hc_info hc;
	//uint64_t result_status = 0;

	ENTERED();

	if (cmd == QL_CURRENT_TASK) {
		QLEOS_DBG("QL_CURRENT_TASK\n");
		task = get_current();
		signum = SIGEVT;
		return 0;
	}

	switch (cmd) {
	case QL_BATCH_MODR_BM_MODE:
		QLEOS_DBG("QL_BATCH_MODR_BM_MODE\n");
		batching = 1;

		atomic_set(&batch_margin, false);
		while (1) {
			if (atomic_read(&batch_margin) == true)
				break;
		}

		atomic_set(&batch_margin, false);
		sensor_batch_mark_off(0);

		return 0;
	case QL_BATCH_MODC_BM_MODE:
		QLEOS_DBG("QL_BATCH_MODC_BM_MODE\n");

		atomic_set(&batch_margin, false);
		while (1) {
			if (atomic_read(&batch_margin) == true)
				break;
		}

		atomic_set(&batch_margin, false);
		sensor_batch_mark_off(1);

		return 0;
	case QL_BATCH_MODR_M4BF_MODE:
		QLEOS_DBG("QL_BATCH_MODR_M4BF_MODE\n");
		batching = 1;

		atomic_set(&m4_batch_buf_full, false);
		while (1) {
			if (atomic_read(&m4_batch_buf_full) == true)
				break;
		}

		atomic_set(&m4_batch_buf_full, false);
		sensor_batch_mark_off(0);

		return 0;
	case QL_BATCH_MODC_M4BF_MODE:
		QLEOS_DBG("QL_BATCH_MODC_M4BF_MODE\n");

		atomic_set(&m4_batch_buf_full, false);
		while (1) {
			if (atomic_read(&m4_batch_buf_full) == true)
				break;
		}

		atomic_set(&m4_batch_buf_full, false);
		sensor_batch_mark_off(1);

		return 0;
	case QL_BATCH_MODR_LE_MODE:
		QLEOS_DBG("QL_BATCH_MODR_LE_MODE\n");
		batching = 1;

		atomic_set(&batch_lat_exp, false);
		while (1) {
			if (atomic_read(&batch_lat_exp) == true)
				break;
		}

		atomic_set(&batch_lat_exp, false);
		sensor_batch_mark_off(0);

		return 0;
	case QL_BATCH_MODC_LE_MODE:
		QLEOS_DBG("QL_BATCH_MODC_LE_MODE\n");

		atomic_set(&batch_lat_exp, false);
		while (1) {
			if (atomic_read(&batch_lat_exp) == true)
				break;
		}

		atomic_set(&batch_lat_exp, false);
		sensor_batch_mark_off(1);

		return 0;
	case QL_BATCH_MODR_FFEBF_MODE:
		QLEOS_DBG("QL_BATCH_MODR_FFEBF_MODE\n");
		batching = 1;

		atomic_set(&ffe_batch_buf_full, false);
		while (1) {
			if (atomic_read(&ffe_batch_buf_full) == true)
				break;
		}

		atomic_set(&ffe_batch_buf_full, false);
		sensor_batch_mark_off(0);

		return 0;
	case QL_BATCH_MODC_FFEBF_MODE:
		QLEOS_DBG("QL_BATCH_MODC_FFEBF_MODE\n");

		atomic_set(&ffe_batch_buf_full, false);
		while (1) {
			if (atomic_read(&ffe_batch_buf_full) == true)
				break;
		}

		atomic_set(&ffe_batch_buf_full, false);
		sensor_batch_mark_off(1);

		return 0;
	case QL_BATCH_MODR_BSM4BF_MODE:
		QLEOS_DBG("QL_BATCH_MODR_FFEBF_MODE\n");
		batching = 1;

		atomic_set(&m4_buf_blank_full, false);
		while (1) {
			if (atomic_read(&m4_buf_blank_full) == true)
				break;
		}

		atomic_set(&m4_buf_blank_full, false);
		sensor_batch_mark_off(0);

		return 0;
	case QL_BATCH_MODR_BSFFEBF_MODE:
		QLEOS_DBG("QL_BATCH_MODR_FFEBF_MODE\n");
		batching = 1;

		atomic_set(&ffe_buf_blank_full, false);
		while (1) {
			if (atomic_read(&ffe_buf_blank_full) == true)
				break;
		}

		atomic_set(&ffe_buf_blank_full, false);
		sensor_batch_mark_off(0);

		return 0;
	case QL_BATCH_MODR_AH_MODE:
		batching = 1;
		sensor_batch_mark_off(0);
		return 0;
	case QL_BATCH_MODC_AH_MODE:
		sensor_batch_mark_off(1);
		return 0;
	default :
		batching = 0;
		break;
	}

	if (copy_from_user(&hc, (void __user *)arg,
		sizeof(struct hc_info)))
		return -EFAULT;

	batching = 0;
	QLEOS_DBG("command = 0x%x\n", hc.command);
	QLEOS_DBG("param = 0x%x\tparam_len = 0x%x\t"
		"response = 0x%x\tresponse_len = 0x%x\n",
		hc.param_data[0], hc.param_data_len,
		hc.response[0], hc.response_len);

	retval = qleoss3_hostcmd(&hc);
	if (retval) {
		QLEOS_ERR("Failed to send the command to s3\n");
		return retval;
	}
	if (hc.response_len) {
		if (copy_to_user((void __user *)hc.response,
			&result_response_buf[8],1024));
		QLEOS_DBG("response is %u\n", hc.response[0]);

		if (copy_to_user((void __user *)arg, &hc, sizeof(hc)))
			return -EFAULT;
	}

	EXITED();
#endif
	return retval;
}

static struct file_operations qleoss3_fops = {
	.owner	  	= THIS_MODULE,
	.read	  	= qleoss3_read,
	.write	  	= qleoss3_write,
	.unlocked_ioctl	= qleoss3_ioctl,
	.open	  	= qleoss3_open,
	.release  	= qleoss3_release,
};


static int __init qleos_s3_init(void)
{
	int retval = 0;
	int ir_major = 0, ir_minor = 0;
	struct device *device;
#ifdef DEBUG_GPIO
	int error_chk;
#endif
	ENTERED();


	/* Get major number */
	if (ir_major) {
		dev = MKDEV(ir_major, ir_minor);
		retval = register_chrdev_region(dev, NUM_DEV, DRIVER_NAME);
	} else {
		retval = alloc_chrdev_region(&dev, ir_minor, NUM_DEV,
			DRIVER_NAME);
		ir_major = MAJOR(dev);
	}
	if (retval < 0) {
		QLEOS_ERR("Can't get major number \n");
		return retval;
	}

	/* Initialize device */
	cdev_init(&cdev, &qleoss3_fops);
	cdev.owner = THIS_MODULE;
	cdev.ops = &qleoss3_fops;

	/* Add device to kernel */
	retval = cdev_add(&cdev, dev, 1);
	if (retval != 0) {
		QLEOS_ERR("Adding driver failed \n");
		goto free_chrdev;
	}

	/* Create device class and device node */
	s_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(s_class)) {
		QLEOS_ERR("Device class creation failed \n");
		retval = PTR_ERR(s_class);
		goto free_cdev;
	}

	device = device_create(s_class, NULL, dev, NULL,
			DEVICE_NAME);
	if (IS_ERR(device)) {
		QLEOS_ERR("Creating /dev/qleoss3 failed \n");
		retval = PTR_ERR(device);
		goto free_class;
	}
	init_waitqueue_head(&wait_int);

	spin_lock_init(&queue_lock);

	mutex_init(&data_mutex);
	mutex_init(&eos3_irq_mutex);
	mutex_init(&eose3_evt_mutex);
	mutex_init(&eose3_eif_mutex);

	atomic_set(&pending_hc_cmplt, false);
	atomic_set(&batch_margin, false);
	atomic_set(&m4_batch_buf_full, false);
	atomic_set(&batch_lat_exp, false);
	atomic_set(&m4_buf_blank_full, false);
	atomic_set(&ffe_batch_buf_full, false);
	atomic_set(&ffe_buf_blank_full, false);
	atomic_set(&ffe_buf_blank_full, false);
	atomic_set(&wfl_S3toAP_status, false);
	atomic_set(&wfl_S3toAP_finish_flg, false);
	atomic_set(&g_sns_flush_type, 0);
	atomic_set(&g_is_flush_exec, false);
	//atomic_set(&g_interrupt_done, false);
	atomic_set(&wait_event_flag, false);
	//evt_wq = create_singlethread_workqueue("evt_wq");
	evt_wq = create_workqueue("evt_wq");
	if (!evt_wq) {
		QLEOS_ERR("can't create interrupt queue-evt_wq\n");
		goto free_device;
	}
	rbt_wq = create_singlethread_workqueue("rbt_wq");
	if (!rbt_wq) {
		QLEOS_ERR("can't create interrupt queue-rbt_wq\n");
		goto free_queue_half;
	}
    INIT_WORK(&qleoss3_rbt_work, hw_reboot_event_work_func);

#ifdef DEBUG_GPIO
	error_chk = gpio_request(24, "debug_gpio");
	if (error_chk >= 0)
		gpio_direction_output(24, 0);
	else
		STEST_ERR("---GPIO 24 req failed %d\n", error_chk);
#endif
	qleoss3_workqueue_init();
	INIT_DELAYED_WORK(&wfl_monitor_dwork, qleos_wfl_monitor);
	INIT_DELAYED_WORK(&rcvry_during_rcvry_dwork, re_rcvry_work);
	//mutex_init(&irq_lock);

	/* allocate 1k memory for resoponse */
	//hc.response = kmalloc(1024, GFP_KERNEL);

	if(QLSPI_Register_Isr(qleos_s3_isr) != QL_STATUS_OK) {
		QLEOS_ERR("failed to register sensor test callback\n");
		retval = -1;
		goto free_queue;
	}

	//retval = qleos_s3_create_sysfs();
	if (retval) {
		QLEOS_ERR("Failed to create sysfs entries\n");
		goto free_queue;
	}
	EXITED();
	return retval;
free_queue:
	if (rbt_wq != NULL) {
		flush_workqueue(rbt_wq);
		destroy_workqueue(rbt_wq);
		rbt_wq = NULL;
	}
free_queue_half:
	if (evt_wq != NULL) {
		flush_workqueue(evt_wq);
		destroy_workqueue(evt_wq);
		evt_wq = NULL;
	}
free_device:
	device_destroy(s_class, dev);
free_class:
	class_destroy(s_class);
free_cdev:
	cdev_del(&cdev);
free_chrdev:
	unregister_chrdev_region(dev, NUM_DEV);

	EXITED();
	return retval;
}

static void __exit qleos_s3_exit(void)
{
	ENTERED();

	if (evt_wq != NULL) {
		destroy_workqueue(evt_wq);
		evt_wq = NULL;
	}
	if (rbt_wq != NULL) {
		destroy_workqueue(rbt_wq);
		rbt_wq = NULL;
	}

	kfree(hc.response);
	device_destroy(s_class, dev);
	class_destroy(s_class);
	cdev_del(&cdev);
	unregister_chrdev_region(dev, NUM_DEV);
#ifdef CONFIG_QLEOSS3_SYSFS
	qleos_s3_remove_sysfs();
#endif
	EXITED();
}


late_initcall(qleos_s3_init);
module_exit(qleos_s3_exit);

MODULE_DESCRIPTION("Sensor AP-M4 Communication Test Driver");
MODULE_AUTHOR("QuickLogic Private Limited");
MODULE_LICENSE("GPL");
