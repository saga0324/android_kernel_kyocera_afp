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


#include <linux/ql/model_file_loader.h>

#define NET_FILENAME			"/vendor/firmware/Default_Net.bin"
#define GRAM_FILENAME			"/vendor/firmware/Default_Gram.bin"
#define NET_TARGET_ADDR			0x20035000
#define GRAM_TARGET_ADDR		0x20048400

static struct net_gram_resp net_gram_details = {0, 0, 0, 0};
static QL_Status load_net_file_to_s3(uint8_t *bin_array);
static QL_Status load_gram_file_to_s3(uint8_t *bin_array);

/*************** Driver Fuctions **********************/
static int bin_write(uint8_t *bin_array, uint8_t turn);
static int change_state(uint8_t state);

static QL_Status load_net_file_to_s3(uint8_t *bin_array)
{
	int retval = 0;

	MLDR_DBG("net file address = 0x%08x, netfile size = %d\n",
			net_gram_details.net_addr,
			net_gram_details.net_size);
	retval = QLSPI_Write_S3_Mem(net_gram_details.net_addr,
			(uint8_t*)(bin_array), net_gram_details.net_size);
	if(retval != QL_STATUS_OK) {
		MLDR_ERR("Net file Write Failed to S3\n");
		return QL_STATUS_ERROR;
	}
	MLDR_DBG("Net File Write Done!!!\n");
	return QL_STATUS_OK;
}

static QL_Status load_gram_file_to_s3(uint8_t *bin_array)
{
	int retval = 0;

	MLDR_DBG("gram file address = 0x%08x, gram file size = %d\n",
			net_gram_details.gram_addr,
			net_gram_details.gram_size);
	retval = QLSPI_Write_S3_Mem(net_gram_details.gram_addr,
			(uint8_t*)bin_array, net_gram_details.gram_size);
	if(retval != QL_STATUS_OK) {
		MLDR_ERR("Gram file Write Failed to S3\n");
		return QL_STATUS_ERROR;
	}
	MLDR_DBG("Gram File Write Done!!!\n");

	return QL_STATUS_OK;
}

static int bin_write(uint8_t *bin_array, uint8_t turn)
{
	MLDR_DBG("start\n");
	switch(turn) {
	case NET_FILE_TURN:
		load_net_file_to_s3(bin_array);
		break;
	case GRAM_FILE_TURN:
		load_gram_file_to_s3(bin_array);
		break;
	default:
		MLDR_ERR("Turn collapsed... It should not come here.\n");
		break;
	}
	MLDR_DBG("end\n");
    return 0;
}

static int change_state(uint8_t state)
{
	int retval = 0;
#ifdef USE_HOSTCMD_SETGETMODELINFO
	struct hc_info hc = {0};
#endif /*USE_HOSTCMD_SETGETMODELINFO*/

	MLDR_DBG("start\n");
	switch (state) {
	case INIT_MODEL_LOADER:
		MLDR_DBG("INIT_MODEL_LOADER\n");
#ifdef USE_HOSTCMD_SETGETMODELINFO
		/* Send get command to get the net file address */
		hc.command = prepare_host_command(VOICE, VOICE_KPD,
				GET_MODEL_INFO);
		hc.response_len = sizeof(net_gram_details);

		//get_command(command, sizeof(net_gram_details),  response_buf);
		//MLDR_DBG("INIT_MODEL_LOADER cmd = 0x%x\n", command);
		qleoss3_hostcmd(&hc);
		memcpy(&net_gram_details, &&hc.response[0], sizeof(net_gram_details));
		MLDR_DBG("net file address = %p, netfile size = %d, gram file address = %p, gram file size = %d\n",
			(void*)net_gram_details.net_addr,
			net_gram_details.net_size,
			(void*)net_gram_details.gram_addr,
			net_gram_details.gram_size);
#endif /*USE_HOSTCMD_SETGETMODELINFO*/
		break;
	case DONE_MODEL_LOADER:
		MLDR_DBG("DONE_MODEL_LOADER\n");
		break;
	default :
		MLDR_ERR("Unknown state\n");
		break;
	}

	MLDR_DBG("end\n");
	return retval;
}

static void set_netgram_detail(uint32_t net_size, uint32_t gram_size)
{
	MLDR_DBG("start\n");
	net_gram_details.net_addr = NET_TARGET_ADDR;
	if(net_size%4) {
		net_size += (4 - (net_size%4) );
		MLDR_DBG("After 4Byte Alligned net_size = %d\n", net_size);
	}
	net_gram_details.net_size = net_size;

	net_gram_details.gram_addr = GRAM_TARGET_ADDR;
	if(gram_size%4) {
		gram_size += (4 - (gram_size%4) );
		MLDR_DBG("After 4Byte Alligned gram_size = %d\n", gram_size);
	}
	net_gram_details.gram_size = gram_size;
	MLDR_DBG("end\n");
}

int write_model_file(void)
{
	int ret = QL_STATUS_OK;
	uint8_t *net_value_array;
	uint8_t *gram_value_array;
	uint32_t net_size = 0;
	uint32_t gram_size = 0;
	MLDR_DBG("start\n");
	//load net/gram bin from /vendor/firmware/
	if(load_fw_to_arr(&net_value_array, &net_size, NET_FILENAME)){
		MLDR_ERR("end [load model net file]\n");
		return QL_STATUS_ERROR;
	}
	if(load_fw_to_arr(&gram_value_array, &gram_size, GRAM_FILENAME)){
		MLDR_ERR("end [load model gram file]\n");
		return QL_STATUS_ERROR;
	}
	set_netgram_detail(net_size, gram_size);
	bin_write(net_value_array, NET_FILE_TURN);
	bin_write(gram_value_array, GRAM_FILE_TURN);
	MLDR_DBG("end[%d]\n" , ret);
	return ret;
}

int start_model_file_loader(void)
{
	int ret = QL_STATUS_OK;
#ifdef CONFIG_MICON_MEAS_PERFORMANCE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
	enter = ktime_get();
#endif /*CONFIG_MICON_MEAS_PERFORMANCE_LOG*/

	MLDR_DBG("start\n");
	change_state(INIT_MODEL_LOADER);
	ret = write_model_file();
	if(ret != QL_STATUS_OK){
		MLDR_ERR("end [model_file write]\n");
		return ret;
	}
	change_state(DONE_MODEL_LOADER);
#ifdef CONFIG_MICON_MEAS_PERFORMANCE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_MICON_MEAS_PERFORMANCE_LOG */

	MLDR_DBG("end\n");
	return ret;
}

static int __init model_file_loader_init(void)
{
	MLDR_DBG("start\n");
	MLDR_DBG("end\n");
	return 0;
}

void __exit model_file_loader_exit(void)
{
	MLDR_DBG("start\n");
	MLDR_DBG("end\n");
}

late_initcall(model_file_loader_init);
module_exit(model_file_loader_exit);

MODULE_DESCRIPTION("Model File Loader driver");
MODULE_AUTHOR("QuickLogic Private Limited");
MODULE_LICENSE("GPL");

