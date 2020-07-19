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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/sensor_prevention.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"
#include <asm/div64.h>

#ifdef CONFIG_USE_DBG_EOS_DATA_EXTRACTION
//#include "sensor_dbg_eoss3.h"
#endif /*CONFIG_USE_DBG_EOS_DATA_EXTRACTION*/


//#define DEBUG_SENSOR_EXT_DRIVER

#ifdef DEBUG_SENSOR_EXT_DRIVER
#undef SENSOR_N_LOG
#define SENSOR_N_LOG(msg, ...) \
    pr_notice("[SENSOR][%s][N](%d): " msg, __func__, __LINE__, ## __VA_ARGS__)
#endif

#define SENSOR_EXT_FUNC_PEDO    0x01
#define SENSOR_EXT_FUNC_VEHI    0x02
#define SENSOR_EXT_FUNC_BARO    0x04
#define SENSOR_EXT_FUNC_IWIFI   0x10

#define EXT_FUNC_VEHI_KIND      0x01
#define EXT_FUNC_PEDO_KIND      0x02
#define EXT_FUNC_IWIFI_KIND     0x10


static atomic_t sns_ext_pedom_interrupt_kind;
static atomic_t sns_ext_pedom_interrupt_flag;
static atomic_t sns_ext_vehi_interrupt_kind;
static atomic_t sns_ext_vehi_interrupt_flag;
static atomic_t sns_ext_iwifi_interrupt_kind;
static atomic_t sns_ext_iwifi_interrupt_flag;
static atomic_t sns_ext_backupStep          = ATOMIC_INIT(0);
static atomic_t sns_ext_backupCal           = ATOMIC_INIT(0);
static atomic_t sns_ext_backupSyncRunCal    = ATOMIC_INIT(0);
static atomic_t sns_ext_ofsStep             = ATOMIC_INIT(0);
static atomic_t sns_ext_ofsCal              = ATOMIC_INIT(0);
static atomic_t sns_ext_ofsSyncRunCal       = ATOMIC_INIT(0);

static struct mutex dailys_vib_mutex;
static int dailys_vib_mode = 0x00;
static int sensor_ext_initialize = 0x00;
static int64_t kernel_basetime_ofs = 0;
static int64_t cur_ktime = 0;
static int32_t cur_miconbasetime = 0;
static bool is_1st_time_of_evmarkoff = true;

static ssize_t sensor_ext_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ext_pedom_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_pedom_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ext_iwifi_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_iwifi_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t sensor_ext_pedom_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_vehi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_iwifi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_clear_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t sensor_ext_rt_state_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t sensor_ext_vib_interlocking_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ext_sound_vib_prevention_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);

static int64_t convert_host_event_timestamp(int32_t ts, int32_t prv_ts,
                                            int64_t tn, int64_t prvtn,
                                            int32_t bts);

static int32_t sensor_ext_open( struct inode* inode, struct file* filp );
static int32_t sensor_ext_release( struct inode* inode, struct file* filp );
static unsigned int sensor_ext_pedom_poll(struct file *fp, poll_table *wait);
static unsigned int sensor_ext_iwifi_poll(struct file *fp, poll_table *wait);
static unsigned int sensor_ext_vehi_poll(struct file *fp, poll_table *wait);

static void sensor_ext_set_input_params( struct input_dev *dev );
static void sensor_ext_clr_backupofs_val(void);
static void sensor_ext_update_backup_step_cal(const uint16_t step, const uint16_t cal);
static void sensor_ext_backup_step_cal(const uint16_t header, void *buf);

static DEVICE_ATTR(dailys_enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_enable_show,
    sensor_ext_enable_store
);
static DEVICE_ATTR(dailys_param,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_pedom_param_show,
    sensor_ext_pedom_param_store
);
static DEVICE_ATTR(dailys_iwifi_param,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_iwifi_param_show,
    sensor_ext_iwifi_param_store
);
static DEVICE_ATTR(dailys_pedo_data,
    S_IRUSR|S_IRGRP,
    sensor_ext_pedom_data_show,
    NULL
);
static DEVICE_ATTR(dailys_vehi_data,
    S_IRUSR|S_IRGRP,
    sensor_ext_vehi_data_show,
    NULL
);
static DEVICE_ATTR(dailys_iwifi_data,
    S_IRUSR|S_IRGRP,
    sensor_ext_iwifi_data_show,
    NULL
);
static DEVICE_ATTR(dailys_clear,
    S_IWUSR|S_IWGRP,
    NULL,
    sensor_ext_clear_store
);
static DEVICE_ATTR(dailys_rt_state,
    S_IRUSR|S_IRGRP,
    sensor_ext_rt_state_show,
    NULL
);
static DEVICE_ATTR(dailys_vib_interlocking,
    S_IWUSR|S_IWGRP,
    NULL,
    sensor_ext_vib_interlocking_store
);
static DEVICE_ATTR(dailys_soundvib_prevention,
    S_IWUSR|S_IWGRP,
    NULL,
    sensor_ext_sound_vib_prevention_store
);
static struct attribute *sensor_ext_attributes[] = {
    &dev_attr_dailys_enable.attr,
    &dev_attr_dailys_param.attr,
    &dev_attr_dailys_iwifi_param.attr,
    &dev_attr_dailys_pedo_data.attr,
    &dev_attr_dailys_vehi_data.attr,
    &dev_attr_dailys_iwifi_data.attr,
    &dev_attr_dailys_clear.attr,
    &dev_attr_dailys_rt_state.attr,
    &dev_attr_dailys_vib_interlocking.attr,
    &dev_attr_dailys_soundvib_prevention.attr,
    NULL
};

static struct attribute_group sensor_ext_attr_grp = {
    .attrs = sensor_ext_attributes
};

struct sensor_input_info_str sensor_ext_input_info =
{
    NULL,
    sensor_ext_set_input_params,
    &sensor_ext_attr_grp,
};

static struct file_operations sensor_ext_pedom_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_open,
  .release = sensor_ext_release,
  .poll    = sensor_ext_pedom_poll,
};
static struct miscdevice sensor_ext_pedom_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_pedom",
  .fops  = &sensor_ext_pedom_fops,
};

static struct file_operations sensor_ext_vehi_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_open,
  .release = sensor_ext_release,
  .poll    = sensor_ext_vehi_poll,
};
static struct miscdevice sensor_ext_vehi_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_vehicle",
  .fops  = &sensor_ext_vehi_fops,
};

static struct file_operations sensor_ext_iwifi_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_open,
  .release = sensor_ext_release,
  .poll    = sensor_ext_iwifi_poll,
};
static struct miscdevice sensor_ext_iwifi_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_iwifi",
  .fops  = &sensor_ext_iwifi_fops,
};

static wait_queue_head_t sensor_ext_pedom_p_queue;
static wait_queue_head_t sensor_ext_iwifi_p_queue;
static wait_queue_head_t sensor_ext_vehi_p_queue;
static DailysSetIWifiParam sensor_ext_iwifi_param;
static struct sensor_ext_pedom_param_str  sensor_ext_pedom_param;
static struct delayed_work vib_work;

static ssize_t sensor_ext_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int enable = 0;
    SENSOR_N_LOG("start");

    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_PEDO)) {
        enable |= SENSOR_EXT_FUNC_PEDO;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_VEHI)) {
        enable |= SENSOR_EXT_FUNC_VEHI;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_IWIFI)) {
        enable |= SENSOR_EXT_FUNC_IWIFI;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_BARO)) {
        enable |= SENSOR_EXT_FUNC_BARO;
    }

    SENSOR_N_LOG("end");
    return sprintf(buf, "%x\n",enable);
}

static bool check_1st_enable(const int32_t type, const bool enable)
{
    bool result = false;
    SENSOR_D_LOG("start");
    if(!g_micon_pedo_status && !g_micon_baro_status){
        if( ( (type & (SENSOR_EXT_FUNC_PEDO|SENSOR_EXT_FUNC_BARO)) != 0) && (enable != 0) ){
            SENSOR_D_LOG("1st enable.");
            result = true;
        }
    }
    SENSOR_D_LOG("end");
    return result;
}

static ssize_t sensor_ext_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t enable = 0;
    int32_t type = 0;
    int32_t period = 0;
    bool ktimestampOfs_get_flg = false;
    SENSOR_N_LOG("start");

    sscanf(buf, "%x %x %x", &type, &enable, &period);
    SENSOR_N_LOG("type[%d] enable[%d] period[%d]",(int)type, (int)enable, (int)period);
    if(check_1st_enable(type, !!enable)){
        ktimestampOfs_get_flg = true;
    }

    if( (type & SENSOR_EXT_FUNC_PEDO) == SENSOR_EXT_FUNC_PEDO ){
        if (enable) {
            sensor_set_poll_time( SENSOR_EXT_PEDO, NULL, period );
            if(!g_micon_pedo_status){
                sensor_ext_clr_backupofs_val();
            }
        }
        sensor_enable( SENSOR_EXT_PEDO, NULL, (bool)enable );
    }

    if ((type & SENSOR_EXT_FUNC_VEHI) == SENSOR_EXT_FUNC_VEHI ){
        sensor_enable( SENSOR_EXT_VEHI, NULL, (bool)enable );
    }

    if ((type & SENSOR_EXT_FUNC_IWIFI) == SENSOR_EXT_FUNC_IWIFI ){
        sensor_enable( SENSOR_EXT_IWIFI, NULL, (bool)enable );
    }

    if( (type & SENSOR_EXT_FUNC_BARO) == SENSOR_EXT_FUNC_BARO ){
        if (enable) {
            sensor_set_poll_time( SENSOR_EXT_BARO, NULL, period );
        }
        sensor_enable( SENSOR_EXT_BARO, NULL, (bool)enable );
    }
    if(ktimestampOfs_get_flg){
        kernel_basetime_ofs = GET_CURRENT_TIMESTAMP_NS();
    }
    if(!g_micon_pedo_status && !g_micon_baro_status){
        kernel_basetime_ofs = GET_CURRENT_TIMESTAMP_NS();
        sensor_ext_clear_ev_markoff_status();
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_pedom_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sprintf( buf, "%d %d %d\n",
                    sensor_ext_pedom_param.step_wide,
                    sensor_ext_pedom_param.weight,
                    sensor_ext_pedom_param.vehi_type );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_pedom_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d",
            &sensor_ext_pedom_param.step_wide,
            &sensor_ext_pedom_param.weight,
            &sensor_ext_pedom_param.vehi_type );

    ret = sensor_ext_set_param(SENSOR_EXT_PEDO,NULL,&sensor_ext_pedom_param);
    if(ret < 0 ){
        SENSOR_ERR_LOG("sensor_ext_set_param --> ret[%d]",(int)ret);
    }


    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_iwifi_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;

    SENSOR_N_LOG("start");
    ret = sprintf( buf, "%d %d %d %d\n",
                    sensor_ext_iwifi_param.m_nPedoStartStep,
                    sensor_ext_iwifi_param.m_nPedoEndTime,
                    sensor_ext_iwifi_param.m_nVehiStartTime,
                    sensor_ext_iwifi_param.m_nVehiEndTime );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_iwifi_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d %d",
            &sensor_ext_iwifi_param.m_nPedoStartStep,
            &sensor_ext_iwifi_param.m_nPedoEndTime,
            &sensor_ext_iwifi_param.m_nVehiStartTime,
            &sensor_ext_iwifi_param.m_nVehiEndTime );

    ret = sensor_ext_set_param(SENSOR_EXT_IWIFI,&sensor_ext_iwifi_param,NULL);
    if(ret < 0 ){
        SENSOR_ERR_LOG("sensor_ext_set_param --> ret[%d]",(int)ret);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_pedom_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;
    char wk_buf[512];

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(sns_get_reset_status() == false){
        get_ret = sensor_ext_get_data( SENSOR_EXT_PEDO, &read_data );
        if(get_ret != SNS_RC_OK){
            SENSOR_ERR_LOG("get pedometer data error.");
            memset(&read_data, 0, sizeof(read_data));
        }
    }

    read_data.pedm_data.usStepCnt    += atomic_read(&sns_ext_ofsStep);
    read_data.pedm_data.usCal        += atomic_read(&sns_ext_ofsCal);
    read_data.pedm_data.usSyncRunCal += atomic_read(&sns_ext_ofsSyncRunCal);

    ret = snprintf( buf, 511,
                        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d "
                        "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                   read_data.pedm_data.usStepCnt,
                   read_data.pedm_data.usWalkTime,
                   read_data.pedm_data.usCal,
                   read_data.pedm_data.usBodyFat,
                   read_data.pedm_data.usExercise,
                   read_data.pedm_data.usMets,
                   read_data.pedm_data.usSpeed,
                   read_data.pedm_data.usRunStatus,
                   read_data.pedm_data.usRunStepCnt,
                   read_data.pedm_data.usRunTime,
                   read_data.pedm_data.usStExercise,
                   read_data.pedm_data.usStCal,
                   read_data.pedm_data.usStBodyFat,
                   read_data.pedm_data.usSportExercise,
                   read_data.pedm_data.usSyncRunStepCnt,
                   read_data.pedm_data.usSyncRunTime,
                   read_data.pedm_data.usSyncRunCal,
                   read_data.pedm_data.usRunCal,
                   read_data.pedm_data.usRunExercise,
                   read_data.pedm_data.usPS_InitBaseHeight,
                   read_data.pedm_data.usPS_ActHeight,
                   read_data.pedm_data.usPS_ActHeightMin,
                   read_data.pedm_data.usPS_ActHeightAve,
                   read_data.pedm_data.usPS_ActHeightMax,
                   read_data.pedm_data.LogTimeStamp3sec,
                   read_data.pedm_data.PD_Log_DetState,
                   read_data.pedm_data.PD_Log_SectionStep,
                   read_data.pedm_data.PD_Log_SectionCal,
                   read_data.pedm_data.RN_Log_SectionStep,
                   read_data.pedm_data.RN_Log_SectionCal,
                   read_data.pedm_data.PD_Log_SectionUpH,
                   read_data.pedm_data.PD_Log_SectionDnH,
                   read_data.pedm_data.PD_Log_SectionUpDnCal,
                   read_data.pedm_data.VC_Log_DetState,
                   read_data.pedm_data.VC_Log_SectionCal,
                   read_data.pedm_data.VC_Log_SectionUpH,
                   read_data.pedm_data.VC_Log_SectionDnH,
                   read_data.pedm_data.VC_Log_SectionUpDnCal,
                   read_data.pedm_data.PS_SectionPress,
                   read_data.pedm_data.PS_SectionHeight,
                   read_data.pedm_data.PS_SectionActiveHeight );

    memset(&read_data, 0, sizeof(read_data));
    if (sns_get_reset_status() == false) {
        get_ret = sensor_ext_get_data( SENSOR_EXT_VEHI, &read_data );
        if(get_ret != SNS_RC_OK){
            SENSOR_ERR_LOG("get vehicle data error.");
            memset(&read_data, 0, sizeof(read_data));
        }
    }
    memset(wk_buf, 0x00, sizeof(wk_buf));
    ret += snprintf( wk_buf,  sizeof(wk_buf) -1, " %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                   read_data.vehi_data.usVehiStatus,
                   read_data.vehi_data.usVehiKind,
                   read_data.vehi_data.usVehiDetectTime,
                   read_data.vehi_data.usVehiRideTime,
                   read_data.vehi_data.usVehiRideCal,
                   read_data.vehi_data.usVehiBodyFat,
                   read_data.vehi_data.usVehiExercise,
                   read_data.vehi_data.usVehiMets,
                   read_data.vehi_data.usVehiStExercise,
                   read_data.vehi_data.usVehiStRideCal,
                   read_data.vehi_data.usVehiStBodyFat,
                   read_data.vehi_data.usVehiBiExercise,
                   read_data.vehi_data.usVehiBiRideCal,
                   read_data.vehi_data.usVehiBiBodyFat,
                   read_data.vehi_data.usVehiSportExercise );
    strcat(buf, wk_buf);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_vehi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(sns_get_reset_status() == false){
        get_ret = sensor_ext_get_data( SENSOR_EXT_VEHI, &read_data );
    }
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                       read_data.vehi_data.usVehiStatus,
                       read_data.vehi_data.usVehiKind,
                       read_data.vehi_data.usVehiDetectTime,
                       read_data.vehi_data.usVehiRideTime,
                       read_data.vehi_data.usVehiRideCal,
                       read_data.vehi_data.usVehiBodyFat,
                       read_data.vehi_data.usVehiExercise,
                       read_data.vehi_data.usVehiMets,
                       read_data.vehi_data.usVehiStExercise,
                       read_data.vehi_data.usVehiStRideCal,
                       read_data.vehi_data.usVehiStBodyFat,
                       read_data.vehi_data.usVehiBiExercise,
                       read_data.vehi_data.usVehiBiRideCal,
                       read_data.vehi_data.usVehiBiBodyFat,
                       read_data.vehi_data.usVehiSportExercise );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_iwifi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(sns_get_reset_status() == false){
        get_ret = sensor_ext_get_data( SENSOR_EXT_IWIFI, &read_data );
    }
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d %d\n",
                       read_data.iwifi_data.usPedoStatus,
                       read_data.iwifi_data.usVehiStatus );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_clear_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t clear_req;
    int32_t ret;

    SENSOR_N_LOG("start");
    sscanf(buf, "%x", &clear_req);

    ret = sensor_ext_clear(clear_req);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_rt_state_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    get_ret = sensor_ext_get_data( SENSOR_EXT_PEDO, &read_data );
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d \n", read_data.pedm_data.usRTState );
    }

    SENSOR_N_LOG("end");
    return ret;
}

void sensor_ext_update_ofs_val(void)
{
    int32_t step   = atomic_read(&sns_ext_backupStep);
    int32_t cal    = atomic_read(&sns_ext_backupCal);
    int32_t sr_cal = atomic_read(&sns_ext_backupSyncRunCal);
    SENSOR_D_LOG("start");
    atomic_set(&sns_ext_ofsStep, step);
    atomic_set(&sns_ext_ofsCal, cal);
    atomic_set(&sns_ext_ofsSyncRunCal, sr_cal);
    SENSOR_ERR_LOG("update StepOfs[%d], CalOfs[%d], SyncRunCalOfs[%d]",
                atomic_read(&sns_ext_ofsStep),
                atomic_read(&sns_ext_ofsCal),
                atomic_read(&sns_ext_ofsSyncRunCal) );
    SENSOR_D_LOG("end");
}

static void sensor_ext_clr_backupofs_val(void)
{
    SENSOR_D_LOG("start");
    atomic_set(&sns_ext_backupStep,0);
    atomic_set(&sns_ext_backupCal,0);
    atomic_set(&sns_ext_backupSyncRunCal,0);
    atomic_set(&sns_ext_ofsStep,0);
    atomic_set(&sns_ext_ofsCal,0);
    atomic_set(&sns_ext_ofsSyncRunCal,0);
    SENSOR_D_LOG("clear all step_ofs, cal_ofs");
    SENSOR_D_LOG("end");
}

static void sensor_ext_update_backup_step_cal(uint16_t step, uint16_t cal)
{
    SENSOR_D_LOG("start");
    atomic_add((int)step, &sns_ext_backupStep);
    atomic_add((int)cal,  &sns_ext_backupCal);
    atomic_add((int)cal,  &sns_ext_backupSyncRunCal);
    SENSOR_D_LOG("backup Step[%d], Cal[%d], SyncRunCal[%d]",
                atomic_read(&sns_ext_backupStep),
                atomic_read(&sns_ext_backupCal),
                atomic_read(&sns_ext_backupSyncRunCal) );
    SENSOR_D_LOG("end");
}

#define ACC_VIB_INTERLOCKING_OFF_TIMEOUT 1000
static void sensor_ext_vib_interlocking_off(struct work_struct *work)
{
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    SENSOR_D_LOG("start");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    sensor_com_mutex_lock();
    sns_dailys_set_vib_interlocking(dailys_vib_mode);
    sensor_com_mutex_unlock();
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    SENSOR_D_LOG("end");
}

static void sensor_ext_vib_interlocking_off_nowait(int ope)
{
    SENSOR_D_LOG("start");
    sensor_com_mutex_lock();
    sns_dailys_set_vib_interlocking(ope);
    sensor_com_mutex_unlock();
    SENSOR_D_LOG("end");
}

static void sensor_ext_vib_interlocking_on(int ope)
{
    SENSOR_D_LOG("start");
    if (SENSOR_ON == sensor_get_status(SENSOR_EXT_PEDO)) {
        cancel_delayed_work_sync(&vib_work);
        sensor_com_mutex_lock();
        sns_dailys_set_vib_interlocking(ope);
        sensor_com_mutex_unlock();
    }
    SENSOR_D_LOG("end");
}

static ssize_t sensor_ext_vib_interlocking_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    uint32_t ope = 0;

    SENSOR_D_LOG("start");
    sscanf(buf, "%d", &ope);
    mutex_lock(&dailys_vib_mutex);
    if(ope){
        dailys_vib_mode |= VIB_INTERLOCKING_ON;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
        enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
        sensor_ext_vib_interlocking_on(dailys_vib_mode);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
        exit = ktime_get();
        func_time_us = ktime_to_us(ktime_sub(exit, enter));
        enter_us = ktime_to_us(enter);
        exit_us = ktime_to_us(exit);
        printk(KERN_NOTICE "[IT_TEST] %s:on_root bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    } else {
        dailys_vib_mode &= VIB_INTERLOCKING_OFF;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
        enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
        schedule_delayed_work(&vib_work,
                      msecs_to_jiffies(ACC_VIB_INTERLOCKING_OFF_TIMEOUT));
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
        exit = ktime_get();
        func_time_us = ktime_to_us(ktime_sub(exit, enter));
        enter_us = ktime_to_us(enter);
        exit_us = ktime_to_us(exit);
        printk(KERN_NOTICE "[IT_TEST] %s:off_root bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    }
    mutex_unlock(&dailys_vib_mutex);

    SENSOR_D_LOG("end");
    return count;
}

int32_t sensor_ext_sound_vib_prevention_switch(uint32_t ope)
{
    int32_t ret = 0;

    if(!sensor_ext_initialize){
        SENSOR_ERR_LOG("sensor_ext_driver_init does not end.");
        ret = -1;
        goto exit;
    }

    SENSOR_D_LOG("start");
    mutex_lock(&dailys_vib_mutex);
    if(ope){
        dailys_vib_mode |= SPEAKER_INTERLOCKING_ON;
        sensor_ext_vib_interlocking_on(dailys_vib_mode);
    } else {
        dailys_vib_mode &= SPEAKER_INTERLOCKING_OFF;
        sensor_ext_vib_interlocking_off_nowait(dailys_vib_mode);
    }
    mutex_unlock(&dailys_vib_mutex);

exit:
    SENSOR_D_LOG("end");
    return ret;
}
EXPORT_SYMBOL(sensor_ext_sound_vib_prevention_switch);

static ssize_t sensor_ext_sound_vib_prevention_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ope = 0;

    SENSOR_D_LOG("start");
    sscanf(buf, "%d", &ope);
    sensor_ext_sound_vib_prevention_switch(ope);
    SENSOR_D_LOG("end");
    return count;
}

static int32_t sensor_ext_open( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_ext_release( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static unsigned int sensor_ext_pedom_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &sensor_ext_pedom_p_queue, wait);
    int_flag = atomic_read(&sns_ext_pedom_interrupt_flag);
    if(int_flag)
    {
        ret = POLLIN | POLLPRI;
        atomic_set(&sns_ext_pedom_interrupt_kind, 0);
        atomic_set(&sns_ext_pedom_interrupt_flag, 0);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static unsigned int sensor_ext_vehi_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &sensor_ext_vehi_p_queue, wait);
    int_flag = atomic_read(&sns_ext_vehi_interrupt_flag);
    if(int_flag)
    {
        ret = POLLIN | POLLPRI;
        atomic_set(&sns_ext_vehi_interrupt_kind, 0);
        atomic_set(&sns_ext_vehi_interrupt_flag, 0);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static unsigned int sensor_ext_iwifi_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &sensor_ext_iwifi_p_queue, wait);
    int_flag = atomic_read(&sns_ext_iwifi_interrupt_flag);
    if(int_flag)
    {
        ret = POLLIN | POLLPRI;
        atomic_set(&sns_ext_iwifi_interrupt_kind, 0);
        atomic_set(&sns_ext_iwifi_interrupt_flag, 0);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static void sensor_ext_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "sensor_ext";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Y, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Z, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_RX, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    input_set_events_per_packet(dev, 512);

    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_pedo_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_EXT_PEDO) ) {
        tmp_kind = atomic_read(&sns_ext_pedom_interrupt_kind);
        tmp_kind |= EXT_FUNC_PEDO_KIND;
        atomic_set(&sns_ext_pedom_interrupt_kind,tmp_kind);
        atomic_set(&sns_ext_pedom_interrupt_flag,1);
        SENSOR_A_LOG("wake up PEDOMETER");
        wake_up_interruptible(&sensor_ext_pedom_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_iwifi_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_EXT_IWIFI) ) {
        tmp_kind = atomic_read(&sns_ext_iwifi_interrupt_kind);
        tmp_kind |= EXT_FUNC_IWIFI_KIND;
        atomic_set(&sns_ext_iwifi_interrupt_kind,tmp_kind);
        atomic_set(&sns_ext_iwifi_interrupt_flag,1);
        SENSOR_A_LOG("wake up IWIFI");
        wake_up_interruptible(&sensor_ext_iwifi_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_vehi_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_EXT_VEHI) ) {
        tmp_kind = atomic_read(&sns_ext_vehi_interrupt_kind);
        tmp_kind |= EXT_FUNC_VEHI_KIND;
        atomic_set(&sns_ext_vehi_interrupt_kind,tmp_kind);
        atomic_set(&sns_ext_vehi_interrupt_flag,1);
        SENSOR_A_LOG("wake up VEHI");
        wake_up_interruptible(&sensor_ext_vehi_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

struct sensor_ext_dailys_event_t {
    uint8_t         header;
    uint8_t         data[0];
};

struct sensor_ext_dailys_event_info_t {
    uint8_t type;
    uint8_t len;
};

void sensor_ext_clear_ev_markoff_status(void)
{
    is_1st_time_of_evmarkoff = true;
}

static inline bool is_kernelts_overflowed(const int64_t now_k_ts)
{
    return ((now_k_ts < kernel_basetime_ofs) ? true : false);
}

#define ADJUSTMENT_COEFF    (100)
#define SEC2NSEC_UNIT       (1000000000)
static int64_t convert_host_event_timestamp(int32_t ts, int32_t prv_ts,
                                            int64_t tn, int64_t prv_tn,
                                            int32_t bts)
{
    int64_t timestamp;
    int64_t ts_Cal = 0;
    int64_t ev_occur_time = 0;
    int64_t evtime_prv_to_cur_micon = 0;
    int64_t evtime_prv_to_cur_host = 0;

    SENSOR_D_LOG("start");
    SENSOR_D_LOG("ts: %d, prvts: %d, bts: %d, is_1st_time? : %s", ts, prv_ts,
                    bts, is_1st_time_of_evmarkoff? "YES":"NO");
    SENSOR_D_LOG("tn: %lld, prv_tn: %lld", tn, prv_tn);
    if(is_1st_time_of_evmarkoff){
        ts_Cal = 1;
        is_1st_time_of_evmarkoff = false;
        ev_occur_time = 3 * (ts * SEC2NSEC_UNIT * ts_Cal);
    } else {
        evtime_prv_to_cur_micon = ts - prv_ts;
        evtime_prv_to_cur_host = tn - prv_tn;
        ts_Cal = evtime_prv_to_cur_host * ADJUSTMENT_COEFF;
        do_div(ts_Cal, evtime_prv_to_cur_micon);
        SENSOR_D_LOG("ts_Cal = %lld, micontime[3sec]: %lld, kerneltime[ns]: %lld",
                        ts_Cal, evtime_prv_to_cur_micon, evtime_prv_to_cur_host);
        if(ts >= bts) {
            ev_occur_time = (ts - bts) * ts_Cal;
        } else {
            ev_occur_time = 0;
        }
        do_div(ev_occur_time, ADJUSTMENT_COEFF);
    }
    if(is_kernelts_overflowed(tn)){
        timestamp = ((S64_MAX - kernel_basetime_ofs) + tn) - ev_occur_time;
    } else {
        timestamp = (tn - kernel_basetime_ofs) - ev_occur_time;
    }
    SENSOR_D_LOG("timestamp = %lld", timestamp);
    SENSOR_D_LOG("end");
    return timestamp;
}



#define DAILYS_EVENT_HEADER_MAX (0x40)
#define DAILYS_EVENT_CLR_HEADER (0xF0u)
#define DAILYS_EVENT_CLR_LEN    (3u)
#define DAILYS_EVENT_EBUFFOF_HEADER (0xF1u)
#define DAILYS_EVENT_EBUFFOF_LEN    (3u)
#define DAILYS_EVENT_SWITCH_WALKRUN (0x02)
#define DAILYS_EVENT_WALK_END       (0x03)
const struct sensor_ext_dailys_event_info_t event_info[DAILYS_EVENT_HEADER_MAX] =
{
/*0x00*/    { 1, 5 },
/*0x01*/    { 1, 2 },
/*0x02*/    { 1, 13 },
/*0x03*/    { 1, 10 },
/*0x04*/    { 1, 9 },
/*0x05*/    { 0, 0 },
/*0x06*/    { 0, 0 },
/*0x07*/    { 0, 0 },
/*0x08*/    { 0, 0 },
/*0x09*/    { 0, 0 },
/*0x0A*/    { 0, 0 },
/*0x0B*/    { 0, 0 },
/*0x0C*/    { 0, 0 },
/*0x0D*/    { 0, 0 },
/*0x0E*/    { 0, 0 },
/*0x0F*/    { 0, 0 },
/*0x10*/    { 1, 6 },
/*0x11*/    { 0, 0 },
/*0x12*/    { 1, 4 },
/*0x13*/    { 1, 7 },
/*0x14*/    { 1, 9 },
/*0x15*/    { 0, 0 },
/*0x16*/    { 0, 0 },
/*0x17*/    { 0, 0 },
/*0x18*/    { 0, 0 },
/*0x19*/    { 0, 0 },
/*0x1A*/    { 1, 4 },
/*0x1B*/    { 1, 8 },
/*0x1C*/    { 0, 0 },
/*0x1D*/    { 0, 0 },
/*0x1E*/    { 0, 0 },
/*0x1F*/    { 0, 0 },
/*0x20*/    { 1, 5 },
/*0x21*/    { 1, 5 },
/*0x22*/    { 1, 5 },
/*0x23*/    { 0, 0 },
/*0x24*/    { 0, 0 },
/*0x25*/    { 0, 0 },
/*0x26*/    { 0, 0 },
/*0x27*/    { 0, 0 },
/*0x28*/    { 0, 0 },
/*0x29*/    { 0, 0 },
/*0x2A*/    { 0, 0 },
/*0x2B*/    { 0, 0 },
/*0x2C*/    { 0, 0 },
/*0x2D*/    { 0, 0 },
/*0x2E*/    { 0, 0 },
/*0x2F*/    { 0, 0 },
/*0x30*/    { 1, 3 },
/*0x31*/    { 1, 3 },
/*0x32*/    { 1, 3 },
/*0x33*/    { 1, 3 },
/*0x34*/    { 0, 0 },
/*0x35*/    { 0, 0 },
/*0x36*/    { 0, 0 },
/*0x37*/    { 0, 0 },
/*0x38*/    { 0, 0 },
/*0x39*/    { 0, 0 },
/*0x3A*/    { 0, 0 },
/*0x3B*/    { 0, 0 },
/*0x3C*/    { 0, 0 },
/*0x3D*/    { 0, 0 },
/*0x3E*/    { 0, 0 },
/*0x3F*/    { 0, 0 },
};

static void sensor_ext_backup_step_cal(const uint16_t header, void *buf)
{
    uint8_t step_l, step_h;
    uint8_t cal_l, cal_h;
    static uint16_t step = 0;
    static uint16_t cal = 0;
    uint8_t *eventdata = (uint8_t *)buf;

    SENSOR_D_LOG("start");
    if(header == DAILYS_EVENT_SWITCH_WALKRUN){
        step_l  = eventdata[9];
        step_h  = eventdata[10];
        cal_l   = eventdata[11];
        cal_h   = eventdata[12];
        SENSOR_D_LOG("header[0x%02x]:step_l[0x%02x] step_h[0x%02x] cal_l[0x%02x] cal_h[0x%02x]",
                    header, step_l, step_h, cal_l, cal_h);
        step    += (uint16_t)(step_h << 8) | ((uint16_t)step_l & 0x00FF);
        cal     += (uint16_t)( cal_h << 8) | ((uint16_t)cal_l  & 0x00FF);
        SENSOR_D_LOG("step[%d], cal[%d]", step, cal);
    }
    else if (header == DAILYS_EVENT_WALK_END){
        step_l  = eventdata[6];
        step_h  = eventdata[7];
        cal_l   = eventdata[8];
        cal_h   = eventdata[9];
        SENSOR_D_LOG("header[0x%02x]:step_l[0x%02x] step_h[0x%02x] cal_l[0x%02x] cal_h[0x%02x]",
                    header, step_l, step_h, cal_l, cal_h);
        step    += (uint16_t)(step_h << 8) | ((uint16_t)step_l & 0x00FF);
        cal     += (uint16_t)( cal_h << 8) | ((uint16_t)cal_l  & 0x00FF);
        SENSOR_D_LOG("step[%d], cal[%d]", step, cal);
        sensor_ext_update_backup_step_cal(step, cal);
        step = 0;
        cal = 0;
    }
    SENSOR_D_LOG("end");
}

void sensor_ext_input_report(int64_t clear_timestamp, void *event, uint32_t len)
{
    uint8_t *buf = (uint8_t *)event;
    uint8_t header;
    int64_t prv_ktime = 0;
    int32_t prv_miconbasetime = 0;
    int32_t ev_ts = 0;
    int64_t timestamp = 0;
    uint32_t timestamp_L = 0;
    uint32_t timestamp_H = 0;
    uint8_t exist_clr_ev = 0;
    int32_t miconts_bef_clr = 0;
    int32_t miconts_aft_clr = 0;
    int report_cnt = 0;
    const struct sensor_ext_dailys_event_info_t *info;

    SENSOR_N_LOG("start len=%d", len);

    prv_ktime = cur_ktime;
    prv_miconbasetime = cur_miconbasetime;
    sensor_get_event_markoff_timestamps(&cur_ktime, &cur_miconbasetime);
    if(len == 0){
        is_1st_time_of_evmarkoff = false;
        goto exit;
    }
    sensor_get_event_markoff_clrinfo(&exist_clr_ev, &miconts_bef_clr);
    if(exist_clr_ev){
        SENSOR_D_LOG("Reserved clear event.");
        miconts_aft_clr = cur_miconbasetime;
        cur_miconbasetime = miconts_bef_clr;
    }
    while (len) {
        header = *buf;
        info = &event_info[header];
        if (header < DAILYS_EVENT_HEADER_MAX && info->type != 0) {
            if (info->type == 1) {
                SENSOR_D_LOG("event header[%02x] len[%d]", header, info->len);
                SENSOR_D_LOG("data[0x%02x 0x%02x 0x%02x 0x%02x]", *(uint8_t*)buf, *(uint8_t*)(buf+1),*(uint8_t*)(buf+2),*(uint8_t*)(buf+3));
                input_report_abs(sensor_ext_input_info.dev, ABS_X, *(int32_t*)buf);
                if (info->len > 4) {
                    SENSOR_D_LOG("data[0x%02x 0x%02x 0x%02x 0x%02x]", *(uint8_t*)(buf+4), *(uint8_t*)(buf+5),*(uint8_t*)(buf+6),*(uint8_t*)(buf+7));
                    input_report_abs(sensor_ext_input_info.dev, ABS_Y, *(int32_t*)(buf+4));
                    if (info->len > 8) {
                    SENSOR_D_LOG("data[0x%02x 0x%02x 0x%02x 0x%02x]", *(uint8_t*)(buf+8), *(uint8_t*)(buf+9),*(uint8_t*)(buf+10),*(uint8_t*)(buf+11));
                        input_report_abs(sensor_ext_input_info.dev, ABS_Z, *(int32_t*)(buf+8));
                        if (info->len > 12) {
                            SENSOR_D_LOG("data[0x%02x 0x%02x 0x%02x 0x%02x]", *(uint8_t*)(buf+12), *(uint8_t*)(buf+13),*(uint8_t*)(buf+14),*(uint8_t*)(buf+15));
                            input_report_abs(sensor_ext_input_info.dev, ABS_RX, *(int32_t*)(buf+12));
                        }
                    }
                }
                SENSOR_D_LOG("current kernel time = %lld, current micontask time = %d",
                                cur_ktime, cur_miconbasetime);
                if(header != 0x01) {
                    ev_ts = ((int32_t)buf[1] & 0x000000FF) | (((int32_t)buf[2] & 0x000000FF) << 8);
                    timestamp = convert_host_event_timestamp(cur_miconbasetime, prv_miconbasetime,
                                                                cur_ktime, prv_ktime, ev_ts);
                    timestamp_L = (uint32_t)(timestamp & 0x00000000FFFFFFFF);
                    timestamp_H = (uint32_t)((timestamp & 0xFFFFFFFF00000000) >> 32);
                    SENSOR_D_LOG("event timestamp = %lld (Low:0x%08X , High:0x%08X)",
                                    timestamp, timestamp_L, timestamp_H);
                    input_report_abs(sensor_ext_input_info.dev, ABS_MISC+1, timestamp_L);
                    input_report_abs(sensor_ext_input_info.dev, ABS_MISC+2, timestamp_H);
                }
                sensor_ext_backup_step_cal(header, (void*)buf);
                input_sync(sensor_ext_input_info.dev);

                buf += info->len;
                len -= info->len;
                report_cnt++;
            } else {
                SENSOR_ERR_LOG("invalid event header[0x%02x] remain[%d]", header, len);
                break;
            }
        } else if (header == DAILYS_EVENT_CLR_HEADER) {
            SENSOR_D_LOG("event clear");
            sensor_report_abs_time(sensor_ext_input_info.dev, ~clear_timestamp);
            sensor_report_abs_time(sensor_ext_input_info.dev, clear_timestamp);
            kernel_basetime_ofs = clear_timestamp;
            cur_miconbasetime = miconts_aft_clr;
            sensor_ext_clr_backupofs_val();
            input_report_abs(sensor_ext_input_info.dev, ABS_MISC, 0);
            input_report_abs(sensor_ext_input_info.dev, ABS_MISC, 1);
            input_sync(sensor_ext_input_info.dev);

            buf += DAILYS_EVENT_CLR_LEN;
            len -= DAILYS_EVENT_CLR_LEN;
            report_cnt++;
        } else if (header == DAILYS_EVENT_EBUFFOF_HEADER) {
            SENSOR_D_LOG("event buffer overflow");
            //sensor_report_abs_time(sensor_ext_input_info.dev, ~clear_timestamp);
            //sensor_report_abs_time(sensor_ext_input_info.dev, clear_timestamp);
            //input_report_abs(sensor_ext_input_info.dev, ABS_MISC, 0);
            //input_report_abs(sensor_ext_input_info.dev, ABS_MISC, 1);
            //input_sync(sensor_ext_input_info.dev);

            buf += DAILYS_EVENT_EBUFFOF_LEN;
            len -= DAILYS_EVENT_EBUFFOF_LEN;
        } else {
            SENSOR_ERR_LOG("invalid event header[%d] remain[%d]", header, len);
            break;
        }
    }
exit:
    SENSOR_N_LOG("end report_cnt=%d", report_cnt);
}

void sensor_ext_driver_init(void)
{
    int32_t ret;

    SENSOR_N_LOG("start");

    atomic_set(&sns_ext_pedom_interrupt_kind,0);
    atomic_set(&sns_ext_pedom_interrupt_flag,0);
    atomic_set(&sns_ext_vehi_interrupt_kind,0);
    atomic_set(&sns_ext_vehi_interrupt_flag,0);
    atomic_set(&sns_ext_iwifi_interrupt_kind,0);
    atomic_set(&sns_ext_iwifi_interrupt_flag,0);
    mutex_init(&dailys_vib_mutex);

    init_waitqueue_head(&sensor_ext_pedom_p_queue);
    init_waitqueue_head(&sensor_ext_iwifi_p_queue);
    init_waitqueue_head(&sensor_ext_vehi_p_queue);

    INIT_DELAYED_WORK(&vib_work, sensor_ext_vib_interlocking_off);

    ret = sensor_input_init( &sensor_ext_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, sensor_ext_input_info.dev );

    if( (0 != ret) || (NULL == (sensor_ext_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        return;
    }

    ret = misc_register(&sensor_ext_pedom_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register():pedom-->ret[%d]",(int)ret);
        return;
    }

    ret = misc_register(&sensor_ext_vehi_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register():vehicle-->ret[%d]",(int)ret);
        return;
    }

    ret = misc_register(&sensor_ext_iwifi_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register():iwifi-->ret[%d]",(int)ret);
        return;
    }
    sensor_ext_initialize = 0x01;
    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(sensor_ext_driver_init);

