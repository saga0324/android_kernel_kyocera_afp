/* This program is free software; you can redistribute it and/or
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 */
#include "m12mo.h"

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm.h"
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <asm/div64.h>

#ifdef KC_BATTERY_LOG_ENABLED
#include <linux/clog.h>
#define KCBLOG(tag, ...)  CLOG(tag, ##__VA_ARGS__)
#else
#define KCBLOG(tag, ...) do{} while(0)
#endif

#define M12MO_SENSOR_NAME "m12mo"
#define PLATFORM_DRIVER_NAME "msm_camera_m12mo"
#define m12mo_obj m12mo_##obj

//#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif


DEFINE_MSM_MUTEX(m12mo_mut);
DEFINE_MUTEX(m12mo_power_lock);
DEFINE_MUTEX(m12mo_af_forcusing_lock);
DEFINE_MUTEX(m12mo_af_free_lock);
static int32_t g_m12mo_af_forcusing_flg = 0;
static int32_t g_m12mo_completion_flg = 0;
static struct msm_sensor_ctrl_t m12mo_s_ctrl;
struct completion g_m12mo_irq_wait;
static char m12mo_irqid[] = "CAM_M12MO_IRQ";

static int32_t g_m12mo_af_mode = FOCUS_MODE_MAX;
static int32_t g_m12mo_fd = 0;

static int32_t g_m12mo_stream_flg = -1;
static int32_t g_m12mo_pending_af_mode = -1;
static int32_t g_m12mo_pending_ev_bias = M12MO_I2C_VAL_AE_EV_BIAS_INVALID;

static int32_t g_m12mo_continuous_shoot_flg = 0;
static int32_t g_m12mo_power_flag = 0;
static m12mo_roi_holder_t g_m12mo_last_af_roi_info;
static m12mo_roi_holder_t g_m12mo_last_face_roi_info;
static m12mo_roi_holder_t g_m12mo_last_aec_roi_info;
static struct sensor_fps_info_t g_m12mo_last_fps_info;
static uint8_t g_m12mo_last_fps_category_data;
static uint8_t g_m12mo_auto_scene_status = 0;
static uint8_t g_m12mo_scene_manual_now = 0;
static int32_t g_m12mo_fd_detecting_flg = 0;

static int32_t g_m12mo_mon_size_width = 0;
static int32_t g_m12mo_mon_size_height = 0;
static int32_t g_m12mo_roi_offset_w = 0;
static int32_t g_m12mo_roi_offset_h = 0;
static int32_t g_m12mo_roi_rate_numer = 1;
static int32_t g_m12mo_roi_rate_denom = 1;
static uint8_t g_m12mo_last_ae_mode = 0xff;
static uint32_t *g_m12mo_zoom_table = NULL;
static int32_t g_m12mo_zoom_position = 0;
static int32_t g_m12mo_force_bining_flg = 0;
static uint8_t g_m12mo_last_cap_mode = M12MO_I2C_VAL_CAP_MODE_INVALID;
static uint8_t g_m12mo_kyocera_camera_flg = 0;
static uint8_t g_m12mo_last_mon_size = 0xff;

static int32_t g_m12mo_led_client = 0;
static int32_t g_m12mo_pending_set_led = CAMERA_LED_INVALID;


static struct msm_sensor_power_setting m12mo_power_setting[] = {
    { // VREG_L10
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VDIG,
        .config_val = 0,
        .delay = 1,
    },
    { // GPIO 23
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VDIG,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
    { // GPIO 01 (PMIC)
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VIO,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
    { // I2C SDA/SCL
        .seq_type = SENSOR_I2C_MUX,
        .seq_val = 0,
        .config_val = 0,
        .delay = 1,
    },
    { // GPIO 26
        .seq_type = SENSOR_CLK,
        .seq_val = SENSOR_CAM_MCLK,
        .config_val = 24000000,
        .delay = 3,
    },
    { // GPIO 35
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_RESET,
        .config_val = GPIO_OUT_HIGH,
        .delay = 11,
    },
};

static struct msm_sensor_power_setting m12mo_power_down_setting[] = {
    { // GPIO 35
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_RESET,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    { // GPIO 26
        .seq_type = SENSOR_CLK,
        .seq_val = SENSOR_CAM_MCLK,
        .config_val = 0,
        .delay = 1,
    },
    { // I2C SDA/SCL
        .seq_type = SENSOR_I2C_MUX,
        .seq_val = 0,
        .config_val = 0,
        .delay = 1,
    },
    { // GPIO 01 (PMIC)
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VIO,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    { // GPIO 23
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VDIG,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    { // VREG_L10
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VDIG,
        .config_val = 0,
        .delay = 0,
    },
};

static const m12mo_rect_value_t m12mo_jpeg_image_size[] = {
    /*----- sort this table to width order -----*/
    /* set,                            width, height, ratio (aspect ratio) */
    { M12MO_I2C_VAL_MAIN_SIZE_5M,       2592,   1944, m12mo_get_aspect_ratio( 4, 3) },
    { M12MO_I2C_VAL_MAIN_SIZE_8M,       3840,   2160, m12mo_get_aspect_ratio(16, 9) },
    { M12MO_I2C_VAL_MAIN_SIZE_13M,      4160,   3120, m12mo_get_aspect_ratio( 4, 3) },
};

static const m12mo_rect_value_t m12mo_thumb_image_size[] = {
    /*----- sort this table to width order -----*/
    /* set,                            width, height, ratio (aspect ratio) */
    { M12MO_I2C_VAL_THUMB_SIZE_QQVGA,    160,    120, m12mo_get_aspect_ratio( 4, 3) },
    { M12MO_I2C_VAL_THUMB_SIZE_QVGA,     320,    240, m12mo_get_aspect_ratio( 4, 3) },
    { M12MO_I2C_VAL_THUMB_SIZE_360x240,  360,    240, m12mo_get_aspect_ratio( 3, 2) },
    { M12MO_I2C_VAL_THUMB_SIZE_WQVGA,    400,    240, m12mo_get_aspect_ratio(16,10) },
    { M12MO_I2C_VAL_THUMB_SIZE_FWQVGA,   426,    240, m12mo_get_aspect_ratio(16, 9) },
    { M12MO_I2C_VAL_THUMB_SIZE_VGA,      640,    480, m12mo_get_aspect_ratio( 4, 3) },
    { M12MO_I2C_VAL_THUMB_SIZE_480P,     720,    480, m12mo_get_aspect_ratio( 3, 2) },
    { M12MO_I2C_VAL_THUMB_SIZE_800x480,  800,    480, m12mo_get_aspect_ratio(16,10) },
    { M12MO_I2C_VAL_THUMB_SIZE_FWVGA,    854,    480, m12mo_get_aspect_ratio(16, 9) },
    { M12MO_I2C_VAL_THUMB_SIZE_HHD,     1280,    720, m12mo_get_aspect_ratio(16, 9) },
    { M12MO_I2C_VAL_THUMB_SIZE_QuadVGA, 1280,    960, m12mo_get_aspect_ratio( 4, 3) },
};

#include "m12mo_fwup.inc"

extern void m12mo_fw_get_version(uint8_t *ver_hi, uint8_t *ver_lo);
extern int m12mo_spi_transfer(void);
extern int m12mo_fw_update(void* data);
static int32_t m12mo_set_focus_mode(struct msm_sensor_ctrl_t *s_ctrl, struct sensor_af_info_t *af_info);
static int32_t m12mo_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl, const int32_t ev);
static int32_t m12mo_lens_move_inf(struct msm_sensor_ctrl_t *s_ctrl);

static struct v4l2_subdev_info m12mo_subdev_info[] = {
    {
        .code   = V4L2_MBUS_FMT_UYVY8_2X8,
        .colorspace = V4L2_COLORSPACE_JPEG,
        .fmt    = 1,
        .order  = 0,
    },
};

static const struct i2c_device_id m12mo_i2c_id[] = {
    {M12MO_SENSOR_NAME, (kernel_ulong_t)&m12mo_s_ctrl},
    { }
};

static irqreturn_t m12mo_irq_handler(int irq, void *dev)
{
    CDBG("%s:E\n",__func__);

    complete(&g_m12mo_irq_wait);

    CDBG("%s:X\n",__func__);
    return IRQ_HANDLED;
}

static int32_t msm_m12mo_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int rc = 0;
    int irqno = client->irq;

    CDBG("%s:E irqno=%d\n", __func__, irqno);

    /* initialize completion object and register IRQ handler */
    init_completion(&g_m12mo_irq_wait);
    rc = request_threaded_irq(irqno, NULL, m12mo_irq_handler,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            client->dev.driver->name, m12mo_irqid);
    if (rc) {
        pr_err("%s:%d request_threaded_irq error. rc=%d\n",
            __func__, __LINE__, rc);
    }

    /* probe sensor device on I2C device */
    rc = msm_sensor_i2c_probe(client, id, &m12mo_s_ctrl);

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static struct i2c_driver m12mo_i2c_driver = {
    .id_table = m12mo_i2c_id,
    .probe  = msm_m12mo_i2c_probe,
    .driver = {
        .name = M12MO_SENSOR_NAME,
    },
};

static struct msm_camera_i2c_client m12mo_sensor_i2c_client = {
    .addr_type = MSM_CAMERA_I2C_4B_ADDR,
};

static const struct of_device_id m12mo_dt_match[] = {
    {.compatible = "qcom,m12mo", .data = &m12mo_s_ctrl},
    {}
};

MODULE_DEVICE_TABLE(of, m12mo_dt_match);

static struct platform_driver m12mo_platform_driver = {
    .driver = {
        .name = "qcom,m12mo",
        .owner = THIS_MODULE,
        .of_match_table = m12mo_dt_match,
    },
};


static int32_t m12mo_i2c_read_category_param(const struct msm_sensor_ctrl_t *s_ctrl,
    const uint8_t category, const uint8_t byte, uint8_t *data, const uint8_t size)
{
    int32_t rc = 0;
    struct msm_camera_i2c_client *sensor_i2c_client;
    const char *sensor_name;
    uint8_t read_data[M12MO_I2C_CATEGORY_READ_MAX];

    CDBG("%s:E Category=0x%02x, Byte=0x%02x\n", __func__, category, byte);

    /* check argument */
    if (!s_ctrl) {
        pr_err("%s:X %d failed: %p\n", __func__, __LINE__, s_ctrl);
        return -EINVAL;
    }
    if (!data) {
        pr_err("%s:X %d read buffer failed: %p\n", __func__, __LINE__, data);
        return -EINVAL;
    }
    if (size >= M12MO_I2C_CATEGORY_READ_MAX) {
        pr_err("%s:X %d read size failed: %d\n", __func__, __LINE__, size);
        return -EINVAL;
    }

    /* check i2c parameters */
    sensor_i2c_client = s_ctrl->sensor_i2c_client;
    sensor_name = s_ctrl->sensordata->sensor_name;
    if (!sensor_i2c_client || !sensor_name) {
        pr_err("%s:X %d failed: %p %p\n", __func__, __LINE__,
            sensor_i2c_client, sensor_name);
        return -EINVAL;
    }

    /* make i2c parameter request/read buffer */
    read_data[0] = M12MO_I2C_CMD_CATEGORY_READ_HEAD_LEN;
    read_data[1] = M12MO_I2C_CMD_CATEGORY_READ;
    read_data[2] = category;
    read_data[3] = byte;
    read_data[4] = size;

    /* send read request and receive raed data */
    rc = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
        sensor_i2c_client, M12MO_I2C_CMD_CATEGORY_READ_HEAD_LEN,
        read_data, size + M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN);
    if (rc < 0) {
        pr_err("%s:X %s: category parameter read failed. rc=%d\n",
            __func__, sensor_name, rc);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id,
            MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }

    /* set return value */
    if (read_data[0] == (int32_t)size + M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN) {
        rc = (int32_t)size;
    } else {
        rc = (int8_t)read_data[0];  /* error value is always negative */
        pr_err("%s: %s: category=0x%02x/byte=0x%02x error detected. code=0x%02x\n", __func__,
            sensor_name, category, byte, read_data[0]);
    }
    memcpy(data, &(read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN]), size);

    /* return value */
    CDBG("%s:X rc=%d. category=0x%02x/byte=0x%02x size=%d: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
        __func__, rc, category, byte, size,
        (size > 0 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 0] : 0x00),
        (size > 1 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 1] : 0x00),
        (size > 2 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 2] : 0x00),
        (size > 3 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 3] : 0x00),
        (size > 4 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 4] : 0x00),
        (size > 5 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 5] : 0x00),
        (size > 6 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 6] : 0x00),
        (size > 7 ? read_data[M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN + 7] : 0x00) );
    return rc;
}

static int32_t m12mo_i2c_write_category_param(const struct msm_sensor_ctrl_t *s_ctrl,
    const uint8_t category, const uint8_t byte, const uint8_t *data, const uint8_t size)
{
    int rc = 0;
    struct msm_camera_i2c_client *sensor_i2c_client;
    const char *sensor_name;
    struct msm_camera_i2c_seq_reg_array write_array;
    struct msm_camera_i2c_seq_reg_setting write_setting;

    CDBG("%s:E Category=0x%02x, Byte=0x%02x\n", __func__, category, byte);

    /* check argument */
    if (!s_ctrl) {
        pr_err("%s:%d failed: %p\n", __func__, __LINE__, s_ctrl);
        return -EINVAL;
    }
    if (!data) {
        pr_err("%s:%d write data failed: %p\n", __func__, __LINE__, data);
        return -EINVAL;
    }
    if (size >= M12MO_I2C_CATEGORY_WRITE_MAX) {
        pr_err("%s:%d write size failed: %d\n", __func__, __LINE__, size);
        return -EINVAL;
    }

    /* check i2c parameters */
    sensor_i2c_client = s_ctrl->sensor_i2c_client;
    sensor_name = s_ctrl->sensordata->sensor_name;
    if (!sensor_i2c_client || !sensor_name) {
        pr_err("%s:%d failed: %p %p\n", __func__, __LINE__,
            sensor_i2c_client, sensor_name);
        return -EINVAL;
    }

    /* write i2c parameter */
    write_array.reg_data[0] = size + M12MO_I2C_CMD_CATEGORY_WRITE_HEAD_LEN;
    write_array.reg_data[1] = M12MO_I2C_CMD_CATEGORY_WRITE;
    write_array.reg_data[2] = category;
    write_array.reg_data[3] = byte;
    memcpy( &(write_array.reg_data[M12MO_I2C_CMD_CATEGORY_WRITE_HEAD_LEN]),
        data, size );
    write_array.reg_data_size = write_array.reg_data[0];
    write_setting.reg_setting = &write_array;
    write_setting.size = 1;
    write_setting.addr_type = sensor_i2c_client->addr_type;
    write_setting.delay = 0;
    rc = sensor_i2c_client->i2c_func_tbl->i2c_write_seq_table(
        sensor_i2c_client, &write_setting);
    if (rc < 0) {
        pr_err("%s: %s: category parameter write failed. rc=%d\n", __func__,
            sensor_name, rc);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id,
            MSM_CAMERA_PRIV_I2C_ERROR);
    } else {
        rc = (int32_t)size; /* set return value to date size */
    }

    /* return value */
    CDBG("%s:X rc=%d. category=0x%02x/byte=0x%02x size=%d: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
        __func__, rc, category, byte, size,
        (size > 0 ? data[0] : 0x00), (size > 1 ? data[1] : 0x00),
        (size > 2 ? data[2] : 0x00), (size > 3 ? data[3] : 0x00),
        (size > 4 ? data[4] : 0x00), (size > 5 ? data[5] : 0x00),
        (size > 6 ? data[6] : 0x00), (size > 7 ? data[7] : 0x00) );
    return rc;
}

static int32_t m12mo_i2c_read_memory_byte(struct msm_sensor_ctrl_t *s_ctrl,
    const uint32_t addr, uint8_t *data, const int32_t size)
{
    int32_t rc;
    struct msm_camera_i2c_client *sensor_i2c_client = s_ctrl->sensor_i2c_client;
    const char *sensor_name = s_ctrl->sensordata->sensor_name;
    uint8_t read_data[size + M12MO_I2C_CMD_MEMORY_RECEIVE_HEAD_LEN];

    CDBG("%s:E Addr=0x%08x, Size=0x%02x\n", __func__, addr, size);

    /* check argument */
    if (!s_ctrl) {
        pr_err("%s:X %d failed: %p\n", __func__, __LINE__, s_ctrl);
        return -EINVAL;
    }
    if (!data) {
        pr_err("%s:X %d read buffer failed: %p\n", __func__, __LINE__, data);
        return -EINVAL;
    }
    if (size >= M12MO_I2C_MEMORY_DATA_MAX) {
        pr_err("%s:X %d read size failed: %d\n", __func__, __LINE__, size);
        return -EINVAL;
    }

    /* make i2c parameter request/read buffer */
    read_data[0] = 0x00;
    read_data[1] = M12MO_I2C_CMD_MEMORY_BYTE_READ;
    read_data[2] = (uint8_t)(addr >> 24);
    read_data[3] = (uint8_t)(addr >> 16);
    read_data[4] = (uint8_t)(addr >>  8);
    read_data[5] = (uint8_t)(addr >>  0);
    read_data[6] = (uint8_t)(size >> 8);
    read_data[7] = (uint8_t)(size >> 0);

    /* send read request and receive raed data */
    rc = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
        sensor_i2c_client, M12MO_I2C_CMD_MEMORY_HEAD_LEN,
        read_data, size + M12MO_I2C_CMD_MEMORY_RECEIVE_HEAD_LEN);
    if (rc < 0) {
        pr_err("%s:X %s: memory byte data read failed. rc=%d\n",
            __func__, sensor_name, rc);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id,
            MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }

    /* set return value */
    rc = ( ((int32_t)read_data[1] << 8) | read_data[2] );   /* get read data size */
    if (rc != size) {
        rc = (int8_t)read_data[0];  /* error value is always negative */
        pr_err("%s: %s: address=0x%08x size=%d error detected. code=0x%02x\n", __func__,
            sensor_name, addr, size, read_data[0]);
    }
    memcpy(data, &read_data[M12MO_I2C_CMD_MEMORY_RECEIVE_HEAD_LEN], size);

    /* return value */
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_i2c_wait_category_value(const struct msm_sensor_ctrl_t *s_ctrl,
    const uint8_t category, const uint8_t byte,
    const uint8_t value, const uint8_t mask, const uint32_t timevalue)
{
    int32_t rc = 0;
    long remain = 0;
    uint8_t int_factor;
    int32_t af_canceled;

    CDBG("%s:E Category=0x%02x, Byte=0x%02x, Value=0x%02x, Mask=0x%02x, Wait=%dms\n",
        __func__, category, byte, value, mask, timevalue);

    /* Wait for interrupt */
    remain = wait_for_completion_timeout(&g_m12mo_irq_wait, msecs_to_jiffies(timevalue));

    M12MO_LOG_MUTEX_LOCK(m12mo_af_forcusing_lock);
    af_canceled = g_m12mo_completion_flg;
    g_m12mo_af_forcusing_flg = 0;
    g_m12mo_completion_flg = 0;
    M12MO_LOG_MUTEX_UNLOCK(m12mo_af_forcusing_lock);

    INIT_COMPLETION(g_m12mo_irq_wait);
    if (remain < 0) {
        rc = (int32_t)remain;
        pr_err("%s:X wait completion call error. rc=%d\n", __func__, rc);
    } else {
        if(remain == 0) {
            /* keep going if timer expired */
            pr_warn("%s: Category 0x%02x: Byte 0x%02x wait %dms timed out. ",
                __func__, category, byte, timevalue);
        }

        /* read category value */
        rc = m12mo_i2c_read_category_param(s_ctrl, category, byte, &int_factor, sizeof(int_factor));
        if (rc != sizeof(int_factor)) {
            pr_err("%s:X Category 0x%02x: Byte 0x%02x wait read failed. "
                "rc=%d, wait=%dms, value=0x%02x:0x%02x\n", __func__,
                category, byte, rc, timevalue, value, int_factor);
        } else {
            if ((int_factor & mask) == value) {
                CDBG("%s:X Category 0x%02x: Byte 0x%02x wait success. "
                    "rc=%d, wait=%dms, value=0x%02x:0x%02x\n", __func__,
                    category, byte, rc, timevalue, value, int_factor);
                /* return success */
                rc = 0;
            } else if (af_canceled) {
                CDBG("%s:X Category 0x%02x: Byte 0x%02x wait canceled. "
                    "rc=%d, wait=%dms, value=0x%02x:0x%02x\n", __func__,
                    category, byte, rc, timevalue, value, int_factor);
                /* return success */
                rc = 0;
            } else {
                pr_err("%s:X Category 0x%02x: Byte 0x%02x wait value failed. "
                    "rc=%d, wait=%dms, value=0x%02x:0x%02x\n", __func__,
                    category, byte, rc, timevalue, value, int_factor);
            }
        }
        /* change return value if rc is received data size */
        if (rc > 0) {
           rc = -EBADE;    /* Invalid exchange */
        }
    }
    return rc;
}

static int32_t m12mo_i2c_wait_int_factor(const struct msm_sensor_ctrl_t *s_ctrl,
    const uint8_t value, const uint32_t mwait)
{
    int32_t rc;
    uint8_t val_mask;

    CDBG("%s:E value=0x%02x\n", __func__, value);

    /* Create mask value */
    val_mask = (value == M12MO_I2C_VAL_INT_INITIALIZED ? M12MO_I2C_VAL_INT_MASK_ALL : value);

    /* Call IRQ wait */
    rc = m12mo_i2c_wait_category_value(s_ctrl,
        M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_INT_FACTOR,
        value, val_mask, mwait);

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_i2c_wait_fw_int_factor(const struct msm_sensor_ctrl_t *s_ctrl,
    const uint8_t value, const uint32_t mwait)
{
    int32_t rc;
    uint8_t val_mask;

    CDBG("%s:E value=0x%02x\n", __func__, value);

    /* Create mask value */
    val_mask = M12MO_I2C_VAL_INT_MASK_ALL;

    /* Call IRQ wait */
    rc = m12mo_i2c_wait_category_value(s_ctrl,
        M12MO_I2C_CAT_INI, M12MO_I2C_CAT_INI_INT_FACTOR,
        value, val_mask, mwait);

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_restart_isp(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc;
    struct msm_camera_i2c_client *sensor_i2c_client = s_ctrl->sensor_i2c_client;
    struct msm_camera_power_ctrl_t *power_info = &s_ctrl->sensordata->power_info;

    if (power_info->cam_pinctrl_status) {
        CDBG("%s:E Power is On\n", __func__);

        msm_camera_power_down(power_info,
            s_ctrl->sensor_device_type, sensor_i2c_client);
        msleep(20);
    } else {
        CDBG("%s:E Power is already Off\n", __func__);
    }
    rc = msm_camera_power_up(power_info, s_ctrl->sensor_device_type,
        sensor_i2c_client);
    if (rc < 0){
        pr_err("%s:X %d power up failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_start_camera_fw(struct msm_sensor_ctrl_t *s_ctrl, int *is_fw_empty )
{
    int rc = 0;
    const uint8_t start_fw = M12MO_I2C_VAL_START_CAMERA_FW;
    uint8_t hw_check;

    CDBG("%s:E\n",__func__);

    /* wait power on status [ms] */
    rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_INITIALIZED, 2000);
    if (rc < 0){
        pr_err("%s:X power on wait %dms irq failed. rc=%d\n", __func__, 2000, rc);
        return rc;
    }

    /* start camera firmware */
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_INI, M12MO_I2C_CAT_INI_START, &start_fw, sizeof(start_fw));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    /* wait firmware download status [ms] */
    rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_MODE, 1000);
    if (rc < 0) {
        pr_err("%s:X start camera firmware wait %dms irq failed. (may be FW is empty) rc=%d\n",
            __func__, 1000, rc);
        if (is_fw_empty != NULL) {
            *is_fw_empty = 1;   /* set firmware empty flag */
        }
        return rc;
    }

    /* check camera module i2c connection */
    rc = m12mo_i2c_read_category_param(s_ctrl,
        M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_HW_I2C_CHECK, &hw_check, sizeof(hw_check));
    if (rc != sizeof(hw_check)) {
        pr_err("%s:X hardware i2c check read failed. rc=%d\n", __func__, rc);
        return rc;
    } else {
        rc = 0; /* return success */
    }
    if (hw_check & M12MO_I2C_VAL_HW_CHK_MASK) {
        /* out put log only, not return failure. */
        pr_err("%s: some device connection failed. HW_I2C_CHECK=0x%02x\n", __func__, hw_check);
    } else {
        CDBG("%s: all device connection success. HW_I2C_CHECK=0x%02x\n", __func__, hw_check);
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_fwdl(struct msm_sensor_ctrl_t *s_ctrl, int erase)
{
    int32_t rc;
    int i;
    struct msm_camera_i2c_client *sensor_i2c_client = s_ctrl->sensor_i2c_client;

    const uint16_t mem_w8_size = sizeof(updater);
    const uint16_t mem_w8_unit = I2C_SEQ_REG_DATA_MAX - M12MO_I2C_CMD_MEMORY_HEAD_LEN;
    const int mem_w8_num = ( mem_w8_size + (mem_w8_unit - 1) ) / mem_w8_unit;
    struct msm_camera_i2c_seq_reg_array *mem_w8_arrays;
    struct msm_camera_i2c_seq_reg_setting mem_w8_setting;
    uint32_t mem_w8_addr;
    uint16_t mem_w8_leave;
    uint16_t mem_w8_len;
    const uint8_t *mem_w8_data;

    const uint8_t start_addr = M12MO_I2C_VAL_START_FROM_ADDR;
    const int checksum_len = 2;
    uint8_t checksum[checksum_len];


    CDBG("%s:E\n", __func__);

    /* ISP Restart (Power Off -> On) */
    rc = m12mo_restart_isp(s_ctrl);
    if (rc < 0){
        pr_err("%s:X %d failed\n", __func__, __LINE__);
        return rc;
    }
    CDBG("%s: Before FWDL ISP Reset success.\n", __func__);

    /* wait Boot completion interrupt [ms] */
    rc = m12mo_i2c_wait_fw_int_factor(s_ctrl, M12MO_I2C_IRQ_BOOT_COMPLETE, 1000);
    if (rc < 0) {
        pr_err("%s:X Boot completion interrupt wait %dms irq failed. rc=%d\n",
            __func__, 1000, rc);
        return rc;
    }
    CDBG("%s: Boot completion interrupt success.\n", __func__);


    /* Load the 'FLASH ROM UPDATER for SIO' */
    /* Updater divide to msm_camera_i2c_seq_reg_array limit */
    mem_w8_arrays = kzalloc(mem_w8_num * sizeof(struct msm_camera_i2c_seq_reg_array),
        GFP_KERNEL);
    if (!mem_w8_arrays) {
        pr_err("%s:X %d failed\n", __func__, __LINE__);
        return -ENOMEM;
    }
    /* Initialize divide values */
    mem_w8_data = updater;
    mem_w8_addr = M12MO_I2C_VAL_START_UPDATER_ADDR;
    mem_w8_leave = mem_w8_size;

    for (i = 0; i < mem_w8_num; i++) {
        /* Calculate send length from data of leave */
        mem_w8_len = (mem_w8_leave >= mem_w8_unit ? mem_w8_unit : mem_w8_leave);

        /* Set data to array */
        mem_w8_arrays[i].reg_data[0] = 0;
        mem_w8_arrays[i].reg_data[1] = M12MO_I2C_CMD_MEMORY_BYTE_WRITE;
        mem_w8_arrays[i].reg_data[2] = (uint8_t)(mem_w8_addr >> 24);
        mem_w8_arrays[i].reg_data[3] = (uint8_t)(mem_w8_addr >> 16);
        mem_w8_arrays[i].reg_data[4] = (uint8_t)(mem_w8_addr >>  8);
        mem_w8_arrays[i].reg_data[5] = (uint8_t)(mem_w8_addr >>  0);
        mem_w8_arrays[i].reg_data[6] = (uint8_t)(mem_w8_len  >>  8);
        mem_w8_arrays[i].reg_data[7] = (uint8_t)(mem_w8_len  >>  0);
        memcpy( &(mem_w8_arrays[i].reg_data[M12MO_I2C_CMD_MEMORY_HEAD_LEN]),
            mem_w8_data, mem_w8_len );
        mem_w8_arrays[i].reg_data_size = M12MO_I2C_CMD_MEMORY_HEAD_LEN + mem_w8_len;
        CDBG("%s: Updater: array[%d].reg_data= 0x%02x 0x%02x 0x%02x%02x%02x%02x 0x%02x%02x 0x%02x...\n",
            __func__, i,
            mem_w8_arrays[i].reg_data[0], mem_w8_arrays[i].reg_data[1], mem_w8_arrays[i].reg_data[2],
            mem_w8_arrays[i].reg_data[3], mem_w8_arrays[i].reg_data[4], mem_w8_arrays[i].reg_data[5],
            mem_w8_arrays[i].reg_data[6], mem_w8_arrays[i].reg_data[7], mem_w8_arrays[i].reg_data[8] );

        /* Update divide values */
        mem_w8_data += mem_w8_len;
        mem_w8_addr += mem_w8_len;
        mem_w8_leave -= mem_w8_len;
    }
    /* set i2c write data */
    mem_w8_setting.reg_setting = mem_w8_arrays;
    mem_w8_setting.size = mem_w8_num;
    mem_w8_setting.addr_type = sensor_i2c_client->addr_type;
    mem_w8_setting.delay = 0;
    /* i2c command: memory write 8bit address */
    rc = sensor_i2c_client->i2c_func_tbl->i2c_write_seq_table(
        sensor_i2c_client, &mem_w8_setting);
    if (rc < 0) {
        pr_err("%s: firmware updater write failed. rc=%d\n", __func__, rc);
        kfree(mem_w8_arrays);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id,
            MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
    CDBG("%s: firmware updater write success.\n", __func__);

    /* Set the PC jump address */
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_INI, M12MO_I2C_CAT_INI_SET_START_ADDRESS,
        &mem_w8_arrays[0].reg_data[2], 4);  /* Jump to Updater start address */
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        kfree(mem_w8_arrays);
        return rc;
    }
    CDBG("%s: Set the PC jump address write success.\n", __func__);

    /* free Updater divide arrays */
    kfree(mem_w8_arrays);


    /* Jump the 'UPDATER' address to ERASE CURRENT FIRMWARE */
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_INI, M12MO_I2C_CAT_INI_START,
        &start_addr, sizeof(start_addr));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    CDBG("%s: Jump the 'UPDATER' address write success.\n", __func__);

    /* wait ready for firmware download interrupt [ms] */
    rc = m12mo_i2c_wait_fw_int_factor(s_ctrl, M12MO_I2C_IRQ_RAM_ACTIVATE, 10000);
    if (rc < 0) {
        pr_err("%s:X ready for firmware download interrupt wait %dms irq failed. rc=%d\n",
            __func__, 10000, rc);
        return rc;
    }
    CDBG("%s: ready for firmware download interrupt success.\n", __func__);


    /* return success if firmware erase requested. */
    if (erase) {
        pr_info("%s: **** CLEARE FIRMWARE SUCCEED - NOW FIRMWARE IS EMPTY ****\n",
            __func__);
        return rc;
    }


    /* Send compressed firmware by SIO transmission. */
    rc = m12mo_spi_transfer();
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    CDBG("%s: Send compressed firmware by SIO transmission success.\n", __func__);


    /* wait firmware update completion interrupt (10.0s) */
    rc = m12mo_i2c_wait_fw_int_factor(s_ctrl, M12MO_I2C_IRQ_FLASHROM_UPDATE, 10000);
    if (rc < 0) {
        pr_err("%s:X firmware update completion interrupt wait %dms irq failed. rc=%d\n",
            __func__, 10000, rc);
        return rc;
    }
    CDBG("%s: firmware update completion interrupt success.\n", __func__);

    /* check firmware checksum */
    rc = m12mo_i2c_read_category_param(s_ctrl,
        M12MO_I2C_CAT_INI, M12MO_I2C_CAT_INI_GET_CHECK_SUM, checksum, checksum_len);
    if (rc != checksum_len) {
        pr_err("%s:X firmware update check sum read failed. rc=%d\n", __func__, rc);
        return rc;
    }
    if (checksum[0] != 0x00 || checksum[1] != 0x00) {
        pr_err("%s:X checksum does not correct. 0x%02x%02x\n", __func__,
            checksum[0], checksum[1]);
        return -EBADE;  /* Invalid exchange */
    }
    CDBG("%s: checksum is correct. 0x%02x%02x\n", __func__,
            checksum[0], checksum[1]);


    /* ISP Restart (Power Off -> On) */
    rc = m12mo_restart_isp(s_ctrl);
    if (rc < 0){
        pr_err("%s:X %d failed\n", __func__, __LINE__);
        return rc;
    }
    CDBG("%s: After FWDL ISP Reset success.\n", __func__);


    /* firmware update completion */
    pr_info("%s:X firmware update completion. rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_power_up_init(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    uint8_t int_enable;
    int zoom_size = M12MO_ZOOM_MAX_LEVEL - M12MO_ZOOM_MIN_LEVEL + 1;
    int zoom_copy = cdata->cfg.power_up.zoom_table.size;
    int i;

    CDBG("%s:E\n",__func__);

    g_m12mo_kyocera_camera_flg = cdata->cfg.power_up.kyocera_camera_flg;

    g_m12mo_stream_flg = -1;
    g_m12mo_pending_af_mode = -1;
    g_m12mo_pending_ev_bias = M12MO_I2C_VAL_AE_EV_BIAS_INVALID;

    g_m12mo_mon_size_width = 0;
    g_m12mo_mon_size_height = 0;
    g_m12mo_last_cap_mode = M12MO_I2C_VAL_CAP_MODE_INVALID;

    g_m12mo_force_bining_flg = (g_m12mo_kyocera_camera_flg ? 0 : 1);

    memset(&g_m12mo_last_fps_info, 0x00, sizeof(struct sensor_fps_info_t));
    g_m12mo_last_fps_category_data = 0xff;

    g_m12mo_af_mode = FOCUS_MODE_MAX;
    g_m12mo_af_forcusing_flg = 0;

    g_m12mo_fd = 0;
    g_m12mo_fd_detecting_flg = 0;

    g_m12mo_last_af_roi_info.is_valid = 0;
    g_m12mo_last_face_roi_info.is_valid = 0;
    g_m12mo_last_aec_roi_info.is_valid = 0;

    g_m12mo_last_ae_mode = 0xff;

    g_m12mo_zoom_position = 0;
    if (zoom_size < zoom_copy) {
        zoom_copy = zoom_size;
    }
    if(g_m12mo_zoom_table){
        kfree(g_m12mo_zoom_table);
        g_m12mo_zoom_table = NULL;
    }
    g_m12mo_zoom_table = kmalloc(sizeof(uint32_t) * zoom_size, GFP_KERNEL);
    if (!g_m12mo_zoom_table) {
        pr_err("%s: failed to alloc mem\n", __func__);
        rc = -ENOMEM;
        return rc;
    }
    if (copy_from_user(g_m12mo_zoom_table, (void *)cdata->cfg.power_up.zoom_table.data,
        sizeof(uint32_t) * zoom_copy)) {
        pr_err("%s:X %d copy_from_user failed\n", __func__, __LINE__);
        kfree(g_m12mo_zoom_table);
        g_m12mo_zoom_table = NULL;
        return -EFAULT;
    }
    for (i = zoom_copy; i < zoom_size; i++) {
        g_m12mo_zoom_table[i] = g_m12mo_zoom_table[zoom_copy - 1];
    }

    g_m12mo_auto_scene_status = MSM_CAMERA_SCENE_MODE_OFF;
    g_m12mo_scene_manual_now = M12MO_I2C_VAL_ATSCENE_MAN_AUTO;

    g_m12mo_led_client = 0;
    g_m12mo_pending_set_led = CAMERA_LED_INVALID;

    s_ctrl->camera_stream_type = MSM_CAMERA_STREAM_INVALID;
    g_m12mo_last_mon_size = 0xff;

    int_enable=M12MO_I2C_VAL_INT_MASK_ALL & ~(M12MO_I2C_VAL_INT_STATUS_CAF | M12MO_I2C_VAL_INT_STATUS_ZOOM);
    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_INT_ENABLE, &int_enable, sizeof(int_enable));
    if(rc < 0){
        pr_err("%s:X %d failed.INT_ENABLE=0x%02x\n", __func__, __LINE__, int_enable);
        return rc;
    }else{
        rc = 0;
    }

    CDBG("%s:X INT_ENABLE=0x%02x\n",__func__,int_enable);
    return rc;
}

static int32_t m12mo_power_up_deinit(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;

    CDBG("%s:E\n",__func__);
    m12mo_lens_move_inf(s_ctrl);
    if(g_m12mo_zoom_table) {
        kfree(g_m12mo_zoom_table);
        g_m12mo_zoom_table = NULL;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static const m12mo_rect_value_t* m12mo_find_image_value(
    const uint16_t width, const uint16_t height, const m12mo_rect_value_t *tbl, const int size)
{
    int ratio = m12mo_get_aspect_ratio(width, height);
    int rd_tmp, wd_tmp;
    int r_diff = INT_MAX;   /* aspect ratio differential */
    int w_diff = INT_MAX;   /* width differential */
    const m12mo_rect_value_t *p;
    const m12mo_rect_value_t *cp = tbl;
    const m12mo_rect_value_t *ep = tbl + size;
    do {
        /* calculate differential: aspect ratio, width */
        rd_tmp = cp->ratio - ratio;
        rd_tmp = (rd_tmp < 0 ? -rd_tmp : rd_tmp);
        wd_tmp = cp->width - width;
        wd_tmp = (wd_tmp < 0 ? -wd_tmp : wd_tmp);
        /* reset hold values when differental is less equal */
        if (rd_tmp <= r_diff) {
            r_diff = rd_tmp;
            if (wd_tmp <= w_diff) {
                w_diff = wd_tmp;
                p = cp;
            }
        }
        /* break find same value or width differendial greater equal width */
        if ((r_diff == 0 && w_diff == 0) || (wd_tmp >= width)) {
            break;
        }
    } while (cp++ != ep);

    return p;
}

static int32_t m12mo_set_raw_image_size(struct msm_sensor_ctrl_t *s_ctrl, const m12mo_rect_value_t *rect_value) {
    int32_t rc = 0;
    uint32_t yuv_length;
    uint32_t jpeg_size_max;
    uint8_t jpeg_size_max_array[4];

    CDBG("%s:E set=0x%02x width=%d height=%d ratio=%d\n",__func__,
         rect_value->set, rect_value->width, rect_value->height, rect_value->ratio);

    /* set yuv thumbnail to preview size (for JPEG RAW snapshot) */
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_THUMB_IMAGE_SIZE,
        &(rect_value->set), sizeof(rect_value->set));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    /* caluculate round up JPEG_SIZE_MAX (YUV420 is 12bit/pixel) */
    yuv_length = (rect_value->width * rect_value->height * 12 + 7) / 8;
    jpeg_size_max = M12MO_JPEG_RAW_SIZE_MAX - yuv_length;
    /* set JPEG_SIZE_MAX (for JPEG RAW snapshot) */
    jpeg_size_max_array[0] = (uint8_t)(jpeg_size_max >> 24);
    jpeg_size_max_array[1] = (uint8_t)(jpeg_size_max >> 16);
    jpeg_size_max_array[2] = (uint8_t)(jpeg_size_max >>  8);
    jpeg_size_max_array[3] = (uint8_t)(jpeg_size_max >>  0);
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_JPEG_SIZE_MAX,
        jpeg_size_max_array, sizeof(jpeg_size_max_array));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d value=%d : 0x%02x%02x%02x%02x\n",
            __func__, __LINE__, rc, jpeg_size_max,
            jpeg_size_max_array[0], jpeg_size_max_array[1],
            jpeg_size_max_array[2], jpeg_size_max_array[3]);
        return rc;
    } else {
        rc = 0; /* return success */
    }
    CDBG("%s:X rc=%d total=%d, jpeg size max=%d : 0x%02x%02x%02x%02x\n", __func__, rc,
        yuv_length + jpeg_size_max, jpeg_size_max,
        jpeg_size_max_array[0], jpeg_size_max_array[1],
        jpeg_size_max_array[2], jpeg_size_max_array[3]);
    return rc;
}

static int32_t m12mo_set_resolution(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    struct sensor_res_info_t *res_val;

    CDBG("%s:E \n",__func__);

    if (!s_ctrl || !cdata) {
        pr_err("%s:X error argument. s_ctrl=0x%p, cdata=0x%p\n", __func__,
            s_ctrl, cdata);
        return -EINVAL;
    }
    res_val = &(cdata->cfg.res_info);
    CDBG("%s: res=%d, stream_mask=0x%04x, %dx%d\n", __func__,
        res_val->res, res_val->stream_mask, res_val->width, res_val->height);

    switch (res_val->res) {
    case 0:
    { /* jpeg capture resolution */
        /* find nearly jpeg size value */
        const m12mo_rect_value_t *p = m12mo_find_image_value(res_val->width, res_val->height,
            m12mo_jpeg_image_size, ARRAY_SIZE(m12mo_jpeg_image_size));
        CDBG("%s: resolution=%4dx%4d(%d), jpeg=0x%02x:%4dx%4d(%d)\n", __func__,
            res_val->width, res_val->height, m12mo_get_aspect_ratio(res_val->width, res_val->height),
            p->set, p->width, p->height, p->ratio);

        /* set main image size */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_MAIN_IMAGE_SIZE,
            &p->set, sizeof(p->set));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            rc = 0; /* return success */
        }

        if (!g_m12mo_kyocera_camera_flg) {
            const m12mo_rect_value_t *rect_value;
            if (p->ratio == m12mo_get_aspect_ratio( 4, 3)) {
                rect_value = m12mo_find_image_value(640, 480, m12mo_thumb_image_size,
                                   sizeof(m12mo_thumb_image_size)/sizeof(m12mo_rect_value_t));
            } else {
                rect_value = m12mo_find_image_value(854, 480, m12mo_thumb_image_size,
                                   sizeof(m12mo_thumb_image_size)/sizeof(m12mo_rect_value_t));
            }
            CDBG("%s: for 3rd camera\n", __func__);
            CDBG("%s: jpeg size=%4dx%4d(%d), yuv thumb=0x%02x:%4dx%4d(%d)\n", __func__,
                res_val->width, res_val->height, m12mo_get_aspect_ratio(res_val->width, res_val->height),
                rect_value->set, rect_value->width, rect_value->height, rect_value->ratio);

            rc = m12mo_set_raw_image_size(s_ctrl, rect_value);
        }
    }
        break;

    case 1:
    case 2:
    case 3:
    case 4:
    { /* preview resolution */
        const uint8_t capture_yuv = M12MO_I2C_VAL_MAIN_SIZE_3_25M;
        int32_t mon_width;
        int32_t mon_height;

        /* set main image size for snapshot */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_MAIN_IMAGE_SIZE,
            &capture_yuv, sizeof(capture_yuv));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            rc = 0; /* return success */
        }

        /* set thumbnail size for jpeg raw snapshot */
        if ( (s_ctrl->camera_stream_type == MSM_CAMERA_STREAM_PREVIEW)               /* Preview */
            || (s_ctrl->camera_stream_type == MSM_CAMERA_STREAM_PREVIEW_SNAPSHOT)    /* ZSL */
        ) {
            if(g_m12mo_kyocera_camera_flg) {
                /* find nearly yuv thumbnail size value */
                const m12mo_rect_value_t *p = m12mo_find_image_value(res_val->width, res_val->height,
                    m12mo_thumb_image_size, sizeof(m12mo_thumb_image_size)/sizeof(m12mo_rect_value_t));
                CDBG("%s: for kyocera camera\n", __func__);
                CDBG("%s: preview=%4dx%4d(%d), yuv thumb=0x%02x:%4dx%4d(%d)\n", __func__,
                    res_val->width, res_val->height, m12mo_get_aspect_ratio(res_val->width, res_val->height),
                    p->set, p->width, p->height, p->ratio);

                rc = m12mo_set_raw_image_size(s_ctrl, p);
            }
        }

        /* calculate roi parameters */
        switch (res_val->res)
        {
        case 1: /*  4:3 */
            mon_width = M12MO_MON_SIZE_3_25M_WIDTH;
            mon_height = M12MO_MON_SIZE_3_25M_HEIGHT;
            break;
        case 2: /* 16:9 */
            mon_width = M12MO_MON_SIZE_2_5M_WIDTH;
            mon_height = M12MO_MON_SIZE_2_5M_HEIGHT;
            break;
        case 3: /* 16:9 */
            mon_width = M12MO_MON_SIZE_1_03M_WIDTH;
            mon_height = M12MO_MON_SIZE_1_03M_HEIGHT;
            break;
        case 4: /*  4:3 */
            mon_width = M12MO_MON_SIZE_SVGA_WIDTH;
            mon_height = M12MO_MON_SIZE_SVGA_HEIGHT;
            break;
        default:
            pr_err("%s: %d failed. res=%d\n", __func__, __LINE__,
                res_val->res);
            mon_width = g_m12mo_mon_size_width;
            mon_height = g_m12mo_mon_size_height;
            break;
        }
        if (g_m12mo_mon_size_width != mon_width || g_m12mo_mon_size_height != mon_height) {
            int32_t isp_ratio, prev_ratio;

            g_m12mo_mon_size_width = mon_width;
            g_m12mo_mon_size_height = mon_height;

            isp_ratio = m12mo_get_aspect_ratio(g_m12mo_mon_size_width, g_m12mo_mon_size_height);
            prev_ratio = m12mo_get_aspect_ratio(res_val->width, res_val->height);
            if(isp_ratio == prev_ratio) {       // w:full h:full
                g_m12mo_roi_rate_numer = g_m12mo_mon_size_width;
                g_m12mo_roi_rate_denom = res_val->width;
                g_m12mo_roi_offset_w = 0;
                g_m12mo_roi_offset_h = 0;
            } else if(isp_ratio < prev_ratio) { // w:full h:clop
                g_m12mo_roi_rate_numer = g_m12mo_mon_size_width;
                g_m12mo_roi_rate_denom = res_val->width;
                g_m12mo_roi_offset_w = 0;
                g_m12mo_roi_offset_h = (g_m12mo_mon_size_height -
                                       res_val->height * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom) / 2;
            } else {                           // w:clop h:full
                g_m12mo_roi_rate_numer = g_m12mo_mon_size_height;
                g_m12mo_roi_rate_denom = res_val->height;
                g_m12mo_roi_offset_w = (g_m12mo_mon_size_width -
                                       res_val->width * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom) / 2;
                g_m12mo_roi_offset_h = 0;
            }
        }
    }
        break;

    default: {
        pr_err("%s:X %d failed. res=%d\n", __func__, __LINE__, res_val->res);
    }
        break;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;

    CDBG("%s:E stream_type=%d\n", __func__, s_ctrl->camera_stream_type);

    if(g_m12mo_stream_flg == 1){
        CDBG("%s:X. nop\n", __func__);
        return 0;
    }

    /* 1st process only */
    if (g_m12mo_stream_flg < 0) {
        const uint8_t jpeg_num = M12MO_JPEG_RAW_TRANSFER_NUM;
        const uint8_t thumb_yuv = M12MO_I2C_VAL_YUVOUT_THUMB_NV21;
        const uint8_t start_stream = M12MO_I2C_VAL_MODE_MONITOR;

        /* set jpeg transfer num */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_CAP_TRANSFER_NUM,
            &jpeg_num, sizeof(jpeg_num));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }

        /* set jpeg raw yuv thumbnail */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_YUVOUT_THUMB,
            &thumb_yuv, sizeof(thumb_yuv));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }

        /* set start streaming (start monitor mode) */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_SYS_MODE,
            &start_stream, sizeof(start_stream));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
        /* wait start streaming status [ms] */
        rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_MODE, 500);
        if (rc < 0) {
            pr_err("%s:X start streaming wait %dms irq failed. rc=%d\n", __func__, 500, rc);
            return rc;
        }

        g_m12mo_stream_flg = 1;

        if(g_m12mo_pending_af_mode >= 0) {
            struct sensor_af_info_t af_info;
            af_info.mode = g_m12mo_pending_af_mode;
            rc = m12mo_set_focus_mode(s_ctrl, &af_info);
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            } else {
                g_m12mo_pending_af_mode = -1;
            }
        }

        if (g_m12mo_pending_ev_bias != M12MO_I2C_VAL_AE_EV_BIAS_INVALID) {
            if (g_m12mo_scene_manual_now != M12MO_I2C_VAL_ATSCENE_MAN_AUTO) {
                pr_warn("%s: scene is not AUTO. pending exposure setting. scene manual=%d, ev=%d\n",
                    __func__, g_m12mo_scene_manual_now, g_m12mo_pending_ev_bias);
            } else {
                rc = m12mo_set_exposure_compensation(s_ctrl, g_m12mo_pending_ev_bias);
                if (rc < 0) {
                    pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                    return rc;
                } else {
                    g_m12mo_pending_ev_bias = M12MO_I2C_VAL_AE_EV_BIAS_INVALID;
                }
            }
        }

        CDBG("%s:X rc=%d\n", __func__, rc);
        return rc;
    }

    switch (s_ctrl->camera_stream_type) {
    case MSM_CAMERA_STREAM_SNAPSHOT: {
        const uint8_t single_shot = M12MO_I2C_VAL_START_SINGLE_SHOT;
        const uint8_t start_tx = M12MO_I2C_VAL_TX_MAIN_IMAGE;

        /* set single shot for stop stream */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_START_CAP,
            &single_shot, sizeof(single_shot));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
        /* wait shutter sound interrupt [ms] */
        rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_SOUND, 2000);
        if (rc < 0) {
            pr_err("%s:X snapshot shutter sound interrupt wait %dms irq failed. rc=%d\n",
                __func__, 2000, rc);
            return rc;
        }

        /* set start transfer image */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_CAP_TRANSFER_START,
            &start_tx, sizeof(start_tx));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            rc = 0;
        }
    }
        break;

    default: {
        const uint8_t restart_stream = M12MO_I2C_VAL_START_RESTART_STREAM;

        /* set capture end for restart stream */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_START_CAP,
            &restart_stream, sizeof(restart_stream));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            rc = 0;
        }
    }
        break;
    }

    g_m12mo_stream_flg = 1;

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc=0;

    CDBG("%s:E stream_type=%d\n", __func__, s_ctrl->camera_stream_type);

    if (g_m12mo_stream_flg <= 0){
        CDBG("%s:X nop\n", __func__);
        return 0;
    }

    switch (s_ctrl->camera_stream_type) {
    case MSM_CAMERA_STREAM_SNAPSHOT: {
        const uint8_t capture_end = M12MO_I2C_VAL_START_STOP_SHOT;

        /* set capture end for ready restart stream */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_START_CAP,
            &capture_end, sizeof(capture_end));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
        /* wait capture end restart streaming status [ms] */
        rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_MODE, 500);
        if (rc < 0) {
            pr_err("%s:X snapshot capture end wait %dms irq failed. rc=%d\n",
                __func__, 500, rc);
            return rc;
        }
    }
        break;

    default: {
        const uint8_t stop_stream = M12MO_I2C_VAL_START_STOP_STREAM;

        /* stop preview stream */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_START_CAP,
            &stop_stream, sizeof(stop_stream));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            rc = 0;
        }
    }
        break;
    }

    g_m12mo_stream_flg = 0;

    CDBG("%s:X g_m12mo_stream_flg=%d\n", __func__, g_m12mo_stream_flg);
    return rc;
}

static int32_t m12mo_set_bining_mode(struct msm_sensor_ctrl_t *s_ctrl, uint8_t cap_mode, uint8_t mon_size)
{
    int32_t rc = 0;
    CDBG("%s:E cap_mode=0x%02x, mon_size=0x%02x\n", __func__, cap_mode, mon_size);

    if((cap_mode == g_m12mo_last_cap_mode) && (mon_size == g_m12mo_last_mon_size)) {
        pr_warn("%s:X same value skip. cap_mode= %d\n", __func__, cap_mode);
        return 0;
    }

    /* already parameter setting mode, before 1st start stream */
    if (g_m12mo_stream_flg >= 0) {
        const uint8_t mode_parm_set = M12MO_I2C_VAL_MODE_PARAM_SETTING;

        /* change mode to parameter setting */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_SYS_MODE,
            &mode_parm_set, sizeof(mode_parm_set));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
        /* wait change mode [ms] */
        rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_MODE, 500);
        if (rc < 0) {
            pr_err("%s:X mode change to parameter setting wait %dms irq failed. rc=%d\n", __func__, 500, rc);
            return rc;
        }
    }

    /* set preview size */
    if (mon_size != M12MO_I2C_VAL_MON_SIZE_INVALID) {
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_PRM_MOD, M12MO_I2C_CAT_PRM_MOD_MON_SIZE,
            &mon_size, sizeof(mon_size));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
    }
    /* change capture mode */
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_CAP_MODE,
        &cap_mode, sizeof(cap_mode));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        g_m12mo_last_cap_mode = cap_mode;
        rc = 0; /* return success. */
    }

    /* not change to monitor mode now, will set after 1st start stream */
    if (g_m12mo_stream_flg >= 0) {
        const uint8_t mode_monitor = M12MO_I2C_VAL_MODE_MONITOR;
        struct sensor_af_info_t af_info;

        /* change mode to monitor */
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_SYS_MODE,
            &mode_monitor, sizeof(mode_monitor));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
        /* wait start streaming status [ms] */
        rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_MODE, 500);
        if (rc < 0) {
            pr_err("%s:X mode change to monitor wait %dms irq failed. rc=%d\n", __func__, 500, rc);
            return rc;
        }

        /* set af mode after change mode to monitor */
        if (g_m12mo_af_mode < FOCUS_MODE_MAX) {
            af_info.mode = g_m12mo_af_mode;
            g_m12mo_af_mode = FOCUS_MODE_MAX;   /* force set mode */
            rc = m12mo_set_focus_mode(s_ctrl, &af_info);
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            }
        }
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_stream_type(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    uint8_t cap_mode = M12MO_I2C_VAL_CAP_MODE_INVALID;
    uint8_t mon_size = M12MO_I2C_VAL_MON_SIZE_INVALID;
    struct sensor_stream_type_info_t stream_type_info;
    enum msm_camera_stream_type_t now = s_ctrl->camera_stream_type;
    enum msm_camera_stream_type_t req;

    CDBG("%s:E now= %d\n", __func__, now);

    if (copy_from_user(&stream_type_info, (void *)cdata->cfg.setting,
        sizeof(struct sensor_stream_type_info_t))) {
        pr_err("%s:X %d failed\n", __func__, __LINE__);
        return -EFAULT;
    }
    req = stream_type_info.stream_type;
    CDBG("%s: req= %d, force_bining= %d\n", __func__,
        req, g_m12mo_force_bining_flg);

    if ( (req == MSM_CAMERA_STREAM_VIDEO)
        || (g_m12mo_force_bining_flg && (now == MSM_CAMERA_STREAM_SNAPSHOT || now == MSM_CAMERA_STREAM_INVALID))
        || (g_m12mo_force_bining_flg && now == MSM_CAMERA_STREAM_PREVIEW && req != MSM_CAMERA_STREAM_SNAPSHOT)
        || (g_m12mo_force_bining_flg && now == MSM_CAMERA_STREAM_VIDEO && req == MSM_CAMERA_STREAM_PREVIEW)) {
        cap_mode = M12MO_I2C_VAL_CAP_MODE_SINGLE;
        switch (stream_type_info.res)
        {
        case 1:
            mon_size = M12MO_I2C_VAL_MON_SIZE_3_25M;
            break;
        case 2:
            mon_size = M12MO_I2C_VAL_MON_SIZE_2_5M;
            break;
        case 3:
            mon_size = M12MO_I2C_VAL_MON_SIZE_1_03M;
            break;
        case 4:
            mon_size = M12MO_I2C_VAL_MON_SIZE_SVGA;
            break;
        default:
            pr_err("%s: %d failed. res=%d\n", __func__, __LINE__,
                stream_type_info.res);
            break;
        }
    } else if ( (now == MSM_CAMERA_STREAM_VIDEO || now == MSM_CAMERA_STREAM_INVALID)
        || (g_m12mo_force_bining_flg && (req == MSM_CAMERA_STREAM_SNAPSHOT)) ) {
        cap_mode = M12MO_I2C_VAL_CAP_MODE_ZSL;
        mon_size = M12MO_I2C_VAL_MON_SIZE_3_25M;
    }

    if ((now == req) && (mon_size == g_m12mo_last_mon_size)) {
        pr_warn("%s:X same value skip. stream_type= %d\n", __func__, req);
        return rc;
    }

    if (cap_mode != M12MO_I2C_VAL_CAP_MODE_INVALID) {
        CDBG("%s write cap_mode= 0x%02x, mon_size= 0x%02x (res= %d)\n", __func__,
            cap_mode, mon_size, stream_type_info.res);
        rc = m12mo_set_bining_mode(s_ctrl, cap_mode, mon_size);
    }

    /* set stream type */
    s_ctrl->camera_stream_type = req;
    g_m12mo_last_mon_size = mon_size;

    CDBG("%s:X new= %d. rc=%d\n", __func__, s_ctrl->camera_stream_type, rc);
    return rc;
}

static int32_t m12mo_set_fps_effect(const struct msm_sensor_ctrl_t *s_ctrl, uint8_t data)
{
    int32_t rc = 0;
    CDBG("%s: data=0x%02x\n", __func__, data);
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_FPS_CTRL, &data, sizeof(data));
    if (rc < 0) {
        pr_err("%s:X %d write fps failed. rc=%d data=%d\n", __func__, __LINE__, rc, data);
    } else {
        const uint8_t update = 0x01;
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_FPS_UPDATE, &update, sizeof(update));
        if (rc < 0) {
            pr_err("%s:X %d write fps update failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
        }
    }
    return rc;
}

static int32_t m12mo_set_fps(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    uint8_t fps8 = 0;
    struct sensor_fps_info_t *fps_info = &(cdata->cfg.fps_info);

    CDBG("%s: g_m12mo_kyocera_camera_flg=%d min_fps:%d max_fps:%d video_min_fps:%d video_max_fps:%d\n",
         __func__, g_m12mo_kyocera_camera_flg, fps_info->min_fps, fps_info->max_fps,
         fps_info->video_min_fps, fps_info->video_max_fps);

    if(g_m12mo_continuous_shoot_flg) {
        pr_warn("%s: continuous shooting return", __func__);
        return 0;
    }

    if((g_m12mo_last_fps_info.min_fps == fps_info->min_fps) &&
       (g_m12mo_last_fps_info.max_fps == fps_info->max_fps) &&
       (g_m12mo_last_fps_info.video_min_fps == fps_info->video_min_fps) &&
       (g_m12mo_last_fps_info.video_max_fps == fps_info->video_max_fps)) {
        pr_warn("%s: same value return", __func__);
        return 0;
    }

    if((fps_info->min_fps != fps_info->video_min_fps) ||
       (fps_info->max_fps != fps_info->video_max_fps)) {
        pr_warn("%s: fps range unmatch (nomral[%d - %d] video[%d - %d])",
                __func__, fps_info->min_fps, fps_info->max_fps,
                fps_info->video_min_fps, fps_info->video_max_fps);
    }

    if(fps_info->min_fps != fps_info->max_fps) {
        fps8 = M12MO_I2C_VAL_FPS_CTRL_AUTO;
    } else {
      switch(fps_info->min_fps) {
      case 120000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_120FPS;
        break;
      case 60000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_60FPS;
        break;
      case 30000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_30FPS;
        break;
      case 28000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_28FPS;
        break;
      case 24000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_24FPS;
        break;
      case 20000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_20FPS;
        break;
      case 15000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_15FPS;
        break;
      case 12000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_12FPS;
        break;
      case 10000:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_10FPS;
        break;
      case 7500:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_7_5FPS;
        break;
      default:
        fps8 = M12MO_I2C_VAL_FPS_CTRL_AUTO;
        break;
      }
    }

    rc = m12mo_set_fps_effect(s_ctrl, fps8);
    if(rc < 0) {
        pr_err("%s:X %d failed.\n", __func__, __LINE__);
    } else {
        g_m12mo_last_fps_info = cdata->cfg.fps_info;
        g_m12mo_last_fps_category_data = fps8;
        if(fps8 == M12MO_I2C_VAL_FPS_CTRL_15FPS && g_m12mo_kyocera_camera_flg) {
            pr_warn("%s: low temperature detected.", __func__);
            g_m12mo_force_bining_flg = 1;
            rc = m12mo_set_bining_mode(s_ctrl,
                M12MO_I2C_VAL_CAP_MODE_SINGLE, M12MO_I2C_VAL_MON_SIZE_INVALID);
        }
    }
    return rc;
}

static int32_t m12mo_set_continuous_shoot_ex(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    int32_t shoot = 0;
    uint8_t fps8;

    if (copy_from_user(&shoot, (void *)cdata->cfg.setting,
        sizeof(shoot))) {
        pr_err("%s:X %d copy_from_user failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    if(shoot == 0) {
        fps8 = (g_m12mo_last_fps_category_data == 0xff)?M12MO_I2C_VAL_FPS_CTRL_AUTO
                                                       :g_m12mo_last_fps_category_data;
    } else {
        fps8 = M12MO_I2C_VAL_FPS_CTRL_30FPS;
    }

    CDBG("%s shoot:%d fps8=0x%02x\n", __func__, shoot, fps8);

    rc = m12mo_set_fps_effect(s_ctrl, fps8);
    if(rc < 0) {
        pr_err("%s:X %d failed.\n", __func__, __LINE__);
    } else {
        g_m12mo_continuous_shoot_flg = shoot;
    }
    return rc;
}

static int32_t m12mo_set_wdr_ex(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    int32_t wdr = 0;
    uint8_t wdr8 = 0;

    if (copy_from_user(&wdr, (void *)cdata->cfg.setting,
        sizeof(wdr))) {
        pr_err("%s:X %d copy_from_user failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    CDBG("%s val:%d", __func__, wdr);

    switch(wdr) {
    case WDR_MODE_OFF:
        wdr8 = 0x00;
        break;
    case WDR_MODE_AUTO:
        wdr8 = 0x02;
        break;
    case WDR_MODE_ON:
        wdr8 = 0x01;
        break;
    default:
        pr_err("%s:X %d invalid wdr mode %d \n", __func__, __LINE__, wdr);
        return -EINVAL;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_WDR_EN, &wdr8, sizeof(wdr8));
    if (rc < 0) {
        pr_err("%s:X %d write wdr mode failed. rc=%d wdr8=%d\n", __func__, __LINE__, rc, wdr8);
    } else {
        rc = 0;
    }
    return rc;
}

static int32_t m12mo_set_af_roi_effect(const struct msm_sensor_ctrl_t *s_ctrl, const struct sensor_roi_info_t *af_roi, int face_flag)
{
    const uint8_t isp_param[][3] = {
        /* cat.               roi_byte                                 upd_byte                            */
        { M12MO_I2C_CAT_LENS, M12MO_I2C_CAT_LENS_AF_ROI_LEFT_POSITION, M12MO_I2C_CAT_LENS_AF_ROI_UPDATE_TRG },
        { M12MO_I2C_CAT_MON , M12MO_I2C_CAT_MON_ROI_LEFT_POSITION    , M12MO_I2C_CAT_MON_ROI_UPDATE_TRG     },
    };

    int32_t rc=0;
    int16_t left, top, right, bottom;
    uint8_t reg_data[8];
    uint8_t update = (face_flag == 0)?0x01:0x02;
    uint8_t area = (face_flag == 0)?0x01:0x02;
    int16_t width, height;
    int16_t after_width, after_height;

    uint8_t cat      = isp_param[face_flag][0];
    uint8_t roi_byte = isp_param[face_flag][1];
    uint8_t upd_byte = isp_param[face_flag][2];

    CDBG("%s:E face_flg= %d cat=0x%02x roi_byte=0x%02x upd_byte=0x%02x\n", __func__, face_flag, cat, roi_byte, upd_byte);

    rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                    M12MO_I2C_CAT_LENS_AF_AREA_MODE,
                                    &area,
                                    sizeof(area));
    if (rc < 0) {
        pr_err("%s:X %d write area mode failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    CDBG("%s: offset_w=%d offset_h=%d rate_numer=%d denom=%d\n", __func__,
         g_m12mo_roi_offset_w, g_m12mo_roi_offset_h, g_m12mo_roi_rate_numer, g_m12mo_roi_rate_denom);

    left   = af_roi->left * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom + g_m12mo_roi_offset_w;
    top    = af_roi->top  * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom + g_m12mo_roi_offset_h;
    right  = left + (af_roi->width  * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom);
    bottom = top  + (af_roi->height * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom);

    left   = clamp_t(int16_t, left,   0, g_m12mo_mon_size_width);
    top    = clamp_t(int16_t, top,    0, g_m12mo_mon_size_height);
    right  = clamp_t(int16_t, right,  0, g_m12mo_mon_size_width);
    bottom = clamp_t(int16_t, bottom, 0, g_m12mo_mon_size_height);

    CDBG("%s: calc data = (%4d,%4d)(%4d,%4d)\n", __func__, left, top, right, bottom);

    width  = right - left;
    height = bottom - top;

    after_width  = (width*g_m12mo_zoom_table[0])/g_m12mo_zoom_table[g_m12mo_zoom_position];
    after_height = (height*g_m12mo_zoom_table[0])/g_m12mo_zoom_table[g_m12mo_zoom_position];

    left  = left + ((width/2) - (after_width/2));
    top   = top  + ((height/2) - (after_height/2));
    right = left + after_width;
    bottom= top  + after_height;
    
    CDBG("%s: calc data (after zoom:%d) = (left,top,right,bottom):(%4d,%4d,%4d,%4d)\n", __func__, g_m12mo_zoom_position, left, top, right, bottom);
    CDBG("%s: (width,height) before:(%4d,%4d) after:(%4d,%4d)\n", __func__, width, height, after_width, after_height);

    reg_data[0] = (left >> 8) & 0xff;
    reg_data[1] = (left & 0xff);
    reg_data[2] = (top >> 8) & 0xff;
    reg_data[3] = (top & 0xff);
    reg_data[4] = (right >> 8) & 0xff;
    reg_data[5] = (right & 0xff);
    reg_data[6] = (bottom >> 8) & 0xff;
    reg_data[7] = (bottom & 0xff);

    CDBG("%s: reg_data = (0x%02x%02x,0x%02x%02x)0x%02x%02xx0x%02x%02x\n",
         __func__,
         reg_data[0],reg_data[1],reg_data[2],reg_data[3],
         reg_data[4],reg_data[5],reg_data[6],reg_data[7]);

    rc = m12mo_i2c_write_category_param(s_ctrl, cat,
                                        roi_byte,
                                        &reg_data[0],
                                        sizeof(reg_data));

    if (rc < 0) {
        pr_err("%s:X %d write roi failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl, cat,
                                        upd_byte,
                                        &update,
                                        sizeof(update));

    if (rc < 0) {
        pr_err("%s:X %d write roi update failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_af_roi(struct msm_sensor_ctrl_t *s_ctrl,
                                struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    struct sensor_roi_info_t *af_roi = (struct sensor_roi_info_t *)&cdata->cfg.roi;
    CDBG("%s: af_roi (%4d,%4d)%4dx%4d\n", __func__,af_roi->left,
         af_roi->top, af_roi->width, af_roi->height);

    if(g_m12mo_fd != 0) {
        pr_warn("%s:X face detection enable skip\n", __func__);
        return 0;
    }

    if(g_m12mo_last_af_roi_info.is_valid &&
       af_roi->top == g_m12mo_last_af_roi_info.roi.top &&
       af_roi->left == g_m12mo_last_af_roi_info.roi.left &&
       af_roi->width == g_m12mo_last_af_roi_info.roi.width &&
       af_roi->height == g_m12mo_last_af_roi_info.roi.height) {
        pr_warn("%s: same value skip\n", __func__);
        return 0;
    }

    if(af_roi->width == 0 && af_roi->height == 0) {
        const uint8_t area = 0x00;
        CDBG("%s: Set to center. g_m12mo_fd_detecting_flg=%d g_m12mo_af_mode=%d\n",
             __func__, g_m12mo_fd_detecting_flg, g_m12mo_af_mode);
        rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                    M12MO_I2C_CAT_LENS_AF_AREA_MODE,
                                    &area,
                                    sizeof(area));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
            g_m12mo_last_af_roi_info.is_valid = 0;
        }
    } else {
        rc = m12mo_set_af_roi_effect(s_ctrl, af_roi, 0);
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
            g_m12mo_last_af_roi_info.roi = *af_roi;
            g_m12mo_last_af_roi_info.is_valid = 1;
        }
    }

    return rc;
}

static int32_t m12mo_set_face_roi(struct msm_sensor_ctrl_t *s_ctrl,
                                  struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    struct sensor_roi_info_t *af_roi = (struct sensor_roi_info_t *)&cdata->cfg.roi;
    g_m12mo_fd_detecting_flg = (af_roi->width > 0 && af_roi->height > 0);

    CDBG("%s:E af_roi (%4d,%4d)%4dx%4d g_m12mo_fd_detecting_flg=%d\n", __func__, af_roi->left,
         af_roi->top, af_roi->width, af_roi->height, g_m12mo_fd_detecting_flg);

    if(g_m12mo_fd == 0) {
        pr_warn("%s:X face detection disable skip\n", __func__);
        return 0;
    }

    if(g_m12mo_fd_detecting_flg &&
       ((g_m12mo_af_mode == FOCUS_MODE_AUTO) ||
        (g_m12mo_af_mode == FOCUS_MODE_CONTINOUS_PICTURE))) {
        rc = m12mo_set_af_roi_effect(s_ctrl, af_roi, 1);
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
            g_m12mo_last_face_roi_info.roi = *af_roi;
            g_m12mo_last_face_roi_info.is_valid = 1;
        }
    } else {
        const uint8_t area = 0x00;
        CDBG("%s: Set to center. g_m12mo_fd_detecting_flg=%d g_m12mo_af_mode=%d\n",
             __func__, g_m12mo_fd_detecting_flg, g_m12mo_af_mode);
        rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                    M12MO_I2C_CAT_LENS_AF_AREA_MODE,
                                    &area,
                                    sizeof(area));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
            g_m12mo_last_face_roi_info.is_valid = 0;
        }
    }

    return rc;
}

static int32_t m12mo_set_ae_mode_effect(const struct msm_sensor_ctrl_t *s_ctrl, uint8_t data) {
    int32_t rc = 0;
    uint8_t update = M12MO_I2C_VAL_AE_PARAM_UPDATE;

    CDBG("%s:E data=0x%02x\n", __func__, data);

    rc = m12mo_i2c_write_category_param(s_ctrl,
             M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_MODE, &data, sizeof(data));
    if (rc < 0) {
        pr_err("%s:X %d write ae mode failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
             M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_PARAM_UPDATE, &update, sizeof(update));
    if (rc < 0) {
        pr_err("%s:X %d write ae mode update failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    CDBG("%s:X rc=%d\n", __func__, rc);
    return 0;
}

static int32_t m12mo_set_aec_roi_effect(const struct msm_sensor_ctrl_t *s_ctrl, const struct sensor_roi_info_t *ae_roi) {
    int32_t rc = 0;
    const uint8_t ae_mode = M12MO_I2C_VAL_AE_MODE_ROI;
    const uint8_t update = M12MO_I2C_VAL_AE_PARAM_UPDATE;
    uint8_t reg_data[8];
    int16_t left, top, right, bottom;
    int16_t width,height;
    int16_t after_width,after_height;

    CDBG("%s:E \n", __func__);
    left   = ae_roi->left * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom + g_m12mo_roi_offset_w;
    top    = ae_roi->top  * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom + g_m12mo_roi_offset_h;
    right  = left + (ae_roi->width  * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom);
    bottom = top  + (ae_roi->height * g_m12mo_roi_rate_numer / g_m12mo_roi_rate_denom);

    left   = clamp_t(int16_t, left,   0, g_m12mo_mon_size_width);
    top    = clamp_t(int16_t, top,    0, g_m12mo_mon_size_height);
    right  = clamp_t(int16_t, right,  0, g_m12mo_mon_size_width);
    bottom = clamp_t(int16_t, bottom, 0, g_m12mo_mon_size_height);

    CDBG("%s: left=%d top=%d right=%d bottom=%d\n", __func__, left, top, right, bottom);

    width  = right - left;
    height = bottom - top;

    after_width  = (width*g_m12mo_zoom_table[0])/g_m12mo_zoom_table[g_m12mo_zoom_position];
    after_height = (height*g_m12mo_zoom_table[0])/g_m12mo_zoom_table[g_m12mo_zoom_position];

    left  = left + ((width/2) - (after_width/2));
    top   = top  + ((height/2) - (after_height/2));
    right = left + after_width;
    bottom= top  + after_height;
    

    CDBG("%s: calc data (after zoom:%d) = (left,top,right,bottom):(%4d,%4d,%4d,%4d)\n", __func__, g_m12mo_zoom_position, left, top, right, bottom);
    CDBG("%s: (width,height) before:(%4d,%4d) after:(%4d,%4d)\n", __func__, width, height, after_width, after_height);

    rc = m12mo_set_ae_mode_effect(s_ctrl, ae_mode);
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    reg_data[0] = (left >> 8) & 0xff;
    reg_data[1] = (left & 0xff);
    reg_data[2] = (top >> 8) & 0xff;
    reg_data[3] = (top & 0xff);
    reg_data[4] = (right >> 8) & 0xff;
    reg_data[5] = (right & 0xff);
    reg_data[6] = (bottom >> 8) & 0xff;
    reg_data[7] = (bottom & 0xff);

    CDBG("%s: reg_data = (0x%02x%02x,0x%02x%02x)0x%02x%02xx0x%02x%02x\n",
         __func__,
         reg_data[0],reg_data[1],reg_data[2],reg_data[3],
         reg_data[4],reg_data[5],reg_data[6],reg_data[7]);

    rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_AE,
                                        M12MO_I2C_CAT_AE_ROI_LEFT_POSITION,
                                        &reg_data[0],
                                        sizeof(reg_data));
    if (rc < 0) {
        pr_err("%s:X %d write ae roi failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_AE,
                                        M12MO_I2C_CAT_AE_ROI_UPDATE_TRG,
                                        &update,
                                        sizeof(update));
    if (rc < 0) {
        pr_err("%s:X %d write ae roi update failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return 0;
}

static int32_t m12mo_set_aec_roi(struct msm_sensor_ctrl_t *s_ctrl,
                                 struct sensorb_cfg_data *cdata) {
    int32_t rc = 0;
    struct sensor_roi_info_t *ae_roi;
    ae_roi = (struct sensor_roi_info_t *)&cdata->cfg.roi;

    CDBG("%s:E ae_roi (%4d,%4d)%4dx%4d\n", __func__,
        ae_roi->left,ae_roi->top, ae_roi->width, ae_roi->height);

    if(g_m12mo_fd != 0) {
        pr_warn("%s:X face detection enable skip\n", __func__);
        return 0;
    }

    if(ae_roi->left < 0 && ae_roi->top < 0) {
        if(g_m12mo_last_ae_mode == 0xff) {
            pr_warn("%s:X g_m12mo_last_ae_mode is not set\n", __func__);
            rc = 0;
        } else {
            rc = m12mo_set_ae_mode_effect(s_ctrl, g_m12mo_last_ae_mode);
        }
        return rc;
    }

    rc = m12mo_set_aec_roi_effect(s_ctrl, ae_roi);
    if(rc < 0){
        pr_err("%s:X %d set ae roi failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    g_m12mo_last_aec_roi_info.roi = *ae_roi;
    g_m12mo_last_aec_roi_info.is_valid = 1;
    CDBG("%s:X\n", __func__);
    return 0;
}

static int32_t m12mo_stop_lens_if_busy(struct msm_sensor_ctrl_t *s_ctrl, int wait_af_op_flg) {
    int32_t rc = 0;
    uint8_t busy = 0;
    uint8_t stop = 0;
    uint8_t start = 0;
    uint8_t result = 0;
    int32_t count;

    CDBG("%s:E wait_af_op_flg=%d\n", __func__, wait_af_op_flg);

    M12MO_LOG_MUTEX_LOCK(m12mo_af_forcusing_lock);
    if (g_m12mo_af_forcusing_flg) {
        CDBG("%s: AF_START completion\n", __func__);
        g_m12mo_completion_flg = 1;
        complete(&g_m12mo_irq_wait);
    }
    M12MO_LOG_MUTEX_UNLOCK(m12mo_af_forcusing_lock);

    rc = m12mo_i2c_read_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                       M12MO_I2C_CAT_LENS_AF_BUSY,
                                       &busy, sizeof(busy));
    CDBG("%s: busy=0x%02x rc=%d\n", __func__, busy, rc);

    if (rc < 0) {
        pr_err("%s:X %d read af busy failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    if(busy == 0) {
        CDBG("%s:X no-op busy=0x%02x\n", __func__, busy);
        return 0;
    }

    if(wait_af_op_flg) {
        for (count = 0; count < 15; count++) {
            rc = m12mo_i2c_read_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                               M12MO_I2C_CAT_LENS_AF_RESULT,
                                               &result, sizeof(result));
            CDBG("%s: count=%d result=0x%02x\n", __func__, count, result);
            if(result != 0x10) {// AF operating.
                CDBG("%s: count=%d AF not operating.\n", __func__, count);
                break;
            } else {
                msleep(10);
            }
        }
    }

    rc = m12mo_i2c_read_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                       M12MO_I2C_CAT_LENS_AF_START,
                                       &start, sizeof(start));

    if (rc < 0) {
        pr_err("%s:X %d read af start failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    CDBG("%s: start=0x%02x\n", __func__, start);

    switch(start) {
    case 0x00: // AF stop by force or AF released
    case 0x04: // CAF paused
        pr_warn("%s:X no-op start=0x%02x\n", __func__, start);
        return 0;
    case 0x03: // CAF operatinig
        stop = 0x04; // CAF pause
        break;
    case 0x01: // AF starting
    case 0x02: // AF done
    default:
        stop = 0x00; // AF stop by force
        break;
    }

    M12MO_LOG_MUTEX_LOCK(m12mo_af_free_lock);
    rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                        M12MO_I2C_CAT_LENS_AF_START,
                                        &stop,
                                        sizeof(stop));

    CDBG("%s: AF_BUSY=0x%02x start=0x%02x stop=0x%02x g_m12mo_af_mode=%d rc=%d\n",
         __func__, busy, start, stop, g_m12mo_af_mode, rc);

    if(rc < 0) {
        M12MO_LOG_MUTEX_UNLOCK(m12mo_af_free_lock);
        pr_err("%s:X %d af stop failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_AF, 1000);
    M12MO_LOG_MUTEX_UNLOCK(m12mo_af_free_lock);
    if (rc < 0) {
        pr_err("%s:X af status interrupt wait %dms irq failed. rc=%d\n",
        __func__, 1000, rc);
        return rc;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_lens_position(struct msm_sensor_ctrl_t *s_ctrl, int32_t focus_mode)
{
    int32_t rc = 0;
    uint8_t start;
    uint8_t range;
    int32_t count;
    int8_t value;

    CDBG("%s:E mode=0x%02x\n", __func__, focus_mode);

    switch(focus_mode) {
    case FOCUS_MODE_AUTO:
    case FOCUS_MODE_INFINITY:
    case FOCUS_MODE_MACRO:
        range = (focus_mode != FOCUS_MODE_MACRO ? 0x01 : 0x04);
        rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                            M12MO_I2C_CAT_LENS_AF_RANGE,
                                            &range,
                                            sizeof(range));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            rc = 0;
        }
        /* Wait lens move completion. */
        for (count = 0; count < 3; count++) {
            msleep(50);
            rc = m12mo_i2c_read_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                               M12MO_I2C_CAT_LENS_AF_BUSY,
                                               &value, sizeof(value));
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            } else {
                rc = 0;
            }
            CDBG("%s: busy =0x%02x\n", __func__, value);
            if(value != 0x01) {// 0x01: Lens busy.
                CDBG("%s: busy break. count=%d\n", __func__, count);
                break;
            }
        }
        break;
    case FOCUS_MODE_CONTINOUS_VIDEO:
    case FOCUS_MODE_CONTINOUS_PICTURE:
        start = 0x01;
        rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                            M12MO_I2C_CAT_LENS_AF_START,
                                            &start,
                                            sizeof(start));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
        /* Wait CAF start completion. */
        for (count = 0; count < 3; count++) {
            msleep(50);
            rc = m12mo_i2c_read_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                               M12MO_I2C_CAT_LENS_AF_START,
                                               &value, sizeof(value));
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            } else {
                rc = 0;
            }
            CDBG("%s: af starting =0x%02x\n", __func__, value);
            if(value != 0x01) {// 0x01:AF starting
                CDBG("%s: af starting break. count=%d\n", __func__, count);
                break;
            }
        }
        break;
    default:
        pr_warn("%s: invalid focus_mode(%d)\n", __func__, focus_mode);
        break;
    }
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_autofocus(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc=0;
    uint8_t start = 0x01;
    int32_t wait_af_op_flag;
    CDBG("%s:E\n", __func__);

    if(g_m12mo_af_mode >= FOCUS_MODE_MAX) {
        CDBG("%s:X no-op\n", __func__);
        return 0;
    }

    wait_af_op_flag = ((g_m12mo_af_mode == FOCUS_MODE_CONTINOUS_VIDEO) ||
                       (g_m12mo_af_mode == FOCUS_MODE_CONTINOUS_PICTURE));

    rc = m12mo_stop_lens_if_busy(s_ctrl, wait_af_op_flag);

    if(rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else if(wait_af_op_flag) {
        CDBG("%s:X wait_af_op_flag=%d\n", __func__, wait_af_op_flag);
        return 0;
    }

    M12MO_LOG_MUTEX_LOCK(m12mo_af_forcusing_lock);
    rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                        M12MO_I2C_CAT_LENS_AF_START,
                                        &start,
                                        sizeof(start));
    if (rc < 0) {
        M12MO_LOG_MUTEX_UNLOCK(m12mo_af_forcusing_lock);
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    rc = 0;

    g_m12mo_af_forcusing_flg = 1;
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_cancel_autofocus(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    CDBG("%s:E\n", __func__);
    if(g_m12mo_af_mode >= FOCUS_MODE_MAX) {
        CDBG("%s:X no-op\n", __func__);
        return 0;
    }
    rc = m12mo_stop_lens_if_busy(s_ctrl, 0);
    if(rc >= 0) {
        rc = m12mo_set_lens_position(s_ctrl, g_m12mo_af_mode);
    }
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_focus_mode(struct msm_sensor_ctrl_t *s_ctrl, struct sensor_af_info_t *af_info)
{
    int32_t rc=0;
    uint8_t mode;

    CDBG("%s:E mode:%d\n", __func__, af_info->mode);

    if(rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    if(af_info->mode == g_m12mo_af_mode) {
        pr_warn("%s:X same af_mode(%d) return\n", __func__, af_info->mode);
        return 0;
    }
    if(g_m12mo_stream_flg < 0) {
        g_m12mo_pending_af_mode = af_info->mode;
        pr_warn("%s:X monitor mode is not started return g_m12mo_pending_af_mode=%d\n", __func__, g_m12mo_pending_af_mode);
        return 0;
    }

    switch(af_info->mode) {
    case FOCUS_MODE_AUTO:
    case FOCUS_MODE_INFINITY:
    case FOCUS_MODE_MACRO:
        mode = (g_m12mo_force_bining_flg ?
            M12MO_I2C_VAL_AF_MODE_SINGLE_CONT :
            M12MO_I2C_VAL_AF_MODE_SINGLE_HYBRID);
        break;
    case FOCUS_MODE_CONTINOUS_VIDEO:
        mode = M12MO_I2C_VAL_AF_MODE_CONTINUOUS_CONT;
        break;
    case FOCUS_MODE_CONTINOUS_PICTURE:
        mode = (g_m12mo_force_bining_flg ?
            M12MO_I2C_VAL_AF_MODE_CONTINUOUS_CONT :
            M12MO_I2C_VAL_AF_MODE_CONTINUOUS_HYBRID);
        break;
    default:
        pr_err("%s:X %d invalid focus mode %d \n", __func__, __LINE__, af_info->mode);
        return -EINVAL;
    }

    CDBG("%s: mode=0x%02x\n", __func__, mode);

    rc = m12mo_stop_lens_if_busy(s_ctrl, 0);
    if (rc < 0) {
        pr_err("%s:X %d stop lens failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                        M12MO_I2C_CAT_LENS_AF_MODE,
                                        &mode,
                                        sizeof(mode));
    if (rc < 0) {
        pr_err("%s:X %d write af mode failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    rc = m12mo_set_lens_position(s_ctrl, af_info->mode);
    if(rc < 0) {
        pr_err("%s:X %d set lens position failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    g_m12mo_af_mode = af_info->mode;
    CDBG("%s:X mode=%d rc=%d\n", __func__, af_info->mode, rc);
    return rc;
}

int32_t m12mo_sensor_get_af_status(struct msm_sensor_ctrl_t *s_ctrl,
    void __user *argp)
{
    int32_t rc=0;
    uint8_t result = 0x00;
    enum sensor_af_t* status = (enum sensor_af_t*)argp;

    CDBG("%s:E\n", __func__);
    if (g_m12mo_af_mode == FOCUS_MODE_CONTINOUS_VIDEO ||
        g_m12mo_af_mode == FOCUS_MODE_CONTINOUS_PICTURE) {
        g_m12mo_af_forcusing_flg = 0;
        M12MO_LOG_MUTEX_UNLOCK(m12mo_af_forcusing_lock);
    } else {
        M12MO_LOG_MUTEX_LOCK(m12mo_af_free_lock);
        M12MO_LOG_MUTEX_UNLOCK(m12mo_af_forcusing_lock);
        rc = m12mo_i2c_wait_int_factor(s_ctrl, M12MO_I2C_VAL_INT_STATUS_AF, 3000);
        M12MO_LOG_MUTEX_UNLOCK(m12mo_af_free_lock);
        if (rc < 0) {
            pr_err("%s:X af status interrupt wait %dms irq failed. rc=%d\n",
            __func__, 3000, rc);
            *status = SENSOR_AF_NOT_FOCUSSED;
            return 0;
        }
    }
    rc = m12mo_i2c_read_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                       M12MO_I2C_CAT_LENS_AF_RESULT,
                                       &result, sizeof(result));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        *status = SENSOR_AF_NOT_FOCUSSED;
        return 0;
    }

    CDBG("%s: result= 0x%02x\n", __func__, result);
    *status = (result == 0x01) ?SENSOR_AF_FOCUSSED :SENSOR_AF_NOT_FOCUSSED;

    CDBG("%s:X status=%d\n", __func__, *status);
    return 0;
}

static int32_t m12mo_set_led(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    int32_t client;
    uint8_t strobe;
    uint8_t led_lvl;
    const uint8_t strobeoff = M12MO_I2C_VAL_AE_STROBE_EN_OFF;

    CDBG("%s:E val:%d\n", __func__, cdata->cfg.led);

    switch(cdata->cfg.led){
    case CAMERA_LED_OFF:
        client = M12MO_LED_CLIENT_CAMERA;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_OFF;
        break;
    case CAMERA_LED_AUTO:
        client = M12MO_LED_CLIENT_CAMERA;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_AUTO;
        break;
    case CAMERA_LED_ON:
        client = M12MO_LED_CLIENT_CAMERA;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_ON;
        break;
    case CAMERA_LED_TORCH:
        client = M12MO_LED_CLIENT_CAMERA;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_TORCH;
        led_lvl = 3; //75mA
        break;
    case CAMERA_LED_TORCH_MID:
        client = M12MO_LED_CLIENT_LIGHTHAL;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_TORCH;
        led_lvl = 3; //75mA
        break;
    case CAMERA_LED_TORCH_HIGH:
        client = M12MO_LED_CLIENT_LIGHTHAL;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_TORCH;
        led_lvl = 5; //125mA
        break;
    case CAMERA_LED_TORCH_LOW:
        client = M12MO_LED_CLIENT_LIGHTHAL;
        strobe = M12MO_I2C_VAL_AE_STROBE_EN_TORCH;
        led_lvl = 1; //25mA
        break;
    case CAMERA_LED_HAL_OFF:
        /* reset LightHAL client */
        client = M12MO_LED_CLIENT_CAMERA;
        g_m12mo_led_client = client;
        if(g_m12mo_pending_set_led != CAMERA_LED_INVALID){
            strobe = g_m12mo_pending_set_led;
            led_lvl = 3; //75mA only
            g_m12mo_pending_set_led = CAMERA_LED_INVALID;
            CDBG("%s: %d Set pending value\n", __func__, __LINE__);
        } else {
            strobe = M12MO_I2C_VAL_AE_STROBE_EN_OFF;
        }
        break;
    default:
        pr_err("%s:X %d Not support strobe\n", __func__, __LINE__);
        return -EINVAL;
        break;
    }

    /* return camera request if light hal not litht off */
    if (client == M12MO_LED_CLIENT_LIGHTHAL) {
        g_m12mo_led_client = M12MO_LED_CLIENT_LIGHTHAL;
    } else if (g_m12mo_led_client == M12MO_LED_CLIENT_LIGHTHAL) {
        g_m12mo_pending_set_led = strobe;
        CDBG("%s:X %d Return except for LightHAL command. pending strobe_en:0x%02x\n",
            __func__, __LINE__, strobe);
        return rc;
    }


    CDBG("%s: strobe_en:0x%02x\n", __func__, strobe);
    if(strobe == M12MO_I2C_VAL_AE_STROBE_EN_TORCH){
        CDBG("%s: torch led level:0x%02x\n", __func__, led_lvl);

        rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_STROBE_EN, &strobeoff, sizeof(strobeoff));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }

        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_LED_LEVEL, &led_lvl, sizeof(led_lvl));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
    M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_STROBE_EN, &strobe, sizeof(strobe));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }
    CDBG("%s:X\n", __func__);
    return rc;
}

static int32_t m12mo_get_lowlight_intensity(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t status = 0;

    CDBG("%s:E\n", __func__);

    //get strobe status
    rc = m12mo_i2c_read_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_STROBE_STATUS, &status, sizeof(status));
    if (rc != sizeof(status)) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        return rc;
    }
    CDBG("%s strobe status:%d\n", __func__, status);

    if (copy_to_user(cdata->cfg.setting, (void *)&status, sizeof(uint8_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_iso(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    int32_t iso = 0;
    uint8_t iso8 = 0;

    if (copy_from_user(&iso, (void *)cdata->cfg.setting,
        sizeof(iso))) {
        pr_err("%s:X %d copy_from_user failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    CDBG("%s val:%d", __func__, iso);

    switch(iso) {
    case ISO_MODE_AUTO:
        iso8 = 0x00;
        break;
    case ISO_MODE_100:
        iso8 = 0x02;
        break;
    case ISO_MODE_200:
        iso8 = 0x03;
        break;
    case ISO_MODE_400:
        iso8 = 0x04;
        break;
    case ISO_MODE_800:
        iso8 = 0x05;
        break;
    case ISO_MODE_1600:
        iso8 = 0x06;
        break;
    default:
        pr_err("%s:X %d invalid iso mode %d \n", __func__, __LINE__, iso);
        return -EINVAL;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_ISOSEL, &iso8, sizeof(iso8));
    if (rc < 0) {
        pr_err("%s:X %d write isomode failed. rc=%d\n", __func__, __LINE__, rc);
    } else {
        const uint8_t update = 0x01;
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_PARAM_UPDATE, &update, sizeof(update));
        if (rc < 0) {
            pr_err("%s:X %d write ae update failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
        }
    }
    return rc;
}

static int32_t m12mo_set_ae_mode(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    int32_t ae = 0;
    uint8_t ae8 = 0;

    if (copy_from_user(&ae, (void *)cdata->cfg.setting,
        sizeof(ae))) {
        pr_err("%s:X %d copy_from_user failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    CDBG("%s:E val=%d", __func__, ae);

    switch(ae) {
    case AEC_MODE_FRAME_AVERAGE:
        ae8 = M12MO_I2C_VAL_AE_MODE_ALL;
        break;
    case AEC_MODE_CENTER_WEIGHTED:
        ae8 = M12MO_I2C_VAL_AE_MODE_CENTER;
        break;
    default:
        pr_err("%s:X %d invalid ae mode %d \n", __func__, __LINE__, ae);
        return -EINVAL;
    }

    rc = m12mo_set_ae_mode_effect(s_ctrl, ae8);
    if(rc == 0) {
        g_m12mo_last_ae_mode = ae8;
    }
    CDBG("%s:X rc=%d g_m12mo_last_ae_mode=%d", __func__, rc, g_m12mo_last_ae_mode);
    return rc;
}

static int32_t m12mo_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl, const int32_t ev)
{
    int32_t rc;
    uint8_t ev8;

    CDBG("%s:E request ev=%d\n", __func__, ev);

    if (ev < (M12MO_I2C_VAL_AE_EV_BIAS_MIN - M12MO_I2C_VAL_AE_EV_BIAS_OFFSET) ||
        ev > (M12MO_I2C_VAL_AE_EV_BIAS_MAX - M12MO_I2C_VAL_AE_EV_BIAS_OFFSET)) {
        pr_err("%s:X %d invalid ev %d \n", __func__, __LINE__, ev);
        return -EINVAL;
    }
    if (g_m12mo_stream_flg < 0 || g_m12mo_scene_manual_now != M12MO_I2C_VAL_ATSCENE_MAN_AUTO) {
        g_m12mo_pending_ev_bias = ev;
        pr_warn("%s:X pending exposure value. stream flag=%d, scene manual=%d, ev=%d\n",
            __func__, g_m12mo_stream_flg, g_m12mo_scene_manual_now, g_m12mo_pending_ev_bias);
        return 0;
    }

    ev8 = (uint8_t)(ev + M12MO_I2C_VAL_AE_EV_BIAS_OFFSET);

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_EV_BIAS, &ev8, sizeof(ev8));
    if (rc < 0) {
        pr_err("%s:X %d write ev failed. rc=%d\n", __func__, __LINE__, rc);
    } else {
        const uint8_t update = 0x01;
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_PARAM_UPDATE, &update, sizeof(update));
        if (rc < 0) {
            pr_err("%s:X %d write ev update failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
        }
    }

    CDBG("%s:X set ev=0x%02x\n", __func__, ev8);
    return rc;
}

static int32_t m12mo_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t mode,manual,update;
    CDBG("%s:E val:%d\n", __func__, cdata->cfg.wb);

    //awb mode
    if(cdata->cfg.wb == MSM_CAMERA_WB_MODE_AUTO){
         mode = M12MO_I2C_VAL_AWB_MODE_AUTO;
    } else {
         mode = M12MO_I2C_VAL_AWB_MODE_MANUAL;
    }
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_WB, M12MO_I2C_CAT_WB_AWB_MODE, &mode, sizeof(mode));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    //manual
    if(cdata->cfg.wb != MSM_CAMERA_WB_MODE_AUTO){
        switch(cdata->cfg.wb){
        case MSM_CAMERA_WB_MODE_INCANDESCENT:
            manual = M12MO_I2C_VAL_AWB_MN_INCANDESCENT;
            break;
        case MSM_CAMERA_WB_MODE_FLUORESCENT:
            manual = M12MO_I2C_VAL_AWB_MN_SHADE;
            break;
        case MSM_CAMERA_WB_MODE_WARM_FLUORESCENT:
            manual = M12MO_I2C_VAL_AWB_MN_FLUORESCENT_LOW;
            break;
        case MSM_CAMERA_WB_MODE_DAYLIGHT:
            manual = M12MO_I2C_VAL_AWB_MN_DAYLIGHT;
            break;
        default:
            pr_err("%s:X %d invalid value\n", __func__, __LINE__);
            return -EINVAL;
        }
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_WB, M12MO_I2C_CAT_WB_AWB_MANUAL, &manual, sizeof(manual));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
    }
   
    //update
    update = M12MO_I2C_VAL_AWB_PARAM_UPDATE;
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_WB, M12MO_I2C_CAT_WB_AWB_PARAM_UPDATE, &update, sizeof(update));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc=0;
    }
    CDBG("%s:X\n", __func__);
    return rc;
}
static int32_t m12mo_set_effect(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t effect;
    CDBG("%s:E val:%d\n", __func__, cdata->cfg.effect);

    switch(cdata->cfg.effect){
    case MSM_CAMERA_EFFECT_MODE_OFF:
        effect = M12MO_I2C_VAL_EFFECT_OFF;
        break;
    case MSM_CAMERA_EFFECT_MODE_NEGATIVE:
        effect = M12MO_I2C_VAL_EFFECT_NEGA;
        break;
    case MSM_CAMERA_EFFECT_MODE_MONO:
        effect = M12MO_I2C_VAL_EFFECT_GRAY;
        break;
    case MSM_CAMERA_EFFECT_MODE_SEPIA:
        effect = M12MO_I2C_VAL_EFFECT_SEPIA;
        break;
    default:
        pr_err("%s:X %d invalid effect mode\n", __func__, __LINE__);
        return -EINVAL;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_EFFECT, &effect, sizeof(effect));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }
    
    CDBG("%s:X effect=%d\n", __func__, effect);
    return rc;
}
static int32_t m12mo_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t flicker;
    uint8_t update;
    CDBG("%s:E val:%d", __func__, cdata->cfg.antibanding);

    switch(cdata->cfg.antibanding){
    case CAMERA_ANTIBANDING_MODE_AUTO:
        flicker = M12MO_I2C_VAL_AE_FLICKER_AUTO;
        break;
    case CAMERA_ANTIBANDING_MODE_50HZ:
        flicker = M12MO_I2C_VAL_AE_FLICKER_50HZ;
        break;
    case CAMERA_ANTIBANDING_MODE_60HZ:
        flicker = M12MO_I2C_VAL_AE_FLICKER_60HZ;
        break;
    case CAMERA_ANTIBANDING_MODE_OFF:
        flicker = M12MO_I2C_VAL_AE_FLICKER_OFF;
        break;
    default:
        pr_err("%s:X %d invalid antibanding mode.\n", __func__, __LINE__);
        return -EINVAL;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_FLICKER, &flicker, sizeof(flicker));
    if (rc<0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }

    //param update
    update = M12MO_I2C_VAL_AE_PARAM_UPDATE;
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_PARAM_UPDATE, &update, sizeof(update));
    if (rc<0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }
    CDBG("%s:X flicker=%d\n", __func__, flicker);
    return rc;
}
static int32_t m12mo_set_bestshot(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;

    // initialize for manual scene setting case.
    uint8_t antihandshake = M12MO_I2C_VAL_AHS_MON_OFF;
    uint8_t lock   = M12MO_I2C_VAL_ATSCENE_LOCK_ON;
    uint8_t detect = M12MO_I2C_VAL_ATSCENE_DETECT_STOP;
    uint8_t manual = M12MO_I2C_VAL_ATSCENE_MAN_AUTO;
    uint8_t update = M12MO_I2C_VAL_ATSCENE_PARAM_UPDATE;

    CDBG("%s:E val:%d\n", __func__, cdata->cfg.bestshot);

    if(g_m12mo_auto_scene_status == cdata->cfg.bestshot){
        pr_warn("%s:%d No change of the request scene\n", __func__, __LINE__);
        return 0;
    }

    switch(cdata->cfg.bestshot){
    case MSM_CAMERA_SCENE_MODE_AUTO:
        //auto scene lock off
        lock = M12MO_I2C_VAL_ATSCENE_LOCK_OFF;
        //mode auto
        manual = M12MO_I2C_VAL_ATSCENE_MAN_AUTO;
        //auto scene detect start
        detect = M12MO_I2C_VAL_ATSCENE_DETECT_START;
        break;
    case MSM_CAMERA_SCENE_MODE_OFF:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_AUTO;
        break;
    case MSM_CAMERA_SCENE_MODE_LANDSCAPE:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_LANDSCAPE;
        break;
    case MSM_CAMERA_SCENE_MODE_BEACH:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_BEACH;
        break;
    case MSM_CAMERA_SCENE_MODE_NIGHT:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_NIGHT;
        break;
    case MSM_CAMERA_SCENE_MODE_PORTRAIT:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_PORTRAIT;
        break;
    case MSM_CAMERA_SCENE_MODE_BACKLIGHT:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_BACKLIGHT;
        break;
    case MSM_CAMERA_SCENE_MODE_SPORTS:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_SPORTS;
        break;
    case MSM_CAMERA_SCENE_MODE_PARTY:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_PARTY;
        break;
    case MSM_CAMERA_SCENE_MODE_MAPTEXT:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_MEMO;
        break;
    case MSM_CAMERA_SCENE_MODE_ANTISHAKE:
        manual = M12MO_I2C_VAL_ATSCENE_MAN_AUTO;
        antihandshake = M12MO_I2C_VAL_AHS_MON_ON;
        break;
    default:
        pr_err("%s:X %d invalid bestshot mode.\n", __func__, __LINE__);
        return -EINVAL;
    }

    if((antihandshake) || (g_m12mo_auto_scene_status == MSM_CAMERA_SCENE_MODE_ANTISHAKE)){
        rc = m12mo_i2c_write_category_param(s_ctrl,
                M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_AHS_MON, &antihandshake, sizeof(antihandshake));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
    }

    CDBG("%s lock=%d, detect=%d, manual=%d\n", __func__, lock, detect, manual);

    //auto scene lock
    rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_LOCK, &lock, sizeof(lock));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    //auto scene_manual
    rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_MANUAL, &manual, sizeof(manual));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    //auto scene_detect
    rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_DETECT_START, &detect, sizeof(detect));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    //auto scene param update
    update = M12MO_I2C_VAL_ATSCENE_PARAM_UPDATE;
    rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_PARAM_UPDATE, &update, sizeof(update));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }
    g_m12mo_auto_scene_status = cdata->cfg.bestshot;
    g_m12mo_scene_manual_now = manual;

    /* Restore pending ev bias setting */
    if (g_m12mo_scene_manual_now == M12MO_I2C_VAL_ATSCENE_MAN_AUTO) {
        if (g_m12mo_pending_ev_bias != M12MO_I2C_VAL_AE_EV_BIAS_INVALID) {
            rc = m12mo_set_exposure_compensation(s_ctrl, g_m12mo_pending_ev_bias);
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            } else {
                pr_warn("%s: restore pending exposure setting. ev=%d\n",
                    __func__, g_m12mo_pending_ev_bias);
                g_m12mo_pending_ev_bias = M12MO_I2C_VAL_AE_EV_BIAS_INVALID;
            }
        }
    }


    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}
static int32_t m12mo_get_auto_scene(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t value;

    CDBG("%s:E\n", __func__);

    if(g_m12mo_auto_scene_status == MSM_CAMERA_SCENE_MODE_AUTO){
        // During the auto scene
        CDBG("%s get scene:%d\n", __func__, g_m12mo_scene_manual_now);

        switch(g_m12mo_scene_manual_now){
        case M12MO_I2C_VAL_ATSCENE_MAN_AUTO:
             value = MSM_CAMERA_SCENE_MODE_OFF;
             break;
        case M12MO_I2C_VAL_ATSCENE_MAN_LANDSCAPE:
             value = MSM_CAMERA_SCENE_MODE_LANDSCAPE;
             break;
        case M12MO_I2C_VAL_ATSCENE_MAN_NIGHT:
             value = MSM_CAMERA_SCENE_MODE_NIGHT;
             break;
        case M12MO_I2C_VAL_ATSCENE_MAN_PORTRAIT:
             value = MSM_CAMERA_SCENE_MODE_PORTRAIT;
             break;
        case M12MO_I2C_VAL_ATSCENE_MAN_BACKLIGHT:
             value = MSM_CAMERA_SCENE_MODE_BACKLIGHT;
             break;
        case M12MO_I2C_VAL_ATSCENE_MAN_PARTY:
             value = MSM_CAMERA_SCENE_MODE_PARTY;
             break;
        default:
             pr_warn("%s:%d Not Support scene detect = %d\n", __func__, __LINE__, value);
             value = MSM_CAMERA_SCENE_MODE_OFF;
             break;
        }
    } else {
        // Not in auto scene
        value = MSM_CAMERA_SCENE_MODE_OFF;
    }

    if (copy_to_user(cdata->cfg.setting, (void *)&value, sizeof(uint8_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }
    
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}
static int32_t m12mo_set_update_scene(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t cur_scene = 0;
    uint8_t manual;
    uint8_t update;

    CDBG("%s:E\n", __func__);

    if(g_m12mo_auto_scene_status == MSM_CAMERA_SCENE_MODE_AUTO){
        if(g_m12mo_fd_detecting_flg == 1){
            // During face detect
            manual = M12MO_I2C_VAL_ATSCENE_MAN_PORTRAIT;
            CDBG("%s During face detect\n", __func__);
        } else {
            // During the auto scene
            rc = m12mo_i2c_read_category_param(s_ctrl,
                M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_CURRENT_SCENE, &cur_scene, sizeof(cur_scene));
            if (rc != sizeof(cur_scene)) {
                pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
                return rc;
            }
            CDBG("%s detect auto scene:%d\n", __func__, cur_scene);

            switch(cur_scene){
            case M12MO_I2C_VAL_CURSCENE_AUTO:
                 manual = M12MO_I2C_VAL_ATSCENE_MAN_AUTO;
                 break;
            case M12MO_I2C_VAL_CURSCENE_LANDSCAPE:
                 manual = M12MO_I2C_VAL_ATSCENE_MAN_LANDSCAPE;
                 break;
            case M12MO_I2C_VAL_CURSCENE_NIGHT:
                 manual = M12MO_I2C_VAL_ATSCENE_MAN_NIGHT;
                 break;
            case M12MO_I2C_VAL_CURSCENE_BACKLIGHT:
                 manual = M12MO_I2C_VAL_ATSCENE_MAN_BACKLIGHT;
                 break;
            case M12MO_I2C_VAL_CURSCENE_PARTY:
                 manual = M12MO_I2C_VAL_ATSCENE_MAN_PARTY;
                 break;
            case M12MO_I2C_VAL_CURSCENE_PORTRAIT:
            case M12MO_I2C_VAL_CURSCENE_BEACH:
            case M12MO_I2C_VAL_CURSCENE_SNOW:
            case M12MO_I2C_VAL_CURSCENE_MAPTEXT:
            case M12MO_I2C_VAL_CURSCENE_QRCODE:
            case M12MO_I2C_VAL_CURSCENE_SUNSET:
                 CDBG("%s:%d not support detect scene = %d\n", __func__, __LINE__, cur_scene);
                 return 0;
            default:
                 CDBG("%s:%d invalid value = %d\n", __func__, __LINE__, cur_scene);
                 return -EINVAL;
            }
        }

        if(manual == g_m12mo_scene_manual_now){
            CDBG("%s:%d No change of the detected scene = %d\n", __func__, __LINE__, manual);
            return 0;
        }

        //auto scene_manual
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_MANUAL, &manual, sizeof(manual));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }

        //auto scene param update
        update = M12MO_I2C_VAL_ATSCENE_PARAM_UPDATE;
        rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_PARAM_UPDATE, &update, sizeof(update));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
            rc = 0;
        }
        g_m12mo_scene_manual_now = manual;
    }
    
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}
static int32_t m12mo_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t value;

    CDBG("%s:E contrast %d\n", __func__, cdata->cfg.contrast);
    switch(cdata->cfg.contrast){
    case 0:
        value = 0;
        break;
    case 1:
        value = 2;
        break;
    case 2:
        value = 4;
        break;
    case 3:
        value = 6;
        break;
    case 4:
        value = 8;
        break;
    default:
        pr_err("%s:X %d invalid contrast value\n", __func__, __LINE__);
#if 0
        return -EINVAL;
#else
        value = 4; //provisional
        break;
#endif
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_CONTRAST_CTL, &value, sizeof(value));
    if (rc<0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }

    CDBG("%s:X set value %d\n", __func__, value);
    return rc;
}
static int32_t m12mo_set_zoom(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t value;


    CDBG("%s:E val:%d\n", __func__, cdata->cfg.zoom);

    if((cdata->cfg.zoom > M12MO_ZOOM_MAX_LEVEL)||(cdata->cfg.zoom < M12MO_ZOOM_MIN_LEVEL)){
      pr_err("%s:X %d invalid value\n", __func__, __LINE__);
      return  -EINVAL;
    }
    if(cdata->cfg.zoom == g_m12mo_zoom_position){
      CDBG("%s:X %d zoom position has not been changed\n", __func__, __LINE__);
      return 0;
    }
    value = (uint8_t)(cdata->cfg.zoom + 1);

    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ZOOM, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        return rc;
    } else {
        rc = 0;
    }
    g_m12mo_zoom_position = value-1; //zoom position(0-60)

    // set roi
    if(g_m12mo_fd != 0) {
        if(g_m12mo_last_face_roi_info.is_valid){
            rc = m12mo_set_af_roi_effect(s_ctrl, &g_m12mo_last_face_roi_info.roi, 1);
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            }
        }
    } else {
        if(g_m12mo_last_af_roi_info.is_valid){
            rc = m12mo_set_af_roi_effect(s_ctrl, &g_m12mo_last_af_roi_info.roi, 0);
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            }
        }
        if(g_m12mo_last_aec_roi_info.is_valid){
            rc = m12mo_set_aec_roi_effect(s_ctrl, &g_m12mo_last_aec_roi_info.roi);
            if (rc < 0) {
                pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
                return rc;
            }
        }
    }
    CDBG("%s:X \n", __func__);
    return 0;
}

static int32_t m12mo_set_stabilization(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t antihandshake =0;
    CDBG("%s:E stabilization %d\n", __func__, cdata->cfg.stabilization);

    switch(cdata->cfg.stabilization){
    case CAMERA_STABILIZATION_MODE_OFF:
        antihandshake = M12MO_I2C_VAL_AHS_MON_OFF;
        break;
    case CAMERA_STABILIZATION_MODE_ON:
        antihandshake = M12MO_I2C_VAL_AHS_MON_ON;
        break;
    default:
        pr_err("%s:X %d invalid value.\n", __func__, __LINE__);
        return -EINVAL;
    }

    rc = m12mo_i2c_write_category_param(s_ctrl,
            M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_AHS_MON, &antihandshake, sizeof(antihandshake));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }
    CDBG("%s:X antihandshake=%d\n", __func__, antihandshake);
    return rc;
}

static int32_t m12mo_set_jpeg_quality_ex(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    uint32_t quality;
    uint8_t ratio;

    CDBG("%s:E\n", __func__);

    if (copy_from_user(&quality, (void *)(cdata->cfg.setting), sizeof(quality))) {
        pr_err("%s:X %d failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    if (quality < 1 || 100 < quality) {
        pr_err("%s:X out of range. quality=%d\n", __func__, quality);
        return -EFAULT;
    }
    ratio = (uint8_t)quality;

    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_JPEG_RATIO,
        &ratio, sizeof(ratio));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0; /* return success */
    }

    CDBG("%s:X ratio=%d, rc=%d\n", __func__, ratio, rc);
    return rc;
}

static int32_t m12mo_set_fd(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    int32_t fd;

    CDBG("%s:E\n", __func__);

    if (copy_from_user(&fd, (void *)(cdata->cfg.setting), sizeof(fd))) {
        pr_err("%s:X %d failed\n", __func__, __LINE__);
        return -EFAULT;
    }

    g_m12mo_fd = fd;

    if(!g_m12mo_fd) {
        uint8_t area = 0x00; // Center
        rc = m12mo_i2c_write_category_param(s_ctrl, M12MO_I2C_CAT_LENS,
                                            M12MO_I2C_CAT_LENS_AF_AREA_MODE,
                                            &area,
                                            sizeof(area));
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        } else {
        rc = 0;
        }
    }

    CDBG("%s:X fd=%d rc=%d\n", __func__, g_m12mo_fd, rc);
    return rc;
}

static int32_t m12mo_get_makernote_ispdata_ex(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    uint8_t addr8[4];
    uint32_t address;
    int32_t size = cdata->cfg.isp_info.size;
    uint8_t *dst = cdata->cfg.isp_info.data;
    uint8_t *info;
    int32_t info_size = M12MO_MAKERNOTE_ISP_DATA_SIZE;

    CDBG("%s:E size=%d.\n", __func__, size);
    if (size == 0 || !dst) {
        pr_err("%s:X %d failed: size=%d data=%p\n", __func__, __LINE__, size, dst);
        return -EINVAL;
    }

    /* isp information data read */
    rc = m12mo_i2c_read_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_CAP_DEBUG_DATA_ADD,
        addr8, sizeof(addr8));
    if (rc != sizeof(addr8)) {
        pr_err("%s:X isp information data address read failed. rc=%d\n", __func__, rc);
        return rc;
    }
    CDBG("%s: CAP_DEBUG_DATA_ADD: 0x%02x%02x%02x%02x\n", __func__,
            addr8[0], addr8[1], addr8[2], addr8[3]);

    /* memory allocate */
    info = kzalloc(info_size, GFP_KERNEL);
    if (!info) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        return -ENOMEM;
    }

    address = ((uint32_t)addr8[0] << 24) | ((uint32_t)addr8[1] << 16)
            | ((uint32_t)addr8[2] << 8) | ((uint32_t)addr8[3]);
    rc = m12mo_i2c_read_memory_byte(s_ctrl, address, info, info_size);
    if (rc < 0) {
        pr_err("%s:X isp information data read failed. address=0x%08x rc=%d\n",
            __func__, address, rc);
        kfree(info);
        return rc;
    } else {
        rc = 0; /* return success */
    }
    /* output isp information data read value (160 Byte head) */
    CDBG("%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n",
        __func__, info[  0], info[  1], info[  2], info[  3], info[  4], info[  5], info[  6], info[  7],
                  info[  8], info[  9], info[ 10], info[ 11], info[ 12], info[ 13], info[ 14], info[ 15],
        __func__, info[ 16], info[ 17], info[ 18], info[ 19], info[ 20], info[ 21], info[ 22], info[ 23],
                  info[ 24], info[ 25], info[ 26], info[ 27], info[ 28], info[ 29], info[ 30], info[ 31],
        __func__, info[ 32], info[ 33], info[ 34], info[ 35], info[ 36], info[ 37], info[ 38], info[ 39],
                  info[ 40], info[ 41], info[ 42], info[ 43], info[ 44], info[ 45], info[ 46], info[ 47],
        __func__, info[ 48], info[ 49], info[ 50], info[ 51], info[ 52], info[ 53], info[ 54], info[ 55],
                  info[ 56], info[ 57], info[ 58], info[ 59], info[ 60], info[ 61], info[ 62], info[ 63],
        __func__, info[ 64], info[ 65], info[ 66], info[ 67], info[ 68], info[ 69], info[ 70], info[ 71],
                  info[ 72], info[ 73], info[ 74], info[ 75], info[ 76], info[ 77], info[ 78], info[ 79],
        __func__, info[ 80], info[ 81], info[ 82], info[ 83], info[ 84], info[ 85], info[ 86], info[ 87],
                  info[ 88], info[ 89], info[ 90], info[ 91], info[ 92], info[ 93], info[ 94], info[ 95],
        __func__, info[ 96], info[ 97], info[ 98], info[ 99], info[100], info[101], info[102], info[103],
                  info[104], info[105], info[106], info[107], info[108], info[109], info[110], info[111],
        __func__, info[112], info[113], info[114], info[115], info[116], info[117], info[118], info[119],
                  info[120], info[121], info[122], info[123], info[124], info[125], info[126], info[127],
        __func__, info[128], info[129], info[130], info[131], info[132], info[133], info[134], info[135],
                  info[136], info[137], info[138], info[139], info[140], info[141], info[142], info[143],
        __func__, info[144], info[145], info[146], info[147], info[148], info[149], info[150], info[151],
                  info[152], info[153], info[154], info[155], info[156], info[157], info[158], info[159]
    );  /*  output max 10 line. */

    size = ( size > info_size ? info_size : size);
    if (copy_to_user((void*)dst, (void *)info, size)) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }

    /* memory free */
    kfree(info);

    /* return value */
    CDBG("%s:X rc=%d.\n", __func__, rc);
    return rc;
}

int32_t m12mo_lens_move_inf(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc;
    const uint8_t range = 0x05; /* Initial DAC position (for Power-Off) */

    CDBG("%s:E range:0x%02x\n", __func__, range);

    /* move lens to DAC position */
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_LENS, M12MO_I2C_CAT_LENS_AF_RANGE, &range, sizeof(range));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0; /* return success */
    }
    /* wait for move time */
    msleep(200);

    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_get_exif_info_ex(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc = 0;
    struct sensor_exif_info_ex_t info;

    memset(&info, 0, sizeof(struct sensor_exif_info_ex_t));

    if ( (s_ctrl->camera_stream_type == MSM_CAMERA_STREAM_VIDEO)                /* Video (Live Shot) */
        || (s_ctrl->camera_stream_type == MSM_CAMERA_STREAM_PREVIEW_SNAPSHOT)   /* ZSL */
        || g_m12mo_continuous_shoot_flg                                         /* Continuous Shot */
    ) {
        /* get from preview stream status */
        CDBG("%s:E preview stream status.\n", __func__);

        /* iso speed */
        {
        uint8_t iso_speed[2];

        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_NOW_ISO, iso_speed, sizeof(iso_speed));
        if (rc != sizeof(iso_speed)) {
            pr_err("%s:X now iso speed read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }

        info.iso_speed = ( ((uint16_t)iso_speed[0] << 8) | iso_speed[1] );
        CDBG("%s: NOW_ISO         : %d(0x%02x%02x)\n", __func__,
            info.iso_speed, iso_speed[0], iso_speed[1]);
        }

        /* exposure time */
        {
        uint8_t exp_time[2];
        uint32_t exposure;

        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_NOW_EXPOSURE, exp_time, sizeof(exp_time));
        if (rc != sizeof(exp_time)) {
            pr_err("%s:X now exposure time read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }

        exposure = ( ((uint32_t)exp_time[0] << 8) | exp_time[1] ); /* exposure time is 1/20000s unit */
        info.exposure_time = exposure * 50;                        /* response time is 1/1000000s unit */
        CDBG("%s: NOW_EXPOSURE    : %d(0x%02x%02x)\n", __func__,
            exposure, exp_time[0], exp_time[1]);
        }
    } else {
        /* get from snapshot time status */
        int64_t  num;
        uint32_t deno;
        int32_t  unit;

        CDBG("%s:E snapshot time status.\n", __func__);

        /* exposure bias */
        {
        uint8_t exp_bias[8];

        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_EXIF, M12MO_I2C_CAT_EXIF_INFO_EBV, exp_bias, sizeof(exp_bias));
        if (rc != sizeof(exp_bias)) {
            pr_err("%s:X exif exposure bias read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }
        num  = (int32_t)
               ( ((int32_t)exp_bias[0] << 24) | ((int32_t)exp_bias[1] << 16)
                |((int32_t)exp_bias[2] <<  8) | ((int32_t)exp_bias[3] <<  0) );
        deno = ( ((uint32_t)exp_bias[4] << 24) | ((uint32_t)exp_bias[5] << 16)
                |((uint32_t)exp_bias[6] <<  8) | ((uint32_t)exp_bias[7] <<  0) );
        unit = 100;

        CDBG("%s: INFO_EBV        : num=%lld(0x%02x%02x%02x%02x), deno=%u(0x%02x%02x%02x%02x)\n", __func__,
            num, exp_bias[0], exp_bias[1], exp_bias[2], exp_bias[3],
            deno, exp_bias[4], exp_bias[5], exp_bias[6], exp_bias[7]);
        if (deno == 0) {
            CDBG("%s: division zero occurred. num=%lld, deno=%u, unit=%d\n",
                __func__, num, deno, unit);
            num = 0;
        } else {
            num *= unit;
            do_div(num, deno);
        }
        info.exposure_bias = (uint32_t)num;
        }

        /* flash information */
        {
        uint8_t info_flash[2];

        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_EXIF, M12MO_I2C_CAT_EXIF_INFO_FLASH,
            info_flash, sizeof(info_flash));
        if (rc != sizeof(info_flash)) {
            pr_err("%s:X flash information read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }

        switch( ((int32_t)info_flash[0] << 8) | info_flash[1] )
        {
        case M12MO_I2C_VAL_FLASH_FIRED:
        case M12MO_I2C_VAL_FLASH_AUTO_FIRED:
        case M12MO_I2C_VAL_FLASH_FIRED_REDEYE:
        case M12MO_I2C_VAL_FLASH_AUTO_FIRED_REDEYE:
            info.is_flash_fired = 1;
            break;
        default:
            info.is_flash_fired = 0;
            break;
        }
        CDBG("%s: INFO_FLASH      : 0x%02x%02x\n", __func__,
            info_flash[0], info_flash[1]);
        }

        /* iso speed */
        {
        uint8_t iso_speed[2];

        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_EXIF, M12MO_I2C_CAT_EXIF_INFO_ISO, iso_speed, sizeof(iso_speed));
        if (rc != sizeof(iso_speed)) {
            pr_err("%s:X exif iso speed read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }

        info.iso_speed = ( ((uint16_t)iso_speed[0] << 8) | iso_speed[1] );
        CDBG("%s: INFO_ISO        : %d(0x%02x%02x)\n", __func__,
            info.iso_speed, iso_speed[0], iso_speed[1]);
        }

        /* exposure time */
        {
        uint8_t exp_time[8];

        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_EXIF, M12MO_I2C_CAT_EXIF_INFO_EXPTIME, exp_time, sizeof(exp_time));
        if (rc != sizeof(exp_time)) {
            pr_err("%s:X exif exposure bias read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }
        num  = ( ((uint32_t)exp_time[0] << 24) | ((uint32_t)exp_time[1] << 16)
                |((uint32_t)exp_time[2] <<  8) | ((uint32_t)exp_time[3] <<  0) );
        deno = ( ((uint32_t)exp_time[4] << 24) | ((uint32_t)exp_time[5] << 16)
                |((uint32_t)exp_time[6] <<  8) | ((uint32_t)exp_time[7] <<  0) );
        unit = 1000000;

        CDBG("%s: INFO_EXPTIME    : num=%lld(0x%02x%02x%02x%02x), deno=%u(0x%02x%02x%02x%02x)\n", __func__,
            num, exp_time[0], exp_time[1], exp_time[2], exp_time[3],
            deno, exp_time[4], exp_time[5], exp_time[6], exp_time[7]);
        if (deno == 0) {
            CDBG("%s: division zero occurred. num=%lld, deno=%u, unit=%d\n",
                __func__, num, deno, unit);
            num = 0;
        } else {
            num *= unit;
            do_div(num, deno);
        }
        info.exposure_time = (uint32_t)num;
        }
    }

    /* metering mode */
    {
    uint8_t mode;
    int face_detect = (g_m12mo_fd && g_m12mo_fd_detecting_flg);

    /* face detecting status is top priority */
    if (face_detect) {
        mode = M12MO_I2C_VAL_AE_MODE_ROI;
    } else {
        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_MODE, &mode, sizeof(mode));
        if (rc != sizeof(mode)) {
            pr_err("%s:X ae metering mode setting read failed. rc=%d\n", __func__, rc);
            return rc;
        } else {
            rc = 0; /* return success. */
        }
    }

    switch (mode) {
    case M12MO_I2C_VAL_AE_MODE_ALL:
        info.metering_mode = 1;    /* Average */
        break;
    case M12MO_I2C_VAL_AE_MODE_CENTER:
        info.metering_mode = 2;    /* Center */
        break;
    case M12MO_I2C_VAL_AE_MODE_ROI:
        info.metering_mode = 3;    /* Spot */
        break;
    default:
        info.metering_mode = 0;    /* N/A */
        break;
    }
    CDBG("%s: AE_MODE         : 0x%02x (FaceDetect= %d)\n", __func__, mode, face_detect);
    }

    /* get yuv thumbnail resolution */
    {
    int i, len;
    const m12mo_rect_value_t *p;
    uint8_t thum_img_size;

    rc = m12mo_i2c_read_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_PRM, M12MO_I2C_CAT_CAP_PRM_THUMB_IMAGE_SIZE,
        &thum_img_size, sizeof(thum_img_size));
    if (rc != sizeof(thum_img_size)) {
        pr_err("%s:X thumb image size read failed. rc=%d\n", __func__, rc);
        return rc;
    } else {
        rc = 0; /* return success. */
    }

    p = m12mo_thumb_image_size;
    len = ARRAY_SIZE(m12mo_thumb_image_size);
    for (i = 0; i < len; i++) {
        if (thum_img_size == p->set) {
            info.yuv_width = p->width;
            info.yuv_height = p->height;
            break;
        }
        p++;
    }
    if (i >= len) {
        pr_warn("%s: setting size not found. response 0x0\n", __func__);
    }
    CDBG("%s: THUMB_IMAGE_SIZE: 0x%02x(%dx%d)\n", __func__,
        thum_img_size, info.yuv_width, info.yuv_height);
    }

    /* get jpeg picture length */
    {
    uint8_t jpeg_img_size[4];

    rc = m12mo_i2c_read_category_param(s_ctrl,
        M12MO_I2C_CAT_CAP_CTL, M12MO_I2C_CAT_CAP_CTL_IMAGE_SIZE,
        jpeg_img_size, sizeof(jpeg_img_size));
    if (rc != sizeof(jpeg_img_size)) {
        pr_err("%s:X jpeg image size read failed. rc=%d\n", __func__, rc);
        return rc;
    } else {
        rc = 0; /* return success. */
    }

    info.jpeg_size = ((int32_t)jpeg_img_size[0] << 24) | ((int32_t)jpeg_img_size[1] << 16)
                    |((int32_t)jpeg_img_size[2] <<  8) | ((int32_t)jpeg_img_size[3] <<  0);
    CDBG("%s: IMAGE_SIZE      : %d(0x%02x%02x%02x%02x)\n", __func__,
        info.jpeg_size,
        jpeg_img_size[0], jpeg_img_size[1], jpeg_img_size[2], jpeg_img_size[3]);
    }

    /* get flash wait time */
    if (info.is_flash_fired) {
        info.flash_wait_time = 2900;    /* TBD 2900ms */
    }
    CDBG("%s: FLASH_WAIT      : %d\n", __func__, info.flash_wait_time);


    /* copy to user land */
    if (copy_to_user(cdata->cfg.setting, (void *)&info,
        sizeof(struct sensor_exif_info_ex_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }

    CDBG("%s:X ebv=%d, flash=%d, iso=%d, expt=%d, aemode=%d, yuv=%dx%d, jpeg=%d, flashwait=%d\n",
            __func__,
            info.exposure_bias, info.is_flash_fired, info.iso_speed,  info.exposure_time,
            info.metering_mode, info.yuv_width,      info.yuv_height, info.jpeg_size,
            info.flash_wait_time);
    return rc;
}

static int32_t m12mo_read_otp(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc;
    const uint32_t addr = MSM_CAMERA_OTP_ADDR;
    const int32_t size = MSM_CAMERA_OTP_BYTE;
    uint8_t otp_data[size];

    CDBG("%s:E Addr=0x%02x, Size=%d\n", __func__, addr, size);

    /* otp data read */
    rc = m12mo_i2c_read_memory_byte(s_ctrl, addr, otp_data, size);
    if (rc < 0) {
        pr_err("%s:X otp data read failed. rc=%d\n", __func__, rc);
        return rc;
    }

    rc = 0; /* return success */
    if (copy_to_user(setting, (void *)otp_data, size)) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }

    /* output otp data read value */
    CDBG("%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n"
        "%s: %02x %02x %02x %02x %02x %02x %02x %02x\n",
        __func__, otp_data[ 0], otp_data[ 1], otp_data[ 2], otp_data[ 3], otp_data[ 4], otp_data[ 5], otp_data[ 6], otp_data[ 7],
                  otp_data[ 8], otp_data[ 9], otp_data[10], otp_data[11], otp_data[12], otp_data[13], otp_data[14], otp_data[15],
        __func__, otp_data[16], otp_data[17], otp_data[18], otp_data[19], otp_data[20], otp_data[21], otp_data[22], otp_data[23],
                  otp_data[24], otp_data[25], otp_data[26], otp_data[27], otp_data[28], otp_data[29], otp_data[30], otp_data[31],
        __func__, otp_data[32], otp_data[33], otp_data[34], otp_data[35], otp_data[36], otp_data[37], otp_data[38], otp_data[39]
    );  /*  output max 10 line. */

    /* return value */
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_read_pdaf_otp(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc;
    const uint32_t addr = MSM_CAMERA_PDAF_ADDR;
    const int32_t size = MSM_CAMERA_PDAF_BYTE;
    uint8_t pdaf_data[size];

    CDBG("%s:E Addr=0x%02x, Size=%d\n", __func__, addr, size);

    /* pdaf data read */
    rc = m12mo_i2c_read_memory_byte(s_ctrl, addr, pdaf_data, size);
    if (rc < 0) {
        pr_err("%s:X otp pdaf data read failed. rc=%d\n", __func__, rc);
        return rc;
    }

    rc = 0; /* return success */
    if (copy_to_user(setting, (void *)pdaf_data, size)) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }

    /* return value */
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_aec_lock(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t value=0;

    CDBG("%s:E val:%d", __func__, cdata->cfg.lock);
    switch(cdata->cfg.lock){
    case 0:
        value = M12MO_I2C_VAL_AE_LOCK_OFF;
        break;
    case 1:
        value = M12MO_I2C_VAL_AE_LOCK_ON;
        break;
    default:
        pr_err("%s:X %d invalid value\n", __func__, __LINE__);
        return -EINVAL;
    }
    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_LOCK, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:%d failed %d\n", __func__, __LINE__, rc);
        return rc;
    }
    //ae param update
    value = M12MO_I2C_VAL_AE_PARAM_UPDATE;
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_AE, M12MO_I2C_CAT_AE_AE_PARAM_UPDATE, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc=0;
    }
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_awb_lock(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t value=0;

    CDBG("%s:E val:%d", __func__, cdata->cfg.lock);
    switch(cdata->cfg.lock){
    case 0:
        value = M12MO_I2C_VAL_AWB_LOCK_OFF;
        break;
    case 1:
        value = M12MO_I2C_VAL_AWB_LOCK_ON;
        break;
    default:
        pr_err("%s:X %d invalid value\n", __func__, __LINE__);
        return -EINVAL;
    }
    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_WB, M12MO_I2C_CAT_WB_AWB_LOCK, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:%d failed %d\n", __func__, __LINE__, rc);
        return rc;
    }
    //awb param update
    value = M12MO_I2C_VAL_AWB_PARAM_UPDATE;
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_WB, M12MO_I2C_CAT_WB_AWB_PARAM_UPDATE, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc=0;
    }
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int32_t m12mo_set_asd_lock(struct msm_sensor_ctrl_t *s_ctrl, struct sensorb_cfg_data *cdata)
{
    int32_t rc=0;
    uint8_t value=0;
    CDBG("%s:E val:%d", __func__, cdata->cfg.lock);
    switch(cdata->cfg.lock){
    case 0:
        value = M12MO_I2C_VAL_ATSCENE_LOCK_OFF;
        break;
    case 1:
        value = M12MO_I2C_VAL_ATSCENE_LOCK_ON;
        break;
    default:
        pr_err("%s:X %d invalid value\n", __func__, __LINE__);
        return -EINVAL;
    }
    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_LOCK, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:%d failed %d\n", __func__, __LINE__, rc);
        return rc;
    }

    //auto scene param update
    value = M12MO_I2C_VAL_ATSCENE_PARAM_UPDATE;
    rc = m12mo_i2c_write_category_param(s_ctrl,
        M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_ATSCENE_PARAM_UPDATE, &value, sizeof(value));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    } else {
        rc = 0;
    }
    CDBG("%s:X rc=%d\n", __func__, rc);
    return rc;
}

static int m12mo_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;

    int fwdl_req = 0;
    const int ver_len = 2;
    uint8_t curr_ver[ver_len];
    uint8_t fw_ver[ver_len];
    struct msm_camera_slave_info *slave_info;
    const int chip_len = 11;
    uint8_t chipids[chip_len];
    uint8_t  reverse = 0;
    uint8_t  mirror = 0;

    CDBG("%s:E\n",__func__);

    /* get firmware version */
    m12mo_fw_get_version(&fw_ver[0], &fw_ver[1]);

    /* start camera firmware (wait power on -> start firmware -> wait fw boot)*/
    rc = m12mo_start_camera_fw(s_ctrl, &fwdl_req);
    if (rc < 0) {
        if (!fwdl_req) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        } else {
            pr_info("%s: **** REQUEST FIRMWARE DOWNLOAD - FIRMWARE IS EMPTY NOW - rc= %d ****\n",
                __func__, rc);

            /* set 0x0000 current firmware version */
            memset(curr_ver, 0x00, sizeof(curr_ver));
        }
    } else {
        /* check firmware version */
        rc = m12mo_i2c_read_category_param(s_ctrl,
            M12MO_I2C_CAT_SYS, M12MO_I2C_CAT_SYS_VER_FIRMWARE, curr_ver, ver_len);
        if (rc != ver_len) {
            pr_err("%s: Firmware version read failed. rc=%d\n", __func__, rc);
            return rc;
        }
        /* firmware update if version is not same */
        if (fw_ver[0] != curr_ver[0] || fw_ver[1] != curr_ver[1]) {
            fwdl_req = 1;
        }
    }

    /* process to firmware update if requested */
    if (fwdl_req) {
        pr_info("%s: **** START UPDATE FIRMWARE - VERSION IS NOT SAME - 0x%02x%02x -> 0x%02x%02x ****\n",
            __func__, curr_ver[0], curr_ver[1], fw_ver[0], fw_ver[1]);

        /* start firmware update */
        rc = m12mo_fwdl(s_ctrl, 0);
        if (rc < 0) {
            pr_err("%s: **** FAILED UPDATE FIRMWARE - rc=%d ****\n", __func__, rc);
            return rc;
        }
        pr_info("%s: **** COMPLETED UPDATE FIRMWARE - VERSION IS UPDATED NOW - 0x%02x%02x -> 0x%02x%02x ****\n",
            __func__, curr_ver[0], curr_ver[1], fw_ver[0], fw_ver[1]);

        /* start camera firmware (wait power on -> start firmware -> wait fw boot)*/
        rc = m12mo_start_camera_fw(s_ctrl, NULL);
        if (rc < 0) {
            pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
            return rc;
        }
    }
    CDBG("%s: success start camera firmware. version= 0x%02x%02x\n", __func__,
        fw_ver[0], fw_ver[1]);

    /* check sensor id read from sensor CUSTOMER_CODE */
    slave_info = s_ctrl->sensordata->slave_info;
    if ( !slave_info ) {
        pr_err("%s:X %d msm_camera_slave_info is null. %p \n", __func__, __LINE__, slave_info);
        return -EINVAL;
    }
    rc = m12mo_i2c_read_category_param(s_ctrl,
        slave_info->sensor_slave_addr, slave_info->sensor_id_reg_addr, chipids, chip_len);
    if (rc != chip_len) {
        pr_err("%s:X Chip ID read failed. rc=%d category=0x%02x/byte=0x%02x\n", __func__,
            rc, slave_info->sensor_slave_addr, slave_info->sensor_id_reg_addr);
        return rc;
    }
    if (chipids[0] != slave_info->sensor_id) {
        pr_err("%s:X msm_sensor_match_id chip id does not match. 0x%02x!=0x%02x\n", __func__,
            chipids[0], slave_info->sensor_id);
        return -ENODEV;
    }

   /* Flip/Mirror */
    reverse = M12MO_I2C_VAL_REVERSE_OFF;
    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_REVERSE, &reverse, sizeof(reverse));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    
    mirror = M12MO_I2C_VAL_MIRROR_OFF;
    rc = m12mo_i2c_write_category_param(s_ctrl,
         M12MO_I2C_CAT_MON, M12MO_I2C_CAT_MON_MIRROR, &mirror, sizeof(mirror));
    if (rc < 0) {
        pr_err("%s:X %d failed. rc=%d\n", __func__, __LINE__, rc);
        return rc;
    }
    
    CDBG("%s:X CUST=0x%02x, PROJ=0x%02x, FIRM=0x%02x%02x, HARD=0x%02x%02x, "
        "PARM=0x%02x%02x, AWB=0x%02x%02x, USER=0x%02x\n", __func__,
        chipids[0], chipids[1], chipids[2], chipids[3], chipids[4], chipids[5],
        chipids[6], chipids[7], chipids[8], chipids[9], chipids[10]);
    return 0;
}

static int32_t m12mo_platform_probe(struct platform_device *pdev)
{
    int32_t rc;
    const struct of_device_id *match;
    match = of_match_device(m12mo_dt_match, &pdev->dev);
    if (!match)
        return -EFAULT;
    rc = msm_sensor_platform_probe(pdev, match->data);
    return rc;
}

static int __init m12mo_init_module(void)
{
    int32_t rc;
    CDBG("%s:%d\n", __func__, __LINE__);
    rc = i2c_add_driver(&m12mo_i2c_driver);
    if (!rc)
        return rc;
    pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
    return platform_driver_probe(&m12mo_platform_driver, m12mo_platform_probe);
}

static void __exit m12mo_exit_module(void)
{
    CDBG("%s:%d\n", __func__, __LINE__);
    if (m12mo_s_ctrl.pdev) {
        msm_sensor_free_sensor_data(&m12mo_s_ctrl);
        platform_driver_unregister(&m12mo_platform_driver);
    } else {
        i2c_del_driver(&m12mo_i2c_driver);
        free_irq(m12mo_s_ctrl.sensor_i2c_client->client->irq,
            m12mo_irqid);
    }
    return;
}

int32_t m12mo_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
    void __user *argp)
{
    struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
    long rc = 0;
    int32_t i = 0;
    mutex_lock(s_ctrl->msm_sensor_mutex);
    CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
        s_ctrl->sensordata->sensor_name, cdata->cfgtype);
    switch (cdata->cfgtype) {
    case CFG_GET_SENSOR_INFO:
        memcpy(cdata->cfg.sensor_info.sensor_name,
            s_ctrl->sensordata->sensor_name,
            sizeof(cdata->cfg.sensor_info.sensor_name));
        cdata->cfg.sensor_info.session_id =
            s_ctrl->sensordata->sensor_info->session_id;
        for (i = 0; i < SUB_MODULE_MAX; i++) {
            cdata->cfg.sensor_info.subdev_id[i] =
                s_ctrl->sensordata->sensor_info->subdev_id[i];
            cdata->cfg.sensor_info.subdev_intf[i] =
                s_ctrl->sensordata->sensor_info->subdev_intf[i];
        }
        cdata->cfg.sensor_info.is_mount_angle_valid =
            s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
        cdata->cfg.sensor_info.sensor_mount_angle =
            s_ctrl->sensordata->sensor_info->sensor_mount_angle;
        cdata->cfg.sensor_info.position =
            s_ctrl->sensordata->sensor_info->position;
        cdata->cfg.sensor_info.modes_supported =
            s_ctrl->sensordata->sensor_info->modes_supported;
        CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
            cdata->cfg.sensor_info.sensor_name);
        CDBG("%s:%d session id %d\n", __func__, __LINE__,
            cdata->cfg.sensor_info.session_id);
        for (i = 0; i < SUB_MODULE_MAX; i++)
            CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
                cdata->cfg.sensor_info.subdev_id[i]);
        CDBG("%s:%d mount angle valid %d value %d\n", __func__,
            __LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
            cdata->cfg.sensor_info.sensor_mount_angle);

        break;
    case CFG_SET_INIT_SETTING:
        CDBG("%s:%d CFG_SET_INIT_SETTING do nothing.\n", __func__, __LINE__);
        break;
    case CFG_SET_RESOLUTION: {
        rc = m12mo_set_resolution(s_ctrl, cdata);
        break;
    }
    case CFG_SET_STOP_STREAM:
        rc = m12mo_stop_stream(s_ctrl);
        break;
    case CFG_SET_START_STREAM:
        rc = m12mo_start_stream(s_ctrl);
        break;
    case CFG_GET_SENSOR_INIT_PARAMS:
        cdata->cfg.sensor_init_params.modes_supported =
            s_ctrl->sensordata->sensor_info->modes_supported;
        cdata->cfg.sensor_init_params.position =
            s_ctrl->sensordata->sensor_info->position;
        cdata->cfg.sensor_init_params.sensor_mount_angle =
            s_ctrl->sensordata->sensor_info->sensor_mount_angle;
        CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
            __LINE__,
            cdata->cfg.sensor_init_params.modes_supported,
            cdata->cfg.sensor_init_params.position,
            cdata->cfg.sensor_init_params.sensor_mount_angle);
        break;
    case CFG_SET_SLAVE_INFO: {
        struct msm_camera_sensor_slave_info *sensor_slave_info;
        struct msm_camera_power_ctrl_t *p_ctrl;
        uint16_t size;
        int slave_index = 0;

        sensor_slave_info = kmalloc(sizeof(struct msm_camera_sensor_slave_info), GFP_KERNEL);
        if (!sensor_slave_info) {
            pr_err("%s: failed to alloc mem\n", __func__);
            rc = -ENOMEM;
            break;
        }
        if (copy_from_user(sensor_slave_info,
            (void *)cdata->cfg.setting,
            sizeof(struct msm_camera_sensor_slave_info))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            kfree(sensor_slave_info);
            rc = -EFAULT;
            break;
        }
        /* Update sensor slave address */
        if (sensor_slave_info->slave_addr) {
            s_ctrl->sensor_i2c_client->cci_client->sid =
                sensor_slave_info->slave_addr >> 1;
        }

        /* Update sensor address type */
        s_ctrl->sensor_i2c_client->addr_type =
            sensor_slave_info->addr_type;

        /* Update power up / down sequence */
        p_ctrl = &s_ctrl->sensordata->power_info;
        size = sensor_slave_info->power_setting_array.size;
        if (p_ctrl->power_setting_size < size) {
            struct msm_sensor_power_setting *tmp;
            tmp = kmalloc(sizeof(struct msm_sensor_power_setting)
                      * size, GFP_KERNEL);
            if (!tmp) {
                pr_err("%s: failed to alloc mem\n", __func__);
                kfree(sensor_slave_info);
                rc = -ENOMEM;
                break;
            }
            kfree(p_ctrl->power_setting);
            p_ctrl->power_setting = tmp;
        }
        p_ctrl->power_setting_size = size;

        rc = copy_from_user(p_ctrl->power_setting, (void *)
            sensor_slave_info->power_setting_array.power_setting,
            size * sizeof(struct msm_sensor_power_setting));
        if (rc) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            kfree(sensor_slave_info);
            rc = -EFAULT;
            break;
        }

        CDBG("%s sensor id %x\n", __func__,
            sensor_slave_info->slave_addr);
        CDBG("%s sensor addr type %d\n", __func__,
            sensor_slave_info->addr_type);
        CDBG("%s sensor reg %x\n", __func__,
            sensor_slave_info->sensor_id_info.sensor_id_reg_addr);
        CDBG("%s sensor id %x\n", __func__,
            sensor_slave_info->sensor_id_info.sensor_id);
        for (slave_index = 0; slave_index <
            p_ctrl->power_setting_size; slave_index++) {
            CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
                slave_index,
                p_ctrl->power_setting[slave_index].seq_type,
                p_ctrl->power_setting[slave_index].seq_val,
                p_ctrl->power_setting[slave_index].config_val,
                p_ctrl->power_setting[slave_index].delay);
        }
        kfree(sensor_slave_info);
        break;
    }
    case CFG_WRITE_I2C_ARRAY: {
        struct msm_camera_i2c_reg_setting conf_array;
        struct msm_camera_i2c_reg_array *reg_setting = NULL;

        if (copy_from_user(&conf_array,
            (void *)cdata->cfg.setting,
            sizeof(struct msm_camera_i2c_reg_setting))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -EFAULT;
            break;
        }

        reg_setting = kzalloc(conf_array.size *
            (sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
        if (!reg_setting) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -ENOMEM;
            break;
        }
        if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
            conf_array.size *
            sizeof(struct msm_camera_i2c_reg_array))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            kfree(reg_setting);
            rc = -EFAULT;
            break;
        }

        conf_array.reg_setting = reg_setting;
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
            s_ctrl->sensor_i2c_client, &conf_array);
        kfree(reg_setting);
        break;
    }
    case CFG_WRITE_I2C_SEQ_ARRAY: {
        struct msm_camera_i2c_seq_reg_setting conf_array;
        struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

        if (copy_from_user(&conf_array,
            (void *)cdata->cfg.setting,
            sizeof(struct msm_camera_i2c_seq_reg_setting))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -EFAULT;
            break;
        }

        reg_setting = kzalloc(conf_array.size *
            (sizeof(struct msm_camera_i2c_seq_reg_array)),
            GFP_KERNEL);
        if (!reg_setting) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -ENOMEM;
            break;
        }
        if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
            conf_array.size *
            sizeof(struct msm_camera_i2c_seq_reg_array))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            kfree(reg_setting);
            rc = -EFAULT;
            break;
        }

        conf_array.reg_setting = reg_setting;
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
            i2c_write_seq_table(s_ctrl->sensor_i2c_client,
            &conf_array);
        kfree(reg_setting);
        break;
    }

    case CFG_POWER_UP:
        mutex_lock(&m12mo_power_lock);
        if (s_ctrl->func_tbl->sensor_power_up){
            rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
        }
        else
            rc = -EFAULT;
        g_m12mo_power_flag = 1;
        rc = m12mo_power_up_init(s_ctrl, cdata);
        mutex_unlock(&m12mo_power_lock);
        break;

    case CFG_POWER_DOWN:
        mutex_lock(&m12mo_power_lock);
        m12mo_power_up_deinit(s_ctrl);
        if (s_ctrl->func_tbl->sensor_power_down){
            rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
        }
        else
            rc = -EFAULT;
        g_m12mo_power_flag = 0;
        mutex_unlock(&m12mo_power_lock);
        break;

    case CFG_SET_STOP_STREAM_SETTING: {
        struct msm_camera_i2c_reg_setting *stop_setting =
            &s_ctrl->stop_setting;
        struct msm_camera_i2c_reg_array *reg_setting = NULL;
        if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
            sizeof(struct msm_camera_i2c_reg_setting))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -EFAULT;
            break;
        }

        reg_setting = stop_setting->reg_setting;
        stop_setting->reg_setting = kzalloc(stop_setting->size *
            (sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
        if (!stop_setting->reg_setting) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -ENOMEM;
            break;
        }
        if (copy_from_user(stop_setting->reg_setting,
            (void *)reg_setting, stop_setting->size *
            sizeof(struct msm_camera_i2c_reg_array))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            kfree(stop_setting->reg_setting);
            stop_setting->reg_setting = NULL;
            stop_setting->size = 0;
            rc = -EFAULT;
            break;
        }
        break;
    }
    case CFG_SET_STREAM_TYPE:
        rc = m12mo_set_stream_type(s_ctrl, cdata);
        break;
    case CFG_SET_FPS:
        rc = m12mo_set_fps(s_ctrl, cdata);
        break;
    case CFG_SET_WDR_EX:
        rc = m12mo_set_wdr_ex(s_ctrl, cdata);
        break;
    case CFG_SET_CONTINUOUS_SHOOT_EX:
        rc = m12mo_set_continuous_shoot_ex(s_ctrl, cdata);
        break;
    case CFG_SET_FACE_ROI:
        rc = m12mo_set_face_roi(s_ctrl, cdata);
        break;
    case CFG_SET_AUTOFOCUS: {
        m12mo_set_autofocus(s_ctrl);
        }
        break;
    case CFG_CANCEL_AUTOFOCUS: {
        m12mo_cancel_autofocus(s_ctrl);
        }
        break;
    case CFG_SET_FOCUS_MODE: {
        struct sensor_af_info_t *af_info;
        af_info = (struct sensor_af_info_t *)&cdata->cfg.af_info;
        CDBG("%s:CFG_SET_FOCUS_MODE %d\n", __func__, af_info->mode);
        rc = m12mo_set_focus_mode(s_ctrl, af_info);
        }
        break;
    case CFG_SET_AF_ROI:
        rc = m12mo_set_af_roi(s_ctrl, cdata);
        break;
    case CFG_SET_LED:
        rc = m12mo_set_led(s_ctrl, cdata);
        break;
    case CFG_GET_LOWLIGHT_INTENSITY:
        rc = m12mo_get_lowlight_intensity(s_ctrl, cdata);
        break;
    case CFG_SET_ISO: {
        rc = m12mo_set_iso(s_ctrl, cdata);
        break;
    }
    case CFG_SET_EXPOSURE_COMPENSATION: {
        int32_t ev;
        if (copy_from_user(&ev, (void *)cdata->cfg.setting, sizeof(ev))) {
            pr_err("%s:%d read failed\n", __func__, __LINE__);
            return -EFAULT;
        }
        rc = m12mo_set_exposure_compensation(s_ctrl, ev);
        break;
    }
    case CFG_SET_AE_MODE: {
        rc = m12mo_set_ae_mode(s_ctrl, cdata);
        break;
    }
    case CFG_SET_WHITE_BALANCE:
        rc = m12mo_set_white_balance(s_ctrl, cdata);
        break;
    case CFG_SET_EFFECT:
        rc = m12mo_set_effect(s_ctrl, cdata);
        break;
    case CFG_SET_ANTIBANDING:
        rc = m12mo_set_antibanding(s_ctrl, cdata);
        break;
    case CFG_SET_CONTRAST:
        rc = m12mo_set_contrast(s_ctrl, cdata);
        break;
    case CFG_SET_AEC_LOCK:
        rc = m12mo_set_aec_lock(s_ctrl, cdata);
        break;
    case CFG_SET_AWB_LOCK:
        rc = m12mo_set_awb_lock(s_ctrl, cdata);
        break;
    case CFG_SET_ASD_LOCK:
        rc = m12mo_set_asd_lock(s_ctrl, cdata);
        break;
    case CFG_SET_ZOOM:
        rc = m12mo_set_zoom(s_ctrl, cdata);
        break;
    case CFG_SET_STABILIZATION_REDUCT:
        rc = m12mo_set_stabilization(s_ctrl, cdata);
        break;
    case CFG_SET_BESTSHOT:
        rc = m12mo_set_bestshot(s_ctrl, cdata);
        break;
    case CFG_GET_AUTO_SCENE:
        rc = m12mo_get_auto_scene(s_ctrl, cdata);
        break;
    case CFG_SET_UPDATE_SCENE:
        rc = m12mo_set_update_scene(s_ctrl, cdata);
        break;
    case CFG_SET_JPEG_QUALITY_EX:
        rc = m12mo_set_jpeg_quality_ex(s_ctrl, cdata);
        break;
    case CFG_SET_FD:
        rc = m12mo_set_fd(s_ctrl, cdata);
        break;
    case CFG_SET_AEC_ROI:
        rc = m12mo_set_aec_roi(s_ctrl, cdata);
        break;
    case CFG_SET_FW_UPDATE:{
        uint8_t fw_ver[2];
        pr_info("%s: **** START SET FIRMWARE ****\n", __func__);
        rc = m12mo_fw_update(cdata->cfg.setting);
        if (rc < 0) {
            pr_err("%s: **** FAILED SET FIRMWARE : rc=%ld ****\n", __func__, rc);
            break;
        }
        pr_info("%s: **** START UPDATE FIRMWARE ****\n", __func__);
        rc = m12mo_fwdl(s_ctrl, 0);
        if (rc < 0) {
            pr_err("%s: **** FAILED UPDATE FIRMWARE : rc=%ld ****\n", __func__, rc);
        }

        if (s_ctrl->sensordata->power_info.cam_pinctrl_status) {
            CDBG("%s: process power down.\n", __func__);
            msm_camera_power_down(&s_ctrl->sensordata->power_info,
                s_ctrl->sensor_device_type, s_ctrl->sensor_i2c_client);
        }
        m12mo_fw_get_version(&fw_ver[0], &fw_ver[1]);
        pr_info("%s: **** FINISH UPDATE FIRMWARE version is 0x%02x%02x - POWER OFF ****\n",
            __func__, fw_ver[0], fw_ver[1]);
        break;
    }
    case CFG_SET_FW_ERASE:
        pr_info("%s: **** START ERASE FIRMWARE ****\n", __func__);
        rc = m12mo_fwdl(s_ctrl, 1);
        if (rc < 0) {
            pr_err("%s: **** FAILED ERASE FIRMWARE : rc=%ld ****\n", __func__, rc);
        }

        if (s_ctrl->sensordata->power_info.cam_pinctrl_status) {
            CDBG("%s: process power down.\n", __func__);
            msm_camera_power_down(&s_ctrl->sensordata->power_info,
                s_ctrl->sensor_device_type, s_ctrl->sensor_i2c_client);
        }
        pr_info("%s: **** FINISH ERASE FIRMWARE - POWER OFF ****\n", __func__);
        break;
    case CFG_GET_MAKERNOTE_ISPDATA_EX:
        rc = m12mo_get_makernote_ispdata_ex(s_ctrl, cdata);
        break;
    case CFG_GET_EXIF_INFO_EX:
        rc = m12mo_get_exif_info_ex(s_ctrl, cdata);
        break;
    case CFG_READ_OTP:
        rc = m12mo_read_otp(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_READ_PDAF_OTP:
        rc = m12mo_read_pdaf_otp(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_WRITE_I2C_CATE_PARA: {
        uint8_t write_data[3];

        if (copy_from_user(write_data, (void *)cdata->cfg.setting,
            sizeof(write_data))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -EFAULT;
            break;
        }
        rc = m12mo_i2c_write_category_param(s_ctrl,
            write_data[0], write_data[1], &write_data[2], sizeof(uint8_t));
        break;
    }
    case CFG_READ_I2C_CATE_PARA: {
        uint8_t read_data[3];

        if (copy_from_user(read_data, (void *)cdata->cfg.setting,
            sizeof(read_data))) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            rc = -EFAULT;
            break;
        }
        rc = m12mo_i2c_read_category_param(s_ctrl,
            read_data[0], read_data[1], &read_data[2], sizeof(uint8_t));
        if (rc > 0) {
            if (copy_to_user((void *)cdata->cfg.setting, (void *)read_data,
                sizeof(read_data))) {
                pr_err("%s:%d copy failed\n", __func__, __LINE__);
                rc = -EFAULT;
            }
        }
        break;
    }
    default:
        break;
    }

    mutex_unlock(s_ctrl->msm_sensor_mutex);

    return rc;
}

static struct msm_sensor_fn_t m12mo_sensor_func_tbl = {
    .sensor_config = m12mo_sensor_config,
    .sensor_power_up = msm_sensor_power_up,
    .sensor_power_down = msm_sensor_power_down,
    .sensor_match_id = m12mo_sensor_match_id,
    .sensor_get_af_status = m12mo_sensor_get_af_status,
};

static struct msm_sensor_ctrl_t m12mo_s_ctrl = {
    .sensor_i2c_client = &m12mo_sensor_i2c_client,
    .power_setting_array.power_setting = m12mo_power_setting,
    .power_setting_array.size = ARRAY_SIZE(m12mo_power_setting),
    .power_setting_array.power_down_setting = m12mo_power_down_setting,
    .power_setting_array.size_down = ARRAY_SIZE(m12mo_power_down_setting),
    .msm_sensor_mutex = &m12mo_mut,
    .sensor_v4l2_subdev_info = m12mo_subdev_info,
    .sensor_v4l2_subdev_info_size = ARRAY_SIZE(m12mo_subdev_info),
    .func_tbl = &m12mo_sensor_func_tbl,
};

module_init(m12mo_init_module);
module_exit(m12mo_exit_module);
MODULE_DESCRIPTION("Sony 13M YUV sensor driver");
MODULE_LICENSE("GPL v2");
