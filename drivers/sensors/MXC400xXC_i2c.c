/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include "alps_acc.h"

#define ACCSNS_DRIVER_NAME              "mxc400xx"
#define ACCSNS_LOG_TAG                  "[mxc400xx]"
#define ACCSNS_OUTXYZ_READ_NUM          6
#define ACCSNS_3AXIS_NUM                3
#define ACCSNS_ON                       1
#define ACCSNS_OFF                      0
#define ACCSNS_G2MG_UNIT                1024
#define ACCSNS_INITIAL_DELAY            100
#define ACCSNS_INIT_CHK_RETRY_NUM       3

/* Register Name for accsns(Mxc4005XC specific) */
#define MXC4005XC_DEVID                 0x02
#define MXC4005XC_PD_OFF                0x00
#define MXC4005XC_PD_ON                 0x01
#define MXC4005XC_INT_SRC0              0x00
#define MXC4005XC_INT_CLR0              0x00
#define MXC4005XC_INT_SRC1              0x01
#define MXC4005XC_INT_CLR1              0x01
#define MXC4005XC_XOUT_U                0x03
#define MXC4005XC_XOUT_L                0x04
#define MXC4005XC_YOUT_U                0x05
#define MXC4005XC_YOUT_L                0x06
#define MXC4005XC_ZOUT_U                0x07
#define MXC4005XC_ZOUT_L                0x08
#define MXC4005XC_TOUT                  0x09
#define MXC4005XC_CONTROL               0x0D
#define MXC4005XC_DEVICEID              0x0E
#define MXC4005XC_WHOAMI                0x0F
#define MXC4005XC_FULL_SCALE_RANGE_2G   0x00
#define MXC4005XC_FULL_SCALE_RANGE_4G   0x01
#define MXC4005XC_FULL_SCALE_RANGE_8G   0x02

#define I2C_RETRIES                     5
#define GPIO_STATE_SUSPEND              0x00
#define GPIO_STATE_ACTIVE               0x01

#undef ACC_MXC400xXC_DEBUG

#ifdef ACC_MXC400xXC_DEBUG
#define ACC_LOG_IO(msg, ...) \
    pr_info("[SENSOR]"ACCSNS_LOG_TAG"[%s][IO](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ACC_LOG_DBG(msg, ...) \
    pr_info("[SENSOR]"ACCSNS_LOG_TAG"[%s][Dbg](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ACC_LOG_INFO(msg, ...) \
    pr_info("[SENSOR]"ACCSNS_LOG_TAG"[%s][Info](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ACC_LOG_ERR(msg, ...) \
    pr_err("[SENSOR]"ACCSNS_LOG_TAG"[%s][Err](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define ACC_LOG_IO(msg, ...) \
    pr_debug("[SENSOR]"ACCSNS_LOG_TAG"[%s][IO](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ACC_LOG_DBG(msg, ...) \
    pr_debug("[SENSOR]"ACCSNS_LOG_TAG"[%s][Dbg](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ACC_LOG_INFO(msg, ...) \
    pr_info("[SENSOR]"ACCSNS_LOG_TAG"[%s][Info](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define ACC_LOG_ERR(msg, ...) \
    pr_err("[SENSOR]"ACCSNS_LOG_TAG"[%s][Err](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

struct accsns_regulator_data {
    bool available;
    struct regulator* vdd_reg;
    uint32_t min_uV;
    uint32_t max_uV;
    uint32_t on_load_uA;
    uint32_t off_load_uA;
};

struct accsns_pinctrl_data {
    struct pinctrl *pinctrl;
    struct pinctrl_state *state_active;
    struct pinctrl_state *state_suspend;
};

struct accsns_ctrl {
    uint8_t fsr01;
    uint16_t sensitivity;
};

static struct accsns_ctrl accsns_ctrl_tbl[] = {
    {MXC4005XC_FULL_SCALE_RANGE_2G, 1024},     /*MXC4005XC_FULL_SCALE_RANGE_2G*/
    {MXC4005XC_FULL_SCALE_RANGE_4G, 512 },     /*MXC4005XC_FULL_SCALE_RANGE_4G*/
    {MXC4005XC_FULL_SCALE_RANGE_8G, 256 }      /*MXC4005XC_FULL_SCALE_RANGE_8G*/
};

enum {
    SNS_X = 0,
    SNS_Y,
    SNS_Z
};

static struct i2c_client *client_accsns;
static atomic_t flgEna;
static atomic_t delay;
static atomic_t flgSuspend;
static atomic_t fsr;
static atomic_t acc_available = ATOMIC_INIT(false);
static struct accsns_regulator_data reg_data;
static struct accsns_pinctrl_data pinctrl_data;
static uint32_t vsensor_18_on_waittime_us;

static int8_t accsns_sw_reset(void);
static int16_t accsns_generate_raw_val(uint8_t out_u, uint8_t out_l, int16_t sensitivity);
static int accsns_reg_set_optimum_mode_on(void);
static int accsns_reg_set_optimum_mode_off(void);
static int accsns_get_acceleration_data(int *xyz);
static int accsns_activate_mxc400xxc(int enable);
static int accsns_activate(int flgatm, int enable, int dtime);
static int8_t accsns_initialize(void);
static int accsns_parse_dt(struct i2c_client *client);
static int accsns_regulater_ope(struct i2c_client *client);
static int acc_pinctrl_init(struct i2c_client *client);


/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int accsns_i2c_read(u8 *rxData, int length)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr   = client_accsns->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = rxData,
        },
        {
            .addr   = client_accsns->addr,
            .flags  = I2C_M_RD,
            .len    = length,
            .buf    = rxData,
        },
    };

    ACC_LOG_IO("start");

    do {
        err = i2c_transfer(client_accsns->adapter,
            msgs, ARRAY_SIZE(msgs));
    } while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

    if (err != ARRAY_SIZE(msgs)) {
        ACC_LOG_ERR("read transfer error");
        err = -EIO;
    } else {
        err = 0;
    }

    ACC_LOG_IO("end [%d]", err);

    return err;
}

static int accsns_i2c_write(u8 *txData, int length)
{
    int err;
    int tries = 0;


    struct i2c_msg msgs[] = {
        {
            .addr   = client_accsns->addr,
            .flags  = 0,
            .len    = length,
            .buf    = txData,
        },
    };

    ACC_LOG_IO("start");

    do {
        err = i2c_transfer(client_accsns->adapter,
            msgs, ARRAY_SIZE(msgs));
    } while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

    if (err != ARRAY_SIZE(msgs)) {
        ACC_LOG_ERR("write transfer error");
        err = -EIO;
    } else {
        err = 0;
    }

    ACC_LOG_IO("end [%d]", err);
    return err;
}


/*--------------------------------------------------------------------------
 * accsns function
 *--------------------------------------------------------------------------*/

static int8_t accsns_sw_reset(void)
{
    int8_t ret = 0;
    uint8_t wbuf[2] = {MXC4005XC_INT_CLR1, 0x10};
    ACC_LOG_IO("start");
    ret = accsns_i2c_write(wbuf, sizeof(wbuf));
    ACC_LOG_IO("end [%d]", ret);
    return ret;
}

static int8_t accsns_set_modectrl_fsr(uint8_t in_fsr, uint8_t pd)
{
    int8_t ret = 0;
    uint8_t wbuf[2];
    uint8_t wdata = 0x00;
    uint8_t set_fsr = (in_fsr << 5) & 0x30;
    uint8_t set_pd  = pd & 0x01;

    ACC_LOG_IO("start");

    wdata = set_fsr | set_pd;
    ACC_LOG_DBG("wdata[0x%02x]", wdata);
    wbuf[0] = MXC4005XC_CONTROL;
    wbuf[1] = wdata;

   ret = accsns_i2c_write(wbuf, sizeof(wbuf));
    if(ret){
        ACC_LOG_ERR("failed to write mode control register");
        return ret;
    }
    atomic_set(&fsr, in_fsr);

    ACC_LOG_IO("end [%d]", ret);

    return ret;
}

static int16_t accsns_generate_raw_val(uint8_t out_u, uint8_t out_l, int16_t sensitivity)
{
    int16_t snsVal;
    int32_t tmp;
    ACC_LOG_IO("start");
    ACC_LOG_DBG("output_upper[0x%02x] output_lower[0x%02x] sensitivity[%d]", out_u, out_l, sensitivity);
    snsVal = (((int16_t) ((out_u << 8) | out_l)) >> 4);
    tmp = (int32_t)snsVal * ACCSNS_G2MG_UNIT;
    snsVal = (int16_t)(tmp / sensitivity);
    ACC_LOG_IO("end");
    return snsVal;
}

static int accsns_get_acceleration_data(int *xyz)
{
    int err = -1;
    u8 buf[ACCSNS_OUTXYZ_READ_NUM];
    int8_t fsr_elem = atomic_read(&fsr);

    ACC_LOG_IO("start");

    if(!atomic_read(&acc_available)){
        ACC_LOG_ERR("Accelerometer MXC4005Xc cannot use.");
        return -ENODEV;
    }

    if (atomic_read(&flgSuspend) == 1){
        ACC_LOG_ERR("skip due to device is suspend.");
        return err;
    }

    buf[0] = MXC4005XC_XOUT_U;
    err = accsns_i2c_read(buf, sizeof(buf));
    xyz[SNS_X] = accsns_generate_raw_val(buf[0], buf[1], accsns_ctrl_tbl[fsr_elem].sensitivity);
    xyz[SNS_Y] = accsns_generate_raw_val(buf[2], buf[3], accsns_ctrl_tbl[fsr_elem].sensitivity);
    xyz[SNS_Z] = accsns_generate_raw_val(buf[4], buf[5], accsns_ctrl_tbl[fsr_elem].sensitivity);

    ACC_LOG_DBG("Org: x:%d,y:%d,z:%d", xyz[SNS_X], xyz[SNS_Y], xyz[SNS_Z]);

    xyz[SNS_X] = -1 * xyz[SNS_X];
    xyz[SNS_Y] =  1 * xyz[SNS_Y];
    xyz[SNS_Z] = -1 * xyz[SNS_Z];

    ACC_LOG_DBG("x:%d,y:%d,z:%d", xyz[SNS_X], xyz[SNS_Y], xyz[SNS_Z]);

    ACC_LOG_IO("end [%d]", err);

    return err;
}

static int accsns_reg_set_optimum_mode_on(void)
{
    int err = 0;
    ACC_LOG_IO("start");
    if(reg_data.available){
    err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
    if( err < 0 ) {
        ACC_LOG_ERR("regulator_set_optimum_mode fail. err=%d", err);
        return err;
    }
    usleep_range(1000,1000);
    } else {
        ACC_LOG_DBG("Regulator control Invalid");
    }
    ACC_LOG_IO("end [%d]", err);
    return err;
}

static int accsns_reg_set_optimum_mode_off(void)
{
    int err = 0;

    ACC_LOG_IO("start");

    if(reg_data.available){
    usleep_range(1000,1000);
    err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.off_load_uA);
    if( err < 0 ) {
        ACC_LOG_ERR("regulator_set_optimum_mode fail. err=%d\n", err);
    }
    } else {
        ACC_LOG_DBG("Regulator control Invalid");
    }

    ACC_LOG_IO("end [%d]", err);

    return err;
}

static int accsns_activate_mxc400xxc(int enable)
{
    int ret = -1;
    uint8_t buf[2];

    ACC_LOG_IO("start");

    buf[0] = MXC4005XC_CONTROL;
    if (enable == ACCSNS_ON){
        accsns_set_modectrl_fsr(MXC4005XC_FULL_SCALE_RANGE_4G, MXC4005XC_PD_OFF);
        msleep(400);
    } else {
        accsns_set_modectrl_fsr(MXC4005XC_FULL_SCALE_RANGE_4G, MXC4005XC_PD_ON);
    }

    ACC_LOG_IO("end [%d]", ret);

    return ret;
}

static int accsns_activate(int flgatm, int enable, int dtime)
{
    int ret = -1;

    ACC_LOG_IO("start");

    if(!atomic_read(&acc_available)){
        ACC_LOG_ERR("Accelerometer MXC4005Xc cannot use.");
        return -ENODEV;
    }
    if (enable != 0)
        enable = ACCSNS_ON;

    if (enable == ACCSNS_ON) {
        ret = accsns_reg_set_optimum_mode_on();
        if(ret < 0){
            ACC_LOG_ERR("accsns_reg_set_optimum_mode_on fail");
            return ret;
        }
    }

    ret = accsns_activate_mxc400xxc(enable);
    if(ret < 0){
        ACC_LOG_ERR("accsns_activate_mxc400xxc fail");
        return ret;
    }

    if (flgatm) {
        atomic_set(&flgEna, enable);
        atomic_set(&delay, dtime);
    }

    if (enable == ACCSNS_OFF) {
        ret = accsns_reg_set_optimum_mode_off();
        if(ret < 0){
            ACC_LOG_ERR("accsns_reg_set_optimum_mode_on fail");
        }
    }
    ACC_LOG_IO("end");

    return ret;
}

static int8_t accsns_initialize(void)
{
    int8_t loopcnt = ACCSNS_INIT_CHK_RETRY_NUM;
    int ret = 0;
    u8  rbuf[1];

    ACC_LOG_IO("start");

    while(loopcnt > ACCSNS_INIT_CHK_RETRY_NUM){
        rbuf[0] = MXC4005XC_DEVICEID;
        if(accsns_i2c_read(rbuf, 0x01)){
            ACC_LOG_ERR("Failed to read device id. retval[%d]", ret);
            return ret;
        }
        if(rbuf[0] == MXC4005XC_DEVICEID){
            ACC_LOG_DBG("It is the correct device id. OK!!");
            break;
        }
        loopcnt--;
        if(loopcnt <= 0){
            ACC_LOG_ERR("device id error. device is broken");
            return -1;
        }
    }

    ret = accsns_sw_reset();
    if(ret){
        ACC_LOG_ERR("Failed to Software Reset. retval[%d]", ret);
        return ret;
    }
    msleep(50);

    ret = accsns_set_modectrl_fsr(MXC4005XC_FULL_SCALE_RANGE_4G, MXC4005XC_PD_ON);
    if(ret){
        ACC_LOG_ERR("Failed to set mode control. retval[%d]", ret);
    }

    ACC_LOG_IO("end [%d]", ret);

    return ret;
}

void accsns_func_register_mxc400xxc(struct acc_if_funcs *f)
{
    ACC_LOG_IO("start");
    f->activate = accsns_activate;
    f->get_data = accsns_get_acceleration_data;
    ACC_LOG_IO("end");
}

static int accsns_parse_dt(struct i2c_client *client)
{
    ACC_LOG_IO("start");
    reg_data.available = of_property_read_bool(client->dev.of_node, "is-regulator-ope-available");
    of_property_read_u32(client->dev.of_node, "acc-vdd-min-voltage", &reg_data.min_uV);
    of_property_read_u32(client->dev.of_node, "acc-vdd-max-voltage", &reg_data.max_uV);
    of_property_read_u32(client->dev.of_node, "acc-vdd-on-load-current", &reg_data.on_load_uA);
    of_property_read_u32(client->dev.of_node, "acc-vdd-off-load-current", &reg_data.off_load_uA);
    ACC_LOG_DBG("available = %d, regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d",
                reg_data.available, reg_data.min_uV, reg_data.max_uV, reg_data.on_load_uA, reg_data.off_load_uA);

    of_property_read_u32(client->dev.of_node, "sensor_18_on_wait-time-us", &vsensor_18_on_waittime_us);
    ACC_LOG_DBG("vsensor_18_on wait time(us) = %d", vsensor_18_on_waittime_us);

    ACC_LOG_IO("end");
    return 0;
}

static int accsns_regulater_ope(struct i2c_client *client)
{
    int ret = 0;

    ACC_LOG_IO("start");

    if(reg_data.available){
        reg_data.vdd_reg = regulator_get(&client->dev, "acc-vdd");
        if( IS_ERR(reg_data.vdd_reg) ) {
            ACC_LOG_ERR("failed regulator_get");
            return -EIO;
        }

        ret = regulator_set_voltage(reg_data.vdd_reg, reg_data.min_uV, reg_data.max_uV);
        if( ret ) {
            ACC_LOG_ERR("regulator_set_voltage fail. err=%d", ret);
            return -EIO;
        }

        ret = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
        if( ret < 0 ) {
            ACC_LOG_ERR("regulator_set_optimum_mode fail. err=%d", ret);
            return -EIO;
        }

        ret = regulator_enable(reg_data.vdd_reg);
        if( ret ) {
            ACC_LOG_ERR("regulator_enable fail. err=%d", ret);
            return -EIO;
        }
    } else {
        ACC_LOG_DBG("It is set not to use regulator control");
    }
    ACC_LOG_IO("end [%d]", ret);
    return ret;
}

static int acc_pinctrl_select(uint8_t gpio_state)
{
    struct pinctrl_state *pins_state;
    int ret;

    ACC_LOG_IO("start");
    pins_state = gpio_state ? pinctrl_data.state_active
                    : pinctrl_data.state_suspend;
    if (!IS_ERR_OR_NULL(pins_state)) {
        ret = pinctrl_select_state(pinctrl_data.pinctrl, pins_state);
        if (ret) {
            ACC_LOG_ERR("can not set %s pins", gpio_state ? "active" : "suspend");
            return ret;
        }
    } else
        ACC_LOG_ERR("not a valid '%s' pinstate", gpio_state ? "active" : "suspend");

    ACC_LOG_IO("end [%d]", ret);
    return 0;
}

static int acc_pinctrl_init(struct i2c_client *client)
{
    int ret = 0;
    struct device *dev = &client->dev;

    ACC_LOG_IO("start");
    pinctrl_data.pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR_OR_NULL(pinctrl_data.pinctrl)) {
        ACC_LOG_ERR("Target does not use pinctrl");
        ret = PTR_ERR(pinctrl_data.pinctrl);
        pinctrl_data.pinctrl = NULL;
        return ret;
    }

    pinctrl_data.state_active
        = pinctrl_lookup_state(pinctrl_data.pinctrl, "sns_power_active");
    if (IS_ERR_OR_NULL(pinctrl_data.state_active)) {
        ACC_LOG_ERR("Can not get active pinstate");
        ret = PTR_ERR(pinctrl_data.state_active);
        pinctrl_data.pinctrl = NULL;
        return ret;
    }

    pinctrl_data.state_suspend
        = pinctrl_lookup_state(pinctrl_data.pinctrl, "sns_power_suspend");
    if (IS_ERR_OR_NULL(pinctrl_data.state_suspend)) {
        ACC_LOG_ERR("Can not get suspend pinstate");
        ret = PTR_ERR(pinctrl_data.state_suspend);
        pinctrl_data.pinctrl = NULL;
        return ret;
    }
    ACC_LOG_IO("end [%d]", ret);

    return ret;
}

/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int accsns_suspend(struct i2c_client *client, pm_message_t mesg)
{
    ACC_LOG_IO("start");
    atomic_set(&flgSuspend, 1);
    accsns_activate(0, 0, atomic_read(&delay));
    ACC_LOG_IO("end");
    return 0;
}

static int accsns_resume(struct i2c_client *client)
{
    ACC_LOG_IO("start");
    atomic_set(&flgSuspend, 0);
    accsns_activate(0, atomic_read(&flgEna), atomic_read(&delay));
    ACC_LOG_IO("end");
    return 0;
}

/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int accsns_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int err = 0;

    ACC_LOG_IO("start");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        ACC_LOG_ERR("client not i2c capable");
        return -ENODEV;
    }

    client_accsns = client;

    atomic_set(&flgEna, 0);
    atomic_set(&delay, ACCSNS_INITIAL_DELAY);
    atomic_set(&flgSuspend, 0);

    accsns_parse_dt(client);
    err = accsns_regulater_ope(client);
    if(err){
        ACC_LOG_ERR("failed to initialize sensor in regulater operation");
        return -ENODEV;
    }

    acc_pinctrl_init(client);
    acc_pinctrl_select(GPIO_STATE_ACTIVE);
    usleep_range(vsensor_18_on_waittime_us,vsensor_18_on_waittime_us);

    if (accsns_initialize()) {
        ACC_LOG_ERR("failed to initialize sensor");
        return -EIO;
    }
    atomic_set(&acc_available, true);

    ACC_LOG_INFO("detected " ACCSNS_DRIVER_NAME " accelerometer");

    ACC_LOG_IO("end");
    return 0;
}

static int accsns_remove(struct i2c_client *client)
{
    ACC_LOG_IO("start");
    accsns_activate(0, 0, atomic_read(&delay));
    acc_pinctrl_select(GPIO_STATE_SUSPEND);
    client_accsns = NULL;
    ACC_LOG_IO("end");
    return 0;
}

/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static const struct i2c_device_id accsns_id[] = {
    { ACCSNS_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver accsns_driver = {
    .probe      = accsns_probe,
    .remove     = accsns_remove,
    .id_table   = accsns_id,
    .driver     = {
    .name       = ACCSNS_DRIVER_NAME,
    },
    .suspend    = accsns_suspend,
    .resume     = accsns_resume,
};

static int __init accsns_init(void)
{
    ACC_LOG_IO("start");
    ACC_LOG_IO("end");
    return i2c_add_driver(&accsns_driver);
}

static void __exit accsns_exit(void)
{
    ACC_LOG_IO("start");
    ACC_LOG_IO("end");
    i2c_del_driver(&accsns_driver);
}

module_init(accsns_init);
module_exit(accsns_exit);

MODULE_DESCRIPTION("Memsic Accelerometer Device");
MODULE_AUTHOR("KYOCERA Corporation");
MODULE_LICENSE("GPL v2");
