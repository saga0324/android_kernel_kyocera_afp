/*
 * This software is contributed or developed by KYOCERA Corporation.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include <linux/kc_led.h>
#include <linux/kc_board.h>
#include <linux/alarmtimer.h>

#include "leds-msm-mdm.h"

// function switch
//#define DISABLE_DISP_DETECT
//#define FORCE_WLED_ALWAYS_ON
//#define FORCE_WLED_ON
//#define KEEP_DEVICE_STATE_ON_PROBE
#define ENABLE_PWM
#define DISCARD_BL_SUBLCDPOWER_OFF

#ifdef FORCE_WLED_ALWAYS_ON
#ifndef FORCE_WLED_ON
#define FORCE_WLED_ON
#endif
#ifdef KEEP_DEVICE_STATE_ON_PROBE
#undef KEEP_DEVICE_STATE_ON_PROBE
#endif
#endif

// debug
//#define DEBUG (1)
//#define BD6098_DEBUG

#ifdef BD6098_DEBUG
#define BD6098_V_LOG(msg, ...)	\
	pr_notice("[LEDDRV][%s][V](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define BD6098_D_LOG(msg, ...)	\
	pr_notice("[LEDDRV][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define BD6098_V_LOG(msg, ...) \
	pr_debug ("[LEDDRV][%s][V](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define BD6098_D_LOG(msg, ...)	\
	pr_debug ("[LEDDRV][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define BD6098_E_LOG(msg, ...)	\
	pr_err   ("[LEDDRV][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define BD6098_N_LOG(msg, ...)	\
	pr_notice("[LEDDRV][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)


#define BD6098_DRV_NAME "leds-oem-bd6098"

#define I2C_RETRIES_NUM				(5)
#define BD6098_RESET_GPIO			(0)
#define BD6098_I2C_WRITE_MSG_NUM	(1)
#define BD6098_I2C_READ_MSG_NUM		(2)
#define BD6098_WRITE_BUF_LEN		(2)

#define BD6098_REG_MAX (30)
struct oem_bd6098_reg_info {
	uint8_t	value;
	bool	written;
};

struct oem_bd6098_reg_write_info {
	uint8_t	reg;
	uint8_t	val;
	uint8_t	mask;
};

struct oem_bd6098_reg_info oem_bd6098_reg_info_save[BD6098_REG_MAX] =
{
/*00,00h*/	{	0x00,	false },
/*01,01h*/	{	0x4E,	false },
/*02,02h*/	{	0x00,	false },
/*03,03h*/	{	0x00,	false },
/*04,04h*/	{	0x00,	false },
/*05,05h*/	{	0x00,	false },
/*06,06h*/	{	0x00,	false },
/*07,07h*/	{	0x31,	false },
/*08,08h*/	{	0xEE,	false },
/*09,09h*/	{	0x00,	false },
/*10,0Ah*/	{	0x03,	false },
/*11,0Bh*/	{	0x07,	false },
/*12,0Ch*/	{	0x07,	false },
/*13,0Dh*/	{	0x07,	false },
/*14,0Eh*/	{	0x07,	false },
/*15,0Fh*/	{	0x10,	false },
/*16,10h*/	{	0x10,	false },
/*17,11h*/	{	0x10,	false },
/*18,12h*/	{	0x10,	false },
/*19,13h*/	{	0x1B,	false },
/*20,14h*/	{	0x1B,	false },
/*21,15h*/	{	0x1B,	false },
/*22,16h*/	{	0x2B,	false },
/*23,17h*/	{	0x3D,	false },
/*24,18h*/	{	0x50,	false },
/*25,19h*/	{	0x77,	false },
/*26,1Ah*/	{	0x2A,	false },
/*27,1Bh*/	{	0x10,	false },
/*28,1Ch*/	{	0x00,	false },
/*29,1Dh*/	{	0x00,	false },
};

#define BD6098_LED_BRIGHT_MASK		0x7f

#define NV_BK_LIGHT_CTL_LOWER_L_I_DEF	(0x02)
#define KEYLED_THRESHOLD_DEFAULT	NV_BK_LIGHT_CTL_LOWER_L_I_DEF

#define LABEL_MLED		"mled"
#define LABEL_RGB		"rgb"
#define LABEL_CLED		"cled"
#define LABEL_KEYLED	"keyled"
#define LABEL_SLED		"sled"

#define NUM_OF_COLORVARIATION	(5)
#define COLORVARIATION_DEFAULT	(3)

enum {
	LED_COLOR_B = 0,
	LED_COLOR_R,
	LED_COLOR_G,
	LED_COLOR_RB,
	LED_COLOR_GB,
	LED_COLOR_RG,
	LED_COLOR_RGB,
	LED_COLOR_MAX
};

static int rgb_table[LED_COLOR_MAX+1] = {
	-1,
	LED_COLOR_R,
	LED_COLOR_G,
	LED_COLOR_RG,
	LED_COLOR_B,
	LED_COLOR_RB,
	LED_COLOR_GB,
	LED_COLOR_RGB
};

#define	oem_bd6098_get_color(n)	rgb_table[n]

enum {
	LEDLIGHT_PATTERN_MANUAL = 0,
	LEDLIGHT_PATTERN_1,
	LEDLIGHT_PATTERN_2,
	LEDLIGHT_PATTERN_3,
	LEDLIGHT_PATTERN_4,
	LEDLIGHT_PATTERN_5,
	LEDLIGHT_PATTERN_6,
	LEDLIGHT_PATTERN_7,
	LEDLIGHT_PATTERN_8,
	LEDLIGHT_PATTERN_9,
	LEDLIGHT_PATTERN_10,
	LEDLIGHT_PATTERN_11,
	LEDLIGHT_PATTERN_12,
	LEDLIGHT_PATTERN_MAX
};

#define BRIGHTNESS_MAX	(0xFFFFFFFFul)

enum {
	LED_TYPE_MLED = 0,
	LED_TYPE_RGB,
	LED_TYPE_CLED,
	LED_TYPE_KEYLED,
	LED_TYPE_SLED,
	LED_TYPE_MAX
};

enum {
	CTRL_MODE_MANUAL = 0,
	CTRL_MODE_ALC,
	CTRL_MODE_ALC_CAMERA,
	CTRL_MODE_DIM,
	CTRL_MODE_MAX
};

#define GET_CTRL_MODE_EXT(a)		(a & 0xf0)
#define GET_CTRL_MODE(a)			(a & 0x0f)
#define CTRL_MODE_EXT_CAMERA		0x10
static uint32_t ext_ctrl_mode;

struct oem_bd6098_led_param {
	uint32_t				value;

	// for rgb
	uint32_t				blink_control;
	uint32_t				blink_low_pause_time;
	uint32_t				blink_high_pause_time;
	uint32_t				blink_off_color;

	// for mled/keyled
	uint32_t				ctrl_mode_mled;
	uint32_t				ctrl_mode_keyled;

	// for keyled
	uint32_t				keyled_threshold;
};

struct oem_bd6098_data;
struct oem_bd6098_led_data;
typedef void (*oem_bd6098_ctrl_func_t)(struct oem_bd6098_led_data *led,
	struct oem_bd6098_led_param *param_next);

struct oem_bd6098_led_data {
	struct oem_bd6098_data			*parent;
	struct led_classdev			cdev;
	bool						cdev_registered;
	struct work_struct			work;
	struct workqueue_struct		*workqueue;
	int							led_type;
	oem_bd6098_ctrl_func_t			ctrl_func;
	struct mutex				param_next_lock;
	struct oem_bd6098_led_param		param_next;
	struct oem_bd6098_led_param		param_prev;
};

struct oem_bd6098_blink_ctl {
	bool						next;
	struct oem_bd6098_led_param		param;
};

struct oem_bd6098_data {
	struct i2c_client			*client;
	int							reset_gpio;
	uint32_t					reset_delay[2];
	int							keyled_gpio;
	struct mdm_led				*mdm_led;

	struct mutex				control_lock;
	struct oem_bd6098_led_data		led_data[LED_TYPE_MAX];
	struct mutex				param_lock;
	struct oem_bd6098_led_param		param;
	int							power_state;
	int							photo_sensor_state;
	struct delayed_work			blink_work;
	struct oem_bd6098_blink_ctl		blink_ctl;
	struct wake_lock			wake_lock;
	struct workqueue_struct		*work_queue;
	int							colorvariation;
	bool						use_alarm_blink;
	struct alarm				blink_alarm;
	bool						in_suspend;
	struct completion			wait_resume;
};

static struct oem_bd6098_data *oem_bd6098_data;

static const uint8_t mled_reg03_manual[256][1] = {
	{	0x00	},	// 0
	{	0x00	},	// 1
	{	0x00	},	// 2
	{	0x01	},	// 3
	{	0x01	},	// 4
	{	0x02	},	// 5
	{	0x02	},	// 6
	{	0x03	},	// 7
	{	0x03	},	// 8
	{	0x04	},	// 9
	{	0x04	},	// 10
	{	0x05	},	// 11
	{	0x05	},	// 12
	{	0x06	},	// 13
	{	0x06	},	// 14
	{	0x07	},	// 15
	{	0x07	},	// 16
	{	0x08	},	// 17
	{	0x08	},	// 18
	{	0x09	},	// 19
	{	0x09	},	// 20
	{	0x0A	},	// 21
	{	0x0A	},	// 22
	{	0x0B	},	// 23
	{	0x0B	},	// 24
	{	0x0C	},	// 25
	{	0x0C	},	// 26
	{	0x0D	},	// 27
	{	0x0D	},	// 28
	{	0x0E	},	// 29
	{	0x0E	},	// 30
	{	0x0F	},	// 31
	{	0x0F	},	// 32
	{	0x10	},	// 33
	{	0x10	},	// 34
	{	0x11	},	// 35
	{	0x11	},	// 36
	{	0x12	},	// 37
	{	0x12	},	// 38
	{	0x13	},	// 39
	{	0x13	},	// 40
	{	0x14	},	// 41
	{	0x14	},	// 42
	{	0x15	},	// 43
	{	0x15	},	// 44
	{	0x16	},	// 45
	{	0x16	},	// 46
	{	0x17	},	// 47
	{	0x17	},	// 48
	{	0x18	},	// 49
	{	0x18	},	// 50
	{	0x19	},	// 51
	{	0x19	},	// 52
	{	0x1A	},	// 53
	{	0x1A	},	// 54
	{	0x1B	},	// 55
	{	0x1B	},	// 56
	{	0x1C	},	// 57
	{	0x1C	},	// 58
	{	0x1D	},	// 59
	{	0x1D	},	// 60
	{	0x1E	},	// 61
	{	0x1E	},	// 62
	{	0x1F	},	// 63
	{	0x1F	},	// 64
	{	0x20	},	// 65
	{	0x20	},	// 66
	{	0x21	},	// 67
	{	0x21	},	// 68
	{	0x22	},	// 69
	{	0x22	},	// 70
	{	0x23	},	// 71
	{	0x23	},	// 72
	{	0x24	},	// 73
	{	0x24	},	// 74
	{	0x25	},	// 75
	{	0x25	},	// 76
	{	0x26	},	// 77
	{	0x26	},	// 78
	{	0x27	},	// 79
	{	0x27	},	// 80
	{	0x28	},	// 81
	{	0x28	},	// 82
	{	0x29	},	// 83
	{	0x29	},	// 84
	{	0x2A	},	// 85
	{	0x2A	},	// 86
	{	0x2B	},	// 87
	{	0x2B	},	// 88
	{	0x2C	},	// 89
	{	0x2C	},	// 90
	{	0x2D	},	// 91
	{	0x2D	},	// 92
	{	0x2E	},	// 93
	{	0x2E	},	// 94
	{	0x2F	},	// 95
	{	0x2F	},	// 96
	{	0x30	},	// 97
	{	0x30	},	// 98
	{	0x31	},	// 99
	{	0x31	},	// 100
	{	0x32	},	// 101
	{	0x32	},	// 102
	{	0x33	},	// 103
	{	0x33	},	// 104
	{	0x34	},	// 105
	{	0x34	},	// 106
	{	0x35	},	// 107
	{	0x35	},	// 108
	{	0x36	},	// 109
	{	0x36	},	// 110
	{	0x37	},	// 111
	{	0x37	},	// 112
	{	0x38	},	// 113
	{	0x38	},	// 114
	{	0x39	},	// 115
	{	0x39	},	// 116
	{	0x3A	},	// 117
	{	0x3A	},	// 118
	{	0x3B	},	// 119
	{	0x3B	},	// 120
	{	0x3C	},	// 121
	{	0x3C	},	// 122
	{	0x3D	},	// 123
	{	0x3D	},	// 124
	{	0x3E	},	// 125
	{	0x3E	},	// 126
	{	0x3F	},	// 127
	{	0x3F	},	// 128
	{	0x40	},	// 129
	{	0x40	},	// 130
	{	0x41	},	// 131
	{	0x41	},	// 132
	{	0x42	},	// 133
	{	0x42	},	// 134
	{	0x43	},	// 135
	{	0x43	},	// 136
	{	0x44	},	// 137
	{	0x44	},	// 138
	{	0x45	},	// 139
	{	0x45	},	// 140
	{	0x46	},	// 141
	{	0x46	},	// 142
	{	0x47	},	// 143
	{	0x47	},	// 144
	{	0x48	},	// 145
	{	0x48	},	// 146
	{	0x49	},	// 147
	{	0x49	},	// 148
	{	0x4A	},	// 149
	{	0x4A	},	// 150
	{	0x4B	},	// 151
	{	0x4B	},	// 152
	{	0x4C	},	// 153
	{	0x4C	},	// 154
	{	0x4D	},	// 155
	{	0x4D	},	// 156
	{	0x4E	},	// 157
	{	0x4E	},	// 158
	{	0x4F	},	// 159
	{	0x4F	},	// 160
	{	0x50	},	// 161
	{	0x50	},	// 162
	{	0x51	},	// 163
	{	0x51	},	// 164
	{	0x52	},	// 165
	{	0x52	},	// 166
	{	0x53	},	// 167
	{	0x53	},	// 168
	{	0x54	},	// 169
	{	0x54	},	// 170
	{	0x55	},	// 171
	{	0x55	},	// 172
	{	0x56	},	// 173
	{	0x56	},	// 174
	{	0x57	},	// 175
	{	0x57	},	// 176
	{	0x58	},	// 177
	{	0x58	},	// 178
	{	0x59	},	// 179
	{	0x59	},	// 180
	{	0x5A	},	// 181
	{	0x5A	},	// 182
	{	0x5B	},	// 183
	{	0x5B	},	// 184
	{	0x5C	},	// 185
	{	0x5C	},	// 186
	{	0x5D	},	// 187
	{	0x5D	},	// 188
	{	0x5E	},	// 189
	{	0x5E	},	// 190
	{	0x5F	},	// 191
	{	0x5F	},	// 192
	{	0x60	},	// 193
	{	0x60	},	// 194
	{	0x61	},	// 195
	{	0x61	},	// 196
	{	0x62	},	// 197
	{	0x62	},	// 198
	{	0x63	},	// 199
	{	0x63	},	// 200
	{	0x64	},	// 201
	{	0x64	},	// 202
	{	0x65	},	// 203
	{	0x65	},	// 204
	{	0x66	},	// 205
	{	0x66	},	// 206
	{	0x67	},	// 207
	{	0x67	},	// 208
	{	0x68	},	// 209
	{	0x68	},	// 210
	{	0x69	},	// 211
	{	0x69	},	// 212
	{	0x6A	},	// 213
	{	0x6A	},	// 214
	{	0x6B	},	// 215
	{	0x6B	},	// 216
	{	0x6C	},	// 217
	{	0x6C	},	// 218
	{	0x6D	},	// 219
	{	0x6D	},	// 220
	{	0x6E	},	// 221
	{	0x6E	},	// 222
	{	0x6F	},	// 223
	{	0x6F	},	// 224
	{	0x70	},	// 225
	{	0x70	},	// 226
	{	0x71	},	// 227
	{	0x71	},	// 228
	{	0x72	},	// 229
	{	0x72	},	// 230
	{	0x73	},	// 231
	{	0x73	},	// 232
	{	0x74	},	// 233
	{	0x74	},	// 234
	{	0x75	},	// 235
	{	0x75	},	// 236
	{	0x76	},	// 237
	{	0x76	},	// 238
	{	0x77	},	// 239
	{	0x77	},	// 240
	{	0x77	},	// 241
	{	0x77	},	// 242
	{	0x77	},	// 243
	{	0x77	},	// 244
	{	0x77	},	// 245
	{	0x77	},	// 246
	{	0x77	},	// 247
	{	0x77	},	// 248
	{	0x77	},	// 249
	{	0x77	},	// 250
	{	0x77	},	// 251
	{	0x77	},	// 252
	{	0x77	},	// 253
	{	0x77	},	// 254
	{	0x77	},	// 255
};

const uint8_t mled_reg03_alc_init[16] = {
	0x03,
	0x07,
	0x07,
	0x07,
	0x07,
	0x10,
	0x10,
	0x10,
	0x10,
	0x1B,
	0x1B,
	0x1B,
	0x2B,
	0x3D,
	0x50,
	0x77,
};

const uint8_t mled_sensor_brightness[16] = {
	0x07,
	0x0F,
	0x0F,
	0x0F,
	0x0F,
	0x21,
	0x21,
	0x21,
	0x21,
	0x37,
	0x37,
	0x37,
	0x57,
	0x7B,
	0xA1,
	0xEF,
};

const uint8_t mled_reg03_dim = 0x01;

/* Local Function */
static int oem_bd6098_i2c_read(struct i2c_client *client, uint8_t uc_reg, uint8_t *rbuf, int len);
static int oem_bd6098_i2c_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val);
static void oem_bd6098_work(struct work_struct *work);
static void oem_bd6098_blink_work(struct work_struct *work);
#ifdef CONFIG_PM
static int oem_bd6098_suspend(struct i2c_client *client, pm_message_t mesg);
static int oem_bd6098_resume(struct i2c_client *client);
#endif
static int oem_bd6098_remove(struct i2c_client *client);
static int oem_bd6098_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int32_t __init oem_bd6098_led_init(void);
static void __exit oem_bd6098_led_exit(void);
static bool light_led_disp_enabled(void);
static void oem_bd6098_rgb_current_write(struct i2c_client *client, int color);
static bool light_sublcd_power_enabled(void);


static int oem_bd6098_dump_state(char *buffer, struct kernel_param *kp)
{
	int i;
	int count = 0;

	count += sprintf(&buffer[count], "dump registers.\n");
	count += sprintf(&buffer[count], "addr value written\n");

	for (i = 0; i < ARRAY_SIZE(oem_bd6098_reg_info_save); i++) {
		count += sprintf(&buffer[count], "0x%02x 0x%02x  %d\n",
			i,
			oem_bd6098_reg_info_save[i].value,
			oem_bd6098_reg_info_save[i].written);
	}

	return count;
}

module_param_call(state, NULL, oem_bd6098_dump_state,
		  NULL, 0440);

static int oem_bd6098_i2c_read(struct i2c_client *client, uint8_t uc_reg, uint8_t *rbuf, int len)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg i2cMsg[BD6098_I2C_READ_MSG_NUM];
	u8 reg = 0;
	int i = 0;

	BD6098_V_LOG("[IN] client=0x%p reg=0x%02X len=%d", client, uc_reg, len);

	if (client == NULL) {
		BD6098_E_LOG("fail client=0x%p", client);
		return -ENODEV;
	}

	reg = uc_reg;

	i2cMsg[0].addr = client->addr;
	i2cMsg[0].flags = 0;
	i2cMsg[0].len = 1;
	i2cMsg[0].buf = &reg;

	i2cMsg[1].addr = client->addr;
	i2cMsg[1].flags = I2C_M_RD;
	i2cMsg[1].len = len;
	i2cMsg[1].buf = rbuf;

	do {
		ret = i2c_transfer(client->adapter, &i2cMsg[0], BD6098_I2C_READ_MSG_NUM);
		BD6098_V_LOG("i2c_transfer() call end ret=%d", ret);
	} while ((ret != BD6098_I2C_READ_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != BD6098_I2C_READ_MSG_NUM) {
		BD6098_E_LOG("fail (try:%d) uc_reg=0x%02x, rbuf=0x%02x ret=%d", retry, uc_reg, *rbuf, ret);
		ret = -1;
	} else {
		ret = 0;
		BD6098_V_LOG("i2c read success");
		for (i = 0; i < len; i++)
		{
			BD6098_D_LOG("i2c read  reg=0x%02x,value=0x%02x", (unsigned int)(uc_reg + i), (unsigned int)*(rbuf + i));
		}
	}
	BD6098_V_LOG("[OUT] ret=%d", ret);

	return ret;
}

static int oem_bd6098_i2c_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg i2cMsg;
	u8 ucwritebuf[BD6098_WRITE_BUF_LEN];

	BD6098_V_LOG("[IN] client=0x%p reg=0x%02x val=0x%02X", client, uc_reg, uc_val);

	if (client == NULL) {
		BD6098_E_LOG("fail client=0x%p", client);
		return -ENODEV;
	}

	ucwritebuf[0] = uc_reg;
	ucwritebuf[1] = uc_val;
	i2cMsg.addr  = client->addr;
	i2cMsg.flags = 0;
	i2cMsg.len   =  sizeof(ucwritebuf);
	i2cMsg.buf   =  &ucwritebuf[0];

	BD6098_D_LOG("i2c write reg=0x%02x,value=0x%02x", uc_reg, uc_val);

	do {
		ret = i2c_transfer(client->adapter, &i2cMsg, BD6098_I2C_WRITE_MSG_NUM);
		BD6098_V_LOG("i2c_transfer() call end ret=%d", ret);
	} while ((ret != BD6098_I2C_WRITE_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != BD6098_I2C_WRITE_MSG_NUM) {
		BD6098_E_LOG("fail (try:%d) uc_reg=0x%02x, uc_val=0x%02x ret=%d", retry, ucwritebuf[0], ucwritebuf[1], ret);
		oem_bd6098_reg_info_save[uc_reg].written = false;
		ret = -1;
	} else {
		BD6098_V_LOG("success reg=0x%02x val=0x%02x", ucwritebuf[0], ucwritebuf[1]);
		oem_bd6098_reg_info_save[uc_reg].written = true;
		oem_bd6098_reg_info_save[uc_reg].value = uc_val;
		ret = 0;
	}

	BD6098_V_LOG("[OUT] ret=%d", ret);

	return ret;
}

static int oem_bd6098_i2c_masked_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val, uint8_t mask)
{
	uint8_t new_value;
	int ret;

	BD6098_V_LOG("[IN] reg=0x%02x val=0x%02x mask=0x%02x", uc_reg, uc_val, mask);

	new_value = (oem_bd6098_reg_info_save[uc_reg].value & ~mask) | (uc_val & mask);

	if (oem_bd6098_reg_info_save[uc_reg].written &&
		(new_value == oem_bd6098_reg_info_save[uc_reg].value)) {
		BD6098_V_LOG("no change");
		return 0;
	}

	ret = oem_bd6098_i2c_write(client, uc_reg, new_value);

	BD6098_V_LOG("[OUT] ret=%d", ret);

	return ret;
}

static void led_set(struct led_classdev *cdev, enum led_brightness value)
{
	struct oem_bd6098_led_data *led =
		container_of(cdev, struct oem_bd6098_led_data, cdev);
	struct oem_bd6098_data *data = led->parent;

	BD6098_D_LOG("[IN] name=%s value=0x%08x", cdev->name, value);

	mutex_lock(&data->param_lock);
	mutex_lock(&led->param_next_lock);
	memcpy(&led->param_next, &data->param, sizeof(led->param_next));
	led->param_next.value = (uint32_t)value;
	mutex_unlock(&led->param_next_lock);
	mutex_unlock(&data->param_lock);

	queue_work(led->workqueue, &led->work);

	BD6098_D_LOG("[OUT]");
}

static enum led_brightness led_get(struct led_classdev *cdev)
{
	int32_t lret = 0;
	struct oem_bd6098_led_data *led =
		container_of(cdev, struct oem_bd6098_led_data, cdev);

	BD6098_D_LOG("[IN] name=%s", cdev->name);

	lret = (int32_t)led->param_next.value;

	BD6098_D_LOG("[OUT] lret=0x%08x", lret);

	return lret;
}

static void oem_bd6098_power_control(struct oem_bd6098_data *data, int led_type, bool on)
{
#define VLED_ON_STATE_MASK \
	((1 << LED_TYPE_RGB) | (1 << LED_TYPE_SLED))

	struct i2c_client *client = data->client;
	int state_prev = data->power_state;
	int state_next = data->power_state;

	BD6098_V_LOG("[IN] led_type=%d on=%d", led_type, on);

	if (on) {
		state_next |=  (1 << led_type);
	} else {
		state_next &= ~(1 << led_type);
	}

	BD6098_V_LOG("state_prev=0x%08x -> state_next=0x%08x", state_prev, state_next);

	if (state_next & VLED_ON_STATE_MASK) {
		oem_bd6098_i2c_masked_write(client, 0x00, 0x30, 0x30);

		if (!(state_prev & VLED_ON_STATE_MASK))
			usleep(200);
	} else {
		oem_bd6098_i2c_masked_write(client, 0x00, 0x00, 0x30);
	}

	data->power_state = state_next;

	BD6098_V_LOG("[OUT]");

}

static void oem_bd6098_photo_sensor_control(struct oem_bd6098_data *data, int led_type, bool on)
{
	struct i2c_client *client = data->client;
	int state_prev = data->photo_sensor_state;
	int state_next = data->photo_sensor_state;

	BD6098_V_LOG("[IN] led_type=%d on=%d", led_type, on);

	if (on) {
		state_next |=  (1 << led_type);
	} else {
		state_next &= ~(1 << led_type);
	}

	if (!state_prev && state_next) {
		BD6098_V_LOG("enable photo sensor");
		oem_bd6098_i2c_masked_write(client, 0x02, 0x08, 0x08);
	} else if (state_prev && !state_next) {
		BD6098_V_LOG("disable photo sensor");
		oem_bd6098_i2c_masked_write(client, 0x02, 0x00, 0x08);
	} else {
		BD6098_V_LOG("no change");
	}

	data->photo_sensor_state = state_next;

	BD6098_V_LOG("[OUT] state_next=0x%08x state_prev=0x%08x", state_next, state_prev);
}

static int oem_bd6098_get_photo_sensor_level(struct oem_bd6098_data *data)
{
	struct i2c_client *client = data->client;
	uint8_t read_buf = 0;
	int ret = 0;
	int level = 0;

	BD6098_V_LOG("[IN]");

	ret = oem_bd6098_i2c_read(client, 0x09, &read_buf, 1);
	if (ret >= 0) {
		level = read_buf & 0xf;
	}

	BD6098_V_LOG("[OUT] level=0x%08x", level);

	return level;
}

#define USLEEP_UNLOCK(sleep_us, lock) {	\
	mutex_unlock(lock);	\
	usleep(sleep_us);	\
	mutex_lock(lock);	\
}

static void oem_bd6098_ctrl_update_alc_table(struct i2c_client *client, uint32_t max_value)
{
	uint32_t value;
	int i;

	BD6098_V_LOG("[IN] max_value=0x%08x", max_value);

	for (i = 0; i < ARRAY_SIZE(mled_reg03_alc_init); i++) {
		value = mled_reg03_alc_init[i];
		if (value > max_value)
			value = max_value;
		oem_bd6098_i2c_masked_write(client, 0x0A + i, value, 0xFF);
	}

	BD6098_V_LOG("[OUT]");
}

static void led_set_sensor(struct led_classdev *cdev, enum led_brightness value)
{
	BD6098_V_LOG("[IN]");
	BD6098_V_LOG("[OUT]");
}

static enum led_brightness led_get_sensor(struct led_classdev *cdev)
{
	int32_t lret = 0;
	struct oem_bd6098_led_data *led =
		container_of(cdev, struct oem_bd6098_led_data, cdev);
	struct oem_bd6098_data *data = led->parent;
	struct oem_bd6098_led_param *param_prev = &led->param_prev;
	int ctrl_mode_prev = GET_CTRL_MODE(param_prev->ctrl_mode_mled);
	int photo_level;
	uint32_t sensor_brightness;
	uint32_t max_level;

	BD6098_V_LOG("[IN] name=%s ctrl_mode_prev=0x%02x param_prev->value=0x%02x", cdev->name, ctrl_mode_prev, param_prev->value);

	mutex_lock(&data->control_lock);
	if (param_prev->value &&
		(ctrl_mode_prev == CTRL_MODE_ALC || ctrl_mode_prev == CTRL_MODE_ALC_CAMERA)) {
		max_level = led->cdev.max_brightness;
		if (max_level > 0xFF)
			max_level = 0xFF;

		photo_level = oem_bd6098_get_photo_sensor_level(data);
		sensor_brightness = mled_sensor_brightness[photo_level&0x0F];
		if (sensor_brightness > max_level)
			sensor_brightness = max_level;
		led->cdev.sensor_brightness = sensor_brightness;
		BD6098_V_LOG("photo_level = 0x%02x, sensor_brightness = 0x%02x", photo_level, sensor_brightness);
	}
	lret = (int32_t)led->cdev.sensor_brightness;
	mutex_unlock(&data->control_lock);

	BD6098_V_LOG("[OUT] lret=0x%08x", lret);

	return lret;
}

static void oem_bd6098_reinit(struct i2c_client *client)
{
	BD6098_V_LOG("[IN]");

#ifdef ENABLE_PWM
	if (OEM_BOARD3_TYPE == OEM_get_board()) {
		oem_bd6098_i2c_write(client, 0x01, 0x4E);
	} else {
		oem_bd6098_i2c_write(client, 0x01, 0xCE);
	}
#else
	oem_bd6098_i2c_write(client, 0x01, 0x4E);
#endif
	oem_bd6098_i2c_write(client, 0x02, 0x00);
	oem_bd6098_i2c_write(client, 0x03, 0x00);
	oem_bd6098_i2c_write(client, 0x06, 0x00);
	oem_bd6098_i2c_write(client, 0x07, 0x31);
	oem_bd6098_i2c_write(client, 0x08, 0xEE);
	oem_bd6098_i2c_write(client, 0x0A, 0x03);
	oem_bd6098_i2c_write(client, 0x0B, 0x07);
	oem_bd6098_i2c_write(client, 0x0C, 0x07);
	oem_bd6098_i2c_write(client, 0x0D, 0x07);
	oem_bd6098_i2c_write(client, 0x0E, 0x07);
	oem_bd6098_i2c_write(client, 0x0F, 0x10);
	oem_bd6098_i2c_write(client, 0x10, 0x10);
	oem_bd6098_i2c_write(client, 0x11, 0x10);
	oem_bd6098_i2c_write(client, 0x12, 0x10);
	oem_bd6098_i2c_write(client, 0x13, 0x1B);
	oem_bd6098_i2c_write(client, 0x14, 0x1B);
	oem_bd6098_i2c_write(client, 0x15, 0x1B);
	oem_bd6098_i2c_write(client, 0x16, 0x2B);
	oem_bd6098_i2c_write(client, 0x17, 0x3D);
	oem_bd6098_i2c_write(client, 0x18, 0x50);
	oem_bd6098_i2c_write(client, 0x19, 0x77);

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_ctrl_mled(struct oem_bd6098_led_data *led,
	struct oem_bd6098_led_param *param_next)
{
	struct oem_bd6098_data *data = led->parent;
	struct i2c_client *client = data->client;
	struct oem_bd6098_led_param *param_prev = &led->param_prev;
	int photo_level;
	uint32_t level;
	uint32_t max_level;
	uint32_t value;
	uint32_t max_value;
	int ctrl_mode_next = GET_CTRL_MODE(param_next->ctrl_mode_mled);
	int ctrl_mode_prev = GET_CTRL_MODE(param_prev->ctrl_mode_mled);
	uint32_t sensor_brightness;

	BD6098_V_LOG("[IN] value=0x%08x", param_next->value);

	if (!light_led_disp_enabled()) {
		BD6098_V_LOG("discard light_led_disp_enabled()");
		return;
	}

	if (param_next->value) {
		if (!param_prev->value) {
			if( !(data->power_state & ((1 << LED_TYPE_MLED) | (1 << LED_TYPE_KEYLED))) ){
				oem_bd6098_reinit(client);
			}
		}

		oem_bd6098_power_control(data, led->led_type, true);

		switch (ctrl_mode_next) {
		case CTRL_MODE_DIM:
			BD6098_V_LOG("CTRL_MODE_DIM");
			led->cdev.sensor_brightness = 3;
			oem_bd6098_i2c_masked_write(client, 0x03, mled_reg03_dim, 0xFF);
			oem_bd6098_i2c_masked_write(client, 0x07, 0xC1, 0xC1);
			oem_bd6098_i2c_masked_write(client, 0x06, 0x00, 0xFF);
			oem_bd6098_i2c_masked_write(client, 0x01, 0x00, 0x01);
			oem_bd6098_i2c_masked_write(client, 0x02, 0x01, 0x01);
			USLEEP_UNLOCK(50000, &data->control_lock);
			oem_bd6098_photo_sensor_control(data, led->led_type, false);
			break;
		case CTRL_MODE_ALC_CAMERA:
			BD6098_V_LOG("CTRL_MODE_ALC_CAMERA");
			/* FALL THROUGH */
		case CTRL_MODE_ALC:
			BD6098_V_LOG("CTRL_MODE_ALC");
			max_level = led->cdev.max_brightness;
			if (max_level > 0xFF)
				max_level = 0xFF;
			max_value = mled_reg03_manual[max_level][0];

			oem_bd6098_ctrl_update_alc_table(client, max_value);
			if (!param_prev->value ||
				(ctrl_mode_prev != CTRL_MODE_ALC && ctrl_mode_prev != CTRL_MODE_ALC_CAMERA)) {
				oem_bd6098_i2c_masked_write(client, 0x01, 0x00, 0x01);
				oem_bd6098_i2c_masked_write(client, 0x06, 0x00, 0xFF);

				oem_bd6098_i2c_masked_write(client, 0x07, 0x01, 0xC1);
				oem_bd6098_photo_sensor_control(data, led->led_type, true);
				USLEEP_UNLOCK(96500, &data->control_lock);
				photo_level = oem_bd6098_get_photo_sensor_level(data);
				value = mled_reg03_alc_init[photo_level&0x0F];
				if (value > max_value)
					value = max_value;
				oem_bd6098_i2c_masked_write(client, 0x03, value, 0xFF);
				oem_bd6098_i2c_masked_write(client, 0x02, 0x01, 0x01);
				USLEEP_UNLOCK(50000, &data->control_lock);
				oem_bd6098_i2c_masked_write(client, 0x06, 0xBB, 0xFF);
				oem_bd6098_i2c_masked_write(client, 0x07, 0x00, 0x01);
				oem_bd6098_i2c_masked_write(client, 0x01, 0x01, 0x01);
				sensor_brightness = mled_sensor_brightness[photo_level&0x0F];
				if (sensor_brightness > max_level)
					sensor_brightness = max_level;
				led->cdev.sensor_brightness = sensor_brightness;
			}
			break;
		case CTRL_MODE_MANUAL:
			BD6098_V_LOG("CTRL_MODE_MANUAL");
			/* FALL THROUGH */
		default:
			BD6098_V_LOG("default");
			level = param_next->value;
			if (level > 0xFF) level = 0xFF;
			led->cdev.sensor_brightness = level;
			oem_bd6098_i2c_masked_write(client, 0x03, mled_reg03_manual[level][0], 0xFF);
			oem_bd6098_i2c_masked_write(client, 0x07, 0xC1, 0xC1);
			oem_bd6098_i2c_masked_write(client, 0x06, 0x00, 0xFF);
			oem_bd6098_i2c_masked_write(client, 0x01, 0x00, 0x01);
			oem_bd6098_i2c_masked_write(client, 0x02, 0x01, 0x01);
			USLEEP_UNLOCK(50000, &data->control_lock);
			oem_bd6098_photo_sensor_control(data, led->led_type, false);
			break;
		}
	} else {
		oem_bd6098_i2c_masked_write(client, 0x07, 0xC1, 0xC1);
		oem_bd6098_i2c_masked_write(client, 0x06, 0x00, 0xFF);
		oem_bd6098_i2c_masked_write(client, 0x02, 0x00, 0x01);
		oem_bd6098_photo_sensor_control(data, led->led_type, false);
		oem_bd6098_power_control(data, led->led_type, false);
		led->cdev.sensor_brightness = 0;
	}

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_ctrl_sled(struct oem_bd6098_led_data *led,
							 struct oem_bd6098_led_param *param_next)
{
	struct oem_bd6098_data *data = led->parent;
	struct i2c_client *client = data->client;

	BD6098_V_LOG("[IN]");

	if(param_next->value & 0x00ffffff){
		if(!light_sublcd_power_enabled()){
			BD6098_V_LOG("discard light_sublcd_power_enabled()");
			return;
		}
		BD6098_V_LOG("SUBLED ON");
		oem_bd6098_power_control(data, led->led_type, true);
		oem_bd6098_i2c_masked_write(client, 0x1C, 0x08, 0x08);
	} else {
		BD6098_V_LOG("SUBLED OFF");
		oem_bd6098_i2c_masked_write(client, 0x1C, 0x00, 0x08);
		oem_bd6098_power_control(data, led->led_type, false);
	}

	BD6098_V_LOG("[OUT]");
	return;
}

static void oem_bd6098_queue_rgb_work(void)
{
	struct oem_bd6098_data *data = oem_bd6098_data;
	struct oem_bd6098_led_data *led;

	BD6098_V_LOG("[IN]");
	if (data) {
		led = &data->led_data[LED_TYPE_RGB];
		if (led->cdev_registered) {
			if (led->param_next.value) {
				queue_work(led->workqueue, &led->work);
			}
		}
	}
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_mdm_rgb_request(struct oem_bd6098_data *data,
	const struct tricolor_led_request *request)
{
	struct mdm_led *mdm_led = data->mdm_led;
	BD6098_V_LOG("[IN]");
	if (mdm_led && mdm_led->send_request_fn)
		mdm_led->send_request_fn(mdm_led, request);
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_mdm_rgb_off(struct oem_bd6098_data *data)
{
	const struct tricolor_led_request off_request = {0};
	BD6098_V_LOG("[IN]");
	oem_bd6098_mdm_rgb_request(data, &off_request);
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_rgb_off(struct oem_bd6098_data *data)
{
	struct i2c_client *client = data->client;
	BD6098_V_LOG("[IN]");
	alarm_cancel(&data->blink_alarm);
	cancel_delayed_work(&data->blink_work);
	wake_unlock(&data->wake_lock);
	oem_bd6098_mdm_rgb_off(data);
	oem_bd6098_i2c_masked_write(client, 0x1C, 0x00, 0x07);
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_rgb_on(struct i2c_client *client, uint32_t value)
{
	uint8_t rgb_on;
	int color;

	BD6098_V_LOG("[IN] value=0x%08x", value);

	rgb_on = ((value & 0x00ff0000) ? 0x01 : 0) |
			 ((value & 0x0000ff00) ? 0x02 : 0) |
			 ((value & 0x000000ff) ? 0x04 : 0);

	// current
	color = oem_bd6098_get_color(rgb_on);
	oem_bd6098_rgb_current_write(client, color);

	// rgb on
	oem_bd6098_i2c_masked_write(client, 0x1C, rgb_on, 0x07);

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_mdm_rgb_blink_request(struct oem_bd6098_data *data, struct oem_bd6098_led_param *param)
{
	int32_t total_time;
	struct tricolor_led_request request = {0};

	BD6098_D_LOG("[IN] value=0x%08x off=0x%08x on=%dms off=%dms",
		param->value, param->blink_off_color,
		param->blink_high_pause_time, param->blink_low_pause_time);

	total_time = param->blink_high_pause_time + param->blink_low_pause_time;

	// modem ctrl
	request.color = param->value;
	request.mode = TRICOLOR_BLINK_REQUEST;
	request.on_time = param->blink_high_pause_time;
	request.off_time = param->blink_low_pause_time;
	request.off_color = param->blink_off_color;

	oem_bd6098_mdm_rgb_request(data, &request);

	BD6098_D_LOG("[OUT]");
}

static enum alarmtimer_restart oem_bd6098_blink_alarm_callback(struct alarm *alarm, ktime_t now)
{
	struct oem_bd6098_data *data = container_of(alarm, struct oem_bd6098_data, blink_alarm);
	struct oem_bd6098_led_data *led = &data->led_data[LED_TYPE_RGB];

	BD6098_V_LOG("[IN]");

	wake_lock(&data->wake_lock);
	queue_delayed_work(led->workqueue, &data->blink_work, 0);

	BD6098_V_LOG("[OUT]");

	return ALARMTIMER_NORESTART;
}

static void oem_bd6098_blink_work(struct work_struct *work)
{
	struct oem_bd6098_data *data =
		container_of(work, struct oem_bd6098_data, blink_work.work);
	struct oem_bd6098_led_data *led = &data->led_data[LED_TYPE_RGB];
	ktime_t delay_ktime;
	uint32_t delay_ms;

	BD6098_V_LOG("[IN]");

	if (data->in_suspend) {
		BD6098_D_LOG("wait for resume begin");
		if (!wait_for_completion_timeout(&data->wait_resume, msecs_to_jiffies(2000))) {
			BD6098_N_LOG("wait for resume timed out");
		}
		BD6098_D_LOG("wait for resume end");
	}

	mutex_lock(&data->control_lock);
	if (data->blink_ctl.next) {
		data->blink_ctl.next = false;
		oem_bd6098_power_control(data, LED_TYPE_RGB, true);
		oem_bd6098_rgb_on(data->client, data->blink_ctl.param.value);
		delay_ms = data->blink_ctl.param.blink_high_pause_time;
	} else {
		data->blink_ctl.next = true;
		oem_bd6098_rgb_on(data->client, data->blink_ctl.param.blink_off_color);
		if (!data->blink_ctl.param.blink_off_color) {
			oem_bd6098_power_control(data, LED_TYPE_RGB, false);
		}
		delay_ms = data->blink_ctl.param.blink_low_pause_time;
	}
	mutex_unlock(&data->control_lock);

	if (data->use_alarm_blink) {
		wake_unlock(&data->wake_lock);
		delay_ktime = ns_to_ktime((u64)delay_ms * NSEC_PER_MSEC);
		alarm_start_relative(&data->blink_alarm, delay_ktime);
	} else {
		queue_delayed_work(led->workqueue, &data->blink_work,
			msecs_to_jiffies(delay_ms));
	}

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_rgb_blink(struct oem_bd6098_data *data, struct oem_bd6098_led_param *param)
{
	bool blink;
	struct i2c_client *client = data->client;
	int32_t total_time;
	struct oem_bd6098_led_data *led = &data->led_data[LED_TYPE_RGB];

	BD6098_V_LOG("[IN]");

	if (!param->value)
		return;

	if (param->blink_high_pause_time && param->blink_low_pause_time)
		blink = true;
	else
		blink = false;

	if (blink) {
		total_time = param->blink_high_pause_time + param->blink_low_pause_time;
		if ((data->mdm_led) &&
			(total_time >= 1000) &&
			((param->value & 0x000000ff) != 0) &&
			((param->value & ~0x000000ff) == 0) &&
			(param->blink_off_color == 0)) {
			oem_bd6098_power_control(data, LED_TYPE_RGB, false);
			oem_bd6098_mdm_rgb_blink_request(data, param);
		} else {
			wake_lock(&data->wake_lock);

			data->blink_ctl.next  = true;
			data->blink_ctl.param = *param;
			queue_delayed_work(led->workqueue, &data->blink_work, 0);
		}
	} else {
		oem_bd6098_power_control(data, LED_TYPE_RGB, true);
		oem_bd6098_rgb_on(client, param->value);
	}

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_rgb_current_write(struct i2c_client *client, int color)
{
	BD6098_V_LOG("[IN]");
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_rgb_pattern_prepare(struct oem_bd6098_led_param *param)
{
	BD6098_V_LOG("[IN]");

	switch(param->blink_control) {
	case LEDLIGHT_PATTERN_1:    param->value = 0x000000FF; break;
	case LEDLIGHT_PATTERN_2:    param->value = 0x00FF0000; break;
	case LEDLIGHT_PATTERN_3:    param->value = 0x0000FF00; break;
	case LEDLIGHT_PATTERN_4:    param->value = 0x00FF00FF; break;
	case LEDLIGHT_PATTERN_5:    param->value = 0x0000FFFF; break;
	case LEDLIGHT_PATTERN_6:    param->value = 0x00FFFF00; break;
	case LEDLIGHT_PATTERN_7:    param->value = 0x00FFFFFF; break;
	case LEDLIGHT_PATTERN_11:   param->value = 0x00FF0000; break;
	default:                    param->value = 0x00000000; break;
	}

	switch(param->blink_control) {
	case LEDLIGHT_PATTERN_1:
	case LEDLIGHT_PATTERN_2:
	case LEDLIGHT_PATTERN_3:
	case LEDLIGHT_PATTERN_4:
	case LEDLIGHT_PATTERN_5:
	case LEDLIGHT_PATTERN_6:
	case LEDLIGHT_PATTERN_7:
	case LEDLIGHT_PATTERN_11:
		param->blink_high_pause_time = 250;
		param->blink_low_pause_time = 250;
		param->blink_off_color = 0x00000000;
		break;
	default:
		param->blink_high_pause_time = 0;
		param->blink_low_pause_time = 0;
		param->blink_off_color = 0x00000000;
		break;
	}

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_rgb_pattern8(struct i2c_client *client, uint32_t value)
{
	BD6098_V_LOG("[IN] value=0x%08x", value);

	oem_bd6098_rgb_on(client, value);

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_ctrl_rgb(struct oem_bd6098_led_data *led,
	struct oem_bd6098_led_param *param_next)
{
	struct oem_bd6098_data *data = led->parent;
	struct i2c_client *client = data->client;
	struct oem_bd6098_led_param *param_prev = &led->param_prev;

	BD6098_V_LOG("[IN] value=0x%08x", param_next->value);

	if (param_next->value) {
		switch(param_next->blink_control) {
		case LEDLIGHT_PATTERN_1:
		case LEDLIGHT_PATTERN_2:
		case LEDLIGHT_PATTERN_3:
		case LEDLIGHT_PATTERN_4:
		case LEDLIGHT_PATTERN_5:
		case LEDLIGHT_PATTERN_6:
		case LEDLIGHT_PATTERN_7:
		case LEDLIGHT_PATTERN_11:
			oem_bd6098_rgb_off(data);
			oem_bd6098_rgb_pattern_prepare(param_next);
			oem_bd6098_rgb_blink(data, param_next);
			break;
		case LEDLIGHT_PATTERN_8:
			if (param_prev->blink_control != param_next->blink_control) {
				oem_bd6098_rgb_off(data);
				wake_lock(&data->wake_lock);
			}
			oem_bd6098_power_control(data, led->led_type, true);
			oem_bd6098_rgb_pattern8(client, param_next->value);
			break;
		case LEDLIGHT_PATTERN_MANUAL:
			oem_bd6098_rgb_off(data);
			oem_bd6098_rgb_blink(data, param_next);
			break;
		default:
			oem_bd6098_rgb_off(data);
			oem_bd6098_power_control(data, led->led_type, false);
			break;
		}
	} else {
		if (param_next->blink_control == LEDLIGHT_PATTERN_8) {
			oem_bd6098_rgb_pattern8(client, param_next->value);
		} else {
			oem_bd6098_rgb_off(data);
			oem_bd6098_power_control(data, led->led_type, false);
		}
	}
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_ctrl_cled(struct oem_bd6098_led_data *led,
	struct oem_bd6098_led_param *param_next)
{
	BD6098_V_LOG("[IN] value=0x%08x", param_next->value);
	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_ctrl_keyled(struct oem_bd6098_led_data *led,
	struct oem_bd6098_led_param *param_next)
{
	struct oem_bd6098_data *data = led->parent;
	struct i2c_client *client = data->client;
	struct oem_bd6098_led_param *param_prev = &led->param_prev;
	int photo_level;

	BD6098_V_LOG("[IN] value=0x%08x", param_next->value);

	if (param_next->value) {
		if (!param_prev->value) {
			if( !(data->power_state & ((1 << LED_TYPE_MLED) | (1 << LED_TYPE_KEYLED))) ){
				oem_bd6098_reinit(client);
			}
		}

		oem_bd6098_power_control(data, led->led_type, true);
		if (param_next->ctrl_mode_keyled == CTRL_MODE_ALC) {
			oem_bd6098_photo_sensor_control(data, led->led_type, true);
			USLEEP_UNLOCK(96500, &data->control_lock);
			photo_level = oem_bd6098_get_photo_sensor_level(data);
			BD6098_D_LOG("photo_level=%d threshold=%d", photo_level, param_next->keyled_threshold);
			if (photo_level <= param_next->keyled_threshold) {
				gpio_set_value(data->keyled_gpio, 1);
			} else {
				if (photo_level > (param_next->keyled_threshold + 1)) {
					gpio_set_value(data->keyled_gpio, 0);
				}
			}
		} else {
			gpio_set_value(data->keyled_gpio, 1);
			oem_bd6098_photo_sensor_control(data, led->led_type, false);
		}
	} else {
		gpio_set_value(data->keyled_gpio, 0);
		oem_bd6098_photo_sensor_control(data, led->led_type, false);
		oem_bd6098_power_control(data, led->led_type, false);
	}

	BD6098_V_LOG("[OUT]");
}

static void oem_bd6098_work(struct work_struct *work)
{
	struct oem_bd6098_led_data *led =
		container_of(work, struct oem_bd6098_led_data, work);
	struct oem_bd6098_data *data = led->parent;
	struct oem_bd6098_led_param param_next;

	BD6098_V_LOG("[IN] led_type[%d]", led->led_type);

	mutex_lock(&led->param_next_lock);
	memcpy(&param_next, &led->param_next, sizeof(param_next));
	mutex_unlock(&led->param_next_lock);

	if (led->ctrl_func) {
		mutex_lock(&data->control_lock);
		led->ctrl_func(led, &param_next);
		mutex_unlock(&data->control_lock);
	}

	memcpy(&led->param_prev, &param_next, sizeof(led->param_prev));

	BD6098_V_LOG("[OUT]");
}

//
// for device ioctl
//
#define LEDLIGHT_PARAM_NUM      (5)
#define LEDLIGHT                'L'
#define LEDLIGHT_SET_BLINK		_IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define LEDLIGHT_SET_PARAM      _IOW(LEDLIGHT, 1, T_LEDLIGHT_IOCTL)

#define LEDLIGHT_PARAM_MLED     (1)
#define LEDLIGHT_PARAM_RGB      (2)
#define LEDLIGHT_PARAM_CLED     (3)
#define LEDLIGHT_PARAM_KEYLED   (4)

#define LEDLIGHT_PARAM_CHANGE_COLORVARIATION   (10)

typedef struct _t_ledlight_ioctl {
	uint32_t data[LEDLIGHT_PARAM_NUM];
} T_LEDLIGHT_IOCTL;

static int32_t leds_open(struct inode* inode, struct file* filp)
{
	BD6098_V_LOG("[IN/OUT]");
	return 0;
}

static int32_t leds_release(struct inode* inode, struct file* filp)
{
	BD6098_V_LOG("[IN/OUT]");
	return 0;
}

static long leds_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int32_t ret = -1;
	T_LEDLIGHT_IOCTL st_ioctl;
	struct oem_bd6098_data *data = oem_bd6098_data;
	struct oem_bd6098_led_param *param = &data->param;

	BD6098_V_LOG("[IN]");

	if (!data) {
		BD6098_N_LOG("Error data == NULL");
		return -EFAULT;
	}
	switch (cmd) {
	case LEDLIGHT_SET_PARAM:
		BD6098_V_LOG("LEDLIGHT_SET_CONTROL 0x%08x", LEDLIGHT_SET_PARAM);
		ret = copy_from_user(&st_ioctl,
					argp,
					sizeof(T_LEDLIGHT_IOCTL));
		if (ret) {
			BD6098_N_LOG("Error leds_ioctl(cmd = LEDLIGHT_SET_CONTROL_MODE)");
			return -EFAULT;
		}
		BD6098_V_LOG("st_ioctl data[0]=[0x%08x] data[1]=[0x%08x] data[2]=[0x%08x] data[3]=[0x%08x] data[4]=[0x%08x]",
			st_ioctl.data[0], st_ioctl.data[1], st_ioctl.data[2], st_ioctl.data[3], st_ioctl.data[4]);

		switch(st_ioctl.data[0]) {
		case LEDLIGHT_PARAM_MLED:
			mutex_lock(&data->param_lock);
			param->ctrl_mode_mled = st_ioctl.data[1];
			ext_ctrl_mode = GET_CTRL_MODE_EXT(param->ctrl_mode_mled);
			mutex_unlock(&data->param_lock);
			BD6098_D_LOG("LEDLIGHT_PARAM_MLED ctrl_mode_mled=0x%x", param->ctrl_mode_mled);
			break;
		case LEDLIGHT_PARAM_RGB:
			mutex_lock(&data->param_lock);
			param->blink_control         = st_ioctl.data[1];
			param->blink_high_pause_time = st_ioctl.data[2];
			param->blink_low_pause_time  = st_ioctl.data[3];
			param->blink_off_color       = st_ioctl.data[4];
			mutex_unlock(&data->param_lock);
			BD6098_D_LOG("LEDLIGHT_PARAM_RGB blink_control=%d high=%d low=%d off_color=0x%08x",
				param->blink_control, param->blink_high_pause_time,
				param->blink_low_pause_time, param->blink_off_color);
			break;
		case LEDLIGHT_PARAM_KEYLED:
			mutex_lock(&data->param_lock);
			param->ctrl_mode_keyled = st_ioctl.data[1];
			param->keyled_threshold = st_ioctl.data[2];
			mutex_unlock(&data->param_lock);
			BD6098_D_LOG("LEDLIGHT_PARAM_KEYLED ctrl_mode_keyled=%d keyled_threshold=%d",
				param->ctrl_mode_keyled, param->keyled_threshold);
			break;
		case LEDLIGHT_PARAM_CHANGE_COLORVARIATION:
			if (st_ioctl.data[1] >= 0 && st_ioctl.data[1] < NUM_OF_COLORVARIATION) {
				data->colorvariation = st_ioctl.data[1];
				BD6098_D_LOG("LEDLIGHT_PARAM_CHANGE_COLORVARIATION colvar=%d", data->colorvariation);
			} else {
				BD6098_N_LOG("invalid param 0x%08x", st_ioctl.data[1]);
				return -EFAULT;
			}
			oem_bd6098_queue_rgb_work();
			break;
		default:
			BD6098_N_LOG("invalid param 0x%08x", st_ioctl.data[0]);
			return -EFAULT;
			break;
		}
		break;
	case LEDLIGHT_SET_BLINK:
		BD6098_N_LOG("not supported. cmd 0x%08x", cmd);
		return -ENOTTY;
	default:
		BD6098_N_LOG("invalid cmd 0x%08x", cmd);
		return -ENOTTY;
	}

	BD6098_V_LOG("[OUT]");

	return 0;
}

static struct file_operations leds_fops = {
	.owner		  = THIS_MODULE,
	.open		 = leds_open,
	.release	= leds_release,
	.unlocked_ioctl = leds_ioctl,
};

static struct miscdevice leds_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "leds-ledlight",
	.fops  = &leds_fops,
};

#ifdef CONFIG_PM
static int oem_bd6098_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct oem_bd6098_data *data = i2c_get_clientdata(client);
	struct oem_bd6098_led_data *led;
	int i;

	BD6098_V_LOG("[IN]");

	for (i = 0; i < LED_TYPE_MAX; i++) {
		led = &data->led_data[i];
		flush_work(&led->work);
	}
	INIT_COMPLETION(data->wait_resume);
	data->in_suspend = true;

	BD6098_V_LOG("[OUT]");

	return 0;
}

static int oem_bd6098_resume(struct i2c_client *client)
{
	struct oem_bd6098_data *data = i2c_get_clientdata(client);

	BD6098_V_LOG("[IN]");

	data->in_suspend = false;
	complete_all(&data->wait_resume);

	BD6098_V_LOG("[OUT]");
	return 0;
}
#else
#define oem_bd6098_suspend    NULL
#define oem_bd6098_resume     NULL
#endif /* CONFIG_PM */

#define BOOT_LED_STAT_BL_ON     0x00000001
#define BOOT_LED_STAT_3COLOR_ON 0x00000002
#define BOOT_LED_STAT_OFF       0x00000000
static unsigned long led_stat_aboot = BOOT_LED_STAT_OFF;
static bool keep_device_state = false;
static int __init oem_bd6098_set_led_stat_aboot(char *buf)
{
	int ret;
	BD6098_V_LOG("[IN]");
	ret = kstrtoul(buf, 0, &led_stat_aboot);
	if (ret) {
		BD6098_E_LOG("failed to kstrtoul ret=%d", ret);
	}
	if (led_stat_aboot) {
		keep_device_state = true;
	}
	BD6098_V_LOG("[OUT] led_stat_aboot=0x%lx", led_stat_aboot);
	return 0;
}
early_param("led_stat", oem_bd6098_set_led_stat_aboot);

static int oem_bd6098_remove(struct i2c_client *client)
{
	struct oem_bd6098_data *data = i2c_get_clientdata(client);
	struct oem_bd6098_led_data *led;
	int i;

	BD6098_V_LOG("[IN]");

	oem_bd6098_data = NULL;

	if (data) {
		misc_deregister(&leds_device);

		for (i = 0; i < LED_TYPE_MAX; i++) {
			led = &data->led_data[i];

			if (led->cdev_registered) {
				led_classdev_unregister(&led->cdev);
				flush_work(&led->work);
				led->cdev_registered = false;
			}
			mutex_destroy(&led->param_next_lock);
		}
		destroy_workqueue(data->work_queue);
		data->work_queue = NULL;
		if (gpio_is_valid(data->keyled_gpio)) {
			gpio_free(data->keyled_gpio);
		}
#ifndef KEEP_DEVICE_STATE_ON_PROBE
		if (!keep_device_state) {
			gpio_free(data->reset_gpio);
		}
#endif
		i2c_set_clientdata(client, NULL);
		mutex_destroy(&data->param_lock);
		mutex_destroy(&data->control_lock);
		wake_lock_destroy(&data->wake_lock);
		kfree(data);
	}

	BD6098_V_LOG("[OUT]");

	return 0;
}

static void oem_bd6098_init_pwm_enable(void)
{
	BD6098_V_LOG("[IN]");
#ifdef ENABLE_PWM
	switch (OEM_get_board()) {
	case OEM_BOARD3_TYPE:
		BD6098_N_LOG("pwm disable.");
		break;
	default:
		BD6098_N_LOG("pwm enable.");
		oem_bd6098_reg_info_save[0x01].value |= 0x80;
		oem_bd6098_reg_info_save[0x01].written = false;
		break;
	}
#else
	BD6098_N_LOG("pwm disable.");
#endif
	BD6098_V_LOG("[OUT]");
}

static int oem_bd6098_init_client(struct i2c_client *client)
{
	struct oem_bd6098_data *data = i2c_get_clientdata(client);
#ifdef FORCE_WLED_ON
	uint8_t read_buf;
#endif
	int gval = 0;

	BD6098_V_LOG("[IN] client=0x%p", client);

	if (led_stat_aboot) {
		if (led_stat_aboot & BOOT_LED_STAT_BL_ON) {
			data->power_state |= 1 << LED_TYPE_MLED;
			oem_bd6098_reg_info_save[0x02].value |= 0x01;
		}
		if (led_stat_aboot & BOOT_LED_STAT_3COLOR_ON) {
			data->power_state |= 1 << LED_TYPE_RGB;
			oem_bd6098_reg_info_save[0x00].value |= 0x30;
		}
		BD6098_V_LOG("led_stat_aboot=0x%lx data->power_state=0x%x",
			led_stat_aboot, data->power_state);
	}
	gval = gpio_get_value(data->reset_gpio);
	BD6098_V_LOG("gpio(%d) = %d", data->reset_gpio, gval);

#ifndef KEEP_DEVICE_STATE_ON_PROBE
	if (!keep_device_state) {
		// GPIO	HARDWARE Reset L->H
		BD6098_V_LOG("gpio_set_value(data->reset_gpio(%d), 0)", data->reset_gpio);
		gpio_set_value(data->reset_gpio, 0);
		BD6098_V_LOG("usleep(%d)", data->reset_delay[0]);
		usleep(data->reset_delay[0]);
	}
#endif
	BD6098_V_LOG("gpio_set_value(data->reset_gpio(%d), 1)", data->reset_gpio);
	gpio_set_value(data->reset_gpio, 1);
	BD6098_V_LOG("usleep(%d)", data->reset_delay[1]);
	usleep(data->reset_delay[1]);

#ifndef KEEP_DEVICE_STATE_ON_PROBE
	if (!keep_device_state) {
		oem_bd6098_i2c_masked_write(client, 0x00, 0x00, 0x00);
	}
#endif
	oem_bd6098_i2c_masked_write(client, 0x01, 0x00, 0x00);
#ifndef KEEP_DEVICE_STATE_ON_PROBE
	if (!keep_device_state) {
		oem_bd6098_i2c_masked_write(client, 0x02, 0x00, 0x00);
		oem_bd6098_i2c_masked_write(client, 0x03, 0x00, 0x00);
	}
#endif
	oem_bd6098_i2c_masked_write(client, 0x06, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x07, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x08, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x0A, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x0B, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x0C, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x0D, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x0E, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x0F, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x10, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x11, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x12, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x13, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x14, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x15, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x16, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x17, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x18, 0x00, 0x00);
	oem_bd6098_i2c_masked_write(client, 0x19, 0x00, 0x00);
#ifndef KEEP_DEVICE_STATE_ON_PROBE
	if (!keep_device_state) {
		oem_bd6098_i2c_masked_write(client, 0x1C, 0x00, 0x00);
	}
#endif
	oem_bd6098_i2c_masked_write(client, 0x1D, 0x00, 0x00);

#ifdef FORCE_WLED_ON
	// force WLED ON
	BD6098_V_LOG("force WLED ON");
	oem_bd6098_i2c_read(client, 0x15, &read_buf, 1);
	oem_bd6098_i2c_write(client, 0x03, 0x7F);
	oem_bd6098_i2c_write(client, 0x02, 0x01);			// WLED ON
	usleep(50000);
#endif
	BD6098_V_LOG("[OUT]");

	return 0;
}

static int oem_bd6098_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct oem_bd6098_data *data;
	struct oem_bd6098_led_data *led;
	struct device_node *node;
	struct device_node *temp;
	const char *led_label;
	const char *linux_name;
	int i;
	int err = 0;

	BD6098_V_LOG("[IN] client=0x%p", client);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		BD6098_E_LOG("fail i2c_check_functionality");
		goto exit;
	}

	node = client->dev.of_node;
	if (node == NULL){
		BD6098_E_LOG("client->dev.of_node == null");
		err = -ENODEV;
		goto exit;
	}

	data = kzalloc(sizeof(struct oem_bd6098_data), GFP_KERNEL);
	if (!data) {
		BD6098_E_LOG("Failed kzalloc");
		err = -ENOMEM;
		goto exit;
	}

	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, BD6098_DRV_NAME);
	mutex_init(&data->control_lock);
	mutex_init(&data->param_lock);
	data->client = client;
	i2c_set_clientdata(client, data);
	INIT_DELAYED_WORK(&data->blink_work, oem_bd6098_blink_work);
	alarm_init(&data->blink_alarm, ALARM_REALTIME, oem_bd6098_blink_alarm_callback);
	init_completion(&data->wait_resume);

	data->work_queue = create_singlethread_workqueue(BD6098_DRV_NAME);
	if (!data->work_queue) {
		err = -ENODEV;
		goto fail_create_workqueue;
	}

	data->keyled_gpio = -ENOSYS;
	temp = NULL;
	while ((temp = of_get_next_child(node, temp))) {

		BD6098_V_LOG("read label");
		err = of_property_read_string(temp, "label", &led_label);
		if (err < 0) {
			BD6098_E_LOG("Failure reading label, Dev=[0x%08x] err=[%d]", (unsigned int)&client->dev, err);
			continue;
		}
		BD6098_V_LOG("read linux,name");
		err = of_property_read_string(temp, "linux,name", &linux_name);
		if (err < 0) {
			BD6098_E_LOG("Failure reading linux,name, Dev=[0x%08x] err=[%d]", (unsigned int)&client->dev, err);
			continue;
		}

		BD6098_V_LOG("label=%s linux,name=%s", led_label, linux_name);
		led = NULL;
		if (strcmp(led_label, LABEL_MLED) == 0) {
			BD6098_V_LOG("probe MLED");
			led = &data->led_data[LED_TYPE_MLED];
			led->led_type = LED_TYPE_MLED;
			led->ctrl_func = oem_bd6098_ctrl_mled;
			led->workqueue = system_wq;
		} else if (strcmp(led_label, LABEL_RGB) == 0) {
			BD6098_V_LOG("probe RGB");
			led = &data->led_data[LED_TYPE_RGB];
			led->led_type = LED_TYPE_RGB;
			led->ctrl_func = oem_bd6098_ctrl_rgb;
			led->workqueue = data->work_queue;
			if (of_property_read_bool(client->dev.of_node, "use-mdm-led")) {
				BD6098_D_LOG("use mdm-led");
				data->mdm_led = mdm_led_create();
				if (!data->mdm_led) {
					BD6098_E_LOG("failed to mdm_led_init");
					goto fail_id_check;
				}
			}
			if (of_property_read_bool(client->dev.of_node, "use-alarm-blink")) {
				data->use_alarm_blink = true;
			}
		} else if (strcmp(led_label, LABEL_CLED) == 0) {
			BD6098_V_LOG("probe CLED");
			led = &data->led_data[LED_TYPE_CLED];
			led->led_type = LED_TYPE_CLED;
			led->ctrl_func = oem_bd6098_ctrl_cled;
			led->workqueue = system_wq;
		} else if (strcmp(led_label, LABEL_KEYLED) == 0) {
			BD6098_V_LOG("probe KEYLED");
			led = &data->led_data[LED_TYPE_KEYLED];
			led->led_type = LED_TYPE_KEYLED;
			led->ctrl_func = oem_bd6098_ctrl_keyled;
			led->workqueue = system_wq;
			data->keyled_gpio = of_get_named_gpio(client->dev.of_node, "kc,keyled-gpio", 0);
			if (!gpio_is_valid(data->keyled_gpio)) {
				BD6098_E_LOG("No valid KEYLED GPIO specified %d", data->keyled_gpio);
				goto fail_id_check;
			}
		} else if (strcmp(led_label, LABEL_SLED) == 0) {
			BD6098_V_LOG("probe SLED");
			led = &data->led_data[LED_TYPE_SLED];
			led->led_type = LED_TYPE_SLED;
			led->ctrl_func = oem_bd6098_ctrl_sled;
			led->workqueue = system_wq;
		} else {
			BD6098_N_LOG("unknown label:%s", led_label);
		}

		if (led) {
			mutex_init(&led->param_next_lock);
			led->cdev.brightness_set = led_set;
			led->cdev.brightness_get = led_get;
			led->cdev.name			 = linux_name;
			led->cdev.max_brightness = BRIGHTNESS_MAX;
			led->cdev.sensor_brightness = -1;
			led->cdev.sensor_brightness_set = led_set_sensor;
			led->cdev.sensor_brightness_get = led_get_sensor;
			led->cdev.usr_brightness_req = 55;
			INIT_WORK(&led->work, oem_bd6098_work);
			led->parent = data;
			err = led_classdev_register(&client->dev, &led->cdev);
			if (err < 0) {
				BD6098_E_LOG("unable to register led %s", led->cdev.name);
				goto fail_id_check;
			}
			led->cdev_registered = true;
		}
	}

	err = misc_register(&leds_device);
	if (err < 0) {
		BD6098_E_LOG("unable to register misc device");
		goto fail_misc_register;
	}

	data->reset_gpio = of_get_named_gpio(client->dev.of_node, "kc,reset-gpio", 0);
	if (!gpio_is_valid(data->reset_gpio)) {
		BD6098_E_LOG("No valid RESET GPIO specified %d", data->reset_gpio);
		err = -ENODEV;
		goto fail_get_reset_gpio;
	}

	err = of_property_read_u32_array(client->dev.of_node, "kc,reset-delay",
		data->reset_delay, ARRAY_SIZE(data->reset_delay));
	BD6098_V_LOG("read kc,reset-delay [0]=%d [1]=%d err=%d",
		data->reset_delay[0], data->reset_delay[1], err);
	if (err < 0) {
		BD6098_E_LOG("failed to read reset-delay err=%d", err);
		err = -ENODEV;
		goto fail_get_reset_delay;
	}

#ifndef KEEP_DEVICE_STATE_ON_PROBE
	if (!keep_device_state) {
		err = gpio_request(data->reset_gpio, BD6098_DRV_NAME);
		BD6098_V_LOG("gpio_request GPIO=%d err=%d", data->reset_gpio, err);
		if (err < 0) {
			BD6098_E_LOG("failed to request GPIO=%d, ret=%d",
					data->reset_gpio,
					err);
			goto fail_request_reset_gpio;
		}
	}
#endif

	if (gpio_is_valid(data->keyled_gpio)) {
		err = gpio_request(data->keyled_gpio, BD6098_DRV_NAME);
		BD6098_V_LOG("gpio_request GPIO=%d err=%d", data->keyled_gpio, err);
		if (err < 0) {
			BD6098_E_LOG("failed to request GPIO=%d, ret=%d",
					data->keyled_gpio,
					err);
			goto fail_request_keyled_gpio;
		}
	}

	oem_bd6098_init_pwm_enable();

	err = oem_bd6098_init_client(client);
	if (err)
	{
		BD6098_E_LOG("Failed BD6098_init_client");
		goto fail_init_client;
	}

	data->param.keyled_threshold = KEYLED_THRESHOLD_DEFAULT;
	data->colorvariation = COLORVARIATION_DEFAULT;

	oem_bd6098_data = data;
	ext_ctrl_mode = 0;

	BD6098_V_LOG("[OUT]");

	return 0;

fail_init_client:
	if (gpio_is_valid(data->keyled_gpio)) {
		gpio_free(data->keyled_gpio);
	}
fail_request_keyled_gpio:
#ifndef KEEP_DEVICE_STATE_ON_PROBE
	if (!keep_device_state) {
		gpio_free(data->reset_gpio);
	}
fail_request_reset_gpio:
#endif
fail_get_reset_delay:
fail_get_reset_gpio:
	misc_deregister(&leds_device);
fail_misc_register:
fail_id_check:
	if (data->mdm_led) {
		mdm_led_destroy(data->mdm_led);
		data->mdm_led = NULL;
	}
	for (i = 0; i < LED_TYPE_MAX; i++) {
		led = &data->led_data[i];

		if (led->cdev_registered) {
			led_classdev_unregister(&led->cdev);
			flush_work(&led->work);
			led->cdev_registered = false;
		}
		mutex_destroy(&led->param_next_lock);
	}
	destroy_workqueue(data->work_queue);
	data->work_queue = NULL;
fail_create_workqueue:
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&data->param_lock);
	mutex_destroy(&data->control_lock);
	wake_lock_destroy(&data->wake_lock);
	kfree(data);
exit:
	BD6098_E_LOG("[OUT] err=%d", err);
	return err;
}

static const struct i2c_device_id oem_bd6098_id[] = {
	{ BD6098_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, oem_bd6098_id);

static struct of_device_id oem_bd6098_match_table[] = {
	{ .compatible = BD6098_DRV_NAME,},
	{ },
};

static struct i2c_driver oem_bd6098_driver = {
	.driver = {
		.name   = BD6098_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = oem_bd6098_match_table,
	},
	.suspend = oem_bd6098_suspend,
	.resume = oem_bd6098_resume,
	.probe  = oem_bd6098_probe,
	.remove = oem_bd6098_remove,
	.id_table = oem_bd6098_id,
};

static int32_t __init oem_bd6098_led_init(void)
{
	int32_t rc = 0;

	BD6098_V_LOG("[IN]");

	rc = i2c_add_driver(&oem_bd6098_driver);
	if (rc != 0) {
		BD6098_E_LOG("can't add i2c driver");
		rc = -ENOTSUPP;
		return rc;
	}

	BD6098_V_LOG("[OUT]");

	return rc;

}

static void __exit oem_bd6098_led_exit(void)
{
	BD6098_V_LOG("[IN]");

	i2c_del_driver(&oem_bd6098_driver);

	BD6098_V_LOG("[OUT]");
}

int32_t light_led_disp_set_panel(e_light_main_wled_disp disp_status, e_light_lcd_panel panel_class)
{
	BD6098_V_LOG("[IN] panel_class=0x%x", panel_class);
	switch( panel_class ){
	case LIGHT_LCD_PANEL0:
		BD6098_V_LOG("panel class = LIGHT_LCD_PANEL0");
		break;
	default:
		BD6098_E_LOG("unknown panel class");
		break;
	}

	return light_led_disp_set(disp_status);
}
EXPORT_SYMBOL(light_led_disp_set_panel);

#ifndef DISABLE_DISP_DETECT

static void oem_bd6098_queue_mled_work(void)
{
	struct oem_bd6098_data *data = oem_bd6098_data;
	struct oem_bd6098_led_data *led;

	if (data) {
		led = &data->led_data[LED_TYPE_MLED];
		if (led->cdev_registered) {
			if (led->param_next.value) {
				queue_work(led->workqueue, &led->work);
			}
		}
	}
}

static atomic_t g_display_detect = ATOMIC_INIT(0);

static bool light_led_disp_enabled(void)
{
	return (atomic_read(&g_display_detect) == 1) ? true : false;
}

int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
	int32_t ret = 0;

	BD6098_V_LOG("[IN] disp_status=[0x%x]", (uint32_t)disp_status);

	if ((atomic_read(&g_display_detect)) != 0) {
		BD6098_V_LOG("already determined.");
		return ret;
	}

	switch(disp_status) {
	case LIGHT_MAIN_WLED_LCD_EN:
		BD6098_N_LOG("LIGHT_MAIN_WLED_LCD_EN");
		atomic_set(&g_display_detect,1);
		oem_bd6098_queue_mled_work();
		break;
	case LIGHT_MAIN_WLED_LCD_DIS:
		BD6098_N_LOG("LIGHT_MAIN_WLED_LCD_DIS");
		atomic_set(&g_display_detect,-1);
		break;
	default:
		break;
	}

	BD6098_V_LOG("[OUT] ret=%d", ret);
	return ret;
}
EXPORT_SYMBOL(light_led_disp_set);

#else  /* DISABLE_DISP_DETECT */

static bool light_led_disp_enabled(void)
{
	return true;
}

int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
	BD6098_N_LOG("DISABLE_DISP_DETECT");
	return 0;
}
EXPORT_SYMBOL(light_led_disp_set);

#endif  /* DISABLE_DISP_DETECT */

#ifdef DISCARD_BL_SUBLCDPOWER_OFF

static void oem_bd6098_queue_sled_work(void)
{
	struct oem_bd6098_data *data = oem_bd6098_data;
	struct oem_bd6098_led_data *led;

	if (data) {
		led = &data->led_data[LED_TYPE_SLED];
		if (led->cdev_registered) {
			if (led->param_next.value) {
				queue_work(led->workqueue, &led->work);
			}
		}
	}
}

static atomic_t g_detect_sublcd_pwren = ATOMIC_INIT(0);

static bool light_sublcd_power_enabled(void)
{
	return (atomic_read(&g_detect_sublcd_pwren) == 1) ? true : false;
}

int32_t light_led_subdisp_power_set(e_light_sub_wled_disp disp_status)
{
	int32_t ret = 0;

	BD6098_V_LOG("[IN] disp_status=[0x%x]", (uint32_t)disp_status);

	switch(disp_status) {
	case LIGHT_SUB_WLED_LCD_PWREN:
		BD6098_V_LOG("LIGHT_SUB_WLED_LCD_PWREN");
		atomic_set(&g_detect_sublcd_pwren,1);
		oem_bd6098_queue_sled_work();
		break;
	case LIGHT_SUB_WLED_LCD_PWRDIS:
		BD6098_V_LOG("LIGHT_SUB_WLED_LCD_PWRDIS");
		atomic_set(&g_detect_sublcd_pwren,-1);
		break;
	default:
		break;
	}

	BD6098_V_LOG("[OUT] ret=%d", ret);
	return ret;
}
EXPORT_SYMBOL(light_led_subdisp_power_set);

#else /*  DISCARD_BL_SUBLCDPOWER_OFF */

static bool light_sublcd_power_enabled(void)
{
	return true;
}

int32_t light_led_subdisp_power_set(e_light_sub_wled_disp disp_status)
{
	BD6098_N_LOG("invalid DISCARD_BL_SUBLCDPOWER_OFF disp_status=0x%x",  (uint32_t)disp_status);
	return 0;
}
EXPORT_SYMBOL(light_led_subdisp_power_set);

#endif /*  DISCARD_BL_SUBLCDPOWER_OFF */

module_init(oem_bd6098_led_init);
module_exit(oem_bd6098_led_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("LED");
MODULE_LICENSE("GPL");
