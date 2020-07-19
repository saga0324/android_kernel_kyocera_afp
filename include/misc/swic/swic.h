/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */

#ifndef SWIC_H
#define SWIC_H

typedef enum {
	SWIC_UNINITIALIZE		= 0x00,
	SWIC_NO_ACCESSORY		= 0xF8,
	SWIC_AC_ADAPTER			= 0xFE,
	SWIC_USB_MODE			= 0xFC,
	SWIC_AUDIO_MIC_MONO		= 0xF0,
	SWIC_AUDIO_CHG_MIC_MONO	= 0xE4,
	SWIC_FAST_CHARGER		= 0xDE,
	SWIC_CARKIT_TYPE2		= 0xDC,
	SWIC_VIDEO_AUDIO		= 0xD0,
	SWIC_VIDEO_AUDIO_CHG	= 0xD4,
	SWIC_AUDIO_MIC_STEREO	= 0xC0,
	SWIC_AUDIO_CHG_STEREO	= 0xC4,
	SWIC_STD_CHARGER		= 0xBE,
	SWIC_CARKIT_TYPE1		= 0xBC,
	SWIC_UART				= 0xB0,
	SWIC_RID_A				= 0xAC,
	SWIC_AUDIO_STEREO		= 0xA0,
	SWIC_RID_B				= 0x9C,
	SWIC_RID_C				= 0x8C,
	SWIC_MIC_STEREO			= 0xC8,
} swic_accessory_enum;

typedef void (*swic_event_func)(swic_accessory_enum accessoryinfo);

struct swic_event_callback {
    swic_event_func fn;
};

extern int32_t swic_reg_cbfunc(struct swic_event_callback* cb);
extern int32_t swic_unreg_cbfunc(struct swic_event_callback* cb);
extern void swic_set_mic_exist(bool mic);
extern swic_accessory_enum swic_get_accessory(void);

static inline bool swic_is_fix_accessory(u8 value)
{
	bool ret = false;
	switch (value) {
	case SWIC_AC_ADAPTER :
	case SWIC_USB_MODE :
	case SWIC_AUDIO_MIC_MONO :
	case SWIC_AUDIO_CHG_MIC_MONO :
	case SWIC_FAST_CHARGER :
	case SWIC_CARKIT_TYPE2 :
	case SWIC_VIDEO_AUDIO :
	case SWIC_VIDEO_AUDIO_CHG :
	case SWIC_AUDIO_MIC_STEREO :
	case SWIC_AUDIO_CHG_STEREO :
	case SWIC_STD_CHARGER :
	case SWIC_CARKIT_TYPE1 :
	case SWIC_UART :
	case SWIC_RID_A :
	case SWIC_AUDIO_STEREO :
	case SWIC_RID_B :
	case SWIC_RID_C :
	case SWIC_MIC_STEREO :
	case SWIC_NO_ACCESSORY :
		ret = true;
		break;
	case SWIC_UNINITIALIZE :
	default :
		break;
	}
	return ret;
}

#endif
