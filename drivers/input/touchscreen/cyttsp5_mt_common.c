/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 */
/*
 * cyttsp5_mt_common.c
 * Cypress TrueTouch(TM) Standard Product V5 Multi-Touch Reports Module.
 * For use with Cypress touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2012-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp5_regs.h"

#define CYTTSP5_MT_NAME "cyttsp5_mt"

#define MT_PARAM_SIGNAL(md, sig_ost) PARAM_SIGNAL(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_MIN(md, sig_ost) PARAM_MIN(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_MAX(md, sig_ost) PARAM_MAX(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_FUZZ(md, sig_ost) PARAM_FUZZ(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_FLAT(md, sig_ost) PARAM_FLAT(md->pdata->frmwrk, sig_ost)

#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
#include <linux/wakelock.h>
static struct wake_lock touchpanel_wake_lock;
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
static struct cyttsp5_core_commands *cmd;
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

static void cyttsp5_mt_lift_all(struct cyttsp5_mt_data *md)
{
	int max = md->si->tch_abs[CY_TCH_T].max;
	struct cyttsp5_core_data *cd = dev_get_drvdata(md->dev);

	dev_dbg(md->dev,"%s: touch_cnt=%d, num_prv_rec=%d\n", __func__, cd->touch_cnt, md->num_prv_rec);

	if (md->num_prv_rec != 0) {
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, max);
		input_sync(md->input);
		md->num_prv_rec = 0;
	}
	cd->touch_cnt = 0;
}

static void cyttsp5_get_touch_axis(struct cyttsp5_mt_data *md,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		dev_vdbg(md->dev,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	dev_vdbg(md->dev,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void cyttsp5_get_touch_hdr(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_mode)
{
	struct device *dev = md->dev;
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_hdr hdr;

	for (hdr = CY_TCH_TIME; hdr < CY_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}

	dev_dbg(dev,
		"%s: time=%X tch_num=%d lo=%d noise=%d counter=%d\n",
		__func__,
		touch->hdr[CY_TCH_TIME],
		touch->hdr[CY_TCH_NUM],
		touch->hdr[CY_TCH_LO],
		touch->hdr[CY_TCH_NOISE],
		touch->hdr[CY_TCH_COUNTER]);
}

static void cyttsp5_get_touch_record(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct device *dev = md->dev;
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_abs abs;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}
}

static void cyttsp5_mt_process_touch(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch)
{
	struct device *dev = md->dev;
	struct cyttsp5_sysinfo *si = md->si;
	int tmp;
	bool flipped;


	/* Orientation is signed */
	touch->abs[CY_TCH_OR] = (int8_t)touch->abs[CY_TCH_OR];

	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		tmp = touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_X] = touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_Y] = tmp;
		if (touch->abs[CY_TCH_OR] > 0)
			touch->abs[CY_TCH_OR] =
				md->or_max - touch->abs[CY_TCH_OR];
		else
			touch->abs[CY_TCH_OR] =
				md->or_min - touch->abs[CY_TCH_OR];
		flipped = true;
	} else
		flipped = false;

	if (md->pdata->flags & CY_MT_FLAG_INV_X) {
		if (flipped)
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_X];
		else
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_OR] *= -1;
	}
	if (md->pdata->flags & CY_MT_FLAG_INV_Y) {
		if (flipped)
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_Y];
		else
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_OR] *= -1;
	}

	/* Convert MAJOR/MINOR from mm to resolution */
	tmp = touch->abs[CY_TCH_MAJ] * 10 * si->sensing_conf_data.res_x;
	touch->abs[CY_TCH_MAJ] = tmp / si->sensing_conf_data.len_x;
	tmp = touch->abs[CY_TCH_MIN] * 10 * si->sensing_conf_data.res_x;
	touch->abs[CY_TCH_MIN] = tmp / si->sensing_conf_data.len_x;

	dev_vdbg(dev, "%s: flip=%s inv-x=%s inv-y=%s x=%04X(%d) y=%04X(%d)\n",
		__func__, flipped ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_X ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_Y ? "true" : "false",
		touch->abs[CY_TCH_X], touch->abs[CY_TCH_X],
		touch->abs[CY_TCH_Y], touch->abs[CY_TCH_Y]);
}

static void cyttsp5_report_event(struct cyttsp5_mt_data *md, int event,
		int value)
{
	int sig = MT_PARAM_SIGNAL(md, event);

	if (sig != CY_IGNORE_VALUE)
		input_report_abs(md->input, sig, value);
}

static void cyttsp5_get_mt_touches(struct cyttsp5_mt_data *md,
		struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct device *dev = md->dev;
	struct cyttsp5_sysinfo *si = md->si;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int sig;
	int i, j, t = 0;
	DECLARE_BITMAP(ids, si->tch_abs[CY_TCH_T].max);
	int mt_sync_count = 0;
	u8 *tch_addr;

	bitmap_zero(ids, si->tch_abs[CY_TCH_T].max);
	memset(tch->abs, 0, sizeof(tch->abs));

	for (i = 0; i < num_cur_tch; i++) {
		tch_addr = si->xy_data + (i * si->desc.tch_record_size);
		cyttsp5_get_touch_record(md, tch, tch_addr);

		/*  Discard proximity event */
		if (tch->abs[CY_TCH_O] == CY_OBJ_PROXIMITY) {
			dev_vdbg(dev, "%s: Discarding proximity event\n",
					__func__);
			continue;
		}

		if ((tch->abs[CY_TCH_O] != CY_OBJ_STANDARD_FINGER)
		&&  (tch->abs[CY_TCH_O] != CY_OBJ_GLOVE)) {
			dev_dbg(dev, "%s: Discarding unknown event\n",
				__func__);
			continue;
		}

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
		dev_dbg(dev, "%s: touch type=%d\n",__func__,tch->abs[CY_TCH_O]);
		if (tch->abs[CY_TCH_O] == CY_OBJ_STANDARD_FINGER) {
			md->check_finger_touch = true;
		}
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

		/* Validate track_id */
		t = tch->abs[CY_TCH_T];
		if (t < md->t_min || t > md->t_max) {
			dev_err(dev, "%s: tch=%d -> bad trk_id=%d max_id=%d\n",
				__func__, i, t, md->t_max);
			if (md->mt_function.input_sync)
				md->mt_function.input_sync(md->input);
			mt_sync_count++;
			continue;
		}

		/* Lift-off */
		if (tch->abs[CY_TCH_E] == CY_EV_LIFTOFF) {
			dev_dbg(dev, "%s: t=%d e=%d lift-off\n",
				__func__, t, tch->abs[CY_TCH_E]);
			goto cyttsp5_get_mt_touches_pr_tch;
		}

		/* Process touch */
		cyttsp5_mt_process_touch(md, tch);

		/* use 0 based track id's */
		t -= md->t_min;

		sig = MT_PARAM_SIGNAL(md, CY_ABS_ID_OST);
		if (sig != CY_IGNORE_VALUE) {
			if (md->mt_function.input_report)
				md->mt_function.input_report(md->input, sig,
						t, tch->abs[CY_TCH_O]);
			__set_bit(t, ids);
		}

		if(tch->abs[CY_TCH_P] > CY_MAX_PRESSURE_SIZE)
			tch->abs[CY_TCH_P] = CY_MAX_PRESSURE_SIZE;

		/* If touch type is hover, send P as distance, reset P */
		if (tch->abs[CY_TCH_O] == CY_OBJ_HOVER) {
			/* CY_ABS_D_OST signal must be in touch framework */
			cyttsp5_report_event(md, CY_ABS_D_OST,
					tch->abs[CY_TCH_P]);
			tch->abs[CY_TCH_P] = 0;
		} else
			cyttsp5_report_event(md, CY_ABS_D_OST, 0);


		/* all devices: position and pressure fields */
		for (j = 0; j <= CY_ABS_W_OST; j++) {
			if (!si->tch_abs[j].report)
				continue;
			cyttsp5_report_event(md, CY_ABS_X_OST + j,
					tch->abs[CY_TCH_X + j]);
		}

		/* Get the extended touch fields */
		for (j = 0; j < CY_NUM_EXT_TCH_FIELDS; j++) {
			if (!si->tch_abs[CY_ABS_MAJ_OST + j].report)
				continue;
			if(tch->abs[CY_TCH_MAJ + j] < CY_MAX_AREA_SIZE)
				cyttsp5_report_event(md, CY_ABS_MAJ_OST + j, tch->abs[CY_TCH_MAJ + j]);
			else
				cyttsp5_report_event(md, CY_ABS_MAJ_OST + j, CY_MAX_AREA_SIZE);
		}
		if (md->mt_function.input_sync)
			md->mt_function.input_sync(md->input);
		mt_sync_count++;

cyttsp5_get_mt_touches_pr_tch:
		dev_dbg(dev,
			"%s: t=%d x=%d y=%d z=%d M=%d m=%d o=%d e=%d obj=%d tip=%d\n",
			__func__, t,
			tch->abs[CY_TCH_X],
			tch->abs[CY_TCH_Y],
			tch->abs[CY_TCH_P],
			tch->abs[CY_TCH_MAJ],
			tch->abs[CY_TCH_MIN],
			tch->abs[CY_TCH_OR],
			tch->abs[CY_TCH_E],
			tch->abs[CY_TCH_O],
			tch->abs[CY_TCH_TIP]);
		if (tch->abs[CY_TCH_E] != CY_EV_LIFTOFF) {
			cd->touch_info[t].x  = tch->abs[CY_TCH_X];
			cd->touch_info[t].y  = tch->abs[CY_TCH_Y];
			cd->touch_info[t].wx = tch->abs[CY_TCH_MAJ];
			cd->touch_info[t].wy = tch->abs[CY_TCH_MIN];
			cd->touch_info[t].z  = tch->abs[CY_TCH_P];
		} else {
			cd->touch_info[t].x  = 0;
			cd->touch_info[t].y  = 0;
			cd->touch_info[t].wx = 0;
			cd->touch_info[t].wy = 0;
			cd->touch_info[t].z  = 0;
		}
	}

	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input,
				si->tch_abs[CY_TCH_T].max, mt_sync_count, ids);

	md->num_prv_rec = num_cur_tch;
	cd->touch_cnt = num_cur_tch;
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5_mt_data *md)
{
	struct device *dev = md->dev;
	struct cyttsp5_sysinfo *si = md->si;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int max_tch = si->sensing_conf_data.max_tch;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;
	int rc = 0;

	cyttsp5_get_touch_hdr(md, &tch, si->xy_mode + 3);

	if(cd->ts_event_control){
		pr_notice("%s is skip\n", __func__);
		if(cd->touch_cnt)
			cyttsp5_mt_lift_all(md);
		return 0;
	}

	num_cur_tch = tch.hdr[CY_TCH_NUM];
	if (num_cur_tch > max_tch) {
		dev_err(dev, "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = max_tch;
	}

	if (tch.hdr[CY_TCH_LO]) {
		dev_dbg(dev, "%s: Large area detected\n", __func__);
		if (md->pdata->flags & CY_MT_FLAG_NO_TOUCH_ON_LO)
			num_cur_tch = 0;
#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
		md->check_large_touch = true;
	} else {
		md->check_large_touch = false;
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */
	}

#ifdef CONFIG_TOUCHSCREEN_UNDERWATER
	if(cd->easy_wakeup_gesture){
		if(unlikely(IS_UNDER_WATER(si->xy_mode[5]))) {
			num_cur_tch = 0;
			if(unlikely(!cd->last_under_water_state)){
				pr_info("%s: UnderWater\n", __func__);
				cd->last_under_water_state = true;
				input_report_key(md->input, BTN_WATER, 1);
				input_sync(md->input);
			}
		} else {
			if(unlikely(cd->last_under_water_state)){
				pr_info("%s: Air detection\n", __func__);
				cd->last_under_water_state = false;
				input_report_key(md->input, BTN_WATER, 0);
				input_sync(md->input);
			}
		}
	}
#endif

	if (num_cur_tch == 0 && md->num_prv_rec == 0)
		goto cyttsp5_xy_worker_exit;

	/* extract xy_data for all currently reported touches */
	dev_vdbg(dev, "%s: extract data num_cur_tch=%d\n", __func__,
		num_cur_tch);
	if (num_cur_tch)
		cyttsp5_get_mt_touches(md, &tch, num_cur_tch);
	else
		cyttsp5_mt_lift_all(md);

	rc = 0;

cyttsp5_xy_worker_exit:
	return rc;
}

static void cyttsp5_mt_send_dummy_event(struct cyttsp5_mt_data *md)
{
	unsigned long ids = 0;

	/* for easy wakeup */
	if (md->mt_function.input_report)
		md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
			0, CY_OBJ_STANDARD_FINGER);
	if (md->mt_function.input_sync)
		md->mt_function.input_sync(md->input);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 0, 1, &ids);
	if (md->mt_function.report_slot_liftoff)
		md->mt_function.report_slot_liftoff(md, 1);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 1, 1, &ids);
}

static int cyttsp5_mt_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	int rc;
#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
	int temp_check_finger = 0;
	int temp_check_large = 0;
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

	if (md->si->xy_mode[2] !=  md->si->desc.tch_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&md->mt_lock);
#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
	wake_lock_timeout(&touchpanel_wake_lock, 2 * HZ); 
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
	temp_check_finger = md->check_finger_touch;
	temp_check_large = md->check_large_touch;
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

	rc = cyttsp5_xy_worker(md);

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
	if (!temp_check_finger && md->check_finger_touch){
		dev_info(dev, "%s detect 1st finger touch\n", __func__);
		cmd->request_set_touchmode_enabled(dev, CY_TME_FINGER_ONLY);
	}
	if (temp_check_large && !md->check_large_touch) {
		md->request_glove_touch = true;
	}
	if (md->request_glove_touch && !md->num_prv_rec) {
		dev_info(dev, "%s detect Large touch\n", __func__);
		md->request_glove_touch = false;
		cmd->request_set_touchmode_enabled(dev, CY_TME_FINGER_AND_GLOVE);
		md->check_finger_touch = false;
	}
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

	mutex_unlock(&md->mt_lock);
	if (rc < 0)
		dev_err(dev, "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_mt_wake_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
	wake_lock_timeout(&touchpanel_wake_lock, 2 * HZ); 
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */
	cyttsp5_mt_send_dummy_event(md);
	mutex_unlock(&md->mt_lock);
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
static int cyttsp5_mt_glove_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	cmd->request_set_touchmode_enabled(dev, CY_TME_FINGER_AND_GLOVE);
	mutex_lock(&md->mt_lock);
	md->check_finger_touch = false;
	mutex_unlock(&md->mt_lock);

	return 0;
}
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL
static int cyttsp5_mt_finger_attention(struct device *dev)
{
	cmd->request_set_touchmode_enabled(dev, CY_TME_FINGER_ONLY);

	return 0;
}
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL */

static int cyttsp5_startup_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_suspend_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_lift_all(md);
	md->is_suspended = true;
	mutex_unlock(&md->mt_lock);

	pm_runtime_put(dev);

	return 0;
}

static int cyttsp5_mt_resume_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	pm_runtime_get(dev);

	mutex_lock(&md->mt_lock);
	md->is_suspended = false;
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	pm_runtime_get_sync(dev);

	mutex_lock(&md->mt_lock);
	md->is_suspended = false;
	mutex_unlock(&md->mt_lock);

	dev_vdbg(dev, "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_IRQ, CYTTSP5_MT_NAME,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_STARTUP, CYTTSP5_MT_NAME,
		cyttsp5_startup_attention, 0);

	/* set up wakeup call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_WAKE, CYTTSP5_MT_NAME,
		cyttsp5_mt_wake_attention, 0);

	/* set up suspend call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_SUSPEND, CYTTSP5_MT_NAME,
		cyttsp5_mt_suspend_attention, 0);

	/* set up resume call back */
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_RESUME, CYTTSP5_MT_NAME,
		cyttsp5_mt_resume_attention, 0);

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_GLOVE, CYTTSP5_MT_NAME,
		cyttsp5_mt_glove_attention, 0);
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL
	_cyttsp5_subscribe_attention(dev, CY_ATTEN_FINGER, CYTTSP5_MT_NAME,
		cyttsp5_mt_finger_attention, 0);
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL */

	return 0;
}

static void cyttsp5_mt_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_IRQ, CYTTSP5_MT_NAME,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_STARTUP, CYTTSP5_MT_NAME,
		cyttsp5_startup_attention, 0);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_WAKE, CYTTSP5_MT_NAME,
		cyttsp5_mt_wake_attention, 0);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_SUSPEND, CYTTSP5_MT_NAME,
		cyttsp5_mt_suspend_attention, 0);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_RESUME, CYTTSP5_MT_NAME,
		cyttsp5_mt_resume_attention, 0);

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_GLOVE, CYTTSP5_MT_NAME,
		cyttsp5_mt_glove_attention, 0);
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL
	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_FINGER, CYTTSP5_MT_NAME,
		cyttsp5_mt_finger_attention, 0);
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL */

	mutex_lock(&md->mt_lock);
	if (!md->is_suspended) {
		pm_runtime_put(dev);
		md->is_suspended = true;
	}
	mutex_unlock(&md->mt_lock);
}

static int cyttsp5_setup_input_device(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	int signal = CY_IGNORE_VALUE;
	int max_x, max_y, max_p, min, max;
	int max_x_tmp, max_y_tmp;
	int i;
	int rc;

	dev_vdbg(dev, "%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, md->input->evbit);
	__set_bit(EV_REL, md->input->evbit);
	__set_bit(EV_KEY, md->input->evbit);
#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, md->input->propbit);
#endif
#ifdef CONFIG_TOUCHSCREEN_UNDERWATER
	__set_bit(BTN_WATER, md->input->keybit);
#endif

	/* If virtualkeys enabled, don't use all screen */
	if (md->pdata->flags & CY_MT_FLAG_VKEYS) {
		max_x_tmp = md->pdata->vkeys_x;
		max_y_tmp = md->pdata->vkeys_y;
	} else {
		max_x_tmp = md->si->sensing_conf_data.res_x;
		max_y_tmp = md->si->sensing_conf_data.res_y;
	}

	/* get maximum values from the sysinfo data */
	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		max_x = max_y_tmp - 1;
		max_y = max_x_tmp - 1;
	} else {
		max_x = max_x_tmp - 1;
		max_y = max_y_tmp - 1;
	}
	max_p = md->si->sensing_conf_data.max_z;

	/* set event signal capabilities */
	for (i = 0; i < NUM_SIGNALS(md->pdata->frmwrk); i++) {
		signal = MT_PARAM_SIGNAL(md, i);
		if (signal != CY_IGNORE_VALUE) {
			__set_bit(signal, md->input->absbit);

			min = MT_PARAM_MIN(md, i);
			max = MT_PARAM_MAX(md, i);
			if (i == CY_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			} else if (i == CY_ABS_X_OST)
				max = max_x;
			else if (i == CY_ABS_Y_OST)
				max = max_y;
			else if (i == CY_ABS_P_OST)
				max = CY_MAX_PRESSURE_SIZE;

			input_set_abs_params(md->input, signal, min, max,
				MT_PARAM_FUZZ(md, i), MT_PARAM_FLAT(md, i));
			dev_dbg(dev, "%s: register signal=%02X min=%d max=%d\n",
				__func__, signal, min, max);
		}
	}

	md->or_min = MT_PARAM_MIN(md, CY_ABS_OR_OST);
	md->or_max = MT_PARAM_MAX(md, CY_ABS_OR_OST);

	md->t_min = MT_PARAM_MIN(md, CY_ABS_ID_OST);
	md->t_max = MT_PARAM_MAX(md, CY_ABS_ID_OST);

	rc = md->mt_function.input_register_device(md->input,
			md->si->tch_abs[CY_TCH_T].max);
	if (rc < 0)
		dev_err(dev, "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		md->input_device_registered = true;

	return rc;
}

static int cyttsp5_setup_input_attention(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	int rc;

	md->si = _cyttsp5_request_sysinfo(dev);
	if (!md->si)
		return -EINVAL;

	rc = cyttsp5_setup_input_device(dev);

	_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_STARTUP, CYTTSP5_MT_NAME,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

int cyttsp5_mt_probe(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	struct cyttsp5_mt_platform_data *mt_pdata;
	int rc = 0;

	if (!pdata || !pdata->mt_pdata) {
		dev_err(dev, "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}
	mt_pdata = pdata->mt_pdata;

	cyttsp5_init_function_ptrs(md);

	mutex_init(&md->mt_lock);
	md->dev = dev;
	md->pdata = mt_pdata;

#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED
	md->check_finger_touch = false;
	md->check_large_touch = false;
	md->request_glove_touch = false;

	cmd = cyttsp5_get_commands();
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_ENABLED */

	/* Create the input device and register it. */
	dev_vdbg(dev, "%s: Create the input device and register it\n",
		__func__);
	md->input = input_allocate_device();
	if (!md->input) {
		dev_err(dev, "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (md->pdata->inp_dev_name)
		md->input->name = md->pdata->inp_dev_name;
	else
		md->input->name = CYTTSP5_MT_NAME;
	scnprintf(md->phys, sizeof(md->phys), "%s/input%d", dev_name(dev),
			cd->phys_num++);
	md->input->phys = md->phys;
	md->input->dev.parent = md->dev;
	md->input->open = cyttsp5_mt_open;
	md->input->close = cyttsp5_mt_close;
	input_set_drvdata(md->input, md);

	/* get sysinfo */
	md->si = _cyttsp5_request_sysinfo(dev);

	if (md->si) {
		rc = cyttsp5_setup_input_device(dev);
		if (rc)
			goto error_init_input;
	} else {
		dev_err(dev, "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, md->si);
		_cyttsp5_subscribe_attention(dev, CY_ATTEN_STARTUP,
			CYTTSP5_MT_NAME, cyttsp5_setup_input_attention, 0);
	}

#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
	wake_lock_init(&touchpanel_wake_lock, WAKE_LOCK_SUSPEND, dev_name(dev));
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */

	return 0;

error_init_input:
	input_free_device(md->input);
error_alloc_failed:
error_no_pdata:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

int cyttsp5_mt_release(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_mt_data *md = &cd->md;

	if (md->input_device_registered) {
		input_unregister_device(md->input);
	} else {
		input_free_device(md->input);
		_cyttsp5_unsubscribe_attention(dev, CY_ATTEN_STARTUP,
			CYTTSP5_MT_NAME, cyttsp5_setup_input_attention, 0);
	}

	return 0;
}
