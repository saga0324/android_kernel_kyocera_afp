/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"BMS: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <oem-bms.h>

#define VBATT_SAMPLES			5
#define POWER_OFF_SOC			0
#define LOW_BATTERY_SOC			1
#define LOW_BATTERY_CHECK_SOC	10
#define BATT_CARE_MODE_SOC		85
#define POWER_OFF_THRESHOLD_UV		(3300 * 1000)
#define LOW_BATTERY_THRESHOLD_UV	(3350 * 1000)
#define VBATT_INITIAL_UV			(4200 * 1000)

static int vbatt_current_uv = VBATT_INITIAL_UV;
static int vbatt_buf_index = 0;
static int vbatt_buf[VBATT_SAMPLES] ={
	VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV, VBATT_INITIAL_UV
};

#define SOC_97_FILTER_TIME_S		(1 * 60)
#define SOC_98_FILTER_TIME_S		(2 * 60)
#define SOC_99_FILTER_TIME_S		(3 * 60)
#define DISCHARGING_FILTER_TIME_S	(10 * 60)
#define CORRECT_DISCHG_CALC_SOC		70

static int pre_battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
static bool full_suppress = false;
static int pre_result_soc = 0;
static int delta_soc = 0;
static unsigned long filter_base_time = 0;

enum {
	BATT_CARE_STATUS_OFF = 0,
	BATT_CARE_STATUS_ON = 1,
};

void oem_recharge_check_start(void);

static int oem_bms_get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int oem_bms_get_batt_prop(enum power_supply_property psp)
{
	union power_supply_propval ret = {0,};
	static struct power_supply *psy;

	psy = power_supply_get_by_name("battery");
	if (psy) {
		/* if battery has been registered, use the status property */
		psy->get_property(psy, psp, &ret);
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_err("battery power supply is not registered\n");
	return -EINVAL;
}

static int oem_bms_set_batt_prop(enum power_supply_property psp, int val)
{
	union power_supply_propval ret = {0,};
	static struct power_supply *psy;

	ret.intval = val;

	psy = power_supply_get_by_name("battery");
	if (psy) {
		/* if battery has been registered, use the status property */
		psy->set_property(psy, psp, &ret);
		return 0;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_err("battery power supply is not registered\n");
	return -EINVAL;
}

static int oem_bms_get_average(int *buf, int num)
{
	int ave, cnt;
	int max = 0, min = 0, sum = 0;

	max = buf[0];
	min = buf[0];

	for (cnt = 0; cnt < num; cnt++) {
		if (max < buf[cnt]) {
			max = buf[cnt];
		} else if (min > buf[cnt]) {
			min = buf[cnt];
		}
		sum += buf[cnt];
	}

	if (num < 3) {
		ave = sum / num;
	} else {
		ave = (sum - max - min) / (num - 2);
	}

	pr_debug("ave:%d sum:%d max:%d min:%d num:%d\n",
		ave, sum, max, min, num);

	return ave;
}

static void oem_bms_update_vbatt_current(void)
{
	int voltage_now = 0;

	voltage_now = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (voltage_now == -EINVAL) {
		pr_err("invalid value:%d\n", voltage_now);
		return;
	}

	vbatt_buf_index++;

	if (vbatt_buf_index == VBATT_SAMPLES) {
		vbatt_buf_index = 0;
	}
	vbatt_buf[vbatt_buf_index] = voltage_now;
	vbatt_current_uv = oem_bms_get_average(vbatt_buf, VBATT_SAMPLES);

	pr_debug("vbatt_buf [0]=%d [1]=%d [2]=%d [3]=%d [4]=%d vbatt_buf_index:%d\n",
		vbatt_buf[0], vbatt_buf[1], vbatt_buf[2], vbatt_buf[3], vbatt_buf[4], vbatt_buf_index);
}

static int oem_bms_check_low_vbatt(int soc, int battery_status)
{
	int result_soc = soc;

	if(vbatt_current_uv <= POWER_OFF_THRESHOLD_UV) {
		result_soc = POWER_OFF_SOC;
		pr_err("set auto power off soc:%d vbatt_uv:%d\n", soc, vbatt_current_uv);
	} else if ((vbatt_current_uv <= LOW_BATTERY_THRESHOLD_UV) &&
				(battery_status != POWER_SUPPLY_STATUS_CHARGING) &&
				(soc <= LOW_BATTERY_CHECK_SOC)) {
		result_soc = LOW_BATTERY_SOC;
	}
	pr_debug("result_soc:%d vbatt_current_uv:%d\n", result_soc, vbatt_current_uv);

	return result_soc;
}

static int oem_batt_care_update(int soc)
{
	int rc = 0;
	int batt_care_status = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_BATTERY_CARE_STATUS);

	if (batt_care_status == BATT_CARE_STATUS_OFF && soc <= BATT_CARE_MODE_SOC) {
		rc = oem_bms_set_batt_prop(POWER_SUPPLY_PROP_BATTERY_CARE_STATUS, BATT_CARE_STATUS_ON);
		if (rc < 0) {
			pr_err("error setting BATT_CARE_STATUS, rc:%d\n", rc);
		} else {
			batt_care_status = BATT_CARE_STATUS_ON;
		}
	}

	return batt_care_status;
}

int oem_bms_correct_calc_soc(int calc_soc, int last_soc)
{
	static bool is_initialized = false;
	unsigned long now_time = filter_base_time;
	int elapsed_time;
	int filter_check_time = 0;
	int correct_calc_soc = calc_soc;
	int battery_status = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_STATUS);

	if (!is_initialized) {
		oem_bms_get_current_time(&filter_base_time);
		if (last_soc == -EINVAL) {
			pre_result_soc = calc_soc;
		} else {
			pre_result_soc = last_soc;
		}
		is_initialized = true;
		pr_info("initialized pre_result_soc:%d last_soc:%d calc_soc:%d filter_base_time:%d\n",
			pre_result_soc, last_soc, calc_soc, (int)filter_base_time);
	}

	if ((battery_status != POWER_SUPPLY_STATUS_DISCHARGING) &&
		(battery_status != POWER_SUPPLY_STATUS_NOT_CHARGING)) {
		delta_soc = 0;
		pre_battery_status = battery_status;
		return correct_calc_soc;
	}

	if (pre_battery_status != battery_status) {
		if ((calc_soc >= CORRECT_DISCHG_CALC_SOC) &&
			(pre_result_soc > calc_soc)) {
			delta_soc = pre_result_soc - calc_soc;
			oem_bms_get_current_time(&filter_base_time);
		} else {
			delta_soc = 0;
		}
		pr_info("status change old:%d new:%d delta_soc:%d pre_result_soc:%d calc_soc:%d filter_base_time:%d\n",
			pre_battery_status, battery_status, delta_soc, pre_result_soc, calc_soc, (int)filter_base_time);
	}

	if (delta_soc) {
		oem_bms_get_current_time(&now_time);
		elapsed_time = now_time - filter_base_time;
		filter_check_time = DISCHARGING_FILTER_TIME_S;

		if (elapsed_time >= filter_check_time) {
			delta_soc -= (elapsed_time / filter_check_time);
			filter_base_time = now_time;

			pr_info("timer expiration delta_soc:%d minus:%d elapsed_time:%d filter_base_time:%d\n",
				delta_soc, elapsed_time / filter_check_time, elapsed_time, (int)filter_base_time);
		}
		correct_calc_soc += delta_soc;

		pr_debug("elapsed_time:%d filter_check_time:%d\n",
			elapsed_time, filter_check_time);
	}
	pre_battery_status = battery_status;

	return correct_calc_soc;
}
EXPORT_SYMBOL(oem_bms_correct_calc_soc);

int oem_bms_correct_soc(int in_soc)
{
	unsigned long now_time = filter_base_time;
	int elapsed_time;
	int filter_check_time = 0;
	int result_soc = in_soc;
	int battery_status = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_STATUS);
	int batt_care_mode = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_BATTERY_CARE_MODE);
	int batt_care_status = BATT_CARE_STATUS_OFF;

	if (battery_status == -EINVAL) {
		pr_err("invalid value battery_status:%d\n", battery_status);
		return result_soc;
	}

	if (!batt_care_mode) {
		if (battery_status == POWER_SUPPLY_STATUS_CHARGING) {
			if (oem_bms_get_full_suppress()) {
				oem_bms_get_current_time(&now_time);
				elapsed_time = now_time - filter_base_time;

				if (pre_result_soc <= 97) {
					filter_check_time = SOC_97_FILTER_TIME_S;
				} else if (pre_result_soc == 98) {
					filter_check_time = SOC_98_FILTER_TIME_S;
				} else {
					filter_check_time = SOC_99_FILTER_TIME_S;
				}

				if (elapsed_time >= filter_check_time) {
					result_soc = pre_result_soc + 1;
					filter_base_time = now_time;
					pr_info("timer expiration elapsed_time:%d filter_base_time:%d\n",
						elapsed_time, (int)filter_base_time);
				} else {
					result_soc = pre_result_soc;
				}

				if (result_soc >= 100) {
					result_soc = 100;
					oem_bms_set_full_suppress(false);
					pr_info("soc 100 release the suppress\n");
					oem_recharge_check_start();
				}
				pr_debug("soc result:%d pre_result:%d time elapsed:%d filter_check:%d\n",
					result_soc, pre_result_soc, elapsed_time, filter_check_time);
			}
		} else if (battery_status == POWER_SUPPLY_STATUS_FULL) {
			result_soc = 100;
		}
	} else {
		batt_care_status = oem_batt_care_update(in_soc);

		if (batt_care_status && (battery_status == POWER_SUPPLY_STATUS_FULL || result_soc >= BATT_CARE_MODE_SOC)) {
			result_soc = BATT_CARE_MODE_SOC;
		}
	}

	oem_bms_update_vbatt_current();

	result_soc = oem_bms_check_low_vbatt(result_soc, battery_status);

	pr_debug("soc pre_result:%d result:%d in:%d delta:%d\n",
		pre_result_soc, result_soc, in_soc, delta_soc);
	pre_result_soc = result_soc;

	return result_soc;
}
EXPORT_SYMBOL(oem_bms_correct_soc);

void oem_bms_set_full_suppress(bool suppress)
{
	int batt_care_mode = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_BATTERY_CARE_MODE);
	if (batt_care_mode && suppress) {
		suppress = false;
		pr_info("inside of battery care mode cancels compensation.\n");
	}

	full_suppress = suppress;
	if (full_suppress) {
		oem_bms_get_current_time(&filter_base_time);
	}
	pr_info("full_suppress:%d filter_base_time:%d\n", full_suppress, (int)filter_base_time);
}
EXPORT_SYMBOL(oem_bms_set_full_suppress);

bool oem_bms_get_full_suppress(void)
{
	bool ret = full_suppress;
	return ret;
}
EXPORT_SYMBOL(oem_bms_get_full_suppress);

