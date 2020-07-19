/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
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
#include <oem-bms.h>
#include <linux/dnand_cdev_driver.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>
#include <oem-chg_control.h>
#include <linux/rtc.h>

#include <soc/qcom/smsm.h>

#define VBATT_SAMPLES			5
#define POWER_OFF_SOC			0
#define LOW_BATTERY_SOC			1
#define LOW_BATTERY_CHECK_SOC	10
#define POWER_OFF_THRESHOLD_UV		(3300 * 1000)
#define LOW_BATTERY_THRESHOLD_UV	(3350 * 1000)
#define VBATT_INITIAL_UV			(4200 * 1000)

#define START_BATT_TEMP_LO	120
#define START_BATT_TEMP_HI	430
#define STOP_BATT_TEMP_HI	450

#define SOC_START	30
#define MONITOR_TIME_S	3

#define ERR_VCHG_SAMPLES	5
#define ERR_VCHG_WAIT_MS	1000
#define ERR_VCHG_CYCLE_MS	10
#define ERR_VCHG_MIN		0
#define ERR_VCHG_MAX		100000

#define DETERIORATION_THRESH_BETTER				70
#define DETERIORATION_THRESH_GOOD				60
#define DETERIORATION_THRESH_GOOD_TO_BETTER		80
#define DETERIORATION_THRESH_BAD_TO_GOOD		70

#define DNAND_OFFSET_LEARNING_MAH			0
#define DNAND_OFFSET_ERR_VCHG				4
#define DNAND_OFFSET_DETERIORATION_STATUS	8

enum {
	LEARNING_STATUS_MONITOR,
	LEARNING_STATUS_MONITOR_STOP,
	LEARNING_STATUS_CHARGER_REMOVAL,
	LEARNING_STATUS_END
};

typedef struct {
	int		learning_mah;
	int		err_vchg;
	uint8_t	deterioration_status;
	uint8_t	reserved[21];
} oem_bms_param_chg_cycle_type;

static oem_bms_param_chg_cycle_type oem_param_chg_cycle = {
	.learning_mah = 0x00000000,
	.err_vchg = 0x00000000,
	.deterioration_status = DETERIORATION_STATUS_BETTER,
	.reserved = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF
	}
};

static atomic_t is_oem_bms_initialized = ATOMIC_INIT(0);

struct wake_lock fcc_learning_wake_lock;
struct delayed_work fcc_learning_monitor_work;
struct delayed_work fcc_learning_end_work;

static int learning_status = LEARNING_STATUS_END;

static int soc_start = 0;
static uint64_t vchg_uvs = 0;
static int monitor_time = 0;
static int check_count = 0;

static int learning_mah_default = 0;

int vchg_work[ERR_VCHG_SAMPLES] = {0};
static int vchg_index = 0;

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
static bool chg_cycle_initialized = false;

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

static int oem_bms_bound_soc_in_charging(int soc)
{
	soc = max(0, soc);
	soc = min(99, soc);
	return soc;
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
	int rc;
	struct qpnp_vadc_result result;

	rc = oem_chg_vadc_read(VBAT_SNS, &result);
	if (rc) {
		pr_err("invalid value rc:%d, vbatt_now_uv:%lld\n", rc, result.physical);
		return;
	}

	vbatt_buf_index++;

	if (vbatt_buf_index == VBATT_SAMPLES) {
		vbatt_buf_index = 0;
	}
	vbatt_buf[vbatt_buf_index] = result.physical;
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

int oem_bms_correct_calc_soc(int calc_soc, int last_soc)
{
	static bool is_initialized = false;
	unsigned long now_time = filter_base_time;
	int elapsed_time;
	int filter_check_time = 0;
	int minus_soc = 0;
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
			minus_soc = (elapsed_time / filter_check_time);
			if(delta_soc >= minus_soc) {
				delta_soc -= minus_soc;
			} else{ 
				delta_soc = 0;
			}
			filter_base_time = now_time;
			
			pr_info("timer expiration delta_soc:%d minus:%d elapsed_time:%d filter_base_time:%d\n",
				delta_soc, minus_soc, elapsed_time, (int)filter_base_time);
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

	if (battery_status == -EINVAL) {
		pr_err("invalid value battery_status:%d\n", battery_status);
		return result_soc;
	}

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
			} else {
				result_soc = oem_bms_bound_soc_in_charging(result_soc);
			}
			pr_debug("soc result:%d pre_result:%d time elapsed:%d filter_check:%d\n",
				result_soc, pre_result_soc, elapsed_time, filter_check_time);
		} else {
			result_soc = oem_bms_bound_soc_in_charging(result_soc);
		}
	} else if (battery_status == POWER_SUPPLY_STATUS_FULL) {
		result_soc = 100;
	}

	oem_bms_update_vbatt_current();

	result_soc = oem_bms_check_low_vbatt(result_soc, battery_status);

	pr_debug("soc pre_result:%d result:%d in:%d delta:%d\n",
		pre_result_soc, result_soc, in_soc, delta_soc);
	pre_result_soc = result_soc;

	return result_soc;
}
EXPORT_SYMBOL(oem_bms_correct_soc);

static void oem_bms_update_fcc_learning(void)
{
	int vchg_mvs = div_u64(vchg_uvs, 1000);
	int cc_mah = (vchg_mvs * 10 / 8 / 3600);
	int err_mah = (oem_param_chg_cycle.err_vchg / 1000) * monitor_time * 10 / 8 / 3600;
	int estimate_mah = ((cc_mah - err_mah) * 100) / (100 - soc_start);
	int fcc_thresh = 0;
	int new_learning_mah;
	int new_deterioration_status;

	if ((estimate_mah >= (oem_param_chg_cycle.learning_mah * 5 / 10)) &&
		(estimate_mah <= oem_param_chg_cycle.learning_mah)) {
		new_learning_mah = (oem_param_chg_cycle.learning_mah * 4 + estimate_mah) / 5;
	} else
	if ((estimate_mah >= oem_param_chg_cycle.learning_mah) &&
		(estimate_mah <= (oem_param_chg_cycle.learning_mah * 1500))) {
		new_learning_mah = (oem_param_chg_cycle.learning_mah * 7 + estimate_mah) / 8;
	} else {
		pr_err("not appropriate learning_mah:%d estimate_mah:%d err_mah:%d cc_mah:%d\n",
			oem_param_chg_cycle.learning_mah, estimate_mah, err_mah, cc_mah);
		return;
	}
	pr_info("learning_mah new:%d old:%d estimate_mah:%d err_mah:%d cc_mah:%d vchg_uvs:%lld\n",
		new_learning_mah, oem_param_chg_cycle.learning_mah, estimate_mah, err_mah, cc_mah, vchg_uvs);

	if ((new_learning_mah < 0) ||
		(new_learning_mah > learning_mah_default)) {
		pr_err("invalid new_learning_mah:%d\n", new_learning_mah);
		return;
	}
	pr_info("set dnand learning_mah new:%d old:%d\n",
		new_learning_mah, oem_param_chg_cycle.learning_mah);
	oem_param_chg_cycle.learning_mah = new_learning_mah;
	kdnand_id_write(DNAND_ID_KERNEL_23, DNAND_OFFSET_LEARNING_MAH,
		(uint8_t*)&oem_param_chg_cycle.learning_mah, sizeof(oem_param_chg_cycle.learning_mah));

	fcc_thresh = (oem_param_chg_cycle.learning_mah * 100) / learning_mah_default;
	pr_info("fcc_thresh:%d learning_mah:%d learning_mah_default:%d\n",
		fcc_thresh, oem_param_chg_cycle.learning_mah, learning_mah_default);

	switch (oem_param_chg_cycle.deterioration_status) {
		case DETERIORATION_STATUS_BETTER:
			if (fcc_thresh >= DETERIORATION_THRESH_BETTER) {
				new_deterioration_status = DETERIORATION_STATUS_BETTER;
			} else {
				new_deterioration_status = DETERIORATION_STATUS_GOOD;
			}
			break;
		case DETERIORATION_STATUS_GOOD:
			if (fcc_thresh >= DETERIORATION_THRESH_GOOD_TO_BETTER) {
				new_deterioration_status = DETERIORATION_STATUS_BETTER;
			} else if (fcc_thresh >= DETERIORATION_THRESH_GOOD) {
				new_deterioration_status = DETERIORATION_STATUS_GOOD;
			} else {
				new_deterioration_status = DETERIORATION_STATUS_BAD;
			}
			break;
		case DETERIORATION_STATUS_BAD:
			if (fcc_thresh >= DETERIORATION_THRESH_BAD_TO_GOOD) {
				new_deterioration_status = DETERIORATION_STATUS_GOOD;
			} else {
				new_deterioration_status = DETERIORATION_STATUS_BAD;
			}
			break;
		default:
			new_deterioration_status = DETERIORATION_STATUS_BETTER;
			break;
	}
	pr_info("set dnand deterioration_status new:%d old:%d\n",
		new_deterioration_status, oem_param_chg_cycle.deterioration_status);
	oem_param_chg_cycle.deterioration_status = new_deterioration_status;
	kdnand_id_write(DNAND_ID_KERNEL_23, DNAND_OFFSET_DETERIORATION_STATUS,
		&oem_param_chg_cycle.deterioration_status, sizeof(oem_param_chg_cycle.deterioration_status));
}

static void oem_bms_fcc_learning_monitor_work(struct work_struct *work)
{
	struct qpnp_vadc_result result;
	struct qpnp_vadc_result temp_result;
	int battery_status = oem_bms_get_batt_prop(POWER_SUPPLY_PROP_STATUS);

	oem_chg_vadc_read(LR_MUX1_BATT_THERM, &temp_result);
	pr_debug("get_bat_temp %lld\n",  temp_result.physical);

	if (battery_status == POWER_SUPPLY_STATUS_FULL) {
		pr_info("POWER_SUPPLY_STATUS_FULL\n");
		oem_bms_update_fcc_learning();
		learning_status = LEARNING_STATUS_MONITOR_STOP;
		wake_unlock(&fcc_learning_wake_lock);
		return;
	}

	if (battery_status != POWER_SUPPLY_STATUS_CHARGING) {
		if (battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			check_count++;
			if (check_count < 3) {
				pr_info("POWER_SUPPLY_STATUS_NOT_CHARGING check_count:%d\n", check_count);
				learning_status = LEARNING_STATUS_MONITOR_STOP;
				wake_unlock(&fcc_learning_wake_lock);
				return;
			}
		} else {
			check_count = 0;
		}
		pr_info("not POWER_SUPPLY_STATUS_CHARGING:%d\n", battery_status);
		learning_status = LEARNING_STATUS_MONITOR_STOP;
		wake_unlock(&fcc_learning_wake_lock);
		return;
	}

	if (temp_result.physical >= STOP_BATT_TEMP_HI) {
		pr_info("STOP_BATT_TEMP_HI:%lld\n", temp_result.physical);
		learning_status = LEARNING_STATUS_MONITOR_STOP;
		wake_unlock(&fcc_learning_wake_lock);
		return;
	}

	oem_chg_vadc_read(P_MUX2_1_3, &result);
	vchg_uvs += result.physical * MONITOR_TIME_S;
	monitor_time += MONITOR_TIME_S;

	pr_debug("vchg:%d vchg_uvs:%lld monitor_time:%d\n",
		(int)result.physical, vchg_uvs, monitor_time);

	schedule_delayed_work(&fcc_learning_monitor_work,
		msecs_to_jiffies(MONITOR_TIME_S * 1000));
}

static void oem_bms_fcc_learning_end_work(struct work_struct *work)
{
	struct qpnp_vadc_result result;
	int new_err_vchg = 0;

	oem_chg_vadc_read(P_MUX2_1_3, &result);
	vchg_work[vchg_index] = (int)result.physical;
	vchg_index++;

	if (vchg_index == ERR_VCHG_SAMPLES) {
		for (vchg_index = 0; vchg_index < ERR_VCHG_SAMPLES; vchg_index++) {
			new_err_vchg += vchg_work[vchg_index];
		}

		new_err_vchg /= ERR_VCHG_SAMPLES;

		if (new_err_vchg <= ERR_VCHG_MAX) {
			pr_info("set dnand err_vchg new:%d old:%d\n",
				new_err_vchg, oem_param_chg_cycle.err_vchg);
			oem_param_chg_cycle.err_vchg = new_err_vchg;
			kdnand_id_write(DNAND_ID_KERNEL_23, DNAND_OFFSET_ERR_VCHG,
				(uint8_t*)&oem_param_chg_cycle.err_vchg, sizeof(oem_param_chg_cycle.err_vchg));
		} else {
			pr_err("invalid new_err_vchg:%d\n", new_err_vchg);
		}
		pr_info("vchg_work [0]:%d [1]:%d [2]:%d [3]:%d [4]:%d\n",
			vchg_work[0], vchg_work[1], vchg_work[2], vchg_work[3], vchg_work[4]);
		learning_status = LEARNING_STATUS_END;
	} else {
		schedule_delayed_work(&fcc_learning_end_work,
			msecs_to_jiffies(ERR_VCHG_CYCLE_MS));
	}
}

static void oem_bms_fcc_learning_init(void)
{
	uint8_t *smem_ptr = NULL;

	INIT_DELAYED_WORK(&fcc_learning_monitor_work,
		oem_bms_fcc_learning_monitor_work);

	INIT_DELAYED_WORK(&fcc_learning_end_work,
		oem_bms_fcc_learning_end_work);

	wake_lock_init(&fcc_learning_wake_lock, WAKE_LOCK_SUSPEND, "oem_bms_fcc_learning");

	if (!chg_cycle_initialized) {
		smem_ptr = (uint8_t *)kc_smem_alloc(SMEM_CHG_PARAM, KCC_SMEM_CHG_PARAM_SIZE);

		if (smem_ptr == NULL) {
			pr_err("read error SMEM_CHG_PARAM\n");
			oem_param_chg_cycle.learning_mah = learning_mah_default;
			oem_param_chg_cycle.err_vchg = 0;
			oem_param_chg_cycle.deterioration_status = DETERIORATION_STATUS_BETTER;
			return;
		}

		memcpy(&oem_param_chg_cycle, smem_ptr,
					sizeof(oem_bms_param_chg_cycle_type));

		chg_cycle_initialized = true;
	}

	if ((oem_param_chg_cycle.learning_mah <= 0) ||
		(oem_param_chg_cycle.learning_mah > learning_mah_default)) {
		pr_err("invalid learning_mah:%d\n", oem_param_chg_cycle.learning_mah);
		oem_param_chg_cycle.learning_mah = learning_mah_default;
	}

	if ((oem_param_chg_cycle.err_vchg < ERR_VCHG_MIN) ||
		(oem_param_chg_cycle.err_vchg > ERR_VCHG_MAX)) {
		pr_err("invalid err_vchg:%d\n", oem_param_chg_cycle.err_vchg);
		oem_param_chg_cycle.err_vchg = 0;
	}

	if ((oem_param_chg_cycle.deterioration_status < DETERIORATION_STATUS_BETTER) ||
		(oem_param_chg_cycle.deterioration_status > DETERIORATION_STATUS_BAD)) {
		pr_err("invalid deterioration_status:%d\n", oem_param_chg_cycle.deterioration_status);
		oem_param_chg_cycle.deterioration_status = DETERIORATION_STATUS_BETTER;
	}

	pr_info("read smem learning_mah:%d err_vchg:%d deterioration_status:%d\n",
		oem_param_chg_cycle.learning_mah, oem_param_chg_cycle.err_vchg, oem_param_chg_cycle.deterioration_status);
}

void oem_bms_start_fcc_learning(int soc_final)
{
	int initialized = atomic_read(&is_oem_bms_initialized);
	struct qpnp_vadc_result result;

	oem_chg_vadc_read(LR_MUX1_BATT_THERM, &result);
	pr_debug("get_bat_temp %lld\n",  result.physical);

	if (!initialized) {
		pr_err("uninitialized\n");
		return;
	}

	if (learning_status != LEARNING_STATUS_END) {
		pr_info("already start learning_status:%d\n",learning_status);
		return;
	}

	if ((result.physical >= START_BATT_TEMP_LO) &&
		(result.physical <= START_BATT_TEMP_HI) &&
		(soc_final <= SOC_START)) {
		wake_lock(&fcc_learning_wake_lock);

		learning_status = LEARNING_STATUS_MONITOR;

		soc_start = soc_final;
		vchg_uvs = 0;
		monitor_time = 0;
		check_count = 0;

		schedule_delayed_work(&fcc_learning_monitor_work,
			msecs_to_jiffies(MONITOR_TIME_S * 1000));

		pr_info("soc_start:%d vchg_uvs:%lld monitor_time:%d check_count:%d\n",
			soc_start, vchg_uvs, monitor_time, check_count);
	} else {
		learning_status = LEARNING_STATUS_MONITOR_STOP;
		pr_info("do not monitor batt_temp:%lld soc_final:%d\n",
			result.physical, soc_final);
	}
}
EXPORT_SYMBOL(oem_bms_start_fcc_learning);

void oem_bms_stop_fcc_learning(void)
{
	int initialized = atomic_read(&is_oem_bms_initialized);

	if (!initialized) {
		pr_err("uninitialized\n");
		return;
	}

	cancel_delayed_work_sync(&fcc_learning_monitor_work);

	if (wake_lock_active(&fcc_learning_wake_lock)) {
		wake_unlock(&fcc_learning_wake_lock);
	}

	if (learning_status >= LEARNING_STATUS_CHARGER_REMOVAL) {
		pr_info("already stop learning_status:%d\n",learning_status);
		return;
	}
	learning_status = LEARNING_STATUS_CHARGER_REMOVAL;
	vchg_index = 0;
	schedule_delayed_work(&fcc_learning_end_work,
		msecs_to_jiffies(ERR_VCHG_WAIT_MS));
}
EXPORT_SYMBOL(oem_bms_stop_fcc_learning);

int oem_bms_get_deterioration_status(void)
{
	uint8_t *smem_ptr = NULL;
	int ret = DETERIORATION_STATUS_BETTER;

	if (!chg_cycle_initialized) {
		smem_ptr = (uint8_t *)kc_smem_alloc(SMEM_CHG_PARAM, KCC_SMEM_CHG_PARAM_SIZE);

		if (smem_ptr == NULL) {
			pr_err("read error SMEM_CHG_PARAM\n");
			return ret;
		}

		memcpy(&oem_param_chg_cycle, smem_ptr,
					sizeof(oem_bms_param_chg_cycle_type));

		chg_cycle_initialized = true;
	}

	ret = (int)oem_param_chg_cycle.deterioration_status;
	return ret;
}
EXPORT_SYMBOL(oem_bms_get_deterioration_status);

void oem_bms_set_full_suppress(bool suppress)
{
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

void oem_bms_init(int fcc)
{
	learning_mah_default = fcc;

	oem_bms_fcc_learning_init();

	atomic_set(&is_oem_bms_initialized, 1);

	pr_info("initialized learning_mah_default:%d\n", learning_mah_default);
}
EXPORT_SYMBOL(oem_bms_init);

