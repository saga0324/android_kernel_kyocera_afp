/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
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
#ifndef SENSOR_COM_H
#define SENSOR_COM_H

#include <linux/module.h>
#include <linux/kernel.h>

void acc_driver_init(void);
void acc_lnr_driver_init(void);
void gyro_driver_init(void);
void gyro_uncal_driver_init(void);
void grv_driver_init(void);
void mag_driver_init(void);
void mag_uncal_driver_init(void);
void relative_humidity_driver_init(void);
void rot_vctr_driver_init(void);
void game_rot_vctr_driver_init(void);
void mag_rot_vctr_driver_init(void);
void ortn_driver_init(void);
void pressure_driver_init(void);
void temperature_driver_init(void);
void temperature_ambient_driver_init(void);
void light_driver_init(void);
void prox_driver_init(void);
void sgnfcnt_mtn_driver_init(void);
void step_cnt_driver_init(void);
void step_dtc_driver_init(void);
void sensor_ext_driver_init(void);
void sensor_ext_vh_driver_init(void);
void kc_motion_sensor_driver_init(void);
void underwater_detect_driver_init(void);

void acc_ring_buffer_timestamp(
    uint32_t time_stamp_acc);
void acc_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void acc_timestamp_report(void);
void acc_lnr_ring_buffer_timestamp(
    uint32_t time_stamp_linacc);
void acc_lnr_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void acc_lnr_timestamp_report(void);
void game_rot_vctr_ring_buffer_timestamp(
    uint32_t time_stamp_gamerota);
void game_rot_vctr_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void game_rot_vctr_timestamp_report(void);
void grv_ring_buffer_timestamp(
    uint32_t time_stamp_gravity);
void grv_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void grv_timestamp_report(void);
void gyro_ring_buffer_timestamp(
    uint32_t time_stamp_gyro);
void gyro_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void gyro_timestamp_report(void);
void gyro_uncal_ring_buffer_timestamp(
    uint32_t time_stamp_gyro_uncal);
void gyro_uncal_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void gyro_uncal2_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void gyro_uncal_timestamp_report(void);
void mag_ring_buffer_timestamp(
    uint32_t time_stamp_mag);
void mag_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void mag_timestamp_report(void);
void mag_rot_vctr_ring_buffer_timestamp(
    uint32_t time_stamp_magrota);
void mag_rot_vctr_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void mag_rot_vctr_timestamp_report(void);
void mag_uncal_ring_buffer_timestamp(
    uint32_t time_stamp_mag_uncal);
void mag_uncal_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void mag_uncal2_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void mag_uncal_timestamp_report(void);
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
void pressure_ring_buffer_timestamp(
    uint32_t time_stamp_pressure);
void pressure_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void pressure_timestamp_report(void);
#else
static inline void pressure_ring_buffer_timestamp(
    uint32_t time_stamp_pressure){}
static inline void pressure_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data ){}
static inline void pressure_timestamp_report(void){}
#endif
void ortn_ring_buffer_timestamp(
    uint32_t time_stamp_ori);
void ortn_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void ortn_timestamp_report(void);
void rot_vctr_ring_buffer_timestamp(
    uint32_t time_stamp_rota);
void rot_vctr_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void rot_vctr_timestamp_report(void);
void step_cnt_batch_ring_buffer(
    uint32_t step_counter);
void step_cnt_step1_ring_buffer_timestamp(
    int32_t time_stamp_step1);
void step_cnt_step2_ring_buffer_timestamp(
    int32_t time_stamp_step2);
void step_cnt_step1_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void step_cnt_step2_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void step_cnt_step_abs_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void step_cnt_timestamp_report(void);
void step_dtc_batch_ring_buffer(uint32_t step_detector);
void step_dtc_step1_ring_buffer_timestamp(
    int32_t time_stamp_step_dtc1);
void step_dtc_step2_ring_buffer_timestamp(
    int32_t time_stamp_step_dtc2);
void step_dtc_step1_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void step_dtc_step2_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void step_dtc_step_abs_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data );
void step_dtc_timestamp_report(void);

void sensor_ext_iwifi_interrupt( void );
void sensor_ext_vh_interrupt( void );
void sensor_ext_vehi_interrupt( void );
void sensor_ext_pedo_interrupt( void );
void sensor_ext_input_report(int64_t timestamp, void *event, uint32_t len);
void prox_interrupt(uint32_t data);
void step_dtc_interrupt(void);
void sgnfcnt_mtn_interrupt(void);
void step_cnt_interrupt(uint32_t data);
void light_input_report( uint32_t data );
void kc_motion_sensor_walk_start_interrupt( void );
void kc_motion_sensor_walk_stop_interrupt( void );
void kc_motion_sensor_train_interrupt( void );
void kc_motion_sensor_vehicle_interrupt( void );
void kc_motion_sensor_bringup_interrupt( void );
void acc_auto_cal_interrupt( void );
void underwater_detect_water_interrupt( void );

#endif /* SENSOR_COM_H */
