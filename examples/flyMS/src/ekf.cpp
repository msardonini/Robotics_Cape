/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>	
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>



#include "../../../libraries/ecl/EKF/ekf.h"

#define MIN_INTERVAL_MS 20

extern "C"{
#include "ekf.h"
#include "flyMS.h"
}

Ekf _ekf;

void* run_ekf(void *ptr)
{
	ekf_filter_t* ekf_filter = (ekf_filter_t*) ptr;

	Vector3f _vel_body_wind;
//	float _acc_hor_filt;


	// Magnetometer Values
	float prev_mag_data[3];	
	uint64_t _timestamp_mag_us;
	uint64_t _mag_time_sum_ms;
	uint32_t _mag_sample_count;
	float _mag_data_sum[3];
	uint64_t _mag_time_ms_last_used;

//	Baro values
	uint64_t _timestamp_balt_us;
	uint64_t _balt_time_sum_ms;
	uint64_t _balt_time_ms_last_used;
	uint32_t _balt_sample_count;
	float _balt_data_sum;

	float accel_bias[3] = {0,0,0};

	while (rc_get_state()!=EXITING) 
	// while (1) 
	{

	//	bool gps_updated = false;
	//	bool magnetomer_updated = false;
	//	bool barometer_updated = false;
		
	//	bool airspeed_updated = false;
		bool optical_flow_updated = false;
		bool range_finder_updated = false;
		bool vehicle_land_detected_updated = false;
		bool vision_position_updated = false;
		bool vision_attitude_updated = false;
//		bool vehicle_status_updated = false;

		struct timespec log_time;
		clock_gettime(CLOCK_MONOTONIC, &log_time);
		uint64_t timestamp=(uint64_t)(log_time.tv_sec) * 1E6 + 
						((uint64_t)(log_time.tv_nsec) / 1000) ;
		
		//printf("input timestamp imu %lld\n", timestamp);

		//uint64_t timestamp = ekf_filter->input.IMU_timestamp * 1E6;
		// push imu data into estimator
		float gyro_integral[3];
		//float gyro_dt = DT;
		gyro_integral[0] = ekf_filter->input.gyro[0] * DT;
		gyro_integral[1] = ekf_filter->input.gyro[1] * DT;
		gyro_integral[2] = ekf_filter->input.gyro[2] * DT;
		float accel_integral[3];
		//float accel_dt =  DT;
		accel_integral[0] = (ekf_filter->input.accel[0] - accel_bias[0]) * DT;
		accel_integral[1] = (ekf_filter->input.accel[1] - accel_bias[1]) * DT;
		accel_integral[2] = (ekf_filter->input.accel[2] - accel_bias[2]) * DT;
		_ekf.setIMUData(timestamp, 5000, 5000,
				gyro_integral, accel_integral);

		
		//Decide if the magnetometer data is new or not
		int is_new_mag_data, i;
		if (prev_mag_data[0] == ekf_filter->input.mag[0]
			&& prev_mag_data[1] == ekf_filter->input.mag[1]
			&& prev_mag_data[2] == ekf_filter->input.mag[2])
		{
			is_new_mag_data = 1;
		}
		else
		{
			is_new_mag_data = 1;
		}
		for (i = 0; i <3; i++)
		{
			prev_mag_data[i] = ekf_filter->input.mag[i];
		}


		// read mag data
		// Basically checking if this data is the same as was the last iteration
		if (is_new_mag_data) 
		{
			_timestamp_mag_us = timestamp;

			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the specified interval is reached.
			_mag_time_sum_ms += _timestamp_mag_us/ 1000;
			_mag_sample_count++;
			_mag_data_sum[0] += ekf_filter->input.mag[0];
			_mag_data_sum[1] += ekf_filter->input.mag[1];
			_mag_data_sum[2] += ekf_filter->input.mag[2];;
			uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;



		//		printf("Calc diff %d actual diff %d \n",(int)(timestamp - _mag_time_ms_last_used),MIN_INTERVAL_MS);

			// if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
			if (mag_time_ms - _mag_time_ms_last_used > MIN_INTERVAL_MS) {
				float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
				// calculate mean of measurements and correct for learned bias offsets
				float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv,
							    _mag_data_sum[1] *mag_sample_count_inv,
							    _mag_data_sum[2] *mag_sample_count_inv
							   };
				_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);
				_mag_time_ms_last_used = mag_time_ms;
				_mag_time_sum_ms = 0;
				_mag_sample_count = 0;
				_mag_data_sum[0] = 0.0f;
				_mag_data_sum[1] = 0.0f;
				_mag_data_sum[2] = 0.0f;
			}
		}
		


		// float mag_data_avg_ga[3];
		// // TODO These arent really the inputs to the filter, fix later
		// mag_data_avg_ga[0] = ekf_filter->input.mag[0];
		// mag_data_avg_ga[1] = ekf_filter->input.mag[1];
		// mag_data_avg_ga[2] = ekf_filter->input.mag[2];

		// _ekf.setMagData((uint64_t)now, mag_data_avg_ga);
		


			
		// if (ekf_filter->input.barometer_updated) 
		// {
		// 	float balt_data_avg = ekf_filter->input.barometer_pressure;

		// 	// push to estimator
		// 	_ekf.set_air_density(0.001f);
		// 	_ekf.setBaroData( (uint64_t)now, balt_data_avg);
		// 	ekf_filter->input.barometer_updated = 0;
		// }
		// read baro data





		if (ekf_filter->input.barometer_updated) 
		{
			_timestamp_balt_us = timestamp;

			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the specified interval is reached.
			_balt_time_sum_ms += _timestamp_balt_us / 1000;
			_balt_sample_count++;
			_balt_data_sum += ekf_filter->input.barometer_alt;
			uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

			if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)MIN_INTERVAL_MS) {
				// take mean across sample period
				float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;

				_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);
				_balt_time_ms_last_used = balt_time_ms;
				_balt_time_sum_ms = 0;
				_balt_sample_count = 0;
				_balt_data_sum = 0.0f;

			}
		}

		// read gps data if available
		if (ekf_filter->input.gps_updated) 
		{
			struct gps_message gps_msg = {};
			gps_msg.time_usec = ekf_filter->input.gps_timestamp;
			gps_msg.lat = (int32_t)(ekf_filter->input.gps_latlon[0]*1E7);
			gps_msg.lon = (int32_t)(ekf_filter->input.gps_latlon[1]*1E7);
			gps_msg.alt = (int32_t)(ekf_filter->input.gps_latlon[3]*1E3);
			
			gps_msg.fix_type = 3;
			// gps_msg.fix_type = ekf_filter->input.gps_fix;
			gps_msg.eph = 1;
			gps_msg.epv = 1;
			gps_msg.sacc = 1;
			gps_msg.vel_m_s = 0;
			// gps_msg.vel_ned[0] = gps.vel_n_m_s;
			// gps_msg.vel_ned[1] = gps.vel_e_m_s;
			// gps_msg.vel_ned[2] = gps.vel_d_m_s;
			gps_msg.vel_ned_valid = 0;
			gps_msg.nsats = ekf_filter->input.nsats;
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.5f;
			

			//_ekf.setGpsData(ekf_filter->input.gps_timestamp, &gps_msg);
			_ekf.setGpsData(timestamp, &gps_msg);
			ekf_filter->input.gps_updated = 0;
		}


		// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
		_ekf.set_is_fixed_wing(0);
		_ekf.set_in_air_status(1);

		if (optical_flow_updated) {
			// flow_message flow;
			// flow.flowdata(0) = optical_flow.pixel_flow_x_integral;
			// flow.flowdata(1) = optical_flow.pixel_flow_y_integral;
			// flow.quality = optical_flow.quality;
			// flow.gyrodata(0) = optical_flow.gyro_x_rate_integral;
			// flow.gyrodata(1) = optical_flow.gyro_y_rate_integral;
			// flow.gyrodata(2) = optical_flow.gyro_z_rate_integral;
			// flow.dt = optical_flow.integration_timespan;

			// if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
			//     PX4_ISFINITE(optical_flow.pixel_flow_x_integral)) {
			// 	_ekf.setOpticalFlowData(optical_flow.timestamp, &flow);
			// }
		}

		if (range_finder_updated) {
			// _ekf.setRangeData(range_finder.timestamp, range_finder.current_distance);
		}

		// get external vision data
		// if error estimates are unavailable, use parameter defined defaults
		if (vision_position_updated || vision_attitude_updated) {
			// ext_vision_message ev_data;
			// ev_data.posNED(0) = ev_pos.x;
			// ev_data.posNED(1) = ev_pos.y;
			// ev_data.posNED(2) = ev_pos.z;
			// matrix::Quatf q(ev_att.q);
			// ev_data.quat = q;

			// // position measurement error from parameters. TODO : use covariances from topic
			// ev_data.posErr = _default_ev_pos_noise;
			// ev_data.angErr = _default_ev_ang_noise;

			// // use timestamp from external computer, clocks are synchronized when using MAVROS
			// _ekf.setExtVisionData(vision_position_updated ? ev_pos.timestamp : ev_att.timestamp, &ev_data);
		}

		if (vehicle_land_detected_updated) {
			_ekf.set_in_air_status(!ekf_filter->input.vehicle_land);
		}


		/* ------------ Run the EKF and Output the data -------- */ 
//			printf("Update Successful \n");
		if (_ekf.update()) 
		{

	//		printf("Update Successful \n");
			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());

			float velocity[3];
			_ekf.get_velocity(velocity);

			float pos_d_deriv;
			_ekf.get_pos_d_deriv(&pos_d_deriv);

		//	float gyro_rad[3];

			
			// generate control state data
			float gyro_bias[3] = {};
			_ekf.get_gyro_bias(gyro_bias);


			/* --------------   Estimate for Gyro Data ---------*/
			ekf_filter->output.gyro[0] = ekf_filter->input.gyro[0]  - gyro_bias[0];
			ekf_filter->output.gyro[1] = ekf_filter->input.gyro[1]  - gyro_bias[1];
			ekf_filter->output.gyro[2] = ekf_filter->input.gyro[2]  - gyro_bias[2];


			// Velocity in body frame
			Vector3f v_n(velocity);
			matrix::Dcm<float> R_to_body(q.inversed());
		//	Vector3f v_b = R_to_body * v_n;


			// Calculate velocity relative to wind in body frame
			float velNE_wind[2] = {};
			_ekf.get_wind_velocity(velNE_wind);
			v_n(0) -= velNE_wind[0];
			v_n(1) -= velNE_wind[1];
			_vel_body_wind = R_to_body * v_n;

			/* --------------   Estimate for Local Position Data ---------*/
			float position[3];
			_ekf.get_position(position);
			//ekf_filter->output.ned_pos[0] = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
			//ekf_filter->output.ned_pos[1] = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;

			ekf_filter->output.ned_pos[0] = position[0];
			ekf_filter->output.ned_pos[1] = position[1];
			ekf_filter->output.ned_pos[2] = position[2];
			/* --------------   Estimate for Velocity Data ---------*/

			// Velocity of body origin in local NED frame (m/s)
			ekf_filter->output.ned_vel[0] = velocity[0];
			ekf_filter->output.ned_vel[1] = velocity[1];
			ekf_filter->output.ned_vel[2]= velocity[2];
			ekf_filter->output.vertical_time_deriv = pos_d_deriv; // vertical position time derivative (m/s)

			// Acceleration data
			 _ekf.get_accel_bias(accel_bias);
			 ekf_filter->output.ned_acc[0] = ekf_filter->input.accel[0] - accel_bias[0];
			 ekf_filter->output.ned_acc[1] = ekf_filter->input.accel[1] - accel_bias[1];
			 ekf_filter->output.ned_acc[2] = ekf_filter->input.accel[2] - accel_bias[2];

			// // compute lowpass filtered horizontal acceleration
			// acceleration = R_to_body.transpose() * acceleration;
			// _acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
			// 		acceleration(1) * acceleration(1));
			// ctrl_state.horz_acc_mag = _acc_hor_filt;
		}
		usleep(5000);
	}
	return NULL;
}

// Ekf2 *Ekf2::instantiate(int argc, char *argv[])
// {
// 	Ekf2 *instance = new Ekf2();

// 	if (instance) {
// 		if (argc >= 2 && !strcmp(argv[1], "-r")) {
// 			instance->set_replay_mode(true);
// 		}
// 	}

// 	return instance;
// }

// int Ekf2::custom_command(int argc, char *argv[])
// {
// 	return print_usage("unknown command");
// }

// int Ekf2::print_usage(const char *reason)
// {
// 	if (reason) {
// 		PX4_WARN("%s\n", reason);
// 	}

// 	PRINT_MODULE_DESCRIPTION(
// 		R"DESCR_STR(
// ### Description
// Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

// The documentation can be found on the [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) page.

// ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
// timestamps from the sensor topics.

// )DESCR_STR");

// 	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
// 	PRINT_MODULE_USAGE_COMMAND("start");
// 	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
// 	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

// 	return 0;
// }

// int Ekf2::task_spawn(int argc, char *argv[])
// {
// 	_task_id = px4_task_spawn_cmd("ekf2",
// 				      SCHED_DEFAULT,
// 				      SCHED_PRIORITY_ESTIMATOR,
// 				      5900,
// 				      (px4_main_t)&run_trampoline,
// 				      (char *const *)argv);

// 	if (_task_id < 0) {
// 		_task_id = -1;
// 		return -errno;
// 	}

// 	return 0;
// }

// int ekf2_main(int argc, char *argv[])
// {
// 	return Ekf2::main(argc, argv);
// }
