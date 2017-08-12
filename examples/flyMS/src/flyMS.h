/*
Copyright (c) 2014, Mike Sardonini
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

#pragma once
#include "linear_algebra.h"
#include "../../../libraries/roboticscape.h"
#include "../../../libraries/pru_handler_client.h"
#include "filter.h"
#include "config.h"
#include "logger.h"
#include "gps.h"
#include "kalman.h"

#ifndef FLIGHT_DEFS_H
#define FLIGHT_DEFS_H


// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and 
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_8
#define BMP_CHECK_HZ	1



#define SAMPLE_RATE	200
#define DT 0.005
#define Integrator_TH 0.55
#define MAX_PITCH_RANGE 0.666 // in radians
#define MAX_ROLL_RANGE 0.666 // in radians
#define MAX_YAW_RATE 2.0 //in Radians per second
#define MIN_THROTTLE 0.3
#define MAX_THROTTLE 0.75
#define MAX_PITCH_COMPONENT 0.25
#define MAX_ROLL_COMPONENT 0.25
#define MAX_YAW_COMPONENT 0.25
#define DEG_TO_RAD 0.01745
#define MAX_ALT_SPEED 0.2 //in meters/second


#define LAT_ACCEL_BIAS 0.02798
#define LON_ACCEL_BIAS -0.1173
#define ALT_ACCEL_BIAS 0.0


/************************** Orientation Matrix Constants *****************************/	
#define cR1 cos(roll_offset)
#define sR1 sin(roll_offset)
#define cP1 cos(pitch_offset)
#define sP1 sin(pitch_offset)
#define cY1 cos(yaw_offset)
#define sY1 sin(yaw_offset)

#define ROTATION_MATRIX1	{{cR1*cY1, -cP1*sY1+sP1*sR1*cY1,  sP1*sY1+cP1*sR1*cY1}, \
							 {cR1*sY1,  cP1*cY1+sP1*sR1*sY1, -sP1*cY1+cP1*sR1*sY1}, \
							 {-sR1  ,  sP1*cR1,		   cP1*cR1		 }}

/******************************* Kalman Filter Constants *****************************/							 
#define PI 3.14
#define SIGMA_A .1
#define SIGMA_P 1

#define cR cos(accel_data_kal->roll)
#define sR sin(accel_data_kal->roll)
#define cP cos(accel_data_kal->pitch)
#define sP sin(accel_data_kal->pitch) 
#define cY cos(accel_data_kal->yaw[0])
#define sY sin(accel_data_kal->yaw[0])

#define ROTATION_MATRIX		{{cP*cY, -cR*sY+sR*sP*cY,  sR*sY+cR*sP*cY}, \
							 {cP*sY,  cR*cY+sR*sP*sY, -sR*cY+cR*sP*sY}, \
							 {-sP  ,  sR*cP,		   cR*cP		 }}


 #define Q_MATRIX			{{DT*DT*DT*DT/4*SIGMA_P*SIGMA_P, DT*DT*DT/2*SIGMA_P*SIGMA_P, 0}, \
							 {DT*DT*DT/2*SIGMA_P*SIGMA_P, DT*DT*SIGMA_P*SIGMA_P,	  0}, \
							 {0, 0, SIGMA_P}}
							 
 #define R_MATRIX SIGMA_A
 
 #define A_MATRIX 			{{1, DT, -DT*DT/2}, \
							 {0, 1,  -DT}, \
							 {0, 0,   1}};
							 
 #define B_MATRIX			{{DT*DT/2}, \
							 {DT}, \
							 {0}}
							 
 #define H_MATRIX			{{1}, \
							 {0}, \
							 {0}}
				 
  #define EYE 				{{1, 0, 0}, \
							 {0, 1, 0}, \
							 {0, 0, 1}};

int initialize_dsm2MS();
float get_dsm2_ch_normalizedMS(int channel);
void* uart4_checkerMS(void *ptr); //background thread
int is_new_dsm2_dataMS();


typedef struct control_variables_t{
	float	pitch, roll, yaw[2];		// Euler angles of aircraft
	float	compass_heading;
	float	d_pitch, d_roll, d_yaw; 			// First derivative of Euler Angles	
	float	d_pitch_f, d_roll_f, d_yaw_f; 		// Filtered First derivative of Eulter Angles	
	int		mag0, mag1, mag2;					// Magnetometer Values
	float	dpitch_setpoint, droll_setpoint;	// Desired attitude
	int		num_wraps;				// Number of spins in Yaw
	float	unwrapped_yaw[2];					// Some Yaw Varibles
	float	initial_yaw;
	float 	throttle;				
	float	droll_err_integrator;
	float	dpitch_err_integrator;
	float	dyaw_err_integrator;
	float 	uyaw, upitch, uroll, uthrottle;		// Controller effort for each state variable
	float	u[4]; 								// Duty Cycle to send to each motor
	float	time; 								// Time since execution of the program
	float	yaw_ref_offset;
	float 	alt_rate_ref, d_alt_filt, alt_ref;	//Height Variables for control with Lidar
	float	height_damping;
	float	baro_alt;							// Barometer Altitude
	double	initial_pos_lon, initial_pos_lat; 	// Lat & Long positions from GPS
	double	lat_error, lon_error;
	float 	kill_switch[2];
 
	float	standing_throttle, alt_error;
}control_variables_t;

typedef struct setpoint_t{
	float	pitch_ref, roll_ref, yaw_ref[2];	// Reference (Desired) Position
	float	filt_pitch_ref, filt_roll_ref;		// LPF of pitch and roll (because they are a func of yaw)
	float	yaw_rate_ref[2];
	float	Aux[2];
	double	lat_setpoint, lon_setpoint;			// Controller Variables for Autonomous Flight
	float	altitudeSetpointRate;
	float	altitudeSetpoint;
}setpoint_t;


typedef struct accel_data_t{
	rc_vector_t  X_state_Lat, X_state_Lon;
	float accel_x, accel_y, accel_z; // Accelerometer Values for Kalman use
	float	pitch, roll, yaw[2];
	uint8_t GPS_kal_flag;			//flag to signal that GPS data is ready 
}accel_data_t;
 
typedef struct function_control_t{
	int 				Lidar_kill_counter;					//Kill the lidar thread if returns negative 20 times consecutively
	int 				dsm2_timeout;						//Shutdown the system if it lost communication with the RC
	int 				alt_pos_mode;						//Signal the first iteration of Altitude position mode
	int					gps_pos_mode;						//Signal the first iteration of GPS position mode
	int					yaw_err_timout;						//Reset Yaw Error if landed for more than 1 second
	int					integrator_reset;
	int					integrator_start;
	uint8_t				altitudeHold;
	uint8_t				altitudeHoldFirstIteration;
	pthread_mutex_t 	lock;
	timespec 			start_time, log_time; //Some Structures to use to keep track of time during flight
	}function_control_t;
	
typedef struct transform_matrix_t{
	rc_matrix_t 	IMU_to_drone_dmp, IMU_to_drone_gyro, IMU_to_drone_accel;
	rc_vector_t 	dmp_imu, gyro_imu, accel_imu;
	rc_vector_t 	dmp_drone, gyro_drone, accel_drone;
}transform_matrix_t;

typedef struct ekf_filter_input_t{
	float mag[3];
	float gyro[3];
	float accel[3];
	
	uint8_t barometer_updated;
	float barometer_pressure;

	uint8_t gps_updated;
	uint64_t gps_timestamp;
	double gps_latlon[3];
	uint8_t gps_fix;
	uint8_t nsats;

	uint8_t vehicle_land;

}ekf_filter_input_t;


typedef struct ekf_filter_output_t{
	double ned_pos[3];
	double ned_vel[3];
	double ned_acc[3];

	float vertical_time_deriv;
	float gyro[3];

}ekf_filter_output_t;

typedef struct ekf_filter_t{
	ekf_filter_input_t input;
	ekf_filter_output_t output;

}ekf_filter_t;

	
typedef struct filters_t{
	digital_filter_t			*pitch_rate_PD;
	digital_filter_t			*roll_rate_PD;
	digital_filter_t			*yaw_rate_PD;
	digital_filter_t			*pitch_PD;
	digital_filter_t			*roll_PD;
	digital_filter_t			*yaw_PD;
	digital_filter_t         	*LPF_d_pitch;
	digital_filter_t         	*LPF_d_roll;
	digital_filter_t         	*LPF_d_yaw;
	digital_filter_t         	*LPF_Yaw_Ref_P;
	digital_filter_t        	*LPF_Yaw_Ref_R;
	digital_filter_t			*Outer_Loop_TF_pitch;
	digital_filter_t			*Outer_Loop_TF_roll;
	digital_filter_t 			*LPF_Accel_Lat;
	digital_filter_t 			*LPF_Accel_Lon;
	digital_filter_t			*LPF_pitch;
	digital_filter_t			*LPF_roll;	
	digital_filter_t			*altitudeHoldPID;
	digital_filter_t			*LPF_baro_alt;
}filters_t;

typedef struct flyMS_threads_t{
	pthread_t led_thread;
	pthread_t kalman_thread;
	pthread_t core_logging_thread;
	pthread_t setpoint_manager_thread;
}flyMS_threads_t;

typedef struct led_thread_t{
	uint8_t GPS_init_check;
	uint8_t GPS_fix_check;	
}led_thread_t;


int ready_check();
void zero_escs();
accel_data_t* get_accel_pointer();
void* barometer_monitor();
int initialize_filters(filters_t *filters, core_config_t *flight_config);
int init_rotation_matrix(transform_matrix_t *transform, core_config_t *flight_config);
void* LED_thread(void *ptr);
void init_esc_hardware();
void* quietEscs(void *ptr);
int initialize_flight_program(flyMS_threads_t *flyMS_threads,
                                core_config_t *flight_config,
                                logger_t *logger,
                                filters_t *filters,
                                pru_client_data_t *pru_client_data,
                                rc_imu_data_t *imu_data,
                                transform_matrix_t *transform,
                                GPS_data_t *GPS_data);


int flyMS_shutdown(			logger_t *logger, 
					GPS_data_t *GPS_data, 
					flyMS_threads_t *flyMS_threads);
void* pru_sender(void* ptr);
void* setpoint_manager(void* ptr);


#endif
