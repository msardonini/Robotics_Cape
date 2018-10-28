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
#ifndef FLYMS_COMMON_H
#define FLYMS_COMMON_H

#pragma once
#include "Fusion.h"
#include "config.h"

#define SAMPLE_RATE	200
#define DT 0.005f
#define DT_US 5000
#define MICROTESLA_TO_GAUSS 0.01f
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


typedef struct imu_t{
	float	compass_heading;
	float	mag[3];
	float	gyro[3];
	float	accel[3];
	float	baro_alt;							// Barometer Altitude
}imu_t;



typedef struct state_t{
	float	euler[3];					// Euler angles of aircraft (in roll, pitch, yaw)
	float	euler_previous[3];			// 1 Timestampe previousEuler angles of aircraft (in roll, pitch, yaw)
	float	euler_rate[3];				// First derivative of euler angles (in roll/s, pitch/s, yaw/s)
	
	int		num_wraps;				// Number of spins in Yaw
	float	initial_yaw;
}state_t;

typedef struct control_variables_t{
	uint64_t	time_us; 								// Time since execution of the program
 
	state_t state;						//	System state information
	imu_t imu;							//	Data measured directly by IMU
	controller_t control;			//	Data contained within the PID controller
	core_config_t flight_config;		//	Struct to contrain all of the configuration data
	ekf_filter_t ekf_filter;			//	Struct which passes data to be sent and received from EKF
	setpoint_t setpoint;				//	Struct which contains all of the controller setpoint values 
}control_variables_t;




#endif