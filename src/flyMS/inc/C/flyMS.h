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

#ifndef FLYMS_H
#define FLYMS_H

#pragma once
#include "pru_handler_client.h"
#include "filter.h"
#include "logger.h"
#include "gps.h"

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and 
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_8
#define BMP_CHECK_HZ	1



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

int initialize_dsm2MS();
float get_dsm2_ch_normalizedMS(int channel);
void* uart4_checkerMS(void *ptr); //background thread
int is_new_dsm2_dataMS();


 
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
	timespec 			start_time, start_loop, end_loop; //Some Structures to use to keep track of time during flight
	uint64_t 			start_time_usec, start_loop_usec, end_loop_usec; 
	}function_control_t;
	
	
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

	digital_filter_t *gyro_lpf[3];
	digital_filter_t *accel_lpf[3];

}filters_t;

typedef struct flyMS_threads_t{
	pthread_t led_thread;
	pthread_t core_logging_thread;
	pthread_t setpoint_manager_thread;
	pthread_t ekf_thread;
	pthread_t flight_core;
}flyMS_threads_t;




int initialize_filters(filters_t *filters, core_config_t *flight_config);
int initialize_flight_program(	control_variables_t *control,
                                filters_t *filters,
                                pru_client_data_t *pru_client_data,
                                GPS_data_t *GPS_data);
int flyMS_shutdown(	GPS_data_t *GPS_data);
uint64_t get_usec_timespec(timespec *tv);
int flyMS_console_print(control_variables_t *control);

#endif
