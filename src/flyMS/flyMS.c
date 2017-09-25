/*
Copyright (c) 2017, Mike Sardonini
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


// flyMS.c Control Program to fly quadcopter
// By Michael Sardonini

#ifdef __cplusplus
extern "C" {
#endif


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include "filter.h"
#include "gps.h"
#include "logger.h"
#include "config.h"
#include "Fusion.h"
#include "flyMS.h"
#include "ekf2.h"
#include "pru_handler_client.h"


void* flight_core(void * ptr);


/************************************************************************
* 	Global Variables				
************************************************************************/
control_variables_t		control;			//Structure to contain all system states
fusion_data_t			fusion;				//Structure to hold the data for IMU fusion (raw imu --> Euler angles)
setpoint_t 				setpoint; 			//Structure to store all Setpoints
GPS_data_t				GPS_data;			//Structure to store data from GPS
function_control_t 		function_control;	//Structure to store variables which control functions
filters_t				filters;			//Struct to contain all the filters
logger_t				logger;
transform_matrix_t		transform;
core_config_t 			flight_config;
pru_client_data_t		pru_client_data;
flyMS_threads_t			flyMS_threads;
uint16_t 				imu_err_count;
rc_imu_data_t			imu_data;			//Struct to relay all IMU info from driver to here
ekf_filter_t		 	ekf_filter;

void * setpoint_manager(void* ptr)
{
	while(rc_get_state()!=EXITING)
	{
		/**********************************************************
		*           Read the RC Controller for Commands           *
		**********************************************************/
	
		if(rc_is_new_dsm_data()){
			//Reset the timout counter back to zero
			function_control.dsm2_timeout=0;
			
			//Set roll reference value
			//DSM2 Receiver is inherently positive to the left
			setpoint.roll_ref= -rc_get_dsm_ch_normalized(2)*MAX_ROLL_RANGE;	
			
			//Set the pitch reference value
			setpoint.pitch_ref= -rc_get_dsm_ch_normalized(3)*MAX_PITCH_RANGE;
			
			
			//If Specified by the config file, convert from Drone Coordinate System to User Coordinate System
			if (flight_config.static_PR_ref)
			{				
				float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
				float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
				setpoint.roll_ref =P_R_MAG*cos(Theta_Ref+control.yaw[0]-control.yaw_ref_offset);
				setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref+control.yaw[0]-control.yaw_ref_offset);
			}
						
			//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
			//Apply the integration outside of current if statement, needs to run at 200Hz
			setpoint.yaw_rate_ref[1]=setpoint.yaw_rate_ref[0];		
			setpoint.yaw_rate_ref[0]=rc_get_dsm_ch_normalized(4)*MAX_YAW_RATE;
			
			//Apply a deadzone to keep integrator from wandering
			if(fabs(setpoint.yaw_rate_ref[0])<0.05) {
				setpoint.yaw_rate_ref[0]=0;
			}
			//Kill Switch
			control.kill_switch[0]=rc_get_dsm_ch_normalized(5)/2;
			
			//Auxillary Switch
			setpoint.Aux[1] = setpoint.Aux[0];
			setpoint.Aux[0]= rc_get_dsm_ch_normalized(6); 
			
			//Set the throttle
			control.throttle=(rc_get_dsm_ch_normalized(1)+1)* 0.5f *
					(flight_config.max_throttle-flight_config.min_throttle)+flight_config.min_throttle;
			//Keep the aircraft at a constant height while making manuevers 
			control.throttle *= 1/(cos(control.pitch)*cos(control.roll));
			
			//Future work, set variables for autonomous flight
			if(setpoint.Aux[0] < 0 && flight_config.enable_autonomy)
			{ 
				setpoint.altitudeSetpointRate = rc_get_dsm_ch_normalized(1) * MAX_ALT_SPEED;
				//Apply a deadzone for altitude hold
				if(fabs(setpoint.altitudeSetpointRate - control.standing_throttle)<0.05)
				{
					setpoint.altitudeSetpointRate=0;
				}
				//Detect if this is the first iteration switching to controlled flight
				if (setpoint.Aux[1] > 0) 
				{
					function_control.altitudeHoldFirstIteration = 1;
				}
				function_control.altitudeHold = 1;
			}
			else  
			{
				function_control.altitudeHold = 0;
			}
		}
		
		//check to make sure too much time hasn't gone by since hearing the RC
		else{
			if(!flight_config.enable_debug_mode)
			{
				function_control.dsm2_timeout=function_control.dsm2_timeout+1;
				if(function_control.dsm2_timeout>1.5/DT) {
					printf("\nLost Connection with Remote!! Shutting Down Immediately \n");	
					fprintf(logger.Error_logger,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
					rc_set_state(EXITING);
				}
			}
		}
		//Finally Update the integrator on the yaw reference value
		setpoint.yaw_ref[1]=setpoint.yaw_ref[0];
		setpoint.yaw_ref[0]=setpoint.yaw_ref[1]+(setpoint.yaw_rate_ref[0]+setpoint.yaw_rate_ref[1])*DT/2;
		
		usleep(DT_US); //Run at 200Hz
	}	
	return NULL;
}



void* flight_core(void* ptr){
	imu_err_count = 0;
	//control_variables_t *STATE = (control_variables_t*)ptr;
	//printf("pointer value %f\n", STATE->pitch);
	//Keep an i for loops
	static uint8_t i=0, i1=0, first_iteration_count=0;

	//Variables to Initialize things upon startup
	static uint8_t First_Iteration=1, First_Iteration_GPS=1;
	
	static float initial_alt = 0;
	//Initialize some variables if it is the first iteration
	if(First_Iteration){
		
		//Set the reference time to the first iteration
		function_control.start_time_usec = get_usec_timespec(&function_control.start_time);

		setpoint.Aux[0] = 1; control.kill_switch[0]=1;
		function_control.dsm2_timeout=0;
		rc_read_barometer();
		initial_alt = rc_bmp_get_altitude_m();
		rc_set_state(RUNNING);

		fprintf(logger.GPS_logger,"time,deg_lon,min_lon,deg_lat,min_lat,speed,direction,gps_alt,hdop,fix\n");
		fflush(logger.GPS_logger);
		printf("First Iteration ");
		}

	while(rc_get_state()!=EXITING)
	{

		/******************************************************************
					Grab the time for Periphal Apps and Logs 			  *
		******************************************************************/
		function_control.start_loop_usec = get_usec_timespec(&function_control.start_loop);
		control.time = (float)(function_control.start_loop_usec - function_control.start_time_usec)/1E6f;


		/**********************************************************
		*    				Read the Raw IMU Data     			  *
		**********************************************************/
		if(rc_read_accel_data(&imu_data)<0){
			printf("read accel data failed\n");
		}
	//	if(rc_read_gyro_data(&imu_data)<0){
	//		printf("read gyro data failed\n");
	//	}
		if(rc_read_mag_data(&imu_data)<0){
			printf("read mag data failed\n");
		}
/*
		for (i = 0; i < 3; i++)
		{
			imu_data.gyro[i] = update_filter(filters.gyro_lpf[i],imu_data.gyro[i]);
			imu_data.accel[i] = update_filter(filters.accel_lpf[i],imu_data.accel[i]);

		}
	*/	updateFusion(&imu_data, &fusion);

		//Bring 3 axes of accel, gyro and angle data in to this program
		for (i=0;i<3;i++) 
		{	
			transform.dmp_imu.d[i] = fusion.eulerAngles.array[i] * DEG_TO_RAD;
			transform.gyro_imu.d[i] = imu_data.gyro[i] * DEG_TO_RAD;
			transform.accel_imu.d[i] = imu_data.accel[i];
		}
		//Convert from IMU coordinate system to drone's
		rc_matrix_times_col_vec(transform.IMU_to_drone_dmp, transform.dmp_imu, &transform.dmp_drone);
		rc_matrix_times_col_vec(transform.IMU_to_drone_gyro, transform.gyro_imu, &transform.gyro_drone);
		rc_matrix_times_col_vec(transform.IMU_to_drone_accel, transform.accel_imu, &transform.accel_drone);

	//	control.pitch 			= fusion.eulerAngles.angle.pitch;
	//	control.roll 			= fusion.eulerAngles.angle.roll;
	//	control.yaw[1] 			= control.yaw[0];	
	//	control.yaw[0] 			= fusion.eulerAngles.angle.yaw + control.num_wraps*2*M_PI;
		
	//	if(fabsf(control.yaw[0] - control.yaw[1])  > 5)
	//	{
	//		if(control.yaw[0] > control.yaw[1]) control.num_wraps--;
	//		if(control.yaw[0] < control.yaw[1]) control.num_wraps++;
	//	}
	//	control.yaw[0]= fusion.eulerAngles.angle.yaw + control.num_wraps*2*M_PI;	
		
		
		control.pitch 			= transform.dmp_drone.d[0];
		control.roll 			= transform.dmp_drone.d[1];
		control.compass_heading = imu_data.compass_heading_raw;	
		control.yaw[1] 			= control.yaw[0];	
		control.yaw[0] 			= transform.dmp_drone.d[2] + control.num_wraps*2*M_PI;

		if(fabs(control.yaw[0] - control.yaw[1])  > 5)
		{
			if(control.yaw[0] > control.yaw[1]) control.num_wraps--;
			if(control.yaw[0] < control.yaw[1]) control.num_wraps++;
		}
		control.yaw[0]= transform.dmp_drone.d[2] + control.num_wraps*2*M_PI;
		control.d_pitch			= transform.gyro_drone.d[0];
		control.d_roll			= transform.gyro_drone.d[1];
		control.d_yaw			= transform.gyro_drone.d[2];
		
		//Allow some time for imu to settle
		if (first_iteration_count < 150)
		{
			first_iteration_count++;
			continue;
		}	


		if(First_Iteration){
			setpoint.yaw_ref[0]=control.yaw[0];
			First_Iteration=0;
			control.yaw_ref_offset = control.yaw[0];
			printf("Started \n");
		}
		/**********************************************************
		*           Read the Barometer for Altitude				  *
		**********************************************************/	
		if (flight_config.enable_barometer)
		{		
			i1++;
			if (i1 == 1) // Only read the barometer at 25Hz
			{
				// perform the i2c reads to the sensor, this takes a bit of time
				if(rc_read_barometer()<0){
					printf("\rERROR: Can't read Barometer");
					fflush(stdout);
				}
				i1=0;
			}
			control.baro_alt = update_filter(filters.LPF_baro_alt,rc_bmp_get_altitude_m() - initial_alt);
			ekf_filter.input.barometer_updated = 1;
			ekf_filter.input.barometer_alt = control.baro_alt;
		}

		/************************************************************************
		*                   	Send data to the EKF                            *
		************************************************************************/
		for (i = 0; i < 3; i++)
		{
			ekf_filter.input.accel[i] = transform.accel_drone.d[i];
			ekf_filter.input.mag[i] = imu_data.mag[i] * MICROTESLA_TO_GAUSS;
		}
		ekf_filter.input.gyro[0] = control.d_pitch;
		ekf_filter.input.gyro[1] = control.d_roll;
		ekf_filter.input.gyro[2] = control.d_yaw;
		ekf_filter.input.IMU_timestamp = control.time;

		/************************************************************************
		*                   	Throttle Controller                             *
		************************************************************************/

	//	float throttle_compensation = 1 / cos(control.roll);
	//	throttle_compensation *= 1 / cos(control.pitch);		

		if(function_control.altitudeHold)
		{
			if(function_control.altitudeHoldFirstIteration)
			{
				control.standing_throttle = control.throttle;
				setpoint.altitudeSetpoint = control.baro_alt;
				function_control.altitudeHoldFirstIteration = 0;
			}
	       // setpoint.altitudeSetpoint=setpoint.altitudeSetpoint+(setpoint.altitudeSetpointRate)*DT;

			//control.uthrottle = update_filter(filters.altitudeHoldPID, setpoint.altitudeSetpoint - control.baro_alt);
			//control.throttle = control.uthrottle + control.standing_throttle;
			
		}
		/************************************************************************
		* 	                  Pitch and Roll Controllers                        *
		************************************************************************/
		if(setpoint.Aux>=0 || 1){
			//Using Remote Control
			function_control.gps_pos_mode=0;
		}
		else{
			//Using GPS Control
			if(function_control.gps_pos_mode==0){
				printf("gps position mode\n");
				setpoint.lat_setpoint=GPS_data.pos_lat;
				setpoint.lon_setpoint=GPS_data.pos_lon;
				function_control.gps_pos_mode=1;
				control.standing_throttle=control.throttle;
				control.alt_ref=GPS_data.gps_altitude+1;
			}
			control.lat_error=setpoint.lat_setpoint-GPS_data.pos_lat;
			control.lon_error=setpoint.lon_setpoint-GPS_data.pos_lon;
			
			setpoint.pitch_ref=0.14*update_filter(filters.Outer_Loop_TF_pitch,control.lat_error);
			setpoint.roll_ref=-0.14*update_filter(filters.Outer_Loop_TF_roll,control.lon_error);
			
			setpoint.pitch_ref=saturateFilter(setpoint.pitch_ref,-0.2,0.2);
			setpoint.roll_ref=saturateFilter(setpoint.roll_ref,-0.2,0.2);
			
			//Convert to Drone Coordinate System from User Coordinate System
			float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
			float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
			setpoint.roll_ref=P_R_MAG*cos(Theta_Ref-control.yaw[0]);
			setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref-control.yaw[0]);
			
			control.alt_error=control.alt_ref-GPS_data.gps_altitude;
			//control.throttle=0.12*update_filter(&filters.Throttle_controller,control.alt_error);
			
			//control.throttle=saturateFilter(control.throttle,-0.15,0.15)+control.standing_throttle;
		}
		//Filter out any high freq noise coming from yaw in the CS translation
	//	setpoint.filt_pitch_ref = update_filter(filters.LPF_Yaw_Ref_P,setpoint.pitch_ref);
	//	setpoint.filt_roll_ref = update_filter(filters.LPF_Yaw_Ref_R,setpoint.roll_ref);
		
		//Filter out high frequency noise in Raw Gyro data
//		control.d_pitch_f = update_filter(filters.LPF_d_pitch,control.d_pitch);			
//		control.d_roll_f = update_filter(filters.LPF_d_roll,control.d_roll);
		
		control.dpitch_setpoint = update_filter(filters.pitch_PD, setpoint.filt_pitch_ref - control.pitch);
		
		control.droll_setpoint = update_filter(filters.roll_PD, setpoint.filt_roll_ref - control.roll);
		
		//Apply the PD Controllers 
		control.upitch = update_filter(filters.pitch_rate_PD,control.dpitch_setpoint - control.d_pitch);
		control.uroll = update_filter(filters.roll_rate_PD,control.droll_setpoint - control.d_roll);				
		
		
		/************************************************************************
		*                        	Yaw Controller                              *
		************************************************************************/	
	//	control.d_yaw_f = update_filter(filters.LPF_d_yaw,control.d_yaw);
		
		setpoint.yaw_ref[1]=setpoint.yaw_ref[0];
		setpoint.yaw_ref[0]=setpoint.yaw_ref[1]+(setpoint.yaw_rate_ref[0]+setpoint.yaw_rate_ref[1])*DT/2;

		control.uyaw = update_filter(filters.yaw_rate_PD,setpoint.yaw_ref[0]-control.yaw[0]);
		
		/************************************************************************
		*                   	Apply the Integrators                           *
		************************************************************************/	
			
		if(control.throttle<MIN_THROTTLE+.01){	
			function_control.integrator_reset++;
			function_control.integrator_start=0;
		}else{
			function_control.integrator_reset=0;
			function_control.integrator_start++;
		}
		
		if(function_control.integrator_reset==300){// if landed, reset integrators and Yaw error
			setpoint.yaw_ref[0]=control.yaw[0];
			control.droll_err_integrator=0; 
			control.dpitch_err_integrator=0;
			control.dyaw_err_integrator=0;
		}
			
		//only use integrators if airborne (above minimum throttle for > 1.5 seconds)
		if(function_control.integrator_start >  400){
			control.droll_err_integrator  += control.uroll  * DT;
			control.dpitch_err_integrator += control.upitch * DT;
			control.dyaw_err_integrator += control.uyaw * DT;		
			
			control.upitch+=flight_config.Dpitch_KI * control.dpitch_err_integrator;
			control.uroll +=  flight_config.Droll_KI * control.droll_err_integrator;
			control.uyaw+=flight_config.yaw_KI * control.dyaw_err_integrator;
		}
		
		//Apply a saturation filter
		control.upitch = saturateFilter(control.upitch,-MAX_PITCH_COMPONENT,MAX_PITCH_COMPONENT);
		control.uroll = saturateFilter(control.uroll,-MAX_ROLL_COMPONENT,MAX_ROLL_COMPONENT);
		control.uyaw = saturateFilter(control.uyaw,-MAX_YAW_COMPONENT,MAX_YAW_COMPONENT);
		
		/************************************************************************
		*  Mixing
		*           	      black				yellow
		*                          CCW 1	  2 CW			
		*                          	   \ /				Y
		*	                           / \            	|_ X
		*                         CW 3	  4 CCW
		*                 	  yellow       	    black
		************************************************************************/
		
		control.u[0]=control.throttle+control.uroll+control.upitch-control.uyaw;
		control.u[1]=control.throttle-control.uroll+control.upitch+control.uyaw;
		control.u[2]=control.throttle+control.uroll-control.upitch+control.uyaw;
		control.u[3]=control.throttle-control.uroll-control.upitch-control.uyaw;		

		float largest_value = 1;
		float smallest_value = 0;

		for(i=0;i<4;i++){ 
			if(control.u[i]>largest_value)largest_value=control.u[i];
			
			if(control.u[i]<smallest_value)control.u[i]=0;
		}
				
		// if upper saturation would have occurred, reduce all outputs evenly
		if(largest_value>1){
			float offset = largest_value - 1;
			for(i=0;i<4;i++) control.u[i]-=offset;
		}

		if(!flight_config.enable_debug_mode)
		{
			//Send Commands to Motors
			for(i=0;i<4;i++){
				pru_client_data.u[i] = control.u[i];
				pru_client_data.send_flag = 1;
			}

			if(control.kill_switch[0] < .5) {
				printf("\nKill Switch Hit! Shutting Down\n");
				fprintf(logger.Error_logger,"\nKill Switch Hit! Shutting Down\n");	
				rc_set_state(EXITING);
			}
		}

		logger.new_entry.time			= control.time;	
		logger.new_entry.pitch			= control.pitch;	
		logger.new_entry.roll			= control.roll;
		logger.new_entry.yaw			= control.yaw[0];
		logger.new_entry.d_pitch		= control.d_pitch;	
		logger.new_entry.d_roll			= control.d_roll;
		logger.new_entry.d_yaw			= control.d_yaw;
		logger.new_entry.u_1			= control.u[0];
		logger.new_entry.u_2			= control.u[1];
		logger.new_entry.u_3			= control.u[2];
		logger.new_entry.u_4			= control.u[3];
		logger.new_entry.throttle		= control.throttle;
		logger.new_entry.upitch			= control.upitch;	
		logger.new_entry.uroll			= control.uroll;
		logger.new_entry.uyaw			= control.uyaw;
		logger.new_entry.pitch_ref		= setpoint.pitch_ref;
		logger.new_entry.roll_ref		= setpoint.roll_ref;
		logger.new_entry.yaw_ref		= setpoint.yaw_ref[0];
		logger.new_entry.yaw_rate_ref	= setpoint.yaw_rate_ref[0];
		logger.new_entry.Aux			= setpoint.Aux[0];
		logger.new_entry.lat_error		= control.lat_error;
		logger.new_entry.lon_error		= control.lon_error;
		logger.new_entry.accel_x		= transform.accel_drone.d[0];
		logger.new_entry.accel_y		= transform.accel_drone.d[1];
		logger.new_entry.accel_z		= transform.accel_drone.d[2];
		logger.new_entry.baro_alt		= control.baro_alt;
		logger.new_entry.v_batt			= 0;
		logger.new_entry.ned_pos_x		= ekf_filter.output.ned_pos[0];
		logger.new_entry.ned_pos_y		= ekf_filter.output.ned_pos[1];
		logger.new_entry.ned_pos_z		= ekf_filter.output.ned_pos[2];
		logger.new_entry.ned_vel_x		= ekf_filter.output.ned_vel[0];
		logger.new_entry.ned_vel_y		= ekf_filter.output.ned_vel[1];
		logger.new_entry.ned_vel_z		= ekf_filter.output.ned_vel[2];
		logger.new_entry.mag_x			= imu_data.mag[0];
		logger.new_entry.mag_y			= imu_data.mag[1];
		logger.new_entry.mag_z			= imu_data.mag[2];
		logger.new_entry.compass_heading= control.compass_heading;
		//logger.new_entry.v_batt			= rc_dc_jack_voltage();
		log_core_data(&logger.core_logger, &logger.new_entry);

		
		//Print some stuff if in debug mode
		if(flight_config.enable_debug_mode)
		{	
			printf("\r ");
		//	printf("time %3.3f ", control.time);
		//	printf("Alt %2.2f ",lidar_data.altitude[0]);
		//	printf("vel %2.2f ",lidar_data.d_altitude[0]);		
		//	printf("H_d %3.3f ", control.height_damping);
		//	printf("Alt_ref %3.1f ",control.alt_ref);
		//	printf(" U1:  %2.2f ",control.u[0]);
		//	printf(" U2: %2.2f ",control.u[1]);
		//	printf(" U3:  %2.2f ",control.u[2]);
		//	printf(" U4: %2.2f ",control.u[3]);	
		//	printf(" Throt %2.2f ", control.throttle);
		//	printf("Aux %2.1f ", setpoint.Aux[0]);
		//	printf("function: %f",rc_get_dsm_ch_normalized(6));
		//	printf("num wraps %d ",control.num_wraps);
		//	printf(" Pitch_ref %2.2f ", setpoint.filt_pitch_ref);
		//	printf(" Roll_ref %2.2f ", setpoint.filt_roll_ref);
		//	printf(" Yaw_ref %2.2f ", setpoint.yaw_ref[0]);
			printf(" Pitch %1.2f ", control.pitch);
			printf(" Roll %1.2f ", control.roll);
			printf(" Yaw %2.3f ", control.yaw[0]); 
		//	printf(" Mag X %4.2f",imu_data.mag[0]);
		//	printf(" Mag Y %4.2f",imu_data.mag[1]);
		//	printf(" Mag Z %4.2f",imu_data.mag[2]);
			// printf(" Pos N %2.3f ", ekf_filter.output.ned_pos[0]); 
			// printf(" Pos E %2.3f ", ekf_filter.output.ned_pos[1]); 
			// printf(" Pos D %2.3f ", ekf_filter.output.ned_pos[2]); 
			printf(" DPitch %1.2f ", control.d_pitch); 
			printf(" DRoll %1.2f ", control.d_roll);
			printf(" DYaw %2.3f ", control.d_yaw); 	
		//	printf(" uyaw %2.3f ", control.upitch); 		
		//	printf(" uyaw %2.3f ", control.uroll); 		
		//	printf(" uyaw %2.3f ", control.uyaw);
		//	printf(" GPS pos lat: %2.2f", GPS_data.pos_lat);
		//	printf(" GPS pos lon: %2.2f", GPS_data.pos_lon);
		//	printf(" HDOP: %f", GPS_data.HDOP);
		//	printf(" Pos_Lat %2.3f ", X_state_Lat1->data[0]);	
		//	printf(" Pos_Lon %2.3f ", X_state_Lon1->data[0]);
		//	printf("control: %d",rc_get_state());
			printf("Baro Alt: %f ",control.baro_alt);
			fflush(stdout);
		}

			/***************** Get GPS Data if available *******************/
		if(is_new_GPS_data())
		{

			 //printf("\n new GPS data in \n");
			fprintf(logger.GPS_logger,"%4.5f,",control.time);
			fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_longitude,GPS_data.min_longitude);
			fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_latitude,GPS_data.min_latitude);
			fprintf(logger.GPS_logger,"%f,%f,",GPS_data.speed,GPS_data.direction);
			fprintf(logger.GPS_logger,"%f,",GPS_data.gps_altitude);
			fprintf(logger.GPS_logger,"%2.2f,%d",GPS_data.HDOP,GPS_data.GPS_fix);
			fprintf(logger.GPS_logger,"\n");
			fflush(logger.GPS_logger);
			//	printf("%s",GPS_data.GGAbuf);
			//	printf("%s",GPS_data.VTGbuf);

			if (First_Iteration_GPS==1  && GPS_data.GPS_fix==1){
				control.initial_pos_lat=GPS_data.meters_lat;
				control.initial_pos_lon=GPS_data.meters_lon;
				First_Iteration_GPS=0;
				GPS_data.GPS_fix_check=1;
				printf("First Iteration GPS\n");
			}
			if(GPS_data.HDOP<4 && GPS_data.GPS_fix==1){
				GPS_data.pos_lat=GPS_data.meters_lat-control.initial_pos_lat;
				GPS_data.pos_lon=GPS_data.meters_lon-control.initial_pos_lon;
			}

			ekf_filter.input.gps_updated = 1;
			ekf_filter.input.gps_timestamp = control.time*1E6;
			ekf_filter.input.gps_latlon[0] = (double)GPS_data.deg_latitude + (double)GPS_data.min_latitude / 60.0;// + control.time*1E7/20000;
			ekf_filter.input.gps_latlon[1] = (double)GPS_data.deg_longitude + (double)GPS_data.min_longitude / 60.0;
			ekf_filter.input.gps_latlon[2] = (double)GPS_data.gps_altitude;
			ekf_filter.input.gps_fix = GPS_data.GPS_fix;
			ekf_filter.input.nsats = 10; // Really need to fix this
		}

	function_control.end_loop_usec = get_usec_timespec(&function_control.end_loop);
	
	if (function_control.end_loop_usec - function_control.start_loop_usec > 1E6) printf("Error timeout detected!\n");
	uint64_t sleep_time = DT_US - (function_control.end_loop_usec -
									function_control.start_loop_usec);

	//Check to make sure the elapsed time wasn't greater than time allowed. If so don't sleep at all
	if (sleep_time < DT_US)	rc_usleep(sleep_time);

	}
	return NULL;
}
	
int main(int argc, char *argv[]){

	//Initialize some cape and beaglebone hardware
	if(rc_initialize()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}

	// load flight_core settings
	if(load_core_config(&flight_config)){
		printf("WARNING: no configuration file found\n");
		printf("loading default settings\n");
		if(create_default_core_config_file(&flight_config)){
			printf("Warning, can't write default flight_config file\n");
		}
	}
	
	int in;
	while ((in = getopt(argc, argv, "d")) != -1)
	{
		switch (in)
		{
			case 'd': 
				flight_config.enable_debug_mode = 1;
				printf("Running in Debug mode \n");
				break;	
			default:
				printf("Invalid Argument \n");
				return -1;
				break;
		}
	}

 	/************************************************************************
	*                    Initialize the Flight Program                      *
	************************************************************************/	
	if(initialize_flight_program(&flyMS_threads,
                               &flight_config,
                               &logger,
                               &filters,
                               &pru_client_data,
                               &imu_data,
                               &transform,
                               &GPS_data,
                               &ekf_filter,
                               &fusion))
	{
		flyMS_shutdown( &logger, 
						&GPS_data, 
						&flyMS_threads); 
		rc_cleanup();
		return -1;
	}
	
	//Start the control program
	pthread_create(&flyMS_threads.flight_core, NULL, flight_core, (void*)NULL);

	printf("Starting \n");
	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) {
		usleep(5000);
		imu_err_count++;
		if (imu_err_count == 5 || imu_err_count % 50 == 0)
		{
			fprintf(logger.Error_logger,"Error! IMU read failed for more than\
											5 consecutive timesteps. time: = %f\
											number of missed reads: %u \n",
											control.time,imu_err_count);
		}
	}

	flyMS_shutdown( &logger, 
					&GPS_data, 
					&flyMS_threads); 
	rc_cleanup();
	return 0;
	}

GPS_data_t* get_GPS_pointer(){
	return &GPS_data;
}

#ifdef __cplusplus
}
#endif
