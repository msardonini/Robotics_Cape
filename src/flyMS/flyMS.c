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
#include "imu_handler.h"


void* flight_core(void * ptr);


/************************************************************************
* 	Global Variables				
************************************************************************/
control_variables_t		control;			//Structure to contain all system states
setpoint_t 				setpoint; 			//Structure to store all Setpoints
GPS_data_t				GPS_data;			//Structure to store data from GPS
function_control_t 		function_control;	//Structure to store variables which control functions
filters_t				filters;			//Struct to contain all the filters
pru_client_data_t		pru_client_data;
flyMS_threads_t			flyMS_threads;
uint16_t 				imu_err_count;


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
			if (control.flight_config.static_PR_ref)
			{				
				//Set roll reference value
				//DSM2 Receiver is inherently positive to the left
				setpoint.roll_ref= -rc_get_dsm_ch_normalized(2)*MAX_ROLL_RANGE;	

				//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
				//Apply the integration outside of current if statement, needs to run at 200Hz
				setpoint.yaw_rate_ref[1]=setpoint.yaw_rate_ref[0];		
				setpoint.yaw_rate_ref[0]=rc_get_dsm_ch_normalized(4)*MAX_YAW_RATE;

				float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
				float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
				setpoint.roll_ref =P_R_MAG*cos(Theta_Ref+control.euler[2]-control.yaw_ref_offset);
				setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref+control.euler[2]-control.yaw_ref_offset);
			}
			else	//This is flying FPV mode
			{
				//Set roll reference value
				//DSM2 Receiver is inherently positive to the left
				setpoint.roll_ref= -rc_get_dsm_ch_normalized(2)*MAX_ROLL_RANGE;	

				//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
				//Apply the integration outside of current if statement, needs to run at 200Hz
				setpoint.yaw_rate_ref[1]=setpoint.yaw_rate_ref[0];		
				setpoint.yaw_rate_ref[0]=rc_get_dsm_ch_normalized(4)*MAX_YAW_RATE;

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
					(control.flight_config.max_throttle-control.flight_config.min_throttle)+control.flight_config.min_throttle;
			//Keep the aircraft at a constant height while making manuevers 
			control.throttle *= 1/(cos(control.euler[1])*cos(control.euler[0]));
			
			//Future work, set variables for autonomous flight
			if(setpoint.Aux[0] < 0 && control.flight_config.enable_autonomy)
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
			if(!control.flight_config.enable_debug_mode)
			{
				function_control.dsm2_timeout=function_control.dsm2_timeout+1;
				if(function_control.dsm2_timeout>1.5/DT) {
					printf("\nLost Connection with Remote!! Shutting Down Immediately \n");	
					fprintf(control.logger.Error_logger,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
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
	static uint8_t i=0, first_iteration_count=0;

	//Variables to Initialize things upon startup
	static uint8_t First_Iteration=1, First_Iteration_GPS=1;
	
	// float initial_alt = 0;
	//Initialize some variables if it is the first iteration
	if(First_Iteration){
		
		//Set the reference time to the first iteration
		function_control.start_time_usec = get_usec_timespec(&function_control.start_time);

		setpoint.Aux[0] = 1; control.kill_switch[0]=1;
		function_control.dsm2_timeout=0;
		rc_read_barometer();
		// initial_alt = rc_bmp_get_altitude_m();
		rc_set_state(RUNNING);

		fprintf(control.logger.GPS_logger,"time,deg_lon,min_lon,deg_lat,min_lat,speed,direction,gps_alt,hdop,fix\n");
		fflush(control.logger.GPS_logger);
		printf("First Iteration ");
		}

	while(rc_get_state()!=EXITING)
	{

		/******************************************************************
					Grab the time for Periphal Apps and Logs 			  *
		******************************************************************/
		function_control.start_loop_usec = get_usec_timespec(&function_control.start_loop);
		control.time = (float)(function_control.start_loop_usec - function_control.start_time_usec)/1E6f;
		

		/******************************************************************
					Read, Parse, and Translate IMU data for Flight		  *
		******************************************************************/
		imu_handler(&control);

		//Allow some time for imu to settle
		if (first_iteration_count < 150)
		{
			first_iteration_count++;
			continue;
		}	


		if(First_Iteration){
			setpoint.yaw_ref[0]=control.euler[2];
			First_Iteration=0;
			control.yaw_ref_offset = control.euler[2];
			printf("Started \n");
		}


		/************************************************************************
		*                   	Throttle Controller                             *
		************************************************************************/

	//	float throttle_compensation = 1 / cos(control.euler[0]);
	//	throttle_compensation *= 1 / cos(control.euler[1]);		

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
			// //Using GPS Control
			// if(function_control.gps_pos_mode==0){
			// 	printf("gps position mode\n");
			// 	setpoint.lat_setpoint=GPS_data.pos_lat;
			// 	setpoint.lon_setpoint=GPS_data.pos_lon;
			// 	function_control.gps_pos_mode=1;
			// 	control.standing_throttle=control.throttle;
			// 	control.alt_ref=GPS_data.gps_altitude+1;
			// }
			// control.lat_error=setpoint.lat_setpoint-GPS_data.pos_lat;
			// control.lon_error=setpoint.lon_setpoint-GPS_data.pos_lon;
			
			// setpoint.pitch_ref=0.14*update_filter(filters.Outer_Loop_TF_pitch,control.lat_error);
			// setpoint.roll_ref=-0.14*update_filter(filters.Outer_Loop_TF_roll,control.lon_error);
			
			// setpoint.pitch_ref=saturateFilter(setpoint.pitch_ref,-0.2,0.2);
			// setpoint.roll_ref=saturateFilter(setpoint.roll_ref,-0.2,0.2);
			
			// //Convert to Drone Coordinate System from User Coordinate System
			// float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
			// float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
			// setpoint.roll_ref=P_R_MAG*cos(Theta_Ref-control.euler[2]);
			// setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref-control.euler[2]);
			
			// control.alt_error=control.alt_ref-GPS_data.gps_altitude;
			// //control.throttle=0.12*update_filter(&filters.Throttle_controller,control.alt_error);
			
			// //control.throttle=saturateFilter(control.throttle,-0.15,0.15)+control.standing_throttle;
		}
		
		control.dpitch_setpoint = update_filter(filters.pitch_PD, setpoint.pitch_ref - control.euler[1]);
		
		control.droll_setpoint = update_filter(filters.roll_PD, setpoint.roll_ref - control.euler[0]);
		
		//Apply the PD Controllers 
		control.upitch = update_filter(filters.pitch_rate_PD,control.dpitch_setpoint - control.euler_rate[1]);
		control.uroll = update_filter(filters.roll_rate_PD,control.droll_setpoint - control.euler_rate[1]);				
		
		
		/************************************************************************
		*                        	Yaw Controller                              *
		************************************************************************/	
	//	control.euler_rate[2]_f = update_filter(filters.LPF_euler_rate[2],control.euler_rate[2]);
		
		setpoint.yaw_ref[1]=setpoint.yaw_ref[0];
		setpoint.yaw_ref[0]=setpoint.yaw_ref[1]+(setpoint.yaw_rate_ref[0]+setpoint.yaw_rate_ref[1])*DT/2;

		control.uyaw = update_filter(filters.yaw_rate_PD,setpoint.yaw_ref[0]-control.euler[2]);
		
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
			setpoint.yaw_ref[0]=control.euler[2];
			control.droll_err_integrator=0; 
			control.dpitch_err_integrator=0;
			control.dyaw_err_integrator=0;
		}
			
		//only use integrators if airborne (above minimum throttle for > 1.5 seconds)
		if(function_control.integrator_start >  400){
			control.droll_err_integrator  += control.uroll  * DT;
			control.dpitch_err_integrator += control.upitch * DT;
			control.dyaw_err_integrator += control.uyaw * DT;		
			
			control.upitch+=control.flight_config.Dpitch_KI * control.dpitch_err_integrator;
			control.uroll +=  control.flight_config.Droll_KI * control.droll_err_integrator;
			control.uyaw+=control.flight_config.yaw_KI * control.dyaw_err_integrator;
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

		if(!control.flight_config.enable_debug_mode)
		{
			//Send Commands to Motors
			for(i=0;i<4;i++){
				pru_client_data.u[i] = control.u[i];
				pru_client_data.send_flag = 1;
			}

			if(control.kill_switch[0] < .5) {
				printf("\nKill Switch Hit! Shutting Down\n");
				fprintf(control.logger.Error_logger,"\nKill Switch Hit! Shutting Down\n");	
				rc_set_state(EXITING);
			}
		}
		



		//Print some stuff if in debug mode
		if(control.flight_config.enable_debug_mode)
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
			// printf(" Pitch_ref %2.2f ", setpoint.pitch_ref);
			// printf(" Roll_ref %2.2f ", setpoint.roll_ref);
			// printf(" Yaw_ref %2.2f ", setpoint.yaw_ref[0]);
			printf(" Roll %1.2f ", control.euler[0]);
			printf(" Pitch %1.2f ", control.euler[1]);
			printf(" Yaw %2.3f ", control.euler[2]); 
		//	printf(" Mag X %4.2f",control.mag[0]);
		//	printf(" Mag Y %4.2f",control.mag[1]);
		//	printf(" Mag Z %4.2f",control.mag[2]);
		// 	printf(" Pos N %2.3f ", control.ekf_filter.output.ned_pos[0]); 
		//	printf(" Pos E %2.3f ", control.ekf_filter.output.ned_pos[1]); 
		//	printf(" Pos D %2.3f ", control.ekf_filter.output.ned_pos[2]); 
		//	printf(" DPitch %1.2f ", control.euler_rate[1]); 
		//	printf(" DRoll %1.2f ", control.euler_rate[1]);
		//	printf(" DYaw %2.3f ", control.euler_rate[2]); 	
		//	printf(" uyaw %2.3f ", control.upitch); 		
		//	printf(" uyaw %2.3f ", control.uroll); 		
		//	printf(" uyaw %2.3f ", control.uyaw);
		//	printf(" GPS pos lat: %2.2f", GPS_data.pos_lat);
		//	printf(" GPS pos lon: %2.2f", GPS_data.pos_lon);
		//	printf(" HDOP: %f", GPS_data.HDOP);
		//	printf(" Pos_Lat %2.3f ", X_state_Lat1->data[0]);	
		//	printf(" Pos_Lon %2.3f ", X_state_Lon1->data[0]);
		//	printf("control: %d",rc_get_state());
		//	printf("Baro Alt: %f ",control.baro_alt);
			fflush(stdout);
		}

			/***************** Get GPS Data if available *******************/
		if(is_new_GPS_data())
		{

			 //printf("\n new GPS data in \n");
			fprintf(control.logger.GPS_logger,"%4.5f,",control.time);
			fprintf(control.logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_longitude,GPS_data.min_longitude);
			fprintf(control.logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_latitude,GPS_data.min_latitude);
			fprintf(control.logger.GPS_logger,"%f,%f,",GPS_data.speed,GPS_data.direction);
			fprintf(control.logger.GPS_logger,"%f,",GPS_data.gps_altitude);
			fprintf(control.logger.GPS_logger,"%2.2f,%d",GPS_data.HDOP,GPS_data.GPS_fix);
			fprintf(control.logger.GPS_logger,"\n");
			fflush(control.logger.GPS_logger);
			//	printf("%s",GPS_data.GGAbuf);
			//	printf("%s",GPS_data.VTGbuf);

			if (First_Iteration_GPS==1  && GPS_data.GPS_fix==1){
				// control.initial_pos_lat=GPS_data.meters_lat;
				// control.initial_pos_lon=GPS_data.meters_lon;
				First_Iteration_GPS=0;
				GPS_data.GPS_fix_check=1;
				printf("First Iteration GPS\n");
			}
			if(GPS_data.HDOP<4 && GPS_data.GPS_fix==1){
				// GPS_data.pos_lat=GPS_data.meters_lat-control.initial_pos_lat;
				// GPS_data.pos_lon=GPS_data.meters_lon-control.initial_pos_lon;
			}

			control.ekf_filter.input.gps_updated = 1;
			control.ekf_filter.input.gps_timestamp = control.time*1E6;
			control.ekf_filter.input.gps_latlon[0] = (double)GPS_data.deg_latitude + (double)GPS_data.min_latitude / 60.0;// + control.time*1E7/20000;
			control.ekf_filter.input.gps_latlon[1] = (double)GPS_data.deg_longitude + (double)GPS_data.min_longitude / 60.0;
			control.ekf_filter.input.gps_latlon[2] = (double)GPS_data.gps_altitude;
			control.ekf_filter.input.gps_fix = GPS_data.GPS_fix;
			control.ekf_filter.input.nsats = 10; // Really need to fix this
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
	if(load_core_config(&control.flight_config)){
		printf("WARNING: no configuration file found\n");
		printf("loading default settings\n");
		if(create_default_core_config_file(&control.flight_config)){
			printf("Warning, can't write default flight_config file\n");
		}
	}
	
	int in;
	while ((in = getopt(argc, argv, "d")) != -1)
	{
		switch (in)
		{
			case 'd': 
				control.flight_config.enable_debug_mode = 1;
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
	if(initialize_flight_program(	&control,
									&flyMS_threads,
									&filters,
									&pru_client_data,
									&GPS_data))
	{
		flyMS_shutdown( &control.logger, 
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
			fprintf(control.logger.Error_logger,"Error! IMU read failed for more than\
											5 consecutive timesteps. time: = %f\
											number of missed reads: %u \n",
											control.time,imu_err_count);
		}
	}

	flyMS_shutdown( &control.logger, 
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
