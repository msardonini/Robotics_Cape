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

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>

#include <flyMS_common.h>
#include <flyMS.h>
#include <imu_handler.h>



//Local functions
static void* flight_core(void * ptr);


/************************************************************************
* 	Local Variables				
************************************************************************/
control_variables_t		flyMSData;			//Structure to contain all system states
GPS_data_t				GPS_data;			//Structure to store data from GPS
function_control_t 		function_control;	//Structure to store variables which control functions
filters_t				filters;			//Struct to contain all the filters
pru_client_data_t		pru_client_data;
uint16_t 				imu_err_count;

static int check_output_range(float u[4])
{
	int i;
	float largest_value = 1;
	float smallest_value = 0;

	for(i=0;i<4;i++){ 
		if(u[i]>largest_value)largest_value=flyMSData.control.u[i];
		
		if(u[i]<smallest_value)flyMSData.control.u[i]=0;
	}
			
	// if upper saturation would have occurred, reduce all outputs evenly
	if(largest_value>1){
		float offset = largest_value - 1;
		for(i=0;i<4;i++) u[i]-=offset;
	}
	return 0;
}


void* flight_core(void* ptr)
{
	imu_err_count = 0;
	//control_variables_t *STATE = (control_variables_t*)ptr;
	//printf("pointer value %f\n", STATE->pitch);
	//Keep an i for loops
	static uint8_t i=0, first_iteration_count=0;

	//Variables to Initialize things upon startup
	static uint8_t First_Iteration=1;
	
	// float initial_alt = 0;
	//Initialize some variables if it is the first iteration

	
	//Set the reference time to the first iteration
	function_control.start_time_usec = get_usec_timespec(&function_control.start_time);

	flyMSData.setpoint.Aux[0] = 1; flyMSData.setpoint.kill_switch[0]=1;
	function_control.dsm2_timeout=0;
	rc_read_barometer();
	// initial_alt = rc_bmp_get_altitude_m();
	rc_set_state(RUNNING);

	while(rc_get_state()!=EXITING)
	{

		/******************************************************************
		*			Grab the time for Periphal Apps and Logs 			  *
		******************************************************************/
		function_control.start_loop_usec = get_usec_timespec(&function_control.start_loop);
		flyMSData.time_us = function_control.start_loop_usec - function_control.start_time_usec;
		
		/******************************************************************
		*			Read, Parse, and Translate IMU data for Flight		  *
		******************************************************************/
		imu_handler(&flyMSData, &filters);
		imu_err_count = 0;

		/******************************************************************
		*				Take Care of Some Initialization Tasks			  *
		******************************************************************/
		if (first_iteration_count < 150)
		{
			first_iteration_count++;
			continue;
		}

		if(First_Iteration){
			flyMSData.setpoint.euler_ref[2]=flyMSData.state.euler[2];
			First_Iteration=0;
			flyMSData.setpoint.yaw_ref_offset = flyMSData.state.euler[2];
			printf("First Iteration Started \n");
		}

		/************************************************************************
		*                   	Throttle Controller                             *
		************************************************************************/

		//	float throttle_compensation = 1 / cos(flyMSData.state.euler[0]);
		//	throttle_compensation *= 1 / cos(flyMSData.state.euler[1]);		

		if(function_control.altitudeHold)
		{
			if(function_control.altitudeHoldFirstIteration)
			{
				flyMSData.control.standing_throttle = flyMSData.setpoint.throttle;
				flyMSData.setpoint.altitudeSetpoint = flyMSData.imu.baro_alt;
				function_control.altitudeHoldFirstIteration = 0;
			}
	       // flyMSData.setpoint.altitudeSetpoint=flyMSData.setpoint.altitudeSetpoint+(flyMSData.setpoint.altitudeSetpointRate)*DT;

			//flyMSData.control.uthrottle = update_filter(filters.altitudeHoldPID, flyMSData.setpoint.altitudeSetpoint - flyMSData.baro_alt);
			//flyMSData.setpoint.throttle = flyMSData.control.uthrottle + flyMSData.standing_throttle;
		}
			
		/************************************************************************
		* 	                  		Pitch Controller	                        *
		************************************************************************/
		flyMSData.setpoint.dpitch_setpoint = update_filter(filters.pitch_PD, flyMSData.setpoint.euler_ref[0] - flyMSData.state.euler[0]);
		flyMSData.control.u_euler[0] = update_filter(filters.pitch_rate_PD,flyMSData.setpoint.dpitch_setpoint - flyMSData.state.euler_rate[0]);
		flyMSData.control.u_euler[0] = saturateFilter(flyMSData.control.u_euler[0],-MAX_PITCH_COMPONENT,MAX_PITCH_COMPONENT);
		/************************************************************************
		* 	                  		Roll Controller		                        *
		************************************************************************/
		flyMSData.setpoint.droll_setpoint = update_filter(filters.roll_PD, flyMSData.setpoint.euler_ref[1] - flyMSData.state.euler[1]);
		flyMSData.control.u_euler[1] = update_filter(filters.roll_rate_PD,flyMSData.setpoint.droll_setpoint - flyMSData.state.euler_rate[1]);				
		flyMSData.control.u_euler[1] = saturateFilter(flyMSData.control.u_euler[1],-MAX_ROLL_COMPONENT,MAX_ROLL_COMPONENT);
		

		
		/************************************************************************
		*                        	Yaw Controller                              *
		************************************************************************/	
		// flyMSData.control.u_euler[2] = update_filter(filters.yaw_rate_PD,flyMSData.setpoint.euler_ref[2]-flyMSData.state.euler[2]);
		flyMSData.control.u_euler[2] = update_filter(filters.yaw_rate_PD,flyMSData.setpoint.yaw_rate_ref[0]-flyMSData.state.euler_rate[2]);
		
		/************************************************************************
		*                   	Apply the Integrators                           *
		************************************************************************/	
			
		if(flyMSData.setpoint.throttle<MIN_THROTTLE+.01){	
			function_control.integrator_reset++;
			function_control.integrator_start=0;
		}else{
			function_control.integrator_reset=0;
			function_control.integrator_start++;
		}
		
		if(function_control.integrator_reset==300){// if landed, reset integrators and Yaw error
			flyMSData.setpoint.euler_ref[2]=flyMSData.state.euler[2];
			flyMSData.control.droll_err_integrator=0; 
			flyMSData.control.dpitch_err_integrator=0;
			flyMSData.control.dyaw_err_integrator=0;
		}
			
		//only use integrators if airborne (above minimum throttle for > 1.5 seconds)
		if(function_control.integrator_start >  400){
			flyMSData.control.dpitch_err_integrator += flyMSData.control.u_euler[0] * DT;
			flyMSData.control.droll_err_integrator  += flyMSData.control.u_euler[1]  * DT;
			flyMSData.control.dyaw_err_integrator += flyMSData.control.u_euler[2] * DT;		
			
			flyMSData.control.u_euler[0] += flyMSData.flight_config.Dpitch_KI * flyMSData.control.dpitch_err_integrator;
			flyMSData.control.u_euler[1] += flyMSData.flight_config.Droll_KI * flyMSData.control.droll_err_integrator;
			flyMSData.control.u_euler[2] += flyMSData.flight_config.yaw_KI * flyMSData.control.dyaw_err_integrator;
		}
		
		//Apply a saturation filter
		flyMSData.control.u_euler[2] = saturateFilter(flyMSData.control.u_euler[2],-MAX_YAW_COMPONENT,MAX_YAW_COMPONENT);
		
		/************************************************************************
		*  Mixing
		*           	      black				yellow
		*                          CCW 1	  2 CW		IMU Orientation:	
		*                          	   \ /				Y
		*	                           / \            	|_ X
		*                         CW 3	  4 CCW
		*                 	  yellow       	    black
		************************************************************************/
		
		flyMSData.control.u[0]=flyMSData.setpoint.throttle+flyMSData.control.u_euler[1]+flyMSData.control.u_euler[0]-flyMSData.control.u_euler[2];
		flyMSData.control.u[1]=flyMSData.setpoint.throttle-flyMSData.control.u_euler[1]+flyMSData.control.u_euler[0]+flyMSData.control.u_euler[2];
		flyMSData.control.u[2]=flyMSData.setpoint.throttle+flyMSData.control.u_euler[1]-flyMSData.control.u_euler[0]+flyMSData.control.u_euler[2];
		flyMSData.control.u[3]=flyMSData.setpoint.throttle-flyMSData.control.u_euler[1]-flyMSData.control.u_euler[0]-flyMSData.control.u_euler[2];		

		/************************************************************************
		*         		Check Output Ranges, if outside, adjust                 *
		************************************************************************/
		check_output_range(flyMSData.control.u);
		
		/************************************************************************
		*         				 Send Commands to ESCs 		                    *
		************************************************************************/
		if(!flyMSData.flight_config.enable_debug_mode)
		{
			//Send Commands to Motors
			for(i=0;i<4;i++){
				pru_client_data.u[i] = flyMSData.control.u[i];
				pru_client_data.send_flag = 1;
			}
		}

		/************************************************************************
		*         		Check the kill Switch and Shutdown if set               *
		************************************************************************/
		if(flyMSData.setpoint.kill_switch[0] < .5) {
			char errMsg[50];
			sprintf(errMsg, "\nKill Switch Hit! Shutting Down\n");
			printf("%s",errMsg);
			flyMS_Error_Log(errMsg);
			rc_set_state(EXITING);
		}

		//Print some stuff to the console in debug mode
		if(flyMSData.flight_config.enable_debug_mode)
		{	
			flyMS_console_print(&flyMSData);
		}
		/************************************************************************
		*         		Check for GPS Data and Handle Accordingly               *
		************************************************************************/
		GPS_handler(&flyMSData, &GPS_data);
		
		/************************************************************************
		*         			Log Important Flight Data For Analysis              *
		************************************************************************/
		log_data(&flyMSData);

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

	// // load flight_core settings
	// if(load_core_config(&flyMSData.flight_config)){
	// 	printf("WARNING: no configuration file found\n");
	// 	printf("loading default settings\n");
	// 	if(create_default_core_config_file(&flyMSData.flight_config)){
	// 		printf("Warning, can't write default flight_config file\n");
	// 	}
	// }
	
	int in;
	while ((in = getopt(argc, argv, "d")) != -1)
	{
		switch (in)
		{
			case 'd': 
				flyMSData.flight_config.enable_debug_mode = 1;
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
	if(initialize_flight_program(	&flyMSData,
									&filters,
									&pru_client_data,
									&GPS_data))
	{
		flyMS_shutdown(	&GPS_data); 
		rc_cleanup();
		return -1;
	}
	
	//Start the control program
	pthread_t flight_core_thread;
	pthread_create(&flight_core_thread, NULL, flight_core, (void*)NULL);

	printf("Starting \n");
	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) 
	{
		usleep(DT_US);
		imu_err_count++;
		if (imu_err_count == 5 || imu_err_count % 50 == 0)
		{
			char errMsg[100];
			sprintf(errMsg,"Error! IMU read failed for more than\
								5 consecutive timesteps. time: = %llu\
								number of missed reads: %u \n",
								flyMSData.time_us,imu_err_count);
			flyMS_Error_Log(errMsg);
		}
	}

	flyMS_shutdown(	&GPS_data); 
	rc_cleanup();
	return 0;
}


#ifdef __cplusplus
}
#endif
