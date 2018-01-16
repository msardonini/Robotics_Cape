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


// setpoint.c Code to manage the reference values for the controller
// By Michael Sardonini

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <time.h>

#include <flyMS_common.h>
#include <setpoint.h>
#include <flyMS.h>

uint8_t new_dsm_data = 0;
float dsm2_data[MAX_DSM2_CHANNELS];
int dsm2_timeout = 0;
pthread_t setpoint_thread;


//Local Functions
static int copy_dsm2_data();
static int handle_rc_data_direct(control_variables_t *control);
static int rc_err_handler(reference_mode_t setpoint_type);
static void * setpoint_manager(void* ptr);



/*
	Init and Deinit Functions
	Called elsewhere to start and stop the setpoint manager
*/

int initialize_setpoint_manager(control_variables_t *control)
{
	pthread_create(&setpoint_thread, NULL, setpoint_manager, (void*)control);
	return 0;
}

int shutdown_setpoint_manager()
{
	pthread_join(setpoint_thread, NULL);
	return 0;
}


/*
	setpoint_manager()
		Handles the setpoint values for roll/pitch/yaw to be fed into the flight controller
		
		2 Main sources of retreiving values
			1. Direct from remote control
			2. Calculated values from GPS navigation for autonomous flight
*/

static void * setpoint_manager(void* ptr)
{
	control_variables_t *control = (control_variables_t*) ptr;
	enum reference_mode_t setpoint_type = RC_DIRECT;

	while (rc_get_state()!= EXITING)
	{
		/**********************************************************
		*           If there is new dsm2 data read it in 		  *
		*			and make a local copy from the driver's data  *
		**********************************************************/
		if (rc_is_new_dsm_data())
		{
			copy_dsm2_data();
			new_dsm_data = 1;
			dsm2_timeout = 0;
		}
		else
		{
			if(!control->flight_config.enable_debug_mode)
			{
				//check to make sure too much time hasn't gone by since hearing the RC
				rc_err_handler(setpoint_type);
			}
			new_dsm_data = 0;
		}

		switch (setpoint_type)
		{
			case RC_DIRECT:
				handle_rc_data_direct(control);
			break;
			case RC_NAVIGATION:

			break;
			default:
				printf("Error, invalid reference mode! \n");
		}
		
		usleep(DT_US); //Run at the control frequency
	}
	return NULL;
}

	
static int handle_rc_data_direct(control_variables_t *control)
{		
		/**********************************************************
		*           Read the RC Controller for Commands           *
		**********************************************************/
		if(new_dsm_data)
		{
			//Set roll reference value
			//DSM2 Receiver is inherently positive to the left
			control->setpoint.euler_ref[1]= -dsm2_data[1]*MAX_ROLL_RANGE;	
			
			//DSM2 Receiver is inherently positive upwards
			control->setpoint.euler_ref[0]= -dsm2_data[2]*MAX_PITCH_RANGE;
			
			//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
			//Apply the integration outside of current if statement, needs to run at 200Hz
			control->setpoint.yaw_rate_ref[1]=control->setpoint.yaw_rate_ref[0];		
			control->setpoint.yaw_rate_ref[0]=dsm2_data[3]*MAX_YAW_RATE;

			//If Specified by the config file, convert from Drone Coordinate System to User Coordinate System
			if (control->flight_config.static_PR_ref)
			{				
				float P_R_MAG=pow(pow(control->setpoint.euler_ref[0],2)+pow(control->setpoint.euler_ref[1],2),0.5);
				float Theta_Ref=atan2f(control->setpoint.euler_ref[0],control->setpoint.euler_ref[1]);
				control->setpoint.euler_ref[1] =P_R_MAG*cos(Theta_Ref+control->euler[2]-control->yaw_ref_offset);
				control->setpoint.euler_ref[0]=P_R_MAG*sin(Theta_Ref+control->euler[2]-control->yaw_ref_offset);
			}
			
			//Apply a deadzone to keep integrator from wandering
			if(fabs(control->setpoint.yaw_rate_ref[0])<0.05) {
				control->setpoint.yaw_rate_ref[0]=0;
			}

			//Kill Switch
			control->kill_switch[0]=dsm2_data[4]/2;
			
			//Auxillary Switch
			control->setpoint.Aux[1] = control->setpoint.Aux[0];
			control->setpoint.Aux[0] = dsm2_data[5]; 
			
			//Set the throttle
			control->throttle=(dsm2_data[0]+1)* 0.5f *
					(control->flight_config.max_throttle-control->flight_config.min_throttle)+control->flight_config.min_throttle;
			//Keep the aircraft at a constant height while making manuevers 
			// control->throttle *= 1/(cos(control->euler[1])*cos(control->euler[0]));
		}

		//Finally Update the integrator on the yaw reference value
		control->setpoint.euler_ref[2]=control->setpoint.euler_ref[2] + 
										(control->setpoint.yaw_rate_ref[0]+control->setpoint.yaw_rate_ref[1])*DT/2;
	return 0;
}

static int copy_dsm2_data()
{
	int i;
	for (i = 0; i < MAX_DSM2_CHANNELS; i++)\
	{
		dsm2_data[i] = rc_get_dsm_ch_normalized(i+1);
	}
	return 0;
}

static int rc_err_handler(reference_mode_t setpoint_type)
{
	dsm2_timeout++;
	if(dsm2_timeout>1.5/DT) //If packet hasn't been received for 1.5 seconds
	{ 
		char errMsg[50];
		sprintf(errMsg,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
		printf("%s",errMsg);
		flyMS_Error_Log(errMsg);
		rc_set_state(EXITING);
	}
	return 0;
}

//Super old code for autonomount flight

// //Using GPS Control
// if(function_control.gps_pos_mode==0){
// 	printf("gps position mode\n");
// 	control.setpoint.lat_setpoint=GPS_data.pos_lat;
// 	control.setpoint.lon_setpoint=GPS_data.pos_lon;
// 	function_control.gps_pos_mode=1;
// 	control.standing_throttle=control.throttle;
// 	control.alt_ref=GPS_data.gps_altitude+1;
// }
// control.lat_error=control.setpoint.lat_setpoint-GPS_data.pos_lat;
// control.lon_error=control.setpoint.lon_setpoint-GPS_data.pos_lon;

// control.setpoint.euler_ref[1]=0.14*update_filter(filters.Outer_Loop_TF_pitch,control.lat_error);
// control.setpoint.euler_ref[0]=-0.14*update_filter(filters.Outer_Loop_TF_roll,control.lon_error);

// control.setpoint.euler_ref[1]=saturateFilter(control.setpoint.euler_ref[1],-0.2,0.2);
// control.setpoint.euler_ref[0]=saturateFilter(control.setpoint.euler_ref[0],-0.2,0.2);

// //Convert to Drone Coordinate System from User Coordinate System
// float P_R_MAG=pow(pow(control.setpoint.euler_ref[0],2)+pow(control.setpoint.euler_ref[1],2),0.5);
// float Theta_Ref=atan2f(control.setpoint.euler_ref[1],control.setpoint.euler_ref[0]);
// control.setpoint.euler_ref[0]=P_R_MAG*cos(Theta_Ref-control.euler[2]);
// control.setpoint.euler_ref[1]=P_R_MAG*sin(Theta_Ref-control.euler[2]);

// control.alt_error=control.alt_ref-GPS_data.gps_altitude;
// //control.throttle=0.12*update_filter(&filters.Throttle_controller,control.alt_error);

// //control.throttle=saturateFilter(control.throttle,-0.15,0.15)+control.standing_throttle;