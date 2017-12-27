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

#include <flyMS_common.h>
#include <flyMS.h>
#include <imu_handler.h>




void * setpoint_manager(void* ptr)
{
	control_variables_t *control = (control_variables_t*) ptr;
	uint32_t dsm2_timeout = 0;

	while(rc_get_state()!=EXITING)
	{
		/**********************************************************
		*           Read the RC Controller for Commands           *
		**********************************************************/
	
		if(rc_is_new_dsm_data()){
			//Reset the timout counter back to zero
			dsm2_timeout=0;
			
			//Set roll reference value
			//DSM2 Receiver is inherently positive to the left
			control->setpoint.euler_ref[0]= -rc_get_dsm_ch_normalized(2)*MAX_ROLL_RANGE;	
			
			//Set the pitch reference value
			control->setpoint.euler_ref[1]= -rc_get_dsm_ch_normalized(3)*MAX_PITCH_RANGE;
			
			
			//If Specified by the config file, convert from Drone Coordinate System to User Coordinate System
			if (control->flight_config.static_PR_ref)
			{				
				//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
				//Apply the integration outside of current if statement, needs to run at 200Hz
				control->setpoint.yaw_rate_ref[1]=control->setpoint.yaw_rate_ref[0];		
				control->setpoint.yaw_rate_ref[0]=rc_get_dsm_ch_normalized(4)*MAX_YAW_RATE;

				float P_R_MAG=pow(pow(control->setpoint.euler_ref[0],2)+pow(control->setpoint.euler_ref[1],2),0.5);
				float Theta_Ref=atan2f(control->setpoint.euler_ref[1],control->setpoint.euler_ref[0]);
				control->setpoint.euler_ref[0] =P_R_MAG*cos(Theta_Ref+control->euler[2]-control->yaw_ref_offset);
				control->setpoint.euler_ref[1]=P_R_MAG*sin(Theta_Ref+control->euler[2]-control->yaw_ref_offset);
			}
			else	//This is flying FPV mode
			{
				//Set roll reference value
				//DSM2 Receiver is inherently positive to the left
				control->setpoint.euler_ref[0]= -rc_get_dsm_ch_normalized(2)*MAX_ROLL_RANGE;	

				//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
				//Apply the integration outside of current if statement, needs to run at 200Hz
				control->setpoint.yaw_rate_ref[1]=control->setpoint.yaw_rate_ref[0];		
				control->setpoint.yaw_rate_ref[0]=rc_get_dsm_ch_normalized(4)*MAX_YAW_RATE;

			}
						
			//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
			//Apply the integration outside of current if statement, needs to run at 200Hz
			control->setpoint.yaw_rate_ref[1]=control->setpoint.yaw_rate_ref[0];		
			control->setpoint.yaw_rate_ref[0]=rc_get_dsm_ch_normalized(4)*MAX_YAW_RATE;
			
			//Apply a deadzone to keep integrator from wandering
			if(fabs(control->setpoint.yaw_rate_ref[0])<0.05) {
				control->setpoint.yaw_rate_ref[0]=0;
			}
			//Kill Switch
			control->kill_switch[0]=rc_get_dsm_ch_normalized(5)/2;
			
			//Auxillary Switch
			control->setpoint.Aux[1] = control->setpoint.Aux[0];
			control->setpoint.Aux[0] = rc_get_dsm_ch_normalized(6); 
			
			//Set the throttle
			control->throttle=(rc_get_dsm_ch_normalized(1)+1)* 0.5f *
					(control->flight_config.max_throttle-control->flight_config.min_throttle)+control->flight_config.min_throttle;
			//Keep the aircraft at a constant height while making manuevers 
			control->throttle *= 1/(cos(control->euler[1])*cos(control->euler[0]));
			
			//Future work, set variables for autonomous flight
			if(control->setpoint.Aux[0] < 0 && control->flight_config.enable_autonomy)
			{ 
				control->setpoint.altitudeSetpointRate = rc_get_dsm_ch_normalized(1) * MAX_ALT_SPEED;
				//Apply a deadzone for altitude hold
				if(fabs(control->setpoint.altitudeSetpointRate - control->standing_throttle)<0.05)
				{
					control->setpoint.altitudeSetpointRate=0;
				}
				//Detect if this is the first iteration switching to controlled flight
				// if (control->setpoint.Aux[1] > 0) 
				// {
				// 	function_control->altitudeHoldFirstIteration = 1;
				// }
				// function_control->altitudeHold = 1;
			}
			// else  
			// {
			// 	function_control->altitudeHold = 0;
			// }
		}
		
		//check to make sure too much time hasn't gone by since hearing the RC
		else{
			if(!control->flight_config.enable_debug_mode)
			{
				dsm2_timeout++;
				if(dsm2_timeout>1.5/DT) { //If packet hasn't been received for 1.5 seconds
					char errMsg[50];
					sprintf(errMsg,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
					printf("%s",errMsg);
					flyMS_Error_Log(errMsg);
					rc_set_state(EXITING);
				}
			}
		}

		//Update the previous values of the references
		int i;

		//Finally Update the integrator on the yaw reference value
		control->setpoint.euler_ref[2]=control->setpoint.euler_ref[2] + 
										(control->setpoint.yaw_rate_ref[0]+control->setpoint.yaw_rate_ref[1])*DT/2;
		
		usleep(DT_US); //Run at 200Hz
	}	
	return NULL;
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