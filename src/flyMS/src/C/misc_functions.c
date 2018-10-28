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

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>
#include <inttypes.h>
	
#include "flyMS_common.h"
#include "flyMS.h"
#include "setpoint.h"
#include "logger.h"
#include "imu_handler.h"
//Coordinate system transformations matrices



//Local Variables and Functions
uint8_t logger_running = 0;
static int ready_check();


int initialize_flight_program(control_variables_t *control,
				filters_t *filters,
				pru_client_data_t *pru_client_data,
				GPS_data_t *GPS_data)
{
	//Starts the pru_client which will send commands to ESCs
	start_pru_client(pru_client_data);

	//Initialize the remote controller
	rc_initialize_dsm();

	//Pause the program until the user toggles the kill switch
	if(!control->flight_config.enable_debug_mode)
	{	
		if(ready_check()){
			printf("Exiting Program \n");
			return -1;
		}
	}

	// load flight_core settings
	int force_debug_mode = control->flight_config.enable_debug_mode;
	if(load_core_config(&control->flight_config)){
		printf("WARNING: no configuration file found\n");
		printf("loading default settings\n");
		if(create_default_core_config_file(&control->flight_config)){
			printf("Warning, can't write default flight_config file\n");
		}
	}
	control->flight_config.enable_debug_mode = (force_debug_mode || control->flight_config.enable_debug_mode);
	
	//Initialize the IMU and Barometer
	initialize_imu(control);

	//Enable the data logger
	if(control->flight_config.enable_logging)
	{
		logger_init();
	}

	//Enable the data logger
	initialize_setpoint_manager(control);

	
	/* --------- Start the EKF for position estimates ----*/
//	pthread_create(&flyMS_threads->ekf_thread, NULL, run_ekf, control->ekf_filter );

	initialize_filters(filters, &control->flight_config);

	/* 			Start the GPS 				*/
	if(control->flight_config.enable_gps)
	{
		GPS_data->GPS_init_check=GPS_init(GPS_data);
	}
	//Should be disabled by default but we don't want to be pumping 5V into our BEC ESC output
	rc_disable_servo_power_rail();
	return 0;
}

uint64_t get_usec_timespec(struct timespec *tv)
{
	clock_gettime(CLOCK_MONOTONIC, tv);
	return tv->tv_sec*(uint64_t)1E6 + tv->tv_nsec/(uint64_t)1E3;
}

static int ready_check(){
	//Toggle the kill switch to get going, to ensure controlled take-off
	//Keep kill switch down to remain operational
    int count=1, toggle = 0, reset_toggle = 0;
	float val[2] = {0.0f , 0.0f};
	printf("Toggle the kill swtich twice and leave up to initialize\n");
	while(count<6 && rc_get_state()!=EXITING)
	{
		
		//Blink the green LED light to signal that the program is ready
		reset_toggle++; // Only blink the led 1/100 the time this loop runs
		if(toggle)
		{
			rc_set_led(GREEN,OFF);
			if (reset_toggle == 20) 
			{
				toggle = 0;
				reset_toggle = 0;
			}
		}
		else
		{
			rc_set_led(GREEN,ON);
			if (reset_toggle == 20)
			{
				toggle=1;
				reset_toggle = 0;
			}
		}

		if(rc_is_new_dsm_data()){

			val[1]=val[0];
			val[0]=rc_get_dsm_ch_normalized(5);
			if(val[0] < -0.75 && val[1] > 0.35){
			count++;
			} 
			if(val[0] > 0.75 && val[1] < 0.35){
			count++;
			}
			
		}
		usleep(10000);
	}
	
	//make sure the kill switch is in the position to fly before starting
	while(val[0] < 0.5 && rc_get_state()!=EXITING)
		{
		if(rc_is_new_dsm_data()){
			val[0]=rc_get_dsm_ch_normalized(5);	
			}
		usleep(10000);
		}
	
	if(rc_get_state() == EXITING)
	{
		printf("State set to exiting, shutting off! \n");
		return -1;
	}
	
	printf("\nInitialized! Starting program\n");
	rc_set_led(GREEN,ON);
	return 0;
}


/************************************************************************
*	initialize_filters()
*	setup of feedback controllers used in flight core
************************************************************************/
int initialize_filters(filters_t *filters, core_config_t *flight_config){

	filters->pitch_PD = generatePID(flight_config->pitch_KP, flight_config->pitch_KI, flight_config->pitch_KD, 0.15, DT);
	filters->roll_PD  = generatePID(flight_config->roll_KP, flight_config->roll_KI, flight_config->roll_KD, 0.15, DT);
	//filters->yaw_PD   = generatePID(YAW_KP,		  0, YAW_KD,	    0.15, 0.005);

	//PD Controller (I is done manually)
	filters->pitch_rate_PD = generatePID(flight_config->Dpitch_KP, 0, flight_config->Dpitch_KD, 0.15, DT);
	filters->roll_rate_PD  = generatePID(flight_config->Droll_KP, 0, flight_config->Droll_KD, 0.15, DT);
	filters->yaw_rate_PD   = generatePID(flight_config->yaw_KP,		  0, flight_config->yaw_KD,	    0.15, DT);
	
	//Gains on Low Pass Filter for raw gyroscope output
	
	filters->altitudeHoldPID  = generatePID(.05,		  .005,  .002,	    0.15, DT);

	//elliptic filter 10th order 0.25 dB passband ripple 80 dB min Cutoff 0.4 cutoff frq
	float num[11] = {   0.003316345545497,   0.006003204398448,   0.015890122416480,   0.022341342884745,   0.031426841006402,
						0.032682319166147,   0.031426841006402,  0.022341342884745,   0.015890122416480,   0.006003204398448,
						0.003316345545497};

	float den[11] = {   1.000000000000000,  -4.302142513532524,  10.963685193359051, -18.990960386921738,  24.544342262847074,
						-24.210021253402012,  18.411553079753368, -10.622846105856944,   4.472385466696109,  -1.251943621469692,
						0.182152641224648};
	
	//elliptic filter 10th order 0.25 dB passband ripple 80 dB min Cutoff 0.05 cutoff frq

	float yaw_num[11] = {0.000000138467082,0.000001384670818,0.000006231018679,0.000016616049812,0.000029078087171,0.000034893704605,0.000029078087171,0.000016616049812,0.000006231018679,0.000001384670818,0.000000138467082};
	float yaw_den[11] = {1.000000000000000,-6.989417698566569,22.323086726703938,-42.824608705880635,54.570406893265300,-48.208486634295596,29.872790631313180,-12.810698156370698,3.636160614880030,-0.616474419461443,0.047382538704228};

	int i;
	for (i = 0; i < 2; i++)
	{
		filters->gyro_lpf[i] = initialize_filter(10, num, den);		
		filters->accel_lpf[i] = initialize_filter(10, num, den);	
	}

	filters->gyro_lpf[2] = initialize_filter(10, yaw_num, yaw_den);		
	filters->accel_lpf[2] = initialize_filter(10, num, den);	
	
	//Gains on Low Pass Filter for Yaw Reference		
	float num2[4] = {  0.0317,    0.0951,    0.0951,    0.0317};
	float den2[4] = { 1.0000,   -1.4590,    0.9104,   -0.1978};					
	filters->LPF_Yaw_Ref_P = initialize_filter(3, num2, den2);							
	filters->LPF_Yaw_Ref_R = initialize_filter(3, num2, den2);		

	//ellip filter, 5th order .5 pass 70 stop .05 cutoff
	float baro_num[6] = {0.000618553374672,  -0.001685890697737,   0.001077182625629,   0.001077182625629,  -0.001685890697737,   0.000618553374672};
	float baro_den[6] =	{1.000000000000000,  -4.785739467762915,   9.195509273069447,  -8.866262182166356,   4.289470039368545,  -0.832957971903594};
	filters->LPF_baro_alt = initialize_filter(5, baro_num, baro_den);
	for (i = 0; i< 3; i++)
	{
		zeroFilter(filters->gyro_lpf[i]);
		zeroFilter(filters->accel_lpf[i]);
	}

	return 0;
}

int flyMS_shutdown(	GPS_data_t *GPS_data) 
{
	//Join the threads for a safe process shutdown
	if(GPS_data->GPS_init_check == 0)
	{
		join_GPS_thread(GPS_data);
		printf("GPS thread joined\n");
	}
	
	logger_deinit();

	close(GPS_data->GPS_file);
	join_pru_client();
	
	shutdown_setpoint_manager();

	rc_set_led(GREEN,OFF);
	rc_set_led(RED,OFF);
	return 0;
}

int flyMS_console_print(control_variables_t *control)
{
	printf("\r ");
//	printf("time %3.3f ", control->time);
//	printf("Alt_ref %3.1f ",control->alt_ref);
//	printf(" U1:  %2.2f ",control->u[0]);
//	printf(" U2: %2.2f ",control->u[1]);
//	printf(" U3:  %2.2f ",control->u[2]);
//	printf(" U4: %2.2f ",control->u[3]);	
//	printf(" Throt %2.2f ", control->throttle);
//	printf("Aux %2.1f ", control->setpoint.Aux[0]);
//	printf("function: %f",rc_get_dsm_ch_normalized(6));
//	printf("num wraps %d ",control->num_wraps);
	// printf(" Pitch_ref %2.2f ", control->setpoint.pitch_ref);
	// printf(" Roll_ref %2.2f ", control->setpoint.roll_ref);
	// printf(" Yaw_ref %2.2f ", control->setpoint.yaw_ref[0]);
	printf(" Pitch %1.2f ", control->state.euler[0]);
	printf(" Roll %1.2f ", control->state.euler[1]);
	printf(" Yaw %2.3f ", control->state.euler[2]); 
//	printf(" Mag X %4.2f",control->mag[0]);
//	printf(" Mag Y %4.2f",control->mag[1]);
//	printf(" Mag Z %4.2f",control->mag[2]);
	// printf(" Accel X %4.2f",control->accel[0]);
	// printf(" Accel Y %4.2f",control->accel[1]);
	// printf(" Accel Z %4.2f",control->accel[2]);
// 	printf(" Pos N %2.3f ", control->ekf_filter.output.ned_pos[0]); 
//	printf(" Pos E %2.3f ", control->ekf_filter.output.ned_pos[1]); 
//	printf(" Pos D %2.3f ", control->ekf_filter.output.ned_pos[2]); 
	// printf(" DPitch %1.2f ", control->euler_rate[0]); 
	// printf(" DRoll %1.2f ", control->euler_rate[1]);
	// printf(" DYaw %2.3f ", control->euler_rate[2]); 	
//	printf(" uyaw %2.3f ", control->upitch); 		
//	printf(" uyaw %2.3f ", control->uroll); 		
//	printf(" uyaw %2.3f ", control->uyaw);
//	printf(" GPS pos lat: %2.2f", control->GPS_data.pos_lat);
//	printf(" GPS pos lon: %2.2f", control->GPS_data.pos_lon);
//	printf(" HDOP: %f", control->GPS_data.HDOP);
//	printf("Baro Alt: %f ",control->baro_alt);
	fflush(stdout);
	return 0;
}


#ifdef __cplusplus
}
#endif
