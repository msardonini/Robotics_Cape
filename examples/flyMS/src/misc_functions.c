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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include "flyMS.h"
#include "logger.h"
#include <pthread.h>
#include "gps.h"
#include <inttypes.h>
#include "../../../libraries/pru_handler_client.h"
//Coordinate system transformations matrices

int initialize_flight_program(flyMS_threads_t *flyMS_threads,
				core_config_t *flight_config,
				logger_t *logger,
				filters_t *filters,
				pru_client_data_t *pru_client_data,
				rc_imu_data_t *imu_data,
				transform_matrix_t *transform,
				GPS_data_t *GPS_data)
{

	
	// flight_config->enable_debug_mode = (debug_mode || flight_config->enable_debug_mode);
	start_pru_client(pru_client_data);

	if(flight_config->enable_barometer)
	{
		if(rc_initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
			printf("initialize_barometer failed\n");
			return -1;
		}
	}
	
	if(flight_config->enable_logging)
	{
		// start a core_log and logging thread
		if(start_core_log(logger)<0){
			printf("WARNING: failed to open a core_log file\n");
		}
		else{
			pthread_create(&flyMS_threads->core_logging_thread, NULL, core_log_writer, &logger->core_logger);
		}
	}
	
	// set up IMU configuration
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE;
	imu_config.orientation = get_orientation_config(flight_config->imu_orientation);
	imu_config.accel_fsr = A_FSR_2G;
	imu_config.enable_magnetometer=1;

	// start imu
	if(rc_initialize_imu_dmp(imu_data, imu_config, (void*)NULL)){
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		rc_blink_led(RED, 5, 5);
		return -1;
	}
	
	//Initialize the remote controller
	rc_initialize_dsm();
	
	if(!flight_config->enable_debug_mode)
	{	
		if(ready_check()){
			printf("Exiting Program \n");
			return -1;
		} //Toggle the kill switch a few times to signal it's ready
	}

	// load flight_core settings
	if(load_core_config(flight_config)){
		printf("WARNING: no configuration file found\n");
		printf("loading default settings\n");
		if(create_default_core_config_file(flight_config)){
			printf("Warning, can't write default flight_config file\n");
		}
	}

	pthread_create(&flyMS_threads->setpoint_manager_thread, NULL, setpoint_manager, (void*)NULL );

	init_rotation_matrix(transform, flight_config); //Initialize the rotation matrix from IMU to drone
	initialize_filters(filters, flight_config);

	//Start the GPS thread, flash the LED's if GPS has a fix
	if(flight_config->enable_gps)
	{
		GPS_data->GPS_init_check=GPS_init(GPS_data);
		
		led_thread_t GPS_ready;
		memset(&GPS_ready,0,sizeof(GPS_ready));
		GPS_ready.GPS_fix_check = GPS_data->GPS_fix_check;
		GPS_ready.GPS_init_check = GPS_data->GPS_init_check;
		//pthread_create(&flyMS_threads->led_thread, NULL, LED_thread, (void*) &GPS_ready);
		
		//Spawn the Kalman Filter Thread if GPS is running
		if (GPS_data->GPS_init_check == 0)
		{
			pthread_create(&flyMS_threads->kalman_thread, NULL , kalman_filter, (void*) NULL);
		}
	}
	//Should be disabled by default but we don't want to be pumping 5V into our BEC ESC output
	rc_disable_servo_power_rail();
	sleep(2); //wait for the IMU to level off
	return 0;
}

int ready_check(){
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
			usleep(100000);
			if(val[0] < -0.75 && val[1] > 0.35){
			count++;
			} 
			if(val[0] > 0.75 && val[1] < 0.35){
			count++;
			}
			usleep(10000);
			
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

void* LED_thread(void *ptr){
	
	led_thread_t *GPS_ready= (led_thread_t*)ptr;
	
	const char *filepath0 = "/sys/class/leds/beaglebone:green:usr0/brightness";
	const char *filepath1 = "/sys/class/leds/beaglebone:green:usr1/brightness";
	const char *filepath2 = "/sys/class/leds/beaglebone:green:usr2/brightness";
	const char *filepath3 = "/sys/class/leds/beaglebone:green:usr3/brightness";
	int i=0;
	FILE *file0 = NULL;
	FILE *file1 = NULL;
	FILE *file2 = NULL;
	FILE *file3 = NULL;
	
	for(i=0;i<50;i++){
		if(rc_get_state() == EXITING) pthread_exit(NULL);
		
		if((file0 = fopen(filepath0, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file0);
			fclose(file0);
		}
		
		if((file1 = fopen(filepath1, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file1);
			fclose(file1);
		}
		
		if((file2 = fopen(filepath2, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file2);
			fclose(file2);
		}
		
		if((file3 = fopen(filepath3, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file3);
			fclose(file3);
		}
		
		usleep(500000-400000*GPS_ready->GPS_fix_check);
		if(rc_get_state() == EXITING) pthread_exit(NULL);
		
		if(GPS_ready->GPS_init_check==-1){
			if((file0 = fopen(filepath0, "r+")) != NULL){
				fwrite("0", sizeof(char), 1, file0);
				fclose(file0);
			}
			
			if((file1 = fopen(filepath1, "r+")) != NULL){
				fwrite("0", sizeof(char), 1, file1);
				fclose(file1);
			}			
		
			if((file2 = fopen(filepath2, "r+")) != NULL){
			fwrite("0", sizeof(char), 1, file2);
			fclose(file2);
			}
		
			if((file3 = fopen(filepath3, "r+")) != NULL){
				fwrite("0", sizeof(char), 1, file3);
				fclose(file3);
			}
		}
		usleep(500000-400000*GPS_ready->GPS_fix_check);
	}
	return NULL;
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

	filters->LPF_d_pitch = initialize_filter(10, num, den);		
	filters->LPF_d_roll= initialize_filter(10, num, den);	
	filters->LPF_d_yaw = initialize_filter(10, num, den);
	
	//ellip filter, 5th order .5 pass 70 stop .2 cutoff
	float num3[6] = {0.002284248527015,   0.001560308456655,   0.003463457796419,   0.003463457796419,   0.001560308456655,   0.002284248527015};
	float den3[6] =	{1.000000000000000,  -3.815166618410549,   6.254410671592536,  -5.434989467207256,   2.491599942967181,  -0.481238499381735};
//	float num3[6] =	{0.0045,    0.0006,    0.0052,    0.0052,    0.0006,    0.0045};
//	float den3[6] =	{1.0000,   -3.6733,    5.8400,   -4.9357,    2.2049,   -0.4153};

	filters->LPF_Accel_Lat = initialize_filter(5, num3, den3);							
	filters->LPF_Accel_Lon = initialize_filter(5, num3, den3);		

	//ellip filter, 5th order .5 pass 70 stop .05 cutoff
	float baro_num[6] = {0.000618553374672,  -0.001685890697737,   0.001077182625629,   0.001077182625629,  -0.001685890697737,   0.000618553374672};
	float baro_den[6] =	{1.000000000000000,  -4.785739467762915,   9.195509273069447,  -8.866262182166356,   4.289470039368545,  -0.832957971903594};
	filters->LPF_baro_alt = initialize_filter(5, baro_num, baro_den);	
	
	//Gains on Low Pass Filter for Yaw Reference		
	float num2[4] = {  0.0317,    0.0951,    0.0951,    0.0317};
	float den2[4] = { 1.0000,   -1.4590,    0.9104,   -0.1978};					
	filters->LPF_Yaw_Ref_P = initialize_filter(3, num2, den2);							
	filters->LPF_Yaw_Ref_R = initialize_filter(3, num2, den2);	
	
/*
	float num3[3] = {  0.0055 ,   0.0111 ,   0.0055};
	float den3[3] = {   1.0000 ,  -1.7786  ,  0.8008};					
	LPF_Height_Damping = initialize_filter(2, DT, num3, den3);				
*/

//	float num4[2] = { 2/(2+DT*DECAY_CONST),  -2/(2+DT*DECAY_CONST)+0.001};
//	float den4[2] = {   1.0000 , (DT*DECAY_CONST-2)/(DT*DECAY_CONST+2)};	
//	filters->Outer_Loop_TF_pitch = initialize_filter(1, DT, num4, den4);		
//	filters->Outer_Loop_TF_roll = initialize_filter(1, DT, num4, den4);
	//Throttle_controller = initialize_filter(1, DT, num4, den4);

	//4th order ellip .1 dp PB 60 dB SB 0.2 wn
	float num5[5] = {0.0088,    0.0144,    0.0197,    0.0144,    0.0088};
	float den5[5] = {1.0000,   -2.6537,    2.9740,   -1.5989,    0.3455};	
	filters->LPF_pitch = initialize_filter(4, num5, den5);		
	filters->LPF_roll = initialize_filter(4, num5, den5);	
	
	//zeroFilter(&core_state.yaw_ctrl);
	zeroFilter(filters->LPF_d_pitch);
	zeroFilter(filters->LPF_d_roll);
	zeroFilter(filters->LPF_d_yaw);
	zeroFilter(filters->LPF_Yaw_Ref_P);
	zeroFilter(filters->LPF_Yaw_Ref_R);
	//zeroFilter(filters->Outer_Loop_TF_pitch);
	//zeroFilter(filters->Outer_Loop_TF_roll);
	return 0;
}

int init_rotation_matrix(transform_matrix_t *transform, core_config_t *flight_config){
	float pitch_offset, roll_offset, yaw_offset;
	int i,j;
	
	rc_alloc_matrix(&transform->IMU_to_drone_dmp,3,3);
	rc_alloc_matrix(&transform->IMU_to_drone_gyro,3,3);
	rc_alloc_matrix(&transform->IMU_to_drone_accel,3,3);
	
	rc_alloc_vector(&transform->dmp_imu,3);
	rc_alloc_vector(&transform->gyro_imu,3);
	rc_alloc_vector(&transform->accel_imu,3);
	rc_alloc_vector(&transform->dmp_drone,3);
	rc_alloc_vector(&transform->gyro_drone,3);
	rc_alloc_vector(&transform->accel_drone,3);

	//pitch_offset = 0; roll_offset = M_PI; yaw_offset = - M_PI;
	pitch_offset = flight_config->pitch_offset_deg*DEG_TO_RAD; 
	roll_offset = flight_config->roll_offset_deg*DEG_TO_RAD;
	yaw_offset = flight_config->yaw_offset_deg*DEG_TO_RAD;
	
	float ROTATION_MAT1[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			transform->IMU_to_drone_dmp.d[i][j]=ROTATION_MAT1[i][j];
			transform->IMU_to_drone_gyro.d[i][j]=ROTATION_MAT1[i][j];
			transform->IMU_to_drone_accel.d[i][j]=ROTATION_MAT1[i][j];
		}
	}


	if (flight_config->imu_orientation == 2)
	{
		roll_offset += 180.0f; 		
		float ROTATION_MAT1[][3] = ROTATION_MATRIX1;
		for(i=0; i<3; i++){
			for(j=0; j<3; j++){
				transform->IMU_to_drone_gyro.d[i][j]=ROTATION_MAT1[i][j];
				transform->IMU_to_drone_accel.d[i][j]=ROTATION_MAT1[i][j];
			}
		}
	}
	printf("Rotation Vectors and Matrices Initialiazed \n");
	return 0;
}


int flyMS_shutdown(	logger_t *logger, 
					GPS_data_t *GPS_data, 
					flyMS_threads_t *flyMS_threads) 
{
	stop_core_log(&logger->core_logger);// finish writing core_log
	//Join the threads for a safe process shutdown
	if(GPS_data->GPS_init_check == 0)
	{
		join_GPS_thread(GPS_data);
		printf("GPS thread joined\n");
		pthread_join(flyMS_threads->kalman_thread, NULL);
		printf("Kalman thread joined\n");
	}
	pthread_join(flyMS_threads->core_logging_thread, NULL);
	printf("Logging thread joined\n");
	static char* StateStrings[] = {	"UNINITIALIZED", "RUNNING", 
									"PAUSED", "EXITING" };
	fprintf(logger->Error_logger,"Exiting program, system state is %s\n", StateStrings[rc_get_state()]);
	fflush(stdout);

	// Close the log files
	close(GPS_data->GPS_file);
	fclose(logger->GPS_logger);
	join_pru_client();
	
	rc_set_led(GREEN,OFF);
	rc_set_led(RED,OFF);
	return 0;
}





