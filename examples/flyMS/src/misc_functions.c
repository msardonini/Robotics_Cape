#include "../../../libraries/roboticscape.h"
#include "flyMS.h"
#include <pthread.h>
#include "gps.h"
#include <inttypes.h>
//Coordinate system transformations matrices


#define PITCH_ROLL_KP 5 
#define PITCH_ROLL_KI 2
#define PITCH_ROLL_KD 0.175


#define PITCH_ROLL_RATE_KP 0.015   //0.0285
#define PITCH_ROLL_RATE_KD 0.00155 //.00175

#define YAW_KP 0.5 //0.6
#define YAW_KD 0.05



int ready_check(control_variables_t *control){
	//Toggle the kill switch to get going, to ensure controlled take-off
	//Keep kill switch down to remain operational
    int count=1;
	printf("Toggle the kill swtich twice and leave up to initialize\n");
	while(count<6 && rc_get_state()!=EXITING){
		if(is_new_dsm2_dataMS()){
			control->kill_switch[1]=control->kill_switch[0];
			control->kill_switch[0]=get_dsm2_ch_normalizedMS(5);
			usleep(100000);
			if(control->kill_switch[0] < -0.75 && control->kill_switch[1] > 0.15){
			count++;
			} 
			if(control->kill_switch[0] > 0.75 && control->kill_switch[1] < 0.35){
			count++;
			}
			usleep(10000);
			
		}
		usleep(10000);
	}
	
	//make sure the kill switch is in the position to fly before starting
	while(control->kill_switch[0] < 0.5 && rc_get_state()!=EXITING)
		{
		if(is_new_dsm2_dataMS()){
			control->kill_switch[0]=get_dsm2_ch_normalizedMS(5);	
			}
		usleep(10000);
		}
	
	if(rc_get_state() == EXITING)
	{
		return -1;
	}
	
	printf("\nInitialized! Starting program\n");
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
	

// Send a zero command to the ESC's to shut them up when not running flight_core
void* quietEscs(void *ptr){
	
	uint8_t *flight_core_running= (uint8_t*)ptr;
	
	while(rc_get_state()!=EXITING)
	{
		if (!*flight_core_running)
		{
			zero_escs();
		}
		usleep(20000);
	}
	
	//keep sending a zero command until program has exitied
	while(!*flight_core_running)
	{	
		zero_escs();	
		usleep(20000);
	}
	return ptr;
  }
	



	
	
	
/************************************************************************
*	initialize_filters()
*	setup of feedback controllers used in flight core
************************************************************************/
int initialize_filters(filters_t *filters){

	
	filters->pitch_PD = generatePID(PITCH_ROLL_KP, PITCH_ROLL_KI, PITCH_ROLL_KD, 0.15, DT);
	filters->roll_PD  = generatePID(PITCH_ROLL_KP, PITCH_ROLL_KI, PITCH_ROLL_KD, 0.15, DT);
	//filters->yaw_PD   = generatePID(YAW_KP,		  0, YAW_KD,	    0.15, 0.005);

	//PD Controller (I is done manually)
	filters->pitch_rate_PD = generatePID(PITCH_ROLL_RATE_KP, 0, PITCH_ROLL_RATE_KD, 0.15, DT);
	filters->roll_rate_PD  = generatePID(PITCH_ROLL_RATE_KP, 0, PITCH_ROLL_RATE_KD, 0.15, DT);
	filters->yaw_rate_PD   = generatePID(YAW_KP,		  0, YAW_KD,	    0.15, DT);
	
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

int init_rotation_matrix(tranform_matrix_t *transform){
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
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = - 5* M_PI/4;
	float ROTATION_MAT1[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		transform->IMU_to_drone_dmp.d[i][j]=ROTATION_MAT1[i][j];
		}
	}
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = -5 * M_PI/4;
	float ROTATION_MAT2[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		transform->IMU_to_drone_gyro.d[i][j]=ROTATION_MAT2[i][j];
		}
	}
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = M_PI/4;
	float ROTATION_MAT3[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		transform->IMU_to_drone_accel.d[i][j]=ROTATION_MAT3[i][j];
		}
	}
	
	printf("Vectors and matrices initialiazed \n");
	return 0;
}

void zero_escs(){
	int i;
	for (i=0;i<8;i++){
		rc_send_esc_pulse_normalized(i+1, 0);	
	}
}


void init_esc_hardware(){
	uint8_t p;
	for (p = 0; p < 100; p++)
	{
		rc_send_esc_pulse_normalized_all(0); 
	usleep(1000000/100); //run at about 50 hz
	}
}


void* barometer_monitor(void* ptr){
	control_variables_t *control= (control_variables_t*)control;
		
	while(rc_get_state()!=EXITING)
	{
		// perform the i2c reads to the sensor, this takes a bit of time
		if(rc_read_barometer()<0){
			printf("\rERROR: Can't read Barometer");
			fflush(stdout);
			continue;
		}
		control->baro_alt = rc_bmp_get_altitude_m();
		usleep(1000000/BMP_CHECK_HZ);
	}
	return NULL;
}
