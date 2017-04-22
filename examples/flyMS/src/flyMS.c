// flyMS.c Control Program to fly quadcopter
// By Michael Sardonini, with help from James Strawson



#define LAT_ACCEL_BIAS -0.0177
#define LON_ACCEL_BIAS  0.0063
#define ALT_ACCEL_BIAS  0.0000
#define YAW_OFFSET	  	1.106



#define PITCH_ROLL_RATE_KI .05
#define YAW_KI .05


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
#include "linear_algebra.h"
#include "filter.h"
#include "kalman.h"
#include "gps.h"
#include "logger.h"
#include "flyMS.h"


int flight_core(void * ptr);
//void* kalman_filter(void*ptr);
//int initialize_structs(accel_data_t *tmp, GPS_data_t *tmp2);


/************************************************************************
* 	Global Variables				
************************************************************************/
control_variables_t		control;			//Structure to contain all system states
setpoint_t 				setpoint; 			//Structure to store all Setpoints
GPS_data_t				GPS_data;			//Structure to store data from GPS
function_control_t 		function_control;	//Structure to store variables which control functions
filters_t				filters;			//Struct to contain all the filters
accel_data_t 			accel_data;			//A struct which is given to kalman.c
logger_t				logger;
tranform_matrix_t		transform;
uint16_t 				imu_err_count;
rc_imu_data_t			imu_data;			//Struct to relay all IMU info from driver to here
float 					accel_bias[3] = {LAT_ACCEL_BIAS, LON_ACCEL_BIAS, ALT_ACCEL_BIAS};
float 					yaw_offset[3] = {0, 0, YAW_OFFSET};
uint8_t 				DEBUG_MODE = 0;		//Run with arg -d to run in debug mode, assumes no battery connected


 
int flight_core(void * ptr){
	imu_err_count = 0;
	//control_variables_t *STATE = (control_variables_t*)ptr;
	//printf("pointer value %f\n", STATE->pitch);
	
	static rc_vector_t *X_state_Lat1, *X_state_Lon1;
	
	//Keep an i for loops
	static uint8_t i=0, i1=0;
	
	//Variables to Initialize things upon startup
	static uint8_t First_Iteration=1, First_Iteration_GPS=1;
	
	static float initial_alt = 0;

	//Initialize some variables if it is the first iteration
	if(First_Iteration){
		clock_gettime(CLOCK_MONOTONIC, &function_control.start_time); //Set the reference time to the first iteration
		setpoint.Aux[0] = 1; control.kill_switch[0]=1;
		function_control.dsm2_timeout=0;
		X_state_Lat1 = get_lat_state();
		X_state_Lon1 = get_lon_state();	
		rc_read_barometer();
		initial_alt = rc_bmp_get_altitude_m();
		rc_set_state(RUNNING);
		fprintf(logger.GPS_logger,"time,deg_lon,min_lon,deg_lat,min_lat,speed,direction,gps_alt,hdop,fix\n");
		printf("First Iteration ");
		}

	//Keep all other threads from interfering from this point until unlock
	pthread_mutex_lock(&function_control.lock);

	/**********************************************************
	*    Read the IMU for Rotational Position and Velocity    *
	**********************************************************/
	
	//Bring 3 axes of accel, gyro and angle data in to this program
	for (i=0;i<3;i++) 
	{	
		transform.dmp_imu.d[i] = imu_data.fused_TaitBryan[i] + yaw_offset[i];
		transform.gyro_imu.d[i] = imu_data.gyro[i] * DEG_TO_RAD;
		transform.accel_imu.d[i] = imu_data.accel[i];
	}
	//Convert from IMU coordinate system to drones
	rc_matrix_times_col_vec(transform.IMU_to_drone_dmp, transform.dmp_imu, &transform.dmp_drone);
	rc_matrix_times_col_vec(transform.IMU_to_drone_gyro, transform.gyro_imu, &transform.gyro_drone);
	rc_matrix_times_col_vec(transform.IMU_to_drone_accel, transform.accel_imu, &transform.accel_drone);

	//Subtract the gravity vector component from lat/lon accel
	transform.accel_drone.d[0]+= 9.8 * sin(transform.dmp_drone.d[0]);
	transform.accel_drone.d[1]+= 9.8 * sin(transform.dmp_drone.d[1]);
	
	//lowpass the accel data and subtract the biases
	accel_data.accel_Lat	= update_filter(filters.LPF_Accel_Lat,transform.accel_drone.d[0]-accel_bias[0]);
	accel_data.accel_Lon	= update_filter(filters.LPF_Accel_Lon,transform.accel_drone.d[1]-accel_bias[1]);
	accel_data.accelz		= transform.accel_drone.d[2]-accel_bias[2];

	control.pitch 			= update_filter(filters.LPF_pitch,transform.dmp_drone.d[0]);
	//control.pitch 			= dmp_drone.d[0];
	control.roll 			= update_filter(filters.LPF_roll,transform.dmp_drone.d[1]);
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
		
		
	//Store some info in the accel_data struct to send to kalman filer
	accel_data.pitch=control.pitch;
	accel_data.roll=control.roll;
	accel_data.yaw[0]=control.yaw[0];
	
	
	if(First_Iteration){
		setpoint.yaw_ref[0]=control.yaw[0];
		First_Iteration=0;
		control.yaw_ref_offset = control.yaw[0];
		printf("Started \n");
	}
	
	/**********************************************************
	*           Read the Barometer for Altitude				  *
	**********************************************************/	
	i1++;
	if (i1 == 8) // Only read the barometer at 25Hz
	{
		// perform the i2c reads to the sensor, this takes a bit of time
		if(rc_read_barometer()<0){
			printf("\rERROR: Can't read Barometer");
			fflush(stdout);
		}
		i1=0;
	}
	
	control.baro_alt = update_filter(filters.LPF_baro_alt,rc_bmp_get_altitude_m() - initial_alt);

	
	/**********************************************************
	*           Read the RC Controller for Commands           *
	**********************************************************/
	
	if(is_new_dsm2_dataMS()){
		//printf("DSM2 In\n");
		
		//Reset the timout counter back to zero
		function_control.dsm2_timeout=0;
		

		
		//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
		setpoint.yaw_rate_ref[1]=setpoint.yaw_rate_ref[0];		
		setpoint.yaw_rate_ref[0]=get_dsm2_ch_normalizedMS(4)*MAX_YAW_RATE;
		
		//Apply a deadzone to keep integrator from wandering
		if(fabs(setpoint.yaw_rate_ref[0])<0.05) {
			setpoint.yaw_rate_ref[0]=0;
		}
		
		//Kill Switch
		control.kill_switch[0]=get_dsm2_ch_normalizedMS(5)/2;
		
		//Auxillary Switch
		setpoint.Aux[1] = setpoint.Aux[0];
		setpoint.Aux[0]=get_dsm2_ch_normalizedMS(6); 
		
		setpoint.roll_ref=-get_dsm2_ch_normalizedMS(2)*MAX_ROLL_RANGE;	//DSM2 Receiver is inherently positive to the left
		setpoint.pitch_ref=get_dsm2_ch_normalizedMS(3)*MAX_PITCH_RANGE;
		
		//Convert from Drone Coordinate System to User Coordinate System
		float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
		float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
		setpoint.roll_ref =P_R_MAG*cos(Theta_Ref-control.yaw[0]+control.yaw_ref_offset);
		setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref-control.yaw[0]+control.yaw_ref_offset);

			if(setpoint.Aux[0]>0)//Remote Controlled Flight
		{ 
			//Set the throttle
			control.throttle=(get_dsm2_ch_normalizedMS(1)+1)*0.5*(MAX_THROTTLE-MIN_THROTTLE)+MIN_THROTTLE;
		
			//Keep the aircraft at a constant height while making manuevers 
			control.throttle *= 1/(cos(control.pitch)*cos(control.roll));
		
			function_control.altitudeHold = 0;
		}
		else  //Flight by GPS and/or Lidar and Barometer
		{
			setpoint.altitudeSetpointRate = get_dsm2_ch_normalizedMS(1) * MAX_ALT_SPEED;
			
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
	}
	else{ //check to make sure too much time hasn't gone by since hearing the RC
		
		if(!DEBUG_MODE)
		{
			function_control.dsm2_timeout=function_control.dsm2_timeout+1;
			
			if(function_control.dsm2_timeout>1.5/DT) {
				printf("\nLost Connection with Remote!! Shutting Down Immediately \n");	
				fprintf(logger.Error_logger,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
				rc_set_state(EXITING);
			}
		}
	}
	
	if(DEBUG_MODE)
	{
		control.throttle = MIN_THROTTLE;
		setpoint.Aux[0] = 0;
		setpoint.roll_ref = 0;
		setpoint.pitch_ref = 0;
		setpoint.yaw_rate_ref[0] = 0;
		setpoint.yaw_rate_ref[1] = 0;
		setpoint.roll_ref = 0;
	}
	
		
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
        setpoint.altitudeSetpoint=setpoint.altitudeSetpoint+(setpoint.altitudeSetpointRate)*DT;

		control.uthrottle = update_filter(filters.altitudeHoldPID, setpoint.altitudeSetpoint - control.baro_alt);
		control.throttle = control.uthrottle + control.standing_throttle;
		
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
	setpoint.filt_pitch_ref = update_filter(filters.LPF_Yaw_Ref_P,setpoint.pitch_ref);
	setpoint.filt_roll_ref = update_filter(filters.LPF_Yaw_Ref_R,setpoint.roll_ref);
	
	//Filter out high frequency noise in Raw Gyro data
	control.d_pitch_f = update_filter(filters.LPF_d_pitch,control.d_pitch);			
	control.d_roll_f = update_filter(filters.LPF_d_roll,control.d_roll);

	//Set the Pitch reference Setpoint
	//control.dpitch_setpoint=((setpoint.filt_pitch_ref - control.pitch)*  
	//				4 - control.d_pitch_f);
	
	control.dpitch_setpoint = update_filter(filters.pitch_PD, setpoint.filt_pitch_ref - control.pitch) - control.d_pitch_f;
	
	//Set the Roll reference Setpoint
	//control.droll_setpoint=((setpoint.filt_roll_ref - control.roll)* 
	//				4 - control.d_roll_f);	
	
	control.droll_setpoint = update_filter(filters.roll_PD, setpoint.filt_roll_ref - control.roll) - control.d_roll_f;
	
	//Apply the PD Controllers 
	control.upitch = update_filter(filters.pitch_rate_PD,control.dpitch_setpoint);
	control.uroll = update_filter(filters.roll_rate_PD,control.droll_setpoint);				
	

	
	/************************************************************************
	*                        	Yaw Controller                              *
	************************************************************************/	
	control.d_yaw_f = update_filter(filters.LPF_d_yaw,control.d_yaw);
	
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
		
		control.upitch+=PITCH_ROLL_RATE_KI * control.dpitch_err_integrator;
		control.uroll +=PITCH_ROLL_RATE_KI * control.droll_err_integrator;
		control.uyaw+=YAW_KI * control.dyaw_err_integrator;
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
	
	control.u[0]=control.throttle+control.uroll-control.upitch+control.uyaw;
	control.u[1]=control.throttle-control.uroll-control.upitch-control.uyaw;
	control.u[2]=control.throttle+control.uroll+control.upitch-control.uyaw;
	control.u[3]=control.throttle-control.uroll+control.upitch+control.uyaw;		

	
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

	

	if(!DEBUG_MODE)
	{
		//Send Commands to Motors
		if(rc_get_state()!=EXITING){
			for(i=0;i<4;i++){
				rc_send_esc_pulse_normalized(i+1,control.u[i]);
			}
		}
		else{
			for(i=0;i<4;i++){
				control.u[i] = 0;
				rc_send_esc_pulse_normalized(i+1,control.u[i]);
			}	
		}
		if(control.kill_switch[0] < .5) {
			printf("\nKill Switch Hit! Shutting Down\n");
			fprintf(logger.Error_logger,"\nKill Switch Hit! Shutting Down\n");	
			rc_set_state(EXITING);
		}
	}	

	fflush(stdout);	
	
	clock_gettime(CLOCK_MONOTONIC, &function_control.log_time);
	control.time=(float)(function_control.log_time.tv_sec - function_control.start_time.tv_sec) + 
						((float)(function_control.log_time.tv_nsec - function_control.start_time.tv_nsec) / 1000000000) ;
	
		
	logger.new_entry.time			= control.time;	
	logger.new_entry.pitch			= control.pitch;	
	logger.new_entry.roll			= control.roll;
	logger.new_entry.yaw			= control.yaw[0];
	logger.new_entry.d_pitch		= control.d_pitch_f;	
	logger.new_entry.d_roll			= control.d_roll_f;
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
	if (X_state_Lat1->initialized)
	{
		logger.new_entry.kalman_lat		= X_state_Lat1->d[0];
		logger.new_entry.kalman_lon		= X_state_Lon1->d[0];
	}
	logger.new_entry.accel_lat		= accel_data.accel_Lat;
	logger.new_entry.accel_lon		= accel_data.accel_Lon;
	logger.new_entry.baro_alt		= control.baro_alt;
	logger.new_entry.v_batt			= rc_dc_jack_voltage();
	log_core_data(&logger.core_logger, &logger.new_entry);
	
		//Print some stuff
		printf("\r ");
		printf("time %3.3f ", control.time);
	//	printf("Alt %2.2f ",lidar_data.altitude[0]);
	//	printf("vel %2.2f ",lidar_data.d_altitude[0]);		
	//	printf("H_d %3.3f ", control.height_damping);
	//	printf("Alt_ref %3.1f ",control.alt_ref);
	//	printf(" U1:  %2.2f ",control.u[0]);
	//	printf(" U2: %2.2f ",control.u[1]);
	//	printf(" U3:  %2.2f ",control.u[2]);
	//	printf(" U4: %2.2f ",control.u[3]);	
		printf(" TH %2.2f ", control.throttle);
		printf("Aux %2.1f ", setpoint.Aux[0]);
	//	printf("function: %f",get_dsm2_ch_normalizedMS(6));
	//	printf("num wraps %d ",control.num_wraps);
		printf(" Pitch_ref %2.2f ", setpoint.filt_pitch_ref);
	//	printf(" Roll_ref %2.2f ", setpoint.filt_roll_ref);
	//	printf(" Yaw_ref %2.2f ", setpoint.yaw_ref[0]);
	//	printf(" Pitch %1.2f ", control.pitch);
	//	printf(" Roll %1.2f ", control.roll);
	//	printf(" Yaw %2.3f ", control.yaw[0]); 
	//	printf(" DPitch %1.2f ", control.d_pitch_f); 
	//	printf(" DRoll %1.2f ", control.d_roll_f);
	//	printf(" DYaw %2.3f ", control.d_yaw); 	
	//	printf(" uyaw %2.3f ", control.upitch); 		
	//	printf(" uyaw %2.3f ", control.uroll); 		
	//	printf(" uyaw %2.3f ", control.uyaw);
	//	printf(" GPS pos lat: %2.2f", GPS_data.pos_lat);
	//	printf(" GPS pos lon: %2.2f", GPS_data.pos_lon);
	//	printf(" HDOP: %f", GPS_data.HDOP);
	//	printf(" Acc_Lat %2.3f ", accel_data.accel_Lat);
	//	printf(" Acc_Lon %2.3f ", accel_data.accel_Lon);
	//	printf(" Acc_z %2.3f", accel_data.accelz);
	//	printf(" Pos_Lat %2.3f ", X_state_Lat1.data[0]);	
	//	printf(" Pos_Lon %2.3f ", X_state_Lon1.data[0]);
	//	printf("control: %d",get_state());
	//	printf("Baro Alt: %f ",baro_alt);
		
		
		/***************** Get GPS Data if available *******************/
	if(is_new_GPS_data()){
		
		accel_data.GPS_kal_flag = 1;
		 //printf("\n new GPS data in \n");
		fprintf(logger.GPS_logger,"%4.5f,",control.time);
		fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_longitude,GPS_data.min_longitude);
		fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_latitude,GPS_data.min_latitude);
		fprintf(logger.GPS_logger,"%f,%f,",GPS_data.speed,GPS_data.direction);
		fprintf(logger.GPS_logger,"%f,",GPS_data.gps_altitude);
		fprintf(logger.GPS_logger,"%2.2f,%d",GPS_data.HDOP,GPS_data.GPS_fix);
		fprintf(logger.GPS_logger,"\n");
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
	}
	pthread_mutex_unlock(&function_control.lock);
		
	
		
	return 0;
}
	
	
int main(int argc, char *argv[]){
	int in;
	while ((in = getopt(argc, argv, "d")) != -1)
	{
		switch (in){
			case 'd': 
				DEBUG_MODE = 1;
				printf("Running in Debug mode, assumes no battery plugged in \n");
				break;	
			default:
				printf("Invalid Argument \n");
				return -1;
				break;
			}
	} 
 
	//Initialize some cape and beaglebone hardware
	if(rc_initialize()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}	
	
	//Define some threads
	pthread_t led_thread;
	pthread_t kalman_thread;
	pthread_t core_logging_thread;
	pthread_t quiet_esc_thread;
	//pthread_t barometer_alt_thread;
	
	uint8_t flight_core_running = 0;
	pthread_create(&quiet_esc_thread, NULL, quietEscs, &flight_core_running);
	 
	
	
	if(rc_initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
		printf("initialize_barometer failed\n");
		return -1;
	}
	
	
	// set up IMU configuration
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE;
	imu_config.orientation = ORIENTATION_Z_UP;
	imu_config.accel_fsr = A_FSR_2G;
	imu_config.enable_magnetometer=1;
	
	control_variables_t *ptr1; 
	ptr1 = malloc(sizeof(control_variables_t));
	ptr1->pitch = 20;
	
	// start imu
	if(rc_initialize_imu_dmp(&imu_data, imu_config, (void*)ptr1)){	
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		rc_blink_led(RED, 5, 5);
		return -1;
	}

	//Initialize the remote controller
		initialize_dsm2MS();
	
	if(!DEBUG_MODE)
	{	
		if(ready_check(&control)){
			printf("Exiting Program \n");
			return -1;
		} //Toggle the kill switch a few times to signal it's ready
	}
	
	
	// start a core_log and logging thread
	if(start_core_log(&logger)<0){
		printf("WARNING: failed to open a core_log file\n");
	}
	else{
		pthread_create(&core_logging_thread, NULL, core_log_writer, &logger.core_logger);
	}

	sleep(2); //wait for the IMU to level off	
	printf("Made it here\n");
	init_rotation_matrix(&transform); //Initialize the rotation matrix from IMU to drone
	initialize_filters(&filters);
	printf("Made it here\n");
	//Start the GPS thread, flash the LED's if GPS has a fix
	GPS_data.GPS_init_check=GPS_init(argc, argv,&GPS_data);
	
	
	led_thread_t GPS_ready;
	memset(&GPS_ready,0,sizeof(GPS_ready));
	GPS_ready.GPS_fix_check = GPS_data.GPS_fix_check;
	GPS_ready.GPS_init_check = GPS_data.GPS_init_check;
	pthread_create(&led_thread, NULL, LED_thread, (void*) &GPS_ready);
	
	
	//Spawn the Kalman Filter Thread if GPS is running
	if (GPS_data.GPS_init_check == 0)
	{
		pthread_create(&kalman_thread, NULL , kalman_filter, (void*) NULL);
	}
	//Give the ESCs a zero command before starting to prevent going into calibration
	//
	

	rc_disable_servo_power_rail();
	init_esc_hardware();	 
	 
	//Start the mutex lock to prevent threads from interfering
	if(pthread_mutex_init(&function_control.lock,NULL))
	{
		printf("Error Lock init failed\n");
	}
		
	//Start the control program
	flight_core_running = 1;
	rc_set_imu_interrupt_func(&flight_core);
	
	printf("Starting \n");
	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) {
		usleep(5000);
		imu_err_count++;
		if (imu_err_count == 3 || imu_err_count % 50 == 0)
		{
			fprintf(logger.Error_logger,"Error! IMU read failed for more than 3 consecutive timesteps. time: = %f number of missed reads: %u\n",control.time,imu_err_count);
		}
	}
	flight_core_running = 0;
	
	stop_core_log(&logger.core_logger);// finish writing core_log
	
	//Join the threads for a safe process shutdown
	if(GPS_data.GPS_init_check == 0)
	{
		join_GPS_thread(&GPS_data);
		printf("GPS thread joined\n");
		pthread_join(kalman_thread, NULL);
		printf("Kalman thread joined\n");
	}
	pthread_join(led_thread, NULL);
	printf("LED thread joined\n");
	pthread_join(core_logging_thread, NULL);
	printf("Logging thread joined\n");
	  
	static char* StateStrings[] = {	"UNINITIALIZED", "RUNNING", 
									"PAUSED", "EXITING" };
	fprintf(logger.Error_logger,"Exiting program, system state is %s\n", StateStrings[rc_get_state()]);
	fflush(stdout);

	// Close the log files
	close(GPS_data.GPS_file);
	fclose(logger.GPS_logger);
	
	flight_core_running = 1;
	pthread_join(quiet_esc_thread, NULL);
	printf("Quiet Esc thread joined\n");
	rc_cleanup();
	return 0;
	}
	
	
accel_data_t* get_accel_pointer(){
	return &accel_data;
}
	
GPS_data_t* get_GPS_pointer(){
	return &GPS_data;
}
