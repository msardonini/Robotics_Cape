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


// parse_imu.c Control Program to fly quadcopter
// By Michael Sardonini

#ifdef __cplusplus
extern "C" {
#endif

#include "flyMS_common.h"
#include "flyMS.h"


//Local Functions
// static int init_fusion_bias(FusionBias *fusionBias);
static void init_fusion(fusion_data_t* fusion, transform_matrix_t *transform);
static void updateFusion(fusion_data_t *fusion, transform_matrix_t *transform);
static void read_transform_imu(transform_matrix_t *transform);
static void init_rotation_matrix(transform_matrix_t *transform, core_config_t *flight_config);

//Local Structure to interpret imu_data
rc_imu_data_t imu_data;

/*
	imu_handler()
		Does all the parsing and interpretting of the IMU
		5 main tasks
			1. Reads the data from the IMU using Robotics_Cape API
			2. Performs a coordinate system transformation from imu -> drone
			3. Unwraps the yaw value for proper PID control
			4. Reads Barometer for altitude measurement
			5. Sends data to the EKF for position control
*/

int imu_handler(control_variables_t *control, filters_t *filters)
{
	int i = 0;
	/**********************************************************
	*    		Read and Translate the Raw IMU Data     			  
	**********************************************************/
	read_transform_imu(&control->transform);

	//Perform the data fusion to calculate pitch, roll, and yaw angles
	updateFusion(&control->fusion, &control->transform);



	//Place the tranformed data into our control struct
	for (i = 0; i < 3; i++)
	{
		control->euler_previous[i] 		= control->euler[i];	
		control->euler[i] 				= control->fusion.eulerAngles.array[i] * DEG_TO_RAD;
		control->euler_rate[i]			= update_filter(filters->gyro_lpf[i],control->transform.gyro_drone.d[i]);
		control->mag[i]					= control->transform.mag_drone.d[i];
		control->accel[i]				= control->transform.accel_drone.d[i];
	}
	control->compass_heading = imu_data.compass_heading_raw;

	/**********************************************************
	*					Unwrap the Yaw value				  *
	**********************************************************/
	control->euler[2] += control->num_wraps*2*M_PI;
	if(fabs(control->euler[2] - control->euler_previous[2])  > 5)
	{
		if(control->euler[2] > control->euler_previous[2]) 
		{
			control->num_wraps--;
			control->euler[2] -= 2*M_PI;
		}
		else
		{
			control->num_wraps++;
			control->euler[2] += 2*M_PI;	
		}
	}

	/**********************************************************
	*           Read the Barometer for Altitude				  *
	**********************************************************/	
	static int i1;
	if (control->flight_config.enable_barometer)
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
		// control->baro_alt = update_filter(filters.LPF_baro_alt,rc_bmp_get_altitude_m() - initial_alt);
		control->baro_alt = rc_bmp_get_altitude_m();
		control->ekf_filter.input.barometer_updated = 1;
		control->ekf_filter.input.barometer_alt = control->baro_alt;
	}

	/************************************************************************
	*                   	Send data to PX4's EKF                          *
	************************************************************************/
	for (i = 0; i < 3; i++)
	{
		control->ekf_filter.input.accel[i] = control->transform.accel_drone.d[i];
		control->ekf_filter.input.mag[i] = imu_data.mag[i] * MICROTESLA_TO_GAUSS;
	}
	control->ekf_filter.input.gyro[0] = control->euler_rate[0];
	control->ekf_filter.input.gyro[1] = control->euler_rate[1];
	control->ekf_filter.input.gyro[2] = control->euler_rate[2];
	control->ekf_filter.input.IMU_timestamp = control->time;

	return 0;
}

/************************************************************************
*					   Update the EKF with GPS Data                     *
************************************************************************/
int update_ekf_gps(control_variables_t *control, GPS_data_t *GPS_data)
{
	control->ekf_filter.input.gps_timestamp = control->time*1E6;
	control->ekf_filter.input.gps_latlon[0] = (double)GPS_data->deg_latitude + (double)GPS_data->min_latitude / 60.0;// + control->time*1E7/20000;
	control->ekf_filter.input.gps_latlon[1] = (double)GPS_data->deg_longitude + (double)GPS_data->min_longitude / 60.0;
	control->ekf_filter.input.gps_latlon[2] = (double)GPS_data->gps_altitude;
	control->ekf_filter.input.gps_fix = GPS_data->GPS_fix;
	control->ekf_filter.input.nsats = 10; // Really need to fix this
	control->ekf_filter.input.gps_updated = 1;
	return 0;
}


/************************************************************************
*							Initialize the IMU                          *
************************************************************************/
int initialize_imu(control_variables_t *control)
{
	//Allocate memory for the tranformation matrices and vectors
	init_rotation_matrix(&control->transform, &control->flight_config);

	//Start the barometer
	if(control->flight_config.enable_barometer)
	{
		if(rc_initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
			printf("initialize_barometer failed\n");
			return -1;
		}
	}

	// set up IMU configuration
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.accel_fsr = A_FSR_2G;
	imu_config.enable_magnetometer=1;
//	imu_config.accel_dlpf = ACCEL_DLPF_5;
//	imu_config.gyro_dlpf = GYRO_DLPF_5;

	// start imu
	if(rc_initialize_imu(&imu_data, imu_config)){
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		rc_blink_led(RED, 5, 5);
		return -1;
	}
	
	//Initialize the fusion library which converts raw IMU data to Euler angles
	init_fusion(&control->fusion, &control->transform);

	return 0;
}

// /************************************************************************
// *              Set the Bias value in the Fuction Algorithm              *
// ************************************************************************/
// static int init_fusion_bias(FusionBias *fusionBias)
// {
// 	// print gyro data
// 	if(rc_read_gyro_data(&imu_data)<0){
// 		printf("read gyro data failed\n");
// 	}
//     FusionBiasInitialise(fusionBias, 50, DT); // assumes 100 Hz sample rate
//     FusionBiasUpdate(fusionBias, imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]); // literal values should be replaced with sensor measurements
//     return 0;
// }

/************************************************************************
*				Transform coordinate system IMU Data 
************************************************************************/
static void read_transform_imu(transform_matrix_t *transform)
{
	if(rc_read_accel_data(&imu_data)<0){
		printf("read accel data failed\n");
	}
	if(rc_read_mag_data(&imu_data)<0){
		printf("read mag data failed\n");
	}

	/**********************************************************
	*		Perform the Coordinate System Transformation	  *
	**********************************************************/
	int i;
	for (i=0;i<3;i++) 
	{	
		transform->mag_imu.d[i] = imu_data.mag[i];
		transform->gyro_imu.d[i] = imu_data.gyro[i];
		transform->accel_imu.d[i] = imu_data.accel[i];
	}
	//Convert from IMU coordinate system to drone's
	rc_matrix_times_col_vec(transform->IMU_to_drone, transform->mag_imu, &transform->mag_drone);
	rc_matrix_times_col_vec(transform->IMU_to_drone, transform->gyro_imu, &transform->gyro_drone);
	rc_matrix_times_col_vec(transform->IMU_to_drone, transform->accel_imu, &transform->accel_drone);
}


/************************************************************************
*              		Initialize the IMU Fusion Algorithm					*
************************************************************************/
static void init_fusion(fusion_data_t* fusion, transform_matrix_t *transform)
{
	FusionAhrsInitialise(&fusion->fusionAhrs, 0.75f, 0.0f, 120.0f); // valid magnetic field defined as 20 uT to 70 uT

	FusionBiasInitialise(&fusion->fusionBias, 25, DT);

	//Give Imu data to the fusion alg for initialization purposes
	while (FusionAhrsIsInitialising(&fusion->fusionAhrs))
	{
		read_transform_imu(transform);
		
		updateFusion(fusion, transform);
		rc_usleep(DT_US);
	}
}
/************************************************************************
*			Allocatate memeory for the Tranformation Matrices
************************************************************************/
static void init_rotation_matrix(transform_matrix_t *transform, core_config_t *flight_config)
{
	float pitch_offset, roll_offset, yaw_offset;
	int i,j;
	
	rc_alloc_matrix(&transform->IMU_to_drone,3,3);
	
	rc_alloc_vector(&transform->gyro_imu,3);
	rc_alloc_vector(&transform->accel_imu,3);
	rc_alloc_vector(&transform->mag_imu,3);
	rc_alloc_vector(&transform->gyro_drone,3);
	rc_alloc_vector(&transform->accel_drone,3);
	rc_alloc_vector(&transform->mag_drone,3);

	//pitch_offset = 0; roll_offset = M_PI; yaw_offset = - M_PI;
	pitch_offset = flight_config->pitch_offset_deg*DEG_TO_RAD; 
	roll_offset = flight_config->roll_offset_deg*DEG_TO_RAD;
	yaw_offset = flight_config->yaw_offset_deg*DEG_TO_RAD;
	
	float ROTATION_MAT1[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			transform->IMU_to_drone.d[i][j]=ROTATION_MAT1[i][j];
		}
	}
}

/************************************************************************
*         Update the Fusion Algorithm, Called once per IMU Update       *
************************************************************************/
static void updateFusion(fusion_data_t *fusion, transform_matrix_t *transform)
{

	FusionVector3 gyroscope;
	FusionVector3 accelerometer; 
	FusionVector3 magnetometer;

	int i;
	for (i = 0; i < 3; i++)
	{
		gyroscope.array[i] = transform->gyro_drone.d[i];
		accelerometer.array[i] = transform->accel_drone.d[i];
		magnetometer.array[i] = transform->mag_drone.d[i];
	}

	FusionAhrsUpdate(&fusion->fusionAhrs, gyroscope, accelerometer, magnetometer, DT);																								
	fusion->eulerAngles = FusionQuaternionToEulerAngles(fusion->fusionAhrs.quaternion);
	
	//We want pitch and roll to be relative to the Drone's Local Coordinate Frame, not the NED Frame
	//	make that conversion here
	float mag, theta;
	mag = powf(powf(fusion->eulerAngles.angle.pitch,2.0f)+powf(fusion->eulerAngles.angle.roll,2.0f),0.5f);
	theta = atan2f(fusion->eulerAngles.angle.roll, fusion->eulerAngles.angle.pitch);
	
	fusion->eulerAngles.angle.roll = mag * cosf(theta-fusion->eulerAngles.angle.yaw*DEG_TO_RAD);
	fusion->eulerAngles.angle.pitch = mag * sinf(theta-fusion->eulerAngles.angle.yaw*DEG_TO_RAD);

	for (i = 0; i < 3; i++)
		fusion->eulerAngles.array[i]*=-1.0f;
}


#ifdef __cplusplus
}
#endif
