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
static void init_fusion(fusion_data_t* fusion);
static void updateFusion(fusion_data_t *fusion);

//Local Structure to interpret imu_data
rc_imu_data_t imu_data;
int imu_orientation_id = 1;

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

int imu_handler(control_variables_t *control)
{
	int i = 0, i1 = 0;
	/**********************************************************
	*    				Read the Raw IMU Data     			  *
	**********************************************************/
	if(rc_read_accel_data(&imu_data)<0){
		printf("read accel data failed\n");
	}

	if(rc_read_mag_data(&imu_data)<0){
		printf("read mag data failed\n");
	}

	//Perform the data fusion to calculate pitch, roll, and yaw angles
	updateFusion(&control->fusion);

	/**********************************************************
	*		Perform the Coordinate System Transformation	  *
	**********************************************************/
	for (i=0;i<3;i++) 
	{	
		control->transform.dmp_imu.d[i] = control->fusion.eulerAngles.array[i] * DEG_TO_RAD;
		control->transform.gyro_imu.d[i] = imu_data.gyro[i] * DEG_TO_RAD;
		control->transform.accel_imu.d[i] = imu_data.accel[i];
	}
	//Convert from IMU coordinate system to drone's
	rc_matrix_times_col_vec(control->transform.IMU_to_drone_dmp, control->transform.dmp_imu, &control->transform.dmp_drone);
	rc_matrix_times_col_vec(control->transform.IMU_to_drone_gyro, control->transform.gyro_imu, &control->transform.gyro_drone);
	rc_matrix_times_col_vec(control->transform.IMU_to_drone_accel, control->transform.accel_imu, &control->transform.accel_drone);
	
	//Save newly calculated data into our control variable structure
	for (i = 0; i < 3; i++)
	{
		control->euler_previous[i] 		= control->euler[i];	
		control->euler[i] 				= control->transform.dmp_drone.d[i];
		control->euler_rate[i]			= control->transform.gyro_drone.d[i];
		control->mag[i]					= imu_data.mag[i];
	}
	control->compass_heading = imu_data.compass_heading_raw;	

	/**********************************************************
	*					Unwrap the Yaw value				  *
	**********************************************************/
	control->euler[2] 			= control->transform.dmp_drone.d[2] + control->num_wraps*2*M_PI;
	if(fabs(control->euler[2] - control->euler_previous[2])  > 5)
	{
		if(control->euler[2] > control->euler_previous[2]) control->num_wraps--;
		if(control->euler[2] < control->euler_previous[2]) control->num_wraps++;
	}
	control->euler[2]= control->transform.dmp_drone.d[2] + control->num_wraps*2*M_PI;


	/**********************************************************
	*           Read the Barometer for Altitude				  *
	**********************************************************/	
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
	control->ekf_filter.input.gyro[0] = control->euler_rate[1];
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
	imu_orientation_id = control->flight_config.imu_orientation;
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
	init_fusion(&control->fusion);

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
*              		Initialize the IMU Fusion Algorithm					*
************************************************************************/
static void init_fusion(fusion_data_t* fusion)
{
	FusionAhrsInitialise(&fusion->fusionAhrs, .75f, 0.0f, 120.0f); // valid magnetic field defined as 20 uT to 70 uT
//	int i;
	
//	FusionBias fusionBias;
	FusionBiasInitialise(&fusion->fusionBias, 25, DT);

	//Give Imu data to the fusion alg for initialization purposes
	while (FusionAhrsIsInitialising(&fusion->fusionAhrs))
	{
		if(rc_read_accel_data(&imu_data)<0){
			printf("read accel data failed\n");
		}
		if(rc_read_mag_data(&imu_data)<0){
			printf("read mag data failed\n");
		}
		updateFusion(fusion);
		rc_usleep(DT_US);
	}

}

/************************************************************************
*         Update the Fusion Algorithm, Called once per IMU Update       *
************************************************************************/
static void updateFusion(fusion_data_t *fusion)
{
	const FusionVector3 gyroscope = 
	{
		.axis.x = imu_data.gyro[0],
		.axis.y = imu_data.gyro[1],
		.axis.z = imu_data.gyro[2],
	};

	FusionVector3 accelerometer; 
	FusionVector3 magnetometer;

	switch (imu_orientation_id)
	{
		case 1:
			accelerometer.axis.x = imu_data.accel[0]/9.81f;
			accelerometer.axis.y = imu_data.accel[1]/9.81f;
			accelerometer.axis.z = imu_data.accel[2]/9.81f;

			magnetometer.axis.x = imu_data.mag[0];
			magnetometer.axis.y = imu_data.mag[1];
			magnetometer.axis.z = imu_data.mag[2];
			break;
		case 2:
			accelerometer.axis.x = -imu_data.accel[0]/9.81f;
			accelerometer.axis.y = imu_data.accel[1]/9.81f;
			accelerometer.axis.z = -imu_data.accel[2]/9.81f;

			magnetometer.axis.x = -imu_data.mag[0];
			magnetometer.axis.y = imu_data.mag[1];
			magnetometer.axis.z = -imu_data.mag[2];
			break;
		default:
			printf("Error Unrecognized IMU orientation\n");
			break;
	}

	FusionAhrsUpdate(&fusion->fusionAhrs, gyroscope, accelerometer, magnetometer, DT);												
//	FusionAhrsUpdate(&fusion->fusionAhrs, gyroscope, accelerometer, FUSION_VECTOR3_ZERO, DT);												
	fusion->eulerAngles = FusionQuaternionToEulerAngles(fusion->fusionAhrs.quaternion);
	
	//We want pitch and roll to be relative to the Drone's Local Coordinate Frame, not the NED Frame
	//	make that conversion here
	float mag, theta;
	mag = powf(powf(fusion->eulerAngles.angle.pitch,2.0f)+powf(fusion->eulerAngles.angle.roll,2.0f),0.5f);
	theta = atan2f(fusion->eulerAngles.angle.roll, fusion->eulerAngles.angle.pitch);
	
	fusion->eulerAngles.angle.roll = mag * cosf(theta-fusion->eulerAngles.angle.yaw*DEG_TO_RAD);
	fusion->eulerAngles.angle.pitch = mag * sinf(theta-fusion->eulerAngles.angle.yaw*DEG_TO_RAD);
	int i;
	for (i = 0; i < 3; i++)
		fusion->eulerAngles.array[i]*=-1.0f;
}


#ifdef __cplusplus
}
#endif
