 /**
 * @file imu.cpp
 * @brief Source code to read data from the IMU and process it accordingly 
 *
 * @author Mike Sardonini
 * @date 10/16/2018
 */

#include "imu.hpp"


// Default Constructor
imu::imu(bool _enableBarometer) : 
	isInitializingFusion(true),
	enableBarometer(_enableBarometer)
{
	//Created memory for the rotation matricies
	this->initializeRotationMatrices();

	//Initialize the fusion library which converts raw IMU data to Euler angles
	init_fusion();
}


//Default Destructor
imu::~imu()
{

}

int imu::initializeImu()
{
	//Allocate memory for the tranformation matrices and vectors
	this->initializeRotationMatrices();

	//Start the barometer
	if(this->enableBarometer)
	{
		if(rc_initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
			printf("initialize_barometer failed\n");
			return -1;
		}
	}

	// set up IMU configuration
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.accel_fsr = A_FSR_4G;
	imu_config.enable_magnetometer=1;
	imu_config.accel_dlpf = ACCEL_DLPF_5;
	imu_config.gyro_dlpf = GYRO_DLPF_5;

	// start imu
	if(rc_initialize_imu(&this->imu_data, imu_config)){
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		rc_blink_led(RED, 5, 5);
		return -1;
	}

	return 0;

}



int imu::update()
{
	/**********************************************************
	*    		Read and Translate the Raw IMU Data     			  
	**********************************************************/
	this->read_transform_imu();
	//Perform the data fusion to calculate pitch, roll, and yaw angles
	this->updateFusion();



	//Place the tranformed data into our control struct
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		this->stateBody.eulerPrevious[i] 		= this->stateBody.euler[i];	
		this->stateBody.euler[i] 				= this->eulerAngles.array[i] * DEG_TO_RAD;
		// this->stateBody.eulerRate[i]			= update_filter(filters->gyro_lpf[i],this->transform.gyro_drone.d[i] * DEG_TO_RAD);
		this->stateBody.eulerRate[i]			= this->transform.gyro_drone.d[i] * DEG_TO_RAD;
		this->stateBody.mag[i]					= this->transform.mag_drone.d[i];
		this->stateBody.accel[i]				= this->transform.accel_drone.d[i];
		this->stateBody.gyro[i]				= this->stateBody.eulerRate[i];
	}
	this->stateBody.compassHeading = imu_data.compass_heading_raw;

	/**********************************************************
	*					Unwrap the Yaw value				  *
	**********************************************************/
	this->stateBody.euler[2] += this->stateBody.num_wraps*2*M_PI;
	if(fabs(this->stateBody.euler[2] - this->stateBody.eulerPrevious[2])  > 5)
	{
		if(this->stateBody.euler[2] > this->stateBody.eulerPrevious[2]) 
		{
			this->stateBody.num_wraps--;
			this->stateBody.euler[2] -= 2*M_PI;
		}
		else
		{
			this->stateBody.num_wraps++;
			this->stateBody.euler[2] += 2*M_PI;	
		}
	}

	/**********************************************************
	*           Read the Barometer for Altitude				  *
	**********************************************************/	
	static int i1;
	if (this->enableBarometer)
	{		
		i1++;
		if (i1 == 10) // Only read the barometer at 25Hz
		{
			// perform the i2c reads to the sensor, this takes a bit of time
			if(rc_read_barometer()<0){
				printf("\rERROR: Can't read Barometer");
				fflush(stdout);
			}
			i1=0;
		}
		// this->baro_alt = update_filter(filters.LPF_baro_alt,rc_bmp_get_altitude_m() - initial_alt);
		this->stateBody.barometerAltitude = rc_bmp_get_altitude_m();
		this->ekfContainer.input.barometer_updated = 1;
		this->ekfContainer.input.barometer_alt = this->stateBody.barometerAltitude;
	}

	/************************************************************************
	*                   	Send data to PX4's EKF                          *
	************************************************************************/
	for (i = 0; i < 3; i++)
	{
		this->ekfContainer.input.accel[i] = this->transform.accel_drone.d[i];
		this->ekfContainer.input.mag[i] = imu_data.mag[i] * MICROTESLA_TO_GAUSS;
		this->ekfContainer.input.gyro[i] = this->stateBody.eulerRate[i];
	}
	//TODO implement the EKF and use real timestamps
	// this->ekfContainer.input.IMU_timestamp = this->time_us;

	return 0;
}



/************************************************************************
*				Transform coordinate system IMU Data 
************************************************************************/
void imu::read_transform_imu()
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
		this->transform.mag_imu.d[i] = imu_data.mag[i];
		this->transform.gyro_imu.d[i] = imu_data.gyro[i];
		this->transform.accel_imu.d[i] = imu_data.accel[i];
	}

	//Convert from IMU coordinate system to drone's
	rc_matrix_times_col_vec(this->transform.IMU_to_drone, this->transform.mag_imu, &this->transform.mag_drone);
	rc_matrix_times_col_vec(this->transform.IMU_to_drone, this->transform.gyro_imu, &this->transform.gyro_drone);
	rc_matrix_times_col_vec(this->transform.IMU_to_drone, this->transform.accel_imu, &this->transform.accel_drone);
}


/************************************************************************
*              		Initialize the IMU Fusion Algorithm					*
************************************************************************/
void imu::init_fusion()
{
	/*	Three params here are: 
		1. gain -  (<= 0) 0 means to only use gyro data in fusion, greater value use accel / mag more
		2. min squared magnetic field, magnetic fields squared less than this will be discarded
		3. max squared magnetic field, magnetic fields squared greater than this will be discarded
	*/
	FusionAhrsInitialise(&this->fusionAhrs, 0.25f, 0.0f, 120.0f); // valid magnetic field defined as 20 uT to 70 uT
	/*
		Two params here are: 
		1. Min ADC threshold - gyroscope value threshold which means the device is stationary
		2. DT - time difference in seconds
	*/
	FusionBiasInitialise(&this->fusionBias, (int)(0.2f / imu_data.gyro_to_degs), DT);

	//Give Imu data to the fusion alg for initialization purposes
	while (FusionAhrsIsInitialising(&this->fusionAhrs) || FusionBiasIsActive(&this->fusionBias))
	{
		read_transform_imu();
		
		this->updateFusion();
		rc_usleep(DT_US);
	}
	this->isInitializingFusion = false;
}

/************************************************************************
*			Allocatate memeory for the Tranformation Matrices
************************************************************************/
void imu::initializeRotationMatrices()
{
	float pitch_offset, roll_offset, yaw_offset;
	int i,j;
	
	rc_alloc_matrix(&this->transform.IMU_to_drone,3,3);
	
	rc_alloc_vector(&this->transform.gyro_imu,3);
	rc_alloc_vector(&this->transform.accel_imu,3);
	rc_alloc_vector(&this->transform.mag_imu,3);
	rc_alloc_vector(&this->transform.gyro_drone,3);
	rc_alloc_vector(&this->transform.accel_drone,3);
	rc_alloc_vector(&this->transform.mag_drone,3);

	//TODO: Initialize all of the rotation matrices using Eigens

	//pitch_offset = 0; roll_offset = M_PI; yaw_offset = - M_PI;
	// pitch_offset = flight_config->pitch_offset_deg*DEG_TO_RAD; 
	// roll_offset = flight_config->roll_offset_deg*DEG_TO_RAD;
	// yaw_offset = flight_config->yaw_offset_deg*DEG_TO_RAD;
	
	// float ROTATION_MAT1[][3] = ROTATION_MATRIX1;
	// for(i=0; i<3; i++){
	// 	for(j=0; j<3; j++){
	// 		this->transform.IMU_to_drone.d[i][j]=ROTATION_MAT1[i][j];
	// 	}
	// }
}

/************************************************************************
*         Update the Fusion Algorithm, Called once per IMU Update       *
************************************************************************/
void imu::updateFusion()
{

	FusionVector3 gyroscope;
	FusionVector3 accelerometer; 
	FusionVector3 magnetometer;

	int i;
	for (i = 0; i < 3; i++)
	{
		gyroscope.array[i] = this->transform.gyro_drone.d[i];
		accelerometer.array[i] = this->transform.accel_drone.d[i] / 9.81f;
		magnetometer.array[i] = this->transform.mag_drone.d[i];
		if (!this->isInitializingFusion)
			gyroscope.array[i] -= this->fusionBias.gyroscopeBias.array[i] * imu_data.gyro_to_degs;
	}

	FusionBiasUpdate(&this->fusionBias, imu_data.raw_gyro[0], imu_data.raw_gyro[1], imu_data.raw_gyro[2]);
	FusionAhrsUpdate(&this->fusionAhrs, gyroscope, accelerometer, magnetometer, DT);																								
	this->eulerAngles = FusionQuaternionToEulerAngles(this->fusionAhrs.quaternion);
	
	//We want pitch and roll to be relative to the Drone's Local Coordinate Frame, not the NED Frame
	//	make that conversion here
	float mag, theta;
	mag = powf(powf(this->eulerAngles.angle.pitch,2.0f)+powf(this->eulerAngles.angle.roll,2.0f),0.5f);
	theta = atan2f(this->eulerAngles.angle.roll, this->eulerAngles.angle.pitch);
	
	this->eulerAngles.angle.roll = mag * sinf(theta-this->eulerAngles.angle.yaw*DEG_TO_RAD);
	this->eulerAngles.angle.pitch = mag * cosf(theta-this->eulerAngles.angle.yaw*DEG_TO_RAD);

	for (i = 0; i < 3; i++)
		this->eulerAngles.array[i]*=-1.0f;
}