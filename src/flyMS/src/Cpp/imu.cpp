 /**
 * @file imu.cpp
 * @brief Source code to read data from the IMU and process it accordingly 
 *
 * @author Mike Sardonini
 * @date 10/16/2018
 */

#include "imu.hpp"


// Default Constructor
imu::imu(flyMSParams _config) :
	isInitializingFusion(true),
	config(_config)
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
	if(this->config.enableBarometer)
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

int imu::getImuData(state_t* state)
{
	memcpy(state, &this->stateBody, sizeof(state_t));
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
	if (this->config.enableBarometer)
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
		//TODO: enable the EKF again

	// int i;
	// for (i = 0; i < 3; i++)
	// {
	// 	// this->ekfContainer.input.accel[i] = this->transform.accel_drone.d[i];
	// 	// this->ekfContainer.input.mag[i] = imu_data.mag[i] * MICROTESLA_TO_GAUSS;
	// 	// this->ekfContainer.input.gyro[i] = this->stateBody.eulerRate[i];
	// }
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
		this->stateIMU.mag(i) = imu_data.mag[i];
		this->stateIMU.gyro(i) = imu_data.gyro[i];
		this->stateIMU.accel(i) = imu_data.accel[i];
	}

	//Convert from IMU frame to body Frame
	this->stateBody.mag = this->imu2Body * this->stateIMU.mag;
	this->stateBody.gyro = this->imu2Body * this->stateIMU.gyro;
	this->stateBody.accel = this->imu2Body * this->stateIMU.accel;
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
	//Make the Direcion Cosine Matric DCM from the input offsets from the config file
	float cR1 = cosf(this->config.rollOffsetDegrees * D2R);
	float sR1 = sinf(this->config.rollOffsetDegrees * D2R);
	float cP1 = cosf(this->config.pitchOffsetDegrees * D2R);
	float sP1 = sinf(this->config.pitchOffsetDegrees * D2R);
	float cY1 = cosf(this->config.yawOffsetDegrees * D2R);
	float sY1 = sinf(this->config.yawOffsetDegrees * D2R);

	this->imu2Body << cR1*cY1, -cP1*sY1+sP1*sR1*cY1 ,  sP1*sY1+cP1*sR1*cY1
				, cR1*sY1 ,  cP1*cY1+sP1*sR1*sY1 , -sP1*cY1+cP1*sR1*sY1
				, -sR1 , sP1*cR1 , cP1*cR1;

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
		gyroscope.array[i] = this->stateBody.gyro(i);
		accelerometer.array[i] = this->stateBody.accel(i) / 9.81f;
		magnetometer.array[i] = this->stateBody.mag(i);
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

	//Save the output
	for (i = 0; i < 3; i++)
	{
		this->stateBody.eulerPrevious(i) 		= this->stateBody.euler(i);	
		this->stateBody.euler(i)				= this->eulerAngles.array[i] * D2R;
		this->stateBody.eulerRate(i)			= this->stateBody.gyro(i) * D2R;
	}

}