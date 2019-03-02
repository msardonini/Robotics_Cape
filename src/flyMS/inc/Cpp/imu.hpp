/**
 * @file imu.hpp
 * @brief Source code to read data from the IMU and process it accordingly 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef IMU_H
#define IMU_H

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and 
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_8
#define BMP_CHECK_HZ	1
#define MICROTESLA_TO_GAUSS 0.01f
#define R2D_IMU 57.2958
#define D2R_IMU	0.01744f

//System Includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>

//Package Includes
#include <Eigen/Dense>
#include "Fusion.h"
#include <rc/mpu.h>
#include "ekf.hpp"
#include <rc/led.h>


//Ours
#include "config.hpp"
#include "logger.hpp"



class imu
{
public:

	imu(config_t _config, logger &loggingModule);

	//Default Descructor
	~imu();

	/************************************************************************
	*							Initialize the IMU                          *
	************************************************************************/
	int initializeImu();

	/************************************************************************
		imu_handler()
			Does all the parsing and interpretting of the IMU
			5 main tasks
				1. Reads the data from the IMU using Robotics_Cape API
				2. Performs a coordinate system transformation from imu -> drone
				3. Unwraps the yaw value for proper PID control
				4. Reads Barometer for altitude measurement
				5. Sends data to the EKF for position control

	************************************************************************/
	int update();




	/************************************************************************
	*					   Update the EKF with GPS Data                     *
	************************************************************************/
	int update_ekf_gps();

	/************************************************************************
					Get the Latest IMU data from the Object
	************************************************************************/
	int getImuData(state_t* state);


	void calculateDCM(float pitchOffsetDeg, float rollOffsetDeg, float yawOffsetDeg);
	
private:
	void init_fusion();
	void updateFusion();
	void read_transform_imu();

	//Variables to control the imu thread
	std::thread imuThread;
	std::mutex imuMutex;

	//Boolean to indicate if we are currently initializing the fusion algorithm
	bool isInitializingFusion;

	//Struct to hold all of the configurable parameters
	config_t config;

	ekf_filter_t ekfContainer;

	//Struct to keep all the state information of the aircraft in the body frame
	state_t stateBody;
	state_t stateIMU;

	//3x3 DCM for converting between imu and body frame
	Eigen::Matrix3f imu2Body;

	//Struct to get passed to the roboticsCape API for interfacing with the imu
	rc_mpu_data_t imu_data;

	//Struct to get passed to the roboticsCape API for interfacing with the bmp
	rc_bmp_data_t bmp_data;

	//Variables which control the Fusion of IMU data for Euler Angle estimation
	FusionVector3 gyroscope;
	FusionVector3 accelerometer;
	FusionVector3 magnetometer;
	FusionAhrs fusionAhrs;
	FusionEulerAngles eulerAngles;
	FusionBias fusionBias;

	//Mainly for flyMS_printf
	logger &loggingModule;
};

#endif //IMU_H