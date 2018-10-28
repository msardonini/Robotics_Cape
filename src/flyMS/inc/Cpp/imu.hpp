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
#define DEG_TO_RAD	0.01744
#define MICROTESLA_TO_GAUSS 0.01f
#define DT_US 5000
#define D2R 0.0174533
#define R2D 57.2958

//System Includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>

//Package Includes
#include <Eigen/Dense>
#include "Fusion.h"
#include "roboticscape.h"
#include "filter.h"

//Ours
#include "ekf.hpp"
#include "config.hpp"


typedef struct state_t{
	Eigen::Vector3f	euler;					// Euler angles of aircraft (in roll, pitch, yaw)
	Eigen::Vector3f eulerPrevious;			// 1 Timestampe previousEuler angles of aircraft (in roll, pitch, yaw)
	Eigen::Vector3f	eulerRate;				// First derivative of euler angles (in roll/s, pitch/s, yaw/s)
	
	Eigen::Vector3f accel;
	Eigen::Vector3f gyro;
	Eigen::Vector3f mag;

	float barometerAltitude;
	float compassHeading;

	int		num_wraps;				// Number of spins in Yaw
	float	initialYaw;
}state_t;


class imu
{
public:
	
	//Default Constructor
	imu(flyMSParams _config);

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



private:
	void init_fusion();
	void updateFusion();
	void read_transform_imu();
	void initializeRotationMatrices();

	//Variables to control the imu thread
	std::thread imuThread;
	std::mutex imuMutex;

	//Boolean to indicate if we are currently initializing the fusion algorithm
	bool isInitializingFusion;

	//Class to hold all of the configurable parameters
	flyMSParams config;

	ekf_filter_t ekfContainer;

	//Struct to keep all the state information of the aircraft in the body frame
	state_t stateBody;
	state_t stateIMU;

	//3x3 DCM for converting between imu and body frame
	Eigen::Matrix3f imu2Body;

	//Struct to get passed to the roboticsCape API for interfacing with the imu
	rc_imu_data_t imu_data;


	//Variables which control the Fusion of IMU data for Euler Angle estimation
	FusionVector3 gyroscope;
	FusionVector3 accelerometer;
	FusionVector3 magnetometer;
	FusionAhrs fusionAhrs;
	FusionEulerAngles eulerAngles;
	FusionBias fusionBias;


};

#endif //IMU_H