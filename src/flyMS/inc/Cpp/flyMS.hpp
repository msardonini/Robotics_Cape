/**
 * @file flyMS.cpp
 * @brief flyMS program source code. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef FLYMS_H
#define FLYMS_H

#define MAX_PITCH_COMPONENT 0.25
#define MAX_ROLL_COMPONENT 0.25
#define MIN_THROTTLE 0.3
#define MAX_THROTTLE 0.75
#define MAX_YAW_COMPONENT 0.25
#define DT_US 5000

//System Includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>

//Package Includes
#include "filter.h"

//Ours
#include "gps.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "pruClient.hpp"
#include "setpoint.hpp"

typedef struct controller_t{
	float	droll_err_integrator;
	float	dpitch_err_integrator;
	float	dyaw_err_integrator;
	float 	u_euler[3];					// Controller output for roll, pitch, yaw
	float	u[4]; 								// Duty Cycle to send to each motor
	float	standing_throttle, alt_error;
}controller_t;

typedef struct filters_t{
	digital_filter_t			*pitch_rate_PD;
	digital_filter_t			*roll_rate_PD;
	digital_filter_t			*yaw_rate_PD;
	digital_filter_t			*pitch_PD;
	digital_filter_t			*roll_PD;
	digital_filter_t			*yaw_PD;
	digital_filter_t         	*LPF_d_pitch;
	digital_filter_t         	*LPF_d_roll;
	digital_filter_t         	*LPF_d_yaw;
	digital_filter_t         	*LPF_Yaw_Ref_P;
	digital_filter_t        	*LPF_Yaw_Ref_R;
	digital_filter_t			*Outer_Loop_TF_pitch;
	digital_filter_t			*Outer_Loop_TF_roll;
	digital_filter_t 			*LPF_Accel_Lat;
	digital_filter_t 			*LPF_Accel_Lon;
	digital_filter_t			*LPF_pitch;
	digital_filter_t			*LPF_roll;	
	digital_filter_t			*altitudeHoldPID;
	digital_filter_t			*LPF_baro_alt;

	digital_filter_t *gyro_lpf[3];
	digital_filter_t *accel_lpf[3];

}filters_t;



class flyMS
{

public:
	// Default Constructor
	flyMS(bool _debugMode);

	//Default Destructor
	~flyMS();

	//Main thread which controls the inner loop FCS
	int	flightCore();

	//Initialize the system's hardware
	int startupRoutine(); 

	int check_output_range(float u[4]);

	int console_print();

	int initializeHardware();

private:

	//Get the current time in microseconds
	uint64_t getTimeMicroseconds();

	//Get input from the user to start the flight program
	int readyCheck();

	int initializeFilters();

	//Boolean for the status of the program
	bool isRunning;

	//Boolean for running in debug mode
	bool isDebugMode;
	bool firstIteration;

	//Classes for all the functions of the program
	pruClient pruClientSender;
	
	//Object and Data struct from the gps manager
	gps gpsModule;
	GPS_data_t gpsData;

	//Object and Data struct from the imu manager
	imu imuModule;
	state_t imuData;

	//Object and Data struct from the setpoint manager
	setpoint setpointModule;
	setpoint_t setpointData;

	//Object and Data struct from the ekf manager
	ekf2 ekfModule;
	ekf_filter_t ekfData;

	//Struct for the control inputs
	controller_t control;
	filters_t filter;

	flyMSParams config;

	int integrator_reset;
	int integrator_start;

	//TODO: Make classes for each of these
	// loggingManager logger;

};



#endif // FLYMS_H