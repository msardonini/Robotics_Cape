/**
 * @file setpoint.hpp
 * @brief Class for controlling the input reference angle to the inner loop controller 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#ifndef SETPOINT_H
#define SETPOINT_H


//System includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#include<thread>
#include<mutex>


//Package includes
#include<roboticscape.h>
// #include <flyMS.hpp>

//Ours
#include "config.hpp"

#define DT 0.005
#define DT_US 5000
#define MAX_DSM2_CHANNELS 8
#define MAX_PITCH_RANGE 0.666 // in radians
#define MAX_ROLL_RANGE 0.666 // in radians
#define MAX_YAW_RATE 2.0 //in Radians per second
#define MIN_THROTTLE 0.3
#define MAX_THROTTLE 0.75

typedef struct setpoint_t{
	float	euler_ref[3];	// Reference (Desired) Position
	float	euler_ref_previous[3];	// Reference (Desired) Position
	float	yaw_rate_ref[2];
	float	Aux[2];
	double	lat_setpoint;
	double 	lon_setpoint;			// Controller Variables for Autonomous Flight
	float	altitudeSetpointRate;
	float	altitudeSetpoint;
	float	dpitch_setpoint; // Desired attitude
	float	droll_setpoint;	// Desired attitude
	float 	throttle;				
	float	yaw_ref_offset;
	float 	kill_switch[2];
}setpoint_t;


typedef enum reference_mode_t
{
	RC_DIRECT,
	RC_NAVIGATION

}reference_mode_t;


class setpoint
{
public:

	//Default Constructor
	setpoint(flyMSParams _config);

	//Default Destructor
	~setpoint();
	
	void getSetpointData(setpoint_t* _setpoint);

	int setpointManager();

private:

	int copy_dsm2_data();
	int handle_rc_data_direct();
	int rc_err_handler(reference_mode_t setpoint_type);

	uint8_t new_dsm_data = 0;
	float dsm2_data[MAX_DSM2_CHANNELS];
	int dsm2_timeout = 0;

	//Variables to control multithreading
	std::thread setpointThread;
	std::mutex setpointMutex;

	//All relevant setpoint data goes here
	setpoint_t setpointData;

	flyMSParams config;

};



#endif // SETPOINT_H