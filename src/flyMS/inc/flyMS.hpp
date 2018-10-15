/**
 * @file flyMS.cpp
 * @brief flyMS program source code. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef FLYMS_H
#define FLYMS_H


//System Includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>

//Package Includes

//Ours
#include "flyMS_common.h"


class flyMS
{

public:
	
	// Default Constructor
	flyMS(bool _debugMode);

	//Default Destructor
	~flyMS();

	int	flight_core();

private:

	//Get the current time in microseconds
	uint64_t getTimeMicroseconds(struct timespec *tv);

	//Boolean for the status of the program
	bool isRunning;

	//Boolean for running in debug mode
	bool isDebugMode;

	pruClient pruClientSender;

	//TODO: Make classes for each of these
	setpointManager setpoint;
	px4EkfManager ekf;
	loggingManager logger;


};



#endif // FLYMS_H