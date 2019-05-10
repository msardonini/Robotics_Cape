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
#include "filter.h"

//Ours
#include "src/common.hpp"
#include "src/logger.hpp"
#include "src/gps.hpp"
#include "src/config.hpp"
#include "src/imu.hpp"
#include "src/pruClient.hpp"
#include "src/setpoint.hpp"
#include "src/ekf.hpp"



class flyMS {

 public:

  flyMS(flyMSParams &configModule);

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
  std::thread flightcoreThread;
  std::mutex flightcoreMutex;

  //Boolean for running in debug mode
  bool firstIteration;

  //Object for managing the user configurable parameters
  flyMSParams &configModule;
  config_t config;

  //Class to handle and write to the log file
  logger loggingModule;

  //Object and Data struct from the imu manager
  imu imuModule;
  state_t imuData;

  //Classes for all the functions of the program
  pruClient pruClientSender;

  //Object and Data struct from the setpoint manager
  setpoint setpointModule;
  setpoint_t setpointData;

  //Object and Data struct from the gps manager
  gps gpsModule;
  GPS_data_t gpsData;

  // Object and Data struct from the ekf manager
  ekf2 ekfModule;
  ekf_filter_t ekfData;

  //Struct for the control inputs
  controller_t control;
  filters_t filters;



  int integrator_reset;
  int integrator_start;


};



#endif // FLYMS_H