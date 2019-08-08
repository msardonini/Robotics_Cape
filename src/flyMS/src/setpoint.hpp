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

#include <atomic>
#include <thread>
#include <mutex>


//Package includes
#include<roboticscape.h>
// #include <flyMS.hpp>

//Ours
#include "src/common.hpp"
#include "src/config.hpp"
#include "src/logger.hpp"


typedef enum reference_mode_t {
  RC_INITIALIZATION,
  RC_DIRECT,
  RC_NAVIGATION

} reference_mode_t;


class setpoint {
 public:

  setpoint(config_t _config, logger& _loggingModule);

  //Default Destructor
  ~setpoint();

  bool getSetpointData(setpoint_t* _setpoint);


  int start();

  //Sets the init flag
  void setInitializationFlag(bool flag);

 private:

  int setpointManager();
  int copy_dsm2_data();
  int handle_rc_data_direct();
  int rc_err_handler(reference_mode_t setpoint_type);

  bool isInitializing;
  enum reference_mode_t setpoint_type;
  bool isReadyToParse;
  std::atomic <bool> isReadyToSend;
  float dsm2_data[MAX_DSM2_CHANNELS];
  int dsm2_timeout;

  //Variables to control multithreading
  std::thread setpointThread;
  std::mutex setpointMutex;

  //All relevant setpoint data goes here
  setpoint_t setpointData;

  config_t config;

  logger& loggingModule;

};



#endif // SETPOINT_H