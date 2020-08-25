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
#include "flyMS/common.hpp"
#include "flyMS/config.hpp"
#include "flyMS/logger.hpp"


/**
 * @brief      The flight mode
 */
enum class SetpointMode {
  Undefined = 0,
  Stabilized = 1,
  Acro = 2,
  Navigation = 3
};


class setpoint {
 public:

  setpoint(config_t _config, logger& _loggingModule);

  //Default Destructor
  ~setpoint();

  /**
   * @brief      Gets the setpoint data.
   *
   * @param      setpoint  The setpoint data
   *
   * @return     The setpoint data.
   */
  bool getSetpointData(setpoint_t* setpoint);


  /**
   * @brief      Initializes the hardware for the RC and starts managing threads
   *
   * @return     0 on success, -1 on failure
   */
  int start();

  //Sets the init flag
  void setInitializationFlag(bool flag);

 private:
  int SetpointManager();
  int HandleRcData();
  int RcErrHandler();

  bool is_initializing_;
  enum SetpointMode setpoint_mode_;
  std::atomic <bool> ready_to_send_;
  float dsm2_data_[RC_MAX_DSM_CHANNELS];
  int dsm2_timeout_;

  //Variables to control multithreading
  std::thread setpoint_thread_;
  std::mutex setpoint_mutex_;

  //All relevant setpoint data goes here
  setpoint_t setpoint_data_;

  config_t config_;

  logger& logging_module_;

};



#endif // SETPOINT_H