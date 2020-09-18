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
#include <memory>

//Package includes
#include <roboticscape.h>
#include "yaml-cpp/yaml.h"
#include "flyMS/position_controller.h"
#include "flyMS/flyMS_types.h"

class setpoint {
 public:

  setpoint(const YAML::Node &config_params);

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

  int SetYawRef(float ref);

 private:
  int SetpointManager();
  int HandleRcData();
  int RcErrHandler();

  std::unique_ptr<PositionController> position_controller_;

  enum SetpointMode setpoint_mode_;
  std::atomic <bool> ready_to_send_;
  float dsm2_data_[RC_MAX_DSM_CHANNELS];
  int dsm2_timeout_;

  //Variables to control multithreading
  std::thread setpoint_thread_;
  std::mutex setpoint_mutex_;

  //All relevant setpoint data goes here
  setpoint_t setpoint_data_;

  // All configurable parameters
  bool is_debug_mode_;
  std::array<float, 3> max_setpoints_stabilized_;
  std::array<float, 3> max_setpoints_acro_;
  std::array<float, 2> throttle_limits_;
  bool is_headless_mode_;
  int flight_mode_;
  float delta_t_;
};



#endif // SETPOINT_H