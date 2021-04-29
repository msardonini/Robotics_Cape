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
#include "yaml-cpp/yaml.h"
#include "rc/dsm.h"
#include "flyMS/position_controller.h"

struct SetpointData {
  float euler_ref[3];  // Reference (Desired) Position in Roll, Pitch, Yaw
  float yaw_rate_ref[2];  // Reference Yaw Rate, current (ind 0) and previous
  float Aux[2];  // The Aux channel value, current (ind 0) and previous
  float throttle;
  float kill_switch[2];  // Current (ind 0) and previous (ind 1) value of the kill switch channel
};


class Setpoint {
 public:

  Setpoint(const YAML::Node &config_params);

  //Default Destructor
  ~Setpoint();

  /**
   * @brief      Gets the setpoint data.
   *
   * @param      setpoint  The setpoint data
   *
   * @return     The setpoint data.
   */
  bool GetSetpointData(SetpointData* setpoint);

  /**
   * @brief Set the Yaw Setpoint value to a user defined offset
   *
   * @param ref
   */
  void SetYawRef(float ref);

  /**
   * @brief Get the Min Throttle
   *
   * @return float
   */
  float GetMinThrottle() const {
    return throttle_limits_[0];
  }

  std::unique_ptr<PositionController> position_controller;
 private:
  int SetpointManager();
  int HandleRcData(std::array<float, RC_MAX_DSM_CHANNELS> dsm2_data);
  int RcErrHandler();

  std::atomic<bool> is_running_;  // Flag to indicate if the object is running vs shutting down

  std::atomic <bool> ready_to_send_;
  std::array<float, RC_MAX_DSM_CHANNELS> dsm2_data_;
  int dsm2_timeout_;

  //Variables to control multithreading
  std::thread setpoint_thread_;
  std::mutex setpoint_mutex_;

  //All relevant setpoint data goes here
  SetpointData setpoint_data_;

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