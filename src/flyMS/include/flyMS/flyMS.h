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
#include <array>

#include "filter.h"
#include "yaml-cpp/yaml.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "flyMS/ulog/ulog.h"
#include "flyMS/gps.h"
#include "flyMS/imu/dmp.h"
#include "flyMS/pruClient.h"
#include "flyMS/setpoint.h"
#include "flyMS/types/state_data.h"
#include "flyMS/mavlink_interface.h"
#include "flyMS/position_generator.h"

class flyMS {
 public:
  flyMS(const YAML::Node &input_params);

  //Default Destructor
  ~flyMS();

  //Main thread which controls the inner loop FCS
  int FlightCore();

  //Initialize the system's hardware
  int StartupRoutine();

 private:
  //Get the current time in microseconds
  inline uint64_t GetTimeMicroseconds();

  int CheckOutputRange(std::array<float, 4> &u);

  int InitializeHardware();

  int InitializeFilters();

  void InitializeSpdlog(const std::string &log_dir);

  std::string GetLogDir(const std::string &log_location);

  int ConsolePrint();

  //Boolean for the status of the program
  std::thread flightcore_thread_;
  std::mutex flightcore_mutex_;

  // Flag for the first iteration
  bool first_iteration_;

  // Variables for working with flyStereo
  bool flyStereo_running_ = false;
  bool flyStereo_streaming_data_ = false;
  float standing_throttle_ = 0.0f;
  float initial_yaw_ = 0.0f;
  PositionGenerator position_generator_;

  // Counter for number of timestamps at min throttle, used to detect landing
  // and reset the integrators in the PID controllers
  int integrator_reset_;

  // Class to handle and write to the log file
  ULog ulog_;

  // Object and Data struct from the imu manager
  std::unique_ptr<ImuDmp> imu_module_;
  StateData imu_data_;

  // Classes for all the functions of the program
  pruClient pru_client_;

  // Object and Data struct from the setpoint manager
  setpoint setpoint_module_;
  SetpointData setpoint_;  // TODO Make this a non class member

  // Object and Data struct from the gps manager
  gps gps_module_;
  GPS_data_t gps_;

  // Object to handle the I/O on the serial port
  MavlinkInterface mavlink_interface_;

  // Struct for the control inputs
  digital_filter_t *roll_inner_PID_ = nullptr;
  digital_filter_t *roll_outer_PID_ = nullptr;
  digital_filter_t *pitch_outer_PID_ = nullptr;
  digital_filter_t *pitch_inner_PID_ = nullptr;
  digital_filter_t *yaw_PID_ = nullptr;
  digital_filter_t *gyro_lpf_[3] = {nullptr, nullptr, nullptr};

  // Containers for controller's output
  std::array<float, 4> u_euler_;
  std::array<float, 4> u_;

  // Configurable parameters
  int flight_mode_;
  std::array<float, 3> max_control_effort_;
  bool is_debug_mode_;
  std::string log_filepath_;
  float delta_t_;
  float min_throttle_;
  float pid_LPF_const_sec_;
  std::array<float, 3> roll_PID_inner_coeff_;
  std::array<float, 3> roll_PID_outer_coeff_;
  std::array<float, 3> pitch_PID_inner_coeff_;
  std::array<float, 3> pitch_PID_outer_coeff_;
  std::array<float, 3> yaw_PID_coeff_;
  std::vector<float> imu_lpf_num_;
  std::vector<float> imu_lpf_den_;

  YAML::Node config_params_;
};

#endif // FLYMS_H
