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
#include "flyMS/ulog/ulog.h"
#include "flyMS/gps.h"
#include "flyMS/imu.h"
#include "flyMS/pruClient.h"
#include "flyMS/setpoint.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

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

  //Get input from the user to start the flight program
  int ReadyCheck();

  int InitializeFilters();

  int InitializeSpdlog(const std::string &log_dir);

  std::string GetLogDir(const std::string &log_location);

  int ConsolePrint();

  //Boolean for the status of the program
  std::thread flightcore_thread_;
  std::mutex flightcore_mutex_;

  // Flag for the first iteration
  bool first_iteration_;

  // Counter for number of timestamps at min throttle, used to detect landing
  // and reset the integrators in the PID controllers
  int integrator_reset_;

  // Class to handle and write to the log file
  ULog ulog_;

  // Object and Data struct from the imu manager
  imu imu_module_;
  state_t imu_data_;

  // Classes for all the functions of the program
  pruClient pru_client_;

  // Object and Data struct from the setpoint manager
  setpoint setpoint_module_;
  setpoint_t setpoint_;

  // Object and Data struct from the gps manager
  gps gps_module_;
  GPS_data_t gps_;

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
  std::array<float, 3> roll_PID_inner_coeff_;
  std::array<float, 3> roll_PID_outer_coeff_;
  std::array<float, 3> pitch_PID_inner_coeff_;
  std::array<float, 3> pitch_PID_outer_coeff_;
  std::array<float, 3> yaw_PID_coeff_;
  std::vector<float> imu_lpf_num_;
  std::vector<float> imu_lpf_den_;
};

#endif // FLYMS_H
