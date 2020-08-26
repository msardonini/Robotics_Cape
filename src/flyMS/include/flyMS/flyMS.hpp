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
#include "yaml-cpp/yaml.h"
#include "flyMS/ulog/ulog.h"
#include "flyMS/common.hpp"
#include "flyMS/gps.hpp"
#include "flyMS/imu.hpp"
#include "flyMS/pruClient.hpp"
#include "flyMS/setpoint.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

typedef struct controller_t {
  float  droll_err_integrator;
  float  dpitch_err_integrator;
  float  dyaw_err_integrator;
  float   u_euler[3];          // Controller output for roll, pitch, yaw
  float  u[4];                 // Duty Cycle to send to each motor
  float  standing_throttle, alt_error;
} controller_t;

typedef struct filters_t {
  digital_filter_t *pitch_rate_PD;
  digital_filter_t *roll_rate_PD;
  digital_filter_t *yaw_rate_PD;
  digital_filter_t *pitch_PD;
  digital_filter_t *roll_PD;
  digital_filter_t *yaw_PD;
  digital_filter_t *LPF_d_pitch;
  digital_filter_t *LPF_d_roll;
  digital_filter_t *LPF_d_yaw;
  digital_filter_t *LPF_Yaw_Ref_P;
  digital_filter_t *LPF_Yaw_Ref_R;
  digital_filter_t *Outer_Loop_TF_pitch;
  digital_filter_t *Outer_Loop_TF_roll;
  digital_filter_t *LPF_Accel_Lat;
  digital_filter_t *LPF_Accel_Lon;
  digital_filter_t *LPF_pitch;
  digital_filter_t *LPF_roll;
  digital_filter_t *altitudeHoldPID;
  digital_filter_t *LPF_baro_alt;

  digital_filter_t *gyro_lpf[3];
  digital_filter_t *accel_lpf[3];

} filters_t;

class flyMS {

 public:

  flyMS(const YAML::Node &input_params);

  //Default Destructor
  ~flyMS();

  //Main thread which controls the inner loop FCS
  int flightCore();

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

  int InitializeSpdlog(const std::string &log_dir);

  std::string GetLogDir(const std::string &log_location);

  //Boolean for the status of the program
  bool isRunning;
  std::thread flightcoreThread;
  std::mutex flightcoreMutex;

  //Boolean for running in debug mode
  bool firstIteration;

  //Class to handle and write to the log file
  ULog ulog_;

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

  //Struct for the control inputs
  controller_t control;
  filters_t filters;

  int integrator_reset;
  int integrator_start;

  // Configurable parameters
  int flight_mode_;
  std::array<float, 3> max_control_effort_;
  bool is_debug_mode_;
  std::string log_filepath_;
  float delta_t_;
  std::array<float, 3> roll_PID_inner_;
  std::array<float, 3> roll_PID_outer_;
  std::array<float, 3> pitch_PID_inner_;
  std::array<float, 3> pitch_PID_outer_;
  std::array<float, 3> yaw_PID_;
};

#endif // FLYMS_H
