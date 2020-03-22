/**
 * @file ekf.hpp
 * @brief Source code to interface with the PX4 EKF & position estimator
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef EKF_HPP
#define EKF_HPP


//System includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>
#include <thread>
#include <mutex>

//Package includes
#include "roboticscape.h"

//Ours
#include "ekf.h"
#include "flyMS/common.hpp"



typedef struct ekf_filter_input_t {
  uint64_t IMU_timestamp;
  float mag[3];
  float gyro[3];
  float accel[3];

  uint64_t barometer_timestamp;
  uint8_t barometer_updated;
  float barometer_alt;

  uint8_t gps_updated;
  uint64_t gps_timestamp;
  double gps_latlon[3];
  uint8_t gps_fix;
  uint8_t nsats;

  uint8_t vehicle_land;

} ekf_filter_input_t;


typedef struct ekf_filter_output_t {
  double ned_pos[3];
  double ned_vel[3];
  double ned_acc[3];

  float vertical_time_deriv;
  float gyro[3];

} ekf_filter_output_t;

typedef struct ekf_filter_t {
  ekf_filter_input_t input;
  ekf_filter_output_t output;

} ekf_filter_t;



class ekf2 {

 public:
  ekf2();
  ~ekf2();

  int setInputs(ekf_filter_t *inputFilter);
  int startEkf();

 private:
  Ekf _ekf;

  int runEkf();
  std::thread ekfThread;
  std::mutex ekfMutex;

  ekf_filter_t ekf_filter;

};


#endif //EKF_HPP