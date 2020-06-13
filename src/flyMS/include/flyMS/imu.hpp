/**
 * @file imu.hpp
 * @brief Source code to read data from the IMU and process it accordingly
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef IMU_H
#define IMU_H

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER  BMP_FILTER_8
#define BMP_CHECK_HZ  1

//System Includes
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>

//Package Includes
#include <Eigen/Dense>
#include "Fusion.h"
#include <rc/mpu.h>
#include "ekf.hpp"
#include <rc/led.h>

//Ours
#include "flyMS/config.hpp"
#include "flyMS/logger.hpp"



/**
 * @brief      This class describes a position/velocity/acceletaion state.
 *
 * @tparam     T     The data type used to store data
 */
template <typename T> class pva_state {
 public:
  pva_state(int history_len) :
    index_(0),
    data(history_len) {}

    Eigen::Matrix<T, 3, 1> get_position() const {
      Eigen::Matrix<T, 3, 1> return_mat;
      return_mat << data[index_](0), data[index_](1), data[index_](2);
    }

    Eigen::Matrix<T, 3, 1> get_velocity() const {
      Eigen::Matrix<T, 3, 1> return_mat;
      return_mat << data[index_](3), data[index_](4), data[index_](5);
    }

    Eigen::Matrix<T, 3, 1> get_acceleration() const {
      Eigen::Matrix<T, 3, 1> return_mat;
      return_mat << data[index_](7), data[index_](8), data[index_](9);
    }

    void set_pva(const T pos[3], const T vel[3], const T acc[3]) {
      index_ = ++index_ % data.size();

      data[index] << pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], acc[0],
        acc[1], acc[2];
    }

    void set_pva(const T pva[9]) {
      data[index] << pva[0], pva[1], pva[2], pva[3], pva[4], pva[5], pva[6], pva[7], pva[8];
    }

    void set_pva(const Eigen::Matrix<T, 3, 1> &pos,
      const Eigen::Matrix<T, 3, 1> &vel,
      const Eigen::Matrix<T, 3, 1> &acc) {
      data(index) << pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), acc(0),
        acc(1), acc(2);
    }

    void set_pva(const Eigen::Matrix<T, 9, 1> &pva) {
      data(index) << pva(0), pva(1), pva(2), pva(3), pva(4), pva(5), pva(6), pva(7), pva(8);
    }


 private:
  int index_;
  std::vector<Eigen::Matrix<T, 9, 1> > data; 
};


class imu {
 public:

  imu(config_t _config, logger &loggingModule);

  //Default Descructor
  ~imu();

  /************************************************************************
  *              Initialize the IMU                          *
  ************************************************************************/
  int initializeImu();

  /************************************************************************
    imu_handler()
      Does all the parsing and interpretting of the IMU
      5 main tasks
        1. Reads the data from the IMU using Robotics_Cape API
        2. Performs a coordinate system transformation from imu -> drone
        3. Unwraps the yaw value for proper PID control
        4. Reads Barometer for altitude measurement
        5. Sends data to the EKF for position control

  ************************************************************************/
  int update();




  /************************************************************************
  *             Update the EKF with GPS Data                     *
  ************************************************************************/
  int update_ekf_gps();

  /************************************************************************
          Get the Latest IMU data from the Object
  ************************************************************************/
  int getImuData(state_t* state);


  void calculateDCM(float pitchOffsetDeg, float rollOffsetDeg, float yawOffsetDeg);

 private:
  void GpioThread();
  void init_fusion();
  void updateFusion();
  void read_transform_imu();
  void send_mavlink();

  std::atomic<bool> is_running_;

  //Variables to control the imu thread
  std::mutex gpio_mutex_;
  std::thread gpio_thread_;
  std::thread imu_thread_;
  std::mutex imu_mutex_;

  //Boolean to indicate if we are currently initializing the fusion algorithm
  bool is_initializing_fusion_;

  //Struct to hold all of the configurable parameters
  config_t config_;

  ekf_filter_t ekf_container_;

  //Struct to keep all the state information of the aircraft in the body frame
  state_t state_body_;
  state_t state_imu_;

  //3x3 DCM for converting between imu and body frame
  Eigen::Matrix3f imu_to_body_;

  //Struct to get passed to the roboticsCape API for interfacing with the imu
  rc_mpu_data_t imu_data_;

  //Struct to get passed to the roboticsCape API for interfacing with the bmp
  rc_bmp_data_t bmp_data_;

  //Variables which control the Fusion of IMU data for Euler Angle estimation
  FusionAhrs fusion_ahrs_;
  FusionEulerAngles euler_angles_;
  FusionBias fusion_bias_;

  //Mainly for flyMS_printf
  logger &logger_;
};

#endif //IMU_H
