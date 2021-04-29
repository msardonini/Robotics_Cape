#ifndef SRC_FLYMS_INCLUDE_FLYMS_TYPES_FLIGHT_FILTERS_H_
#define SRC_FLYMS_INCLUDE_FLYMS_TYPES_FLIGHT_FILTERS_H_

#include <array>
#include <vector>

#include "filter.h"

struct FlightFilters {
  FlightFilters() = delete;  // Forbid the default constructor

  FlightFilters(std::array<float, 3> roll_PID_inner_coeff,
    std::array<float, 3> roll_PID_outer_coeff,
    std::array<float, 3> pitch_PID_inner_coeff,
    std::array<float, 3> pitch_PID_outer_coeff,
    std::array<float, 3> yaw_PID_coeff,
    std::vector<float> imu_lpf_num,
    std::vector<float> imu_lpf_den,
    float dt,
    float lowpass_filter_const) :
    roll_inner_PID(generatePID(roll_PID_inner_coeff[0], roll_PID_inner_coeff[0],
      roll_PID_inner_coeff[0], lowpass_filter_const, dt)),
    roll_outer_PID(generatePID(roll_PID_outer_coeff[0], roll_PID_outer_coeff[1],
      roll_PID_outer_coeff[2], lowpass_filter_const, dt)),
    pitch_inner_PID(generatePID(pitch_PID_inner_coeff[0], pitch_PID_inner_coeff[1],
      pitch_PID_inner_coeff[2], lowpass_filter_const, dt)),
    pitch_outer_PID(generatePID(pitch_PID_outer_coeff[0], pitch_PID_outer_coeff[1],
      pitch_PID_outer_coeff[2], lowpass_filter_const, dt)),
    yaw_PID(generatePID(yaw_PID_coeff[0], yaw_PID_coeff[1], yaw_PID_coeff[2],
      lowpass_filter_const, dt)),
    gyro_lpf{initialize_filter(imu_lpf_num.size(), imu_lpf_num.data(), imu_lpf_den.data()),
      initialize_filter(imu_lpf_num.size(), imu_lpf_num.data(), imu_lpf_den.data()),
      initialize_filter(imu_lpf_num.size(), imu_lpf_num.data(), imu_lpf_den.data())} {}

  ~FlightFilters() {
    free(roll_inner_PID);
    free(roll_outer_PID);
    free(pitch_inner_PID);
    free(pitch_outer_PID);
    free(yaw_PID);
    free(gyro_lpf[0]);
    free(gyro_lpf[1]);
    free(gyro_lpf[2]);
  }

  void ZeroPIDs() {
    zeroFilter(roll_inner_PID);
    zeroFilter(roll_outer_PID);
    zeroFilter(pitch_inner_PID);
    zeroFilter(pitch_outer_PID);
    zeroFilter(yaw_PID);
  }


  digital_filter_t *roll_inner_PID = nullptr;
  digital_filter_t *roll_outer_PID = nullptr;
  digital_filter_t *pitch_inner_PID = nullptr;
  digital_filter_t *pitch_outer_PID = nullptr;
  digital_filter_t *yaw_PID = nullptr;
  digital_filter_t *gyro_lpf[3] = {nullptr, nullptr, nullptr};

};

#endif  // SRC_FLYMS_INCLUDE_FLYMS_TYPES_FLIGHT_FILTERS_H_
