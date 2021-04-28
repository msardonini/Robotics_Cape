#ifndef SRC_FLYMS_INCLUDE_FLYMS_TYPES_STATE_DATA_H_
#define SRC_FLYMS_INCLUDE_FLYMS_TYPES_STATE_DATA_H_

#include "Eigen/Dense"

struct StateData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t timestamp_us;

  Eigen::Vector3f euler;  // Euler angles of aircraft (in roll, pitch, yaw)
  Eigen::Vector3f eulerPrevious;
  Eigen::Vector3f eulerRate;  // First derivative of euler angles (in roll/s, pitch/s, yaw/s)

  Eigen::Vector3f accel;  // Raw data from accelerometer
  Eigen::Vector3f gyro;  // Raw data from gyroscope
  Eigen::Vector3f mag;  // Raw data from magnetometer

  float barometerAltitude;
  float compassHeading;

  // int    num_wraps;        // Number of spins in Yaw
  float  initialYaw;
};

#endif  // SRC_FLYMS_INCLUDE_FLYMS_TYPES_STATE_DATA_H_
