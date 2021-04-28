#ifndef SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_
#define SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"

/**
 * @brief      The flight mode
 */
enum class SetpointMode {
  Undefined = 0,
  Stabilized = 1,
  Acro = 2,
  Navigation = 3
};

struct vio_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf quat;
};

typedef struct setpoint_t {
  float euler_ref[3];  // Reference (Desired) Position
  float euler_ref_previous[3];  // Reference (Desired) Position
  float yaw_rate_ref[2];
  float Aux[2];
  double lat_setpoint;
  double lon_setpoint;      // Controller Variables for Autonomous Flight
  float altitudeSetpointRate;
  float altitudeSetpoint;
  float dpitch_setpoint; // Desired attitude
  float droll_setpoint;  // Desired attitude
  float throttle;
  float yaw_ref_offset;
  float kill_switch[2];
} setpoint_t;

typedef struct state_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t timestamp_us;

  Eigen::Vector3f euler;  // Euler angles of aircraft (in roll, pitch, yaw)
  // 1 Timestampe previousEuler angles of aircraft (in roll, pitch, yaw)
  Eigen::Vector3f eulerPrevious;
  Eigen::Vector3f eulerRate;  // First derivative of euler angles (in roll/s, pitch/s, yaw/s)

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  Eigen::Vector3f mag;

  float barometerAltitude;
  float compassHeading;

  // int    num_wraps;        // Number of spins in Yaw
  float  initialYaw;
} state_t;

#endif  // SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_
