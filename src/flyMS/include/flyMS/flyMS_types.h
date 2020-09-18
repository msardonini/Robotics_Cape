#ifndef SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_
#define SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_

#include "Eigen/Dense"

/**
 * @brief      The flight mode
 */
enum class SetpointMode {
  Undefined = 0,
  Stabilized = 1,
  Acro = 2,
  Navigation = 3
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
  uint64_t timestamp_us;
  uint32_t time_since_trigger_us;
  uint32_t trigger_count;

  Eigen::Vector3f euler;  // Euler angles of aircraft (in roll, pitch, yaw)
  // 1 Timestampe previousEuler angles of aircraft (in roll, pitch, yaw)
  Eigen::Vector3f eulerPrevious;
  Eigen::Vector3f eulerRate;  // First derivative of euler angles (in roll/s, pitch/s, yaw/s)

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  Eigen::Vector3f mag;

  float barometerAltitude;
  float compassHeading;

  int    num_wraps;        // Number of spins in Yaw
  float  initialYaw;
} state_t;

#endif  // SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_