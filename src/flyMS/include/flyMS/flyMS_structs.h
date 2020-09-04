#ifndef SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_
#define SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_

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

#endif  // SRC_FLYMS_INCLUDE_FLYMS_FLYMS_STRUCTS_H_