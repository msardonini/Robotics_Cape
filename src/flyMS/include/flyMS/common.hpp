/**
 * @file common.hpp
 * @brief Common structs used within the flyMS flight program
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#ifndef SRC_FLYMS_INCLUDE_FLYMS_COMMON_H_
#define SRC_FLYMS_INCLUDE_FLYMS_COMMON_H_

#include "Eigen/Dense"
#include "filter.h"

constexpr float MICROTESLA_TO_GAUSSf = 0.01f;
constexpr double MICROTESLA_TO_GAUSS = 0.01;
constexpr float R2Df =  57.295779513f;
constexpr float D2Rf =  0.0174532925199f;
constexpr double R2D =  57.295779513;
constexpr double D2R =  0.0174532925199;
constexpr double DT = 0.01;


typedef struct state_t {
  Eigen::Vector3f  euler;          // Euler angles of aircraft (in roll, pitch, yaw)
  Eigen::Vector3f eulerPrevious;      // 1 Timestampe previousEuler angles of aircraft (in roll, pitch, yaw)
  Eigen::Vector3f  eulerRate;        // First derivative of euler angles (in roll/s, pitch/s, yaw/s)

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  Eigen::Vector3f mag;

  float barometerAltitude;
  float compassHeading;

  int    num_wraps;        // Number of spins in Yaw
  float  initialYaw;
} state_t;

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

typedef struct setpoint_t {
  float euler_ref[3];  // Reference (Desired) Position
  float euler_ref_previous[3];  // Reference (Desired) Position
  float yaw_rate_ref[2];
  float Aux[2];
  double lat_setpoint;
  double  lon_setpoint;      // Controller Variables for Autonomous Flight
  float altitudeSetpointRate;
  float altitudeSetpoint;
  float dpitch_setpoint; // Desired attitude
  float droll_setpoint;  // Desired attitude
  float throttle;
  float yaw_ref_offset;
  float kill_switch[2];
} setpoint_t;

#endif  // SRC_FLYMS_INCLUDE_FLYMS_COMMON_H_
