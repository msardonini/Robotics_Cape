#ifndef FLYMS_INCLUDE_FLYMS_POSITION_CONTROLLER_H_
#define FLYMS_INCLUDE_FLYMS_POSITION_CONTROLLER_H_

#include <mutex>

#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"
#include "filter.h"
#include "flyMS/types/vio_data.h"

template <typename T>
struct pos_vel {
  std::array<std::array<T, 3>, 2> data;
};

class PositionController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionController(const YAML::Node &config_params);
  ~PositionController();

  int ReceiveVio(const VioData &vio);

  int GetSetpoint(Eigen::Vector3f &setpoint_orientation, float &yaw);

  int SetReferencePosition(const Eigen::Vector3f &position);

  void ResetController();

 private:
  // Setpoint Position, velocity and orientation
  Eigen::Vector3f setpoint_position_;
  Eigen::Vector3f setpoint_velocity_;
  Eigen::Vector3f setpoint_orientation_;
  float yaw_;

  // Roll pitch and yaw saturdation limits, do not exceed these values
  std::array<float, 3> RPY_saturation_limits_;

  // Conversion matrix to map roll, pitch, throttle commands from PID output in XYZ frame
  Eigen::Matrix3f XYZ_to_RollPitchThrottle_;

  // Filters to calculate the PID output. First dimention is X,Y,Z controllers, second is outer
  // loop and inner loop. Ex. pid_[0][1] is inner loop for X axis, pid_[3][0] is outer loop for
  // Z axis
  std::array<std::array<digital_filter_t*, 2>, 3> pid_;

  std::mutex output_mutex_;

  // Configurable params
  float delta_t_;
  // PID Constants for the outer and inner loops (position/velocity)for x, y, and z
  std::array<std::array<float, 3>, 2> pid_coeffs_x_;
  std::array<std::array<float, 3>, 2> pid_coeffs_y_;
  std::array<std::array<float, 3>, 2> pid_coeffs_z_;
};



#endif  // FLYMS_INCLUDE_FLYMS_POSITION_CONTROLLER_H_