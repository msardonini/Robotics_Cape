#include "flyMS/position_controller.h"


PositionController::PositionController(const YAML::Node &config_params) {
  delta_t_ = config_params["delta_t"].as<float>();
  pid_coeffs_x_[0] = config_params["pid_coeffs_x_outer"].as<std::array<float, 3> >();
  pid_coeffs_x_[1] = config_params["pid_coeffs_x_inner"].as<std::array<float, 3> >();
  pid_coeffs_y_[0] = config_params["pid_coeffs_y_outer"].as<std::array<float, 3> >();
  pid_coeffs_y_[1] = config_params["pid_coeffs_y_inner"].as<std::array<float, 3> >();
  pid_coeffs_z_[0] = config_params["pid_coeffs_z_outer"].as<std::array<float, 3> >();
  pid_coeffs_z_[1] = config_params["pid_coeffs_z_inner"].as<std::array<float, 3> >();

  // Roll, pitch, and yaw output saturdation limits
  RPY_saturation_limits_ = config_params["RPY_saturation_limits"].as<std::array<float, 3> >();

  for (int i = 0; i < 2; i++) {
    pid_[0][i] = generatePID(pid_coeffs_x_[i][0], pid_coeffs_x_[i][1], pid_coeffs_x_[i][2],
      0.15, delta_t_);
    pid_[1][i] = generatePID(pid_coeffs_y_[i][0], pid_coeffs_y_[i][1], pid_coeffs_y_[i][2],
      0.15, delta_t_);
    pid_[2][i] = generatePID(pid_coeffs_z_[i][0], pid_coeffs_z_[i][1], pid_coeffs_z_[i][2],
      0.15, delta_t_);
  }

  // Zero out the reference values
  setpoint_position_ = Eigen::Vector3f::Zero();
  setpoint_velocity_ = Eigen::Vector3f::Zero();
  setpoint_orientation_ = Eigen::Vector3f::Zero();
}

PositionController::~PositionController() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      free(pid_[i][j]);
    }
  }
}

int PositionController::ReceiveVio(const Eigen::Vector3f &position,
  const Eigen::Vector3f &velocity, const Eigen::Vector3f &orientation) {
  Eigen::Vector3f setpoint_orientation_xyz;

  Eigen::Matrix3f R_xyz_body = Eigen::AngleAxis(orientation(0), Eigen::Vector3f::UnitX()) * 
    Eigen::AngleAxis(orientation(1), Eigen::Vector3f::UnitY()) * 
    Eigen::AngleAxis(orientation(2), Eigen::Vector3f::UnitZ()).toRotationMatrix().transpose();

  // Calculate the PIDs for the outer and inner loops on XYZ axis
  for (int i = 0; i < 3; i++) {
    setpoint_velocity_[i] = update_filter(pid_[i][0], setpoint_position_[i] - position[i]);
    setpoint_orientation_xyz(0) = update_filter(pid_[i][1], setpoint_velocity_(i) - velocity(i));
  }

  // lock the mutex to protect our output variable
  std::lock_guard<std::mutex> lock(output_mutex_);

  // Convert from rotations about XYZ axis to pitch/roll/yaw in body frame
  setpoint_orientation_ = R_xyz_body * setpoint_orientation_xyz;

  // Apply a saturation filter to keep the behavior in check
  for (int i = 0; i < 3; i++) {
    setpoint_orientation_(i) = saturateFilter(setpoint_orientation_(i), -RPY_saturation_limits_[0],
      RPY_saturation_limits_[0]);
  }

  return 0;
}

int PositionController::GetSetpoint(Eigen::Vector3f &setpoint_orientation) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  setpoint_orientation = setpoint_orientation_;
  return 0;
}


int PositionController::SetReferencePosition(const Eigen::Vector3f &position) {
  setpoint_position_ = position;
  return 0;
}
