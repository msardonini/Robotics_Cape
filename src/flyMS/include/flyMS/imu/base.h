#ifndef SRC_FLYMS_INCLUDE_FLYMS_IMU_BASE_H_
#define SRC_FLYMS_INCLUDE_FLYMS_IMU_BASE_H_

#include <atomic>
#include <exception>

#include "Eigen/Dense"
#include "flyMS/flyMS_types.h"

class ImuBase {
 public:
  ImuBase();

  ImuBase(const ImuBase&) = delete;  // Forbid the copy ctor
  void operator=(const ImuBase&) = delete;  // Forbid the assignment operator

  /**
   * @brief Get the Latest IMU data from the sensor
   *
   * @param imu_state The output imu data converted to the body coordinate frame
   * @return int 0 on success, -1 on failure
   */
  virtual int GetImuData(state_t *imu_state) = 0;

  /**
   * @brief Get the Lastest IMU data from the sensor. If no new data is available, block until data
   *        is received
   *
   * @param imu_state The output imu data converted to the body coordinate frame
   * @return int 0 on success, -1 on failure
   */
  virtual int GetImuDataBlock(state_t *imu_state) = 0;

  void UnwrapYaw(state_t *imu_state_body);

 protected:

  // Is the object still running or shutting down
  std::atomic<bool> is_running_;

  // 3D rotation that trasforms the imu coordiante system to the drone's body frame
  Eigen::Matrix<float, 3, 3> R_imu_body_;

  // Number of wraps we have turned in yaw
  int num_wraps_ = 0;

  static bool does_imu_obj_exist;
};

#endif  // SRC_FLYMS_INCLUDE_FLYMS_IMU_BASE_H_