#ifndef SRC_FLYMS_INCLUDE_FLYMS_IMU_DMP_H_
#define SRC_FLYMS_INCLUDE_FLYMS_IMU_DMP_H_

#include "flyMS/imu/base.h"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"

/**
 * @brief An implementation of the Imu class that uses the DMP (digital motion processor) on the
 *        MPU-9150 as a way to calculate orientation
 *
 */
class ImuDmp : public ImuBase {
 public:
  ImuDmp(const YAML::Node &input_params);

  /**
   * @brief Get the Latest IMU data from the sensor
   *
   * @param imu_state The output imu data
   * @return int 0 on success, -1 on failure
   */
  virtual int GetImuData(StateData *imu_state) override;

  /**
   * @brief Get the Lastest IMU data from the sensor. If no new data is available, block until data
   *        is received
   *
   * @param imu_state
   * @return int 0 on success, -1 on failure
   */
  virtual int GetImuDataBlock(StateData *imu_state) override;

 private:
  // Callback function for the rc API to call when new dmp data is ready
  void DmpCallback();

  // 3D rotation that trasforms the DMP coordiante system to the drone's body frame,
  // which is usually different than the coordiante system of the IMU
  Eigen::Matrix<float, 3, 3> R_imu_body_dmp_;


  // Value of the previous iteration's orientation
  Eigen::Vector3f euler_prev_body_;
};

#endif  // SRC_FLYMS_INCLUDE_FLYMS_IMU_DMP_H_
