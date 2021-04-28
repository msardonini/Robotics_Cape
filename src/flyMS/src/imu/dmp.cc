#include "flyMS/imu/dmp.h"
#include "rc/mpu.h"
#include "rc/led.h"
#include "spdlog/spdlog.h"

constexpr float R2Df =  57.295779513f;
constexpr float D2Rf =  0.0174532925199f;

static rc_mpu_data_t imu_data_shared;
static rc_mpu_data_t imu_data_local;
std::mutex local_mutex;

static void dmpCallback(void) {
  std::lock_guard<std::mutex> lock(local_mutex);
  imu_data_local = imu_data_shared;
}

ImuDmp::ImuDmp(const YAML::Node &input_params) {
  // Load the transform from imu to body frame
  R_imu_body_ = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(input_params["R_imu_body"].
    as<std::array<float, 9> >().data());
  R_imu_body_dmp_ = R_imu_body_;

  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = 2;
  conf.gpio_interrupt_pin_chip = 3;
  conf.gpio_interrupt_pin = 21;
  conf.enable_magnetometer = 1;
  conf.dmp_fetch_accel_gyro = 1;
  conf.dmp_sample_rate = 100;
  conf.orient = ORIENTATION_Z_UP;

  //Check our DCM for the proper orientation config parameter
  const float thresh = 0.95f;
  if (R_imu_body_(0, 2) > thresh) {
    conf.orient = ORIENTATION_X_UP;
    throw std::invalid_argument("X up orientation not yet tested\n");
  } else if (R_imu_body_(0, 2) < -thresh) {
    conf.orient = ORIENTATION_X_DOWN;
    throw std::invalid_argument("X down orientation not yet tested\n");
  } else if (R_imu_body_(1, 2) > thresh) {
    conf.orient = ORIENTATION_Y_UP;
    throw std::invalid_argument("Y up orientation not yet tested\n");
  } else if (R_imu_body_(1, 2) < -thresh) {
    conf.orient = ORIENTATION_Y_DOWN;
    throw std::invalid_argument("Y down orientation not yet tested\n");
  } else if (R_imu_body_(2, 2) > thresh) {
    conf.orient = ORIENTATION_Z_UP;
  } else if (R_imu_body_(2, 2) < -thresh) {
    conf.orient = ORIENTATION_Z_DOWN;
    Eigen::Matrix3f dmp_conv;
    dmp_conv << 1, 0, 0,
                0,-1, 0,
                0, 0,-1;
    R_imu_body_dmp_ = dmp_conv * R_imu_body_dmp_;
  } else {
    std::string err_msg("Error! In order to be in DMP mode, "
      "one of the X,Y,Z vectors on the IMU needs to be parallel with Gravity\n");
    spdlog::error(err_msg);
    rc_led_set(RC_LED_RED, 1);
    rc_led_set(RC_LED_GREEN, 0);
    throw std::invalid_argument(err_msg);
  }

  if (rc_mpu_initialize_dmp(&imu_data_shared, conf)) {
    spdlog::error("rc_mpu_initialize failed");
    throw std::invalid_argument("rc_mpu_initialize failed");
  }

  is_running_.store(true);
}

int ImuDmp::GetImuData(state_t *imu_data_body) {
  std::lock_guard<std::mutex> lock(local_mutex);

  state_t imu_data_imu;
  for (int i = 0; i < 3; i++) {
    imu_data_imu.euler(i)        = imu_data_local.dmp_TaitBryan[i];
    imu_data_imu.eulerRate(i)      = imu_data_local.gyro[i] * D2Rf;
    imu_data_imu.mag(i) = imu_data_local.mag[i];
    imu_data_imu.gyro(i) = imu_data_local.gyro[i];
    imu_data_imu.accel(i) = imu_data_local.accel[i];
  }

  imu_data_body->eulerPrevious = euler_prev_body_;

  // Convert from IMU frame to body Frame
  imu_data_body->mag = R_imu_body_ * imu_data_imu.mag;
  imu_data_body->gyro = R_imu_body_ * imu_data_imu.gyro;
  imu_data_body->accel = R_imu_body_ * imu_data_imu.accel;
  imu_data_body->euler = R_imu_body_dmp_ * imu_data_imu.euler;
  imu_data_body->eulerRate = R_imu_body_dmp_ * imu_data_imu.eulerRate;

  // Save the orientation value for the next iteration
  euler_prev_body_ = imu_data_body->euler;

  // Unwrap the Yaw value
  UnwrapYaw(imu_data_body);

  return 0;
}

int ImuDmp::GetImuDataBlock(state_t *imu_data_body) {
  throw std::invalid_argument("This method is not implemented yet!");
}
