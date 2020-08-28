/**
* @file imu.cpp
* @brief Source code to read data from the IMU and process it accordingly
*
* @author Mike Sardonini
* @date 10/16/2018
*/

#include "flyMS/imu.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rc/gpio.h"
#include "rc/time.h"
#include "flyMS/mavlink/fly_stereo/mavlink.h"
#include "spdlog/spdlog.h"

constexpr float R2Df =  57.295779513f;
constexpr float D2Rf =  0.0174532925199f;

static rc_mpu_data_t imuDataShared;
rc_mpu_data_t imuDataLocal;
std::mutex localMutex;

static void dmpCallback(void) {
  std::lock_guard<std::mutex> lock(localMutex);
  memcpy(&imuDataLocal, &imuDataShared, sizeof(rc_mpu_data_t));
}

imu::imu(const YAML::Node &input_params) :
  is_running_(true),
  is_initializing_fusion_(true) {
  delta_t_ = input_params["delta_t"].as<float>();
  // Parse the config parameters
  YAML::Node imu_params = input_params["imu_params"];
  pitch_offset_deg_ = imu_params["pitch_offset_deg"].as<float>();
  roll_offset_deg_ = imu_params["roll_offset_deg"].as<float>();
  yaw_offset_deg_ = imu_params["yaw_offset_deg"].as<float>();

  enable_dmp_ = imu_params["enable_dmp"].as<bool>();
  enable_fusion_ = imu_params["enable_fusion"].as<bool>();
  enable_barometer_ = imu_params["enable_barometer"].as<bool>();

  //Calculate the DCM with out offsets
  calculateDCM(pitch_offset_deg_, roll_offset_deg_, yaw_offset_deg_);

  // Open the serial port to send messages to the Nano
  serial_dev_ = open("/dev/ttyS5", O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_dev_ == -1) {
    std::cerr << "Failed to open the serial device" << std::endl;
    return;
  }

  fcntl(serial_dev_, F_SETFL, fcntl(serial_dev_, F_GETFL) & ~O_NONBLOCK);

  // Get the current configuration of the serial interface
  struct termios serial_config;
  if (tcgetattr(serial_dev_, &serial_config) < 0) {
    std::cerr << "Failed to get the serial device attributes" << std::endl;
    return;
  }

  // Set the serial device configs
  serial_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP
    | IXON);
  serial_config.c_oflag = 0;
  serial_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  serial_config.c_cflag &= ~(CSIZE | PARENB);
  serial_config.c_cflag |= CS8;

  // One input byte is enough to return from read() Inter-character timer off
  serial_config.c_cc[VMIN]  = 0;
  serial_config.c_cc[VTIME] = 0;

  // Communication speed (simple version, using the predefined constants)
  if (cfsetispeed(&serial_config, B115200) < 0 || cfsetospeed(&serial_config, B115200) < 0) {
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return;
  }

  // Finally, apply the configuration
  if (tcsetattr(serial_dev_, TCSAFLUSH, &serial_config) < 0) {
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return;
  }

  // Register the GPIO pin that will tell us when images are being captured
  rc_gpio_init_event(1, 25, 0, GPIOEVENT_REQUEST_FALLING_EDGE);

  gpio_thread_ = std::thread(&imu::GpioThread, this);
  serial_read_thread_ = std::thread(&imu::SerialReadThread, this);
}


//Default Destructor
imu::~imu() {
  rc_mpu_power_off();

  // Shut down the gpio thread
  is_running_.store(false);
  if (gpio_thread_.joinable()) {
    gpio_thread_.join();
  }
  // Shut down the serial read
  is_running_.store(false);
  if (serial_read_thread_.joinable()) {
    serial_read_thread_.join();
  }
  // spdlog::info("imu Destructor\n");
}

void imu::calculateDCM(float pitchOffsetDeg, float rollOffsetDeg, float yawOffsetDeg) {
  //Make the Direcion Cosine Matric DCM from the input offsets from the config file
  float cR1 = cosf(pitchOffsetDeg * D2Rf);
  float sR1 = sinf(pitchOffsetDeg * D2Rf);
  float cP1 = cosf(rollOffsetDeg * D2Rf);
  float sP1 = sinf(rollOffsetDeg * D2Rf);
  float cY1 = cosf(yawOffsetDeg * D2Rf);
  float sY1 = sinf(yawOffsetDeg * D2Rf);

  imu_to_body_ << cR1*cY1, -cP1*sY1 + sP1*sR1*cY1 ,  sP1*sY1 + cP1*sR1*cY1,
    cR1*sY1,  cP1*cY1 + sP1*sR1*sY1, -sP1*cY1 + cP1*sR1*sY1, -sR1, sP1*cR1,
    cP1*cR1;
}


int imu::initializeImu() {
  //Start the barometer
  if (enable_barometer_) {
    if (rc_bmp_init(OVERSAMPLE, INTERNAL_FILTER)) {
      spdlog::error("initialize_barometer failed\n");
      return -1;
    }
  }

  if (enable_fusion_) {
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = 2;
    conf.enable_magnetometer = 1;
    conf.show_warnings = 1;

    conf.gpio_interrupt_pin_chip = 3;
    conf.gpio_interrupt_pin = 21;

    if (rc_mpu_initialize(&imu_data_, conf)) {
      spdlog::error("ERROR: can't talk to IMU, all hope is lost\n");
      rc_led_set(RC_LED_RED, 1);
      rc_led_set(RC_LED_GREEN, 0);
      return -1;
    }

    //Initialize the fusion library which converts raw IMU data to Euler angles
    init_fusion();
  } else if (enable_dmp_) {
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = 2;
    conf.gpio_interrupt_pin_chip = 3;
    conf.gpio_interrupt_pin = 21;
    conf.enable_magnetometer = 1;
    conf.dmp_fetch_accel_gyro = 1;
    conf.dmp_sample_rate = 100;
    conf.orient = ORIENTATION_Z_UP;

    //Check our DCM for the proper orientation config parameter
    float thresh = 0.95f;
    if (imu_to_body_(0, 2) > thresh) {
      conf.orient = ORIENTATION_X_UP;
      calculateDCM(0.0f, roll_offset_deg_, 0.0f);
    } else if (imu_to_body_(0, 2) < -thresh) {
      conf.orient = ORIENTATION_X_DOWN;
      calculateDCM(0.0f, roll_offset_deg_, 0.0f);
    } else if (imu_to_body_(1, 2) > thresh) {
      conf.orient = ORIENTATION_Y_UP;
      calculateDCM(pitch_offset_deg_, 0.0f, 0.0f);
    } else if (imu_to_body_(1, 2) < -thresh) {
      conf.orient = ORIENTATION_Y_DOWN;
      calculateDCM(pitch_offset_deg_, 0.0f, 0.0f);
    } else if (imu_to_body_(2, 2) > thresh) {
      conf.orient = ORIENTATION_Z_UP;
      calculateDCM(0.0f, 0.0f, yaw_offset_deg_);
    } else if (imu_to_body_(2, 2) < -thresh) {
      conf.orient = ORIENTATION_Z_DOWN;
      calculateDCM(0.0f, 0.0f, yaw_offset_deg_);
    } else {
      spdlog::error("Error! In order to be in DMP mode, "
        "one of the X,Y,Z vectors on the IMU needs to be parallel with Gravity\n");
      rc_led_set(RC_LED_RED, 1);
      rc_led_set(RC_LED_GREEN, 0);
      return -1;
    }

    if (rc_mpu_initialize_dmp(&imuDataShared, conf)) {
      spdlog::error("rc_mpu_initialize_failed\n");
      return -1;
    }
    rc_mpu_set_dmp_callback(&dmpCallback);
  }
  return 0;
}

int imu::getImuData(state_t* state) {
  memcpy(state, &state_body_, sizeof(state_t));
  return 0;
}


int imu::update() {
  /**********************************************************
  *        Read and Translate the Raw IMU Data
  **********************************************************/
  read_transform_imu();
  //Perform the data fusion to calculate pitch, roll, and yaw angles
  if (enable_fusion_)
    updateFusion();

  send_mavlink();

  /**********************************************************
  *          Unwrap the Yaw value          *
  **********************************************************/
  state_body_.euler[2] += state_body_.num_wraps * 2 * M_PI;
  if (fabs(state_body_.euler[2] - state_body_.eulerPrevious[2])  > 5) {
    if (state_body_.euler[2] > state_body_.eulerPrevious[2]) {
      state_body_.num_wraps--;
      state_body_.euler[2] -= 2 * M_PI;
    } else {
      state_body_.num_wraps++;
      state_body_.euler[2] += 2 * M_PI;
    }
  }

  /**********************************************************
  *           Read the Barometer for Altitude          *
  **********************************************************/
  static int i1;
  if (enable_barometer_) {
    i1++;
    if (i1 == 10) { // Only read the barometer at 25Hz
      // perform the i2c reads to the sensor, this takes a bit of time
      if (rc_bmp_read(&bmp_data_) < 0) {
        spdlog::error("\rERROR: Can't read Barometer");
        fflush(stdout);
      }
      i1 = 0;
    }
  }
  return 0;
}

/************************************************************************
*        Transform coordinate system IMU Data
************************************************************************/
void imu::read_transform_imu() {

  if (enable_fusion_) {
    if (rc_mpu_read_accel(&imu_data_) < 0) {
      spdlog::error("read accel data failed\n");
    }
    if (rc_mpu_read_gyro(&imu_data_) < 0) {
      spdlog::error("read gyro data failed\n");
    }
    if (rc_mpu_read_mag(&imu_data_)) {
      spdlog::error("read mag data failed\n");
    }
  } else if (enable_dmp_) {
    localMutex.lock();
    memcpy(&imu_data_, &imuDataLocal, sizeof(rc_mpu_data_t));
    localMutex.unlock();

    for (int i = 0; i < 3; i++) {
      state_body_.eulerPrevious(i)     = state_body_.euler(i);
      state_body_.euler(i)        = imu_data_.dmp_TaitBryan[i];
      state_body_.eulerRate(i)      = imu_data_.gyro[i] * D2Rf;
    }
  }

  /**********************************************************
  *    Perform the Coordinate System Transformation    *
  **********************************************************/

  int i;
  for (i = 0; i < 3; i++) {
    state_imu_.mag(i) = imu_data_.mag[i];
    state_imu_.gyro(i) = imu_data_.gyro[i];
    state_imu_.accel(i) = imu_data_.accel[i];
  }

  //Convert from IMU frame to body Frame
  state_body_.mag = imu_to_body_ * state_imu_.mag;
  state_body_.gyro = imu_to_body_ * state_imu_.gyro;
  state_body_.accel = imu_to_body_ * state_imu_.accel;
  state_body_.euler = imu_to_body_ * state_body_.euler;
  state_body_.eulerRate = imu_to_body_ * state_body_.eulerRate;
}


/************************************************************************
*                  Initialize the IMU Fusion Algorithm          *
************************************************************************/
void imu::init_fusion() {
  /*  Three params here are:
    1. gain -  (<= 0) 0 means to only use gyro data in fusion, greater value use accel / mag more
    2. min squared magnetic field, magnetic fields squared less than this will be discarded
    3. max squared magnetic field, magnetic fields squared greater than this will be discarded
  */
  FusionAhrsInitialise(&fusion_ahrs_, 0.25f, 0.0f, 120.0f); // valid magnetic field defined as 20 uT to 70 uT
  /*
    Two params here are:
    1. Min ADC threshold - gyroscope value threshold which means the device is stationary
    2. delta_t_ - time difference in seconds
  */
  FusionBiasInitialise(&fusion_bias_, (int)(0.2f / imu_data_.gyro_to_degs), delta_t_);

  //Give Imu data to the fusion alg for initialization purposes
  while (FusionAhrsIsInitialising(&fusion_ahrs_) || FusionBiasIsActive(&fusion_bias_)) {
    read_transform_imu();

    updateFusion();
    rc_usleep(static_cast<uint64_t>(delta_t_*1.0E6));
  }
  is_initializing_fusion_ = false;
}

/************************************************************************
*         Update the Fusion Algorithm, Called once per IMU Update       *
************************************************************************/
void imu::updateFusion() {

  FusionVector3 gyroscope;
  FusionVector3 accelerometer;
  FusionVector3 magnetometer;

  int i;
  for (i = 0; i < 3; i++) {
    gyroscope.array[i] = state_body_.gyro(i);
    accelerometer.array[i] = state_body_.accel(i) / 9.81f;
    magnetometer.array[i] = state_body_.mag(i);
    if (!is_initializing_fusion_)
      gyroscope.array[i] -= fusion_bias_.gyroscopeBias.array[i] * imu_data_.gyro_to_degs;
  }

  FusionBiasUpdate(&fusion_bias_, imu_data_.raw_gyro[0], imu_data_.raw_gyro[1], imu_data_.raw_gyro[2]);
  FusionAhrsUpdate(&fusion_ahrs_, gyroscope, accelerometer, magnetometer, delta_t_);
  euler_angles_ = FusionQuaternionToEulerAngles(fusion_ahrs_.quaternion);
  Eigen::Vector3f w1; w1 << euler_angles_.angle.roll , euler_angles_.angle.pitch , euler_angles_.angle.yaw;

  //We want pitch and roll to be relative to the Drone's Local Coordinate Frame, not the NED Frame
  //  make that conversion here
  float mag, theta;
  mag = powf(powf(euler_angles_.angle.pitch, 2.0f) + powf(euler_angles_.angle.roll, 2.0f), 0.5f);
  theta = atan2f(euler_angles_.angle.roll, euler_angles_.angle.pitch);

  euler_angles_.angle.roll = mag * sinf(theta - euler_angles_.angle.yaw * D2Rf);
  euler_angles_.angle.pitch = mag * cosf(theta - euler_angles_.angle.yaw * D2Rf);

  for (i = 0; i < 3; i++)
    euler_angles_.array[i] *= -1.0f;

  //Save the output
  for (i = 0; i < 3; i++) {
    state_body_.eulerPrevious(i) = state_body_.euler(i);
    state_body_.euler(i) = euler_angles_.array[i] * D2Rf;
    state_body_.eulerRate(i) = state_body_.gyro(i) * D2Rf;
  }

}

void imu::send_mavlink() {
  mavlink_message_t msg;
  mavlink_imu_t attitude;
  uint8_t buf[1024];

  attitude.roll = state_body_.euler(0);
  attitude.pitch = state_body_.euler(1);
  attitude.yaw = state_body_.euler(2);

  for (int i = 0; i < 3; i++) {
    attitude.gyroXYZ[i] = state_body_.eulerRate(i);
    attitude.accelXYZ[i] = state_body_.accel(i);
  }

  uint64_t imu_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::
    system_clock::now().time_since_epoch()).count();
  std::lock_guard<std::mutex> lock(trigger_time_mutex_);
  attitude.timestamp_us = imu_time;
  attitude.time_since_trigger_us = static_cast<uint32_t>((imu_time - (trigger_time_ / 1E3)));
  // std::cout << "time_since_trig " << attitude.time_since_trigger_us << std::endl;
  attitude.trigger_count = trigger_count_;

  mavlink_msg_imu_encode(1, 200, &msg, &attitude);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  write(serial_dev_, buf, len);
}

void imu::GpioThread() {
  int timeout_ms = 1000;
  uint64_t event_time;

  trigger_time_mutex_.lock();
  trigger_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::
    now().time_since_epoch()).count();
  trigger_count_ = 0;
  trigger_time_mutex_.unlock();

  while(is_running_.load()) {
    int ret = rc_gpio_poll(1, 25, timeout_ms, &event_time);

    if (ret == RC_GPIOEVENT_FALLING_EDGE) {
      std::lock_guard<std::mutex> lock(trigger_time_mutex_);
      trigger_time_ = event_time;
      trigger_count_++;
      // spdlog::error("Falling edge detected, time %llu\n", event_time);
    } else if (ret == RC_GPIOEVENT_RISING_EDGE) {
      spdlog::warn("Error! Rising edge detected, should only be returning on falling "
        "edge");
    } else if (ret == RC_GPIOEVENT_TIMEOUT) {
      // spdlog::warn("GPIO timeout");
    } else if (ret == RC_GPIOEVENT_ERROR) {
      spdlog::warn("GPIO error");
    }
  }
}

void imu::SerialReadThread() {
  unsigned char buf[1024];
  size_t buf_size = 1024;
  std::cout << "starting serial read thread!\n\n";

  while (is_running_.load()) {
    ssize_t ret = read(serial_dev_, buf, buf_size);

    if (ret < 0) {
      std::cerr << "Error on read(), errno: " << strerror(errno) << std::endl;
    }

    for (int i = 0; i < ret; i++) {
      mavlink_status_t mav_status;
      mavlink_message_t mav_message;
      uint8_t msg_received = mavlink_parse_char(MAVLINK_COMM_1, buf[i],
        &mav_message, &mav_status);
    
      if (msg_received) {
        switch(mav_message.msgid) {
          case MAVLINK_MSG_ID_IMU: {
            mavlink_imu_t attitude_msg;
            mavlink_msg_imu_decode(&mav_message, &attitude_msg);
            break;
          }
          case MAVLINK_MSG_ID_RESET_COUNTERS: {
            mavlink_reset_counters_t reset_msg;
            mavlink_msg_reset_counters_decode(&mav_message, &reset_msg);
            std::lock_guard<std::mutex> lock(trigger_time_mutex_);
            trigger_count_ = 0;
            trigger_time_ = 0;
            std::cout << "Received counter reset msg!\n\n";
            break;
          }
          default:
            std::cerr << "Unrecognized message with ID:" << static_cast<int>(
              mav_message.msgid) << std::endl;
            break;
        }
      }
    }
    // Sleep so we don't overload the CPU. This isn't an ideal method, but if
    // use a blocking call on read(), we can't break out of it on the 
    // destruction of this object. It will hang forever until bytes are read,
    // which is not always the case
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}
