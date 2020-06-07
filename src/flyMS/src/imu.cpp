/**
* @file imu.cpp
* @brief Source code to read data from the IMU and process it accordingly
*
* @author Mike Sardonini
* @date 10/16/2018
*/

#include "flyMS/imu.hpp"

#include "flyMS/mavlink/common/mavlink.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

rc_mpu_data_t imuDataShared;
rc_mpu_data_t imuDataLocal;
std::mutex localMutex;

static void dmpCallback(void) {
  std::lock_guard<std::mutex> lock(localMutex);
  memcpy(&imuDataLocal, &imuDataShared, sizeof(rc_mpu_data_t));
}

imu::imu(config_t _config, logger& _loggingModule) :
  is_running_(true),
  isInitializingFusion(true),
  config(_config),
  loggingModule(_loggingModule) {
  //Calculate the DCM with out offsets
  this->calculateDCM(this->config.pitchOffsetDegrees, this->config.rollOffsetDegrees, this->config.yawOffsetDegrees);


  // Open the serial port to send messages to
  // Open the device
  serial_dev_ = open("/dev/ttyS5", O_RDWR | O_NOCTTY | O_NDELAY);
  if(serial_dev_ == -1) {
   std::cerr << "Failed to open the serial device" << std::endl;
   return;
  }

  struct termios  config;
  //
  // Get the current configuration of the serial interface
  //
  if(tcgetattr(serial_dev_, &config) < 0) {
   std::cerr << "Failed to get the serial device attributes" << std::endl;
   return;
  }

  // Set the serial device configs
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP
   | IXON);
  config.c_oflag = 0;
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN]  = 1;
  config.c_cc[VTIME] = 0;

  //
  // Communication speed (simple version, using the predefined
  // constants)
  //
  if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
   std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
   return;
  }

  //
  // Finally, apply the configuration
  //
  if(tcsetattr(serial_dev_, TCSAFLUSH, &config) < 0) {
   std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
   return;
  }

  // Register the GPIO pin that will tell us when images are being captured
  rc_gpio_init_event(1, 25, 0, GPIOEVENT_REQUEST_FALLING_EDGE);

  gpioThread = std::thread(&imu::GpioThread, this);
}


//Default Destructor
imu::~imu() {
  rc_mpu_power_off();
  // this->loggingModule.flyMS_printf("imu Destructor\n");
}

void imu::calculateDCM(float pitchOffsetDeg, float rollOffsetDeg, float yawOffsetDeg) {
  //Make the Direcion Cosine Matric DCM from the input offsets from the config file
  float cR1 = cosf(pitchOffsetDeg * D2R_IMU);
  float sR1 = sinf(pitchOffsetDeg * D2R_IMU);
  float cP1 = cosf(rollOffsetDeg * D2R_IMU);
  float sP1 = sinf(rollOffsetDeg * D2R_IMU);
  float cY1 = cosf(yawOffsetDeg * D2R_IMU);
  float sY1 = sinf(yawOffsetDeg * D2R_IMU);

  this->imu2Body << cR1*cY1, -cP1*sY1 + sP1*sR1*cY1 ,  sP1*sY1 + cP1*sR1*cY1,
    cR1*sY1,  cP1*cY1 + sP1*sR1*sY1, -sP1*cY1 + cP1*sR1*sY1, -sR1, sP1*cR1,
    cP1*cR1;
}


int imu::initializeImu() {
  //Start the barometer
  if (this->config.enableBarometer) {
    if (rc_bmp_init(OVERSAMPLE, INTERNAL_FILTER)) {
      this->loggingModule.flyMS_printf("initialize_barometer failed\n");
      return -1;
    }
  }

  if (this->config.enableFusion) {
    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = 2;
    conf.enable_magnetometer = 1;
    conf.show_warnings = 0;
    if (rc_mpu_initialize(&this->imu_data, conf)) {
      this->loggingModule.flyMS_printf("ERROR: can't talk to IMU, all hope is lost\n");
      rc_led_set(RC_LED_RED, 1);
      rc_led_set(RC_LED_GREEN, 0);
      return -1;
    }

    //Initialize the fusion library which converts raw IMU data to Euler angles
    this->init_fusion();
  } else if (this->config.enableDMP) {
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
    if (this->imu2Body(0, 2) > thresh) {
      conf.orient = ORIENTATION_X_UP;
      this->calculateDCM(0.0f, this->config.rollOffsetDegrees, 0.0f);
    } else if (this->imu2Body(0, 2) < -thresh) {
      conf.orient = ORIENTATION_X_DOWN;
      this->calculateDCM(0.0f, this->config.rollOffsetDegrees, 0.0f);
    } else if (this->imu2Body(1, 2) > thresh) {
      conf.orient = ORIENTATION_Y_UP;
      this->calculateDCM(this->config.pitchOffsetDegrees, 0.0f, 0.0f);
    } else if (this->imu2Body(1, 2) < -thresh) {
      conf.orient = ORIENTATION_Y_DOWN;
      this->calculateDCM(this->config.pitchOffsetDegrees, 0.0f, 0.0f);
    } else if (this->imu2Body(2, 2) > thresh) {
      conf.orient = ORIENTATION_Z_UP;
      this->calculateDCM(0.0f, 0.0f, this->config.yawOffsetDegrees);

    } else if (this->imu2Body(2, 2) < -thresh) {
      conf.orient = ORIENTATION_Z_DOWN;
      this->calculateDCM(0.0f, 0.0f, this->config.yawOffsetDegrees);
    } else {
      this->loggingModule.flyMS_printf("Error! In order to be in DMP mode, \
one of the X,Y,Z vectors on the IMU needs to be parallel with Gravity\n");
      rc_led_set(RC_LED_RED, 1);
      rc_led_set(RC_LED_GREEN, 0);
      return -1;
    }

    if (rc_mpu_initialize_dmp(&imuDataShared, conf)) {
      this->loggingModule.flyMS_printf("rc_mpu_initialize_failed\n");
      return -1;
    }
    rc_mpu_set_dmp_callback(&dmpCallback);
  }
  return 0;
}

int imu::getImuData(state_t* state) {
  memcpy(state, &this->stateBody, sizeof(state_t));
  return 0;
}


int imu::update() {
  /**********************************************************
  *        Read and Translate the Raw IMU Data
  **********************************************************/
  this->read_transform_imu();
  //Perform the data fusion to calculate pitch, roll, and yaw angles
  if (this->config.enableFusion)
    this->updateFusion();

  send_mavlink();

  /**********************************************************
  *          Unwrap the Yaw value          *
  **********************************************************/
  this->stateBody.euler[2] += this->stateBody.num_wraps * 2 * M_PI;
  if (fabs(this->stateBody.euler[2] - this->stateBody.eulerPrevious[2])  > 5) {
    if (this->stateBody.euler[2] > this->stateBody.eulerPrevious[2]) {
      this->stateBody.num_wraps--;
      this->stateBody.euler[2] -= 2 * M_PI;
    } else {
      this->stateBody.num_wraps++;
      this->stateBody.euler[2] += 2 * M_PI;
    }
  }

  /**********************************************************
  *           Read the Barometer for Altitude          *
  **********************************************************/
  static int i1;
  if (this->config.enableBarometer) {
    i1++;
    if (i1 == 10) { // Only read the barometer at 25Hz
      // perform the i2c reads to the sensor, this takes a bit of time
      if (rc_bmp_read(&bmp_data) < 0) {
        this->loggingModule.flyMS_printf("\rERROR: Can't read Barometer");
        fflush(stdout);
      }
      i1 = 0;
    }
    // this->baro_alt = update_filter(filters.LPF_baro_alt,rc_bmp_get_altitude_m() - initial_alt);

    //TODO: Interface with the barometer data
    // this->stateBody.barometerAltitude = rc_bmp_get_altitude_m();
    // this->ekfContainer.input.barometer_updated = 1;
    // this->ekfContainer.input.barometer_alt = this->stateBody.barometerAltitude;
  }

  /************************************************************************
  *                     Send data to PX4's EKF                          *
  ************************************************************************/
  //TODO: enable the EKF again

  // int i;
  // for (i = 0; i < 3; i++)
  // {
  //   // this->ekfContainer.input.accel[i] = this->transform.accel_drone.d[i];
  //   // this->ekfContainer.input.mag[i] = imu_data.mag[i] * MICROTESLA_TO_GAUSS;
  //   // this->ekfContainer.input.gyro[i] = this->stateBody.eulerRate[i];
  // }
  //TODO implement the EKF and use real timestamps
  // this->ekfContainer.input.IMU_timestamp = this->time_us;

  return 0;
}



/************************************************************************
*        Transform coordinate system IMU Data
************************************************************************/
void imu::read_transform_imu() {

  if (this->config.enableFusion) {
    if (rc_mpu_read_accel(&this->imu_data) < 0) {
      this->loggingModule.flyMS_printf("read accel data failed\n");
    }
    if (rc_mpu_read_gyro(&this->imu_data) < 0) {
      this->loggingModule.flyMS_printf("read gyro data failed\n");
    }
    if (rc_mpu_read_mag(&this->imu_data)) {
      this->loggingModule.flyMS_printf("read mag data failed\n");
    }
  } else if (this->config.enableDMP) {
    localMutex.lock();
    memcpy(&this->imu_data, &imuDataLocal, sizeof(rc_mpu_data_t));
    localMutex.unlock();

    for (int i = 0; i < 3; i++) {
      this->stateBody.eulerPrevious(i)     = this->stateBody.euler(i);
      this->stateBody.euler(i)        = this->imu_data.dmp_TaitBryan[i];
      this->stateBody.eulerRate(i)      = this->imu_data.gyro[i] * D2R_IMU;
    }
  }
  /**********************************************************
  *    Perform the Coordinate System Transformation    *
  **********************************************************/

  int i;
  for (i = 0; i < 3; i++) {
    this->stateIMU.mag(i) = this->imu_data.mag[i];
    this->stateIMU.gyro(i) = this->imu_data.gyro[i];
    this->stateIMU.accel(i) = this->imu_data.accel[i];
  }

  //Convert from IMU frame to body Frame
  this->stateBody.mag = this->imu2Body * this->stateIMU.mag;
  this->stateBody.gyro = this->imu2Body * this->stateIMU.gyro;
  this->stateBody.accel = this->imu2Body * this->stateIMU.accel;
  this->stateBody.euler = this->imu2Body * this->stateBody.euler;
  this->stateBody.eulerRate = this->imu2Body * this->stateBody.eulerRate;
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
  FusionAhrsInitialise(&this->fusionAhrs, 0.25f, 0.0f, 120.0f); // valid magnetic field defined as 20 uT to 70 uT
  /*
    Two params here are:
    1. Min ADC threshold - gyroscope value threshold which means the device is stationary
    2. DT - time difference in seconds
  */
  FusionBiasInitialise(&this->fusionBias, (int)(0.2f / imu_data.gyro_to_degs), DT);

  //Give Imu data to the fusion alg for initialization purposes
  while (FusionAhrsIsInitialising(&this->fusionAhrs) || FusionBiasIsActive(&this->fusionBias)) {
    read_transform_imu();

    this->updateFusion();
    rc_usleep(DT_US);
  }
  this->isInitializingFusion = false;
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
    gyroscope.array[i] = this->stateBody.gyro(i);
    accelerometer.array[i] = this->stateBody.accel(i) / 9.81f;
    magnetometer.array[i] = this->stateBody.mag(i);
    if (!this->isInitializingFusion)
      gyroscope.array[i] -= this->fusionBias.gyroscopeBias.array[i] * imu_data.gyro_to_degs;
  }

  FusionBiasUpdate(&this->fusionBias, imu_data.raw_gyro[0], imu_data.raw_gyro[1], imu_data.raw_gyro[2]);
  FusionAhrsUpdate(&this->fusionAhrs, gyroscope, accelerometer, magnetometer, DT);
  this->eulerAngles = FusionQuaternionToEulerAngles(this->fusionAhrs.quaternion);

  //We want pitch and roll to be relative to the Drone's Local Coordinate Frame, not the NED Frame
  //  make that conversion here
  float mag, theta;
  mag = powf(powf(this->eulerAngles.angle.pitch, 2.0f) + powf(this->eulerAngles.angle.roll, 2.0f), 0.5f);
  theta = atan2f(this->eulerAngles.angle.roll, this->eulerAngles.angle.pitch);

  this->eulerAngles.angle.roll = mag * sinf(theta - this->eulerAngles.angle.yaw * D2R_IMU);
  this->eulerAngles.angle.pitch = mag * cosf(theta - this->eulerAngles.angle.yaw * D2R_IMU);

  for (i = 0; i < 3; i++)
    this->eulerAngles.array[i] *= -1.0f;

  //Save the output
  for (i = 0; i < 3; i++) {
    this->stateBody.eulerPrevious(i) = this->stateBody.euler(i);
    this->stateBody.euler(i) = this->eulerAngles.array[i] * D2R_IMU;
    this->stateBody.eulerRate(i) = this->stateBody.gyro(i) * D2R_IMU;
  }

}

void imu::send_mavlink() {
  mavlink_message_t msg;
  mavlink_attitude_t attitude;
  uint8_t buf[1024];

  attitude.roll = this->stateBody.euler(0);
  attitude.pitch = this->stateBody.euler(1);
  attitude.yaw = this->stateBody.euler(2);

  attitude.rollspeed = this->stateBody.eulerRate(0);
  attitude.pitchspeed = this->stateBody.eulerRate(1);
  attitude.yawspeed = this->stateBody.eulerRate(1);

  uint16_t len = mavlink_msg_attitude_encode(1, 200, &msg, &attitude);

  len = mavlink_msg_to_send_buffer(buf, &msg);

  write(serial_dev_, buf, len);

}

void imu::GpioThread() {
  int timeout_ms = 10000;
  uint64_t event_time;

  while(is_running_.load()) {
    int ret = rc_gpio_poll(1,25, timeout_ms, &event_time);

    if (ret == RC_GPIOEVENT_FALLING_EDGE)
      loggingModule.flyMS_printf("Falling edge detected, time %llu\n", event_time);
    else if (ret == RC_GPIOEVENT_RISING_EDGE)
      loggingModule.flyMS_printf("Rising edge detected, time %llu\n", event_time);
    else if (ret == RC_GPIOEVENT_TIMEOUT)
      loggingModule.flyMS_printf("GPIO timeout");
    else if (ret == RC_GPIOEVENT_ERROR)
      loggingModule.flyMS_printf("GPIO error");
  }


}


