
#include "flyMS/mavlink_interface.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

#include "flyMS/mavlink/fly_stereo/mavlink.h"
#include "rc/gpio.h"

MavlinkInterface::MavlinkInterface(const YAML::Node input_params) {
  YAML::Node mavlink_params = input_params["mavlink_interface"];
  serial_dev_file_ = mavlink_params["serial_device"].as<std::string>();
}

int MavlinkInterface::Init() {
  // Open the serial port to send messages to the Nano
  serial_dev_ = open(serial_dev_file_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_dev_ == -1) {
    std::cerr << "Failed to open the serial device" << std::endl;
    return -1;
  }

  // Ensire the port is set to NONBLOCK
  fcntl(serial_dev_, F_SETFL, fcntl(serial_dev_, F_GETFL) & ~O_NONBLOCK);

  // Get the current configuration of the serial interface
  struct termios serial_config;
  if (tcgetattr(serial_dev_, &serial_config) < 0) {
    std::cerr << "Failed to get the serial device attributes" << std::endl;
    return -1;
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
    return -1;
  }

  // Finally, apply the configuration
  if (tcsetattr(serial_dev_, TCSAFLUSH, &serial_config) < 0) {
    std::cerr << "Failed to set the baudrate on the serial port!" << std::endl;
    return -1;
  }
  serial_read_thread_ = std::thread(&MavlinkInterface::SerialReadThread, this);

  // Start the GPIO thread that watches for the image trigger pulses
  gpio_thread_ = std::thread(&MavlinkInterface::GpioThread, this);
  return 0;
}

MavlinkInterface::~MavlinkInterface() {
  is_running_.store(false);
  if (serial_read_thread_.joinable()) {
    serial_read_thread_.join();
  }
}

int MavlinkInterface::SendImuMessage(const StateData &imu_state) {
  mavlink_message_t msg;
  mavlink_imu_t attitude;
  uint8_t buf[1024];

  attitude.roll = imu_state.euler(0);
  attitude.pitch = imu_state.euler(1);
  attitude.yaw = imu_state.euler(2);

  for (int i = 0; i < 3; i++) {
    attitude.gyroXYZ[i] = imu_state.eulerRate(i);
    attitude.accelXYZ[i] = imu_state.accel(i);
  }

  attitude.timestamp_us = imu_state.timestamp_us;

  trigger_time_mutex_.lock();
  attitude.time_since_trigger_us = static_cast<uint32_t>((imu_state.timestamp_us -
    (trigger_time_ / 1E3)));
  attitude.trigger_count = trigger_count_;
  trigger_time_mutex_.unlock();

  mavlink_msg_imu_encode(1, 200, &msg, &attitude);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  std::lock_guard<std::mutex> lock(serial_send_mutex_);
  if(write(serial_dev_, buf, len) < len) {
    spdlog::error("[MavlinkInterface] write() did not succeed");
    return -1;
  }
  return 0;
}

int MavlinkInterface::SendStartCommand() {
  mavlink_message_t msg;
  mavlink_command_t command;
  uint8_t buf[1024];

  memset(&command, 0, sizeof(command));
  command.engage = 1;

  mavlink_msg_command_encode(1, 200, &msg, &command);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  std::lock_guard<std::mutex> lock(serial_send_mutex_);
  if(write(serial_dev_, buf, len) < len) {
    spdlog::error("[MavlinkInterface] write() did not succeed");
  }
  return 0;
}

int MavlinkInterface::SendShutdownCommand() {
  mavlink_message_t msg;
  mavlink_command_t command;
  uint8_t buf[1024];


  memset(&command, 0, sizeof(command));
  command.shutdown = 1;

  mavlink_msg_command_encode(1, 200, &msg, &command);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  std::lock_guard<std::mutex> lock(serial_send_mutex_);
  if(write(serial_dev_, buf, len) < len) {
    spdlog::error("[MavlinkInterface] write() did not succeed");
  }
  return 0;
}

bool MavlinkInterface::GetVioData(VioData *vio) {
  std::lock_guard<std::mutex> lock(vio_mutex_);
  if (is_new_vio_data_) {
    *vio = vio_;
    is_new_vio_data_ = false;
    return true;
  } else {
    return false;
  }
}

void MavlinkInterface::SerialReadThread() {
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

            // TODO Reset the trigger counter on the IMU
            std::cout << "Received counter reset msg!\n\n";
            ResetCounter();
            break;
          }
          case MAVLINK_MSG_ID_VIO: {
            mavlink_vio_t vio;
            mavlink_msg_vio_decode(&mav_message, &vio);
            std::lock_guard<std::mutex> lock(vio_mutex_);
            vio_.position << vio.position[0], vio.position[1], vio.position[2];
            vio_.velocity << vio.velocity[0], vio.velocity[1], vio.velocity[2];
            vio_.quat = Eigen::Quaternion(vio.quat[0], vio.quat[1], vio.quat[2], vio.quat[3]);
            // spdlog::info("new vio data {}, {}, {}, {}!!", vio.position[0], vio.position[1], vio.position[2], vio.position[3]);
            is_new_vio_data_ = true;
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

void MavlinkInterface::GpioThread() {
  // Register the GPIO pin that will tell us when images are being captured
  rc_gpio_init_event(1, 25, 0, GPIOEVENT_REQUEST_FALLING_EDGE);

  int timeout_ms = 1000;

  trigger_time_mutex_.lock();
  trigger_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::
    now().time_since_epoch()).count();
  trigger_count_ = 0;
  trigger_time_mutex_.unlock();

  while(is_running_.load()) {
    uint64_t event_time;
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

void MavlinkInterface::ResetCounter() {
  std::lock_guard<std::mutex> lock(trigger_time_mutex_);
  trigger_count_ = 0;
}


