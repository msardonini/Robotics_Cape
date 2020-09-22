#ifndef SRC_FLYMS_INCLUDE_FLYMS_MAVLINK_INTERFACE_H_
#define SRC_FLYMS_INCLUDE_FLYMS_MAVLINK_INTERFACE_H_

#include <thread>
#include <atomic>

#include "yaml-cpp/yaml.h"
#include "spdlog/spdlog.h"
#include "flyMS/flyMS_types.h"
#include "flyMS/imu.h"

class MavlinkInterface {
 public:
  // Delete Unrelated constructors
  MavlinkInterface() = delete;
  MavlinkInterface(const MavlinkInterface&) = delete;
  MavlinkInterface(MavlinkInterface&&) = delete;
  
  MavlinkInterface(const YAML::Node input_params, imu *imu_ptr);
  ~MavlinkInterface();

  int Init();

  int SendImuMessage(const state_t &imu_state);
  int SendStartCommand();
  int SendShutdownCommand();

  bool GetVioData(vio_t *vio);

 private:
  void SerialReadThread();

  // Flag to tell if we are still operating
  std::atomic<bool> is_running_;

  imu* imu_ptr_;

  // file descriptor for the serial port
  int serial_dev_;
  std::string serial_dev_file_;

  std::thread serial_read_thread_;
  std::mutex serial_send_mutex_;

  std::mutex vio_mutex_;
  vio_t vio_;
  bool is_new_vio_data_ = false;
};

#endif  // SRC_FLYMS_INCLUDE_FLYMS_MAVLINK_INTERFACE_H_
