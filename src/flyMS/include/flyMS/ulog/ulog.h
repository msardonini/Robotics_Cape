/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef SRC_FLYMS_INCLUDE_ULOG_ULOG_H_
#define SRC_FLYMS_INCLUDE_ULOG_ULOG_H_

#include <string>
#include <fstream>

#include "flyMS/setpoint.h"
#include "flyMS/types/state_data.h"
#include "flyMS/types/vio_data.h"

struct ULogFlightMsg {
  ULogFlightMsg() {}

  ULogFlightMsg(uint64_t _timestamp_us, const StateData &state, const SetpointData &setpoint,
    const std::array<float, 4> &u, const std::array<float, 4> u_euler) :
  timestamp_us(_timestamp_us),
  RPY{state.euler(0), state.euler(1), state.euler(2)},
  gyro{state.gyro(0), state.gyro(1), state.gyro(2)},
  gyro_filt{state.eulerRate(0), state.eulerRate(1), state.eulerRate(2)},
  accel{state.accel(0), state.accel(1), state.accel(2)},
  motor_cmds{u[0], u[1], u[2], u[3]},
  u{u_euler[0], u_euler[1], u_euler[2], setpoint.throttle},
  RPY_ref{setpoint.euler_ref[0], setpoint.euler_ref[1],
      setpoint.euler_ref[2]} {}

  uint64_t timestamp_us;
  float RPY[3];
  float gyro[3];
  float gyro_filt[3];
  float accel[3];
  float motor_cmds[4];
  float u[4];
  float RPY_ref[3];

  static constexpr char FORMAT[] = "flight:uint64_t timestamp;float[3] RPY;float[3] gyro;float[3] "
    "gyro_filt;float[3] accel;float[4] motor_cmds;float[4] u;float[3] RPY_ref;";
  static constexpr uint16_t ID() {return 0x0000;};
}__attribute__((packed));

struct ULogGpsMsg {
  // Default Constructor
  ULogGpsMsg() {}

  // Data Constructor
  ULogGpsMsg(uint64_t _timestamp_us, double _LLA[3]) : timestamp_us(_timestamp_us),
    LLA{_LLA[0], _LLA[1], _LLA[2]} {}

  // Data Members
  uint64_t timestamp_us;
  double LLA[3];

  static constexpr char FORMAT[] = "gps:uint64_t timestamp;double[3] LLA;";
  static constexpr uint16_t ID() {return 0x0001;};
}__attribute__((packed));

struct ULogPosCntrlMsg {
  ULogPosCntrlMsg(uint64_t _timestamp_us, float _position[3], float _velocity[3], float _quat[4],
    float _command_RPWT[4]) :
    timestamp(_timestamp_us),
    position{_position[0], _position[1], _position[2]},
    velocity{_velocity[0], _velocity[1], _velocity[2]},
    quat{_quat[0], _quat[1], _quat[2], _quat[3]},
    command_RPWT{_command_RPWT[0], _command_RPWT[1], _command_RPWT[2], _command_RPWT[3]} {}

  // Data Members
  uint64_t timestamp;
  float position[3];
  float velocity[3];
  float quat[4];
  float command_RPWT[4];

  static constexpr char FORMAT[] = "pos_cntrl:uint64_t timestamp;float[3] position;float[3] "
    "velocity;float[4] quat;float[4] command;";
  static constexpr uint16_t ID() {return 0x0002;};
}__attribute__((packed));

typedef enum class ULogMessageType {
  FORMAT = 'F',
  DATA = 'D',
  INFO = 'I',
  INFO_MULTIPLE = 'M',
  PARAMETER = 'P',
  ADD_LOGGED_MSG = 'A',
  REMOVE_LOGGED_MSG = 'R',
  SYNC = 'S',
  DROPOUT = 'O',
  LOGGING = 'L',
  FLAG_BITS = 'B',
}ULogMessageType;

/** first bytes of the file */
struct ulog_file_header_s {
  uint8_t magic[8];
  uint64_t timestamp;
}__attribute__((packed));

#define ULOG_MSG_HEADER_LEN 3 //accounts for msg_size and msg_type
struct ulog_message_header_s {
  uint16_t msg_size;
  uint8_t msg_type;
}__attribute__((packed));

struct ulog_message_format_s {
  struct ulog_message_header_s header;
  char format[2096];
}__attribute__((packed));

struct ulog_message_add_logged_s {
  struct ulog_message_header_s header;

  uint8_t multi_id;
  uint16_t msg_id;
  char message_name[255];
}__attribute__((packed));

struct ulog_message_remove_logged_s {
  struct ulog_message_header_s header;

  uint16_t msg_id;
}__attribute__((packed));

struct ulog_message_sync_s {
  struct ulog_message_header_s header;

  uint8_t sync_magic[8];
}__attribute__((packed));

struct ulog_message_dropout_s {
  struct ulog_message_header_s header;

  uint16_t duration; //in ms
}__attribute__((packed));

struct ulog_message_data_header_s {
  struct ulog_message_header_s header;

  uint16_t msg_id;
}__attribute__((packed));

struct ulog_message_info_header_s {
  struct ulog_message_header_s header;

  uint8_t key_len;
  char key[255];
}__attribute__((packed));

struct ulog_message_info_multiple_header_s {
  struct ulog_message_header_s header;

  uint8_t is_continued; ///< can be used for arrays: set to 1, if this message is part of the previous with the same key
  uint8_t key_len;
  char key[255];
}__attribute__((packed));

struct ulog_message_logging_s {
  struct ulog_message_header_s header;

  uint8_t log_level; //same levels as in the linux kernel
  uint64_t timestamp;
  char message[128]; //defines the maximum length of a logged message string
}__attribute__((packed));

struct ulog_message_parameter_header_s {
  struct ulog_message_header_s header;

  uint8_t key_len;
  char key[255];
}__attribute__((packed));


struct ulog_message_flag_bits_s {
  struct ulog_message_header_s header;

  uint8_t compat_flags[8];
  uint8_t incompat_flags[8]; ///< @see ULOG_INCOMPAT_FLAG_*
  uint64_t appended_offsets[3]; ///< file offset(s) for appended data if ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK is set
}__attribute__((packed));

class ULog {
 public:
  ULog();

  ~ULog();

  int InitUlog(const std::string &filename);

  template<typename T>
  void WriteFlightData(const T &data, uint16_t msg_id) {
    size_t write_size = sizeof(struct ulog_message_data_header_s) + sizeof(T);

    // Populate the header
    ulog_message_header_s header;
    header.msg_size = write_size - ULOG_MSG_HEADER_LEN;

    header.msg_type = (char) ULogMessageType::DATA;

    // Populate the header
    struct ulog_message_data_header_s data_header;
    data_header.header = header;
    data_header.msg_id = msg_id;

    // Copy the header
    char buf[1024];
    memcpy(buf, &data_header, sizeof(struct ulog_message_data_header_s));

    // Copy the struct
    memcpy(buf + sizeof(struct ulog_message_data_header_s), &data, sizeof(T));

    // Write to File
    WriteMessage(buf, write_size);
  }

 private:
  void WriteHeader();
  void WriteFormats(const std::string &msg_format);
  void WriteAddLog(uint16_t id, std::string msg_name);
  uint64_t getTimeMircos();
  void WriteMessage(void* buf, size_t size);

  // File descriptor for interfacing with the file
  std::ofstream fd_;
};

#endif  // SRC_FLYMS_INCLUDE_ULOG_ULOG_H_