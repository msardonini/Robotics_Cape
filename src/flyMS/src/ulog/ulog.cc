#include "flyMS/ulog/ulog.h"

//System Includes
#include<unistd.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <string>
#include <iostream>


ULog::ULog() {}

ULog::~ULog() {
  fd_.close();
}

int ULog::InitUlog(const std::string &log_folder) {
  // Create the log file
  fd_ = std::ofstream(log_folder + "/logger.ulg", std::ios::binary);
  if (!fd_.is_open()) {
    std::cerr << "Error! Could not open file" << std::endl;
    return -1;
  }

  // Writes the header to the file
  WriteHeader();

  std::string flight_msg_format("flight:uint64_t timestamp;float[3] RPY;float[3] gyro;"
    "float[3] gyro_filt;float[3] accel;float[4] motor_cmds;float[4] u;float[3] RPY_ref;");
  std::string gps_msg_format("gps:uint64_t timestamp;double[3] LLA;");

  // Write the file formats to the beginning of the ulog file
  WriteFormats(flight_msg_format);
  WriteFormats(gps_msg_format);

  // Give the Message Id's an associated name
  WriteAddLog(FLIGHT_MSG_ID, "flight");
  WriteAddLog(GPS_MSG_ID, "gps");

  return 0;
}

void ULog::WriteAddLog(uint16_t id, std::string msg_name) {
  struct ulog_message_add_logged_s msg = {};
  size_t write_size = sizeof(struct ulog_message_add_logged_s) - sizeof(msg.message_name) +
    msg_name.size();

  // Populate the header
  ulog_message_header_s header;
  header.msg_size = write_size - ULOG_MSG_HEADER_LEN;
  header.msg_type = (char) ULogMessageType::ADD_LOGGED_MSG;

  // Populate the message
  msg.header = header;
  msg.multi_id = 0x00;
  msg.msg_id = id;
  strcpy(msg.message_name, msg_name.c_str());

  WriteMessage(&msg, write_size);
}

void ULog::WriteFormats(const std::string &msg_format) {
  struct ulog_message_format_s msg;
  size_t write_size = sizeof(struct ulog_message_format_s) - sizeof(msg.format) +
    msg_format.size();

  // Populate the header
  ulog_message_header_s header;
  header.msg_size = write_size - ULOG_MSG_HEADER_LEN;
  header.msg_type = (char) ULogMessageType::FORMAT;

  // Populate the message
  msg.header = header;
  strcpy(msg.format, msg_format.c_str());

  // Write the message to disk
  WriteMessage(&msg, write_size);
}

void ULog::WriteHeader() {
  struct ulog_file_header_s ulog_header = {};
  ulog_header.magic[0] = 'U';
  ulog_header.magic[1] = 'L';
  ulog_header.magic[2] = 'o';
  ulog_header.magic[3] = 'g';
  ulog_header.magic[4] = 0x01;
  ulog_header.magic[5] = 0x12;
  ulog_header.magic[6] = 0x35;
  ulog_header.magic[7] = 0x01;  // file version 1
  ulog_header.timestamp = getTimeMircos();

  WriteMessage(&ulog_header, sizeof(ulog_header));

  // write the Flags message: this MUST be written right after the ulog header
  ulog_message_header_s header;
  header.msg_size = sizeof(struct ulog_message_flag_bits_s) - ULOG_MSG_HEADER_LEN;
  header.msg_type = (char) ULogMessageType::FLAG_BITS;

  struct ulog_message_flag_bits_s flag_bits;
  memset(&flag_bits, 0, sizeof(flag_bits));
  flag_bits.header = header;
  
  WriteMessage(&flag_bits, sizeof(flag_bits));
}

uint64_t ULog::getTimeMircos() {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return tv.tv_sec*(uint64_t)1E6 + tv.tv_nsec/(uint64_t)1E3;
}

int ULog::WriteMessage(void* buf, size_t size) {
  fd_.write(reinterpret_cast<const char*>(buf), size);
}