/**
 * @file logger.hpp
 * @brief logging module for the flight controller
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef LOGGER_H
#define LOGGER_H

//System Includes
#include <string>
#include <mutex>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <stdarg.h>
#include <unistd.h>

//Local Includes
#include "flyMS/common.hpp"
#include "flyMS/config.hpp"
// #include "flyMS/setpoint.hpp"


class logger {
 public:
  logger(std::string log_filepath);
  ~logger();
  int createLogFiles();

  // Smart method for print statements. Sends to the terminal if open, also sends to console log
  // file.
  int flyMS_printf(const char* format, ...);

 private:
  //Bool to indicate if this process is running as a daemon or from a terminal
  bool isRunningConsole;
  std::mutex printMutex;

};

#endif  //LOGGER_H
