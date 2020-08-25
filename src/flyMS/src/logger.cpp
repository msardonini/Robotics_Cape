/**
 * @file logger.cpp
 * @brief logging module for the flight controller
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/logger.hpp"

logger::logger(std::string log_filepath) {
  //  Determine if this process started from a terminal
  //      If so, have the console output redirected to the terminal
  if (isatty(fileno(stdout)))
    this->isRunningConsole = true;
}

logger::~logger() {}


int logger::flyMS_printf(const char* format, ...) {
  //Add a lock in case other threads try printing stuff at the same time
  std::lock_guard<std::mutex> lock(this->printMutex);

  //Needed for standard use of vsprtinf
  va_list args;
  va_start (args, format);

  //Allocate buffer for output
  char *buffer = (char*)malloc(vsnprintf (NULL, 0, format, args) + 1);

  //Copy the buffer
  vsprintf (buffer, format, args);

  //If running from a console also publist to stdout
  if (this->isRunningConsole)
    fprintf(stdout, buffer);

  va_end(args);

  //Free our dynamically allocated string buffer
  free(buffer);
}
