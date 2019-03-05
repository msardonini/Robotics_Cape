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

//Package Includes


//Local Includes
#include "src/common.hpp"
#include "src/config.hpp"


#define CORE_LOG_TABLE \
    X(uint64_t,  "%lu",  timeMicroseconds) \
    X(float,  "%f",  roll           ) \
    X(float,  "%f",  pitch          ) \
    X(float,  "%f",  yaw            ) \
    X(float,  "%f",  d_roll         ) \
    X(float,  "%f",  d_pitch        ) \
    X(float,  "%f",  d_yaw          ) \
    X(float,  "%f",  u_1            ) \
    X(float,  "%f",  u_2            ) \
    X(float,  "%f",  u_3            ) \
    X(float,  "%f",  u_4            ) \
    X(float,  "%f",  throttle       ) \
    X(float,  "%f",  upitch         ) \
    X(float,  "%f",  uroll          ) \
    X(float,  "%f",  uyaw           ) \
    X(float,  "%f",  pitch_ref      ) \
    X(float,  "%f",  roll_ref       ) \
    X(float,  "%f",  yaw_ref        ) \
    X(float,  "%f",  yaw_rate_ref   ) \
    X(float,  "%f",  Aux            ) \
    X(float,  "%f",  lat_error      ) \
    X(float,  "%f",  lon_error      ) \
    X(float,  "%f",  kalman_lat     ) \
    X(float,  "%f",  kalman_lon     ) \
    X(float,  "%f",  accel_x        ) \
    X(float,  "%f",  accel_y        ) \
    X(float,  "%f",  accel_z        ) \
    X(float,  "%f",  baro_alt       ) \
    X(float,  "%f",  v_batt         ) \
    X(float,  "%f",  compass_heading) \
    X(float,  "%f",  ned_pos_x) \
    X(float,  "%f",  ned_pos_y) \
    X(float,  "%f",  ned_pos_z) \
    X(float,  "%f",  ned_vel_x) \
    X(float,  "%f",  ned_vel_y) \
    X(float,  "%f",  ned_vel_z) \
    X(float,  "%f",  mag_x) \
    X(float,  "%f",  mag_y) \
    X(float,  "%f",  mag_z) \
    X(float,  "%f",  droll_setpoint) \
    X(float,  "%f",  dpitch_setpoint)

class logger
{

public:
    logger(std::string log_filepath);
    ~logger();	
    int createLogFiles();
    
    int writeToLog(state_t *bodyState, controller_t *controller, setpoint_t *setpoint);

    //Smart method for print statements. Sends to the terminal if open, also sends to console log file.
    int flyMS_printf(const char* format, ...);

    //defines the config params as public members 
    #define X(type, fmt, name) type name;
    CORE_LOG_TABLE
    #undef X

private:

    bool fileExists(std::string filename);
    
    //Bool to indicate if this process is running as a daemon or from a terminal
    bool isRunningConsole;
    std::mutex printMutex;

    //Objects to handle program output to file
    std::string log_filepath_;
    std::ofstream dataLogFid;
    std::ofstream errorFid;
    std::ofstream consoleLogFid;
    std::ostringstream consoleInitBuffer;

};

#endif	//LOGGER_H