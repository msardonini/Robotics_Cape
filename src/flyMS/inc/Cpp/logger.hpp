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
#include <fstream>
#include <iostream>
#include <sys/stat.h>

//Package Includes


//Local Includes
#include "config.hpp"


#define CORE_LOG_TABLE \
    X(uint64_t,  "%lu",  time       ) \
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
    logger();
    ~logger();	
    int createLogFiles();
    
    int writeToLog();


    //defines the config params as public members 
    #define X(type, fmt, name) type name;
    CORE_LOG_TABLE
    #undef X

private:

    bool fileExists(std::string filename);

    std::ofstream dataLogFid;
    std::ofstream errorFid;
    std::ofstream configFid;
    std::ofstream consoleLogFid;

};

#endif	//LOGGER_H