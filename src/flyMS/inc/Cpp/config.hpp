/**
 * @file config.hpp
 * @brief flyMS progrom configuration parameters management
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <fstream>
#include <iostream>
#include "yaml-cpp/yaml.h"

#define CORE_CONFIG_TABLE\
    X(bool,  "%d", enableBarometer,     1       )\
    X(int,  "%d", enable_gps,           1       )\
    X(int,  "%d", flightMode,           1       )\
    X(int,  "%d", enable_logging,       1       )\
    X(bool,  "%d", isDebugMode      ,    false       )\
    X(int,  "%d", enable_autonomy,      0       )\
    X(bool,  "%d", isHeadlessMode,        false       )\
    X(bool,  "%d", enableDMP,           true       )\
    X(bool,  "%d", enableFusion,        false       )\
                                                \
    X(float,  "%f", pitchOffsetDegrees,   0.0f    )\
    X(float,  "%f", rollOffsetDegrees,    0.0f    )\
    X(float,  "%f", yawOffsetDegrees,     0.0f    )\
                                                \
    X(float,  "%f", alt_KP,             0.0f    )\
    X(float,  "%f", alt_KD,             0.0f    )\
    X(float,  "%f", alt_KI,             0.0f    )\
                                                \
                                                \
    X(float,  "%f", roll_KP,            5.0f    )\
    X(float,  "%f", roll_KD,            0.5f    )\
    X(float,  "%f", roll_KI,            0.175f  )\
    X(float,  "%f", Droll_KP,           0.0155f )\
    X(float,  "%f", Droll_KI,           0.5f    )\
    X(float,  "%f", Droll_KD,           0.0012f )\
    X(float,  "%f", max_roll_setpoint,  0.4f    )\
                                                \
    X(float,  "%f", pitch_KP,           5.0f    )\
    X(float,  "%f", pitch_KD,           0.5f    )\
    X(float,  "%f", pitch_KI,           0.175f  )\
    X(float,  "%f", Dpitch_KP,          0.0155f )\
    X(float,  "%f", Dpitch_KI,          0.5f    )\
    X(float,  "%f", Dpitch_KD,          0.0012f )\
    X(float,  "%f", max_pitch_setpoint, 0.4f    )\
                                                \
    X(float,  "%f", yaw_KP,             0.5f    )\
    X(float,  "%f", yaw_KI,             0.05f   )\
    X(float,  "%f", yaw_KD,             0.05f   )\
                                                \
    X(float,  "%f", max_yaw_rate,       3.0f    )\
    X(float,  "%f", idle_speed,         0.3f    )\
    X(float,  "%f", min_throttle,       0.3f    )\
    X(float,  "%f", max_throttle,       0.75f   )\
                                                \
    X(float,  "%f", max_vert_velocity,  3.0f    )\
    X(float,  "%f", max_horz_velocity,  3.0f    )\
                                                \
    X(float,  "%f", aux1,   0   )\
    X(float,  "%f", aux2,   0   )\
    X(float,  "%f", aux3,   0   )\
    X(float,  "%f", aux4,   0   )\
    X(float,  "%f", aux5,   0   )\
    X(float,  "%f", aux6,   0   )


typedef struct config_t
{
    //defines the config params as public members 
    #define X(type, fmt, name, defaultVal) type name;
    CORE_CONFIG_TABLE
    #undef X
} config_t;

class flyMSParams
{
  public:

    flyMSParams();
    ~flyMSParams(); 
    void loadConfigFile(std::string filename);
    int writeConfigFile(std::string filename);

    YAML::Node flyMSYamlNode;
    
    config_t config;


    // bool enableBarometer;
    // bool enableGPS;
    // bool enableLogging;
    // bool enableDebugMode;
    // bool enableAutonomy;
    // bool enableHeadlessMode;

    // float pitchOffsetDegrees;
    // float rollOffsetDegrees;
    // float yawOffsetDegrees;

    // float  alt_KP;
    // float  alt_KD;
    // float  alt_KI;

    // float  roll_KP;
    // float  roll_KD;
    // float  roll_KI;
    // float  Droll_KP;
    // float  Droll_KI;
    // float  Droll_KD;
    // float  max_roll_setpoint;

    // float  pitch_KP;
    // float  pitch_KD;
    // float  pitch_KI;
    // float  Dpitch_KP;
    // float  Dpitch_KI;
    // float  Dpitch_KD;
    // float  max_pitch_setpoint;

    // float  yaw_KP;
    // float  yaw_KI;
    // float  yaw_KD;

    // float  maxYawRate;
    // float  minThrottle;
    // float  maxThrottle;

};

#endif	//CONFIG_H