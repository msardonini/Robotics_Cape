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
#include "yaml-cpp/yaml.h"


class flyMSParams
{
  public:

    flyMSParams();
    ~flyMSParams();	
    void readConfigFile(std::string filename);
    void configureflyMSParams();
    YAML::Node flyMSYamlNode;
    

    bool enableBarometer;
    bool enableGPS;
    bool enableLogging;
    bool enableDebugMode;
    bool enableAutonomy;
    bool enableHeadlessMode;

    float pitchOffsetDegrees;
    float rollOffsetDegrees;
    float yawOffsetDegrees;

    float  alt_KP;
    float  alt_KD;
    float  alt_KI;

    float  roll_KP;
    float  roll_KD;
    float  roll_KI;
    float  Droll_KP;
    float  Droll_KI;
    float  Droll_KD;
    float  max_roll_setpoint;

    float  pitch_KP;
    float  pitch_KD;
    float  pitch_KI;
    float  Dpitch_KP;
    float  Dpitch_KI;
    float  Dpitch_KD;
    float  max_pitch_setpoint;

    float  yaw_KP;
    float  yaw_KI;
    float  yaw_KD;

    float  maxYawRate;
    float  minThrottle;
    float  maxThrottle;

};

#endif	//CONFIG_H