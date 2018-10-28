#include "config.hpp"

flyMSParams::flyMSParams(){}
flyMSParams::~flyMSParams(){}

void flyMSParams::readConfigFile(std::string filename){
	YAML::Node rootNode = YAML::LoadFile(filename);
    this->flyMSYamlNode = rootNode["flyMSParams"];
}

void flyMSParams::configureflyMSParams()
{
    this->enableGPS = this->flyMSYamlNode["enableGPS"].as<bool>();
    this->enableLogging = this->flyMSYamlNode["enableLogging"].as<bool>();
    this->enableDebugMode = this->flyMSYamlNode["enableDebugMode"].as<bool>();
    this->enableAutonomy = this->flyMSYamlNode["enableAutonomy"].as<bool>();
    this->enableHeadlessMode = this->flyMSYamlNode["enableHeadlessMode"].as<bool>();
    
	//Imu mounting offsets
    this->pitchOffsetDegrees = this->flyMSYamlNode["pitchOffsetDegrees"].as<float>();
    this->rollOffsetDegrees = this->flyMSYamlNode["rollOffsetDegrees"].as<float>();
    this->yawOffsetDegrees = this->flyMSYamlNode["yawOffsetDegrees"].as<float>();
 
 	//Altitude Controller gains
 	this->alt_KP = this->flyMSYamlNode["alt_KP"].as<float>();
 	this->alt_KI = this->flyMSYamlNode["alt_KI"].as<float>();
 	this->alt_KD = this->flyMSYamlNode["alt_KD"].as<float>();


 	//Outler Loop Controller Gains For Pitch
 	this->pitch_KP = this->flyMSYamlNode["pitch_KP"].as<float>();
 	this->pitch_KI = this->flyMSYamlNode["pitch_KI"].as<float>();
 	this->pitch_KD = this->flyMSYamlNode["pitch_KD"].as<float>();

 	//Inner Loop Controller Gains For Pitch
 	this->Dpitch_KP = this->flyMSYamlNode["Dpitch_KP"].as<float>();
 	this->Dpitch_KI = this->flyMSYamlNode["Dpitch_KI"].as<float>();
 	this->Dpitch_KD = this->flyMSYamlNode["Dpitch_KD"].as<float>();

 	//Outler Loop Controller Gains For Roll
 	this->roll_KP = this->flyMSYamlNode["roll_KP"].as<float>();
 	this->roll_KI = this->flyMSYamlNode["roll_KI"].as<float>();
 	this->roll_KD = this->flyMSYamlNode["roll_KD"].as<float>();

 	//Inner Loop Controller Gains For Roll
 	this->Droll_KP = this->flyMSYamlNode["Droll_KP"].as<float>();
 	this->Droll_KI = this->flyMSYamlNode["Droll_KI"].as<float>();
 	this->Droll_KD = this->flyMSYamlNode["Droll_KD"].as<float>();

 	//Inner Loop Controller Gains For Yaw
 	this->yaw_KP = this->flyMSYamlNode["yaw_KP"].as<float>();
 	this->yaw_KI = this->flyMSYamlNode["yaw_KI"].as<float>();
 	this->yaw_KD = this->flyMSYamlNode["yaw_KD"].as<float>();
 	
 	//Max Yaw Rate
 	this->maxYawRate = this->flyMSYamlNode["maxYawRate"].as<float>();

 	//Min and Max throttle values
 	this->minThrottle = this->flyMSYamlNode["minThrottle"].as<float>();
 	this->maxThrottle = this->flyMSYamlNode["maxThrottle"].as<float>();

}
