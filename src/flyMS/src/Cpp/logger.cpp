/**
 * @file logger.cpp
 * @brief logging module for the flight controller 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */



#include "logger.hpp"

logger::logger(){}
logger::~logger(){}


int logger::createLogFiles()
{
    int nInt=1;
    char nStr[6];
    struct stat st = {0};

    sprintf(nStr,"%03d",nInt);
    std::string filePath(FLYMS_ROOT_DIR + std::string("/log/run") + std::string(nStr));

    //Find the next run number folder that isn't in use
    while(!stat(filePath.c_str(), &st))
    {
        nInt++;
        sprintf(nStr,"%03d",nInt);
        filePath.replace(0, filePath.length(), FLYMS_ROOT_DIR + std::string("/log/run") + std::string(nStr));
    }

    //Create files and verify that all of them were made successfully
    
    bool fileExistsCheck = true;
    this->dataLogFid = std::ofstream(filePath + "/logger.txt");
    fileExistsCheck &= this->fileExists(filePath + "/logger.txt");

    this->errorFid = std::ofstream(filePath + "/errorLog.txt");
    fileExistsCheck &= this->fileExists(filePath + "/errorLog.txt");

    this->configFid = std::ofstream(filePath + "/config.txt");
    fileExistsCheck &= this->fileExists(filePath + "/config.txt");

    this->consoleLogFid = std::ofstream(filePath + "/consoleLog.txt");
    fileExistsCheck &= this->fileExists(filePath + "/consoleLog.txt");

    if (!fileExistsCheck)
        return -1;

    // //take the loaded config file and put into the class varialbes
    // #define X(type, fmt, name, default) this->name = this->flyMSYamlNode[#name].as<type>();
    // CORE_CONFIG_TABLE
    // #undef X
    
    //Write the Header to the Data log file
    #define X(type, fmt, name) this->dataLogFid << #name << ",";
    CORE_LOG_TABLE
    #undef X
    this->dataLogFid << std::endl;

    return 0;
}

int logger::writeToLog()
{

    // std::ofstream configfile(filename);

    // //Start creating the config file
    // configfile << "---" <<std::endl;
    // configfile << "flyMSParams: " <<std::endl;

    #define X(type, fmt, name) this->dataLogFid << this->name << ",";
    CORE_LOG_TABLE
    #undef X
    this->dataLogFid << std::endl;

    return 0;
}


// bool fileExists(const std::string filename)
bool logger::fileExists( std::string filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}