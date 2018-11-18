/**
 * @file logger.cpp
 * @brief logging module for the flight controller 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */



#include "logger.hpp"

logger::logger()
{
    //  Determine if this process started from a terminal
    //      If so, have the console output redirected to the terminal

    if(isatty(fileno(stdout)))
        this->isRunningConsole = true;
}


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

    //Make a new folder to hold the logged data
    mkdir(filePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    //Create files and verify that all of them were made successfully
    
    bool fileExistsCheck = true;
    this->dataLogFid.open(filePath + "/logger.txt");
    fileExistsCheck &= this->fileExists(filePath + "/logger.txt");

    this->errorFid.open(filePath + "/errorLog.txt");
    fileExistsCheck &= this->fileExists(filePath + "/errorLog.txt");

    //Copy the yaml file over to the log directory to save the config params for this run
    std::string copyConfigstring ("cp ");
    copyConfigstring.append(FLYMS_ROOT_DIR);
    copyConfigstring.append("/config/flyMSConfig.yaml ");
    copyConfigstring.append(filePath);
    system(copyConfigstring.c_str());


    this->consoleLogFid.open(filePath + "/consoleLog.txt");
    fileExistsCheck &= this->fileExists(filePath + "/consoleLog.txt");

    this->flyMS_printf("creating log files in %s\n",  filePath.c_str());
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

int logger::flyMS_printf ( const char * format, ... )
{
    //Add a lock in case other threads try printing stuff at the same time
    std::lock_guard<std::mutex> lock(this->printMutex);

    //Needed for standard use of vsprtinf
    va_list args;
    va_start (args, format);

    //Allocate buffer for output
    char *buffer = (char*)malloc(vsnprintf (NULL, 0,format, args) + 1);
    
    //Copy the buffer
    vsprintf (buffer,format, args);
    
    //If running from a console also publist to stdout
    if(this->isRunningConsole)
        fprintf(stdout, buffer);

    //Write the character string to file
    if (this->consoleLogFid.is_open())
        this->consoleLogFid << buffer;

    va_end (args);

    //Free our dynamically allocated string buffer
    free(buffer);
}

int logger::writeToLog(state_t *bodyState, controller_t *controller, setpoint_t *setpoint)
{

    //Get the time for timestamp
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    //Set the values for all the data we want to record
    this->timeMicroseconds =  tv.tv_sec*(uint64_t)1E6 + tv.tv_nsec/(uint64_t)1E3;
    this->roll = bodyState->euler(0);
    this->pitch = bodyState->euler(1);
    this->yaw = bodyState->euler(2);
    this->d_roll = bodyState->eulerRate(0);
    this->d_pitch = bodyState->eulerRate(1);
    this->d_yaw = bodyState->eulerRate(2);
    this->u_1 = controller->u[0];
    this->u_2 = controller->u[1];
    this->u_3 = controller->u[2];
    this->u_4 = controller->u[3];
    this->throttle = setpoint->throttle;
    this->uroll = controller->u_euler[0];
    this->upitch = controller->u_euler[1];
    this->uyaw = controller->u_euler[2];
    this->roll_ref = setpoint->euler_ref[0];
    this->pitch_ref = setpoint->euler_ref[1];
    this->yaw_ref = setpoint->euler_ref[2];
    this->Aux = setpoint->Aux[0];



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