/**
 * @file flyMS.cpp
 * @brief Application entry point of the flyMS program. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

//System Includes
#include <signal.h>

//Our Includes
#include "flyMS.hpp"

int parseInputs(int argc, char* argv[], flyMSParams* configParams);
void initSignalHandler();
void onSignalReceived(int signo);


int main(int argc, char *argv[])
{
	//Enable the signal handler so we can exit cleanly on SIGINT
	initSignalHandler();

	// Read the command line arguments and config file inputs
	flyMSParams configParams;
	parseInputs(argc, argv, &configParams);

	//Initialize the flight hardware
	// startupRoutine();

	rc_set_state(RUNNING);
	flyMS fly();
	// flyMS fly(configParams.config);
	// fly.startupRoutine();

	while (rc_get_state() != EXITING) 
	{
		sleep(1);
	}

	// rc_cleanup();
	printf("exiting now!\n");
	return 0;
}


int parseInputs(int argc, char* argv[], flyMSParams* configParams)
{
	bool isDebugMode = false;

	//Parse the command line arguments
	int in;
	while ((in = getopt(argc, argv, "dr:h")) != -1)
	{
		switch (in)
		{
			case 'd': 
				isDebugMode = true;
				printf("Running in Debug mode \n");
				break;
			case 'r': 
				//Run Program in replay mode, need to provide filepath to log file
				//TODO: Add command line arg parser
				printf("Running in Replay mode \n");
				break;
			case 'h': 
				//Display the command line help options
                //TODO: make this function
                // print_usage();
				return 0;
			default:
				printf("Invalid Argument \n");
				return -1;
		}
	}

	std::string configFileName(FLYMS_ROOT_DIR);
	configFileName.append("/config/flyMSConfig.yaml");

	configParams->loadConfigFile(configFileName);


	//Merge the common parameters between the config file and the command line inputs
	configParams->config.isDebugMode |= isDebugMode;

	return 0;
}

void onSignalReceived(int signo)
{
	rc_set_state(EXITING);
}


void initSignalHandler()
{
     signal(SIGINT, onSignalReceived);
     signal(SIGKILL, onSignalReceived);
     signal(SIGHUP, onSignalReceived);
}