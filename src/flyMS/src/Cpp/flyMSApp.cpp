/**
 * @file flyMS.cpp
 * @brief Application entry point of the flyMS program. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */



#include "flyMS.hpp"

int parseInputs(int argc, char* argv[], flyMSParams* configParams);


int main(int argc, char *argv[])
{
	//Initialize the cape and beaglebone hardware
	if(rc_initialize()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}

	// Read the command line arguments and config file inputs
	flyMSParams configParams;
	parseInputs(argc, argv, &configParams);

	//Initialize the flight hardware
	// startupRoutine();

	flyMS fly(configParams);

	fly.startupRoutine();

	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) 
	{
		sleep(1);
	}

	rc_cleanup();
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
	configParams->isDebugMode |= isDebugMode;


	return 0;
}