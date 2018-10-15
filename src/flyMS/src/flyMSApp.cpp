/**
 * @file flyMS.cpp
 * @brief Application entry point of the flyMS program. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */



#include "flyMS.hpp"
#include "flyMS_common.h"


int main(int argc, char *argv[])
{

	//Initialize the cape and beaglebone hardware
	if(rc_initialize()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}

	//Parse the command line arguments
	int in;
	while ((in = getopt(argc, argv, "dr:h")) != -1)
	{
		switch (in)
		{
			case 'd': 
				flyMSData.flight_config.enable_debug_mode = 1;
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

	//Initialize the flight hardware
	startupRoutine();

	flyMS fly;

	rc_set_state(RUNNING);
	while (rc_get_state() != EXITING) 
	{
		sleep(1);
	}

	rc_cleanup();
	return 0;


}