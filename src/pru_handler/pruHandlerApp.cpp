/**
 * @file pruHandlerApp.hpp
 * @brief Process which communicates with the PRU that controlls quadrotor ESCs. 
 		This process connects to applications via TCP packets over localhost and 
 		relays the data to the PRU. If no program is active, zero throttle is sent. 
 *
 * @author Mike Sardonini
 * @date 11/10/2018
 */



#include "pruHandler.hpp"

pruHandler handler;
bool isRunning = true;


void onSignalReceived(int signo)
{
	handler.shutdownPruHandler(signo);
	isRunning = false;
}


void initSignalHandler()
{
     signal(SIGINT, onSignalReceived);
     signal(SIGKILL, onSignalReceived);
     signal(SIGHUP, onSignalReceived);
}


int main(int argc, char* argv[])
{
	int count = 0;
	while( count < 60)
	{
		if(access("/sys/class/remoteproc/remoteproc1/state", F_OK ) != 0)
		{
			printf("ERROR:  pru-rproc driver not loaded\n");
		} 
		else break;
		
		sleep(1);
		count++;
	}
	
	initSignalHandler();

	handler.init_pru_handler();

	while(isRunning)
	{
		sleep(1);
	}



}
