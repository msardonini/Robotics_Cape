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

void onSignalReceived(int signo)
{
	handler.shutdownPruHandler(signo);
}


void initSignalHandler(pruHandler* handler)
{
     signal(SIGINT, onSignalReceived);
     signal(SIGKILL, onSignalReceived);
     signal(SIGHUP, onSignalReceived);
}




int main(int argc, char* argv[])
{

	handler.init_pru_handler();

	while(1)
	{
		sleep(1);
	}



}