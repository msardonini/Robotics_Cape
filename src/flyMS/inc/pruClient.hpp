/**
 * @file flyMS.cpp
 * @brief flyMS program source code. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#ifndef PRU_CLIENT_H
#define PRU_CLIENT_H


//System Includes
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

//C++ includes
#include <thread>
#include <mutex>
#include <vector>


//Package Includes
#include "roboticscape.h"

//Locally used macros
#define PRU_PID_FILE "/var/run/pru_handler.pid"

//Struct to send the PRU data over with
typedef struct pru_client_data_t{

 
}pru_client_data_t;


class pruClient
{

public:
	
	// Default Constructor
	pruClient();

	//Default Destructor
	~pruClient();

	int setSendData(std::vector<float> u);

private:

	int	pruSender();

	//Boolean for the status of the program
	bool isRunning;

	//Thread and mutex objects
	std::thread pruSenderThread;
	std::mutex pruSenderMutex;
	//Data we send over the network interface
	uint8_t send_flag;
	vector<float> u;

	//Socket variables
	int sockfd;
    std::vector<char> sendBuff;
    struct sockaddr_in serv_addr;

};



#endif //PRU_CLIENT_H