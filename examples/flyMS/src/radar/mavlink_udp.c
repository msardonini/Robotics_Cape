/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 ****************************************************************************/
/*
 This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.
 
 
 I compiled this program sucessfully on Ubuntu 10.04 with the following command
 
 gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c
 
 the rt library is needed for the clock_gettime on linux
 */
/* These headers are for QNX, but should all be standard on unix/linux */
#define true 1
#define false 0


#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
int baudrate = 115200;

int init_uart(int* fd_serial);
uint64_t microsSinceEpoch();

int main(int argc, char* argv[])
{
	int fd_serial = 0;
	init_uart(&fd_serial);
	static mavlink_radar_filtered_t filter_struct;
	static mavlink_radar_t radar_struct;

	//struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	mavlink_message_t msg;
	int i = 0;
	
	
	for (;;) 
    {
		memset(buf, 0, BUFFER_LENGTH);
		//recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		recsize = read(fd_serial, buf, sizeof(buf));

		if (recsize > 0)
      	{
      
			// Something received - print out all bytes and parse packet
			memset(&msg,0,sizeof(msg));
			mavlink_status_t status;
			
			//printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (i = 0; i < recsize; ++i)
			{
				//printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					//printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
					
					printf("msg id %d\n", msg.msgid);
					if (msg.msgid == 181)
					{
						mavlink_msg_radar_filtered_decode(&msg, &filter_struct);
					 	printf("\nDecoded Filter pos1 %f vel1 %f \n",filter_struct.distance[0],filter_struct.confidence[0]);
					}
					if (msg.msgid == 180)
					{
						mavlink_msg_radar_decode(&msg, &radar_struct);
					 	printf("\nDecoded Radar pos1 %u\n",radar_struct.distance[0]);
					}
								

				}
			}
			//printf("\n");
		}
		memset(buf, 0, BUFFER_LENGTH);
		usleep(100000); // Sleep < one second
    }
}


/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}
#else
uint64_t microsSinceEpoch()
{
	
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}
#endif



int init_uart(int* fd_serial)
{
	int okay = false;
	// Open serial port

	*fd_serial = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY );
	// Check for Errors
	if (*fd_serial == -1)
        printf( "ERROR! Could not open port \n" );
	

	//fcntl(this->fd, F_SETFL, flags | O_NONBLOCK);
	int flags = fcntl(*fd_serial, F_GETFL, 0);
	fcntl(*fd_serial, F_SETFL, flags | O_NONBLOCK);
	
if(!isatty(*fd_serial))
        printf( "ERROR! Port is not a serial port \n" );
    else
    {
	    // Read file descritor configuration
	    struct termios config;
	    if(tcgetattr(*fd_serial, &config) < 0)
            printf("Could not read configuration of file descriptor ");

	    // Input flags - Turn off input processing
	    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	    					INLCR | PARMRK | INPCK | ISTRIP | IXON);

	    // Output flags - Turn off output processing
	    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	    					 ONOCR | OFILL | OPOST);

	    #ifdef OLCUC
	    	config.c_oflag &= ~OLCUC;
	    #endif
	    #ifdef ONOEOT
	    	config.c_oflag &= ~ONOEOT;
	    #endif

	    // No line processing:
	    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	    // Turn off character processing
	    config.c_cflag &= ~(CSIZE | PARENB);
	    config.c_cflag |= CS8;

	    // One input byte is enough to return from read()
	    // Inter-character timer off
	    config.c_cc[VMIN]  = 1;
	    config.c_cc[VTIME] = 10; // was 0

    	//int flags = fcntl(this->fd, F_GETFL, 0);
		//fcntl(this->fd, F_SETFL, flags | O_NONBLOCK);

	    // Get the current options for the port
	    //struct termios options;
	    //tcgetattr(this->fd, &options);

	    // Apply baudrate
        int baudOkay = true;
	    switch (baudrate)
	    {
	    	case 1200:
	    		if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
                    baudOkay = false;
	    		break;
	    	case 1800:
	    		cfsetispeed(&config, B1800);
	    		cfsetospeed(&config, B1800);
	    		break;
	    	case 9600:
	    		cfsetispeed(&config, B9600);
	    		cfsetospeed(&config, B9600);
	    		break;
	    	case 19200:
	    		cfsetispeed(&config, B19200);
	    		cfsetospeed(&config, B19200);
	    		break;
	    	case 38400:
	    		if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
                    baudOkay = false;
	    		break;
	    	case 57600:
	    		if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
                    baudOkay = false;
	    		break;
	    	case 115200:
	    		if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
                    baudOkay = false;
	    		break;
	    	case 460800:
	    		if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
                    baudOkay = false;
	    		break;
	    	case 921600:
	    		if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
                    baudOkay = false;
	    		break;
	    	default:
                baudOkay = false;
	    		printf("Desired baud rate %d could not be set", baudrate);
	    		break;
	    }

        if (!baudOkay)
            printf("Could not set desired baud rate of %d baud", baudrate);
        else if(tcsetattr(*fd_serial, TCSAFLUSH, &config) < 0)
            printf( "Could not set configuration of fd %d", *fd_serial);
        else
            okay = true;  // finally

	}

	return okay;		
}