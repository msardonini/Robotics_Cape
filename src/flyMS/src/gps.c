/*
Copyright (c) 2014, Mike Sardonini
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

#ifdef __cplusplus
{
#endif


#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
	
#include "flyMS_common.h"
#include "gps.h"
#include "imu_handler.h"

int GPS_handler(control_variables_t *control, GPS_data_t *GPS_data)
{
	static int First_Iteration_GPS = 1;

	if(is_new_GPS_data())
	{
		log_GPS_data(GPS_data, control->time);

		if (First_Iteration_GPS==1  && GPS_data->GPS_fix==1){
			// control->initial_pos_lat=GPS_data.meters_lat;
			// control->initial_pos_lon=GPS_data.meters_lon;
			First_Iteration_GPS=0;
			GPS_data->GPS_fix_check=1;
			printf("First Iteration GPS\n");
		}
		if(GPS_data->HDOP<4 && GPS_data->GPS_fix==1){
			// GPS_data->pos_lat=GPS_data->meters_lat-control.initial_pos_lat;
			// GPS_data->pos_lon=GPS_data->meters_lon-control.initial_pos_lon;
		}

		update_ekf_gps(control, GPS_data);
	}
	return 0;
}


/**		Globals 		*/
uint8_t GGA_flag;
uint8_t VTG_flag;
uint8_t GPS_data_flag;


int GPS_init(GPS_data_t * GPS_data){
	int res;
	struct termios newtio;
	char buf[255];
    // Load the pin configuration
    //int ret = system("echo uart2 > /sys/devices/bone_capemgr.9/slots");
    /* Open modem device for reading and writing and not as controlling tty
       because we don't want to get killed if linenoise sends CTRL-C. */
    GPS_data->GPS_file = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
    if (GPS_data->GPS_file < 0) { perror(MODEMDEVICE); exit(-1); }

	chbaud:
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
       CRTSCTS : output hardware flow control (only used if the cable has
                 all necessary lines. See sect. 7 of Serial-HOWTO)
       CS8     : 8n1 (8bit,no parity,1 stopbit)
       CLOCAL  : local connection, no modem contol
       CREAD   : enable receiving characters */
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    /* IGNPAR  : ignore bytes with parity errors
       otherwise make device raw (no other input processing) */
    newtio.c_iflag = IGNPAR;

    /*  Raw output  */
    newtio.c_oflag = 0;

    /* ICANON  : enable canonical input
       disable all echo functionality, and don't send signals to calling program */
    newtio.c_lflag = ICANON;
    /* now clean the modem line and activate the settings for the port */
    tcflush(GPS_data->GPS_file, TCIFLUSH);
    tcsetattr(GPS_data->GPS_file,TCSANOW,&newtio);
	
	// Initialize file descriptor sets
	fd_set read_fds, write_fds, except_fds;
	FD_ZERO(&read_fds);
	FD_ZERO(&write_fds);
	FD_ZERO(&except_fds);
	FD_SET(GPS_data->GPS_file, &read_fds);

	// Set timeout to 1.0 seconds
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	// Wait for input to become ready or until the time out; the first parameter is
	// 1 more than the largest file descriptor in any of the sets
	if (select(GPS_data->GPS_file + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1)
	{
		
		printf("\n Changing Baud to 57600. Look for >>$PTNLRPT,A*3D<< for successful transmission\n");
		write(GPS_data->GPS_file,"$PTNLSPT,057600,8,N,1,4,4*12\r\n",30); //Change Baud Rade to 57600
		usleep(300000);
		res = read(GPS_data->GPS_file, buf, 255);
		buf[res] = 0;  	
		//printf("No Fix, Message transmitted from Module is: %s",buf);
		
	}
	else
	{
		// timeout or error
		//printf("Successfully taken off of 9800 Baud, Proceeding,\n");
	}
	
	//Now the device is reading faster at 57600 Baud, change settings on the beagleboard
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
    newtio.c_cflag = BAUDRATE2 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;   
	newtio.c_oflag = 0;
	newtio.c_lflag = ICANON;
    tcflush(GPS_data->GPS_file, TCIFLUSH);
    tcsetattr(GPS_data->GPS_file,TCSANOW,&newtio);
	
	struct timeval timeout2;
	timeout2.tv_sec = 1;
	timeout2.tv_usec = 0;
	/*** Make Sure that the GPS Unit is Reading, if not Disable *******/
	fd_set read_fds2, write_fds2, except_fds2;
	FD_ZERO(&read_fds2);
	FD_ZERO(&write_fds2);
	FD_ZERO(&except_fds2);
	FD_SET(GPS_data->GPS_file, &read_fds2);
	
		if (select(GPS_data->GPS_file + 1, &read_fds2, &write_fds2, &except_fds2, &timeout2) == 1)
	{
		printf("GPS init successful\n");
		}
	else
	{
		// timeout or error
		printf("GPS Failed, Contining on without GPS\n");
		return -1;
	}
	
	int argc = 0;
	char *argv[1];	
	//Check for user inputs between hot, warm, cold, or factory reset starts
	if(argc==2 && 0){
			if(!strcmp(argv[1],"hot")){	
				if(write(GPS_data->GPS_file, "$PTNLSRT,H,1,,*20<CR><LF>\r\n", 27)==27){
						printf("Hot Start Selected. \n\n Look for >>$PTNLRRT,A*3F<< for successful confirmation from device \n\n");
				} 
				else printf("Write Error \n");
			}
			else if(!strcmp(argv[1],"warm")){
					if(write(GPS_data->GPS_file, "$PTNLSRT,W,1,,*20<CR><LF>\r\n", 27)==27){
							printf("Warm Start Selected. \n\n Look for >>$PTNLRRT,A*3F<< for successful confirmation from device \n\n");
					} 
					else printf("Write Error \n");
			}
			else if(!strcmp(argv[1],"cold")){
				if(write(GPS_data->GPS_file, "$PTNLSRT,C,1,,*20<CR><LF>\r\n", 27)==27) { //Cold Start
					printf("Cold Start Selected. \n\n Look for >>$PTNLRRT,A*3F<< for successful confirmation from device \n\n");
				
				argc=1;				
				goto chbaud;
				}
			else printf("Write Error \n");
			}

			else if(!strcmp(argv[1],"factory")){
					if(write(GPS_data->GPS_file, "$PTNLSRT,F,1,,*10<CR><LF>\r\n", 27)==27){
						printf("Factory Reset Selected. \n\n Look for >>$PTNLRRT,A*3F<< for successful confirmation from device \n\n");
					}
					else printf("Write Error \n");
			}
		}
	
	
	usleep(50000);
	write(GPS_data->GPS_file,"$PTNLSNM,0005,01*52\r\n",21); //NMEA message to output GGA  & VTG only
	usleep(50000);
	//res = read(GPS_data->GPS_file, buf, 255);
    //buf[res] = 0;  		
	//printf("%s", buf);
	
	write(GPS_data->GPS_file,"$PTNLQBA*54\r\n",13); //antenna check
	//printf("Antenna query\n");
	usleep(50000); 
	//res = read(GPS_data->GPS_file, buf, 255);
    //buf[res] = 0;  		
	//printf("%s", buf);
	
	
	/*
	usleep(50000); 
	res = read(GPS_data->GPS_file, buf, 255);
    buf[res] = 0;  		
	printf("%s", buf);
	
	usleep(50000); 
	res = read(GPS_data->GPS_file, buf, 255);
    buf[res] = 0;  		
	printf("%s", buf);
	*/
	
	
	//Start the GPS thread
	pthread_create(&GPS_data->gps_thread, NULL, GPS_data_watcher, (void*) GPS_data);
	//printf("GPS Thread Started\n");
	return 0;
}


void* GPS_data_watcher(void *ptr){
	
	GPS_data_t *GPS_data = (GPS_data_t*)ptr;
	
	char buf[255];
  	int res, comma[15];
	char Deg_Lat_buf[12],Min_Lat_buf[12],Deg_Lon_buf[12],Min_Lon_buf[12];
	while(rc_get_state()!=EXITING){
		memset(buf,0,sizeof(buf));
		
		
		res = read(GPS_data->GPS_file, buf, 255);
		buf[res] = 0;
		//printf("%s\n",buf);
		//Clear all the buffers before reading any data
		memset(Deg_Lat_buf,0,sizeof(Deg_Lat_buf)); memset(Min_Lat_buf,0,sizeof(Min_Lat_buf)); 
		memset(Deg_Lon_buf,0,sizeof(Deg_Lon_buf)); memset(Min_Lon_buf,0,sizeof(Min_Lon_buf));
		memset(comma,0,sizeof(comma));
		int count=0, i=0;
		if(buf[3] == 'G') if(buf[4]=='G') if(buf[5]=='A'){		
			while(count<13){
				if(buf[i]==',') {
					comma[count]=i;
					count++;
				}
				i++;
			}
				
			if((buf[comma[5]+1]=='1') || (buf[comma[5]+1]=='2')){
				GPS_data->GPS_fix=1;
			}
			else
			{
				GPS_data->GPS_fix=0;
			}
							
			//Parse Latitude
			for(i=comma[1];i<=comma[2]-2;i++){
				if(i<=comma[1]+1){
					Deg_Lat_buf[i-comma[1]]=buf[i+1];
				}
				else{
					Min_Lat_buf[i-comma[1]-2]=buf[i+1];
				}
			}
			//Parse Longitude
			for(i=comma[3];i<=comma[4]-2;i++){
				if(i<=comma[3]+2){
					Deg_Lon_buf[i-comma[3]]=buf[i+1];
				}
				else{
					Min_Lon_buf[i-comma[3]-3]=buf[i+1];
				}
			}
			
			//Parse Horizontal Dilution of Presision, accuracy of measurements
			GPS_data->HDOP=get_NMEA_field(8, buf, comma);
			
			//Get altitude
			GPS_data->gps_altitude=get_NMEA_field(9, buf, comma);

			//Convert Strings to floats for use
			GPS_data->deg_latitude=strtod(Deg_Lat_buf,NULL);
			GPS_data->deg_longitude=strtod(Deg_Lon_buf,NULL);
			GPS_data->min_latitude=strtod(Min_Lat_buf,NULL);
			GPS_data->min_longitude=strtod(Min_Lon_buf,NULL);
			strcpy(GPS_data->GGAbuf,buf);
			if(GPS_data->min_latitude!=0) {
				
				GPS_data->meters_lat= (GPS_data->deg_latitude + 
									  GPS_data->min_latitude/60)*111000;  
				GPS_data->meters_lon= (GPS_data->deg_longitude +
									  GPS_data->min_longitude/60)* 111000 *
									  cos((GPS_data->deg_latitude + 
									  GPS_data->min_latitude/60)*DEG_TO_RAD);
				GGA_flag=1;
			}
			
		}
		if(buf[3] == 'V') if(buf[4]=='T') if(buf[5]=='G'){
			while(count<8){
				if(buf[i]==',') {
					comma[count]=i;
					count++;
				}
				i++;
			}
			
			//Get Speed from NMEA Message
			GPS_data->speed=get_NMEA_field(6, buf, comma);
			//Get Direction from NMEA Message
			GPS_data->direction=get_NMEA_field(1, buf, comma);

			
			strcpy(GPS_data->VTGbuf,buf);
			VTG_flag=1;	
		}
		
	//	printf("GGA: %d VTG %d Data ready: %d\n\n",GGA_flag,VTG_flag,GPS_data_flag);
		if(GGA_flag==1 && VTG_flag==1) {GPS_data_flag=1;} //Flag to note that GPS data is ready
	
		//printf("GGA flag %d VTG flag %d\n",GGA_flag, VTG_flag);
	
		usleep(45000); //GPS only updates at 1 Hz, but need to process 2 messages


	}
	printf("GPS Thread Ended\n");

	return NULL;
}

uint8_t is_new_GPS_data(){
	uint8_t tmp = GPS_data_flag;
	if (GPS_data_flag)
	{
		GGA_flag = 0;
		VTG_flag = 0;
		GPS_data_flag = 0;
	}
	return tmp;
}

float get_NMEA_field(int field, char buf[], int comma[]){
	int i;
	char value_buf[10];
	memset(value_buf,0,sizeof(value_buf));
	for(i=comma[field-1];i<=comma[field]-2;i++){
		value_buf[i-comma[field-1]]=buf[i+1];
	}
	return strtod(value_buf,NULL);
}

void read_raw_gps(char *buf, GPS_data_t *GPS_data){
	int res;
	res = read(GPS_data->GPS_file, buf, 255);
	buf[res] = 0;
}

int join_GPS_thread(GPS_data_t *GPS_data){
	return pthread_join(GPS_data->gps_thread, NULL);
}


#ifdef __cplusplus
}
#endif
