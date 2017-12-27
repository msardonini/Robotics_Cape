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

#ifndef GPS_H
#define GPS_H

typedef struct GPS_data_t{
//	char 	band_aid_buff[500];
	int 	GPS_file;	//file functions use to access GPS data
	float 	deg_longitude;
	float 	deg_latitude;
	float 	gps_altitude;
	float 	meters_lat;
	float 	meters_lon;
	float 	speed;
	float 	direction;
	double 	min_longitude;
	double 	min_latitude;
	float 	HDOP;
	int 	GPS_fix;
	char 	GGAbuf[225];
	char 	VTGbuf[225];
	double	pos_lon, pos_lat;
	int 	GPS_init_check;
	int 	GPS_fix_check;
	pthread_t gps_thread;
}GPS_data_t;


/******************** GPS STUFF ********************/
#define BAUDRATE B4800
#define BAUDRATE2 B57600
#define MODEMDEVICE "/dev/ttyO1" 


GPS_data_t* get_GPS_pointer();
float get_NMEA_field(int field, char buf[], int comma[]);
int join_GPS_thread(GPS_data_t *GPS_data);
int GPS_init(GPS_data_t * GPS_data);
uint8_t is_new_GPS_data();
void* GPS_data_watcher(void *ptr);
void read_raw_gps(char *buf, GPS_data_t *GPS_data);
int GPS_handler(control_variables_t *control, GPS_data_t *GPS_data);



#endif
