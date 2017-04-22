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
int GPS_init(int argc, char *argv[], GPS_data_t * GPS_data);
uint8_t is_new_GPS_data();
void* GPS_data_watcher(void *ptr);
void read_raw_gps(char *buf, GPS_data_t *GPS_data);


#endif