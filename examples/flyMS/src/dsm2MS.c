#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "../../../libraries/roboticscape.h"

//// Spektrum DSM2 RC Radio
#define RC_CHANNELS 9
#define CONFIG_DIRECTORY "/var/lib/roboticscape/"
#define DSM2_CAL_FILE	"dsm.cal"
#define UART4_PATH "/dev/ttyO4"

/*************************DSM2 Stuff ***************/
//// DSM2 Spektrum RC radio functions
int is_new_dsm2_dataMS();
int initialize_dsm2MS();
float get_dsm2_ch_normalizedMS(int channel);
void* uart4_checkerMS(void *ptr); //background thread
int get_dsm2_ch_rawMS(int channel);

// DSM2 Spektrum radio & UART4
int rc_channels[RC_CHANNELS];
int rc_maxes[RC_CHANNELS];
int rc_mins[RC_CHANNELS];
int tty4_fd;
int new_dsm2_flag;


////  DSM2  Spektrum  RC Stuff  
int initialize_dsm2MS(){
	//if calibration file exists, load it and start spektrum thread
	FILE *cal;
	char file_path[100];

	// construct a new file path string
	strcpy (file_path, CONFIG_DIRECTORY);
	strcat (file_path, DSM2_CAL_FILE);
	
	// open for reading
	cal = fopen(file_path, "r");

	if (cal == NULL) {
		printf("\nDSM2 Calibration File Doesn't Exist Yet\n");
		printf("Use calibrate_dsm2 example to create one\n");
		return -1;
	}
	else{
		int i;
		for(i=0;i<RC_CHANNELS;i++){
			fscanf(cal,"%d %d", &rc_mins[i],&rc_maxes[i]);
			//printf("%d %d\n", rc_mins[i],rc_maxes[i]);
		}
		printf("DSM2 Calibration Loaded\n");
	}
	fclose(cal);
	pthread_t uart4_thread;
	pthread_create(&uart4_thread, NULL, uart4_checkerMS, (void*) NULL);
	printf("DSM2 Thread Started\n");
	return 0;
}

float get_dsm2_ch_normalizedMS(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & %d",RC_CHANNELS);
		return -1;
	}
	float range = rc_maxes[ch-1]-rc_mins[ch-1];
	if(range!=0) {
		new_dsm2_flag = 0;
		float center = (rc_maxes[ch-1]+rc_mins[ch-1])/2;
		return 2*(rc_channels[ch-1]-center)/range;
	}
	else{
		return 0;
	}
}

int get_dsm2_ch_rawMS(int ch){
	if(ch<1 || ch > RC_CHANNELS){
		printf("please enter a channel between 1 & %d",RC_CHANNELS);
		return -1;
	}
	else{
		new_dsm2_flag = 0;
		return rc_channels[ch-1];
	}
}

int is_new_dsm2_dataMS(){
	return new_dsm2_flag;
}

// uncomment this define to print raw data for debugging
// #define DEBUG_DSM2
void* uart4_checkerMS(void *ptr){
	
	//set up sart/stop bit and 115200 baud
	struct termios config;
	memset(&config,0,sizeof(config));
	config.c_iflag=0;
	config.c_iflag=0;
    config.c_oflag=0;
    config.c_cflag= CS8|CREAD|CLOCAL;   // 8n1, see termios.h for more info
    config.c_lflag=0;
    config.c_cc[VTIME]=0; // no timeout condition
	config.c_cc[VMIN]=1;  // only return if something is in the buffer
	
	// open for blocking reads
	if ((tty4_fd = open (UART4_PATH, O_RDWR | O_NOCTTY)) < 0) {
		printf("error opening uart4\n");
	}
	// Spektrum and Oragne recievers are 115200 baud
	if(cfsetispeed(&config, B115200) < 0) {
		printf("cannot set uart4 baud rate\n");
		return NULL;
	}

	if(tcsetattr(tty4_fd, TCSAFLUSH, &config) < 0) { 
		printf("cannot set uart4 attributes\n");
		return NULL;
	}

	while(rc_get_state()!=EXITING){
		char buf[64]; // large serial buffer to catch doubled up packets
		int i,j;
		
		memset(&buf, 0, sizeof(buf)); // clear buffer
		
		i = read(tty4_fd,&buf,sizeof(buf)); // blocking read
		
		#ifdef DEBUG_DSM2
		printf("recieved %d bytes, ", i);
		#endif
		
		if(i<0){
			#ifdef DEBUG_DSM2
			printf("error, read returned -1\n");
			#endif
			
			goto end;
		}
		else if(i>16){
			#ifdef DEBUG_DSM2
			printf("error: read too many bytes.\n");
			#endif
			goto end;
		}
		
		//incomplete packet read, wait and read the rest of the buffer
		else if(i<16){	
			// packet takes about 1.4ms at 115200 baud. sleep for 2.0
			usleep(2000); 
			//read the rest
			j = read(tty4_fd,&buf[i],sizeof(buf)-i); // blocking read
			
			#ifdef DEBUG_DSM2
				printf("then %d bytes, ", j);
			#endif
			
			//if the rest of the packet failed to arrive, error
			if(j+i != 16){
				#ifdef DEBUG_DSM2
				printf("error: wrong sized packet\n");
				#endif
				
				goto end;
			}
		}
		// if we've gotten here, we have a 16 byte packet
		
		
		// first check if it's from an Orange TX and read channels in order
		// 8 and 9 ch dsmx radios end in 0xFF too, but those are also 0xFF
		// on bytes 5-13
		if(buf[14]==0xFF && buf[15]==0xFF && buf[13]!=0xFF){
			// i from 1 to 7 to get last 6 words of packet
			// first word contains lost frames so skip it
			for(i=1;i<=7;i++){
				int16_t value;
				
				// merge bytes
				value = buf[i*2]<<8 ^ buf[(i*2)+1];
				 
				// on Orange tx, each raw channel is 1000 larger than the last
				// remove this extra 1000 to get back to microseconds
				value -= 1000*(i-2);
				// rc_channels is 0 indexed, so i-1
				rc_channels[i-1] = value;
				#ifdef DEBUG_DSM2
				printf("%d %d  ", i, rc_channels[i-1]);
				#endif
			}
		}
		
		// must be a Spektrum packet instead, read a channel at a time
		else{
			unsigned char ch_id;
			int16_t value;
			
			#ifdef DEBUG_DSM2
			printf("ch_id: ");
			#endif
			
			// packet is 16 bytes, 8 words long
			// first word doesn't have channel data, so iterate through last 7
			for(i=1;i<=7;i++){
				// in dsmX 8 and 9 ch radios, unused words are 0xFF
				// skip if one of them
				if(buf[2*i]!=0xFF || buf[(2*i)+1]!=0xFF){
					// grab channel id from first byte
					ch_id = (buf[i*2]&0b01111100)>>2; 
					// grab value from least 11 bytes
					value = ((buf[i*2]&0b00000011)<<8) + buf[(2*i)+1];
					value += 1000; // shift range so 1500 is neutral
					
					#ifdef DEBUG_DSM2
					printf("%d %d  ",ch_id,value);
					// printf(byte_to_binary(buf[i*2]));
					// printf(" ");
					// printf(byte_to_binary(buf[(i*2)+1]));
					// printf("   ");
					#endif
					
					if(ch_id>9){
						#ifdef DEBUG_DSM2
						printf("error: bad channel id\n");
						#endif
						
						goto end;
					}
					// throttle is channel 1 always
					// ch_id is 0 indexed, though
					// and rc_channels is 0 indexed, so also 0
					rc_channels[ch_id] = value;
				}
			}
		}
		// indicate new a new packet has been processed
		new_dsm2_flag=1;
		
		#ifdef DEBUG_DSM2
		printf("\n");
		#endif
		
end:
		// wait and clear the buffer
		usleep(2000);
		tcflush(tty4_fd,TCIOFLUSH);
		// packets arrive in every 11ms or 22ms
		// to account for possible wait above, sleep for 6ms
		usleep(6000);
	}
	return NULL;
}




