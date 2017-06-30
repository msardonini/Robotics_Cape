/*******************************************************************************
* rc_calibrate_escs.c
*
* James Strawson 2016
* Typical brushless speed controllers (ESCs) accept a variety of pulse widths,
* usually from 900-2100 microseconds. Before using them, you must calibrate the
* ESC so it knows what pulse widths correspond to minimum and maximum throttle.
* Typically ESCs go into calibration mode by applying any pulse width greater 
* than roughly 1000us when it is powered on. Once in calibration mode, the user
* then applies max and min pulse widths briefly before turning off the signal to
* exit calibration mode. This is typically done with the throttle stick on your 
* RC transmitter.
*
* The calibrate_escs example assists you with this process by sending the right 
* pulse widths. Follow the instructions that are displayed in the console when 
* you execute the program.
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

typedef struct pru_client_data_t{
         uint8_t         send_flag;
         float           u[4];
 
}pru_client_data_t;
void *pru_sender(void* ptr);

float width; // global variable for normalized pulse width to send
pru_client_data_t pru_client_data;

// background thread to send pulses at 50hz to ESCs
void *send_pulses(void *params){
	int i;
	while(rc_get_state()!=EXITING){
//		rc_send_esc_pulse_normalized_all(width);
		for (i=0; i<4; i++) pru_client_data.u[i] = width;
		pru_client_data.send_flag = 1;
		rc_usleep(20000);
	}
	return 0;
}

int main(){
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	pthread_t pru_sender_thread;

	pthread_create(&pru_sender_thread, NULL, pru_sender, &pru_client_data);

	printf("\nDISCONNECT PROPELLERS FROM MOTORS\n");
	printf("DISCONNECT POWER FROM ESCS\n");
	printf("press enter to start sending max pulse width\n");
	if(rc_continue_or_quit()<1){
		printf("aborting calibrate_escs\n");
		goto END;
	}

	//Send full throttle until the user hits enter
	width = 1;
	pthread_t  send_pulse_thread;
	pthread_create(&send_pulse_thread, NULL, send_pulses, (void*) NULL);
	printf("\n");
	printf("Now reapply power to the ESCs.\n");
	printf("Press enter again after the ESCs finish chirping\n");
	rc_set_led(GREEN,1);
	if(rc_continue_or_quit()<1){
		printf("aborting calibrate_escs\n");
		goto END;
	}

	// now set lower bound
	printf("\n");
	printf("Sending minimum width pulses\n");
	printf("Press enter again after the ESCs chirping to finish calibration\n");
	width = 0;
	if(rc_continue_or_quit()<1){
		printf("aborting rc_calibrate_escs\n");
		goto END;
	}
	

	// cleanup and close
	printf("\nCalibration complete, check with rc_test_servos\n");
END:
	rc_set_state(EXITING); // this tells the send_pulses thread to stop
	pthread_join(send_pulse_thread, NULL); // wait for it to stop
	rc_cleanup();
	return 0;
}


void* pru_sender(void* ptr){
	
	pru_client_data_t *client_data = (pru_client_data_t*) ptr;
	int sockfd = 0;
    char sendBuff[16];
    struct sockaddr_in serv_addr; 

    memset(sendBuff, '0', sizeof(sendBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket \n");
        return NULL;
    } 

    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5000); 

   // if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
    if(inet_pton(AF_INET, "127.0.0.1" , &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return NULL;
    } 

    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return NULL;
    } 	
	uint16_t tmp16 = 0x0000;
	int i;
	printf("made it to the while \n");
    while (rc_get_state()!=EXITING)
    {
		if(client_data->send_flag)
		{				
			sendBuff[0] = 0xAA;
			sendBuff[1] = 0xBB;
			sendBuff[10] = 0xEE;
			sendBuff[11] = 0xFF;
			
			for (i = 0; i < 4 ; i++) 
			{
				if (client_data->u[i] == 0.0f) tmp16 = 0;
				else tmp16 = (uint16_t)(client_data->u[i]*65536.0f)-1;
				
				sendBuff[2*i+2] = tmp16 & 0xFF;
				sendBuff[2*i+2] = tmp16 >> 8;
			}			
			write(sockfd, sendBuff, sizeof(sendBuff)-1);
			client_data->send_flag = 0;
		}
		usleep(2500); //run at 400hz just to check
    }
	
	//Issue the shutdown command to the pru handler
	for (i = 0; i < 4; i++)
	{
		sendBuff[0] = 0xAA;
		sendBuff[1] = 0xBB;
		sendBuff[10] = 0xEE;
		sendBuff[11] = 0xFF;
		sendBuff[2] = 's';
		sendBuff[3] = 'h';
		sendBuff[4] = 'u';
		sendBuff[5] = 't';
		sendBuff[6] = 'd';
		sendBuff[7] = 'o';
		sendBuff[8] = 'w';
		sendBuff[9] = 'n';
		write(sockfd, sendBuff, sizeof(sendBuff)-1);
		usleep(10000);
	}
	
	return NULL;
}


