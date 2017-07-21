#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 
#include "../../../libraries/roboticscape.h"
#include "../../../libraries/other/rc_pru.h"
#define PID_FILE_PRU "/var/run/pru_handler.pid"

typedef enum pru_state_t {
         PRUUNINITIALIZED,
         PRURUNNING,
         PRUPAUSED,
         PRUEXITING
} pru_state_t;

pru_state_t pru_state = PRUUNINITIALIZED;

pru_state_t pru_get_state();
int pru_set_state(pru_state_t new_state);

int pru_set_state(pru_state_t new_state){
         pru_state = new_state;
         return 0;
}

pru_state_t pru_get_state(){
         return pru_state;
}


void shutdown_pru_handler(int signo);

void shutdown_pru_handler(int signo){
         switch(signo){
         case SIGINT: // normal ctrl-c shutdown interrupt
                 pru_set_state(PRUEXITING);
                 printf("\nreceived SIGINT Ctrl-C\n");
                 break;
         case SIGTERM: // catchable terminate signal
                 pru_set_state(PRUEXITING);
                 printf("\nreceived SIGTERM\n");
                 break;
         case SIGHUP: // terminal closed or disconnected, carry on anyway
                 break;
         default:
                 break;
         }
         return;
 }


void pru_disable_signal_handler(){
         signal(SIGINT, shutdown_pru_handler);
         signal(SIGKILL, shutdown_pru_handler);
         signal(SIGHUP, shutdown_pru_handler);
         return;
}



int rc_kill_pru(){
	FILE* fd;
	int old_pid, i;
	// start by checking if a pid file exists
	if(access(PID_FILE_PRU, F_OK ) != 0){
		// PID file missing
		return 0;
	}
	// attempt to open PID file
	// if the file didn't open, no project is runnning in the background
	// so return 0
	fd = fopen(PID_FILE_PRU, "r");
	if(fd==NULL) return 0;
	// try to read the current process ID
	fscanf(fd,"%d", &old_pid);
	fclose(fd);
	// if the file didn't contain a PID number, remove it and 
	// return -1 indicating weird behavior
	if(old_pid == 0){
		remove(PID_FILE_PRU);
		return -2;
	}
	// check if it's our own pid, if so return 0
	if(old_pid == (int)getpid()) return 0;
	// now see if the process for the read pid is still running
	if(getpgid(old_pid) < 0){
		// process not running, remove the pid file
		remove(PID_FILE_PRU);
		return 0;
	}
	// process must be running, attempt a clean shutdown
	kill((pid_t)old_pid, SIGINT);
	// check every 0.1 seconds to see if it closed 
	for(i=0; i<30; i++){
		if(getpgid(old_pid) >= 0) rc_usleep(100000);
		else{ // succcess, it shut down properly
			remove(PID_FILE_PRU);
			return 1; 
		}
	}
	// otherwise force kill the program if the PID file never got cleaned up
	kill((pid_t)old_pid, SIGKILL);
	rc_usleep(500000);
	// delete the old PID file if it was left over
	remove(PID_FILE_PRU);
	// return -1 indicating the program had to be killed
	return -1;
}


int init_pru_handler()
{
	//Check to see if this process is running somewhere else
	rc_kill_pru();

	//startup the signal handler
	pru_disable_signal_handler();	
	
	pru_set_state(RUNNING);

	// start PRU
	#ifdef DEBUG
	printf("Initializing: PRU\n");
	#endif
	initialize_pru();
	
	// create new pid file with process id
	#ifdef DEBUG
		printf("opening PID_FILE\n");
	#endif
	FILE *fd; 
	fd = fopen(PID_FILE_PRU, "ab+");
	if (fd == NULL) {
		fprintf(stderr,"error opening PID_FILE for writing\n");
		return -1;
	}
	pid_t current_pid = getpid();
	fprintf(fd,"%d",(int)current_pid);
	fflush(fd);
	fclose(fd);

	// Print current PID
	#ifdef DEBUG
	printf("Process ID: %d\n", (int)current_pid); 
 	#endif
	
	return 0;
}




int main(int argc, char *argv[])
{
	init_pru_handler();
	
	
	
	
	//----------------- start server code -----------------/
	int listenfd = 0, connfd = 0, n = 0;
	struct sockaddr_in serv_addr; 

    char rcvBuff[16];

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(rcvBuff, '0', sizeof(rcvBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 
	printf("made it to while\n");
	
	int flags = fcntl(listenfd, F_GETFL, 0);
	fcntl(listenfd, F_SETFL, flags | O_NONBLOCK);	
	struct timeval tv;
	
	start_over:
	while(pru_get_state()!=PRUEXITING)
	{
		connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
		if (connfd >=0) break;
		usleep(20000);
		rc_send_esc_pulse_normalized_all(-0.09);

	}
	while(pru_get_state()!=PRUEXITING)
    {
	
		tv.tv_sec = 0;  /* 30 Secs Timeout */
		tv.tv_usec = 30000;  // Not init'ing this can cause strange errors
		setsockopt(connfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));
	n = read(connfd, rcvBuff, sizeof(rcvBuff)-1);
	if (n <= 0)
	{
		printf("Timeout detected!! \n");
		rc_send_esc_pulse_normalized_all(0.0);
	}
	else
	{
		rcvBuff[n] = 0;
		
		float val[4];
		int i;
		
		//Only proceed if the start/end bytes come in as expected
		if (rcvBuff[0]	== 0xAA && rcvBuff[1] == 0xBB
			&&	rcvBuff[10] == 0xEE && rcvBuff[11] == 0xFF)
		{
			//Check for shutdown command
			if (	rcvBuff[2] == 's'
				&&	rcvBuff[3] == 'h' 
				&&	rcvBuff[4] == 'u'
				&&	rcvBuff[5] == 't'
				&&	rcvBuff[6] == 'd'
				&&	rcvBuff[7] == 'o'
				&&	rcvBuff[8] == 'w'
				&&	rcvBuff[9] == 'n')
			{
				close(connfd);
				goto start_over;
			}
			else
			{
			printf("Sending Values: ");
				for (i = 0; i < 8; i++)
				{ 
					val[i] = ((float)((rcvBuff[2*i+2] << 8) + rcvBuff[2*i+3]))/65536.0f;
					rc_send_esc_pulse_normalized(i+1,val[i]);
					printf(" %f, ", val[i]);
				}
			printf("\n");
			}
		}
	}
    }
        close(connfd);
	remove(PID_FILE_PRU);
	
	return 0;
}


