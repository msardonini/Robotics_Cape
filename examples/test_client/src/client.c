#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 

int main(int argc, char *argv[])
{

	//-------------------- start client code -------------------/
	 int sockfd = 0;
    char sendBuff[16];
    struct sockaddr_in serv_addr; 

    memset(sendBuff, '0', sizeof(sendBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket \n");
        return 1;
    } 

    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5000); 

   // if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
    if(inet_pton(AF_INET, "127.0.0.1" , &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return 1;
    } 

    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return 1;
    } 
	printf("made it to while\n");
	int count = 0;
    while (1)
    {
		sendBuff[0] = 0x01;
		sendBuff[1] = 0x01;
		if(count == 10) sendBuff[2] = 0xFF; 
		write(sockfd, sendBuff, sizeof(sendBuff)-10);
		sleep(1);
		count++;
    } 
	return 0;
}