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


// test_log_file.c
// James Strawson - 2014
// sample to demonstrate logging robot data to a file
// specifically this logs IMU sensor readings to a new log file

#ifdef __cplusplus
extern "C" 
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include "roboticscape.h"
#include "logger.h"


/************************************************************************
* 	print_entry()
*	write the contents of one entry to the console
************************************************************************/
int print_entry(core_logger_t* logger, core_log_entry_t* entry){	
	
	#define X(type, fmt, name) printf("%s " fmt "\n", #name, entry->name);
	CORE_LOG_TABLE
	#undef X
	
	return 0;
}


/************************************************************************
* 	log_core_data()
*	called by an outside function to quickly add new data to local buffer
************************************************************************/
int log_core_data(core_logger_t* log, core_log_entry_t* new_entry){
	if(log->needs_writing && log->buffer_pos >= CORE_LOG_BUF_LEN){
		printf("warning, both logging buffers full\n");
		return -1;
	}
	log->log_buffer[log->current_buf][log->buffer_pos] = *new_entry;
	log->buffer_pos ++;
	log->num_entries ++;
	// we've filled a buffer, set the write flag and swap to other buffer
	if(log->buffer_pos >= CORE_LOG_BUF_LEN){
		log->buffer_pos = 0;
		log->needs_writing = 1;
		if(log->current_buf==0) log->current_buf = 1;
		else log->current_buf = 0;
		
	}
	return 0;
}

/************************************************************************
* 	write_core_log_entry()
*	append a single entry to the log file
************************************************************************/
int write_core_log_entry(FILE* f, core_log_entry_t* entry){
	#define X(type, fmt, name) fprintf(f, fmt "," , entry->name);
    CORE_LOG_TABLE
	#undef X	
	fprintf(f, "\n");
	return 0;
}

/************************************************************************
* 	core_log_writer()
*	independent thread that monitors the needs_writing flag
*	and dumps a buffer to file in one go
************************************************************************/
void* core_log_writer(void* new_log){
	core_logger_t *log = (core_logger_t *)new_log;
	while(rc_get_state()!=EXITING){
		int i,j;
		if(log->needs_writing){
			if(log->current_buf == 0) j=1;
			else j=0;
			for(i=0;i<CORE_LOG_BUF_LEN;i++){
				write_core_log_entry(log->log_file, &log->log_buffer[j][i]);
			}
			fflush(log->log_file);
			log->needs_writing = 0;
		}
		usleep(10000);
	}
	return NULL;
}


/************************************************************************
* 	start_core_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	and start a thread to write
************************************************************************/
int start_core_log(logger_t *logger){
		
	char logger_filepath[strlen(FLYMS_ROOT_DIR) + 40];
	char GPS_filepath[strlen(FLYMS_ROOT_DIR) + 40];
	char Error_filepath[strlen(FLYMS_ROOT_DIR) + 40];	

	memset(logger_filepath,0,strlen(logger_filepath));
	memset(GPS_filepath,0,strlen(GPS_filepath));
	memset(Error_filepath,0,strlen(Error_filepath));

	char n[6];
	int m=1;
	struct stat st = {0};
	
	sprintf(n,"%03d",m);
	strcpy(logger_filepath,FLYMS_ROOT_DIR);
	strcat(logger_filepath,"/flight_logs/run");
	strcat(logger_filepath,n);
	
	//Find the next run number folder that isn't in use
	while(!stat(logger_filepath, &st))
	{
		m++;
		sprintf(n,"%03d",m);
		
		strcpy(logger_filepath,FLYMS_ROOT_DIR);
		strcat(logger_filepath,"/flight_logs/run");
		strcat(logger_filepath,n);
	}
	
	//Make a new directory for log files
	mkdir(logger_filepath,0700);
	printf("Saving log files in: %s\n",logger_filepath);
	
	//Create a filename for GPS logs
	strcpy(GPS_filepath,logger_filepath);
	strcat(GPS_filepath,"/GPS_logger.csv");

	//Create a filename for Error logs
	strcpy(Error_filepath,logger_filepath);
	strcat(Error_filepath,"/Error_logger.txt");

	//Finally finish off the logger filepath 
	strcat(logger_filepath,"/logger.csv");
	
	//Open logging file and check
	logger->core_logger.log_file = fopen(logger_filepath, "w");
	if (logger->core_logger.log_file==NULL){
		printf("could not open logging directory\n");
		printf("Attempted File name %s\n", logger_filepath);
		return -1;
	}
	
	//Open GPS log file and check
	logger->GPS_logger=fopen(GPS_filepath,"w+");
	if(logger->GPS_logger == NULL) 
	{
		printf("Error! GPS_logger.csv failed to open\n");
		printf("Attempted File name %s\n", GPS_filepath);
		return -1;
	}

	//Open Error logger and check
	logger->Error_logger=fopen(Error_filepath,"w+");
	if(logger->Error_logger == NULL) 
	{
		printf("Error! Error_logger.csv failed to open\n");
		printf("Attempted File name %s\n", Error_filepath);
		return -1;
	}
	
	#define X(type, fmt, name) fprintf(logger->core_logger.log_file, "%s," , #name);
    CORE_LOG_TABLE
	#undef X
	fprintf(logger->core_logger.log_file, "\n");
	fflush(logger->core_logger.log_file);

	return 0;
}

/************************************************************************
* 	stop_core_log()
*	finish writing remaining data to log and close it
************************************************************************/
int stop_core_log(core_logger_t* log){
	int i;
	// wait for previous write to finish if it was going
	while(log->needs_writing){
		usleep(10000);
	}
	
	// if there is a partially filled buffer, write to file
	if(log->buffer_pos > 0){
		for(i=0;i<log->buffer_pos;i++){
			write_core_log_entry(log->log_file, &log->log_buffer[log->current_buf][i]);
		}
		fflush(log->log_file);
		log->needs_writing = 0;
	}
	fclose(log->log_file);
	return 0;
}


char* concat(char *s1, char *s2){
    char *result = (char *)malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
    //in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}


#ifdef __cplusplus
}
#endif
