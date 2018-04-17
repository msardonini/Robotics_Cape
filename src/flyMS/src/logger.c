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
#include "flyMS_common.h"
#include "logger.h"

//Local Variables
logger_t logger;
pthread_t logging_thread;

//Local Functions

/************************************************************************
* 	log_core_data()
*	called by an outside function to quickly add new data to local buffer
************************************************************************/
static int log_core_data(core_logger_t* log, core_log_entry_t* new_entry);

/************************************************************************
* 	write_core_log_entry()
*	append a single entry to the log file
************************************************************************/
static int write_core_log_entry(FILE* f, core_log_entry_t* entry);

	
/************************************************************************
* 	core_log_writer()
*	independent thread that monitors the needs_writing flag
*	and dumps a buffer to file in one go
************************************************************************/
static void* core_log_writer(void* new_log);

/************************************************************************
* 	start_core_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	and start a thread to write
************************************************************************/
static int start_core_log();

/************************************************************************
* 	stop_core_log()
*	finish writing remaining data to log and close it
************************************************************************/
static int stop_core_log(core_logger_t* log);
	


/************************************************************************
* 	logger_init()
*	Called by an outside function to initialize the logger and start
*	the logging thread
************************************************************************/
int logger_init()
{
	// start a core_log and logging thread
	if(start_core_log()<0){
		printf("WARNING: failed to open a core_log file\n");
		return -1;
	}
	else{
		pthread_create(&logging_thread, NULL, core_log_writer, &logger.core_logger);
		logger.logger_running = 1;
	}
	return 0;
}
/************************************************************************
* 	logger_deinit()
*	Called by an outside function to deinit the logger. Finishes writing
*	to the log and stops the logging thread for shutdown
************************************************************************/
int logger_deinit()
{
	if(logger.logger_running)
	{		
		stop_core_log(&logger.core_logger);// finish writing core_log
		pthread_join(logging_thread, NULL);
		printf("Logging thread joined\n");

		static char* StateStrings[] = {	"UNINITIALIZED", "RUNNING", 
										"PAUSED", "EXITING" };
		char endMsg[50];
		sprintf(endMsg, "Exiting program, system state is %s\n", StateStrings[rc_get_state()]);

		flyMS_Error_Log(endMsg);
		fflush(stdout);

		// Close the log files
		fclose(logger.GPS_logger);
		fclose(logger.Error_logger);

	}
	return 0;
}
/************************************************************************
* 	print_entry()
*	populates the logger substructure and pushes it out to the logfile
************************************************************************/
int log_data(control_variables_t *control)
{
		logger.new_entry.time			= control->time_us;	
		logger.new_entry.pitch			= control->state.euler[0];	
		logger.new_entry.roll			= control->state.euler[1];
		logger.new_entry.yaw			= control->state.euler[2];
		logger.new_entry.d_pitch		= control->state.euler_rate[0];	
		logger.new_entry.d_roll			= control->state.euler_rate[1];
		logger.new_entry.d_yaw			= control->state.euler_rate[2];
		logger.new_entry.u_1			= control->control.u[0];
		logger.new_entry.u_2			= control->control.u[1];
		logger.new_entry.u_3			= control->control.u[2];
		logger.new_entry.u_4			= control->control.u[3];
		logger.new_entry.throttle		= control->setpoint.throttle;
		logger.new_entry.upitch			= control->control.u_euler[0];	
		logger.new_entry.uroll			= control->control.u_euler[1];
		logger.new_entry.uyaw			= control->control.u_euler[2];
		logger.new_entry.pitch_ref		= control->setpoint.euler_ref[0];
		logger.new_entry.roll_ref		= control->setpoint.euler_ref[1];
		logger.new_entry.yaw_ref		= control->setpoint.euler_ref[2];
		logger.new_entry.yaw_rate_ref	= control->setpoint.yaw_rate_ref[0];
		logger.new_entry.Aux			= control->setpoint.Aux[0];
		// control->logger.new_entry.lat_error		= control->lat_error;
		// control->logger.new_entry.lon_error		= control->lon_error;
		logger.new_entry.accel_x		= control->imu.accel[0];
		logger.new_entry.accel_y		= control->imu.accel[1];
		logger.new_entry.accel_z		= control->imu.accel[2];
		logger.new_entry.baro_alt		= control->imu.baro_alt;
		logger.new_entry.v_batt			= 0;
		logger.new_entry.ned_pos_x		= control->ekf_filter.output.ned_pos[0];
		logger.new_entry.ned_pos_y		= control->ekf_filter.output.ned_pos[1];
		logger.new_entry.ned_pos_z		= control->ekf_filter.output.ned_pos[2];
		logger.new_entry.ned_vel_x		= control->ekf_filter.output.ned_vel[0];
		logger.new_entry.ned_vel_y		= control->ekf_filter.output.ned_vel[1];
		logger.new_entry.ned_vel_z		= control->ekf_filter.output.ned_vel[2];
		logger.new_entry.mag_x			= control->imu.mag[0];
		logger.new_entry.mag_y			= control->imu.mag[1];
		logger.new_entry.mag_z			= control->imu.mag[2];
		logger.new_entry.compass_heading= control->imu.compass_heading;
		logger.new_entry.droll_setpoint	= control->setpoint.droll_setpoint;
		logger.new_entry.dpitch_setpoint= control->setpoint.dpitch_setpoint;
		//control->logger.new_entry.v_batt			= rc_dc_jack_voltage();
		log_core_data(&logger.core_logger, &logger.new_entry);
	return 0;
}

/************************************************************************
* 	log_GPS_data()
*	populates the logger substructure and pushes it out to the logfile
*	GPS data is only logged once per second so efficiency is less important
************************************************************************/
int log_GPS_data(GPS_data_t *GPS_data, uint64_t timestamp_sec)
{
	fprintf(logger.GPS_logger,"%llu,",timestamp_sec);
	fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data->deg_longitude,GPS_data->min_longitude);
	fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data->deg_latitude,GPS_data->min_latitude);
	fprintf(logger.GPS_logger,"%f,%f,",GPS_data->speed,GPS_data->direction);
	fprintf(logger.GPS_logger,"%f,",GPS_data->gps_altitude);
	fprintf(logger.GPS_logger,"%2.2f,%d",GPS_data->HDOP,GPS_data->GPS_fix);
	fprintf(logger.GPS_logger,"\n");
	fflush(logger.GPS_logger);
	return 0;
}
/************************************************************************
* 	flyMS_Error_Log()
*	Called by an outside function to write error debug information to the
*	dedicated error logger file
************************************************************************/
int flyMS_Error_Log(const char* errString)
{
	if(logger.logger_running)
	{		
		fprintf(logger.Error_logger, "%s", errString);
	}
	return 0;
}

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
static int log_core_data(core_logger_t* log, core_log_entry_t* new_entry){
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
static int write_core_log_entry(FILE* f, core_log_entry_t* entry){
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
static int start_core_log(){
		
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
	logger.core_logger.log_file = fopen(logger_filepath, "w");
	if (logger.core_logger.log_file==NULL){
		printf("could not open logging directory\n");
		printf("Attempted File name %s\n", logger_filepath);
		return -1;
	}
	
	//Open GPS log file and check
	logger.GPS_logger=fopen(GPS_filepath,"w+");
	if(logger.GPS_logger == NULL) 
	{
		printf("Error! GPS_logger.csv failed to open\n");
		printf("Attempted File name %s\n", GPS_filepath);
		return -1;
	}
	//Write the header for the GPS log file
	fprintf(logger.GPS_logger,"time,deg_lon,min_lon,deg_lat,min_lat,speed,direction,gps_alt,hdop,fix\n");
	fflush(logger.GPS_logger);

	//Open Error logger and check
	logger.Error_logger=fopen(Error_filepath,"w+");
	if(logger.Error_logger == NULL) 
	{
		printf("Error! Error_logger.csv failed to open\n");
		printf("Attempted File name %s\n", Error_filepath);
		return -1;
	}
	fflush(logger.Error_logger);
	
	#define X(type, fmt, name) fprintf(logger.core_logger.log_file, "%s," , #name);
    CORE_LOG_TABLE
	#undef X
	fprintf(logger.core_logger.log_file, "\n");
	fflush(logger.core_logger.log_file);

	return 0;
}

/************************************************************************
* 	stop_core_log()
*	finish writing remaining data to log and close it
************************************************************************/
static int stop_core_log(core_logger_t* log){
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

#ifdef __cplusplus
}
#endif
