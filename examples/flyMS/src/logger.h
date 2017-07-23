/*
Copyright (c) 2014, James Strawson
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


#pragma once
#include <stdlib.h>
#include <stdio.h>

#ifndef LOGGER_H
#define LOGGER_H

 
#define CORE_LOG_TABLE \
    X(float,  "%f",	 time			) \
    X(float,  "%f",	 roll			) \
    X(float,  "%f",	 pitch			) \
    X(float,  "%f",	 yaw			) \
	X(float,  "%f",	 d_roll			) \
    X(float,  "%f",	 d_pitch		) \
    X(float,  "%f",	 d_yaw			) \
	X(float,  "%f",	 u_1			) \
    X(float,  "%f",	 u_2			) \
    X(float,  "%f",	 u_3			) \
	X(float,  "%f",	 u_4			) \
    X(float,  "%f",	 throttle		) \
    X(float,  "%f",	 upitch			) \
	X(float,  "%f",	 uroll			) \
    X(float,  "%f",	 uyaw			) \
    X(float,  "%f",	 pitch_ref		) \
    X(float,  "%f",	 roll_ref		) \
    X(float,  "%f",	 yaw_ref		) \
    X(float,  "%f",	 yaw_rate_ref	) \
    X(float,  "%f",	 Aux			) \
    X(float,  "%f",	 lat_error		) \
    X(float,  "%f",	 lon_error		) \
    X(float,  "%f",	 kalman_lat		) \
    X(float,  "%f",	 kalman_lon		) \
    X(float,  "%f",	 accel_x		) \
    X(float,  "%f",  accel_y        ) \
    X(float,  "%f",	 accel_z		) \
    X(float,  "%f",	 baro_alt		) \
    X(float,  "%f",	 v_batt			) \
    X(float,  "%f",	 compass_heading)
	

#define CORE_LOG_BUF_LEN 200 //once per second is reasonable

/************************************************************************
* 	core_log_entry_t
*	struct definition to contain single line of the log
************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct core_log_entry_t { CORE_LOG_TABLE } core_log_entry_t;
#undef X

/************************************************************************
* 	Global Variables
************************************************************************/
typedef struct core_logger_t{
	long num_entries;	// number of entries logged so far
	int buffer_pos; // position in current buffer
	int current_buf; //0 or 1 to indicate which buffer is being filled
	int needs_writing;
	FILE* log_file;
	// array of two buffers so one can fill while writing the other to file
	core_log_entry_t log_buffer[2][CORE_LOG_BUF_LEN];
}core_logger_t;

typedef struct logger_t{
	core_logger_t			core_logger;
	FILE 					*logger;			//File to log data with
	FILE 					*GPS_logger;		//File to log GPS data with
	FILE					*Error_logger;		//File with catches errors and shutdowns
	core_log_entry_t 		new_entry;
}logger_t;


/************************************************************************
* 	print_entry()
*	write the contents of one entry to the console
************************************************************************/
int print_entry(core_logger_t* logger, core_log_entry_t* entry);


/************************************************************************
* 	log_core_data()
*	called by an outside function to quickly add new data to local buffer
************************************************************************/
int log_core_data(core_logger_t* log, core_log_entry_t* new_entry);

/************************************************************************
* 	write_core_log_entry()
*	append a single entry to the log file
************************************************************************/
int write_core_log_entry(FILE* f, core_log_entry_t* entry);

	
/************************************************************************
* 	core_log_writer()
*	independent thread that monitors the needs_writing flag
*	and dumps a buffer to file in one go
************************************************************************/
void* core_log_writer(void* new_log);

/************************************************************************
* 	start_core_log()
*	create a new csv log file with the date and time as a name
*	also print header as the first line to give variable names
*	and start a thread to write
************************************************************************/
int start_core_log(logger_t *logger);

/************************************************************************
* 	stop_core_log()
*	finish writing remaining data to log and close it
************************************************************************/
int stop_core_log(core_logger_t* log);

	
char* concat(char *s1, char *s2);
	

#endif