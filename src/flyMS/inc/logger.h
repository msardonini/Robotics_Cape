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
#include "gps.h"

#ifndef LOGGER_H
#define LOGGER_H

 
#define CORE_LOG_TABLE \
    X(float,  "%f",  time           ) \
    X(float,  "%f",  roll           ) \
    X(float,  "%f",  pitch          ) \
    X(float,  "%f",  yaw            ) \
    X(float,  "%f",  d_roll         ) \
    X(float,  "%f",  d_pitch        ) \
    X(float,  "%f",  d_yaw          ) \
    X(float,  "%f",  u_1            ) \
    X(float,  "%f",  u_2            ) \
    X(float,  "%f",  u_3            ) \
    X(float,  "%f",  u_4            ) \
    X(float,  "%f",  throttle       ) \
    X(float,  "%f",  upitch         ) \
    X(float,  "%f",  uroll          ) \
    X(float,  "%f",  uyaw           ) \
    X(float,  "%f",  pitch_ref      ) \
    X(float,  "%f",  roll_ref       ) \
    X(float,  "%f",  yaw_ref        ) \
    X(float,  "%f",  yaw_rate_ref   ) \
    X(float,  "%f",  Aux            ) \
    X(float,  "%f",  lat_error      ) \
    X(float,  "%f",  lon_error      ) \
    X(float,  "%f",  kalman_lat     ) \
    X(float,  "%f",  kalman_lon     ) \
    X(float,  "%f",  accel_x        ) \
    X(float,  "%f",  accel_y        ) \
    X(float,  "%f",  accel_z        ) \
    X(float,  "%f",  baro_alt       ) \
    X(float,  "%f",  v_batt         ) \
    X(float,  "%f",  compass_heading) \
    X(float,  "%f",  ned_pos_x) \
    X(float,  "%f",  ned_pos_y) \
    X(float,  "%f",  ned_pos_z) \
    X(float,  "%f",  ned_vel_x) \
    X(float,  "%f",  ned_vel_y) \
    X(float,  "%f",  ned_vel_z) \
    X(float,  "%f",  mag_x) \
    X(float,  "%f",  mag_y) \
    X(float,  "%f",  mag_z) \
    X(float,  "%f",  droll_setpoint) \
    X(float,  "%f",  dpitch_setpoint)
    

#define CORE_LOG_BUF_LEN 200 //once per second is reasonable

/************************************************************************
*   core_log_entry_t
*   struct definition to contain single line of the log
************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct core_log_entry_t { CORE_LOG_TABLE } core_log_entry_t;
#undef X

typedef struct core_logger_t{
    long num_entries;   // number of entries logged so far
    int buffer_pos; // position in current buffer
    int current_buf; //0 or 1 to indicate which buffer is being filled
    int needs_writing;
    FILE* log_file;
    // array of two buffers so one can fill while writing the other to file
    core_log_entry_t log_buffer[2][CORE_LOG_BUF_LEN];
}core_logger_t;

typedef struct logger_t{
    int                     logger_running;
    core_logger_t           core_logger;
    FILE                    *logger;            //File to log data with
    FILE                    *GPS_logger;        //File to log GPS data with
    FILE                    *Error_logger;      //File with catches errors and shutdowns
    core_log_entry_t        new_entry;
}logger_t;


/************************************************************************
*   logger_init()
*   Called by an outside function to initialize the logger and start
*   the logging thread
************************************************************************/
int logger_init();

/************************************************************************
*   logger_deinit()
*   Called by an outside function to deinit the logger. Finishes writing
*   to the log and stops the logging thread for shutdown
************************************************************************/
int logger_deinit();

/************************************************************************
*   log_data()
*   populates the logger substructure and pushes it out to the logfile
************************************************************************/
int log_data(control_variables_t *control);

/************************************************************************
*   log_GPS_data()
*   populates the logger substructure and pushes it out to the logfile
*   GPS data is only logged once per second so efficiency is less important
************************************************************************/
int log_GPS_data(GPS_data_t *GPS_data, float timestamp_sec);

/************************************************************************
*   flyMS_Error_Log()
*   Called by an outside function to deinit the logger. Finishes writing
*   to the log and stops the logging thread for shutdown
************************************************************************/
int flyMS_Error_Log(const char* errString);

/************************************************************************
* 	print_entry()
*	write the contents of one entry to the console
************************************************************************/
int print_entry(core_logger_t* logger, core_log_entry_t* entry);


#endif