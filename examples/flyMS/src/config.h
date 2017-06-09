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
#include <stdint.h>

#define CONFIG_FILE "flight_config.txt"
#define CONFIG_DIRECTORY "/root/Robotics_Cape/config_files/"

#ifndef CONFIG_H
#define CONFIG_H

#define CORE_CONFIG_FILE "flight_core_config.txt"
#define CORE_FILE_HEADER "flight_core_config_name: "
#define CORE_CONFIG_TABLE\
	X(int,  "%d", enable_barometer,		1		)\
	X(int,  "%d", enable_gps,			1		)\
	X(int,  "%d", enable_logging,		1		)\
	X(int,  "%d", enable_debug_mode,	0		)\
	X(int,  "%d", enable_autonomy,		0		)\
												\
	X(float,  "%f", alt_KP,				0		)\
	X(float,  "%f", alt_KD, 			0		)\
	X(float,  "%f", alt_KI, 			0		)\
												\
												\
	X(float,  "%f", roll_KP,  			5		)\
	X(float,  "%f", roll_KD,  			2		)\
	X(float,  "%f", roll_KI, 			0.175	)\
	X(float,  "%f", Droll_KP, 			0.05	)\
	X(float,  "%f", Droll_KI, 			0.32	)\
	X(float,  "%f", Droll_KD, 			0.00155	)\
	X(float,  "%f", max_roll_setpoint,	0.4		)\
												\
												\
	X(float,  "%f", pitch_KP,  			5		)\
	X(float,  "%f", pitch_KD,  			2		)\
	X(float,  "%f", pitch_KI, 			0.175	)\
	X(float,  "%f", Dpitch_KP, 			0.05	)\
	X(float,  "%f", Dpitch_KI, 			0.32	)\
	X(float,  "%f", Dpitch_KD, 			0.003	)\
	X(float,  "%f", max_pitch_setpoint,	0.4		)\
												\
	X(float,  "%f", yaw_KP, 			0.5		)\
	X(float,  "%f", yaw_KI, 			0.05	)\
	X(float,  "%f", yaw_KD, 			0.05	)\
												\
	X(float,  "%f", max_yaw_rate,		3		)\
	X(float,  "%f", idle_speed,			0.3		)\
	X(float,  "%f", min_throttle, 		0.3	)\
	X(float,  "%f", max_throttle, 		0.75	)\
												\
	X(float,  "%f", max_vert_velocity,	3		)\
	X(float,  "%f", max_horz_velocity, 	3		)\
												\
	X(float,  "%f", aux1,	0	)\
	X(float,  "%f", aux2,	0	)\
	X(float,  "%f", aux3,	0	)\
	X(float,  "%f", aux4,	0	)\
	X(float,  "%f", aux5,	0	)\
	X(float,  "%f", aux6,	0	)


/************************************************************************
* 	core_config_t
*	this contains all configuration data for the flight_core
*	the global instance core_config is populated before launching 
*	the flight core. It can be modified while flying, eg to adjust gains
*	an instance should be declared in your own C file
************************************************************************/
#define X(type, fmt, name, default) type name ;
typedef struct core_config_t { CORE_CONFIG_TABLE } core_config_t;
#undef X




/************************************************************************
* 	print_core_config()
*	print configuration table to console
************************************************************************/
int print_core_config(core_config_t* config);


/************************************************************************
* 	create_empty_core_config_file()
*	creates a new file from the robotics cape library definition for 
*	config directory and this file's definition for config file name
************************************************************************/
FILE* create_empty_core_config_file(char name[]);

/************************************************************************
* 	save_core_config()
*	write the config struct to disk
* 	should be called after writing the following header line to the file
* 	fprintf(f, "flight_core_config_name: your_name");
************************************************************************/
int save_core_config(FILE* f, core_config_t* config);


/************************************************************************
* 	load_core_config()
*	read from the disk. returns -1 if unsuccessful 
************************************************************************/
int load_core_config(core_config_t* config);

/************************************************************************
* 	create_default_core_config_file()
*	creates a new file and returns a struct containing the default values
************************************************************************/
int create_default_core_config_file(core_config_t* config);

#endif