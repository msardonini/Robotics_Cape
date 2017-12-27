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

#pragma once
#include "roboticscape.h"
#include "pru_handler_client.h"
#include "Fusion.h"
#include "filter.h"
#include "config.h"
#include "logger.h"
#include "gps.h"

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

/************************************************************************
	imu_handler()
		Does all the parsing and interpretting of the IMU
		5 main tasks
			1. Reads the data from the IMU using Robotics_Cape API
			2. Performs a coordinate system transformation from imu -> drone
			3. Unwraps the yaw value for proper PID control
			4. Reads Barometer for altitude measurement
			5. Sends data to the EKF for position control

************************************************************************/
int imu_handler(control_variables_t *control);



/************************************************************************
*							Initialize the IMU                          *
************************************************************************/
int initialize_imu(control_variables_t *control);


/************************************************************************
*					   Update the EKF with GPS Data                     *
************************************************************************/
int update_ekf_gps(control_variables_t *control, GPS_data_t *GPS_data);

#endif