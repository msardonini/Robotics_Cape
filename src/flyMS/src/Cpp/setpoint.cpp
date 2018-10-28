/**
 * @file setpoint.cpp
 * @brief Class for controlling the input reference angle to the inner loop controller 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "setpoint.hpp"

setpoint::setpoint(enum reference_mode_t refMode, bool _enableHeadlessMode, bool _enableDebugMode) : 
	enableHeadlessMode(_enableHeadlessMode),
	enableDebugMode(_enableDebugMode)
{
	this->setpointThread = std::thread(&setpoint::setpointManager, this);
}

setpoint::~setpoint()
{
	this->setpointThread.join();
}

void setpoint::getSetpointData(setpoint_t* _setpoint)
{
	this->setpointMutex.lock();
	memcpy(_setpoint, &this->setpointData, sizeof(setpoint_t));
	this->setpointMutex.unlock();
}

/*
	setpoint_manager()
		Handles the setpoint values for roll/pitch/yaw to be fed into the flight controller
		
		2 Main sources of retreiving values
			1. Direct from remote flyMSData
			2. Calculated values from GPS navigation for autonomous flight
*/

int setpoint::setpointManager()
{
	enum reference_mode_t setpoint_type = RC_DIRECT;

	while (rc_get_state()!= EXITING)
	{
		/**********************************************************
		*           If there is new dsm2 data read it in 		  *
		*			and make a local copy from the driver's data  *
		**********************************************************/
		if (rc_is_new_dsm_data())
		{
			copy_dsm2_data();
			new_dsm_data = 1;
			dsm2_timeout = 0;
		}
		else
		{
			if(!this->enableDebugMode)
			{
				//check to make sure too much time hasn't gone by since hearing the RC
				rc_err_handler(setpoint_type);
			}
			new_dsm_data = 0;
		}

		switch (setpoint_type)
		{
			case RC_DIRECT:
				handle_rc_data_direct();
			break;
			case RC_NAVIGATION:

			break;
			default:
				printf("Error, invalid reference mode! \n");
		}

		usleep(DT_US); //Run at the control frequency
	}
	return NULL;
}

	
int setpoint::handle_rc_data_direct()
{
	this->setpointMutex.lock();

	/**********************************************************
	*           Read the RC Controller for Commands           *
	**********************************************************/
	if(new_dsm_data)
	{
		//Set roll reference value
		//DSM2 Receiver is inherently positive to the left
		this->setpointData.euler_ref[1]= -dsm2_data[1]*MAX_ROLL_RANGE;	
		
		//DSM2 Receiver is inherently positive upwards
		this->setpointData.euler_ref[0]= -dsm2_data[2]*MAX_PITCH_RANGE;
		
		//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
		//Apply the integration outside of current if statement, needs to run at 200Hz
		this->setpointData.yaw_rate_ref[1]=this->setpointData.yaw_rate_ref[0];		
		this->setpointData.yaw_rate_ref[0]=dsm2_data[3]*MAX_YAW_RATE;

		//If Specified by the config file, convert from Drone Coordinate System to User Coordinate System
		if (this->enableHeadlessMode)
		{				
			float P_R_MAG=pow(pow(this->setpointData.euler_ref[0],2)+pow(this->setpointData.euler_ref[1],2),0.5);
			float Theta_Ref=atan2f(this->setpointData.euler_ref[0],this->setpointData.euler_ref[1]);
		
			//TODO: Give this thread access to state information so it can fly in headless mode
			// this->setpointData.euler_ref[1] =P_R_MAG*cos(Theta_Ref+this->state.euler[2]-this->setpointData.yaw_ref_offset);
			// this->setpointData.euler_ref[0]=P_R_MAG*sin(Theta_Ref+this->state.euler[2]-this->setpointData.yaw_ref_offset);
		}
		
		//Apply a deadzone to keep integrator from wandering
		if(fabs(this->setpointData.yaw_rate_ref[0])<0.05) {
			this->setpointData.yaw_rate_ref[0]=0;
		}

		//Kill Switch
		this->setpointData.kill_switch[0]=dsm2_data[4]/2;
		
		//Auxillary Switch
		this->setpointData.Aux[1] = this->setpointData.Aux[0];
		this->setpointData.Aux[0] = dsm2_data[5]; 
		
		//Set the throttle
		//TODO: make these values configuarable
		this->setpointData.throttle=(dsm2_data[0]+1)* 0.5f *
				(MAX_THROTTLE-MIN_THROTTLE)+MIN_THROTTLE;
		//Keep the aircraft at a constant height while making manuevers 
		// this->setpointData.throttle *= 1/(cos(this->state.euler[1])*cos(this->state.euler[0]));
	}

	//Finally Update the integrator on the yaw reference value
	this->setpointData.euler_ref[2]=this->setpointData.euler_ref[2] + 
									(this->setpointData.yaw_rate_ref[0]+this->setpointData.yaw_rate_ref[1])*DT/2;
	this->setpointMutex.unlock();

	return 0;
}

int setpoint::copy_dsm2_data()
{
	int i;
	for (i = 0; i < MAX_DSM2_CHANNELS; i++)\
	{
		dsm2_data[i] = rc_get_dsm_ch_normalized(i+1);
	}
	return 0;
}

int setpoint::rc_err_handler(reference_mode_t setpoint_type)
{
	dsm2_timeout++;
	if(dsm2_timeout>1.5/DT) //If packet hasn't been received for 1.5 seconds
	{ 
		char errMsg[50];
		sprintf(errMsg,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
		printf("%s",errMsg);
		//TODO: add error message to the error log
		// flyMS_Error_Log(errMsg);
		rc_set_state(EXITING);
	}
	return 0;
}