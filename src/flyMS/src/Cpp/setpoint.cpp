/**
 * @file setpoint.cpp
 * @brief Class for controlling the input reference angle to the inner loop controller 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "setpoint.hpp"

setpoint::setpoint(config_t _config, logger& _loggingModule) :
	isInitializing(true),
	isReadyToParse(false),
	isReadyToSend(false),
	setpoint_type(RC_DIRECT),
	config(_config),
	loggingModule(_loggingModule)
{
}

setpoint::~setpoint()
{
	if(this->setpointThread.joinable())
		this->setpointThread.join();
}

int setpoint::start()
{
	int ret = rc_dsm_init();
	this->setpointThread = std::thread(&setpoint::setpointManager, this);
	return ret;
}

//Gets the data from the local thread. Returns zero if no new data is available
bool setpoint::getSetpointData(setpoint_t* _setpoint)
{
	if(this->isReadyToSend.load())
	{
		if(this->setpointMutex.try_lock_for(std::chrono::milliseconds(5)))
		{
			memcpy(_setpoint, &this->setpointData, sizeof(setpoint_t));
			this->isReadyToSend.store(false);
			this->setpointMutex.unlock();
			return true;
		}
	}
	return false;
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

	while (rc_get_state()!= EXITING)
	{
		/**********************************************************
		*           If there is new dsm2 data read it in 		  *
		*			and make a local copy from the driver's data  *
		**********************************************************/
		if (rc_dsm_is_new_data())
		{
			this->copy_dsm2_data();
			this->isReadyToParse = true;
			dsm2_timeout = 0;
		}
		else
		{
			if(!this->config.isDebugMode)
			{
				//check to make sure too much time hasn't gone by since hearing the RC
				this->rc_err_handler(setpoint_type);
			}
		}

		this->setpointMutex.lock();
		switch (setpoint_type)
		{
			case RC_DIRECT:
				handle_rc_data_direct();
			break;
			case RC_NAVIGATION:

			break;
			case RC_INITIALIZATION:

			break;
			default:
				this->loggingModule.flyMS_printf("Error, invalid reference mode! \n");
		}
		this->setpointMutex.unlock();
		usleep(DT_US); //Run at the control frequency
	}
	return 0;
}

	
int setpoint::handle_rc_data_direct()
{
	

	/**********************************************************
	*           Read the RC Controller for Commands           *
	**********************************************************/
	if(this->isReadyToParse)
	{
		//Set roll reference value
		//DSM2 Receiver is inherently positive to the left
		if(this->config.flightMode == 1) //Stabilized Flight Mode
		{
			this->setpointData.euler_ref[0]= dsm2_data[1]*MAX_ROLL_RANGE;	
			this->setpointData.euler_ref[1]= -dsm2_data[2]*MAX_PITCH_RANGE;
		}
		else if (this->config.flightMode == 2)
		{
			this->setpointData.euler_ref[0]= dsm2_data[1]*MAX_ROLL_RANGE_ACRO;	
			this->setpointData.euler_ref[1]= -dsm2_data[2]*MAX_PITCH_RANGE_ACRO;	
		}
		//DSM2 Receiver is inherently positive upwards
		
		//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
		//Apply the integration outside of current if statement, needs to run at 200Hz
		this->setpointData.yaw_rate_ref[1]=this->setpointData.yaw_rate_ref[0];		
		this->setpointData.yaw_rate_ref[0]=dsm2_data[3]*MAX_YAW_RATE;

		//If Specified by the config file, convert from Drone Coordinate System to User Coordinate System
		if (this->config.isHeadlessMode)
		{				
			//TODO: Give this thread access to state information so it can fly in headless mode
		
			// float P_R_MAG=pow(pow(this->setpointData.euler_ref[0],2)+pow(this->setpointData.euler_ref[1],2),0.5);
			// float Theta_Ref=atan2f(this->setpointData.euler_ref[0],this->setpointData.euler_ref[1]);
		
			// this->setpointData.euler_ref[1] =P_R_MAG*cos(Theta_Ref+this->state.euler[2]-this->setpointData.yaw_ref_offset);
			// this->setpointData.euler_ref[0]=P_R_MAG*sin(Theta_Ref+this->state.euler[2]-this->setpointData.yaw_ref_offset);
		}
		
		//Apply a deadzone to keep integrator from wandering
		if(fabs(this->setpointData.yaw_rate_ref[0])<0.05) {
			this->setpointData.yaw_rate_ref[0]=0;
		}

		//Kill Switch
		this->setpointData.kill_switch[1] = this->setpointData.kill_switch[0];
		this->setpointData.kill_switch[0]=dsm2_data[4];
		
		//Auxillary Switch
		this->setpointData.Aux[1] = this->setpointData.Aux[0];
		this->setpointData.Aux[0] = dsm2_data[5]; 
		
		//Set the throttle
		//TODO: make these values configuarable
		this->setpointData.throttle=(dsm2_data[0]) *
				(MAX_THROTTLE-MIN_THROTTLE) + MIN_THROTTLE;
		//Keep the aircraft at a constant height while making manuevers 
		// this->setpointData.throttle *= 1/(cos(this->state.euler[1])*cos(this->state.euler[0]));

		//Finally Update the integrator on the yaw reference value
		this->setpointData.euler_ref[2]=this->setpointData.euler_ref[2] + 
										(this->setpointData.yaw_rate_ref[0]+this->setpointData.yaw_rate_ref[1])*DT/2;

		this->isReadyToParse = false;
		this->isReadyToSend.store(true);

	}
	

	return 0;
}

int setpoint::copy_dsm2_data()
{
	int i;
	for (i = 0; i < MAX_DSM2_CHANNELS; i++)\
	{
		dsm2_data[i] = rc_dsm_ch_normalized(i+1);
	}
	return 0;
}

int setpoint::rc_err_handler(reference_mode_t setpoint_type)
{
	//If we are in the initializing stage don't bother shutting down the program
	if (this->isInitializing)
		return 0;

	this->dsm2_timeout++;
	if(this->dsm2_timeout>1.5/DT) //If packet hasn't been received for 1.5 seconds
	{
		this->loggingModule.flyMS_printf("\nLost Connection with Remote!! Shutting Down Immediately \n");
		rc_set_state(EXITING);
		return -1;
	}
	return 0;
}

void setpoint::setInitializationFlag(bool flag)
{
	this->isInitializing = flag;
}
