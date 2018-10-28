/**
 * @file flyMS.cpp
 * @brief flyMS program source code. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS.hpp"


// Default Constructor
flyMS::flyMS(bool _debugMode) : 
	isDebugMode(_debugMode),
	firstIteration(true) , 
	setpointModule(RC_DIRECT, false, false),
	imuModule(true)
{

}


//Default Destructor
flyMS::~flyMS()
{

}


int flyMS::flightCore()
{
	//Local Variables
	int i=0;
	uint64_t timeStart;
	uint64_t timeFinish;

	rc_set_state(RUNNING);

	while(rc_get_state()!=EXITING)
	{

		/******************************************************************
		*			Grab the time for Periphal Apps and Logs 			  *
		******************************************************************/
		timeStart = this->getTimeMicroseconds();
		
		/******************************************************************
		*			Read, Parse, and Translate IMU data for Flight		  *
		******************************************************************/
		this->imuModule.update();
		this->imuModule.getImuData(&this->imuData);

		/******************************************************************
		*				Take Care of Some Initialization Tasks			  *
		******************************************************************/

		if(this->firstIteration){
			this->setpointData.euler_ref[2]=this->imuData.euler[2];
			this->setpointData.yaw_ref_offset = this->imuData.euler[2];
			this->firstIteration=false;
		}

		/************************************************************************
		*                   	Throttle Controller                             *
		************************************************************************/

		//	float throttle_compensation = 1 / cos(this->imuData.euler[0]);
		//	throttle_compensation *= 1 / cos(this->imuData.euler[1]);		

		// if(function_control.altitudeHold)
		// {
		// 	if(function_control.altitudeHoldFirstIteration)
		// 	{
		// 		// this->control.standing_throttle = this->setpointData.throttle;
		// 		// this->setpointData.altitudeSetpoint = flyMSData.imu.baro_alt;
		// 		// function_control.altitudeHoldFirstIteration = 0;
		// 	}
	 //       // this->setpointData.altitudeSetpoint=this->setpointData.altitudeSetpoint+(this->setpointData.altitudeSetpointRate)*DT;

		// 	//this->control.uthrottle = update_filter(this->filter.altitudeHoldPID, this->setpointData.altitudeSetpoint - flyMSData.baro_alt);
		// 	//this->setpointData.throttle = this->control.uthrottle + flyMSData.standing_throttle;
		// }
			
		/************************************************************************
		* 	                  		Pitch Controller	                        *
		************************************************************************/
		this->setpointData.dpitch_setpoint = update_filter(this->filter.pitch_PD, this->setpointData.euler_ref[0] - this->imuData.euler[0]);
		this->control.u_euler[0] = update_filter(this->filter.pitch_rate_PD,this->setpointData.dpitch_setpoint - this->imuData.eulerRate[0]);
		this->control.u_euler[0] = saturateFilter(this->control.u_euler[0],-MAX_PITCH_COMPONENT,MAX_PITCH_COMPONENT);
		
		/************************************************************************
		* 	                  		Roll Controller		                        *
		************************************************************************/
		this->setpointData.droll_setpoint = update_filter(this->filter.roll_PD, this->setpointData.euler_ref[1] - this->imuData.euler[1]);
		this->control.u_euler[1] = update_filter(this->filter.roll_rate_PD,this->setpointData.droll_setpoint - this->imuData.eulerRate[1]);				
		this->control.u_euler[1] = saturateFilter(this->control.u_euler[1],-MAX_ROLL_COMPONENT,MAX_ROLL_COMPONENT);
		
		/************************************************************************
		*                        	Yaw Controller                              *
		************************************************************************/	
		// this->control.u_euler[2] = update_filter(this->filter.yaw_rate_PD,this->setpointData.euler_ref[2]-this->imuData.euler[2]);
		this->control.u_euler[2] = update_filter(this->filter.yaw_rate_PD,this->setpointData.yaw_rate_ref[0]-this->imuData.eulerRate[2]);
		
		/************************************************************************
		*                   	Apply the Integrators                           *
		************************************************************************/	
			
		if(this->setpointData.throttle<MIN_THROTTLE+.01){	
			this->integrator_reset++;
			this->integrator_start=0;
		}else{
			this->integrator_reset=0;
			this->integrator_start++;
		}
		
		if(this->integrator_reset==300){// if landed, reset integrators and Yaw error
			this->setpointData.euler_ref[2]=this->imuData.euler[2];
			this->control.droll_err_integrator=0; 
			this->control.dpitch_err_integrator=0;
			this->control.dyaw_err_integrator=0;
		}
			
		//only use integrators if airborne (above minimum throttle for > 1.5 seconds)
		if(this->integrator_start >  400){
			this->control.dpitch_err_integrator += this->control.u_euler[0] * DT;
			this->control.droll_err_integrator  += this->control.u_euler[1] * DT;
			this->control.dyaw_err_integrator += this->control.u_euler[2] * DT;		
			
			this->control.u_euler[0] += this->config.Dpitch_KI * this->control.dpitch_err_integrator;
			this->control.u_euler[1] += this->config.Droll_KI * this->control.droll_err_integrator;
			this->control.u_euler[2] += this->config.yaw_KI * this->control.dyaw_err_integrator;
		}
		
		//Apply a saturation filter
		this->control.u_euler[2] = saturateFilter(this->control.u_euler[2],-MAX_YAW_COMPONENT,MAX_YAW_COMPONENT);
		
		/************************************************************************
		*  Mixing
		*           	      black				yellow
		*                          CCW 1	  2 CW		IMU Orientation:	
		*                          	   \ /				Y
		*	                           / \            	|_ X
		*                         CW 3	  4 CCW
		*                 	  yellow       	    black
		************************************************************************/
		
		this->control.u[0]=this->setpointData.throttle+this->control.u_euler[1]+this->control.u_euler[0]-this->control.u_euler[2];
		this->control.u[1]=this->setpointData.throttle-this->control.u_euler[1]+this->control.u_euler[0]+this->control.u_euler[2];
		this->control.u[2]=this->setpointData.throttle+this->control.u_euler[1]-this->control.u_euler[0]+this->control.u_euler[2];
		this->control.u[3]=this->setpointData.throttle-this->control.u_euler[1]-this->control.u_euler[0]-this->control.u_euler[2];		

		/************************************************************************
		*         		Check Output Ranges, if outside, adjust                 *
		************************************************************************/
		check_output_range(this->control.u);
		
		/************************************************************************
		*         				 Send Commands to ESCs 		                    *
		************************************************************************/
		if(!this->isDebugMode)
		{
			std::vector<float> sendVec(4, 0.0);
			//Send Commands to Motors
			for(i=0;i<4;i++){
				sendVec[i] = this->control.u[i];
			}
			pruClientSender.setSendData(sendVec);
		}

		/************************************************************************
		*         		Check the kill Switch and Shutdown if set               *
		************************************************************************/
		if(this->setpointData.kill_switch[0] < .5) {
			char errMsg[50];
			sprintf(errMsg, "\nKill Switch Hit! Shutting Down\n");
			printf("%s",errMsg);
			// flyMS_Error_Log(errMsg);
			rc_set_state(EXITING);
		}

		//Print some stuff to the console in debug mode
		if(this->isDebugMode)
		{	
			// this->flyMS_console_print(&flyMSData);
		}
		/************************************************************************
		*         		Check for GPS Data and Handle Accordingly               *
		************************************************************************/
		this->gpsModule.getGpsData(&this->gpsData);

		/************************************************************************
		*         			Log Important Flight Data For Analysis              *
		************************************************************************/
		// log_data(&flyMSData);

		timeFinish = this->getTimeMicroseconds();
		
		// if (timeFinish - timeStart > 1E5) flyMS_Error_Log("Error timeout detected! Control Loop backed up\n");
		uint64_t sleep_time = DT_US - (timeFinish - timeStart);

		//Check to make sure the elapsed time wasn't greater than time allowed. If so don't sleep at all
		if (sleep_time < DT_US)	rc_usleep(sleep_time);

	}
	return 0;

}


uint64_t flyMS::getTimeMicroseconds()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return tv.tv_sec*(uint64_t)1E6 + tv.tv_nsec/(uint64_t)1E3;
}


int flyMS::initializeHardware()
{

	//Initialize the remote controller
	rc_initialize_dsm();

	//Pause the program until the user toggles the kill switch
	if(!this->isDebugMode)
	{	
		if(this->readyCheck()){
			printf("Exiting Program \n");
			return -1;
		}
	}

	//TODO load settings from the config file
	
	//TODO create the logging module and start

	this->initializeFilters();

	//Should be disabled by default but we don't want to be pumping 5V into our BEC ESC output
	rc_disable_servo_power_rail();
	return 0;

}


int flyMS::console_print()
{
	printf("\r ");
//	printf("time %3.3f ", control->time);
//	printf("Alt_ref %3.1f ",control->alt_ref);
//	printf(" U1:  %2.2f ",control->u[0]);
//	printf(" U2: %2.2f ",control->u[1]);
//	printf(" U3:  %2.2f ",control->u[2]);
//	printf(" U4: %2.2f ",control->u[3]);	
//	printf(" Throt %2.2f ", control->throttle);
//	printf("Aux %2.1f ", control->setpoint.Aux[0]);
//	printf("function: %f",rc_get_dsm_ch_normalized(6));
//	printf("num wraps %d ",control->num_wraps);
	// printf(" Pitch_ref %2.2f ", control->setpoint.pitch_ref);
	// printf(" Roll_ref %2.2f ", control->setpoint.roll_ref);
	// printf(" Yaw_ref %2.2f ", control->setpoint.yaw_ref[0]);
	printf(" Pitch %1.2f ", this->imuData.euler[0]);
	printf(" Roll %1.2f ", this->imuData.euler[1]);
	printf(" Yaw %2.3f ", this->imuData.euler[2]); 
//	printf(" Mag X %4.2f",control->mag[0]);
//	printf(" Mag Y %4.2f",control->mag[1]);
//	printf(" Mag Z %4.2f",control->mag[2]);
	// printf(" Accel X %4.2f",control->accel[0]);
	// printf(" Accel Y %4.2f",control->accel[1]);
	// printf(" Accel Z %4.2f",control->accel[2]);
// 	printf(" Pos N %2.3f ", control->ekf_filter.output.ned_pos[0]); 
//	printf(" Pos E %2.3f ", control->ekf_filter.output.ned_pos[1]); 
//	printf(" Pos D %2.3f ", control->ekf_filter.output.ned_pos[2]); 
	// printf(" DPitch %1.2f ", control->euler_rate[0]); 
	// printf(" DRoll %1.2f ", control->euler_rate[1]);
	// printf(" DYaw %2.3f ", control->euler_rate[2]); 	
//	printf(" uyaw %2.3f ", control->upitch); 		
//	printf(" uyaw %2.3f ", control->uroll); 		
//	printf(" uyaw %2.3f ", control->uyaw);
//	printf(" GPS pos lat: %2.2f", control->GPS_data.pos_lat);
//	printf(" GPS pos lon: %2.2f", control->GPS_data.pos_lon);
//	printf(" HDOP: %f", control->GPS_data.HDOP);
//	printf("Baro Alt: %f ",control->baro_alt);
	fflush(stdout);
	return 0;
}

int flyMS::check_output_range(float u[4])
{
	int i;
	float largest_value = 1;
	float smallest_value = 0;

	for(i=0;i<4;i++){ 
		if(u[i]>largest_value) largest_value=u[i];
		
		if(u[i]<smallest_value) u[i]=0;
	}
			
	// if upper saturation would have occurred, reduce all outputs evenly
	if(largest_value>1){
		float offset = largest_value - 1;
		for(i=0;i<4;i++) u[i]-=offset;
	}
	return 0;
}

