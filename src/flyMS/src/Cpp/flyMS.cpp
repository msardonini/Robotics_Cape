/**
 * @file flyMS.cpp
 * @brief flyMS program source code. 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS.hpp"


flyMS::flyMS(flyMSParams &_config) :
	configModule(_config),
	firstIteration(true),
	loggingModule(_config.config.log_filepath),
	config(_config.config),
	imuModule(_config.config, this->loggingModule),
	setpointModule(_config.config, this->loggingModule),
	gpsModule(_config.config, this->loggingModule)
{

}


//Default Destructor
flyMS::~flyMS()
{
	//Join the thread if executing
	if(this->flightcoreThread.joinable())
		this->flightcoreThread.join();
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
		// printf("time diff start %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));
		
		/******************************************************************
		*			Read, Parse, and Translate IMU data for Flight		  *
		******************************************************************/
		this->imuModule.update();
		this->imuModule.getImuData(&this->imuData);

		// printf("time diff imu %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));

		/******************************************************************
		*				Take Care of Some Initialization Tasks			  *
		******************************************************************/

		if(this->firstIteration){
			this->setpointData.euler_ref[2]=this->imuData.euler[2];
			this->setpointData.yaw_ref_offset = this->imuData.euler[2];
			this->firstIteration=false;
		}

		/************************************************************************
		*                       	Get Setpoint Data                           *
		************************************************************************/
		this->setpointModule.getSetpointData(&this->setpointData);

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

		// 	//this->control.uthrottle = update_filter(this->filters.altitudeHoldPID, this->setpointData.altitudeSetpoint - flyMSData.baro_alt);
		// 	//this->setpointData.throttle = this->control.uthrottle + flyMSData.standing_throttle;
		// }
			
		
		/************************************************************************
		* 	                  		Roll Controller		                        *
		************************************************************************/
		if (this->config.flightMode == 1) // Stabilized Flight Mode
			this->setpointData.droll_setpoint = update_filter(this->filters.roll_PD, this->setpointData.euler_ref[0] - this->imuData.euler[0]);
		else if (this->config.flightMode == 2) // Acro mode
			this->setpointData.droll_setpoint = this->setpointData.euler_ref[0];
		else
		{
			this->loggingModule.flyMS_printf("[flyMS] Error! Invalid flight mode. Shutting down now");
			rc_set_state(EXITING);
			return -1;
		}

		this->imuData.eulerRate[0] = update_filter(this->filters.gyro_lpf[0], this->imuData.eulerRate[0]);
		this->control.u_euler[0] = update_filter(this->filters.roll_rate_PD,this->setpointData.droll_setpoint - this->imuData.eulerRate[0]);				
		this->control.u_euler[0] = saturateFilter(this->control.u_euler[0],-MAX_ROLL_COMPONENT,MAX_ROLL_COMPONENT);

		/************************************************************************
		* 	                  		Pitch Controller	                        *
		************************************************************************/
		if (this->config.flightMode == 1) // Stabilized Flight Mode
			this->setpointData.dpitch_setpoint = update_filter(this->filters.pitch_PD, this->setpointData.euler_ref[1] - this->imuData.euler[1]);
		else if (this->config.flightMode == 2) // Acro mode
			this->setpointData.dpitch_setpoint = this->setpointData.euler_ref[1];


		this->imuData.eulerRate[1] = update_filter(this->filters.gyro_lpf[1], this->imuData.eulerRate[1]);
		this->control.u_euler[1] = update_filter(this->filters.pitch_rate_PD,this->setpointData.dpitch_setpoint - this->imuData.eulerRate[1]);
		this->control.u_euler[1] = saturateFilter(this->control.u_euler[1],-MAX_PITCH_COMPONENT,MAX_PITCH_COMPONENT);
		
		/************************************************************************
		*                        	Yaw Controller                              *
		************************************************************************/	
		this->imuData.eulerRate[2] = update_filter(this->filters.gyro_lpf[2], this->imuData.eulerRate[2]);
		this->control.u_euler[2] = update_filter(this->filters.yaw_rate_PD,this->setpointData.euler_ref[2]-this->imuData.euler[2]);
		// this->control.u_euler[2] = update_filter(this->filters.yaw_rate_PD,this->setpointData.yaw_rate_ref[0]-this->imuData.eulerRate[2]);
		
		/************************************************************************
		*                   	Apply the Integrators                           *
		************************************************************************/	
			
		// if(this->setpointData.throttle<MIN_THROTTLE+.01){	
		// 	this->integrator_reset++;
		// 	this->integrator_start=0;
		// }else{
		// 	this->integrator_reset=0;
		// 	this->integrator_start++;
		// }
		
		// if(this->integrator_reset==300){// if landed, reset integrators and Yaw error
		// 	this->setpointData.euler_ref[2]=this->imuData.euler[2];
		// 	this->control.droll_err_integrator=0; 
		// 	this->control.dpitch_err_integrator=0;
		// 	this->control.dyaw_err_integrator=0;
		// }
			
		// //only use integrators if airborne (above minimum throttle for > 1.5 seconds)
		// if(this->integrator_start >  400){
		// 	this->control.dpitch_err_integrator += this->control.u_euler[0] * DT;
		// 	this->control.droll_err_integrator  += this->control.u_euler[1] * DT;
		// 	this->control.dyaw_err_integrator += this->control.u_euler[2] * DT;		
			
		// 	this->control.u_euler[0] += this->config.Dpitch_KI * this->control.dpitch_err_integrator;
		// 	this->control.u_euler[1] += this->config.Droll_KI * this->control.droll_err_integrator;
		// 	this->control.u_euler[2] += this->config.yaw_KI * this->control.dyaw_err_integrator;
		// }
		
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
		
		this->control.u[0]=this->setpointData.throttle-this->control.u_euler[0]+this->control.u_euler[1]-this->control.u_euler[2];
		this->control.u[1]=this->setpointData.throttle+this->control.u_euler[0]+this->control.u_euler[1]+this->control.u_euler[2];
		this->control.u[2]=this->setpointData.throttle-this->control.u_euler[0]-this->control.u_euler[1]+this->control.u_euler[2];
		this->control.u[3]=this->setpointData.throttle+this->control.u_euler[0]-this->control.u_euler[1]-this->control.u_euler[2];		

		/************************************************************************
		*         		Check Output Ranges, if outside, adjust                 *
		************************************************************************/
		this->check_output_range(this->control.u);
		
		// printf("time diff control %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));
		/************************************************************************
		*         				 Send Commands to ESCs 		                    *
		************************************************************************/
		if(!this->config.isDebugMode)
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
		if(this->setpointData.kill_switch[0] < 0.5 && !this->config.isDebugMode) {
			this->loggingModule.flyMS_printf("\nKill Switch Hit! Shutting Down\n");
			rc_set_state(EXITING);
		}

		//Print some stuff to the console in debug mode
		if(this->config.isDebugMode)
		{	
			this->console_print();
		}
		/************************************************************************
		*         		Check for GPS Data and Handle Accordingly               *
		************************************************************************/
		this->gpsModule.getGpsData(&this->gpsData);

		// printf("time diff GPS %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));
		/************************************************************************
		*         			Log Important Flight Data For Analysis              *
		************************************************************************/
		this->loggingModule.writeToLog(&this->imuData, &this->control, &this->setpointData);

		timeFinish = this->getTimeMicroseconds();
		// printf("time diff finish %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));
		
		// if (timeFinish - timeStart > 1E5) flyMS_Error_Log("Error timeout detected! Control Loop backed up\n");
		uint64_t sleep_time = DT_US - (timeFinish - timeStart);

		//Check to make sure the elapsed time wasn't greater than time allowed. If so don't sleep at all
		if (sleep_time < DT_US)	rc_usleep(sleep_time);
		else this->loggingModule.flyMS_printf("[flyMS] Error! Control thread too slow! time in milliseconds: %u \n", (timeFinish - timeStart)/1000);

	}
	return 0;

}


uint64_t flyMS::getTimeMicroseconds()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return (uint64_t)tv.tv_sec*1E6 + (uint64_t)tv.tv_nsec/1E3;
}


int flyMS::console_print()
{
	this->loggingModule.flyMS_printf("\r ");
//	this->loggingModule.flyMS_printf("time %3.3f ", control->time);
//	this->loggingModule.flyMS_printf("Alt_ref %3.1f ",control->alt_ref);
//	this->loggingModule.flyMS_printf(" U1:  %2.2f ",control->u[0]);
//	this->loggingModule.flyMS_printf(" U2: %2.2f ",control->u[1]);
//	this->loggingModule.flyMS_printf(" U3:  %2.2f ",control->u[2]);
//	this->loggingModule.flyMS_printf(" U4: %2.2f ",control->u[3]);	
//	this->loggingModule.flyMS_printf("Aux %2.1f ", control->setpoint.Aux[0]);
//	this->loggingModule.flyMS_printf("function: %f",rc_get_dsm_ch_normalized(6));
//	this->loggingModule.flyMS_printf("num wraps %d ",control->num_wraps);
	// this->loggingModule.flyMS_printf(" Throt %2.2f ", this->setpointData.throttle);
	 this->loggingModule.flyMS_printf(" Roll_ref %2.2f ", this->setpointData.euler_ref[0]);
	 this->loggingModule.flyMS_printf(" Pitch_ref %2.2f ", this->setpointData.euler_ref[1]);
	 this->loggingModule.flyMS_printf(" Yaw_ref %2.2f ", this->setpointData.euler_ref[2]);
	this->loggingModule.flyMS_printf(" Roll %1.2f ", this->imuData.euler[0]);
	this->loggingModule.flyMS_printf(" Pitch %1.2f ", this->imuData.euler[1]);
	this->loggingModule.flyMS_printf(" Yaw %2.3f ", this->imuData.euler[2]); 
//	this->loggingModule.flyMS_printf(" Mag X %4.2f",control->mag[0]);
//	this->loggingModule.flyMS_printf(" Mag Y %4.2f",control->mag[1]);
//	this->loggingModule.flyMS_printf(" Mag Z %4.2f",control->mag[2]);
	// this->loggingModule.flyMS_printf(" Accel X %4.2f",control->accel[0]);
	// this->loggingModule.flyMS_printf(" Accel Y %4.2f",control->accel[1]);
	// this->loggingModule.flyMS_printf(" Accel Z %4.2f",control->accel[2]);
// 	this->loggingModule.flyMS_printf(" Pos N %2.3f ", control->ekf_filter.output.ned_pos[0]); 
//	this->loggingModule.flyMS_printf(" Pos E %2.3f ", control->ekf_filter.output.ned_pos[1]); 
//	this->loggingModule.flyMS_printf(" Pos D %2.3f ", control->ekf_filter.output.ned_pos[2]); 
	// this->loggingModule.flyMS_printf(" DPitch %1.2f ", control->euler_rate[0]); 
	// this->loggingModule.flyMS_printf(" DRoll %1.2f ", control->euler_rate[1]);
	// this->loggingModule.flyMS_printf(" DYaw %2.3f ", control->euler_rate[2]); 	
//	this->loggingModule.flyMS_printf(" uyaw %2.3f ", control->upitch); 		
//	this->loggingModule.flyMS_printf(" uyaw %2.3f ", control->uroll); 		
//	this->loggingModule.flyMS_printf(" uyaw %2.3f ", control->uyaw);
//	this->loggingModule.flyMS_printf(" GPS pos lat: %2.2f", control->GPS_data.pos_lat);
//	this->loggingModule.flyMS_printf(" GPS pos lon: %2.2f", control->GPS_data.pos_lon);
//	this->loggingModule.flyMS_printf(" HDOP: %f", control->GPS_data.HDOP);
//	this->loggingModule.flyMS_printf("Baro Alt: %f ",control->baro_alt);
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

