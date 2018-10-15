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
	pruClientSender()
{

}


//Default Destructor
flyMS::~flyMS()
{

}


int flyMS::flight_core()
{

	imu_err_count = 0;

	int i=0;

	//Variables to Initialize things upon startup
	static uint8_t First_Iteration=1;
	
	// float initial_alt = 0;
	//Initialize some variables if it is the first iteration

	
	//Set the reference time to the first iteration
	function_control.start_time_usec = get_usec_timespec(&function_control.start_time);

	flyMSData.setpoint.Aux[0] = 1; flyMSData.setpoint.kill_switch[0]=1;
	function_control.dsm2_timeout=0;
	rc_read_barometer();
	// initial_alt = rc_bmp_get_altitude_m();
	rc_set_state(RUNNING);

	while(rc_get_state()!=EXITING)
	{

		/******************************************************************
		*			Grab the time for Periphal Apps and Logs 			  *
		******************************************************************/
		function_control.start_loop_usec = get_usec_timespec(&function_control.start_loop);
		flyMSData.time_us = function_control.start_loop_usec - function_control.start_time_usec;
		
		/******************************************************************
		*			Read, Parse, and Translate IMU data for Flight		  *
		******************************************************************/
		imu_handler(&flyMSData, &filters);
		imu_err_count = 0;

		/******************************************************************
		*				Take Care of Some Initialization Tasks			  *
		******************************************************************/

		if(First_Iteration){
			flyMSData.setpoint.euler_ref[2]=flyMSData.state.euler[2];
			First_Iteration=0;
			flyMSData.setpoint.yaw_ref_offset = flyMSData.state.euler[2];
			printf("First Iteration Started \n");
		}

		/************************************************************************
		*                   	Throttle Controller                             *
		************************************************************************/

		//	float throttle_compensation = 1 / cos(flyMSData.state.euler[0]);
		//	throttle_compensation *= 1 / cos(flyMSData.state.euler[1]);		

		if(function_control.altitudeHold)
		{
			if(function_control.altitudeHoldFirstIteration)
			{
				flyMSData.control.standing_throttle = flyMSData.setpoint.throttle;
				flyMSData.setpoint.altitudeSetpoint = flyMSData.imu.baro_alt;
				function_control.altitudeHoldFirstIteration = 0;
			}
	       // flyMSData.setpoint.altitudeSetpoint=flyMSData.setpoint.altitudeSetpoint+(flyMSData.setpoint.altitudeSetpointRate)*DT;

			//flyMSData.control.uthrottle = update_filter(filters.altitudeHoldPID, flyMSData.setpoint.altitudeSetpoint - flyMSData.baro_alt);
			//flyMSData.setpoint.throttle = flyMSData.control.uthrottle + flyMSData.standing_throttle;
		}
			
		/************************************************************************
		* 	                  		Pitch Controller	                        *
		************************************************************************/
		flyMSData.setpoint.dpitch_setpoint = update_filter(filters.pitch_PD, flyMSData.setpoint.euler_ref[0] - flyMSData.state.euler[0]);
		flyMSData.control.u_euler[0] = update_filter(filters.pitch_rate_PD,flyMSData.setpoint.dpitch_setpoint - flyMSData.state.euler_rate[0]);
		flyMSData.control.u_euler[0] = saturateFilter(flyMSData.control.u_euler[0],-MAX_PITCH_COMPONENT,MAX_PITCH_COMPONENT);
		
		/************************************************************************
		* 	                  		Roll Controller		                        *
		************************************************************************/
		flyMSData.setpoint.droll_setpoint = update_filter(filters.roll_PD, flyMSData.setpoint.euler_ref[1] - flyMSData.state.euler[1]);
		flyMSData.control.u_euler[1] = update_filter(filters.roll_rate_PD,flyMSData.setpoint.droll_setpoint - flyMSData.state.euler_rate[1]);				
		flyMSData.control.u_euler[1] = saturateFilter(flyMSData.control.u_euler[1],-MAX_ROLL_COMPONENT,MAX_ROLL_COMPONENT);
		
		/************************************************************************
		*                        	Yaw Controller                              *
		************************************************************************/	
		// flyMSData.control.u_euler[2] = update_filter(filters.yaw_rate_PD,flyMSData.setpoint.euler_ref[2]-flyMSData.state.euler[2]);
		flyMSData.control.u_euler[2] = update_filter(filters.yaw_rate_PD,flyMSData.setpoint.yaw_rate_ref[0]-flyMSData.state.euler_rate[2]);
		
		/************************************************************************
		*                   	Apply the Integrators                           *
		************************************************************************/	
			
		if(flyMSData.setpoint.throttle<MIN_THROTTLE+.01){	
			function_control.integrator_reset++;
			function_control.integrator_start=0;
		}else{
			function_control.integrator_reset=0;
			function_control.integrator_start++;
		}
		
		if(function_control.integrator_reset==300){// if landed, reset integrators and Yaw error
			flyMSData.setpoint.euler_ref[2]=flyMSData.state.euler[2];
			flyMSData.control.droll_err_integrator=0; 
			flyMSData.control.dpitch_err_integrator=0;
			flyMSData.control.dyaw_err_integrator=0;
		}
			
		//only use integrators if airborne (above minimum throttle for > 1.5 seconds)
		if(function_control.integrator_start >  400){
			flyMSData.control.dpitch_err_integrator += flyMSData.control.u_euler[0] * DT;
			flyMSData.control.droll_err_integrator  += flyMSData.control.u_euler[1]  * DT;
			flyMSData.control.dyaw_err_integrator += flyMSData.control.u_euler[2] * DT;		
			
			flyMSData.control.u_euler[0] += flyMSData.flight_config.Dpitch_KI * flyMSData.control.dpitch_err_integrator;
			flyMSData.control.u_euler[1] += flyMSData.flight_config.Droll_KI * flyMSData.control.droll_err_integrator;
			flyMSData.control.u_euler[2] += flyMSData.flight_config.yaw_KI * flyMSData.control.dyaw_err_integrator;
		}
		
		//Apply a saturation filter
		flyMSData.control.u_euler[2] = saturateFilter(flyMSData.control.u_euler[2],-MAX_YAW_COMPONENT,MAX_YAW_COMPONENT);
		
		/************************************************************************
		*  Mixing
		*           	      black				yellow
		*                          CCW 1	  2 CW		IMU Orientation:	
		*                          	   \ /				Y
		*	                           / \            	|_ X
		*                         CW 3	  4 CCW
		*                 	  yellow       	    black
		************************************************************************/
		
		flyMSData.control.u[0]=flyMSData.setpoint.throttle+flyMSData.control.u_euler[1]+flyMSData.control.u_euler[0]-flyMSData.control.u_euler[2];
		flyMSData.control.u[1]=flyMSData.setpoint.throttle-flyMSData.control.u_euler[1]+flyMSData.control.u_euler[0]+flyMSData.control.u_euler[2];
		flyMSData.control.u[2]=flyMSData.setpoint.throttle+flyMSData.control.u_euler[1]-flyMSData.control.u_euler[0]+flyMSData.control.u_euler[2];
		flyMSData.control.u[3]=flyMSData.setpoint.throttle-flyMSData.control.u_euler[1]-flyMSData.control.u_euler[0]-flyMSData.control.u_euler[2];		

		/************************************************************************
		*         		Check Output Ranges, if outside, adjust                 *
		************************************************************************/
		check_output_range(flyMSData.control.u);
		
		/************************************************************************
		*         				 Send Commands to ESCs 		                    *
		************************************************************************/
		if(!flyMSData.flight_config.enable_debug_mode)
		{
			//Send Commands to Motors
			for(i=0;i<4;i++){
				pru_client_data.u[i] = flyMSData.control.u[i];
				pru_client_data.send_flag = 1;
			}
		}

		/************************************************************************
		*         		Check the kill Switch and Shutdown if set               *
		************************************************************************/
		if(flyMSData.setpoint.kill_switch[0] < .5) {
			char errMsg[50];
			sprintf(errMsg, "\nKill Switch Hit! Shutting Down\n");
			printf("%s",errMsg);
			flyMS_Error_Log(errMsg);
			rc_set_state(EXITING);
		}

		//Print some stuff to the console in debug mode
		if(flyMSData.flight_config.enable_debug_mode)
		{	
			flyMS_console_print(&flyMSData);
		}
		/************************************************************************
		*         		Check for GPS Data and Handle Accordingly               *
		************************************************************************/
		GPS_handler(&flyMSData, &GPS_data);
		
		/************************************************************************
		*         			Log Important Flight Data For Analysis              *
		************************************************************************/
		log_data(&flyMSData);

		function_control.end_loop_usec = get_usec_timespec(&function_control.end_loop);
		
		if (function_control.end_loop_usec - function_control.start_loop_usec > 1E6) printf("Error timeout detected!\n");
		uint64_t sleep_time = DT_US - (function_control.end_loop_usec -
										function_control.start_loop_usec);

		//Check to make sure the elapsed time wasn't greater than time allowed. If so don't sleep at all
		if (sleep_time < DT_US)	rc_usleep(sleep_time);

	}
	return NULL;

}


uint64_t flyMS::getTimeMicroseconds()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return tv.tv_sec*(uint64_t)1E6 + tv.tv_nsec/(uint64_t)1E3;
}


int flyMS::initializeHardware()
{

	//Starts the pru_client which will send commands to ESCs
	start_pru_client(pru_client_data);

	//Initialize the remote controller
	rc_initialize_dsm();

	//Pause the program until the user toggles the kill switch
	if(!control->flight_config.enable_debug_mode)
	{	
		if(ready_check()){
			printf("Exiting Program \n");
			return -1;
		}
	}

	// load flight_core settings
	int force_debug_mode = control->flight_config.enable_debug_mode;
	if(load_core_config(&control->flight_config)){
		printf("WARNING: no configuration file found\n");
		printf("loading default settings\n");
		if(create_default_core_config_file(&control->flight_config)){
			printf("Warning, can't write default flight_config file\n");
		}
	}
	control->flight_config.enable_debug_mode = (force_debug_mode || control->flight_config.enable_debug_mode);
	
	//Initialize the IMU and Barometer
	initialize_imu(control);

	//Enable the data logger
	if(control->flight_config.enable_logging)
	{
		logger_init();
	}

	//Enable the data logger
	initialize_setpoint_manager(control);

	
	/* --------- Start the EKF for position estimates ----*/
//	pthread_create(&flyMS_threads->ekf_thread, NULL, run_ekf, control->ekf_filter );

	initialize_filters(filters, &control->flight_config);

	/* 			Start the GPS 				*/
	if(control->flight_config.enable_gps)
	{
		GPS_data->GPS_init_check=GPS_init(GPS_data);
	}
	//Should be disabled by default but we don't want to be pumping 5V into our BEC ESC output
	rc_disable_servo_power_rail();
	return 0;

}