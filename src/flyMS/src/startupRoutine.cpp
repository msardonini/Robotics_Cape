/**
 * @file startupRoutine.cpp
 * @brief Initialize the flight hardware 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */




int flyMS::startupRoutine()
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