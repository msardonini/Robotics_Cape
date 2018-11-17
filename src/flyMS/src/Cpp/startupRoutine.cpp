/**
 * @file startupRoutine.cpp
 * @brief Initialize the flight hardware 
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */


#include "flyMS.hpp"

int flyMS::startupRoutine()
{
	//Initialize the remote controller through the setpoint object
	if(this->setpointModule.start())
		std::cerr<<"[flyMS] Error initializing Radio Coms!" << std::endl;


	//Pause the program until the user toggles the kill switch
	if(!this->config.isDebugMode || 1)
	{	
		if(this->readyCheck()){
			printf("Exiting Program \n");
			return -1;
		}
	}
	return 0;
}


int flyMS::readyCheck(){
	//Toggle the kill switch to get going, to ensure controlled take-off
	//Keep kill switch down to remain operational
    int count=1, toggle = 0, reset_toggle = 0;
	float val[2] = {0.0f , 0.0f};
	printf("Toggle the kill swtich twice and leave up to initialize\n");
	while(count<6 && rc_get_state()!=EXITING)
	{
		
		//Blink the green LED light to signal that the program is ready
		reset_toggle++; // Only blink the led 1/100 the time this loop runs
		if(toggle)
		{
			rc_led_set(RC_LED_GREEN, 0);
			if (reset_toggle == 20) 
			{
				toggle = 0;
				reset_toggle = 0;
			}
		}
		else
		{
			rc_led_set(RC_LED_GREEN, 1);
			if (reset_toggle == 20)
			{
				toggle=1;
				reset_toggle = 0;
			}
		}

		this->setpointModule.getSetpointData(&this->setpointData);

		val[1]=this->setpointData.kill_switch[1];
		val[0]=this->setpointData.kill_switch[0];
		if(val[0] < -0.75 && val[1] > 0.35)
			count++;
		if(val[0] > 0.75 && val[1] < 0.35)
			count++;

			
		usleep(10000);
	}
	
	//make sure the kill switch is in the position to fly before starting
	while(val[0] < 0.5 && rc_get_state()!=EXITING)
		{
		if(rc_dsm_is_new_data()){
			val[0]=rc_dsm_ch_normalized(5);	
			}
		usleep(10000);
		}
	
	if(rc_get_state() == EXITING)
	{
		printf("State set to exiting, shutting off! \n");
		return -1;
	}
	
	printf("\nInitialized! Starting program\n");
	rc_led_set(RC_LED_GREEN, 1);
	return 0;
}


/************************************************************************
*	initialize_filters()
*	setup of feedback controllers used in flight core
************************************************************************/
int flyMS::initializeFilters()
{

	// filters->pitch_PD = generatePID(flight_config->pitch_KP, flight_config->pitch_KI, flight_config->pitch_KD, 0.15, DT);
	// filters->roll_PD  = generatePID(flight_config->roll_KP, flight_config->roll_KI, flight_config->roll_KD, 0.15, DT);
	// //filters->yaw_PD   = generatePID(YAW_KP,		  0, YAW_KD,	    0.15, 0.005);

	// //PD Controller (I is done manually)
	// filters->pitch_rate_PD = generatePID(flight_config->Dpitch_KP, 0, flight_config->Dpitch_KD, 0.15, DT);
	// filters->roll_rate_PD  = generatePID(flight_config->Droll_KP, 0, flight_config->Droll_KD, 0.15, DT);
	// filters->yaw_rate_PD   = generatePID(flight_config->yaw_KP,		  0, flight_config->yaw_KD,	    0.15, DT);
	
	// //Gains on Low Pass Filter for raw gyroscope output
	
	// filters->altitudeHoldPID  = generatePID(.05,		  .005,  .002,	    0.15, DT);

	// //elliptic filter 10th order 0.25 dB passband ripple 80 dB min Cutoff 0.4 cutoff frq
	// float num[11] = {   0.003316345545497,   0.006003204398448,   0.015890122416480,   0.022341342884745,   0.031426841006402,
	// 					0.032682319166147,   0.031426841006402,  0.022341342884745,   0.015890122416480,   0.006003204398448,
	// 					0.003316345545497};

	// float den[11] = {   1.000000000000000,  -4.302142513532524,  10.963685193359051, -18.990960386921738,  24.544342262847074,
	// 					-24.210021253402012,  18.411553079753368, -10.622846105856944,   4.472385466696109,  -1.251943621469692,
	// 					0.182152641224648};
	
	// //elliptic filter 10th order 0.25 dB passband ripple 80 dB min Cutoff 0.05 cutoff frq

	// float yaw_num[11] = {0.000000138467082,0.000001384670818,0.000006231018679,0.000016616049812,0.000029078087171,0.000034893704605,0.000029078087171,0.000016616049812,0.000006231018679,0.000001384670818,0.000000138467082};
	// float yaw_den[11] = {1.000000000000000,-6.989417698566569,22.323086726703938,-42.824608705880635,54.570406893265300,-48.208486634295596,29.872790631313180,-12.810698156370698,3.636160614880030,-0.616474419461443,0.047382538704228};

	// int i;
	// for (i = 0; i < 2; i++)
	// {
	// 	filters->gyro_lpf[i] = initialize_filter(10, num, den);		
	// 	filters->accel_lpf[i] = initialize_filter(10, num, den);	
	// }

	// filters->gyro_lpf[2] = initialize_filter(10, yaw_num, yaw_den);		
	// filters->accel_lpf[2] = initialize_filter(10, num, den);	
	
	// //Gains on Low Pass Filter for Yaw Reference		
	// float num2[4] = {  0.0317,    0.0951,    0.0951,    0.0317};
	// float den2[4] = { 1.0000,   -1.4590,    0.9104,   -0.1978};					
	// filters->LPF_Yaw_Ref_P = initialize_filter(3, num2, den2);							
	// filters->LPF_Yaw_Ref_R = initialize_filter(3, num2, den2);		

	// //ellip filter, 5th order .5 pass 70 stop .05 cutoff
	// float baro_num[6] = {0.000618553374672,  -0.001685890697737,   0.001077182625629,   0.001077182625629,  -0.001685890697737,   0.000618553374672};
	// float baro_den[6] =	{1.000000000000000,  -4.785739467762915,   9.195509273069447,  -8.866262182166356,   4.289470039368545,  -0.832957971903594};
	// filters->LPF_baro_alt = initialize_filter(5, baro_num, baro_den);
	// for (i = 0; i< 3; i++)
	// {
	// 	zeroFilter(filters->gyro_lpf[i]);
	// 	zeroFilter(filters->accel_lpf[i]);
	// }

	return 0;
}