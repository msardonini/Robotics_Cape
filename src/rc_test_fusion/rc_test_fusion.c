/*******************************************************************************
* rc_test_imu.c
*
* This serves as an example of how to read the IMU with direct reads to the
* sensor registers. To use the DMP or interrupt-driven timing see test_dmp.c
*******************************************************************************/

#include "rc_usefulincludes.h"
#include "roboticscape.h"
#include "Fusion.h"
#include "filter.h"
#include <inttypes.h>
#include <stdbool.h>
int init_fusion(FusionBias *fusionBias);

// possible modes, user selected with command line arguments
typedef enum m_mode_t{
	RAD,
	DEG,
	RAW
} m_mode_t;

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf("-r		print raw values instead of radians\n");
	printf("-d		print gyro in degrees/s instead of radians\n");
	printf("-h		print this help message\n");
	printf("\n");
}

int main(int argc, char *argv[]){
	rc_imu_data_t data; //struct to hold new data
	int c;
	m_mode_t mode = RAD; // default to radian mode.
	bool console_output = true;
	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "rdhi")) != -1){
		switch (c){
		case 'r':
			if(mode!=RAD) print_usage();
			mode = RAW;
			printf("\nRaw values are from 16-bit ADC\n");
			break;
		case 'd':
			if(mode!=RAD) print_usage();
			mode = DEG;
			break;
		case 'h':
			print_usage();
			return 0;
			break;
		case 'i':
			console_output = false;
			break;
		default:
			print_usage();
			return -1;
			break;
		}
	}

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	// use defaults for now, except also enable magnetometer.
	rc_imu_config_t conf = rc_default_imu_config();
	conf.enable_magnetometer=1;

	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}

	// print a header
	if (console_output)
	{
		printf("\ntry 'test_imu -h' to see other options\n\n");
		switch(mode){
		case RAD:
			printf("   Accel XYZ(m/s^2)  |");
			printf("   Gyro XYZ (rad/s)  |");
			break;
		case DEG:
			printf("   Accel XYZ(m/s^2)  |");
			printf("   Gyro XYZ (deg/s)  |");
			break;
		case RAW:
			printf("  Accel XYZ(raw adc) |");
			printf("  Gyro XYZ (raw adc) |");
			break;
		default:
			printf("ERROR: invalid mode\n");
			return -1;
		}
		printf("  Mag Field XYZ(uT)  |");
		printf(" Temp (C)");
		printf("\n");
	}
	

	FILE *fd;
	fd = fopen("logger.csv","w+");
	fprintf(fd,"time,roll,pitch,yaw\n");

	FusionAhrs  fusionAhrs;
	FusionAhrsInitialise(&fusionAhrs, 8.0f, 0.0f, 70.0f); // valid magnetic field defined as 20 uT to 70 uT
	
	FusionBias fusionBias;
	init_fusion(&fusionBias);

	digital_filter_t *accel_lpf[3];

	// int i = 0;
	// for (i = 0; i < 3; i++)
	// 	accel_lpf[i] = initialize_filter(4, (float[5]){0.0940f, 0.3759f, 0.5639f, 0.3759f, 0.0940f},(float[5]){1.0000f, 0.0000f, 0.4860f, 0.0000f, 0.0177f});


	struct timeval start_time, start_loop, end_loop;
	uint64_t start_time_usec, start_loop_usec, end_loop_usec;
	uint64_t sleep_time;
	
	//Set the start time of the program
	gettimeofday(&start_time,NULL);
	start_time_usec = start_time.tv_sec*(uint64_t)1E6 + start_time.tv_usec;

	//now just wait, print_data will run
	while (rc_get_state() != EXITING) 
	{
		//Set start time of the loop
		gettimeofday(&start_loop,NULL);
		start_loop_usec = start_loop.tv_sec*(uint64_t)1E6 + start_loop.tv_usec;
		if(console_output)
			printf("\r");		
		// print accel
		if(rc_read_accel_data(&data)<0){
			printf("read accel data failed\n");
		}
		if (console_output)
		{		
			if(mode==RAW){
				printf("%6d %6d %6d |",			data.raw_accel[0],\
												data.raw_accel[1],\
												data.raw_accel[2]);
			}
			else{
				printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
												data.accel[1],\
												data.accel[2]);
			}
		}
		
		// print gyro data
		if(rc_read_gyro_data(&data)<0){
			printf("read gyro data failed\n");
		}
		if (console_output)
		{
			switch(mode){
			case RAD:
				printf("%6.1f %6.1f %6.1f |",	data.gyro[0]*DEG_TO_RAD,\
												data.gyro[1]*DEG_TO_RAD,\
												data.gyro[2]*DEG_TO_RAD);
				break;
			case DEG:
				printf("%6.1f %6.1f %6.1f |",	data.gyro[0],\
												data.gyro[1],\
												data.gyro[2]);
				break;
			case RAW:
				printf("%6d %6d %6d |",			data.raw_gyro[0],\
												data.raw_gyro[1],\
												data.raw_gyro[2]);
				break;
			default:
				printf("ERROR: invalid mode\n");
				return -1;
			}
		}

		// read magnetometer
		if(rc_read_mag_data(&data)<0){
			printf("read mag data failed\n");
		}	
		else if (console_output) 
			printf("%6.1f %6.1f %6.1f |",	data.mag[0],\
											data.mag[1],\
											data.mag[2]);

		// read temperature
		if(rc_read_imu_temp(&data)<0){
			printf("read temp data failed\n");
		}
		else if (console_output)
			printf(" %4.1f ", data.temp);



		const FusionVector3 gyroscope = 
		{
			.axis.x = data.gyro[0],
			.axis.y = data.gyro[1],
			.axis.z = data.gyro[2],
		}; // literal values should be replaced with sensor measurements

		const FusionVector3 accelerometer = 
		{
			// .axis.x = update_filter(accel_lpf[0],data.raw_accel[0]/9.81f),
			// .axis.y = update_filter(accel_lpf[1],data.raw_accel[1]/9.81f),
			// .axis.z = update_filter(accel_lpf[2],data.raw_accel[2]/9.81f),
			.axis.x = data.raw_accel[0]/9.81f,
			.axis.y = data.raw_accel[1]/9.81f,
			.axis.z = data.raw_accel[2]/9.81f,
		}; // literal values should be replaced with sensor measurements

		const FusionVector3 magnetometer = 
		{
			.axis.x = data.mag[0],
			.axis.y = data.mag[1],
			.axis.z = data.mag[2],
		}; // literal values should be replaced with sensor measurements
 		FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, magnetometer, 0.01f); // assumes 100 Hz sample rate												
		FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(fusionAhrs.quaternion);
		if (console_output)
		{	
			printf("%f, %f, %f,%llu", eulerAngles.angle.pitch,eulerAngles.angle.roll,eulerAngles.angle.yaw, start_loop_usec-start_time_usec);
		}
		fflush(stdout);
		fprintf(fd, "%llu,%f,%f,%f\n", start_loop_usec-start_time_usec,  eulerAngles.angle.roll,eulerAngles.angle.pitch,eulerAngles.angle.yaw);
		
		//Set time at the end of loop
		gettimeofday(&end_loop,NULL);
		end_loop_usec = end_loop.tv_sec*(uint64_t)1E6 + end_loop.tv_usec;
		sleep_time = (uint64_t)5000 - (end_loop_usec - start_loop_usec);
		//sleep for the elapsed time
		rc_usleep(sleep_time);
	}
	fflush(stdout);
	rc_power_off_imu();
	rc_cleanup();
	return 0;
}



int init_fusion(FusionBias *fusionBias)
{
	rc_imu_data_t data; //struct to hold new data
	// print gyro data
	if(rc_read_gyro_data(&data)<0){
		printf("read gyro data failed\n");
	}
    FusionBiasInitialise(fusionBias, 50, 0.01f); // assumes 100 Hz sample rate
    FusionBiasUpdate(fusionBias, data.gyro[0], data.gyro[1], data.gyro[2]); // literal values should be replaced with sensor measurements
    return 0;
}
