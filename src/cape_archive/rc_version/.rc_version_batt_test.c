/*******************************************************************************
* rc_version.c
*
* Prints the current version of the robotics cape library. 
*******************************************************************************/

#include "../../libraries/rc_usefulincludes.h"
#include "../../libraries/roboticscape.h"

int main(){
	// simply print the version and a newline
	printf("%s\n", rc_version_string());
	// alternatively you could use rc_version_float() for numeric comparisons

	rc_initialize();
	rc_set_state(RUNNING);
	float volts;
	while(rc_get_state()!=EXITING)
	{	
		volts = rc_dc_jack_voltage();
		printf("volts %f\r",volts);
		usleep(1000000/200);
		fflush(stdout);
	}


	return 0;
}
