#include "filter.h"
#include <stdio.h>


digital_filter_t* initialize_filter(uint8_t order, float num[], float den[]){

	uint8_t i;

	digital_filter_t *filter;
	filter=malloc(sizeof(digital_filter_t) +4*(order+1)*sizeof(float));

	filter->order=order;	
	filter->filter_len=order+1;

	// First denominator term should be zero. If not, normalize the filter constants
	if (den[0] > 1.001f || den[0] < 0.999f){
		printf("Warning! Leading denominator term is not one\n");
		for (i=order; i > 0 ; i--){
			num[i] = num[i]/den[0];
			den[i] = den[i]/den[0];
			
		}
	}

	//Store the given constants in the digital filter struct
	for (i=0;i<order+1; i++){
		filter->data[i] = num[i];
		filter->data[i+filter->filter_len] = den[i];
	}

	for (i=0; i<filter->filter_len; i++){
		filter->data[i+2*filter->filter_len] = 0.0f;
		filter->data[i+3*filter->filter_len] = 0.0f;
	}

	//Set the starting index to be zero
	filter->current_index_f = 0;

	//Set an initialized flag
	filter->initialized = TRUE;
	return filter;
}

float update_filter(digital_filter_t *filter, float new_val){
	//Check if the given filter has been initialiazed
	uint8_t temp = filter->initialized;
	if (temp == FALSE){
		printf("Error!, Need to initialize filter before using it, %u\n", temp);
		return -1;
	}

	filter->data[filter->current_index_f + 2*filter->filter_len] = new_val;
	//Save a history of the amount of values needed 

	//	to apply a filter of a given order
	uint8_t i, k = 0;

	//Apply the difference Equation
	float new_filt_val = 0;
	for (i=filter->current_index_f; i < (filter->order + filter->current_index_f + 1); i++)
	{
		new_filt_val  += filter->data[k] * filter->data[(i % (filter->order + 1))+2*filter->filter_len];
		if (i == filter->current_index_f) 
		{
			k++;
			continue;
		}
		new_filt_val  -= filter->data[k + filter->filter_len] * filter->data[(i % (filter->order + 1)) + 3*filter->filter_len];
		k++;

	}
	filter->data[filter->current_index_f + 3*filter->filter_len] = new_filt_val;
	
	if (filter->current_index_f == 0) 	filter->current_index_f = filter->order;
	else		filter->current_index_f = filter->current_index_f - 1;
	
	return new_filt_val;
}


 
float saturateFilter(float filter, float min, float max){
	if(filter > max){
		filter = max;
		return filter;
	}
	else if(filter < min){
		filter = min;
		return filter;
	}
	else return filter;
}

int zeroFilter(digital_filter_t* filter){
	uint8_t i = 0;
	
	for(i=0; i<filter->filter_len; i++){
		filter->data[i + 2*filter->filter_len] = 0;
		filter->data[i + 3*filter->filter_len] = 0;
	}
	return 0;
}

int prefill_filter(digital_filter_t *filter, float value)
{

        if (filter->initialized == FALSE){
               printf("Error!, Need to initialize filter before using it\n");
               return -1;
        }
	
	uint8_t i;
	for (i = 0; i < (filter->order+1); i++)
	{
		filter->data[i + 2*filter->filter_len] = value;
		filter->data[i + 3*filter->filter_len] = value;
	}
	return 0;
}

digital_filter_t* generateFirstOrderLowPass(float dt, float time_constant){
	const float lp_const = dt/time_constant;
	float numerator[]   = {lp_const, 0};
	float denominator[] = {1, lp_const-1};
	return initialize_filter(1,numerator,denominator);
}

digital_filter_t* generateFirstOrderHighPass(float dt, float time_constant){
	float hp_const = dt/time_constant;
	float numerator[] = {1-hp_const, hp_const-1};
	float denominator[] = {1,hp_const-1};
	return initialize_filter(1,numerator,denominator);
}


digital_filter_t* generateIntegrator(float dt){
	float numerator[]   = {0, dt};
	float denominator[] = {1, -1};
	return initialize_filter(1,numerator,denominator);
}

digital_filter_t* generatePID(float kp, float ki, float kd, float Tf, float dt){
	if(Tf <= 2*dt){
		printf("Tf must be > 2kd for stability\n");
		return initialize_filter(0,0,0);
	}
	// if ki==0, return a PD filter with rolloff
	if(ki==0){
		float numerator[] = {(kp*Tf+kd)/Tf, 
							-(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								-(Tf-dt)/Tf};
		return initialize_filter(1,numerator,denominator);
	}
	//otherwise PID with roll off
	else{
		float numerator[] = {(kp*Tf+kd)/Tf, 
							(ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2.0*kd)/Tf,
							(((ki*dt-kp)*(dt-Tf))+kd)/Tf};
		float denominator[] = 	{1, 
								(dt-(2.0*Tf))/Tf, 
								(Tf-dt)/Tf};
		return initialize_filter(2,numerator,denominator);
	}
}

void print_filter(digital_filter_t *filter)
{
	uint8_t i;
	printf("\nNumerator Coefficients: \n");
	for (i = 0; i<=filter->order; i++)
	{
		printf("%2.3f, ",(double)filter->data[i]);
	}
	printf("\n");
 	printf("Denominator Coefficients: \n");
	for (i = 0; i<=filter->order; i++)
    {
        printf("%2.3f, ",(double)filter->data[i + filter->filter_len]);
    }


}

