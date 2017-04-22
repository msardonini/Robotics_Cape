#pragma once
#include <stdint.h>
#include <stdlib.h>

#ifndef FILTER_H
#define FILTER_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/** \var typedef struct digital_filter_t
    \brief A type definition for a struct that contains all the info needed for a digital filter    
*/
typedef struct digital_filter_t{

	uint8_t order; 							/**< Specified order of the filter*/
	uint8_t filter_len; 							/**< Specified order of the filter*/
	uint8_t current_index_f;				/**< Index showing where in the vector current and past data is */
	uint8_t initialized;					/**< Boolean saying if filter has been initialized*/
	float data[];							/**< Denominator Coefficients*/

} digital_filter_t;






/* 
--- March Filter ---
march the filter forward in time one step with new input data
returns new output which could also be accessed with filter.current_output
*/
float update_filter(digital_filter_t *filter, float new_val);


/* 
--- Saturate Filter ---
limit the output of filter to be between min&max
returns 1 if saturation was hit 
returns 0 if output was within bounds
*/
float saturateFilter(float filter, float min, float max);


/*
--- Zero Filter ---
reset all input and output history to 0
*/
int zeroFilter(digital_filter_t* filter);

/*
--- PreFill Filter ---
fill the past inputs with the curent input
use before marchFilter when starting to avoid ugly step input
*/
int prefill_filter(digital_filter_t *filter, float value);


/*
--- Generate Filter ---
Dynamically allocate memory for a filter of specified order
and set transfer function constants.

Note: A normalized transfer function should have a leading 1 
in the denominator but can be !=1 in this library
*/
digital_filter_t* initialize_filter(uint8_t order, float num[], float den[]);

// discrete-time implementation of a parallel PID controller with derivative filter
// similar to Matlab pid command
//
// N is the pole location for derivative filter. Must be greater than 2*DT
// smaller N gives faster filter decay
digital_filter_t* generatePID(float kp, float ki, float kd, float Tf, float dt);

// print order, numerator, and denominator constants
void print_filter(digital_filter_t *filter);




#endif
