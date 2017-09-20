/*
Copyright (c) 2014, Mike Sardonini
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

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
--- Initialize Filter ---
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
