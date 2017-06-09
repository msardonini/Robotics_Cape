/*******************************************************************************
* linear_algebra.h
*
* A couple of functions that either weren't included in the original or needed adjustments
*******************************************************************************/
#pragma once
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <unistd.h>
#include "../../../libraries/roboticscape.h"
	
#ifndef LINEAR_ALGEBGRAMS_H
#define LINEAR_ALGEBGRAMS_H
	
// Basic Matrix creation, modification, and access
int add_vectors(rc_vector_t A,rc_vector_t B, rc_vector_t *out);
void vector_times_scalar2(rc_vector_t* v, float s, rc_vector_t* o);
int copy_matrix(rc_matrix_t A, rc_matrix_t *out);

#endif

