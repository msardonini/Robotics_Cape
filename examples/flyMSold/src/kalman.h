#ifndef KALMAN_H
#define KALMAN_H

#pragma once
#include "flyMS.h"

void update_rot_matrix(rc_matrix_t* Rot_matrix);
void* kalman_filter(void *ptr);
rc_vector_t* get_lat_state();
rc_vector_t* get_lon_state();


#endif