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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

#include "flyMS.h"
#include "linear_algebra.h"
#include "roboticscape.h"
#include "flyMS.h"

#include "gps.h"
#include "kalman.h"


/************** Globals ********************/

static accel_data_t *accel_data_kal;
rc_vector_t X_state_Lat, X_state_Lon;



rc_vector_t* get_lat_state(){
	return &X_state_Lat;
}

rc_vector_t* get_lon_state(){
	return &X_state_Lon;
}


void* kalman_filter(void *ptr){		
	int i,j;
	
	rc_matrix_t A_mat, Q_mat, A_mat_tr, eye;
	rc_vector_t B_mat, H_mat;
	

	//Variable Matrices Latitude
	rc_matrix_t tmp33_Lat, Pk_Lat, tmp33_2_Lat;
	rc_vector_t tmp31_Lat, Kalman_gain_Lat;

	//Variable Matrices Longitude
	rc_matrix_t tmp33_Lon, Pk_Lon, tmp33_2_Lon;
	rc_vector_t tmp31_Lon, Kalman_gain_Lon;
	
	
	//Other Variable Matrices
	rc_matrix_t Rot_matrix;
	rc_vector_t accel_mat_global, accel_mat_local;
	

	static GPS_data_t *GPS_data;

	/************************************
	Allocate Memory for all the Matrices 
	************************************/
	
	/************************************
	Allocate Memory for all the Matrices 
	************************************/
	
	//Constant Matrices
	rc_alloc_matrix(&A_mat,3,3);
	rc_alloc_vector(&B_mat,3);
	rc_alloc_matrix(&A_mat_tr,3,3);
	rc_alloc_matrix(&eye,3,3);
	

	rc_alloc_matrix(&Q_mat,3,3);
	rc_alloc_vector(&H_mat,3);
	
	//Latitude Matrices 
	rc_alloc_vector(&X_state_Lat,3);
	rc_alloc_vector(&tmp31_Lat,3);
	rc_alloc_matrix(&tmp33_Lat,3,3);
	rc_alloc_matrix(&Pk_Lat,3,3);
	rc_alloc_vector(&Kalman_gain_Lat,3);
	rc_alloc_matrix(&tmp33_2_Lat,3,3);

	
	//Longitude Matrices
	rc_alloc_vector(&X_state_Lon,3);
	rc_alloc_vector(&tmp31_Lat,3);
	rc_alloc_matrix(&tmp33_Lon,3,3);
	rc_alloc_matrix(&Pk_Lon,3,3);
	rc_alloc_vector(&Kalman_gain_Lon,3);
	rc_alloc_matrix(&tmp33_2_Lon,3,3);
	
	//Other Matrices
	rc_alloc_vector(&accel_mat_local,3);
	rc_alloc_vector(&accel_mat_global,3);
	rc_alloc_matrix(&Rot_matrix,3,3);
	 
	float Q_matrix[][3] = Q_MATRIX;
	float A_matrix[][3] = A_MATRIX;
	float B_matrix[][1] = B_MATRIX;
	float eye_mat[][3] = EYE;
	float H_Mat[][1] = H_MATRIX;
	
	for(i=0; i<3; i++){
		X_state_Lat.d[i]=0;
		X_state_Lon.d[i]=0;
	}

	GPS_data = get_GPS_pointer();
	accel_data_kal = get_accel_pointer();

	i=0; j=0;
	for(i=0; i<3; i++){
		B_mat.d[i]=B_matrix[i][0];	
		H_mat.d[i]=H_Mat[i][0];
		for(j=0; j<3; j++){
			A_mat.d[i][j]=A_matrix[i][j];
			A_mat_tr.d[i][j]=A_matrix[i][j];
			Q_mat.d[i][j]=Q_matrix[i][j];
			eye.d[i][j]=eye_mat[i][j];
			Pk_Lon.d[i][j] = eye_mat[i][j];
			Pk_Lat.d[i][j] = eye_mat[i][j];
		}
	}
	rc_matrix_transpose(A_mat,&A_mat_tr);

	sleep(5); // wait for other things to start before running
	static int debug_cout = 0;
	
	while (rc_get_state()!=EXITING)
	{
		debug_cout++;
		
		accel_mat_local.d[0]=accel_data_kal->accel_x;
		accel_mat_local.d[1]=accel_data_kal->accel_y;
		accel_mat_local.d[2]=accel_data_kal->accel_x;
		update_rot_matrix(&Rot_matrix);
		rc_matrix_times_col_vec(Rot_matrix, accel_mat_local, &accel_mat_global);

		/*********************** LATITUDE ESTIMATION STEP KALMAN FILTER ******************************/
			//print_vector(X_state_Lat);

		//time update step of Kalman Filter
		rc_matrix_times_col_vec(A_mat, X_state_Lat, &X_state_Lat);
		vector_times_scalar2(&B_mat, accel_mat_global.d[0], &tmp31_Lat);
		add_vectors(X_state_Lat, tmp31_Lat,&X_state_Lat);

		 //Calculate Estimated Covariance Matric Pk_Lat 
		rc_multiply_matrices(A_mat, Pk_Lat, &tmp33_Lat);
		rc_multiply_matrices(tmp33_Lat, A_mat_tr, &tmp33_Lat);
		rc_add_matrices(tmp33_Lat, Q_mat, &Pk_Lat);

		 /*********************** LONGITUDE ESTIMATION STEP KALMAN FILTER ******************************/
		//time update step of Kalman Filter
		rc_matrix_times_col_vec(A_mat, X_state_Lon, &X_state_Lon);
		vector_times_scalar2(&B_mat, accel_mat_global.d[1], &tmp31_Lon);
		add_vectors(X_state_Lon, tmp31_Lon, &X_state_Lon);
		
		 //Calculate Estimated Covariance Matric Pk_Lon 
		rc_multiply_matrices(A_mat, Pk_Lon, &tmp33_Lon);
		rc_multiply_matrices(tmp33_Lon, A_mat_tr, &tmp33_Lon);
		rc_add_matrices(tmp33_Lon, Q_mat, &Pk_Lon);
		
		 /*********************** LAT/LON UPDATE STEP KALMAN FILTER ******************************/	
		if (accel_data_kal->GPS_kal_flag)
		{
			float temp_float_lat = 1/(Pk_Lat.d[0][0]+R_MATRIX);
			for (i=0;i<3;i++) Kalman_gain_Lat.d[i] = Pk_Lat.d[i][0] * temp_float_lat;
			
			//Update the estimated state accounting for measurement
			vector_times_scalar2(&Kalman_gain_Lat, GPS_data->pos_lat, &tmp31_Lat);
			//tmp31_Lat = vector_times_scalar2(&Kalman_gain_Lat, 0-X_state_Lat.d[0]);
			add_vectors(tmp31_Lat, X_state_Lat, &X_state_Lat);

			//Calculate the next Covariance Matric Pk_Lat
			rc_vector_outer_product(Kalman_gain_Lat, H_mat, &tmp33_Lat);
			rc_matrix_times_scalar(&tmp33_Lat, -1);
			rc_add_matrices(eye,tmp33_Lat, &tmp33_Lat);			
			copy_matrix(Pk_Lat, &tmp33_2_Lat); 
			rc_multiply_matrices(tmp33_Lat,tmp33_2_Lat, &Pk_Lat);

		
		
			//printf("Updating Measurement \n"); 
			float temp_float_lon = 1/(Pk_Lon.d[0][0]+R_MATRIX);
			for (i=0;i<3;i++) Kalman_gain_Lon.d[i] = Pk_Lon.d[i][0] * temp_float_lon;
			 
			 //Update the estimated state accounting for measurement
			vector_times_scalar2(&Kalman_gain_Lon, GPS_data->pos_lon, &tmp31_Lon);
			//tmp31_Lon = vector_times_scalar2(&Kalman_gain_Lon, 0-X_state_Lon.d[0]);
			add_vectors(tmp31_Lon, X_state_Lon, &X_state_Lon);

			 //Calculate the next Covariance Matric Pk_Lon
			rc_vector_outer_product(Kalman_gain_Lon, H_mat, &tmp33_Lon);
			rc_matrix_times_scalar(&tmp33_Lon, -1);
			rc_add_matrices(eye,tmp33_Lon, &tmp33_Lon);
			copy_matrix(Pk_Lon, &tmp33_2_Lon);
			rc_multiply_matrices(tmp33_Lon,tmp33_2_Lon, &Pk_Lon);	
			
			accel_data_kal->GPS_kal_flag = 0;
		}
		usleep(DT*1000000);
 	
	}
	
	
//	printf("Q mat2 \n");
	// print_matrix(Q_mat);

	// printf("A mat2\n");
	// print_matrix(A_mat);

//	printf("A mat tr2 \n");
//	 print_matrix(A_mat_tr);	 

	 printf("\n P_k Latitude\n");
	 rc_print_matrix(Pk_Lat);
   
   	 printf("\n P_k Longitude\n");
	rc_print_matrix(Pk_Lon);
   
   /*
   	printf("tmp33_Lat \n");
	print_matrix(tmp33_Lat);	

   	 printf("\n tmp33_Lon\n");
	print_matrix(tmp33_Lon);	
	 
	printf("\n tmp33_2_Lat\n");
	print_matrix (tmp33_2_Lat);
	
	printf("\n P_k tmp33_2_Lon\n");
	print_matrix(tmp33_2_Lon);
   */
   
	printf("\n X_state_Lat\n");	
	rc_print_vector(X_state_Lat);
	
	printf("\n X_state_Lon\n");	
	rc_print_vector(X_state_Lon);
   
  	printf("\n Kalman_gain_Lat\n");
	rc_print_vector(Kalman_gain_Lat);
	
	printf("\n Kalman_gain_Lon\n");	
	rc_print_vector(Kalman_gain_Lon);

	 
	//fclose(logger);
	
	//Unallocate all the vectors and matrices
	rc_free_matrix(&A_mat);
	rc_free_vector(&B_mat);
	rc_free_matrix(&A_mat_tr);
	rc_free_matrix(&eye);

	rc_free_matrix(&Q_mat);
	rc_free_vector(&H_mat);

	rc_free_vector(&X_state_Lat);
	rc_free_vector(&tmp31_Lat);
	rc_free_matrix(&tmp33_Lat);
	rc_free_matrix(&Pk_Lat);
	rc_free_vector(&Kalman_gain_Lat);
	rc_free_matrix(&tmp33_2_Lat);

	rc_free_vector(&X_state_Lon);
	rc_free_vector(&tmp31_Lon);
	rc_free_matrix(&tmp33_Lon);
	rc_free_matrix(&Pk_Lon);
	rc_free_vector(&Kalman_gain_Lon);
	rc_free_matrix(&tmp33_2_Lon);
	
	rc_free_vector(&accel_mat_local);
	rc_free_vector(&accel_mat_global);
	rc_free_matrix(&Rot_matrix);
	
	return NULL;
}

void update_rot_matrix(rc_matrix_t* Rot_matrix){
	int i, j;
	float ROTATION_MAT[][3] = ROTATION_MATRIX;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		Rot_matrix->d[i][j]=ROTATION_MAT[i][j];
		}
	}
}


#ifdef __cplusplus
}
#endif
