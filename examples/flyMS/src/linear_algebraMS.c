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


#include"linear_algebra.h"


/*******************************************************************************
* int vector_times_scalar(rc_vector_t* v, float s)
*
* 
*******************************************************************************/
void vector_times_scalar2(rc_vector_t* v, float s, rc_vector_t* o){
	int i;
	if(!v->initialized){
		printf("ERROR: vector not initialized yet\n");
	}
	for(i=0;i<(v->len);i++){	
		o->d[i] = s*v->d[i];
	}
}



/*******************************************************************************
* rc_vector_t add_vectors(vecotr_t A, rc_vector_t B)
*
* 
*******************************************************************************/
int add_vectors(rc_vector_t A, rc_vector_t B, rc_vector_t *out){
	int i;
	
	if(!A.initialized||!B.initialized){
		printf("ERROR: vector not initialized yet\n");
		return -1;
	}
	if (A.len != B.len){
		printf("Invalid vector sizes");
		return -1;
	}
	for(i=0;i<(A.len);i++){	
			out->d[i] = A.d[i] + B.d[i];
		
	}
	return 0;
}

/*******************************************************************************
* rc_matrix_t copy_matrix(rc_matrix_t A)
*
* 
*******************************************************************************/
int copy_matrix(rc_matrix_t A, rc_matrix_t *temp){
	int i,j;
	if(!A.initialized){
		printf("ERROR: matrix not initialized yet\n");
		return -1;
	}
	// create new matrix
	for(i=0;i<(A.rows);i++){
		for(j=0;j<(A.cols);j++){	
			temp->d[i][j] = A.d[i][j];
		}
	}
	return 0;
}

