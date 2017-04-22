#include"linear_algebra.h"


/*******************************************************************************
* int vector_times_scalar(vector_t* v, float s)
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
* vector_t add_vectors(vecotr_t A, vector_t B)
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
* matrix_t copy_matrix(matrix_t A)
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

