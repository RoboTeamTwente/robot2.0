/*
 * Kalman.c
 *
 *  Created on: Feb 19, 2019
 *      Author: kjhertenberg
 */

/*
arm_matrix_instance_f32 makes a matrix object
arm_mat_init_f32(&matrix, row, columns, (float32_t *)array) maps the float32_t array onto the matrix with a certain amount of rows and columns
arm_mat_mult_f32(&A, &B, &C) puts the dot product of matrix A dot matrix B into matrix C
arm_mat_trans_f32(&A, &B) puts the transpose of matrix A into matrix B
arm_mat_inverse_f32(&A, &B) puts the inverse of matrix A into matrix B
Note: The inverse operation does edit the original matrix
Note: A B and C should all be different matrices otherwise the results turn weird

The matrix indexes can be read from the arrays that they are connected with
edits in the array means edits the matrix so each matrix needs it's own array basically

*/

#include "Kalman.h"
#include "KalmanV.h"
#include "../PuttyInterface/PuttyInterface.h"

void Kalman_init(){
	transMatrix(aH, aHt, OBSERVE, STATE);
	transMatrix(aF, aFt, STATE, STATE);
}


void KalmanState(float accel[2], float vel[2], float controlInput[STATE]){

	// Predict
//	for (int i = 0; i < STATE; i++) {
//		aU[i] = controlInput[i];
//	}

	multiplyMatrix(aF, aXold, aFX, STATE, 1, STATE);
	multiplyMatrix(aB, aU, aBU, STATE, 1, STATE);
	addMatrix(aFX, aBU, aXcurrent, STATE);

	// Get measurement
	az[0] = vel[0];
	az[1] = accel[0];
	az[2] = vel[1];
	az[3] = accel[1];

	// Process data
	multiplyMatrix(aH, aXcurrent, aHX, OBSERVE, 1, STATE);
	subMatrix(az, aHX, ayold, OBSERVE);

	// Update
	multiplyMatrix(aK, ayold, aKy, STATE, 1, OBSERVE);
	addMatrix(aXcurrent, aKy, aXnew, STATE);

	for (int i=0; i<STATE; i++){
		aXold[i] = aXnew[i];
	}

}

void KalmanK(){

	static float count = 0;

	if (count != 100){
		static float oldk[STATE*OBSERVE] = {0};
		multiplyMatrix(aF, aPold, aFP, STATE, STATE, STATE);
		multiplyMatrix(aFP, aFt, aFPFt, STATE, STATE, STATE);
		addMatrix(aFPFt, aQ, aPcurrent, STATE*STATE);

		multiplyMatrix(aPcurrent, aHt, aPHt, STATE, OBSERVE, STATE);
		multiplyMatrix(aH, aPHt, aHPHt, OBSERVE, OBSERVE, STATE);
		addMatrix(aR, aHPHt, aS, OBSERVE*OBSERVE);

		// Compute Kalman Gain
		inverse(aS, aSi);
		multiplyMatrix(aPHt, aSi, aK, STATE, OBSERVE, OBSERVE);

		transMatrix(aK, aKt, STATE, OBSERVE);
		multiplyMatrix(aK, aR, aKR, STATE, OBSERVE, OBSERVE);
		multiplyMatrix(aKR, aKt, aKRKt, STATE, STATE, OBSERVE);
		multiplyMatrix(aK, aH, aKH, STATE, STATE, OBSERVE);
		subMatrix(aI, aKH, aI_KH, STATE*STATE);
		transMatrix(aI_KH, aI_KHt, STATE, STATE);
		multiplyMatrix(aI_KH, aPcurrent, aI_KHP, STATE, STATE, STATE);
		multiplyMatrix(aI_KHP, aI_KHt, aI_KHPI_KHt, STATE, STATE, STATE);
		addMatrix(aI_KHPI_KHt, aKRKt, aPnew, STATE*STATE);


		float same = 0;
		for (int i=0; i<STATE*OBSERVE; i++){
			if (oldk[i] - aK[i] < 0.0000001 && oldk[i] - aK[i] > -0.0000001){
				same +=1;
			}
			oldk[i] = aK[i];
		}

		if (same == STATE*OBSERVE){
			count += 1;
		}

		for (int i=0; i<STATE*STATE; i++){
			aPold[i] = aPnew[i];
		}
	}
	//for (int i = 0; i < 4; i++) {
	//	uprintf(": %f %f %f %f\n\r", aK[i*4], aK[i*4+1], aK[i*4+2], aK[i*4+3]);
	//}
}

void inverse(float input[OBSERVE*OBSERVE], float output[OBSERVE*OBSERVE]){
	//First 2X2
	float a = input[0];
	float b = input[1];
	float c = input[4];
	float d = input[5];
	float determinant = a*d-b*c;
	output[0] = d/determinant;
	output[1] = -b/determinant;
	output[4] = -c/determinant;
	output[5] = a/determinant;

	//Second 2X2
	a = input[10];
	b = input[11];
	c = input[14];
	d = input[15];
	determinant = a*d-b*c;
	output[10] = d/determinant;
	output[11] = -b/determinant;
	output[14] = -c/determinant;
	output[15] = a/determinant;
}

void multiplyMatrix(float A[], float B[], float C[], int m, int n, int c){ //mXc matrix A, cXn matrix B, mXn matrix C
	for (int i=0; i<m*n; i++){
		C[i] = 0;
	}
	for (int i=0; i<m; i++){
		for (int j=0; j<n; j++){
			for (int k=0; k<c; k++){
				C[i*n+j] += A[i*n+k] * B[k*n+j];
			}
		}
	}
}

void addMatrix(float A[], float B[], float C[], int len){
	for (int i=0; i<len; i++){
		C[i] = A[i] + B[i];
	}
}

void subMatrix(float A[], float B[], float C[], int len){
	for (int i=0; i<len; i++){
		C[i] = A[i] - B[i];
	}
}

void transMatrix(float A[], float B[], int m, int n){ //mXn matrix A, nXm matrix B
	for (int i=0; i<n; i++){
		for (int j=0; j<m; j++){
			B[i*m+j] = A[j*n+i];
		}
	}
}

void getState(float state[STATE]) {
	for (int i=0; i<STATE; i++) {
		state[i] = aXold[i];
	}
}

void getKGain(float gain[STATE][OBSERVE]) {
	for (int j=0; j<OBSERVE; j++) {
		for (int i=0; i<STATE; i++) {
			gain[i][j] = aK[i+j*STATE];
		}
	}

}

void getP(float P[STATE*STATE]) {
	for (int i = 0; i < STATE*STATE; i++) {
		P[i] = aPold[i];
	}
}



