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
	//Link arrays and matrix objects
	arm_mat_init_f32(&Xold, STATE, 1, (float32_t *)aXold);
	arm_mat_init_f32(&F, STATE, STATE, (float32_t *)aF);
	arm_mat_init_f32(&Xcurrent, STATE, 1, (float32_t *)aXcurrent);
	arm_mat_init_f32(&Pold, STATE, STATE, (float32_t *)aPold);
	arm_mat_init_f32(&Ft, STATE, STATE, (float32_t *)aFt);
	arm_mat_init_f32(&FP, STATE, STATE, (float32_t *)aFP);
	arm_mat_init_f32(&Pcurrent, STATE, STATE, (float32_t *)aPcurrent);
	arm_mat_init_f32(&yold, OBSERVE, 1, (float32_t *)ayold);
	arm_mat_init_f32(&H, OBSERVE, STATE, (float32_t *)aH);
	arm_mat_init_f32(&HX, OBSERVE, 1, (float32_t *)aHX);
	arm_mat_init_f32(&S, OBSERVE, OBSERVE, (float32_t *)aS);
	arm_mat_init_f32(&R, OBSERVE, OBSERVE, (float32_t *)aR);
	arm_mat_init_f32(&Ht, STATE, OBSERVE, (float32_t *)aHt);
	arm_mat_init_f32(&PHt, STATE, OBSERVE, (float32_t *)aPHt);
	arm_mat_init_f32(&HPHt, OBSERVE, OBSERVE, (float32_t *)aHPHt);
	arm_mat_init_f32(&K, STATE, OBSERVE, (float32_t *)aK);
	arm_mat_init_f32(&Si, OBSERVE, OBSERVE, (float32_t *)aSi);
	arm_mat_init_f32(&Xnew, STATE, 1, (float32_t *)aXnew);
	arm_mat_init_f32(&Ky, STATE, 1, (float32_t *)aKy);
	arm_mat_init_f32(&Pnew, STATE, STATE, (float32_t *)aPnew);
	arm_mat_init_f32(&Kt, OBSERVE, STATE, (float32_t *)aKt);
	arm_mat_init_f32(&KR, STATE, OBSERVE, (float32_t *)aKR);
	arm_mat_init_f32(&KRKt, STATE, STATE, (float32_t *)aKRKt);
	arm_mat_init_f32(&KH, STATE, STATE, (float32_t *)aKH);
	arm_mat_init_f32(&I_KH, STATE, STATE, (float32_t *)aI_KH);
	arm_mat_init_f32(&I_KHt, STATE, STATE, (float32_t *)aI_KHt);
	arm_mat_init_f32(&I_KHP, STATE, STATE, (float32_t *)aI_KHP);
	arm_mat_init_f32(&I_KHPI_KHt, STATE, STATE, (float32_t *)aI_KHPI_KHt);
	arm_mat_init_f32(&B, STATE, STATE, (float32_t *)aB);
	arm_mat_init_f32(&U, STATE, 1, (float32_t *)aU);
	arm_mat_init_f32(&BU, STATE, 1, (float32_t *)aBU);
	arm_mat_init_f32(&z, OBSERVE, 1, (float32_t *)az);
	arm_mat_init_f32(&Q, STATE, STATE, (float32_t *)aQ);
	//arm_mat_init_f32(&HXnew, OBSERVE, 1, (float32_t *)aHXnew);
	//arm_mat_init_f32(&ynew, OBSERVE, 1, (float32_t *)aynew);
}


void Kalman(float acc[2], float vel[2], float controlInput[STATE]){

	// Predict
//	for (int i = 0; i < STATE; i++) {
//		aU[i] = controlInput[i];
//	}

	arm_mat_mult_f32(&F, &Xold, &Xcurrent);
	arm_mat_mult_f32(&B, &U, &BU);
	arm_mat_add_f32(&Xcurrent, &BU, &Xcurrent);

	arm_mat_trans_f32(&F, &Ft);
	arm_mat_mult_f32(&F, &Pold, &FP);
	arm_mat_mult_f32(&FP, &Ft, &Pcurrent);
	arm_mat_add_f32(&Pcurrent, &Q, &Pcurrent);


	// Get measurement
	az[0] = 0;
	az[1] = acc[0];
	az[2] = 0;
	az[3] = acc[1];


	// Process data
	arm_mat_mult_f32(&H, &Xcurrent, &HX);
	arm_mat_sub_f32(&z, &HX, &yold);

	arm_mat_trans_f32(&H, &Ht);
	arm_mat_mult_f32(&Pcurrent, &Ht, &PHt);
	arm_mat_mult_f32(&H, &PHt, &HPHt);
	arm_mat_add_f32(&R, &HPHt, &S);


	// Compute Kalman Gain
	inverse(aS, aSi);
	arm_mat_mult_f32(&PHt, &Si, &K);

	// Update
	arm_mat_mult_f32(&K, &yold, &Ky);
	arm_mat_add_f32(&Xcurrent, &Ky, &Xnew);

	arm_mat_trans_f32(&K, &Kt);
	arm_mat_mult_f32(&K, &R, &KR);
	arm_mat_mult_f32(&KR, &Kt, &KRKt);
	arm_mat_mult_f32(&K, &H, &KH);
	arm_mat_sub_f32(&I, &KH, &I_KH);
	arm_mat_trans_f32(&I_KH, &I_KHt);
	arm_mat_mult_f32(&I_KH, &Pcurrent, &I_KHP);
	arm_mat_mult_f32(&I_KHP, &I_KHt, &I_KHPI_KHt);
	arm_mat_add_f32(&I_KHPI_KHt, &KRKt, &Pnew);

	//uprintf("M = | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | \n\r", aXold[0], aXold[1], aK[0], aK[1], ayold[0], az[0]);

	/*
	arm_mat_mult_f32(&H, &Xnew, &HXnew);
	for (int i=0; i<STATE; i++){
		aynew[i] = az[i] + aHXnew[i];
	}
	*/
	for (int i=0; i<STATE; i++){
		aXold[i] = aXnew[i];
	}
	for (int i=0; i<STATE*STATE; i++){
		aPold[i] = aPnew[i];
	}
}

void inverse(float32_t input[OBSERVE*OBSERVE], float32_t output[OBSERVE*OBSERVE]){
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



