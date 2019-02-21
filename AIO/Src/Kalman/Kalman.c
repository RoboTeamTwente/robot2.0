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
	//arm_mat_init_f32(&HXnew, OBSERVE, 1, (float32_t *)aHXnew);
	//arm_mat_init_f32(&ynew, OBSERVE, 1, (float32_t *)aynew);
}


void Kalman(){


	arm_mat_mult_f32(&F, &Xold, &Xcurrent);

	arm_mat_trans_f32(&F, &Ft);
	arm_mat_mult_f32(&F, &Pold, &FP);
	arm_mat_mult_f32(&FP, &Ft, &Pcurrent);

	arm_mat_mult_f32(&H, &Xcurrent, &HX);
	for (int i=0; i<OBSERVE; i++){
		ayold[i] = az[i] - aHX[i];
	}


	arm_mat_trans_f32(&H, &Ht);
	arm_mat_mult_f32(&Pcurrent, &Ht, &PHt);
	arm_mat_mult_f32(&H, &PHt, &HPHt);
	for (int i=0; i<OBSERVE*OBSERVE; i++){
		aS[i] = aR[i] - aHPHt[i];
	}

	arm_mat_inverse_f32(&S, &Si);
	arm_mat_mult_f32(&PHt, &Si, &K);

	arm_mat_mult_f32(&K, &yold, &Ky);
	for (int i=0; i<STATE; i++){
		aXnew[i] = aXcurrent[i] + aKy[i];
	}

	arm_mat_trans_f32(&K, &Kt);
	arm_mat_mult_f32(&K, &R, &KR);
	arm_mat_mult_f32(&KR, &Kt, &KRKt);
	arm_mat_mult_f32(&K, &H, &KH);
	for (int i=0; i<STATE*STATE; i++){
		aI_KH[i] = aI[i] - aKH[i];
	}
	arm_mat_trans_f32(&I_KH, &I_KHt);
	arm_mat_mult_f32(&I_KH, &Pcurrent, &I_KHP);
	arm_mat_mult_f32(&I_KHP, &I_KHt, &I_KHPI_KHt);
	for (int i=0; i<STATE*STATE; i++){
		aPnew[i] = aI_KHPI_KHt[i] + aKRKt[i];
	}

	uprintf("M = | %.3f | %.3f | %.3f | %.3f | %.3f | \n\r", aXold[0], aXold[1], aK[0], aK[1], ayold[0]);

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





