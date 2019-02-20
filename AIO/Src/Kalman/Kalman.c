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
#include "arm_math.h"

#define SIZE 2


float32_t aXold[SIZE];
float32_t aF[SIZE*SIZE];
float32_t aXcurrent[SIZE];
float32_t aPold[SIZE*SIZE];
float32_t aFt[SIZE*SIZE];
float32_t aFP[SIZE*SIZE];
float32_t aPcurrent[SIZE*SIZE];
float32_t ayold[SIZE];
float32_t az[SIZE];
float32_t aH[SIZE*SIZE];
float32_t aHX[SIZE];
float32_t aS[SIZE*SIZE];
float32_t aR[SIZE*SIZE];
float32_t aHt[SIZE*SIZE];
float32_t aPHt[SIZE*SIZE];
float32_t aHPHt[SIZE*SIZE];
float32_t aK[SIZE*SIZE];
float32_t aSi[SIZE*SIZE];
float32_t aPHtSi[SIZE*SIZE];
float32_t aXnew[SIZE];
float32_t aKy[SIZE];
float32_t aPnew[SIZE*SIZE];
float32_t aKt[SIZE*SIZE];
float32_t aKR[SIZE*SIZE];
float32_t aKRKt[SIZE*SIZE];
float32_t aI[SIZE*SIZE];
float32_t aKH[SIZE*SIZE];
float32_t aI_KH[SIZE*SIZE];
float32_t aI_KHt[SIZE*SIZE];
float32_t aI_KHP[SIZE*SIZE];
float32_t aI_KHPI_KHt[SIZE*SIZE];
float32_t aHXnew[SIZE];
float32_t aynew[SIZE];

arm_matrix_instance_f32 Xold;
arm_matrix_instance_f32 F;
arm_matrix_instance_f32 Xcurrent;
arm_matrix_instance_f32 Pold;
arm_matrix_instance_f32 Ft;
arm_matrix_instance_f32 FP;
arm_matrix_instance_f32 Pcurrent;
arm_matrix_instance_f32 yold;
arm_matrix_instance_f32 z;
arm_matrix_instance_f32 H;
arm_matrix_instance_f32 HX;
arm_matrix_instance_f32 S;
arm_matrix_instance_f32 R;
arm_matrix_instance_f32 Ht;
arm_matrix_instance_f32 PHt;
arm_matrix_instance_f32 HPHt;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 Si;
arm_matrix_instance_f32 PHtSi;
arm_matrix_instance_f32 Xnew;
arm_matrix_instance_f32 Ky;
arm_matrix_instance_f32 Pnew;
arm_matrix_instance_f32 Kt;
arm_matrix_instance_f32 KR;
arm_matrix_instance_f32 KRKt;
arm_matrix_instance_f32 I;
arm_matrix_instance_f32 KH;
arm_matrix_instance_f32 I_KH;
arm_matrix_instance_f32 I_KHt;
arm_matrix_instance_f32 I_KHP;
arm_matrix_instance_f32 I_KHPI_KHt;
arm_matrix_instance_f32 HXnew;
arm_matrix_instance_f32 ynew;

void Kalman_init(){
	arm_mat_init_f32(&Xold, SIZE, 1, aXold);
	arm_mat_init_f32(&F, SIZE, SIZE, aF);
	arm_mat_init_f32(&Xcurrent, SIZE, 1, aXcurrent);
	arm_mat_init_f32(&Pold, SIZE, SIZE, aPold);
	arm_mat_init_f32(&Ft, SIZE, SIZE, aFt);
	arm_mat_init_f32(&FP, SIZE, SIZE, aFP);
	arm_mat_init_f32(&Pcurrent, SIZE, SIZE, aPcurrent);
	arm_mat_init_f32(&yold, SIZE, 1, ayold);
	arm_mat_init_f32(&z, SIZE, 1, az);
	arm_mat_init_f32(&H, SIZE, SIZE, aH);
	arm_mat_init_f32(&HX, SIZE, 1, aHX);
	arm_mat_init_f32(&S, SIZE, SIZE, aS);
	arm_mat_init_f32(&R, SIZE, SIZE, aR);
	arm_mat_init_f329(&Ht, SIZE, SIZE, aHt);
	arm_mat_init_f32(&PHt, SIZE, SIZE, aPHt);
	arm_mat_init_f32(&HPHt, SIZE, SIZE, aHPHt);
	arm_mat_init_f32(&K, SIZE, SIZE, aK);
	arm_mat_init_f32(&Si, SIZE, SIZE, aSi);
	arm_mat_init_f32(&PHtSi, SIZE, SIZE, aPHtSi);
	arm_mat_init_f32(&Xnew, SIZE, 1, aXnew);
	arm_mat_init_f32(&Ky, SIZE, 1, aKy);
	arm_mat_init_f32(&Pnew, SIZE, SIZE, aPnew);
	arm_mat_init_f32(&Kt, SIZE, SIZE, aKt);
	arm_mat_init_f32(&KR, SIZE, SIZE, aKR);
	arm_mat_init_f32(&KRKt, SIZE, SIZE, aKRKt);
	arm_mat_init_f32(&I, SIZE, SIZE, aI);
	arm_mat_init_f32(&KH, SIZE, SIZE, aKH);
	arm_mat_init_f32(&I_KH, SIZE, SIZE, aI_KH);
	arm_mat_init_f32(&I_KHt, SIZE, SIZE, aI_KHt);
	arm_mat_init_f32(&I_KHP, SIZE, SIZE, aI_KHP);
	arm_mat_init_f32(&I_KHPI_KHt, SIZE, SIZE, aI_KHPI_KHt);
	arm_mat_init_f32(&HXnew, SIZE, 1, aHXnew);
	arm_mat_init_f32(&ynew, SIZE, SIZE, aynew);
}


void Kalman(){
	arm_mat_mult_f32(&F, &Xold, &Xcurrent);

	arm_mat_trans_f32(&F, &Ft);
	arm_mat_mult_f32(&F, &Pold, &FP);
	arm_mat_mult_f32(&FP, &Ft, &Pcurrent);

	arm_mat_mult_f32(&H, &Xcurrent, &HX);
	for (int i=0; i<SIZE; i++){
		ayold[i] = az[i] - aHX[i];
	}

	arm_mat_trans_f32(&H, &Ht);
	arm_mat_mult_f32(&Pcurrent, &Ht, &PHt);
	arm_mat_mult_f32(&H, &PHt, &HPHt);
	for (int i=0; i<SIZE*SIZE; i++){
		aS[i] = aR[i] - aHPHt[i];
	}

	arm_mat_inverse_f32(&S, &Si);
	arm_mat_mult_f32(&PHt, &Si, &K);

	arm_mat_mult_f32(&K, &yold, &Ky);
	for (int i=0; i<SIZE; i++){
		aXnew[i] = aXcurrent[i] + aKy[i];
	}

	arm_mat_trans_f32(&K, &Kt);
	arm_mat_mult_f32(&K, &R, &KR);
	arm_mat_mult_f32(&KR, &Kt, &KRKt);
	arm_mat_mult_f32(&K, &H, &KH);
	for (int i=0; i<SIZE*SIZE; i++){
		aI_KH[i] = aI[i] - aKH[i];
	}
	arm_mat_trans_f32(&I_KH, &I_KHt);
	arm_mat_mult_f32(&I_KH, &Pcurrent, &I_KHP);
	arm_mat_mult_f32(&I_KHP, &I_KHt, &I_KHPI_KHt);
	for (int i=0; i<SIZE*SIZE; i++){
		aPnew[i] = aI_KHPI_KHt[i] + aKRKt[i];
	}

	arm_mat_mult_f32(&H, &Xnew, &HXnew);
	for (int i=0; i<SIZE; i++){
		aynew[i] = az[i] + aHXnew[i];
	}

}



