/*
 * KalmanV.h
 *
 *  Created on: Feb 21, 2019
 *      Author: kjhertenberg
 */

#ifndef KALMAN_KALMANV_H_
#define KALMAN_KALMANV_H_

#include "arm_math.h"

#define STATE 4
#define OBSERVE 2
#define TIMESTEP 0.01

//create arrays
float32_t aXold[STATE] = {0};
float32_t aF[STATE*STATE] = {
		1, TIMESTEP, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, TIMESTEP,
		0, 0, 0, 1};
float32_t aH[OBSERVE*STATE] = {
		1, 0, 0, 0,
		0, 0, 1, 0};
float32_t aR[OBSERVE*OBSERVE] = {
		6.25, 0,
		0, 6.25};
float32_t az[OBSERVE] = {0};
float32_t aI[STATE*STATE] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};
float32_t aPold[STATE*STATE] = {
		0.5,0,0,0,
		0,0.5,0,0,
		0,0,0.5,0,
		0,0,0,0.5};

//empty arrays
float32_t aXcurrent[STATE] = {0.0f};
float32_t aFt[STATE*STATE] = {0.0f};
float32_t aFP[STATE*STATE] = {0.0f};
float32_t aPcurrent[STATE*STATE] = {0.0f};
float32_t ayold[OBSERVE] = {0.0f};
float32_t aHX[OBSERVE] = {0.0f};
float32_t aS[OBSERVE*OBSERVE] = {0.0f};
float32_t aHt[STATE*OBSERVE] = {0.0f};
float32_t aPHt[STATE*OBSERVE] = {0.0f};
float32_t aHPHt[OBSERVE*OBSERVE] = {0.0f};
float32_t aK[STATE*OBSERVE] = {0.0f};
float32_t aSi[OBSERVE*OBSERVE] = {0.0f};
float32_t aXnew[STATE] = {0.0f};
float32_t aKy[STATE] = {0.0f};
float32_t aPnew[STATE*STATE] = {0.0f};
float32_t aKt[OBSERVE*STATE] = {0.0f};
float32_t aKR[STATE*OBSERVE] = {0.0f};
float32_t aKRKt[STATE*STATE] = {0.0f};
float32_t aKH[STATE*STATE] = {0.0f};
float32_t aI_KH[STATE*STATE] = {0.0f};
float32_t aI_KHt[STATE*STATE] = {0.0f};
float32_t aI_KHP[STATE*STATE] = {0.0f};
float32_t aI_KHPI_KHt[STATE*STATE] = {0.0f};
//float32_t aHXnew[STATE] = {0.0f};
//float32_t aynew[STATE] = {0.0f};

//create matrix objects
arm_matrix_instance_f32 Xold;
arm_matrix_instance_f32 F;
arm_matrix_instance_f32 Xcurrent;
arm_matrix_instance_f32 Pold;
arm_matrix_instance_f32 Ft;
arm_matrix_instance_f32 FP;
arm_matrix_instance_f32 Pcurrent;
arm_matrix_instance_f32 yold;
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

#endif /* KALMAN_KALMANV_H_ */
