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
#define OBSERVE 4
#define TIMESTEP 0.01

// certainties
#define VEL_VAR 0.5F // variance in the velocity measurements
#define ACC_VAR 6.25F // variance in the acceleration measurements
#define STATE_VAR 1.0F // variance in the predicted state
#define RAND_VAR 1.0F // variance in the random force

//create arrays
float aXold[STATE] = {0.0f};
float aF[STATE*STATE] = {
		1, TIMESTEP, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, TIMESTEP,
		0, 0, 0, 1};
float aH[OBSERVE*STATE] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1};
float aR[OBSERVE*OBSERVE] = {
		VEL_VAR, 0, 0, 0,
		0, ACC_VAR, 0, 0,
		0, 0, VEL_VAR, 0,
		0, 0, 0, ACC_VAR};
float az[OBSERVE] = {0.0f};
float aI[STATE*STATE] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};
float aPold[STATE*STATE] = {
		STATE_VAR,0,0,0,
		0,STATE_VAR,0,0,
		0,0,STATE_VAR,0,
		0,0,0,STATE_VAR};
float aB[STATE*STATE] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1};
float aQ[STATE*STATE] = {
		TIMESTEP*TIMESTEP*RAND_VAR, TIMESTEP*RAND_VAR, 0, 0,
		TIMESTEP*RAND_VAR, RAND_VAR, 0, 0,
		0, 0, TIMESTEP*TIMESTEP*RAND_VAR, TIMESTEP*RAND_VAR,
		0, 0, TIMESTEP*RAND_VAR, RAND_VAR};

//empty arrays
float aXcurrent[STATE] = {0.0f};
float aFX[STATE] = {0.0f};
float aFt[STATE*STATE] = {0.0f};
float aFP[STATE*STATE] = {0.0f};
float aPcurrent[STATE*STATE] = {0.0f};
float ayold[OBSERVE] = {0.0f};
float aHX[OBSERVE] = {0.0f};
float aS[OBSERVE*OBSERVE] = {0.0f};
float aHt[STATE*OBSERVE] = {0.0f};
float aPHt[STATE*OBSERVE] = {0.0f};
float aFPFt[STATE*STATE] = {0.0f};
float aHPHt[OBSERVE*OBSERVE] = {0.0f};
float aK[STATE*OBSERVE] = {0.0f};
float aSi[OBSERVE*OBSERVE] = {0.0f};
float aXnew[STATE] = {0.0f};
float aKy[STATE] = {0.0f};
float aPnew[STATE*STATE] = {0.0f};
float aKt[OBSERVE*STATE] = {0.0f};
float aKR[STATE*OBSERVE] = {0.0f};
float aKRKt[STATE*STATE] = {0.0f};
float aKH[STATE*STATE] = {0.0f};
float aI_KH[STATE*STATE] = {0.0f};
float aI_KHt[STATE*STATE] = {0.0f};
float aI_KHP[STATE*STATE] = {0.0f};
float aI_KHPI_KHt[STATE*STATE] = {0.0f};
float aU[STATE] = {0.0f};
float aBU[STATE] = {0.0f};
//float aHXnew[STATE] = {0.0f};
//float aynew[STATE] = {0.0f};

//create matrix objects
/*
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
arm_matrix_instance_f32 B;
arm_matrix_instance_f32 U;
arm_matrix_instance_f32 BU;
arm_matrix_instance_f32 z;
arm_matrix_instance_f32 Q;
//arm_matrix_instance_f32 HXnew;
//arm_matrix_instance_f32 ynew;
*/
#endif /* KALMAN_KALMANV_H_ */
