/* Generated with MatlabSOS2CMSIS
Phil Birch, University of Sussex, 2017*/
#define NUM_SECTIONS 2
float32_t pCoeffs[]={ 7.06171642E-01, -1.41234328E+00, 7.06171642E-01, 1.99728693E+00, -9.97299394E-01,
 1.00000000E+00, -2.00000000E+00, 1.00000000E+00, 1.99348014E+00, -9.93492586E-01,};

/*Example usage:
#include "test.h"
float32_t pState[NUM_SECTIONS*4]={0};
arm_biquad_casd_df1_inst_f32 S;

In the your main function init the filter
arm_biquad_cascade_df1_init_f32(&S,NUM_SECTIONS,pCoeffs,pState);
To run the filter:
arm_biquad_cascade_df1_f32(&S,pSrc,pDest,BUFFER_SIZE);
See CMSIS doc for varible descriptions
*/