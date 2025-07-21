/*
 * mldivide.h
 *
 * Code generation for function 'mldivide'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
int32_T b_mldivide(const emlrtStack *sp, const real_T A_data[],
                   const int32_T A_size[2], const real_T B[6], real_T Y_data[]);

int32_T mldivide(const emlrtStack *sp, const real_T A_data[],
                 const int32_T A_size[2], const real_T B_data[], int32_T B_size,
                 real_T Y_data[]);

/* End of code generation (mldivide.h) */
