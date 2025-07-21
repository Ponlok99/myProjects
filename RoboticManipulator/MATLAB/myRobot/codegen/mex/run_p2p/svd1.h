/*
 * svd1.h
 *
 * Code generation for function 'svd1'
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
int32_T b_svd(const emlrtStack *sp, const real_T A_data[],
              const int32_T A_size[2], real_T U[36], real_T s_data[],
              real_T V_data[], int32_T V_size[2]);

/* End of code generation (svd1.h) */
