/*
 * diag.h
 *
 * Code generation for function 'diag'
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
int32_T diag(const emlrtStack *sp, const real_T v_data[],
             const int32_T v_size[2], real_T d_data[]);

/* End of code generation (diag.h) */
