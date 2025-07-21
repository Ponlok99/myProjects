/*
 * polyder.h
 *
 * Code generation for function 'polyder'
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
void b_polyder(const emlrtStack *sp, const real_T u[7], real_T a_data[],
               int32_T a_size[2]);

void c_polyder(const emlrtStack *sp, const real_T u[6], real_T a_data[],
               int32_T a_size[2]);

void polyder(const emlrtStack *sp, const real_T u[8], real_T a_data[],
             int32_T a_size[2]);

/* End of code generation (polyder.h) */
