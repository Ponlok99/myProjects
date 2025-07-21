/*
 * wrapToPi.h
 *
 * Code generation for function 'wrapToPi'
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
void b_wrapToPi(real_T theta[3]);

void c_wrapToPi(const emlrtStack *sp, real_T theta_data[],
                const int32_T theta_size[2]);

void wrapToPi(real_T *theta);

/* End of code generation (wrapToPi.h) */
