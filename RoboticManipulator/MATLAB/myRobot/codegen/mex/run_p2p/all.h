/*
 * all.h
 *
 * Code generation for function 'all'
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
boolean_T all(const boolean_T x[2]);

boolean_T b_all(const boolean_T x[5]);

boolean_T c_all(const boolean_T x[3]);

void d_all(const emlrtStack *sp, const boolean_T x[48], boolean_T y[16]);

int32_T e_all(const emlrtStack *sp, const boolean_T x_data[],
              const int32_T x_size[2], boolean_T y_data[]);

/* End of code generation (all.h) */
