/*
 * sort.h
 *
 * Code generation for function 'sort'
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
void b_sort(const emlrtStack *sp, real_T x_data[], const int32_T *x_size);

int32_T c_sort(const emlrtStack *sp, real_T x_data[], const int32_T *x_size,
               int32_T idx_data[]);

void sort(real_T x[2]);

/* End of code generation (sort.h) */
