/*
 * unique.h
 *
 * Code generation for function 'unique'
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
int32_T unique_rows(const emlrtStack *sp, const real_T a_data[],
                    const int32_T a_size[2], real_T b_data[], int32_T b_size[2],
                    int32_T ndx_data[]);

/* End of code generation (unique.h) */
