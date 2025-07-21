/*
 * find.h
 *
 * Code generation for function 'find'
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
int32_T b_eml_find(const emlrtStack *sp, const boolean_T x[10],
                   int32_T i_data[]);

int32_T eml_find(const emlrtStack *sp, const boolean_T x[8], int32_T i_data[]);

/* End of code generation (find.h) */
