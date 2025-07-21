/*
 * constructM.h
 *
 * Code generation for function 'constructM'
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
void b_constructM(const emlrtStack *sp, const real_T constraints[10],
                  real_T M[100]);

void constructM(const emlrtStack *sp, const real_T constraints[8],
                real_T M[64]);

/* End of code generation (constructM.h) */
