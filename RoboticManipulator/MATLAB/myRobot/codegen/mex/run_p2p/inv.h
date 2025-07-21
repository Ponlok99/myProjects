/*
 * inv.h
 *
 * Code generation for function 'inv'
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
void b_checkcond(const emlrtStack *sp, const real_T x[25],
                 const real_T xinv[25]);

void checkcond(const emlrtStack *sp, const real_T x[16], const real_T xinv[16]);

/* End of code generation (inv.h) */
