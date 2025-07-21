/*
 * rottraj.h
 *
 * Code generation for function 'rottraj'
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
void rottraj(const emlrtStack *sp, const real_T R0[4], const real_T RF[4],
             const real_T varargin_2[303], real_T R[404], real_T omega[303],
             real_T alpha[303]);

/* End of code generation (rottraj.h) */
