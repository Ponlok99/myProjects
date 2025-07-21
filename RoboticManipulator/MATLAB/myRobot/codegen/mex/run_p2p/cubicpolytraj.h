/*
 * cubicpolytraj.h
 *
 * Code generation for function 'cubicpolytraj'
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
void b_cubicpolytraj(const emlrtStack *sp, const real_T wayPoints[6],
                     const real_T varargin_2[6], real_T q[303], real_T qd[303],
                     real_T qdd[303]);

/* End of code generation (cubicpolytraj.h) */
