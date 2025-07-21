/*
 * computePolyCoefAndTimeOfArrival.h
 *
 * Code generation for function 'computePolyCoefAndTimeOfArrival'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_computePolyCoefAndTimeOfArriv(const emlrtStack *sp,
                                     const real_T constraints[30],
                                     real_T pp[30],
                                     emxArray_real_T *timeOfArrival);

void c_computePolyCoefAndTimeOfArriv(const emlrtStack *sp,
                                     const real_T constraints[8],
                                     emxArray_real_T *timeOfArrival,
                                     real_T pp[8]);

void computePolyCoefAndTimeOfArrival(const emlrtStack *sp,
                                     const real_T constraints[24],
                                     real_T pp[24],
                                     emxArray_real_T *timeOfArrival);

/* End of code generation (computePolyCoefAndTimeOfArrival.h) */
