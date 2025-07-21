/*
 * interpSnapTraj.h
 *
 * Code generation for function 'interpSnapTraj'
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
void interpSnapTraj(const emlrtStack *sp, const real_T pp[30],
                    const emxArray_real_T *timePoints, real_T q[303],
                    real_T qd[303], real_T qdd[303], real_T qddd[303],
                    real_T qdddd[303], real_T tSamples[101]);

/* End of code generation (interpSnapTraj.h) */
