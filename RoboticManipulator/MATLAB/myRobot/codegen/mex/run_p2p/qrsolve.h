/*
 * qrsolve.h
 *
 * Code generation for function 'qrsolve'
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
int32_T rankFromQR(const emlrtStack *sp, const emxArray_real_T *A);

/* End of code generation (qrsolve.h) */
