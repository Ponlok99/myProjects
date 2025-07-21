/*
 * histcounts.h
 *
 * Code generation for function 'histcounts'
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
void histcounts(const emlrtStack *sp, const real_T x[101],
                const emxArray_real_T *varargin_1, emxArray_real_T *n,
                real_T bin[101]);

/* End of code generation (histcounts.h) */
