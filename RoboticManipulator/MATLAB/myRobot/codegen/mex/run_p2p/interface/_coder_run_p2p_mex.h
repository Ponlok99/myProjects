/*
 * _coder_run_p2p_mex.h
 *
 * Code generation for function '_coder_run_p2p_mex'
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
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void run_p2p_mexFunction(run_p2pStackData *SD, int32_T nlhs, mxArray *plhs[4],
                         int32_T nrhs, const mxArray *prhs[7]);

/* End of code generation (_coder_run_p2p_mex.h) */
