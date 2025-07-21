/*
 * _coder_run_p2p_mex.c
 *
 * Code generation for function '_coder_run_p2p_mex'
 *
 */

/* Include files */
#include "_coder_run_p2p_mex.h"
#include "_coder_run_p2p_api.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_initialize.h"
#include "run_p2p_terminate.h"
#include "run_p2p_types.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  run_p2pStackData *run_p2pStackDataGlobal = NULL;
  run_p2pStackDataGlobal = (run_p2pStackData *)emlrtMxCalloc(
      (size_t)1, (size_t)1U * sizeof(run_p2pStackData));
  mexAtExit(&run_p2p_atexit);
  run_p2p_initialize();
  run_p2p_mexFunction(run_p2pStackDataGlobal, nlhs, plhs, nrhs, prhs);
  run_p2p_terminate();
  emlrtMxFree(run_p2pStackDataGlobal);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void run_p2p_mexFunction(run_p2pStackData *SD, int32_T nlhs, mxArray *plhs[4],
                         int32_T nrhs, const mxArray *prhs[7])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[4];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        7, "run_p2p");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 7,
                        "run_p2p");
  }
  /* Call the function. */
  run_p2p_api(SD, prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_run_p2p_mex.c) */
