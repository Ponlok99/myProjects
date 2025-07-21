/*
 * run_p2p_terminate.c
 *
 * Code generation for function 'run_p2p_terminate'
 *
 */

/* Include files */
#include "run_p2p_terminate.h"
#include "_coder_run_p2p_mex.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void run_p2p_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  /* Free instance data */
  covrtFreeInstanceData(&emlrtCoverageInstance);
  /* Free instance data */
  covrtFreeInstanceData(&emlrtCoverageInstance);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void run_p2p_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (run_p2p_terminate.c) */
