/*
 * diag.c
 *
 * Code generation for function 'diag'
 *
 */

/* Include files */
#include "diag.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRTEInfo ab_emlrtRTEI = {
    102,    /* lineNo */
    19,     /* colNo */
    "diag", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\diag.m" /* pName
                                                                       */
};

/* Function Definitions */
int32_T diag(const emlrtStack *sp, const real_T v_data[],
             const int32_T v_size[2], real_T d_data[])
{
  int32_T d_size;
  int32_T k;
  if (v_size[1] == 1) {
    emlrtErrorWithMessageIdR2018a(sp, &ab_emlrtRTEI,
                                  "Coder:toolbox:diag_varsizedMatrixVector",
                                  "Coder:toolbox:diag_varsizedMatrixVector", 0);
  }
  if (v_size[1] > 0) {
    d_size = muIntScalarMin_sint32(6, v_size[1]);
  } else {
    d_size = 0;
  }
  for (k = 0; k < d_size; k++) {
    d_data[k] = v_data[k + 6 * k];
  }
  return d_size;
}

/* End of code generation (diag.c) */
