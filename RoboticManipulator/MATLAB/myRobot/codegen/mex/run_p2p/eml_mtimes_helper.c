/*
 * eml_mtimes_helper.c
 *
 * Code generation for function 'eml_mtimes_helper'
 *
 */

/* Include files */
#include "eml_mtimes_helper.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Definitions */
void dynamic_size_checks(const emlrtStack *sp, const int32_T b_size[2],
                         int32_T innerDimA, int32_T innerDimB)
{
  if (innerDimA != innerDimB) {
    if (b_size[1] == 1) {
      emlrtErrorWithMessageIdR2018a(
          sp, &s_emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &q_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
}

/* End of code generation (eml_mtimes_helper.c) */
