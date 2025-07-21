/*
 * sqrt.c
 *
 * Code generation for function 'sqrt'
 *
 */

/* Include files */
#include "sqrt.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
void b_sqrt(const emlrtStack *sp, real_T *x)
{
  if (*x < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        sp, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  *x = muDoubleScalarSqrt(*x);
}

/* End of code generation (sqrt.c) */
