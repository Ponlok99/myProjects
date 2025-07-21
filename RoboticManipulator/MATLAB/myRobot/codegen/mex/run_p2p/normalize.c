/*
 * normalize.c
 *
 * Code generation for function 'normalize'
 *
 */

/* Include files */
#include "normalize.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo hk_emlrtRSI = {
    10,                         /* lineNo */
    "quaternionBase/normalize", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\normalize.m" /* pathName */
};

/* Function Definitions */
void quaternionBase_normalize(const emlrtStack *sp, quaternion *q)
{
  emlrtStack st;
  real_T n;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &hk_emlrtRSI;
  n = ((q->a * q->a + q->b * q->b) + q->c * q->c) + q->d * q->d;
  if (n < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  n = muDoubleScalarSqrt(n);
  q->a /= n;
  q->b /= n;
  q->c /= n;
  q->d /= n;
}

/* End of code generation (normalize.c) */
