/*
 * normalizeRows.c
 *
 * Code generation for function 'normalizeRows'
 *
 */

/* Include files */
#include "normalizeRows.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void normalizeRows(const emlrtStack *sp, const real_T matrix[3],
                   real_T normRowMatrix[3])
{
  __m128d r;
  emlrtStack st;
  real_T y[3];
  real_T x;
  st.prev = sp;
  st.tls = sp->tls;
  r = _mm_loadu_pd(&matrix[0]);
  _mm_storeu_pd(&y[0], _mm_mul_pd(r, r));
  y[2] = matrix[2] * matrix[2];
  x = c_sumColumnB(y);
  st.site = &jl_emlrtRSI;
  if (x < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  x = muDoubleScalarSqrt(x);
  x = 1.0 / x;
  _mm_storeu_pd(&normRowMatrix[0],
                _mm_mul_pd(_mm_loadu_pd(&matrix[0]), _mm_set1_pd(x)));
  normRowMatrix[2] = matrix[2] * x;
}

/* End of code generation (normalizeRows.c) */
