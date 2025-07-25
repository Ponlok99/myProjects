/*
 * qrsolve.c
 *
 * Code generation for function 'qrsolve'
 *
 */

/* Include files */
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_mexutil.h"
#include "run_p2p_types.h"
#include "warning.h"
#include "mwmathutil.h"

/* Function Definitions */
int32_T rankFromQR(const emlrtStack *sp, const emxArray_real_T *A)
{
  static const int32_T b_iv[2] = {1, 6};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  emlrtStack b_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  const real_T *A_data;
  real_T tol;
  int32_T maxmn;
  int32_T minmn;
  int32_T r;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  A_data = A->data;
  r = 0;
  tol = 0.0;
  if (A->size[0] < A->size[1]) {
    minmn = A->size[0];
    maxmn = A->size[1];
  } else {
    minmn = A->size[1];
    maxmn = A->size[0];
  }
  if (minmn > 0) {
    tol = muDoubleScalarMin(1.4901161193847656E-8,
                            2.2204460492503131E-15 * (real_T)maxmn) *
          muDoubleScalarAbs(A_data[0]);
    while ((r < minmn) &&
           (!(muDoubleScalarAbs(A_data[r + A->size[0] * r]) <= tol))) {
      r++;
    }
  }
  if (r < minmn) {
    char_T str[14];
    st.site = &uf_emlrtRSI;
    y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(&st, 6, m, &rfmt[0]);
    emlrtAssign(&y, m);
    b_y = NULL;
    m = emlrtCreateDoubleScalar(tol);
    emlrtAssign(&b_y, m);
    b_st.site = &rv_emlrtRSI;
    emlrt_marshallIn(&b_st, b_sprintf(&b_st, y, b_y, &b_emlrtMCI),
                     "<output of sprintf>", str);
    st.site = &vf_emlrtRSI;
    c_warning(&st, r, str);
  }
  return r;
}

/* End of code generation (qrsolve.c) */
