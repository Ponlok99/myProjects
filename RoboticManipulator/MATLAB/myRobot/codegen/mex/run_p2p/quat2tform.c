/*
 * quat2tform.c
 *
 * Code generation for function 'quat2tform'
 *
 */

/* Include files */
#include "quat2tform.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo el_emlrtRSI = {
    23,           /* lineNo */
    "quat2tform", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\quat2tform.m" /* pathName */
};

static emlrtRSInfo fl_emlrtRSI = {
    22,          /* lineNo */
    "quat2rotm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\quat2rotm.m" /* pathName */
};

static emlrtRSInfo gl_emlrtRSI = {
    33,          /* lineNo */
    "quat2rotm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\quat2rotm.m" /* pathName */
};

static emlrtRSInfo hl_emlrtRSI = {
    34,          /* lineNo */
    "quat2rotm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\quat2rotm.m" /* pathName */
};

static emlrtRSInfo il_emlrtRSI = {
    35,          /* lineNo */
    "quat2rotm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\quat2rotm.m" /* pathName */
};

/* Function Definitions */
void quat2tform(const emlrtStack *sp, const real_T q[4], real_T H[16])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T R[9];
  real_T tempR[9];
  real_T normRowMatrix[4];
  real_T b_tempR_tmp;
  real_T c_tempR_tmp;
  real_T d_tempR_tmp;
  real_T e_tempR_tmp;
  real_T f_tempR_tmp;
  real_T g_tempR_tmp;
  real_T h_tempR_tmp;
  real_T i_tempR_tmp;
  real_T j_tempR_tmp;
  real_T k_tempR_tmp;
  real_T tempR_tmp;
  real_T x;
  int32_T i;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &el_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &fl_emlrtRSI;
  r = _mm_loadu_pd(&q[0]);
  _mm_storeu_pd(&normRowMatrix[0], _mm_mul_pd(r, r));
  r = _mm_loadu_pd(&q[2]);
  _mm_storeu_pd(&normRowMatrix[2], _mm_mul_pd(r, r));
  x = sumColumnB(normRowMatrix);
  c_st.site = &jl_emlrtRSI;
  if (x < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  x = muDoubleScalarSqrt(x);
  r = _mm_set1_pd(1.0 / x);
  _mm_storeu_pd(&normRowMatrix[0], _mm_mul_pd(_mm_loadu_pd(&q[0]), r));
  _mm_storeu_pd(&normRowMatrix[2], _mm_mul_pd(_mm_loadu_pd(&q[2]), r));
  b_st.site = &gl_emlrtRSI;
  b_st.site = &gl_emlrtRSI;
  b_st.site = &hl_emlrtRSI;
  b_st.site = &hl_emlrtRSI;
  b_st.site = &il_emlrtRSI;
  b_st.site = &il_emlrtRSI;
  x = normRowMatrix[3] * normRowMatrix[3];
  tempR_tmp = normRowMatrix[2] * normRowMatrix[2];
  b_tempR_tmp = 1.0 - 2.0 * (tempR_tmp + x);
  tempR[0] = b_tempR_tmp;
  c_tempR_tmp = normRowMatrix[1] * normRowMatrix[2];
  d_tempR_tmp = normRowMatrix[0] * normRowMatrix[3];
  e_tempR_tmp = 2.0 * (c_tempR_tmp - d_tempR_tmp);
  tempR[1] = e_tempR_tmp;
  f_tempR_tmp = normRowMatrix[1] * normRowMatrix[3];
  g_tempR_tmp = normRowMatrix[0] * normRowMatrix[2];
  h_tempR_tmp = 2.0 * (f_tempR_tmp + g_tempR_tmp);
  tempR[2] = h_tempR_tmp;
  c_tempR_tmp = 2.0 * (c_tempR_tmp + d_tempR_tmp);
  tempR[3] = c_tempR_tmp;
  d_tempR_tmp = normRowMatrix[1] * normRowMatrix[1];
  x = 1.0 - 2.0 * (d_tempR_tmp + x);
  tempR[4] = x;
  i_tempR_tmp = normRowMatrix[2] * normRowMatrix[3];
  j_tempR_tmp = normRowMatrix[0] * normRowMatrix[1];
  k_tempR_tmp = 2.0 * (i_tempR_tmp - j_tempR_tmp);
  tempR[5] = k_tempR_tmp;
  f_tempR_tmp = 2.0 * (f_tempR_tmp - g_tempR_tmp);
  tempR[6] = f_tempR_tmp;
  g_tempR_tmp = 2.0 * (i_tempR_tmp + j_tempR_tmp);
  tempR[7] = g_tempR_tmp;
  tempR_tmp = 1.0 - 2.0 * (d_tempR_tmp + tempR_tmp);
  tempR[8] = tempR_tmp;
  R[0] = b_tempR_tmp;
  R[1] = e_tempR_tmp;
  R[2] = h_tempR_tmp;
  R[3] = c_tempR_tmp;
  R[4] = x;
  R[5] = k_tempR_tmp;
  R[6] = f_tempR_tmp;
  R[7] = g_tempR_tmp;
  R[8] = tempR_tmp;
  for (k = 0; k < 3; k++) {
    R[k] = tempR[3 * k];
    R[k + 3] = tempR[3 * k + 1];
    R[k + 6] = tempR[3 * k + 2];
  }
  memset(&H[0], 0, 16U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    k = i << 2;
    H[k] = R[3 * i];
    H[k + 1] = R[3 * i + 1];
    H[k + 2] = R[3 * i + 2];
  }
  H[15] = 1.0;
}

/* End of code generation (quat2tform.c) */
