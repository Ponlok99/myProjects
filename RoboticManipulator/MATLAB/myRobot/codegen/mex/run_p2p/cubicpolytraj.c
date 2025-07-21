/*
 * cubicpolytraj.c
 *
 * Code generation for function 'cubicpolytraj'
 *
 */

/* Include files */
#include "cubicpolytraj.h"
#include "mpower.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ii_emlrtRSI = {
    62,              /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo ji_emlrtRSI = {
    75,              /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo ki_emlrtRSI = {
    76,              /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo li_emlrtRSI = {
    77,              /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo mi_emlrtRSI = {
    103,             /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo ni_emlrtRSI = {
    113,             /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo oi_emlrtRSI = {
    124,             /* lineNo */
    "cubicpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo pi_emlrtRSI = {
    171,                   /* lineNo */
    "generateCubicCoeffs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

static emlrtRSInfo qi_emlrtRSI = {
    172,                   /* lineNo */
    "generateCubicCoeffs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\cubicpolytraj.m" /* pathName
                                                                         */
};

/* Function Definitions */
void b_cubicpolytraj(const emlrtStack *sp, const real_T wayPoints[6],
                     const real_T varargin_2[6], real_T q[303], real_T qd[303],
                     real_T qdd[303])
{
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T dCoeffs[36];
  real_T ddCoeffs[36];
  real_T coefsWithFlatStart[24];
  real_T coefMat[12];
  real_T coeffMat[12];
  real_T coeffVec[4];
  int32_T i;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ii_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(wayPoints[k])) &&
        (!muDoubleScalarIsNaN(wayPoints[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:cubicpolytraj:expectedFinite", 3, 4, 9, "wayPoints");
  }
  st.site = &ji_emlrtRSI;
  b_st.site = &gd_emlrtRSI;
  st.site = &ki_emlrtRSI;
  st.site = &li_emlrtRSI;
  for (k = 0; k < 3; k++) {
    real_T wayPoints_idx_0;
    real_T wayPoints_idx_1;
    st.site = &mi_emlrtRSI;
    coeffVec[0] = wayPoints[k];
    coeffVec[1] = varargin_2[k];
    b_st.site = &pi_emlrtRSI;
    b_st.site = &qi_emlrtRSI;
    b_st.site = &qi_emlrtRSI;
    wayPoints_idx_0 = wayPoints[k + 3] - (coeffVec[0] + coeffVec[1]);
    wayPoints_idx_1 = varargin_2[k + 3] - (0.0 * coeffVec[0] + coeffVec[1]);
    coeffVec[3] = coeffVec[0];
    coefMat[k] = -2.0 * wayPoints_idx_0 + wayPoints_idx_1;
    coefMat[k + 3] = 3.0 * wayPoints_idx_0 - wayPoints_idx_1;
    coefMat[k + 6] = coeffVec[1];
    coefMat[k + 9] = coeffVec[3];
  }
  st.site = &ni_emlrtRSI;
  memset(&coeffMat[0], 0, 12U * sizeof(real_T));
  r = _mm_loadu_pd(&coefMat[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(0.0));
  r1 = _mm_loadu_pd(&coefMat[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(0.0));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&coefMat[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(0.0));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&coefMat[9]);
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&coeffMat[9], r);
  coeffMat[11] =
      ((coefMat[2] * 0.0 + coefMat[5] * 0.0) + coefMat[8] * 0.0) + coefMat[11];
  memset(&coefsWithFlatStart[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    coefsWithFlatStart[6 * i] = coeffMat[3 * i];
    coefsWithFlatStart[6 * i + 3] = coefMat[3 * i];
    k = 3 * i + 1;
    coefsWithFlatStart[6 * i + 1] = coeffMat[k];
    coefsWithFlatStart[6 * i + 4] = coefMat[k];
    k = 3 * i + 2;
    coefsWithFlatStart[6 * i + 2] = coeffMat[k];
    coefsWithFlatStart[6 * i + 5] = coefMat[k];
  }
  b_st.site = &gi_emlrtRSI;
  c_st.site = &hi_emlrtRSI;
  mpower(&c_st, 1.0, 3.0);
  c_st.site = &hi_emlrtRSI;
  mpower(&c_st, 1.0, 2.0);
  c_st.site = &hi_emlrtRSI;
  mpower(&c_st, 1.0, 1.0);
  c_st.site = &hi_emlrtRSI;
  mpower(&c_st, 1.0, 0.0);
  memset(&coeffMat[0], 0, 12U * sizeof(real_T));
  r = _mm_loadu_pd(&coefsWithFlatStart[3]);
  r1 = _mm_loadu_pd(&coefsWithFlatStart[9]);
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&coefsWithFlatStart[15]);
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&coefsWithFlatStart[21]);
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&coeffMat[9], r);
  coeffMat[11] = ((coefsWithFlatStart[5] + coefsWithFlatStart[11]) +
                  coefsWithFlatStart[17]) +
                 coefsWithFlatStart[23];
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 6; k++) {
      ddCoeffs[k + 9 * i] = coefsWithFlatStart[k + 6 * i];
    }
    ddCoeffs[9 * i + 6] = coeffMat[3 * i];
    ddCoeffs[9 * i + 7] = coeffMat[3 * i + 1];
    ddCoeffs[9 * i + 8] = coeffMat[3 * i + 2];
  }
  coeffVec[0] = -1.0;
  coeffVec[1] = 0.0;
  coeffVec[2] = 1.0;
  coeffVec[3] = 2.0;
  c_ppval(coeffVec, ddCoeffs, q);
  st.site = &oi_emlrtRSI;
  coeffVec[0] = -1.0;
  coeffVec[1] = 0.0;
  coeffVec[3] = 2.0;
  coeffVec[2] = 1.01;
  memset(&dCoeffs[0], 0, 36U * sizeof(real_T));
  for (k = 0; k < 3; k++) {
    r = _mm_loadu_pd(&ddCoeffs[9 * k]);
    i = 9 * (k + 1);
    r1 = _mm_set1_pd((4.0 - ((real_T)k + 2.0)) + 1.0);
    _mm_storeu_pd(&dCoeffs[i], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&ddCoeffs[9 * k + 2]);
    _mm_storeu_pd(&dCoeffs[i + 2], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&ddCoeffs[9 * k + 4]);
    _mm_storeu_pd(&dCoeffs[i + 4], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&ddCoeffs[9 * k + 6]);
    _mm_storeu_pd(&dCoeffs[i + 6], _mm_mul_pd(r1, r));
    dCoeffs[i + 8] = ((4.0 - ((real_T)k + 2.0)) + 1.0) * ddCoeffs[9 * k + 8];
  }
  c_ppval(coeffVec, dCoeffs, qd);
  memset(&ddCoeffs[0], 0, 36U * sizeof(real_T));
  for (k = 0; k < 3; k++) {
    r = _mm_loadu_pd(&dCoeffs[9 * k]);
    i = 9 * (k + 1);
    r1 = _mm_set1_pd((4.0 - ((real_T)k + 2.0)) + 1.0);
    _mm_storeu_pd(&ddCoeffs[i], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&dCoeffs[9 * k + 2]);
    _mm_storeu_pd(&ddCoeffs[i + 2], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&dCoeffs[9 * k + 4]);
    _mm_storeu_pd(&ddCoeffs[i + 4], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&dCoeffs[9 * k + 6]);
    _mm_storeu_pd(&ddCoeffs[i + 6], _mm_mul_pd(r1, r));
    ddCoeffs[i + 8] = ((4.0 - ((real_T)k + 2.0)) + 1.0) * dCoeffs[9 * k + 8];
  }
  c_ppval(coeffVec, ddCoeffs, qdd);
}

/* End of code generation (cubicpolytraj.c) */
