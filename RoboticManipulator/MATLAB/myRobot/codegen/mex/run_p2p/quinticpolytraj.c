/*
 * quinticpolytraj.c
 *
 * Code generation for function 'quinticpolytraj'
 *
 */

/* Include files */
#include "quinticpolytraj.h"
#include "mpower.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo th_emlrtRSI = {
    72,                /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo uh_emlrtRSI = {
    85,                /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo vh_emlrtRSI = {
    86,                /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo wh_emlrtRSI = {
    87,                /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo xh_emlrtRSI = {
    88,                /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo yh_emlrtRSI = {
    115,               /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo ai_emlrtRSI = {
    125,               /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo bi_emlrtRSI = {
    136,               /* lineNo */
    "quinticpolytraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo ci_emlrtRSI = {
    183,                     /* lineNo */
    "generateQuinticCoeffs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo di_emlrtRSI = {
    188,                     /* lineNo */
    "generateQuinticCoeffs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo ei_emlrtRSI = {
    189,                     /* lineNo */
    "generateQuinticCoeffs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

static emlrtRSInfo fi_emlrtRSI = {
    190,                     /* lineNo */
    "generateQuinticCoeffs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\quinticpolytraj.m" /* pathName
                                                                           */
};

/* Function Definitions */
void b_quinticpolytraj(const emlrtStack *sp, const real_T wayPoints[6],
                       const real_T varargin_2[6], const real_T varargin_4[6],
                       real_T q[303], real_T qd[303], real_T qdd[303])
{
  static const real_T invTMatF[9] = {10.0, -15.0, 6.0,  -4.0, 7.0,
                                     -3.0, 0.5,   -1.0, 0.5};
  static const int8_T TMat0[9] = {1, 0, 0, 1, 1, 0, 1, 2, 2};
  static const int8_T b_iv[6] = {0, 0, 0, 0, 0, 1};
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T dCoeffs[54];
  real_T ddCoeffs[54];
  real_T coefsWithFlatStart[36];
  real_T coefMat[18];
  real_T coeffMat[18];
  real_T coeffVec[6];
  real_T derivativeBreaks[4];
  real_T xtmp;
  int32_T b_i;
  int32_T i;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &th_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &vc_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 6)) {
    if ((!muDoubleScalarIsInf(wayPoints[i])) &&
        (!muDoubleScalarIsNaN(wayPoints[i]))) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:quinticpolytraj:expectedFinite", 3, 4, 9, "wayPoints");
  }
  st.site = &uh_emlrtRSI;
  b_st.site = &gd_emlrtRSI;
  st.site = &vh_emlrtRSI;
  st.site = &wh_emlrtRSI;
  st.site = &xh_emlrtRSI;
  for (i = 0; i < 3; i++) {
    real_T y[3];
    real_T wayPoints_idx_1;
    real_T wayPoints_idx_2;
    st.site = &yh_emlrtRSI;
    coeffVec[0] = wayPoints[i];
    coeffVec[1] = varargin_2[i];
    coeffVec[2] = varargin_4[i] / 2.0;
    coeffVec[3] = 0.0;
    coeffVec[4] = 0.0;
    coeffVec[5] = 0.0;
    b_st.site = &ci_emlrtRSI;
    xtmp = coeffVec[0];
    wayPoints_idx_1 = coeffVec[1];
    wayPoints_idx_2 = coeffVec[2];
    for (b_i = 0; b_i < 3; b_i++) {
      y[b_i] = ((real_T)TMat0[b_i] * xtmp +
                (real_T)TMat0[b_i + 3] * wayPoints_idx_1) +
               (real_T)TMat0[b_i + 6] * wayPoints_idx_2;
    }
    b_st.site = &di_emlrtRSI;
    b_st.site = &di_emlrtRSI;
    b_st.site = &ei_emlrtRSI;
    b_st.site = &ei_emlrtRSI;
    b_st.site = &ei_emlrtRSI;
    b_st.site = &fi_emlrtRSI;
    b_st.site = &fi_emlrtRSI;
    b_st.site = &fi_emlrtRSI;
    xtmp = wayPoints[i + 3] - y[0];
    wayPoints_idx_1 = varargin_2[i + 3] - y[1];
    wayPoints_idx_2 = varargin_4[i + 3] - y[2];
    r = _mm_loadu_pd(&invTMatF[0]);
    r = _mm_mul_pd(r, _mm_set1_pd(xtmp));
    r1 = _mm_loadu_pd(&invTMatF[3]);
    r1 = _mm_mul_pd(r1, _mm_set1_pd(wayPoints_idx_1));
    r = _mm_add_pd(r, r1);
    r1 = _mm_loadu_pd(&invTMatF[6]);
    r1 = _mm_mul_pd(r1, _mm_set1_pd(wayPoints_idx_2));
    r = _mm_add_pd(r, r1);
    _mm_storeu_pd(&coeffVec[3], r);
    coeffVec[5] = (invTMatF[2] * xtmp + invTMatF[5] * wayPoints_idx_1) +
                  invTMatF[8] * wayPoints_idx_2;
    xtmp = coeffVec[0];
    coeffVec[0] = coeffVec[5];
    coeffVec[5] = xtmp;
    xtmp = coeffVec[1];
    coeffVec[1] = coeffVec[4];
    coeffVec[4] = xtmp;
    xtmp = coeffVec[2];
    coeffVec[2] = coeffVec[3];
    coeffVec[3] = xtmp;
    for (b_i = 0; b_i < 6; b_i++) {
      coefMat[i + 3 * b_i] = coeffVec[b_i];
    }
  }
  st.site = &ai_emlrtRSI;
  memset(&coeffMat[0], 0, 18U * sizeof(real_T));
  for (b_i = 0; b_i < 3; b_i++) {
    xtmp = 0.0;
    for (i = 0; i < 6; i++) {
      xtmp += coefMat[b_i + 3 * i] * (real_T)b_iv[i];
    }
    coeffMat[b_i + 15] = xtmp;
  }
  memset(&coefsWithFlatStart[0], 0, 36U * sizeof(real_T));
  for (b_i = 0; b_i < 6; b_i++) {
    coefsWithFlatStart[6 * b_i] = coeffMat[3 * b_i];
    coefsWithFlatStart[6 * b_i + 3] = coefMat[3 * b_i];
    i = 3 * b_i + 1;
    coefsWithFlatStart[6 * b_i + 1] = coeffMat[i];
    coefsWithFlatStart[6 * b_i + 4] = coefMat[i];
    i = 3 * b_i + 2;
    coefsWithFlatStart[6 * b_i + 2] = coeffMat[i];
    coefsWithFlatStart[6 * b_i + 5] = coefMat[i];
  }
  b_st.site = &gi_emlrtRSI;
  for (i = 0; i < 6; i++) {
    c_st.site = &hi_emlrtRSI;
    mpower(&c_st, 1.0, 6.0 - ((real_T)i + 1.0));
  }
  memset(&coeffMat[0], 0, 18U * sizeof(real_T));
  for (b_i = 0; b_i < 3; b_i++) {
    xtmp = 0.0;
    for (i = 0; i < 6; i++) {
      xtmp += coefsWithFlatStart[(b_i + 6 * i) + 3];
    }
    coeffMat[b_i + 15] = xtmp;
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      ddCoeffs[i + 9 * b_i] = coefsWithFlatStart[i + 6 * b_i];
    }
    ddCoeffs[9 * b_i + 6] = coeffMat[3 * b_i];
    ddCoeffs[9 * b_i + 7] = coeffMat[3 * b_i + 1];
    ddCoeffs[9 * b_i + 8] = coeffMat[3 * b_i + 2];
  }
  derivativeBreaks[0] = -1.0;
  derivativeBreaks[1] = 0.0;
  derivativeBreaks[2] = 1.0;
  derivativeBreaks[3] = 2.0;
  b_ppval(derivativeBreaks, ddCoeffs, q);
  st.site = &bi_emlrtRSI;
  derivativeBreaks[0] = -1.0;
  derivativeBreaks[1] = 0.0;
  derivativeBreaks[3] = 2.0;
  derivativeBreaks[2] = 1.01;
  memset(&dCoeffs[0], 0, 54U * sizeof(real_T));
  for (i = 0; i < 5; i++) {
    r = _mm_loadu_pd(&ddCoeffs[9 * i]);
    b_i = 9 * (i + 1);
    r1 = _mm_set1_pd((6.0 - ((real_T)i + 2.0)) + 1.0);
    _mm_storeu_pd(&dCoeffs[b_i], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&ddCoeffs[9 * i + 2]);
    _mm_storeu_pd(&dCoeffs[b_i + 2], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&ddCoeffs[9 * i + 4]);
    _mm_storeu_pd(&dCoeffs[b_i + 4], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&ddCoeffs[9 * i + 6]);
    _mm_storeu_pd(&dCoeffs[b_i + 6], _mm_mul_pd(r1, r));
    dCoeffs[b_i + 8] = ((6.0 - ((real_T)i + 2.0)) + 1.0) * ddCoeffs[9 * i + 8];
  }
  b_ppval(derivativeBreaks, dCoeffs, qd);
  memset(&ddCoeffs[0], 0, 54U * sizeof(real_T));
  for (i = 0; i < 5; i++) {
    r = _mm_loadu_pd(&dCoeffs[9 * i]);
    b_i = 9 * (i + 1);
    r1 = _mm_set1_pd((6.0 - ((real_T)i + 2.0)) + 1.0);
    _mm_storeu_pd(&ddCoeffs[b_i], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&dCoeffs[9 * i + 2]);
    _mm_storeu_pd(&ddCoeffs[b_i + 2], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&dCoeffs[9 * i + 4]);
    _mm_storeu_pd(&ddCoeffs[b_i + 4], _mm_mul_pd(r1, r));
    r = _mm_loadu_pd(&dCoeffs[9 * i + 6]);
    _mm_storeu_pd(&ddCoeffs[b_i + 6], _mm_mul_pd(r1, r));
    ddCoeffs[b_i + 8] = ((6.0 - ((real_T)i + 2.0)) + 1.0) * dCoeffs[9 * i + 8];
  }
  b_ppval(derivativeBreaks, ddCoeffs, qdd);
}

/* End of code generation (quinticpolytraj.c) */
