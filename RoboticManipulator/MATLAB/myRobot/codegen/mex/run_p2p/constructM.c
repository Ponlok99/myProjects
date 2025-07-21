/*
 * constructM.c
 *
 * Code generation for function 'constructM'
 *
 */

/* Include files */
#include "constructM.h"
#include "find.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ce_emlrtRSI = {
    18,           /* lineNo */
    "constructM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m" /* pathName */
};

static emlrtRSInfo de_emlrtRSI = {
    19,           /* lineNo */
    "constructM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m" /* pathName */
};

static emlrtRSInfo ee_emlrtRSI = {
    45,           /* lineNo */
    "constructM", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m" /* pathName */
};

static emlrtRSInfo fe_emlrtRSI = {
    39,     /* lineNo */
    "find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

static emlrtBCInfo x_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    22,           /* lineNo */
    19,           /* colNo */
    "",           /* aName */
    "constructM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m", /* pName */
    0                                      /* checkKind */
};

static emlrtBCInfo y_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    28,           /* lineNo */
    18,           /* colNo */
    "",           /* aName */
    "constructM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m", /* pName */
    0                                      /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = {
    1,            /* iFirst */
    8,            /* iLast */
    28,           /* lineNo */
    27,           /* colNo */
    "",           /* aName */
    "constructM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m", /* pName */
    3                                      /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = {
    1,            /* iFirst */
    10,           /* iLast */
    28,           /* lineNo */
    27,           /* colNo */
    "",           /* aName */
    "constructM", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\constructM.m", /* pName */
    3                                      /* checkKind */
};

/* Function Definitions */
void b_constructM(const emlrtStack *sp, const real_T constraints[10],
                  real_T M[100])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T M1[100];
  real_T Mcontinuity[100];
  real_T y[100];
  int32_T fixedBCIdx_data[10];
  int32_T ii_data[10];
  int32_T b_i;
  int32_T i;
  int32_T ii_size;
  int32_T k;
  int32_T kk;
  boolean_T b_fixedBCIdx_tmp[10];
  boolean_T fixedBCIdx_tmp[10];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  memset(&Mcontinuity[0], 0, 100U * sizeof(real_T));
  memset(&M1[0], 0, 100U * sizeof(real_T));
  for (i = 0; i < 10; i++) {
    fixedBCIdx_tmp[i] = muDoubleScalarIsNaN(constraints[i]);
  }
  st.site = &ce_emlrtRSI;
  for (k = 0; k < 10; k++) {
    b_fixedBCIdx_tmp[k] = !fixedBCIdx_tmp[k];
  }
  b_st.site = &fe_emlrtRSI;
  ii_size = b_eml_find(&b_st, b_fixedBCIdx_tmp, ii_data);
  i = ii_size;
  if (ii_size - 1 >= 0) {
    memcpy(&fixedBCIdx_data[0], &ii_data[0],
           (uint32_T)ii_size * sizeof(int32_T));
  }
  st.site = &de_emlrtRSI;
  b_st.site = &fe_emlrtRSI;
  ii_size = b_eml_find(&b_st, fixedBCIdx_tmp, ii_data);
  for (k = 0; k < i; k++) {
    if (k + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, i, &x_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    M1[(fixedBCIdx_data[k] + 10 * k) - 1] = 1.0;
  }
  k = 10 - i;
  for (kk = 0; kk < k; kk++) {
    if (kk + 1 > ii_size) {
      emlrtDynamicBoundsCheckR2012b(kk + 1, 1, ii_size, &y_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    b_i = (i + kk) + 1;
    if (b_i > 10) {
      emlrtDynamicBoundsCheckR2012b(b_i, 1, 10, &eb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    M1[(ii_data[kk] + 10 * (b_i - 1)) - 1] = 1.0;
  }
  for (i = 0; i < 10; i++) {
    Mcontinuity[i + 10 * i] = 1.0;
  }
  st.site = &ee_emlrtRSI;
  b_mtimes(Mcontinuity, M1, y);
  for (k = 0; k < 10; k++) {
    for (b_i = 0; b_i < 10; b_i++) {
      M[b_i + 10 * k] = y[k + 10 * b_i];
    }
  }
}

void constructM(const emlrtStack *sp, const real_T constraints[8], real_T M[64])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T fixedBCIdx_data[8];
  int32_T ii_data[8];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T ii_size;
  int32_T k;
  int8_T M1[64];
  int8_T Mcontinuity[64];
  boolean_T b_fixedBCIdx_tmp[8];
  boolean_T fixedBCIdx_tmp[8];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  memset(&Mcontinuity[0], 0, 64U * sizeof(int8_T));
  memset(&M1[0], 0, 64U * sizeof(int8_T));
  for (i = 0; i < 8; i++) {
    fixedBCIdx_tmp[i] = muDoubleScalarIsNaN(constraints[i]);
  }
  st.site = &ce_emlrtRSI;
  for (b_i = 0; b_i < 8; b_i++) {
    b_fixedBCIdx_tmp[b_i] = !fixedBCIdx_tmp[b_i];
  }
  b_st.site = &fe_emlrtRSI;
  ii_size = eml_find(&b_st, b_fixedBCIdx_tmp, ii_data);
  i = ii_size;
  if (ii_size - 1 >= 0) {
    memcpy(&fixedBCIdx_data[0], &ii_data[0],
           (uint32_T)ii_size * sizeof(int32_T));
  }
  st.site = &de_emlrtRSI;
  b_st.site = &fe_emlrtRSI;
  ii_size = eml_find(&b_st, fixedBCIdx_tmp, ii_data);
  for (k = 0; k < i; k++) {
    if (k + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, i, &x_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    M1[(fixedBCIdx_data[k] + (k << 3)) - 1] = 1;
  }
  b_i = 8 - i;
  for (k = 0; k < b_i; k++) {
    if (k + 1 > ii_size) {
      emlrtDynamicBoundsCheckR2012b(k + 1, 1, ii_size, &y_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i1 = (i + k) + 1;
    if (i1 > 8) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, 8, &ab_emlrtBCI, (emlrtConstCTX)sp);
    }
    M1[(ii_data[k] + ((i1 - 1) << 3)) - 1] = 1;
  }
  for (i = 0; i < 8; i++) {
    Mcontinuity[i + (i << 3)] = 1;
  }
  for (b_i = 0; b_i < 8; b_i++) {
    for (i1 = 0; i1 < 8; i1++) {
      real_T d;
      d = 0.0;
      for (i = 0; i < 8; i++) {
        d += (real_T)(Mcontinuity[i1 + (i << 3)] * M1[i + (b_i << 3)]);
      }
      M[b_i + (i1 << 3)] = d;
    }
  }
}

/* End of code generation (constructM.c) */
