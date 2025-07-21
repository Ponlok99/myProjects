/*
 * rotm2eul.c
 *
 * Code generation for function 'rotm2eul'
 *
 */

/* Include files */
#include "rotm2eul.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo mp_emlrtRSI =
    {
        49,         /* lineNo */
        "rotm2eul", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutils\\rotm2eul"
        ".m" /* pathName */
};

static emlrtRSInfo np_emlrtRSI = {
    43,         /* lineNo */
    "rotm2eul", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\rotm2eul.m" /* pathName */
};

static emlrtRSInfo op_emlrtRSI = {
    140,                    /* lineNo */
    "calculateEulerAngles", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\rotm2eul.m" /* pathName */
};

static emlrtRSInfo pp_emlrtRSI = {
    130,                    /* lineNo */
    "calculateEulerAngles", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\rotm2eul.m" /* pathName */
};

static emlrtRSInfo up_emlrtRSI = {
    43,    /* lineNo */
    "cat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

static emlrtECInfo ab_emlrtECI = {
    -1,                     /* nDims */
    140,                    /* lineNo */
    13,                     /* colNo */
    "calculateEulerAngles", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\rotm2eul.m" /* pName */
};

static const int32_T iv1[3] = {1, 3, 1};

/* Function Definitions */
void b_rotm2eul(const emlrtStack *sp, const real_T R[9], real_T eul[3])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T sy;
  real_T sySq;
  real_T varargin_1_data;
  real_T varargin_1_data_idx_0;
  int32_T R_size[3];
  int32_T b_R_size[3];
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &mp_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_st.site = &np_emlrtRSI;
  sySq = R[5] * R[5] + R[2] * R[2];
  c_st.site = &pp_emlrtRSI;
  if (sySq < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  sy = muDoubleScalarSqrt(sySq);
  eul[0] = muDoubleScalarAtan2(R[5], R[2]);
  eul[1] = muDoubleScalarAtan2(sy, 1.0);
  eul[2] = muDoubleScalarAtan2(R[7], -R[6]);
  if (sySq < 2.2204460492503131E-15) {
    int32_T varargin_1_size[3];
    int32_T loop_ub;
    c_st.site = &op_emlrtRSI;
    R_size[0] = 1;
    R_size[1] = 1;
    R_size[2] = 1;
    b_R_size[0] = 1;
    b_R_size[1] = 1;
    b_R_size[2] = 1;
    sySq = -R[1];
    d_st.site = &op_emlrtRSI;
    b_atan2(&d_st, (real_T *)&sySq, R_size, (real_T *)&R[4], b_R_size,
            (real_T *)&varargin_1_data, varargin_1_size);
    d_st.site = &up_emlrtRSI;
    e_st.site = &qd_emlrtRSI;
    if (varargin_1_size[2] != 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &p_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if (varargin_1_size[2] != 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &p_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    R_size[0] = 1;
    R_size[1] = 3;
    R_size[2] = 1;
    emlrtSubAssignSizeCheckR2012b(&R_size[0], 3, &iv1[0], 3, &ab_emlrtECI,
                                  &b_st);
    loop_ub = varargin_1_size[2];
    for (i = 0; i < loop_ub; i++) {
      varargin_1_data_idx_0 = varargin_1_data;
    }
    eul[0] = varargin_1_data_idx_0;
    eul[1] = muDoubleScalarAtan2(sy, R[8]);
    eul[2] = 0.0;
  }
  r = _mm_loadu_pd(&eul[0]);
  _mm_storeu_pd(&eul[0], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  eul[2] = -eul[2];
  sySq = eul[0];
  eul[0] = eul[2];
  eul[2] = sySq;
}

void rotm2eul(const emlrtStack *sp, const real_T R[9], real_T eul[3])
{
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T sy;
  real_T sySq;
  real_T varargin_1_data;
  real_T varargin_1_data_idx_0;
  int32_T R_size[3];
  int32_T b_R_size[3];
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &mp_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_st.site = &np_emlrtRSI;
  sySq = R[5] * R[5] + R[2] * R[2];
  c_st.site = &pp_emlrtRSI;
  if (sySq < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  sy = muDoubleScalarSqrt(sySq);
  eul[0] = muDoubleScalarAtan2(R[5], R[2]);
  eul[1] = muDoubleScalarAtan2(sy, R[8]);
  eul[2] = muDoubleScalarAtan2(R[7], -R[6]);
  if (sySq < 2.2204460492503131E-15) {
    int32_T varargin_1_size[3];
    int32_T loop_ub;
    c_st.site = &op_emlrtRSI;
    R_size[0] = 1;
    R_size[1] = 1;
    R_size[2] = 1;
    b_R_size[0] = 1;
    b_R_size[1] = 1;
    b_R_size[2] = 1;
    sySq = -R[1];
    d_st.site = &op_emlrtRSI;
    b_atan2(&d_st, (real_T *)&sySq, R_size, (real_T *)&R[4], b_R_size,
            (real_T *)&varargin_1_data, varargin_1_size);
    d_st.site = &up_emlrtRSI;
    e_st.site = &qd_emlrtRSI;
    if (varargin_1_size[2] != 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &p_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if (varargin_1_size[2] != 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &p_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    R_size[0] = 1;
    R_size[1] = 3;
    R_size[2] = 1;
    emlrtSubAssignSizeCheckR2012b(&R_size[0], 3, &iv1[0], 3, &ab_emlrtECI,
                                  &b_st);
    loop_ub = varargin_1_size[2];
    for (i = 0; i < loop_ub; i++) {
      varargin_1_data_idx_0 = varargin_1_data;
    }
    eul[0] = varargin_1_data_idx_0;
    eul[1] = muDoubleScalarAtan2(sy, R[8]);
    eul[2] = 0.0;
  }
  r = _mm_loadu_pd(&eul[0]);
  _mm_storeu_pd(&eul[0], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
  eul[2] = -eul[2];
  sySq = eul[0];
  eul[0] = eul[2];
  eul[2] = sySq;
}

/* End of code generation (rotm2eul.c) */
