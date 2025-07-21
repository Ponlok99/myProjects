/*
 * interpSnapTraj.c
 *
 * Code generation for function 'interpSnapTraj'
 *
 */

/* Include files */
#include "interpSnapTraj.h"
#include "histcounts.h"
#include "linspace.h"
#include "polyder.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo oh_emlrtRSI = {
    25,               /* lineNo */
    "interpSnapTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pathName */
};

static emlrtRSInfo ph_emlrtRSI = {
    39,               /* lineNo */
    "interpSnapTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pathName */
};

static emlrtRSInfo qh_emlrtRSI = {
    45,               /* lineNo */
    "interpSnapTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pathName */
};

static emlrtRSInfo rh_emlrtRSI = {
    52,               /* lineNo */
    "interpSnapTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pathName */
};

static emlrtRSInfo sh_emlrtRSI = {
    59,               /* lineNo */
    "interpSnapTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pathName */
};

static emlrtECInfo fb_emlrtECI = {
    -1,               /* nDims */
    60,               /* lineNo */
    13,               /* colNo */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pName */
};

static emlrtBCInfo ve_emlrtBCI = {
    1,                /* iFirst */
    6,                /* iLast */
    60,               /* lineNo */
    26,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo we_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    62,               /* lineNo */
    72,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo gb_emlrtECI = {
    -1,               /* nDims */
    53,               /* lineNo */
    13,               /* colNo */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pName */
};

static emlrtBCInfo xe_emlrtBCI = {
    1,                /* iFirst */
    7,                /* iLast */
    53,               /* lineNo */
    25,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo ye_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    55,               /* lineNo */
    71,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo hb_emlrtECI = {
    -1,               /* nDims */
    46,               /* lineNo */
    13,               /* colNo */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pName */
};

static emlrtBCInfo af_emlrtBCI = {
    1,                /* iFirst */
    8,                /* iLast */
    46,               /* lineNo */
    24,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo bf_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    48,               /* lineNo */
    70,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo ib_emlrtECI = {
    -1,               /* nDims */
    40,               /* lineNo */
    13,               /* colNo */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pName */
};

static emlrtBCInfo cf_emlrtBCI = {
    1,                /* iFirst */
    9,                /* iLast */
    40,               /* lineNo */
    23,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo df_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    41,               /* lineNo */
    69,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo ef_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    41,               /* lineNo */
    40,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo ff_emlrtBCI = {
    1,                /* iFirst */
    1,                /* iLast */
    33,               /* lineNo */
    88,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo gf_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    33,               /* lineNo */
    68,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo hf_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    33,               /* lineNo */
    39,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo if_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    10,               /* lineNo */
    50,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo jf_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    31,               /* lineNo */
    46,               /* colNo */
    "",               /* aName */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtRTEInfo dd_emlrtRTEI = {
    1,                /* lineNo */
    47,               /* colNo */
    "interpSnapTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpSnapTraj.m" /* pName */
};

/* Function Definitions */
void interpSnapTraj(const emlrtStack *sp, const real_T pp[30],
                    const emxArray_real_T *timePoints, real_T q[303],
                    real_T qd[303], real_T qdd[303], real_T qddd[303],
                    real_T qdddd[303], real_T tSamples[101])
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *a__1;
  real_T segmentNum[101];
  real_T polydCoef[9];
  real_T polyddCoef[8];
  const real_T *timePoints_data;
  int32_T b_k;
  int32_T i;
  int32_T i1;
  int32_T k;
  int32_T kk;
  int32_T ny;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  timePoints_data = timePoints->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  i = timePoints->size[1];
  if (i < 1) {
    emlrtDynamicBoundsCheckR2012b(i, 1, i, &if_emlrtBCI, (emlrtConstCTX)sp);
  }
  linspace(timePoints_data[0], timePoints_data[i - 1], tSamples);
  emxInit_real_T(sp, &a__1, 2, &dd_emlrtRTEI);
  st.site = &oh_emlrtRSI;
  histcounts(&st, tSamples, timePoints, a__1, segmentNum);
  emxFree_real_T(sp, &a__1);
  for (k = 0; k < 3; k++) {
    real_T d;
    d = pp[10 * k + 9];
    for (kk = 0; kk < 101; kk++) {
      real_T derivative_data[9];
      real_T a_data[8];
      real_T polydddCoef[7];
      real_T polyddddCoef[6];
      real_T d1;
      real_T delT;
      real_T v;
      int32_T derivative_size[2];
      int32_T nlead0;
      int32_T q_tmp;
      d1 = segmentNum[kk];
      if (((int32_T)d1 < 1) || ((int32_T)d1 > i)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, i, &jf_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      delT = tSamples[kk] - timePoints_data[(int32_T)d1 - 1];
      if ((int32_T)d1 > 1) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, 1, &ff_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (((int32_T)(d1 + 1.0) < 1) || ((int32_T)(d1 + 1.0) > i)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(d1 + 1.0), 1, i, &hf_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if ((int32_T)d1 > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, i, &gf_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = pp[10 * k];
        for (ny = 0; ny < 9; ny++) {
          v = delT * v + pp[(ny + 10 * k) + 1];
        }
      }
      q_tmp = k + 3 * kk;
      q[q_tmp] = v;
      memset(&polydCoef[0], 0, 9U * sizeof(real_T));
      st.site = &ph_emlrtRSI;
      b_st.site = &ch_emlrtRSI;
      nlead0 = 0;
      b_k = 0;
      while ((b_k < 8) && (pp[b_k + 10 * k] == 0.0)) {
        nlead0++;
        b_k++;
      }
      ny = 9 - nlead0;
      if (9 - nlead0 > 9) {
        emlrtErrorWithMessageIdR2018a(&b_st, &cb_emlrtRTEI,
                                      "Coder:builtins:AssertionFailed",
                                      "Coder:builtins:AssertionFailed", 0);
      }
      if (9 - nlead0 < 0) {
        emlrtNonNegativeCheckR2012b(9 - nlead0, &i_emlrtDCI, &b_st);
      }
      for (b_k = 0; b_k < ny; b_k++) {
        derivative_data[b_k] = pp[(b_k + nlead0) + 10 * k];
      }
      ny = 9 - nlead0;
      for (b_k = 0; b_k <= ny - 2; b_k++) {
        derivative_data[b_k] *= (real_T)(8 - (nlead0 + b_k)) + 1.0;
      }
      if (muDoubleScalarIsInf(d) || muDoubleScalarIsNaN(d)) {
        derivative_data[8 - nlead0] = rtNaN;
      }
      if (nlead0 + 1 > 9) {
        b_k = -1;
        i1 = -1;
      } else {
        if (nlead0 + 1 > 9) {
          emlrtDynamicBoundsCheckR2012b(10, 1, 9, &cf_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        b_k = nlead0 - 1;
        i1 = 8;
      }
      i1 -= b_k;
      if (i1 != 9 - nlead0) {
        emlrtSubAssignSizeCheck1dR2017a(i1, 9 - nlead0, &ib_emlrtECI,
                                        (emlrtConstCTX)sp);
      }
      for (i1 = 0; i1 < ny; i1++) {
        polydCoef[(b_k + i1) + 1] = derivative_data[i1];
      }
      if ((int32_T)(d1 + 1.0) > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(d1 + 1.0), 1, i, &ef_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if ((int32_T)d1 > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, i, &df_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = polydCoef[0];
        for (ny = 0; ny < 8; ny++) {
          v = delT * v + polydCoef[ny + 1];
        }
      }
      qd[q_tmp] = v;
      memset(&polyddCoef[0], 0, 8U * sizeof(real_T));
      st.site = &qh_emlrtRSI;
      b_st.site = &ch_emlrtRSI;
      nlead0 = 0;
      b_k = 0;
      while ((b_k < 7) && (polydCoef[b_k] == 0.0)) {
        nlead0++;
        b_k++;
      }
      ny = 8 - nlead0;
      if (8 - nlead0 > 8) {
        emlrtErrorWithMessageIdR2018a(&b_st, &cb_emlrtRTEI,
                                      "Coder:builtins:AssertionFailed",
                                      "Coder:builtins:AssertionFailed", 0);
      }
      if (8 - nlead0 < 0) {
        emlrtNonNegativeCheckR2012b(8 - nlead0, &i_emlrtDCI, &b_st);
      }
      for (b_k = 0; b_k < ny; b_k++) {
        a_data[b_k] = polydCoef[b_k + nlead0];
      }
      for (b_k = 0; b_k <= ny - 2; b_k++) {
        a_data[b_k] *= (real_T)(7 - (nlead0 + b_k)) + 1.0;
      }
      if (muDoubleScalarIsInf(polydCoef[8]) ||
          muDoubleScalarIsNaN(polydCoef[8])) {
        a_data[7 - nlead0] = rtNaN;
      }
      if (nlead0 + 1 > 8) {
        b_k = -1;
        i1 = -1;
      } else {
        if (nlead0 + 1 > 8) {
          emlrtDynamicBoundsCheckR2012b(9, 1, 8, &af_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        b_k = nlead0 - 1;
        i1 = 7;
      }
      i1 -= b_k;
      if (i1 != 8 - nlead0) {
        emlrtSubAssignSizeCheck1dR2017a(i1, 8 - nlead0, &hb_emlrtECI,
                                        (emlrtConstCTX)sp);
      }
      for (i1 = 0; i1 < ny; i1++) {
        polyddCoef[(b_k + i1) + 1] = a_data[i1];
      }
      if ((int32_T)d1 > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, i, &bf_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = polyddCoef[0];
        for (ny = 0; ny < 7; ny++) {
          v = delT * v + polyddCoef[ny + 1];
        }
      }
      qdd[q_tmp] = v;
      for (b_k = 0; b_k < 7; b_k++) {
        polydddCoef[b_k] = 0.0;
      }
      st.site = &rh_emlrtRSI;
      polyder(&st, polyddCoef, derivative_data, derivative_size);
      ny = derivative_size[1];
      if (8 - derivative_size[1] > 7) {
        b_k = -1;
        i1 = -1;
      } else {
        if (8 - derivative_size[1] > 7) {
          emlrtDynamicBoundsCheckR2012b(8, 1, 7, &xe_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        b_k = 6 - derivative_size[1];
        i1 = 6;
      }
      i1 -= b_k;
      if (i1 != derivative_size[1]) {
        emlrtSubAssignSizeCheck1dR2017a(i1, derivative_size[1], &gb_emlrtECI,
                                        (emlrtConstCTX)sp);
      }
      for (i1 = 0; i1 < ny; i1++) {
        polydddCoef[(b_k + i1) + 1] = derivative_data[i1];
      }
      if ((int32_T)d1 > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, i, &ye_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = polydddCoef[0];
        for (ny = 0; ny < 6; ny++) {
          v = delT * v + polydddCoef[ny + 1];
        }
      }
      qddd[q_tmp] = v;
      for (b_k = 0; b_k < 6; b_k++) {
        polyddddCoef[b_k] = 0.0;
      }
      st.site = &sh_emlrtRSI;
      b_polyder(&st, polydddCoef, derivative_data, derivative_size);
      ny = derivative_size[1];
      if (7 - derivative_size[1] > 6) {
        b_k = -1;
        i1 = -1;
      } else {
        if (7 - derivative_size[1] > 6) {
          emlrtDynamicBoundsCheckR2012b(7, 1, 6, &ve_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        b_k = 5 - derivative_size[1];
        i1 = 5;
      }
      i1 -= b_k;
      if (i1 != derivative_size[1]) {
        emlrtSubAssignSizeCheck1dR2017a(i1, derivative_size[1], &fb_emlrtECI,
                                        (emlrtConstCTX)sp);
      }
      for (i1 = 0; i1 < ny; i1++) {
        polyddddCoef[(b_k + i1) + 1] = derivative_data[i1];
      }
      if ((int32_T)d1 > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d1, 1, i, &we_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = polyddddCoef[0];
        for (ny = 0; ny < 5; ny++) {
          v = delT * v + polyddddCoef[ny + 1];
        }
      }
      qdddd[q_tmp] = v;
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (interpSnapTraj.c) */
