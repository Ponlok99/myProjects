/*
 * minjerkpolytraj.c
 *
 * Code generation for function 'minjerkpolytraj'
 *
 */

/* Include files */
#include "minjerkpolytraj.h"
#include "assertValidSizeArg.h"
#include "computePolyCoefAndTimeOfArrival.h"
#include "histcounts.h"
#include "linspace.h"
#include "polyder.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "validateTrajConstNameValueInputs.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo mc_emlrtRSI =
    {
        128,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI =
    {
        156,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI =
    {
        159,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI =
    {
        164,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo qc_emlrtRSI =
    {
        180,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo rc_emlrtRSI =
    {
        183,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo sc_emlrtRSI =
    {
        188,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo tc_emlrtRSI =
    {
        192,               /* lineNo */
        "minjerkpolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pathName */
};

static emlrtRSInfo vg_emlrtRSI = {
    26,               /* lineNo */
    "interpJerkTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pathName */
};

static emlrtRSInfo wg_emlrtRSI = {
    40,               /* lineNo */
    "interpJerkTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pathName */
};

static emlrtRSInfo xg_emlrtRSI = {
    46,               /* lineNo */
    "interpJerkTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pathName */
};

static emlrtRSInfo yg_emlrtRSI = {
    53,               /* lineNo */
    "interpJerkTraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pathName */
};

static emlrtECInfo k_emlrtECI = {
    -1,                 /* nDims */
    25,                 /* lineNo */
    13,                 /* colNo */
    "orderConstraints", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\orderConstraints.m" /* pName */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    34,               /* lineNo */
    39,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    34,               /* lineNo */
    68,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    1,                /* iFirst */
    1,                /* iLast */
    34,               /* lineNo */
    88,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    42,               /* lineNo */
    40,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    42,               /* lineNo */
    69,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    1,                /* iFirst */
    7,                /* iLast */
    41,               /* lineNo */
    23,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo l_emlrtECI = {
    -1,               /* nDims */
    41,               /* lineNo */
    13,               /* colNo */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pName */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    49,               /* lineNo */
    70,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = {
    1,                /* iFirst */
    6,                /* iLast */
    47,               /* lineNo */
    24,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo m_emlrtECI = {
    -1,               /* nDims */
    47,               /* lineNo */
    13,               /* colNo */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pName */
};

static emlrtBCInfo t_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    56,               /* lineNo */
    42,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    56,               /* lineNo */
    71,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = {
    1,                /* iFirst */
    5,                /* iLast */
    54,               /* lineNo */
    25,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtECInfo n_emlrtECI = {
    -1,               /* nDims */
    54,               /* lineNo */
    13,               /* colNo */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m" /* pName */
};

static emlrtBCInfo w_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    32,               /* lineNo */
    46,               /* colNo */
    "",               /* aName */
    "interpJerkTraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\interpJerkTraj.m", /* pName */
    0                                          /* checkKind */
};

static emlrtRTEInfo ic_emlrtRTEI =
    {
        1,                 /* lineNo */
        51,                /* colNo */
        "minjerkpolytraj", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minjerkpolytraj.m" /* pName */
};

/* Function Definitions */
void b_minjerkpolytraj(const emlrtStack *sp, const real_T waypoints[6],
                       const real_T varargin_2[6], const real_T varargin_4[6],
                       const real_T varargin_6[6], real_T q[303],
                       real_T qd[303], real_T qdd[303])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_real_T *a__1;
  emxArray_real_T *timeOfArrival;
  real_T segmentNum[101];
  real_T tSamples[101];
  real_T constraints[24];
  real_T ppMatrix[24];
  real_T derivative_data[7];
  real_T accBC[6];
  real_T b_waypoints[6];
  real_T jerkBC[6];
  real_T polyddCoef[6];
  real_T newsize[3];
  real_T *timeOfArrival_data;
  int32_T b_iv[2];
  int32_T derivative_size[2];
  int32_T i;
  int32_T i1;
  int32_T ic;
  int32_T k;
  int32_T kk;
  int32_T waypoints_tmp;
  boolean_T x_data[6];
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &mc_emlrtRSI;
  b_st.site = &uc_emlrtRSI;
  c_st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(waypoints[k])) &&
        (!muDoubleScalarIsNaN(waypoints[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:minjerkpolytraj:expectedFinite", 3, 4, 9, "waypoints");
  }
  st.site = &nc_emlrtRSI;
  b_st.site = &wc_emlrtRSI;
  c_st.site = &gd_emlrtRSI;
  b_st.site = &xc_emlrtRSI;
  b_st.site = &yc_emlrtRSI;
  b_st.site = &ad_emlrtRSI;
  b_st.site = &bd_emlrtRSI;
  b_st.site = &cd_emlrtRSI;
  b_st.site = &dd_emlrtRSI;
  b_st.site = &ed_emlrtRSI;
  b_st.site = &fd_emlrtRSI;
  st.site = &oc_emlrtRSI;
  c_validateTrajConstNameValueInp();
  st.site = &pc_emlrtRSI;
  for (i = 0; i < 6; i++) {
    x_data[i] = muDoubleScalarIsInf(varargin_2[i]);
  }
  p = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 5)) {
    if (x_data[k]) {
      p = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &e_emlrtRTEI, "shared_uav_rst:minjerkpolytraj:VelocityBCFinite",
        "shared_uav_rst:minjerkpolytraj:VelocityBCFinite", 0);
  }
  for (i = 0; i < 6; i++) {
    x_data[i] = muDoubleScalarIsInf(varargin_4[i]);
  }
  p = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 5)) {
    if (x_data[k]) {
      p = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &f_emlrtRTEI,
        "shared_uav_rst:minjerkpolytraj:AccelerationBCFinite",
        "shared_uav_rst:minjerkpolytraj:AccelerationBCFinite", 0);
  }
  for (i = 0; i < 6; i++) {
    x_data[i] = muDoubleScalarIsInf(varargin_6[i]);
  }
  p = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 5)) {
    if (x_data[k]) {
      p = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &g_emlrtRTEI, "shared_uav_rst:minjerkpolytraj:JerkBCFinite",
        "shared_uav_rst:minjerkpolytraj:JerkBCFinite", 0);
  }
  st.site = &qc_emlrtRSI;
  for (i = 0; i < 3; i++) {
    waypoints_tmp = i << 1;
    b_waypoints[waypoints_tmp] = waypoints[i];
    b_waypoints[waypoints_tmp + 1] = waypoints[i + 3];
  }
  for (i = 0; i < 6; i++) {
    polyddCoef[i] = b_waypoints[i];
  }
  for (i = 0; i < 3; i++) {
    waypoints_tmp = i << 1;
    b_waypoints[waypoints_tmp] = varargin_2[i];
    accBC[waypoints_tmp] = varargin_4[i];
    jerkBC[waypoints_tmp] = varargin_6[i];
    b_waypoints[waypoints_tmp + 1] = varargin_2[i + 3];
    accBC[waypoints_tmp + 1] = varargin_4[i + 3];
    jerkBC[waypoints_tmp + 1] = varargin_6[i + 3];
  }
  memset(&constraints[0], 0, 24U * sizeof(real_T));
  derivative_size[0] = 4;
  derivative_size[1] = 3;
  for (k = 0; k < 2; k++) {
    real_T b_polyddCoef[12];
    i = k << 2;
    ic = (k + 1) << 2;
    if (i + 1 > ic) {
      i = 0;
      ic = 0;
    }
    waypoints_tmp = ic - i;
    b_iv[0] = waypoints_tmp;
    b_iv[1] = 3;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &derivative_size[0], 2,
                                  &k_emlrtECI, &st);
    b_polyddCoef[0] = polyddCoef[k];
    b_polyddCoef[1] = b_waypoints[k];
    b_polyddCoef[2] = accBC[k];
    b_polyddCoef[3] = jerkBC[k];
    b_polyddCoef[4] = polyddCoef[k + 2];
    b_polyddCoef[5] = b_waypoints[k + 2];
    b_polyddCoef[6] = accBC[k + 2];
    b_polyddCoef[7] = jerkBC[k + 2];
    b_polyddCoef[8] = polyddCoef[k + 4];
    b_polyddCoef[9] = b_waypoints[k + 4];
    b_polyddCoef[10] = accBC[k + 4];
    b_polyddCoef[11] = jerkBC[k + 4];
    for (ic = 0; ic < 3; ic++) {
      for (i1 = 0; i1 < waypoints_tmp; i1++) {
        constraints[(i + i1) + (ic << 3)] =
            b_polyddCoef[i1 + waypoints_tmp * ic];
      }
    }
  }
  emxInit_real_T(sp, &timeOfArrival, 2, &ic_emlrtRTEI);
  st.site = &rc_emlrtRSI;
  computePolyCoefAndTimeOfArrival(&st, constraints, ppMatrix, timeOfArrival);
  timeOfArrival_data = timeOfArrival->data;
  st.site = &sc_emlrtRSI;
  b_st.site = &sg_emlrtRSI;
  if (timeOfArrival->size[1] < 2) {
    emlrtErrorWithMessageIdR2018a(&b_st, &h_emlrtRTEI,
                                  "MATLAB:chckxy:NotEnoughPts",
                                  "MATLAB:chckxy:NotEnoughPts", 0);
  }
  if (timeOfArrival->size[1] - 1 == 8) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &i_emlrtRTEI, "Coder:toolbox:OrderOneRequiresColumnVector",
        "Coder:toolbox:OrderOneRequiresColumnVector", 0);
  }
  newsize[0] = 3.0;
  newsize[1] = (real_T)timeOfArrival->size[1] - 1.0;
  newsize[2] = 8.0;
  if (!(3.0 * ((real_T)timeOfArrival->size[1] - 1.0) * 8.0 == 24.0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &j_emlrtRTEI,
                                  "Coder:toolbox:MKPPSizeMismatch",
                                  "Coder:toolbox:MKPPSizeMismatch", 0);
  }
  c_st.site = &tg_emlrtRSI;
  d_st.site = &ug_emlrtRSI;
  assertValidSizeArg(&d_st, newsize);
  if (timeOfArrival->size[1] - 1 > 24) {
    emlrtErrorWithMessageIdR2018a(&c_st, &k_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((3 * (timeOfArrival->size[1] - 1)) << 3 != 24) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &l_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  st.site = &tc_emlrtRSI;
  i = timeOfArrival->size[1];
  linspace(timeOfArrival_data[0],
           timeOfArrival_data[timeOfArrival->size[1] - 1], tSamples);
  emxInit_real_T(&st, &a__1, 2, &ic_emlrtRTEI);
  b_st.site = &vg_emlrtRSI;
  histcounts(&b_st, tSamples, timeOfArrival, a__1, segmentNum);
  emxFree_real_T(&st, &a__1);
  for (k = 0; k < 3; k++) {
    for (kk = 0; kk < 101; kk++) {
      real_T polydCoef[7];
      real_T d;
      real_T delT;
      real_T v;
      int32_T q_tmp;
      d = segmentNum[kk];
      if (((int32_T)d < 1) || ((int32_T)d > i)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, i, &w_emlrtBCI, &st);
      }
      delT = tSamples[kk] - timeOfArrival_data[(int32_T)d - 1];
      if ((int32_T)d > 1) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 1, &n_emlrtBCI, &st);
      }
      if (((int32_T)(d + 1.0) < 1) || ((int32_T)(d + 1.0) > i)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(d + 1.0), 1, i, &l_emlrtBCI,
                                      &st);
      }
      if ((int32_T)d > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, i, &m_emlrtBCI, &st);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        waypoints_tmp = k << 3;
        v = ppMatrix[waypoints_tmp];
        for (ic = 0; ic < 7; ic++) {
          v = delT * v + ppMatrix[(ic + waypoints_tmp) + 1];
        }
      }
      q_tmp = k + 3 * kk;
      q[q_tmp] = v;
      for (ic = 0; ic < 7; ic++) {
        polydCoef[ic] = 0.0;
      }
      b_st.site = &wg_emlrtRSI;
      polyder(&b_st, &ppMatrix[k << 3], derivative_data, derivative_size);
      waypoints_tmp = derivative_size[1];
      if (8 - derivative_size[1] > 7) {
        ic = -1;
        i1 = -1;
      } else {
        if (8 - derivative_size[1] > 7) {
          emlrtDynamicBoundsCheckR2012b(8, 1, 7, &q_emlrtBCI, &st);
        }
        ic = 6 - derivative_size[1];
        i1 = 6;
      }
      i1 -= ic;
      if (i1 != derivative_size[1]) {
        emlrtSubAssignSizeCheck1dR2017a(i1, derivative_size[1], &l_emlrtECI,
                                        &st);
      }
      for (i1 = 0; i1 < waypoints_tmp; i1++) {
        polydCoef[(ic + i1) + 1] = derivative_data[i1];
      }
      if ((int32_T)(d + 1.0) > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(d + 1.0), 1, i, &o_emlrtBCI,
                                      &st);
      }
      if ((int32_T)d > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, i, &p_emlrtBCI, &st);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = polydCoef[0];
        for (ic = 0; ic < 6; ic++) {
          v = delT * v + polydCoef[ic + 1];
        }
      }
      qd[q_tmp] = v;
      for (ic = 0; ic < 6; ic++) {
        polyddCoef[ic] = 0.0;
      }
      b_st.site = &xg_emlrtRSI;
      b_polyder(&b_st, polydCoef, derivative_data, derivative_size);
      waypoints_tmp = derivative_size[1];
      if (7 - derivative_size[1] > 6) {
        ic = -1;
        i1 = -1;
      } else {
        if (7 - derivative_size[1] > 6) {
          emlrtDynamicBoundsCheckR2012b(7, 1, 6, &s_emlrtBCI, &st);
        }
        ic = 5 - derivative_size[1];
        i1 = 5;
      }
      i1 -= ic;
      if (i1 != derivative_size[1]) {
        emlrtSubAssignSizeCheck1dR2017a(i1, derivative_size[1], &m_emlrtECI,
                                        &st);
      }
      for (i1 = 0; i1 < waypoints_tmp; i1++) {
        polyddCoef[(ic + i1) + 1] = derivative_data[i1];
      }
      if ((int32_T)d > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, i, &r_emlrtBCI, &st);
      }
      if (muDoubleScalarIsNaN(delT)) {
        v = rtNaN;
      } else {
        v = polyddCoef[0];
        for (ic = 0; ic < 5; ic++) {
          v = delT * v + polyddCoef[ic + 1];
        }
      }
      qdd[q_tmp] = v;
      b_st.site = &yg_emlrtRSI;
      c_polyder(&b_st, polyddCoef, derivative_data, derivative_size);
      if (6 - derivative_size[1] > 5) {
        ic = 1;
        i1 = 0;
      } else {
        if (6 - derivative_size[1] > 5) {
          emlrtDynamicBoundsCheckR2012b(6, 1, 5, &v_emlrtBCI, &st);
        }
        ic = 6 - derivative_size[1];
        i1 = 5;
      }
      ic = (i1 - ic) + 1;
      if (ic != derivative_size[1]) {
        emlrtSubAssignSizeCheck1dR2017a(ic, derivative_size[1], &n_emlrtECI,
                                        &st);
      }
      if ((int32_T)(d + 1.0) > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(d + 1.0), 1, i, &t_emlrtBCI,
                                      &st);
      }
      if ((int32_T)d > i) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, i, &u_emlrtBCI, &st);
      }
    }
  }
  emxFree_real_T(&st, &timeOfArrival);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_minjerkpolytraj(const emlrtStack *sp, real_T q[101], real_T qd[101],
                       real_T qdd[101])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_real_T *a__1;
  emxArray_real_T *c_timeOfArrival;
  real_T segmentNum[101];
  real_T tSamples[101];
  real_T constraints[8];
  real_T ppMatrix[8];
  real_T derivative_data[7];
  real_T newsize[3];
  real_T *timeOfArrival_data;
  int32_T b_timeOfArrival;
  int32_T i;
  int32_T i1;
  int32_T kk;
  int32_T timeOfArrival;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &nc_emlrtRSI;
  b_st.site = &wc_emlrtRSI;
  c_st.site = &gd_emlrtRSI;
  b_st.site = &xc_emlrtRSI;
  b_st.site = &yc_emlrtRSI;
  b_st.site = &ad_emlrtRSI;
  b_st.site = &bd_emlrtRSI;
  b_st.site = &cd_emlrtRSI;
  b_st.site = &dd_emlrtRSI;
  b_st.site = &ed_emlrtRSI;
  b_st.site = &fd_emlrtRSI;
  st.site = &oc_emlrtRSI;
  c_validateTrajConstNameValueInp();
  st.site = &pc_emlrtRSI;
  st.site = &qc_emlrtRSI;
  timeOfArrival = 4;
  b_timeOfArrival = 4;
  emlrtSubAssignSizeCheckR2012b(&b_timeOfArrival, 1, &timeOfArrival, 1,
                                &k_emlrtECI, &st);
  constraints[0] = 0.0;
  constraints[1] = 0.0;
  constraints[2] = 0.0;
  constraints[3] = 0.0;
  b_timeOfArrival = 4;
  emlrtSubAssignSizeCheckR2012b(&b_timeOfArrival, 1, &timeOfArrival, 1,
                                &k_emlrtECI, &st);
  constraints[4] = 1.0;
  constraints[5] = 0.0;
  constraints[6] = 0.0;
  constraints[7] = 0.0;
  emxInit_real_T(sp, &c_timeOfArrival, 2, &ic_emlrtRTEI);
  st.site = &rc_emlrtRSI;
  c_computePolyCoefAndTimeOfArriv(&st, constraints, c_timeOfArrival, ppMatrix);
  timeOfArrival_data = c_timeOfArrival->data;
  st.site = &sc_emlrtRSI;
  b_st.site = &sg_emlrtRSI;
  if (c_timeOfArrival->size[1] < 2) {
    emlrtErrorWithMessageIdR2018a(&b_st, &h_emlrtRTEI,
                                  "MATLAB:chckxy:NotEnoughPts",
                                  "MATLAB:chckxy:NotEnoughPts", 0);
  }
  if (c_timeOfArrival->size[1] - 1 == 8) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &i_emlrtRTEI, "Coder:toolbox:OrderOneRequiresColumnVector",
        "Coder:toolbox:OrderOneRequiresColumnVector", 0);
  }
  newsize[0] = 1.0;
  newsize[1] = (real_T)c_timeOfArrival->size[1] - 1.0;
  newsize[2] = 8.0;
  if (!(((real_T)c_timeOfArrival->size[1] - 1.0) * 8.0 == 8.0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &j_emlrtRTEI,
                                  "Coder:toolbox:MKPPSizeMismatch",
                                  "Coder:toolbox:MKPPSizeMismatch", 0);
  }
  c_st.site = &tg_emlrtRSI;
  d_st.site = &ug_emlrtRSI;
  assertValidSizeArg(&d_st, newsize);
  if (c_timeOfArrival->size[1] - 1 > 8) {
    emlrtErrorWithMessageIdR2018a(&c_st, &k_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((c_timeOfArrival->size[1] - 1) << 3 != 8) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &l_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  st.site = &tc_emlrtRSI;
  b_timeOfArrival = c_timeOfArrival->size[1];
  linspace(timeOfArrival_data[0],
           timeOfArrival_data[c_timeOfArrival->size[1] - 1], tSamples);
  emxInit_real_T(&st, &a__1, 2, &ic_emlrtRTEI);
  b_st.site = &vg_emlrtRSI;
  histcounts(&b_st, tSamples, c_timeOfArrival, a__1, segmentNum);
  emxFree_real_T(&st, &a__1);
  for (kk = 0; kk < 101; kk++) {
    real_T polydCoef[7];
    real_T polyddCoef[6];
    real_T d;
    real_T delT;
    real_T v;
    int32_T derivative_size[2];
    d = segmentNum[kk];
    if (((int32_T)d < 1) || ((int32_T)d > b_timeOfArrival)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, b_timeOfArrival, &w_emlrtBCI,
                                    &st);
    }
    delT = tSamples[kk] - timeOfArrival_data[(int32_T)d - 1];
    if ((int32_T)d > 1) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 1, &n_emlrtBCI, &st);
    }
    if (((int32_T)(d + 1.0) < 1) || ((int32_T)(d + 1.0) > b_timeOfArrival)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(d + 1.0), 1, b_timeOfArrival,
                                    &l_emlrtBCI, &st);
    }
    if ((int32_T)d > b_timeOfArrival) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, b_timeOfArrival, &m_emlrtBCI,
                                    &st);
    }
    if (muDoubleScalarIsNaN(delT)) {
      v = rtNaN;
    } else {
      v = ppMatrix[0];
      for (timeOfArrival = 0; timeOfArrival < 7; timeOfArrival++) {
        v = delT * v + ppMatrix[timeOfArrival + 1];
      }
    }
    q[kk] = v;
    for (i = 0; i < 7; i++) {
      polydCoef[i] = 0.0;
    }
    b_st.site = &wg_emlrtRSI;
    polyder(&b_st, ppMatrix, derivative_data, derivative_size);
    timeOfArrival = derivative_size[1];
    if (8 - derivative_size[1] > 7) {
      i = -1;
      i1 = -1;
    } else {
      if (8 - derivative_size[1] > 7) {
        emlrtDynamicBoundsCheckR2012b(8, 1, 7, &q_emlrtBCI, &st);
      }
      i = 6 - derivative_size[1];
      i1 = 6;
    }
    i1 -= i;
    if (i1 != derivative_size[1]) {
      emlrtSubAssignSizeCheck1dR2017a(i1, derivative_size[1], &l_emlrtECI, &st);
    }
    for (i1 = 0; i1 < timeOfArrival; i1++) {
      polydCoef[(i + i1) + 1] = derivative_data[i1];
    }
    if ((int32_T)(d + 1.0) > b_timeOfArrival) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(d + 1.0), 1, b_timeOfArrival,
                                    &o_emlrtBCI, &st);
    }
    if ((int32_T)d > b_timeOfArrival) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, b_timeOfArrival, &p_emlrtBCI,
                                    &st);
    }
    if (muDoubleScalarIsNaN(delT)) {
      v = rtNaN;
    } else {
      v = polydCoef[0];
      for (timeOfArrival = 0; timeOfArrival < 6; timeOfArrival++) {
        v = delT * v + polydCoef[timeOfArrival + 1];
      }
    }
    qd[kk] = v;
    for (i = 0; i < 6; i++) {
      polyddCoef[i] = 0.0;
    }
    b_st.site = &xg_emlrtRSI;
    b_polyder(&b_st, polydCoef, derivative_data, derivative_size);
    timeOfArrival = derivative_size[1];
    if (7 - derivative_size[1] > 6) {
      i = -1;
      i1 = -1;
    } else {
      if (7 - derivative_size[1] > 6) {
        emlrtDynamicBoundsCheckR2012b(7, 1, 6, &s_emlrtBCI, &st);
      }
      i = 5 - derivative_size[1];
      i1 = 5;
    }
    i1 -= i;
    if (i1 != derivative_size[1]) {
      emlrtSubAssignSizeCheck1dR2017a(i1, derivative_size[1], &m_emlrtECI, &st);
    }
    for (i1 = 0; i1 < timeOfArrival; i1++) {
      polyddCoef[(i + i1) + 1] = derivative_data[i1];
    }
    if ((int32_T)d > b_timeOfArrival) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, b_timeOfArrival, &r_emlrtBCI,
                                    &st);
    }
    if (muDoubleScalarIsNaN(delT)) {
      v = rtNaN;
    } else {
      v = polyddCoef[0];
      for (timeOfArrival = 0; timeOfArrival < 5; timeOfArrival++) {
        v = delT * v + polyddCoef[timeOfArrival + 1];
      }
    }
    qdd[kk] = v;
    b_st.site = &yg_emlrtRSI;
    c_polyder(&b_st, polyddCoef, derivative_data, derivative_size);
    if (6 - derivative_size[1] > 5) {
      i = 1;
      i1 = 0;
    } else {
      if (6 - derivative_size[1] > 5) {
        emlrtDynamicBoundsCheckR2012b(6, 1, 5, &v_emlrtBCI, &st);
      }
      i = 6 - derivative_size[1];
      i1 = 5;
    }
    i = (i1 - i) + 1;
    if (i != derivative_size[1]) {
      emlrtSubAssignSizeCheck1dR2017a(i, derivative_size[1], &n_emlrtECI, &st);
    }
    if ((int32_T)(d + 1.0) > b_timeOfArrival) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(d + 1.0), 1, b_timeOfArrival,
                                    &t_emlrtBCI, &st);
    }
    if ((int32_T)d > b_timeOfArrival) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, b_timeOfArrival, &u_emlrtBCI,
                                    &st);
    }
  }
  emxFree_real_T(&st, &c_timeOfArrival);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (minjerkpolytraj.c) */
