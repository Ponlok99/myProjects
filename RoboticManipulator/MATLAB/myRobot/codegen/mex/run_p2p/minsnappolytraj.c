/*
 * minsnappolytraj.c
 *
 * Code generation for function 'minsnappolytraj'
 *
 */

/* Include files */
#include "minsnappolytraj.h"
#include "assertValidSizeArg.h"
#include "computePolyCoefAndTimeOfArrival.h"
#include "interpSnapTraj.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "validateTrajConstNameValueInputs.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fh_emlrtRSI =
    {
        139,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo gh_emlrtRSI =
    {
        169,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo hh_emlrtRSI =
    {
        172,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo ih_emlrtRSI =
    {
        178,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo jh_emlrtRSI =
    {
        193,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo kh_emlrtRSI =
    {
        196,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo lh_emlrtRSI =
    {
        201,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo mh_emlrtRSI =
    {
        205,               /* lineNo */
        "minsnappolytraj", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pathName */
};

static emlrtRSInfo nh_emlrtRSI = {
    17,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

static emlrtECInfo o_emlrtECI = {
    -1,                 /* nDims */
    27,                 /* lineNo */
    13,                 /* colNo */
    "orderConstraints", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\orderConstraints.m" /* pName */
};

static emlrtRTEInfo rc_emlrtRTEI =
    {
        1,                 /* lineNo */
        51,                /* colNo */
        "minsnappolytraj", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_"
        "rst\\matlab\\minsnappolytraj.m" /* pName */
};

/* Function Definitions */
void b_minsnappolytraj(const emlrtStack *sp, const real_T waypoints[6],
                       const real_T varargin_2[6], const real_T varargin_4[6],
                       const real_T varargin_6[6], real_T q[303],
                       real_T qd[303], real_T qdd[303])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_real_T *timeOfArrival;
  real_T qddd[303];
  real_T qdddd[303];
  real_T tSamples[101];
  real_T constraints[30];
  real_T ppMatrix[30];
  real_T c_waypoints[15];
  real_T accBC[6];
  real_T b_waypoints[6];
  real_T jerkBC[6];
  real_T velBC[6];
  real_T newsize[3];
  int32_T b_iv[2];
  int32_T b_iv1[2];
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T k;
  int32_T waypoints_tmp;
  int8_T snapBC[6];
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
  st.site = &fh_emlrtRSI;
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
        "MATLAB:minsnappolytraj:expectedFinite", 3, 4, 9, "waypoints");
  }
  st.site = &gh_emlrtRSI;
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
  b_st.site = &nh_emlrtRSI;
  st.site = &hh_emlrtRSI;
  c_validateTrajConstNameValueInp();
  st.site = &ih_emlrtRSI;
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
        &st, &e_emlrtRTEI, "shared_uav_rst:minsnappolytraj:VelocityBCFinite",
        "shared_uav_rst:minsnappolytraj:VelocityBCFinite", 0);
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
        "shared_uav_rst:minsnappolytraj:AccelerationBCFinite",
        "shared_uav_rst:minsnappolytraj:AccelerationBCFinite", 0);
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
        &st, &g_emlrtRTEI, "shared_uav_rst:minsnappolytraj:JerkBCFinite",
        "shared_uav_rst:minsnappolytraj:JerkBCFinite", 0);
  }
  st.site = &jh_emlrtRSI;
  for (i = 0; i < 3; i++) {
    waypoints_tmp = i << 1;
    b_waypoints[waypoints_tmp] = waypoints[i];
    velBC[waypoints_tmp] = varargin_2[i];
    accBC[waypoints_tmp] = varargin_4[i];
    jerkBC[waypoints_tmp] = varargin_6[i];
    snapBC[waypoints_tmp] = 0;
    b_waypoints[waypoints_tmp + 1] = waypoints[i + 3];
    velBC[waypoints_tmp + 1] = varargin_2[i + 3];
    accBC[waypoints_tmp + 1] = varargin_4[i + 3];
    jerkBC[waypoints_tmp + 1] = varargin_6[i + 3];
    snapBC[waypoints_tmp + 1] = 0;
  }
  memset(&constraints[0], 0, 30U * sizeof(real_T));
  b_iv[0] = 5;
  b_iv[1] = 3;
  c_waypoints[4] = 0.0;
  c_waypoints[9] = 0.0;
  for (k = 0; k < 2; k++) {
    i = 5 * k;
    i1 = 5 * (k + 1);
    if (i + 1 > i1) {
      i = 0;
      i1 = 0;
    }
    waypoints_tmp = i1 - i;
    b_iv1[0] = waypoints_tmp;
    b_iv1[1] = 3;
    emlrtSubAssignSizeCheckR2012b(&b_iv1[0], 2, &b_iv[0], 2, &o_emlrtECI, &st);
    c_waypoints[0] = b_waypoints[k];
    c_waypoints[1] = velBC[k];
    c_waypoints[2] = accBC[k];
    c_waypoints[3] = jerkBC[k];
    c_waypoints[5] = b_waypoints[k + 2];
    c_waypoints[6] = velBC[k + 2];
    c_waypoints[7] = accBC[k + 2];
    c_waypoints[8] = jerkBC[k + 2];
    c_waypoints[10] = b_waypoints[k + 4];
    c_waypoints[11] = velBC[k + 4];
    c_waypoints[12] = accBC[k + 4];
    c_waypoints[13] = jerkBC[k + 4];
    c_waypoints[14] = snapBC[k + 4];
    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < waypoints_tmp; i2++) {
        constraints[(i + i2) + 10 * i1] = c_waypoints[i2 + waypoints_tmp * i1];
      }
    }
  }
  emxInit_real_T(sp, &timeOfArrival, 2, &rc_emlrtRTEI);
  st.site = &kh_emlrtRSI;
  b_computePolyCoefAndTimeOfArriv(&st, constraints, ppMatrix, timeOfArrival);
  st.site = &lh_emlrtRSI;
  b_st.site = &sg_emlrtRSI;
  if (timeOfArrival->size[1] < 2) {
    emlrtErrorWithMessageIdR2018a(&b_st, &h_emlrtRTEI,
                                  "MATLAB:chckxy:NotEnoughPts",
                                  "MATLAB:chckxy:NotEnoughPts", 0);
  }
  if (timeOfArrival->size[1] - 1 == 10) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &i_emlrtRTEI, "Coder:toolbox:OrderOneRequiresColumnVector",
        "Coder:toolbox:OrderOneRequiresColumnVector", 0);
  }
  newsize[0] = 3.0;
  newsize[1] = (real_T)timeOfArrival->size[1] - 1.0;
  newsize[2] = 10.0;
  if (!(3.0 * ((real_T)timeOfArrival->size[1] - 1.0) * 10.0 == 30.0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &j_emlrtRTEI,
                                  "Coder:toolbox:MKPPSizeMismatch",
                                  "Coder:toolbox:MKPPSizeMismatch", 0);
  }
  c_st.site = &tg_emlrtRSI;
  d_st.site = &ug_emlrtRSI;
  assertValidSizeArg(&d_st, newsize);
  if (timeOfArrival->size[1] - 1 > 30) {
    emlrtErrorWithMessageIdR2018a(&c_st, &k_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if (3 * (timeOfArrival->size[1] - 1) * 10 != 30) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &l_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  st.site = &mh_emlrtRSI;
  interpSnapTraj(&st, ppMatrix, timeOfArrival, q, qd, qdd, qddd, qdddd,
                 tSamples);
  emxFree_real_T(sp, &timeOfArrival);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (minsnappolytraj.c) */
