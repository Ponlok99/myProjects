/*
 * run_p2p.c
 *
 * Code generation for function 'run_p2p'
 *
 */

/* Include files */
#include "run_p2p.h"
#include "cubicpolytraj.h"
#include "diag.h"
#include "eul2quat.h"
#include "handle.h"
#include "ikine_myRobot.h"
#include "importrobot.h"
#include "minjerkpolytraj.h"
#include "minsnappolytraj.h"
#include "mldivide.h"
#include "quat2tform.h"
#include "quinticpolytraj.h"
#include "rigidBodyTree.h"
#include "rottraj.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "svd.h"
#include "trapveltraj.h"
#include "trvec2tform.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    4,                                       /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    42,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    47,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    52,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    56,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    59,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    69,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    72,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    75,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    77,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    78,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    79,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    80,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    81,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    82,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    83,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    89,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    3,                                       /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

static emlrtRSInfo dl_emlrtRSI =
    {
        30,           /* lineNo */
        "quat2tform", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutils\\quat2tfo"
        "rm.m" /* pathName */
};

static emlrtRSInfo iv_emlrtRSI = {
    15,    /* lineNo */
    "min", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\min.m" /* pathName
                                                                        */
};

static emlrtRSInfo jv_emlrtRSI =
    {
        66,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo kv_emlrtRSI =
    {
        112,       /* lineNo */
        "minimum", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtMCInfo emlrtMCI = {
    85,                                      /* lineNo */
    13,                                      /* colNo */
    "run_p2p",                               /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pName */
};

static emlrtRTEInfo emlrtRTEI = {
    198,             /* lineNo */
    27,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    -1,                                      /* nDims */
    81,                                      /* lineNo */
    9,                                       /* colNo */
    "run_p2p",                               /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    -1,                                      /* nDims */
    87,                                      /* lineNo */
    9,                                       /* colNo */
    "run_p2p",                               /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    -1,                                       /* iFirst */
    -1,                                       /* iLast */
    87,                                       /* lineNo */
    28,                                       /* colNo */
    "q0",                                     /* aName */
    "run_p2p",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m", /* pName */
    0                                         /* checkKind */
};

static emlrtECInfo c_emlrtECI = {
    -1,                                      /* nDims */
    80,                                      /* lineNo */
    9,                                       /* colNo */
    "run_p2p",                               /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pName */
};

static emlrtECInfo d_emlrtECI = {
    -1,                                      /* nDims */
    79,                                      /* lineNo */
    9,                                       /* colNo */
    "run_p2p",                               /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                                       /* iFirst */
    -1,                                       /* iLast */
    81,                                       /* lineNo */
    52,                                       /* colNo */
    "q0",                                     /* aName */
    "run_p2p",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m", /* pName */
    0                                         /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                                       /* iFirst */
    -1,                                       /* iLast */
    78,                                       /* lineNo */
    48,                                       /* colNo */
    "q0",                                     /* aName */
    "run_p2p",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m", /* pName */
    0                                         /* checkKind */
};

static emlrtRTEInfo gc_emlrtRTEI = {
    1,                                       /* lineNo */
    48,                                      /* colNo */
    "run_p2p",                               /* fName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pName */
};

static emlrtRSInfo qv_emlrtRSI = {
    85,                                      /* lineNo */
    "run_p2p",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\run_p2p.m" /* pathName */
};

/* Function Declarations */
static void disp(const emlrtStack *sp, const mxArray *m, emlrtMCInfo *location);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const char_T u[30]);

/* Function Definitions */
static void disp(const emlrtStack *sp, const mxArray *m, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = m;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 1, &pArray, "disp", true,
                        location);
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const char_T u[30])
{
  static const int32_T b_iv[2] = {1, 30};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 30, m, &u[0]);
  emlrtAssign(&y, m);
  return y;
}

void run_p2p(run_p2pStackData *SD, const emlrtStack *sp, const real_T wpts[6],
             const real_T orientations[6], p2pMode runP2pMode,
             trajMode runTrajMode, const real_T waypointVels[6],
             const real_T waypointAccels[6], const real_T waypointJerks[6],
             real_T pose[303], real_T qInterp[606], real_T qdInterp[606],
             real_T jointTorq[606])
{
  static const char_T b_cv[30] = {'T', 'h', 'e', ' ', 'r', 'o', 'b', 'o',
                                  't', ' ', 'i', 's', ' ', 'a', 't', ' ',
                                  'a', ' ', 's', 'i', 'n', 'g', 'u', 'l',
                                  'a', 'r', 'i', 't', 'y', '.'};
  d_robotics_manip_internal_Rigid lobj_1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  rigidBodyTree robot;
  rigidBodyTree *r;
  real_T a__2_data[2401];
  real_T qddInterp[606];
  real_T R[404];
  real_T alpha[303];
  real_T b_min_singular_value[303];
  real_T omega[303];
  real_T p[303];
  real_T pd[303];
  real_T pdd[303];
  real_T S_data[294];
  real_T jacobian_data[294];
  real_T q0_data[192];
  real_T min_singular_value[101];
  real_T sd[101];
  real_T sdd[101];
  real_T tmp_data[49];
  real_T b_tmp_data[6];
  int32_T a__2_size[2];
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T b_omega_tmp;
  int32_T b_tmp_size;
  int32_T i;
  int32_T omega_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  c_emxInitStruct_robotics_manip_(sp, &lobj_1, &gc_emlrtRTEI);
  for (i = 0; i < 22; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_1._pobj0[i],
                                      (void *)&c_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 22; i++) {
    lobj_1._pobj0[i].matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 14; i++) {
    emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_1._pobj1[i],
                                      (void *)&b_handle_matlabCodegenDestructo,
                                      NULL, NULL, NULL);
  }
  for (i = 0; i < 14; i++) {
    lobj_1._pobj1[i].matlabCodegenIsDeleted = true;
  }
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_1.Base,
                                    (void *)&b_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  lobj_1.Base.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &lobj_1,
                                    (void *)&handle_matlabCodegenDestructor,
                                    NULL, NULL, NULL);
  lobj_1.matlabCodegenIsDeleted = true;
  emlrtPushHeapReferenceStackR2021a((emlrtCTX)sp, true, &robot,
                                    (void *)&d_handle_matlabCodegenDestructo,
                                    NULL, NULL, NULL);
  robot.matlabCodegenIsDeleted = true;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 0U);
  /*  Load robot URDF */
  r = &robot;
  st.site = &r_emlrtRSI;
  importrobot(SD, &st, &lobj_1, &r);
  st.site = &emlrtRSI;
  rigidBodyTree_set_Gravity(&robot);
  /*  Define trajectory parameters */
  /*  Time vector */
  /*  Parse optional inputs */
  covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 0, false);
  covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 1, false);
  covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 2, false);
  covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 3, false);
  covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 4, false);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 6U);
  /*     %% */
  /*  Trajectory following loop */
  /*     %% Initialize arrays */
  memset(&qInterp[0], 0, 606U * sizeof(real_T));
  /*  Get the initial and final rotations and times for the segment */
  switch (runTrajMode) {
  case minjerkpolytraj:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 7U);
    st.site = &b_emlrtRSI;
    b_minjerkpolytraj(&st, wpts, waypointVels, waypointAccels, waypointJerks, p,
                      pd, pdd);
    break;
  case minsnappolytraj:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 0, 2);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 8U);
    st.site = &c_emlrtRSI;
    b_minsnappolytraj(&st, wpts, waypointVels, waypointAccels, waypointJerks, p,
                      pd, pdd);
    break;
  case quinticpolytraj:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 0, 3);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 9U);
    st.site = &d_emlrtRSI;
    b_quinticpolytraj(&st, wpts, waypointVels, waypointAccels, p, pd, pdd);
    break;
  case cubicpolytraj:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 0, 4);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 10U);
    st.site = &e_emlrtRSI;
    b_cubicpolytraj(&st, wpts, waypointVels, p, pd, pdd);
    break;
  default:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 0, 0);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 11U);
    st.site = &f_emlrtRSI;
    b_trapveltraj(&st, wpts, p, pd, pdd);
    break;
  }
  switch (runP2pMode) {
  case moveL:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 1, 1);
    break;
  case moveJ:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 1, 2);
    break;
  case moveJI:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 1, 3);
    break;
  default:
    covrtLogSwitch(&emlrtCoverageInstance, 0U, 0U, 1, 0);
    break;
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 12U);
  /*  Find the quaternions from trajectory generation */
  st.site = &g_emlrtRSI;
  c_minjerkpolytraj(&st, min_singular_value, sd, sdd);
  /*  [s, sd, sdd] = quinticpolytraj([0 1], [0 1], tvec); */
  for (i = 0; i < 101; i++) {
    b_min_singular_value[3 * i] = min_singular_value[i];
    b_min_singular_value[3 * i + 1] = sd[i];
    b_min_singular_value[3 * i + 2] = sdd[i];
  }
  real_T dv[4];
  real_T dv1[4];
  eul2quat(&orientations[0], dv);
  eul2quat(&orientations[3], dv1);
  st.site = &h_emlrtRSI;
  rottraj(&st, dv, dv1, b_min_singular_value, R, omega, alpha);
  for (b_i = 0; b_i < 101; b_i++) {
    real_T b[16];
    real_T dv2[16];
    real_T tfInterp[16];
    real_T b_omega[6];
    real_T d;
    real_T ex;
    int32_T jacobian_size[2];
    int32_T q0_size[2];
    int32_T idx;
    covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 13U);
    st.site = &i_emlrtRSI;
    b_st.site = &dl_emlrtRSI;
    quat2tform(&b_st, &R[b_i << 2], b);
    trvec2tform(&p[3 * b_i], dv2);
    for (i = 0; i < 4; i++) {
      real_T d1;
      real_T d2;
      d = dv2[i];
      ex = dv2[i + 4];
      d1 = dv2[i + 8];
      d2 = dv2[i + 12];
      for (omega_tmp = 0; omega_tmp < 4; omega_tmp++) {
        idx = omega_tmp << 2;
        tfInterp[i + idx] = ((d * b[idx] + ex * b[idx + 1]) + d1 * b[idx + 2]) +
                            d2 * b[idx + 3];
      }
    }
    pose[b_i] = tfInterp[12] / tfInterp[15];
    pose[b_i + 101] = tfInterp[13] / tfInterp[15];
    pose[b_i + 202] = tfInterp[14] / tfInterp[15];
    st.site = &j_emlrtRSI;
    ikine_myRobot(&st, tfInterp, qInterp[6 * b_i + 5], q0_data, q0_size);
    if (q0_size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, q0_size[0], &c_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    a__2_size[0] = 1;
    idx = q0_size[1];
    a__2_size[1] = q0_size[1];
    for (i = 0; i < idx; i++) {
      b_omega[i] = q0_data[q0_size[0] * i];
    }
    st.site = &k_emlrtRSI;
    rigidBodyTree_geometricJacobian(&st, &robot, b_omega, a__2_size,
                                    jacobian_data, jacobian_size);
    b_omega[0] = omega[3 * b_i];
    b_omega[3] = pd[3 * b_i];
    omega_tmp = 3 * b_i + 1;
    b_omega[1] = omega[omega_tmp];
    b_omega[4] = pd[omega_tmp];
    b_omega_tmp = 3 * b_i + 2;
    b_omega[2] = omega[b_omega_tmp];
    b_omega[5] = pd[b_omega_tmp];
    st.site = &l_emlrtRSI;
    b_tmp_size =
        b_mldivide(&st, jacobian_data, jacobian_size, b_omega, tmp_data);
    i = 6;
    emlrtSubAssignSizeCheckR2012b(&i, 1, &b_tmp_size, 1, &d_emlrtECI,
                                  (emlrtCTX)sp);
    for (i = 0; i < 6; i++) {
      qdInterp[i + 6 * b_i] = tmp_data[i];
    }
    b_omega[0] = alpha[3 * b_i];
    b_omega[3] = pdd[3 * b_i];
    b_omega[1] = alpha[omega_tmp];
    b_omega[4] = pdd[omega_tmp];
    b_omega[2] = alpha[b_omega_tmp];
    b_omega[5] = pdd[b_omega_tmp];
    st.site = &m_emlrtRSI;
    b_tmp_size =
        b_mldivide(&st, jacobian_data, jacobian_size, b_omega, tmp_data);
    i = 6;
    emlrtSubAssignSizeCheckR2012b(&i, 1, &b_tmp_size, 1, &c_emlrtECI,
                                  (emlrtCTX)sp);
    for (i = 0; i < 6; i++) {
      qddInterp[i + 6 * b_i] = tmp_data[i];
    }
    if (q0_size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, q0_size[0], &b_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    a__2_size[0] = 1;
    for (i = 0; i < idx; i++) {
      b_omega[i] = q0_data[q0_size[0] * i];
    }
    st.site = &n_emlrtRSI;
    rigidBodyTree_inverseDynamics(&st, &robot, b_omega, a__2_size,
                                  &qdInterp[6 * b_i], &qddInterp[6 * b_i],
                                  b_tmp_data, tmp_size);
    i = 6;
    tmp_size[0] = 1;
    tmp_size[1] = 6;
    emlrtSubAssignSizeCheckR2012b(&i, 1, &tmp_size[0], 2, &emlrtECI,
                                  (emlrtCTX)sp);
    for (i = 0; i < 6; i++) {
      jointTorq[i + 6 * b_i] = b_tmp_data[i];
    }
    real_T a__1[36];
    st.site = &o_emlrtRSI;
    svd(&st, jacobian_data, jacobian_size, a__1, S_data, tmp_size, a__2_data,
        a__2_size);
    st.site = &p_emlrtRSI;
    b_st.site = &p_emlrtRSI;
    omega_tmp = diag(&b_st, S_data, tmp_size, b_omega);
    b_st.site = &iv_emlrtRSI;
    c_st.site = &jv_emlrtRSI;
    d_st.site = &kv_emlrtRSI;
    if (omega_tmp < 1) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }
    if (omega_tmp <= 2) {
      if (omega_tmp == 1) {
        ex = b_omega[0];
      } else if ((b_omega[0] > b_omega[1]) ||
                 (muDoubleScalarIsNaN(b_omega[0]) &&
                  (!muDoubleScalarIsNaN(b_omega[1])))) {
        ex = b_omega[1];
      } else {
        ex = b_omega[0];
      }
    } else {
      if (!muDoubleScalarIsNaN(b_omega[0])) {
        idx = 1;
      } else {
        boolean_T exitg1;
        idx = 0;
        b_omega_tmp = 2;
        exitg1 = false;
        while ((!exitg1) && (b_omega_tmp <= omega_tmp)) {
          if (!muDoubleScalarIsNaN(b_omega[b_omega_tmp - 1])) {
            idx = b_omega_tmp;
            exitg1 = true;
          } else {
            b_omega_tmp++;
          }
        }
      }
      if (idx == 0) {
        ex = b_omega[0];
      } else {
        ex = b_omega[idx - 1];
        i = idx + 1;
        for (b_omega_tmp = i; b_omega_tmp <= omega_tmp; b_omega_tmp++) {
          d = b_omega[b_omega_tmp - 1];
          if (ex > d) {
            ex = d;
          }
        }
      }
    }
    if (covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 5, ex < 0.005)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 14U);
      st.site = &qv_emlrtRSI;
      disp(&st, emlrt_marshallOut(&st, b_cv), &emlrtMCI);
    }
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 15U);
    if (q0_size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, q0_size[0], &emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i = 6;
    tmp_size[0] = 1;
    tmp_size[1] = q0_size[1];
    emlrtSubAssignSizeCheckR2012b(&i, 1, &tmp_size[0], 2, &b_emlrtECI,
                                  (emlrtCTX)sp);
    for (i = 0; i < 6; i++) {
      qInterp[i + 6 * b_i] = q0_data[q0_size[0] * i];
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 0);
  st.site = &q_emlrtRSI;
  d_handle_matlabCodegenDestructo(&st, &robot);
  st.site = &q_emlrtRSI;
  handle_matlabCodegenDestructor(&st, &lobj_1);
  st.site = &q_emlrtRSI;
  b_handle_matlabCodegenDestructo(&st, &lobj_1.Base);
  for (i = 0; i < 14; i++) {
    st.site = &q_emlrtRSI;
    b_handle_matlabCodegenDestructo(&st, &lobj_1._pobj1[i]);
  }
  for (i = 0; i < 22; i++) {
    st.site = &q_emlrtRSI;
    c_handle_matlabCodegenDestructo(&st, &lobj_1._pobj0[i]);
  }
  d_emxFreeStruct_robotics_manip_(sp, &lobj_1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (run_p2p.c) */
