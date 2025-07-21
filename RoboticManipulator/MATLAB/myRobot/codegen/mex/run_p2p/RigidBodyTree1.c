/*
 * RigidBodyTree1.c
 *
 * Code generation for function 'RigidBodyTree1'
 *
 */

/* Include files */
#include "RigidBodyTree1.h"
#include "indexShapeCheck.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo qe_emlrtRSI = {
    34,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRSInfo us_emlrtRSI = {
    1462,                              /* lineNo */
    "RigidBodyTree/forwardKinematics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo vs_emlrtRSI = {
    1463,                              /* lineNo */
    "RigidBodyTree/forwardKinematics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ws_emlrtRSI = {
    1469,                              /* lineNo */
    "RigidBodyTree/forwardKinematics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo xs_emlrtRSI = {
    1470,                              /* lineNo */
    "RigidBodyTree/forwardKinematics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRTEInfo n_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtBCInfo qc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1474,                              /* lineNo */
    27,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtRTEInfo tb_emlrtRTEI = {
    1466,                              /* lineNo */
    21,                                /* colNo */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtDCInfo q_emlrtDCI = {
    1469,                              /* lineNo */
    27,                                /* colNo */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo rc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1469,                              /* lineNo */
    27,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo r_emlrtDCI = {
    1469,                              /* lineNo */
    29,                                /* colNo */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo sc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1469,                              /* lineNo */
    29,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo s_emlrtDCI = {
    1474,                              /* lineNo */
    38,                                /* colNo */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo tc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1474,                              /* lineNo */
    38,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo uc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1474,                              /* lineNo */
    62,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo t_emlrtDCI = {
    37,       /* lineNo */
    14,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m", /* pName
                                                                          */
    4 /* checkKind */
};

static emlrtBCInfo vc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1470,                              /* lineNo */
    23,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/forwardKinematics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

/* Function Definitions */
void RigidBodyTree_forwardKinematics(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *obj,
                                     const real_T qvec_data[],
                                     int32_T qvec_size,
                                     cell_wrap_63 Ttree_data[],
                                     int32_T Ttree_size[2])
{
  c_robotics_manip_internal_Rigid *body;
  emlrtStack b_st;
  emlrtStack st;
  real_T a[16];
  real_T b_a[16];
  real_T k;
  real_T n;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T jtilecol;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &us_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  n = obj->NumBodies;
  if (!(n <= 7.0)) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  st.site = &vs_emlrtRSI;
  b_st.site = &qe_emlrtRSI;
  if ((n != muDoubleScalarFloor(n)) || muDoubleScalarIsInf(n) ||
      (n < -2.147483648E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &n_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (!(n >= 0.0)) {
    emlrtNonNegativeCheckR2012b(n, &t_emlrtDCI, &st);
  }
  Ttree_size[0] = 1;
  i = (int32_T)n;
  Ttree_size[1] = (int32_T)n;
  if ((int32_T)n != 0) {
    for (jtilecol = 0; jtilecol < i; jtilecol++) {
      for (i1 = 0; i1 < 16; i1++) {
        Ttree_data[jtilecol].f1[i1] = iv[i1];
      }
    }
  }
  k = 1.0;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, n, mxDOUBLE_CLASS, (int32_T)n,
                                &tb_emlrtRTEI, (emlrtConstCTX)sp);
  if ((int32_T)n - 1 >= 0) {
    b_iv[0] = 1;
  }
  for (b_i = 0; b_i < i; b_i++) {
    real_T b_qvec_data[6];
    real_T pnum;
    body = obj->Bodies[b_i];
    pnum = body->JointInternal.PositionNumber;
    pnum += k;
    if (k > pnum - 1.0) {
      i1 = 0;
      i2 = 0;
    } else {
      if (k != (int32_T)muDoubleScalarFloor(k)) {
        emlrtIntegerCheckR2012b(k, &q_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)k < 1) || ((int32_T)k > qvec_size)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)k, 1, qvec_size, &rc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      i1 = (int32_T)k - 1;
      if (pnum - 1.0 != (int32_T)muDoubleScalarFloor(pnum - 1.0)) {
        emlrtIntegerCheckR2012b(pnum - 1.0, &r_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)(pnum - 1.0) < 1) || ((int32_T)(pnum - 1.0) > qvec_size)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(pnum - 1.0), 1, qvec_size,
                                      &sc_emlrtBCI, (emlrtConstCTX)sp);
      }
      i2 = (int32_T)(pnum - 1.0);
    }
    jtilecol = i2 - i1;
    b_iv[1] = jtilecol;
    st.site = &ws_emlrtRSI;
    indexShapeCheck(&st, qvec_size, b_iv);
    for (i2 = 0; i2 < jtilecol; i2++) {
      b_qvec_data[i2] = qvec_data[i1 + i2];
    }
    if (b_i > (int32_T)n - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)n - 1, &vc_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    st.site = &xs_emlrtRSI;
    c_rigidBodyJoint_transformBodyT(&st, &body->JointInternal, b_qvec_data,
                                    jtilecol, Ttree_data[b_i].f1);
    k = pnum;
    if (body->ParentIndex > 0.0) {
      if (body->ParentIndex !=
          (int32_T)muDoubleScalarFloor(body->ParentIndex)) {
        emlrtIntegerCheckR2012b(body->ParentIndex, &s_emlrtDCI,
                                (emlrtConstCTX)sp);
      }
      i1 = (int32_T)body->ParentIndex - 1;
      if ((i1 < 0) || (i1 > (int32_T)n - 1)) {
        emlrtDynamicBoundsCheckR2012b(i1, 0, (int32_T)n - 1, &tc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      memcpy(&a[0], &Ttree_data[i1].f1[0], 16U * sizeof(real_T));
      if (b_i > (int32_T)n - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)n - 1, &uc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      for (i1 = 0; i1 < 4; i1++) {
        real_T d;
        real_T d1;
        real_T d2;
        pnum = a[i1];
        d = a[i1 + 4];
        d1 = a[i1 + 8];
        d2 = a[i1 + 12];
        for (i2 = 0; i2 < 4; i2++) {
          jtilecol = i2 << 2;
          b_a[i1 + jtilecol] = ((pnum * Ttree_data[b_i].f1[jtilecol] +
                                 d * Ttree_data[b_i].f1[jtilecol + 1]) +
                                d1 * Ttree_data[b_i].f1[jtilecol + 2]) +
                               d2 * Ttree_data[b_i].f1[jtilecol + 3];
        }
      }
      if (b_i > (int32_T)n - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)n - 1, &qc_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      memcpy(&Ttree_data[b_i].f1[0], &b_a[0], 16U * sizeof(real_T));
    }
  }
}

/* End of code generation (RigidBodyTree1.c) */
