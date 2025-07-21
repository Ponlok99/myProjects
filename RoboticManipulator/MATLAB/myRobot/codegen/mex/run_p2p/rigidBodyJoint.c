/*
 * rigidBodyJoint.c
 *
 * Code generation for function 'rigidBodyJoint'
 *
 */

/* Include files */
#include "rigidBodyJoint.h"
#include "normalizeRows.h"
#include "quat2tform.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo
    qb_emlrtRSI =
        {
            279,                                 /* lineNo */
            "rigidBodyJoint/set.MotionSubspace", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    cc_emlrtRSI =
        {
            304,                                 /* lineNo */
            "rigidBodyJoint/get.MotionSubspace", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    at_emlrtRSI =
        {
            614,                                    /* lineNo */
            "rigidBodyJoint/transformBodyToParent", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    bt_emlrtRSI =
        {
            442,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    ct_emlrtRSI =
        {
            444,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    dt_emlrtRSI =
        {
            445,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    et_emlrtRSI =
        {
            447,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    ft_emlrtRSI =
        {
            448,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    gt_emlrtRSI =
        {
            451,                             /* lineNo */
            "rigidBodyJoint/jointTransform", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo
    ht_emlrtRSI =
        {
            312,                            /* lineNo */
            "rigidBodyJoint/get.JointAxis", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo it_emlrtRSI = {
    21,            /* lineNo */
    "axang2tform", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutils\\axang2tform."
    "m" /* pathName */
};

static emlrtRSInfo jt_emlrtRSI = {
    24,            /* lineNo */
    "axang2tform", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutils\\axang2tform."
    "m" /* pathName */
};

static emlrtRSInfo kt_emlrtRSI = {
    21,                      /* lineNo */
    "validateNumericMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\+validation\\validateNumericM"
    "atrix.m" /* pathName */
};

static emlrtRSInfo lt_emlrtRSI = {
    24,            /* lineNo */
    "axang2tform", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\axang2tform.m" /* pathName */
};

static emlrtRSInfo mt_emlrtRSI = {
    37,           /* lineNo */
    "axang2rotm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\axang2rotm.m" /* pathName */
};

static emlrtDCInfo d_emlrtDCI =
    {
        280,                                 /* lineNo */
        48,                                  /* colNo */
        "rigidBodyJoint/set.MotionSubspace", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        1    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI =
    {
        1,                                   /* iFirst */
        6,                                   /* iLast */
        280,                                 /* lineNo */
        48,                                  /* colNo */
        "",                                  /* aName */
        "rigidBodyJoint/set.MotionSubspace", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtECInfo
    i_emlrtECI =
        {
            -1,                                  /* nDims */
            280,                                 /* lineNo */
            17,                                  /* colNo */
            "rigidBodyJoint/set.MotionSubspace", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pName */
};

static emlrtDCInfo h_emlrtDCI =
    {
        305,                                 /* lineNo */
        60,                                  /* colNo */
        "rigidBodyJoint/get.MotionSubspace", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        1    /* checkKind */
};

static emlrtBCInfo k_emlrtBCI =
    {
        1,                                   /* iFirst */
        6,                                   /* iLast */
        305,                                 /* lineNo */
        60,                                  /* colNo */
        "",                                  /* aName */
        "rigidBodyJoint/get.MotionSubspace", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtRTEInfo vb_emlrtRTEI = {
    18,              /* lineNo */
    23,              /* colNo */
    "validatencols", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatencols.m" /* pName */
};

static emlrtBCInfo wc_emlrtBCI =
    {
        -1,                              /* iFirst */
        -1,                              /* iLast */
        451,                             /* lineNo */
        55,                              /* colNo */
        "",                              /* aName */
        "rigidBodyJoint/jointTransform", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtBCInfo pe_emlrtBCI =
    {
        1,                               /* iFirst */
        1,                               /* iLast */
        451,                             /* lineNo */
        55,                              /* colNo */
        "",                              /* aName */
        "rigidBodyJoint/jointTransform", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        0    /* checkKind */
};

/* Function Declarations */
static void rigidBodyJoint_get_JointAxis(const emlrtStack *sp,
                                         const rigidBodyJoint *obj,
                                         real_T ax[3]);

/* Function Definitions */
static void rigidBodyJoint_get_JointAxis(const emlrtStack *sp,
                                         const rigidBodyJoint *obj,
                                         real_T ax[3])
{
  static const char_T b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  emlrtStack b_st;
  emlrtStack st;
  int32_T exitg1;
  int32_T kstr;
  boolean_T b_bool;
  boolean_T guard1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ht_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &pb_emlrtRSI;
  if (obj->TypeInternal.Length < 1.0) {
    kstr = 0;
  } else {
    if (obj->TypeInternal.Length !=
        (int32_T)muDoubleScalarFloor(obj->TypeInternal.Length)) {
      emlrtIntegerCheckR2012b(obj->TypeInternal.Length, &c_emlrtDCI, &b_st);
    }
    if (((int32_T)obj->TypeInternal.Length < 1) ||
        ((int32_T)obj->TypeInternal.Length > 200)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)obj->TypeInternal.Length, 1, 200,
                                    &f_emlrtBCI, &b_st);
    }
    kstr = (int32_T)obj->TypeInternal.Length;
  }
  b_bool = false;
  if (kstr == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (obj->TypeInternal.Vector[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    st.site = &ht_emlrtRSI;
    b_st.site = &pb_emlrtRSI;
    if (obj->TypeInternal.Length < 1.0) {
      kstr = 0;
    } else {
      if (obj->TypeInternal.Length !=
          (int32_T)muDoubleScalarFloor(obj->TypeInternal.Length)) {
        emlrtIntegerCheckR2012b(obj->TypeInternal.Length, &c_emlrtDCI, &b_st);
      }
      if (((int32_T)obj->TypeInternal.Length < 1) ||
          ((int32_T)obj->TypeInternal.Length > 200)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)obj->TypeInternal.Length, 1, 200,
                                      &f_emlrtBCI, &b_st);
      }
      kstr = (int32_T)obj->TypeInternal.Length;
    }
    b_bool = false;
    if (kstr == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (obj->TypeInternal.Vector[kstr] != cv[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = rtNaN;
      ax[1] = rtNaN;
      ax[2] = rtNaN;
    }
  }
  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

void c_rigidBodyJoint_get_MotionSubs(const emlrtStack *sp,
                                     const rigidBodyJoint *obj,
                                     real_T msubspace_data[],
                                     int32_T msubspace_size[2])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T i1;
  int32_T kstr;
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &cc_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &pb_emlrtRSI;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    if (obj->TypeInternal.Length !=
        (int32_T)muDoubleScalarFloor(obj->TypeInternal.Length)) {
      emlrtIntegerCheckR2012b(obj->TypeInternal.Length, &c_emlrtDCI, &b_st);
    }
    if (((int32_T)obj->TypeInternal.Length < 1) ||
        ((int32_T)obj->TypeInternal.Length > 200)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)obj->TypeInternal.Length, 1, 200,
                                    &f_emlrtBCI, &b_st);
    }
    i = (int32_T)obj->TypeInternal.Length;
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (obj->TypeInternal.Vector[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    real_T d;
    d = obj->VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &h_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)d < 1) || ((int32_T)d > 6)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 6, &k_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      kstr = (int32_T)d;
    }
    msubspace_size[0] = 6;
    msubspace_size[1] = kstr;
    for (i = 0; i < kstr; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        int32_T msubspace_data_tmp;
        msubspace_data_tmp = i1 + 6 * i;
        msubspace_data[msubspace_data_tmp] =
            obj->MotionSubspaceInternal[msubspace_data_tmp];
      }
    }
  } else {
    msubspace_size[0] = 6;
    msubspace_size[1] = 1;
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0.0;
    }
  }
}

void c_rigidBodyJoint_set_MotionSubs(const emlrtStack *sp, rigidBodyJoint *obj,
                                     const real_T msubspace_data[],
                                     const int32_T msubspace_size[2])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_iv[2];
  int32_T i;
  int32_T i1;
  int32_T kstr;
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &qb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &pb_emlrtRSI;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    if (obj->TypeInternal.Length !=
        (int32_T)muDoubleScalarFloor(obj->TypeInternal.Length)) {
      emlrtIntegerCheckR2012b(obj->TypeInternal.Length, &c_emlrtDCI, &b_st);
    }
    if (((int32_T)obj->TypeInternal.Length < 1) ||
        ((int32_T)obj->TypeInternal.Length > 200)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)obj->TypeInternal.Length, 1, 200,
                                    &f_emlrtBCI, &b_st);
    }
    i = (int32_T)obj->TypeInternal.Length;
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (obj->TypeInternal.Vector[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    real_T d;
    d = obj->VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      if (d != (int32_T)muDoubleScalarFloor(d)) {
        emlrtIntegerCheckR2012b(d, &d_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)d < 1) || ((int32_T)d > 6)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 6, &g_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      kstr = (int32_T)d;
    }
    b_iv[0] = 6;
    b_iv[1] = kstr;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &msubspace_size[0], 2,
                                  &i_emlrtECI, (emlrtCTX)sp);
    for (i = 0; i < kstr; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        int32_T i2;
        i2 = i1 + 6 * i;
        obj->MotionSubspaceInternal[i2] = msubspace_data[i2];
      }
    }
  } else {
    for (i = 0; i < 6; i++) {
      obj->MotionSubspaceInternal[i] = 0.0;
    }
  }
}

void c_rigidBodyJoint_transformBodyT(const emlrtStack *sp,
                                     const rigidBodyJoint *obj,
                                     const real_T q_data[], int32_T q_size,
                                     real_T T[16])
{
  static const char_T b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b[16];
  real_T b_I[16];
  real_T tempR[9];
  real_T b_tempR_tmp;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  int32_T exitg1;
  int32_T i;
  int32_T i1;
  int32_T kstr;
  boolean_T result;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &at_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_st.site = &bt_emlrtRSI;
  c_st.site = &pb_emlrtRSI;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    if (obj->TypeInternal.Length !=
        (int32_T)muDoubleScalarFloor(obj->TypeInternal.Length)) {
      emlrtIntegerCheckR2012b(obj->TypeInternal.Length, &c_emlrtDCI, &c_st);
    }
    if (((int32_T)obj->TypeInternal.Length < 1) ||
        ((int32_T)obj->TypeInternal.Length > 200)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)obj->TypeInternal.Length, 1, 200,
                                    &f_emlrtBCI, &c_st);
    }
    i = (int32_T)obj->TypeInternal.Length;
  }
  result = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != obj->TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    result = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (cv[kstr] != obj->TypeInternal.Vector[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      kstr = 1;
    } else {
      result = false;
      if (i == 8) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 8) {
            if (b_cv1[kstr] != obj->TypeInternal.Vector[kstr]) {
              exitg1 = 1;
            } else {
              kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        kstr = 2;
      } else {
        kstr = -1;
      }
    }
  }
  switch (kstr) {
  case 0: {
    real_T R[9];
    real_T result_data[4];
    real_T v[3];
    real_T c_tempR_tmp;
    real_T d_tempR_tmp;
    real_T e_tempR_tmp;
    real_T f_tempR_tmp;
    real_T g_tempR_tmp;
    real_T h_tempR_tmp;
    real_T i_tempR_tmp;
    real_T j_tempR_tmp;
    int8_T input_sizes_idx_1;
    b_st.site = &ct_emlrtRSI;
    rigidBodyJoint_get_JointAxis(&b_st, obj, v);
    b_st.site = &dt_emlrtRSI;
    c_st.site = &pd_emlrtRSI;
    d_st.site = &qd_emlrtRSI;
    if ((q_size != 1) && (q_size != 0)) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &p_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    input_sizes_idx_1 = (int8_T)(q_size != 0);
    result_data[0] = v[0];
    result_data[1] = v[1];
    result_data[2] = v[2];
    kstr = input_sizes_idx_1;
    if (kstr - 1 >= 0) {
      result_data[3] = q_data[0];
    }
    b_st.site = &dt_emlrtRSI;
    c_st.site = &it_emlrtRSI;
    d_st.site = &kt_emlrtRSI;
    e_st.site = &vc_emlrtRSI;
    if (input_sizes_idx_1 + 3 != 4) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &vb_emlrtRTEI,
          "Coder:toolbox:ValidateattributesincorrectNumcols",
          "MATLAB:axang2tform:incorrectNumcols", 5, 4, 5, "axang", 6, 4.0);
    }
    c_st.site = &jt_emlrtRSI;
    d_st.site = &lt_emlrtRSI;
    e_st.site = &mt_emlrtRSI;
    normalizeRows(&e_st, &result_data[0], v);
    cth = muDoubleScalarCos(result_data[3]);
    sth = muDoubleScalarSin(result_data[3]);
    c_tempR_tmp = v[0] * v[0] * (1.0 - cth) + cth;
    tempR[0] = c_tempR_tmp;
    d_tempR_tmp = v[0] * v[1] * (1.0 - cth);
    e_tempR_tmp = v[2] * sth;
    f_tempR_tmp = d_tempR_tmp - e_tempR_tmp;
    tempR[1] = f_tempR_tmp;
    g_tempR_tmp = v[0] * v[2] * (1.0 - cth);
    h_tempR_tmp = v[1] * sth;
    i_tempR_tmp = g_tempR_tmp + h_tempR_tmp;
    tempR[2] = i_tempR_tmp;
    d_tempR_tmp += e_tempR_tmp;
    tempR[3] = d_tempR_tmp;
    e_tempR_tmp = v[1] * v[1] * (1.0 - cth) + cth;
    tempR[4] = e_tempR_tmp;
    j_tempR_tmp = v[1] * v[2] * (1.0 - cth);
    tempR_tmp = v[0] * sth;
    b_tempR_tmp = j_tempR_tmp - tempR_tmp;
    tempR[5] = b_tempR_tmp;
    g_tempR_tmp -= h_tempR_tmp;
    tempR[6] = g_tempR_tmp;
    h_tempR_tmp = j_tempR_tmp + tempR_tmp;
    tempR[7] = h_tempR_tmp;
    j_tempR_tmp = v[2] * v[2] * (1.0 - cth) + cth;
    tempR[8] = j_tempR_tmp;
    R[0] = c_tempR_tmp;
    R[1] = f_tempR_tmp;
    R[2] = i_tempR_tmp;
    R[3] = d_tempR_tmp;
    R[4] = e_tempR_tmp;
    R[5] = b_tempR_tmp;
    R[6] = g_tempR_tmp;
    R[7] = h_tempR_tmp;
    R[8] = j_tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[kstr] = tempR[3 * kstr];
      R[kstr + 3] = tempR[3 * kstr + 1];
      R[kstr + 6] = tempR[3 * kstr + 2];
    }
    memset(&b[0], 0, 16U * sizeof(real_T));
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = R[3 * i];
      b[kstr + 1] = R[3 * i + 1];
      b[kstr + 2] = R[3 * i + 2];
    }
    b[15] = 1.0;
  } break;
  case 1: {
    real_T v[3];
    b_st.site = &et_emlrtRSI;
    rigidBodyJoint_get_JointAxis(&b_st, obj, v);
    b_st.site = &ft_emlrtRSI;
    c_st.site = &ke_emlrtRSI;
    if (q_size != 1) {
      if (q_size == 1) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &s_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    memset(&tempR[0], 0, 9U * sizeof(real_T));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = tempR[3 * i];
      b[kstr + 1] = tempR[3 * i + 1];
      b[kstr + 2] = tempR[3 * i + 2];
      b[i + 12] = v[i] * q_data[0];
    }
    b[3] = 0.0;
    b[7] = 0.0;
    b[11] = 0.0;
    b[15] = 1.0;
  } break;
  case 2: {
    real_T b_b[16];
    real_T result_data[4];
    if (q_size < 5) {
      emlrtDynamicBoundsCheckR2012b(5, 1, q_size, &wc_emlrtBCI, &st);
    }
    if (q_size < 6) {
      emlrtDynamicBoundsCheckR2012b(6, 1, 5, &wc_emlrtBCI, &st);
    }
    if (q_size < 7) {
      emlrtDynamicBoundsCheckR2012b(7, 1, 6, &wc_emlrtBCI, &st);
    }
    memset(&b_I[0], 0, 16U * sizeof(real_T));
    b_I[0] = 1.0;
    b_I[5] = 1.0;
    b_I[10] = 1.0;
    b_I[15] = 1.0;
    b_I[12] = q_data[4];
    b_I[13] = q_data[5];
    b_I[14] = q_data[6];
    result_data[0] = q_data[0];
    result_data[1] = q_data[1];
    result_data[2] = q_data[2];
    result_data[3] = q_data[3];
    b_st.site = &gt_emlrtRSI;
    quat2tform(&b_st, result_data, b_b);
    for (i = 0; i < 4; i++) {
      sth = b_I[i];
      tempR_tmp = b_I[i + 4];
      b_tempR_tmp = b_I[i + 8];
      cth = b_I[i + 12];
      for (i1 = 0; i1 < 4; i1++) {
        kstr = i1 << 2;
        b[i + kstr] = ((sth * b_b[kstr] + tempR_tmp * b_b[kstr + 1]) +
                       b_tempR_tmp * b_b[kstr + 2]) +
                      cth * b_b[kstr + 3];
      }
    }
  } break;
  default:
    memset(&b[0], 0, 16U * sizeof(real_T));
    b[0] = 1.0;
    b[5] = 1.0;
    b[10] = 1.0;
    b[15] = 1.0;
    break;
  }
  for (i = 0; i < 4; i++) {
    sth = obj->JointToParentTransform[i];
    tempR_tmp = obj->JointToParentTransform[i + 4];
    b_tempR_tmp = obj->JointToParentTransform[i + 8];
    cth = obj->JointToParentTransform[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      kstr = i1 << 2;
      b_I[i + kstr] = ((sth * b[kstr] + tempR_tmp * b[kstr + 1]) +
                       b_tempR_tmp * b[kstr + 2]) +
                      cth * b[kstr + 3];
    }
    sth = b_I[i];
    tempR_tmp = b_I[i + 4];
    b_tempR_tmp = b_I[i + 8];
    cth = b_I[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      kstr = i1 << 2;
      T[i + kstr] = ((sth * obj->ChildToJointTransform[kstr] +
                      tempR_tmp * obj->ChildToJointTransform[kstr + 1]) +
                     b_tempR_tmp * obj->ChildToJointTransform[kstr + 2]) +
                    cth * obj->ChildToJointTransform[kstr + 3];
    }
  }
}

void d_rigidBodyJoint_transformBodyT(const emlrtStack *sp,
                                     const rigidBodyJoint *obj, real_T T[16])
{
  static const char_T b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char_T b_cv1[8] = {'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b[16];
  real_T b_obj[16];
  real_T tempR[9];
  real_T b_tempR_tmp;
  real_T c_tempR_tmp;
  real_T d_tempR_tmp;
  real_T tempR_tmp;
  int32_T exitg1;
  int32_T i;
  int32_T i1;
  int32_T kstr;
  boolean_T result;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &at_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_st.site = &bt_emlrtRSI;
  c_st.site = &pb_emlrtRSI;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    if (obj->TypeInternal.Length !=
        (int32_T)muDoubleScalarFloor(obj->TypeInternal.Length)) {
      emlrtIntegerCheckR2012b(obj->TypeInternal.Length, &c_emlrtDCI, &c_st);
    }
    if (((int32_T)obj->TypeInternal.Length < 1) ||
        ((int32_T)obj->TypeInternal.Length > 200)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)obj->TypeInternal.Length, 1, 200,
                                    &f_emlrtBCI, &c_st);
    }
    i = (int32_T)obj->TypeInternal.Length;
  }
  result = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != obj->TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    result = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (cv[kstr] != obj->TypeInternal.Vector[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      kstr = 1;
    } else {
      result = false;
      if (i == 8) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 8) {
            if (b_cv1[kstr] != obj->TypeInternal.Vector[kstr]) {
              exitg1 = 1;
            } else {
              kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        kstr = 2;
      } else {
        kstr = -1;
      }
    }
  }
  switch (kstr) {
  case 0: {
    real_T R[9];
    real_T b_v[3];
    real_T v[3];
    real_T e_tempR_tmp;
    real_T f_tempR_tmp;
    real_T g_tempR_tmp;
    real_T h_tempR_tmp;
    real_T i_tempR_tmp;
    b_st.site = &ct_emlrtRSI;
    rigidBodyJoint_get_JointAxis(&b_st, obj, v);
    b_st.site = &dt_emlrtRSI;
    c_st.site = &jt_emlrtRSI;
    d_st.site = &lt_emlrtRSI;
    e_st.site = &mt_emlrtRSI;
    normalizeRows(&e_st, v, b_v);
    tempR_tmp = b_v[0] * b_v[0] * 0.0 + 1.0;
    tempR[0] = tempR_tmp;
    b_tempR_tmp = b_v[0] * b_v[1] * 0.0;
    c_tempR_tmp = b_tempR_tmp - b_v[2] * 0.0;
    tempR[1] = c_tempR_tmp;
    d_tempR_tmp = b_v[0] * b_v[2] * 0.0;
    e_tempR_tmp = d_tempR_tmp + b_v[1] * 0.0;
    tempR[2] = e_tempR_tmp;
    b_tempR_tmp += b_v[2] * 0.0;
    tempR[3] = b_tempR_tmp;
    f_tempR_tmp = b_v[1] * b_v[1] * 0.0 + 1.0;
    tempR[4] = f_tempR_tmp;
    g_tempR_tmp = b_v[1] * b_v[2] * 0.0;
    h_tempR_tmp = g_tempR_tmp - b_v[0] * 0.0;
    tempR[5] = h_tempR_tmp;
    d_tempR_tmp -= b_v[1] * 0.0;
    tempR[6] = d_tempR_tmp;
    g_tempR_tmp += b_v[0] * 0.0;
    tempR[7] = g_tempR_tmp;
    i_tempR_tmp = b_v[2] * b_v[2] * 0.0 + 1.0;
    tempR[8] = i_tempR_tmp;
    R[0] = tempR_tmp;
    R[1] = c_tempR_tmp;
    R[2] = e_tempR_tmp;
    R[3] = b_tempR_tmp;
    R[4] = f_tempR_tmp;
    R[5] = h_tempR_tmp;
    R[6] = d_tempR_tmp;
    R[7] = g_tempR_tmp;
    R[8] = i_tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[kstr] = tempR[3 * kstr];
      R[kstr + 3] = tempR[3 * kstr + 1];
      R[kstr + 6] = tempR[3 * kstr + 2];
    }
    memset(&b[0], 0, 16U * sizeof(real_T));
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = R[3 * i];
      b[kstr + 1] = R[3 * i + 1];
      b[kstr + 2] = R[3 * i + 2];
    }
    b[15] = 1.0;
  } break;
  case 1: {
    real_T v[3];
    b_st.site = &et_emlrtRSI;
    rigidBodyJoint_get_JointAxis(&b_st, obj, v);
    memset(&tempR[0], 0, 9U * sizeof(real_T));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = tempR[3 * i];
      b[kstr + 1] = tempR[3 * i + 1];
      b[kstr + 2] = tempR[3 * i + 2];
      b[i + 12] = v[i] * 0.0;
    }
    b[3] = 0.0;
    b[7] = 0.0;
    b[11] = 0.0;
    b[15] = 1.0;
  } break;
  case 2:
    emlrtDynamicBoundsCheckR2012b(5, 1, 1, &pe_emlrtBCI, &st);
    break;
  default:
    memset(&b[0], 0, 16U * sizeof(real_T));
    b[0] = 1.0;
    b[5] = 1.0;
    b[10] = 1.0;
    b[15] = 1.0;
    break;
  }
  for (i = 0; i < 4; i++) {
    tempR_tmp = obj->JointToParentTransform[i];
    b_tempR_tmp = obj->JointToParentTransform[i + 4];
    c_tempR_tmp = obj->JointToParentTransform[i + 8];
    d_tempR_tmp = obj->JointToParentTransform[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      kstr = i1 << 2;
      b_obj[i + kstr] = ((tempR_tmp * b[kstr] + b_tempR_tmp * b[kstr + 1]) +
                         c_tempR_tmp * b[kstr + 2]) +
                        d_tempR_tmp * b[kstr + 3];
    }
    tempR_tmp = b_obj[i];
    b_tempR_tmp = b_obj[i + 4];
    c_tempR_tmp = b_obj[i + 8];
    d_tempR_tmp = b_obj[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      kstr = i1 << 2;
      T[i + kstr] = ((tempR_tmp * obj->ChildToJointTransform[kstr] +
                      b_tempR_tmp * obj->ChildToJointTransform[kstr + 1]) +
                     c_tempR_tmp * obj->ChildToJointTransform[kstr + 2]) +
                    d_tempR_tmp * obj->ChildToJointTransform[kstr + 3];
    }
  }
}

/* End of code generation (rigidBodyJoint.c) */
