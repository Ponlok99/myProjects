/*
 * importrobot.c
 *
 * Code generation for function 'importrobot'
 *
 */

/* Include files */
#include "importrobot.h"
#include "RigidBody.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo s_emlrtRSI = {
    87,            /* lineNo */
    "importrobot", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\importrobot.m" /* pathName
                                                                           */
};

static emlrtRSInfo u_emlrtRSI = {
    1410,                                  /* lineNo */
    "RigidBodyTree/initVisualizationInfo", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    47,                                    /* lineNo */
    "VisualizationInfo/VisualizationInfo", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\VisualizationInfo.m" /* pathName */
};

static emlrtRSInfo ab_emlrtRSI = {
    111,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    133,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    192,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    182,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    185,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    201,                           /* lineNo */
    "RigidBodyTree/RigidBodyTree", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    1370,                                             /* lineNo */
    "RigidBodyTree/defaultInitializeBodiesCellArray", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo
    kc_emlrtRSI =
        {
            145,                           /* lineNo */
            "rigidBodyTree/rigidBodyTree", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyTre"
            "e.m" /* pathName */
};

static emlrtDCInfo qb_emlrtDCI = {
    70,             /* lineNo */
    17,             /* colNo */
    "randomString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\VisualizationInfo.m", /* pName */
    1                                /* checkKind */
};

static emlrtBCInfo ue_emlrtBCI = {
    1,              /* iFirst */
    62,             /* iLast */
    70,             /* lineNo */
    17,             /* colNo */
    "",             /* aName */
    "randomString", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\VisualizationInfo.m", /* pName */
    0                                /* checkKind */
};

/* Function Definitions */
void importrobot(run_p2pStackData *SD, const emlrtStack *sp,
                 d_robotics_manip_internal_Rigid *iobj_0,
                 rigidBodyTree **iobj_1)
{
  static const char_T b_cv[10] = {'d', 'u', 'm', 'm', 'y',
                                  'b', 'o', 'd', 'y', '1'};
  static const char_T b_cv1[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '2'};
  static const char_T b_cv2[10] = {'d', 'u', 'm', 'm', 'y',
                                   'b', 'o', 'd', 'y', '3'};
  static const char_T cv3[10] = {'d', 'u', 'm', 'm', 'y',
                                 'b', 'o', 'd', 'y', '4'};
  static const char_T cv4[10] = {'d', 'u', 'm', 'm', 'y',
                                 'b', 'o', 'd', 'y', '5'};
  static const char_T cv5[10] = {'d', 'u', 'm', 'm', 'y',
                                 'b', 'o', 'd', 'y', '6'};
  static const char_T cv6[10] = {'d', 'u', 'm', 'm', 'y',
                                 'b', 'o', 'd', 'y', '7'};
  static const int8_T b_iv[14] = {1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, -1};
  __m128d r;
  __m128d r1;
  __m128d r2;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T idx[5];
  real_T dv[2];
  real_T dv1[2];
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &s_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_st.site = &s_emlrtRSI;
  c_st.site = &ab_emlrtRSI;
  d_st.site = &t_emlrtRSI;
  c_st.site = &bb_emlrtRSI;
  d_st.site = &u_emlrtRSI;
  e_st.site = &v_emlrtRSI;
  emlrtRandu(&idx[0], 5);
  r = _mm_loadu_pd(&idx[0]);
  r1 = _mm_set1_pd(62.0);
  _mm_storeu_pd(&dv[0], _mm_mul_pd(r, r1));
  dv1[0] = muDoubleScalarFloor(dv[0]);
  dv1[1] = muDoubleScalarFloor(dv[1]);
  r = _mm_loadu_pd(&dv1[0]);
  r2 = _mm_set1_pd(1.0);
  _mm_storeu_pd(&idx[0], _mm_add_pd(r, r2));
  r = _mm_loadu_pd(&idx[2]);
  _mm_storeu_pd(&dv[0], _mm_mul_pd(r, r1));
  dv1[0] = muDoubleScalarFloor(dv[0]);
  dv1[1] = muDoubleScalarFloor(dv[1]);
  r = _mm_loadu_pd(&dv1[0]);
  _mm_storeu_pd(&idx[2], _mm_add_pd(r, r2));
  idx[4] = muDoubleScalarFloor(idx[4] * 62.0) + 1.0;
  for (i = 0; i < 5; i++) {
    real_T d;
    d = idx[i];
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &qb_emlrtDCI, &e_st);
    }
    if (((int32_T)d < 1) || ((int32_T)d > 62)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 62, &ue_emlrtBCI, &e_st);
    }
  }
  iobj_0->NumBodies = 7.0;
  c_st.site = &db_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[0] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[0], b_cv, &iobj_0->_pobj0[0]);
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[1] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[1], b_cv1, &iobj_0->_pobj0[1]);
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[2] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[2], b_cv2, &iobj_0->_pobj0[2]);
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[3] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[3], cv3, &iobj_0->_pobj0[3]);
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[4] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[4], cv4, &iobj_0->_pobj0[4]);
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[5] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[5], cv5, &iobj_0->_pobj0[5]);
  d_st.site = &gb_emlrtRSI;
  iobj_0->Bodies[6] =
      RigidBody_RigidBody(&d_st, &iobj_0->_pobj1[6], cv6, &iobj_0->_pobj0[6]);
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[0] =
      b_RigidBody_RigidBody(SD, &c_st, &iobj_0->_pobj1[7], &iobj_0->_pobj0[7]);
  iobj_0->Bodies[0]->Index = 1.0;
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[1] =
      c_RigidBody_RigidBody(SD, &c_st, &iobj_0->_pobj1[8], &iobj_0->_pobj0[9]);
  iobj_0->Bodies[1]->Index = 2.0;
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[2] =
      d_RigidBody_RigidBody(SD, &c_st, &iobj_0->_pobj1[9], &iobj_0->_pobj0[11]);
  iobj_0->Bodies[2]->Index = 3.0;
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[3] = e_RigidBody_RigidBody(SD, &c_st, &iobj_0->_pobj1[10],
                                            &iobj_0->_pobj0[13]);
  iobj_0->Bodies[3]->Index = 4.0;
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[4] = f_RigidBody_RigidBody(SD, &c_st, &iobj_0->_pobj1[11],
                                            &iobj_0->_pobj0[15]);
  iobj_0->Bodies[4]->Index = 5.0;
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[5] =
      g_RigidBody_RigidBody(&c_st, &iobj_0->_pobj1[12], &iobj_0->_pobj0[17]);
  iobj_0->Bodies[5]->Index = 6.0;
  c_st.site = &eb_emlrtRSI;
  iobj_0->Bodies[6] =
      h_RigidBody_RigidBody(&c_st, &iobj_0->_pobj1[13], &iobj_0->_pobj0[19]);
  iobj_0->Bodies[6]->Index = 7.0;
  c_st.site = &cb_emlrtRSI;
  iobj_0->Gravity[0] = 0.0;
  iobj_0->Gravity[1] = 0.0;
  iobj_0->Gravity[2] = 0.0;
  iobj_0->PositionNumber = 6.0;
  iobj_0->VelocityNumber = 6.0;
  for (i = 0; i < 14; i++) {
    iobj_0->PositionDoFMap[i] = b_iv[i];
  }
  for (i = 0; i < 14; i++) {
    iobj_0->VelocityDoFMap[i] = b_iv[i];
  }
  c_st.site = &fb_emlrtRSI;
  i_RigidBody_RigidBody(SD, &c_st, &iobj_0->Base, &iobj_0->_pobj0[20]);
  iobj_0->Base.Index = 0.0;
  iobj_0->matlabCodegenIsDeleted = false;
  b_st.site = &kc_emlrtRSI;
  c_st.site = &t_emlrtRSI;
  (*iobj_1)->TreeInternal = iobj_0;
  (*iobj_1)->matlabCodegenIsDeleted = false;
}

/* End of code generation (importrobot.c) */
