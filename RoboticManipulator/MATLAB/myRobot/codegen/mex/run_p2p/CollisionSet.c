/*
 * CollisionSet.c
 *
 * Code generation for function 'CollisionSet'
 *
 */

/* Include files */
#include "CollisionSet.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRTEInfo c_emlrtRTEI = {
    62,                          /* lineNo */
    25,                          /* colNo */
    "CollisionSet/CollisionSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m" /* pName */
};

static emlrtBCInfo h_emlrtBCI = {
    -1,                          /* iFirst */
    -1,                          /* iLast */
    63,                          /* lineNo */
    45,                          /* colNo */
    "",                          /* aName */
    "CollisionSet/CollisionSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m", /* pName */
    0                           /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    38,                          /* lineNo */
    65,                          /* colNo */
    "CollisionSet/CollisionSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m", /* pName */
    1                           /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    38,                          /* lineNo */
    65,                          /* colNo */
    "CollisionSet/CollisionSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m", /* pName */
    4                           /* checkKind */
};

static emlrtRTEInfo hc_emlrtRTEI = {
    38,             /* lineNo */
    65,             /* colNo */
    "CollisionSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m" /* pName */
};

/* Function Definitions */
d_robotics_manip_internal_Colli *
CollisionSet_CollisionSet(const emlrtStack *sp,
                          d_robotics_manip_internal_Colli *obj,
                          real_T maxElements)
{
  static const void *t0_GeometryInternal = NULL;
  c_robotics_manip_internal_Colli expl_temp;
  d_robotics_manip_internal_Colli *b_obj;
  real_T d;
  int32_T b_i;
  int32_T i;
  b_obj = obj;
  b_obj->Size = 0.0;
  b_obj->MaxElements = maxElements;
  if (!(b_obj->MaxElements >= 0.0)) {
    emlrtNonNegativeCheckR2012b(b_obj->MaxElements, &f_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  d = b_obj->MaxElements;
  if (d != (int32_T)muDoubleScalarFloor(d)) {
    emlrtIntegerCheckR2012b(d, &e_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = b_obj->CollisionGeometries->size[0] * b_obj->CollisionGeometries->size[1];
  b_obj->CollisionGeometries->size[0] = 1;
  b_obj->CollisionGeometries->size[1] = (int32_T)d;
  c_emxEnsureCapacity_robotics_ma(sp, b_obj->CollisionGeometries, i,
                                  &hc_emlrtRTEI);
  d = b_obj->MaxElements;
  i = (int32_T)d;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                &c_emlrtRTEI, (emlrtConstCTX)sp);
  for (b_i = 0; b_i < i; b_i++) {
    int32_T i1;
    i1 = b_obj->CollisionGeometries->size[1];
    expl_temp.CollisionPrimitive = (void *)t0_GeometryInternal;
    if (b_i > i1 - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, i1 - 1, &h_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    b_obj->CollisionGeometries->data[b_i] = expl_temp;
  }
  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

/* End of code generation (CollisionSet.c) */
