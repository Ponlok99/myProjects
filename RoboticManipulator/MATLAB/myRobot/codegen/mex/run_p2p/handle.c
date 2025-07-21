/*
 * handle.c
 *
 * Code generation for function 'handle'
 *
 */

/* Include files */
#include "handle.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "collisioncodegen_api.hpp"

/* Variable Definitions */
static emlrtRSInfo nv_emlrtRSI = {
    22,                                            /* lineNo */
    "matlabCodegenHandle/matlabCodegenDestructor", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\matlabCodegenHandle.m" /* pathName */
};

static emlrtRSInfo ov_emlrtRSI = {
    296,                   /* lineNo */
    "CollisionSet/delete", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m" /* pathName */
};

static emlrtRTEInfo bc_emlrtRTEI = {
    295,                   /* lineNo */
    25,                    /* colNo */
    "CollisionSet/delete", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m" /* pName */
};

static emlrtBCInfo qe_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    296,                   /* lineNo */
    79,                    /* colNo */
    "",                    /* aName */
    "CollisionSet/delete", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo re_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    296,                   /* lineNo */
    45,                    /* colNo */
    "",                    /* aName */
    "CollisionSet/delete", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CollisionSet.m", /* pName */
    0                           /* checkKind */
};

/* Function Definitions */
void b_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void c_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *obj)
{
  c_robotics_manip_internal_Colli b_obj;
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (!obj->matlabCodegenIsDeleted) {
    real_T d;
    int32_T i;
    obj->matlabCodegenIsDeleted = true;
    st.site = &nv_emlrtRSI;
    d = obj->Size;
    i = (int32_T)d;
    emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                  &bc_emlrtRTEI, &st);
    for (b_i = 0; b_i < i; b_i++) {
      int32_T i1;
      b_st.site = &ov_emlrtRSI;
      i1 = obj->CollisionGeometries->size[1] - 1;
      if (b_i > i1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, i1, &qe_emlrtBCI, &b_st);
      }
      b_obj = obj->CollisionGeometries->data[b_i];
      collisioncodegen_destructGeometry(&b_obj.CollisionPrimitive);
      i1 = obj->CollisionGeometries->size[1] - 1;
      if (b_i > i1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, i1, &re_emlrtBCI, &st);
      }
      obj->CollisionGeometries->data[b_i] = b_obj;
    }
  }
}

void d_handle_matlabCodegenDestructo(const emlrtStack *sp, rigidBodyTree *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void handle_matlabCodegenDestructor(const emlrtStack *sp,
                                    d_robotics_manip_internal_Rigid *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* End of code generation (handle.c) */
