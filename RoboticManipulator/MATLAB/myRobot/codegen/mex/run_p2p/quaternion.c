/*
 * quaternion.c
 *
 * Code generation for function 'quaternion'
 *
 */

/* Include files */
#include "quaternion.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"

/* Variable Definitions */
static emlrtECInfo jb_emlrtECI = {
    -1,                         /* nDims */
    175,                        /* lineNo */
    13,                         /* colNo */
    "quaternioncg/parenAssign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\+coder\\@quat"
    "ernioncg\\quaternioncg.m" /* pName */
};

static emlrtECInfo kb_emlrtECI = {
    -1,                         /* nDims */
    176,                        /* lineNo */
    13,                         /* colNo */
    "quaternioncg/parenAssign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\+coder\\@quat"
    "ernioncg\\quaternioncg.m" /* pName */
};

static emlrtECInfo lb_emlrtECI = {
    -1,                         /* nDims */
    177,                        /* lineNo */
    13,                         /* colNo */
    "quaternioncg/parenAssign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\+coder\\@quat"
    "ernioncg\\quaternioncg.m" /* pName */
};

static emlrtECInfo mb_emlrtECI = {
    -1,                         /* nDims */
    178,                        /* lineNo */
    13,                         /* colNo */
    "quaternioncg/parenAssign", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\+coder\\@quat"
    "ernioncg\\quaternioncg.m" /* pName */
};

/* Function Definitions */
void quaternion_parenAssign(
    const emlrtStack *sp, quaternion *obj, const real_T rhs_a_data[],
    const int32_T rhs_a_size[2], const real_T rhs_b_data[],
    const int32_T rhs_b_size[2], const real_T rhs_c_data[],
    const int32_T rhs_c_size[2], const real_T rhs_d_data[],
    const int32_T rhs_d_size[2])
{
  if (rhs_a_size[1] != 1) {
    emlrtSubAssignSizeCheck1dR2017a(1, rhs_a_size[1], &jb_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  obj->a = rhs_a_data[0];
  if (rhs_b_size[1] != 1) {
    emlrtSubAssignSizeCheck1dR2017a(1, rhs_b_size[1], &kb_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  obj->b = rhs_b_data[0];
  if (rhs_c_size[1] != 1) {
    emlrtSubAssignSizeCheck1dR2017a(1, rhs_c_size[1], &lb_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  obj->c = rhs_c_data[0];
  if (rhs_d_size[1] != 1) {
    emlrtSubAssignSizeCheck1dR2017a(1, rhs_d_size[1], &mb_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  obj->d = rhs_d_data[0];
}

void quaternion_parenReference(real_T obj_a, real_T obj_b, real_T obj_c,
                               real_T obj_d, real_T o_a_data[],
                               int32_T o_a_size[2], real_T o_b_data[],
                               int32_T o_b_size[2], real_T o_c_data[],
                               int32_T o_c_size[2], real_T o_d_data[],
                               int32_T o_d_size[2])
{
  o_a_size[0] = 1;
  o_a_size[1] = 1;
  o_b_size[0] = 1;
  o_a_data[0] = obj_a;
  o_b_size[1] = 1;
  o_c_size[0] = 1;
  o_b_data[0] = obj_b;
  o_c_size[1] = 1;
  o_d_size[0] = 1;
  o_c_data[0] = obj_c;
  o_d_size[1] = 1;
  o_d_data[0] = obj_d;
}

/* End of code generation (quaternion.c) */
