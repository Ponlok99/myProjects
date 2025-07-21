/*
 * quaternion.h
 *
 * Code generation for function 'quaternion'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "run_p2p_internal_types.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void quaternion_parenAssign(
    const emlrtStack *sp, quaternion *obj, const real_T rhs_a_data[],
    const int32_T rhs_a_size[2], const real_T rhs_b_data[],
    const int32_T rhs_b_size[2], const real_T rhs_c_data[],
    const int32_T rhs_c_size[2], const real_T rhs_d_data[],
    const int32_T rhs_d_size[2]);

void quaternion_parenReference(real_T obj_a, real_T obj_b, real_T obj_c,
                               real_T obj_d, real_T o_a_data[],
                               int32_T o_a_size[2], real_T o_b_data[],
                               int32_T o_b_size[2], real_T o_c_data[],
                               int32_T o_c_size[2], real_T o_d_data[],
                               int32_T o_d_size[2]);

/* End of code generation (quaternion.h) */
