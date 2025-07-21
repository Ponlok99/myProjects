/*
 * rigidBodyJoint.h
 *
 * Code generation for function 'rigidBodyJoint'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void c_rigidBodyJoint_get_MotionSubs(const emlrtStack *sp,
                                     const rigidBodyJoint *obj,
                                     real_T msubspace_data[],
                                     int32_T msubspace_size[2]);

void c_rigidBodyJoint_set_MotionSubs(const emlrtStack *sp, rigidBodyJoint *obj,
                                     const real_T msubspace_data[],
                                     const int32_T msubspace_size[2]);

void c_rigidBodyJoint_transformBodyT(const emlrtStack *sp,
                                     const rigidBodyJoint *obj,
                                     const real_T q_data[], int32_T q_size,
                                     real_T T[16]);

void d_rigidBodyJoint_transformBodyT(const emlrtStack *sp,
                                     const rigidBodyJoint *obj, real_T T[16]);

/* End of code generation (rigidBodyJoint.h) */
