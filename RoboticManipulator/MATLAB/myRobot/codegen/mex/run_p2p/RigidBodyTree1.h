/*
 * RigidBodyTree1.h
 *
 * Code generation for function 'RigidBodyTree1'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void RigidBodyTree_forwardKinematics(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *obj,
                                     const real_T qvec_data[],
                                     int32_T qvec_size,
                                     cell_wrap_63 Ttree_data[],
                                     int32_T Ttree_size[2]);

/* End of code generation (RigidBodyTree1.h) */
