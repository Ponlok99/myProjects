/*
 * RigidBodyTreeDynamics.h
 *
 * Code generation for function 'RigidBodyTreeDynamics'
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
int32_T c_RigidBodyTreeDynamics_inverse(
    const emlrtStack *sp, d_robotics_manip_internal_Rigid *robot,
    const real_T q_data[], int32_T q_size, const real_T qdot_data[],
    const real_T qddot_data[], const real_T fext_data[],
    const int32_T fext_size[2], real_T tau_data[]);

/* End of code generation (RigidBodyTreeDynamics.h) */
