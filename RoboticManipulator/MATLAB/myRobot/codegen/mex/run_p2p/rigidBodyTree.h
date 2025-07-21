/*
 * rigidBodyTree.h
 *
 * Code generation for function 'rigidBodyTree'
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
void rigidBodyTree_geometricJacobian(const emlrtStack *sp, rigidBodyTree *obj,
                                     const real_T Q_data[],
                                     const int32_T Q_size[2], real_T Jac_data[],
                                     int32_T Jac_size[2]);

void rigidBodyTree_inverseDynamics(const emlrtStack *sp, rigidBodyTree *obj,
                                   const real_T varargin_1_data[],
                                   const int32_T varargin_1_size[2],
                                   const real_T varargin_2[6],
                                   const real_T varargin_3[6],
                                   real_T tau_data[], int32_T tau_size[2]);

void rigidBodyTree_set_Gravity(rigidBodyTree *obj);

/* End of code generation (rigidBodyTree.h) */
