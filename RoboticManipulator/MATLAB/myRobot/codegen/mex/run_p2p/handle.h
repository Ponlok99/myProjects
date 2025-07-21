/*
 * handle.h
 *
 * Code generation for function 'handle'
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
void b_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     c_robotics_manip_internal_Rigid *obj);

void c_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *obj);

void d_handle_matlabCodegenDestructo(const emlrtStack *sp, rigidBodyTree *obj);

void handle_matlabCodegenDestructor(const emlrtStack *sp,
                                    d_robotics_manip_internal_Rigid *obj);

/* End of code generation (handle.h) */
