/*
 * RigidBodyTreeUtils.h
 *
 * Code generation for function 'RigidBodyTreeUtils'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
int32_T RigidBodyTreeUtils_distance(const emlrtStack *sp, real_T config1,
                                    const real_T config2_data[],
                                    const int32_T config2_size[2],
                                    real_T dist_data[]);

/* End of code generation (RigidBodyTreeUtils.h) */
