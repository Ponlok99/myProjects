/*
 * slerp.h
 *
 * Code generation for function 'slerp'
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
quaternion quaternionBase_slerp(const emlrtStack *sp, const quaternion q1,
                                const quaternion q2, real_T t);

/* End of code generation (slerp.h) */
