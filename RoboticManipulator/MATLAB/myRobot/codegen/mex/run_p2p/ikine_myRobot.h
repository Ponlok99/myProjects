/*
 * ikine_myRobot.h
 *
 * Code generation for function 'ikine_myRobot'
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
void ikine_myRobot(const emlrtStack *sp, const real_T eeTform[16],
                   real_T referenceConfig, real_T qOpts_data[],
                   int32_T qOpts_size[2]);

/* End of code generation (ikine_myRobot.h) */
