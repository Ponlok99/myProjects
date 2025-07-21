/*
 * rotm2eul.h
 *
 * Code generation for function 'rotm2eul'
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
void b_rotm2eul(const emlrtStack *sp, const real_T R[9], real_T eul[3]);

void rotm2eul(const emlrtStack *sp, const real_T R[9], real_T eul[3]);

/* End of code generation (rotm2eul.h) */
