/*
 * sortLE.h
 *
 * Code generation for function 'sortLE'
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
boolean_T sortLE(const real_T v_data[], const int32_T v_size[2],
                 const int32_T dir_data[], const int32_T dir_size[2],
                 int32_T idx1, int32_T idx2);

/* End of code generation (sortLE.h) */
