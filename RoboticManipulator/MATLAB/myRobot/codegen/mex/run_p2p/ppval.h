/*
 * ppval.h
 *
 * Code generation for function 'ppval'
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
void b_ppval(const real_T pp_breaks[4], const real_T pp_coefs[54],
             real_T v[303]);

void c_ppval(const real_T pp_breaks[4], const real_T pp_coefs[36],
             real_T v[303]);

void ppval(const real_T pp_breaks[6], const real_T pp_coefs_data[],
           const int32_T pp_coefs_size[3], const real_T x[101], real_T v_data[],
           int32_T v_size[2]);

/* End of code generation (ppval.h) */
