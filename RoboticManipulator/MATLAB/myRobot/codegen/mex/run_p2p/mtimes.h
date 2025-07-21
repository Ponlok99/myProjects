/*
 * mtimes.h
 *
 * Code generation for function 'mtimes'
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
void b_mtimes(const real_T A[100], const real_T B[100], real_T C[100]);

void c_mtimes(const real_T A[100], const real_T B[100], real_T C[100]);

void d_mtimes(const real_T A[100], const real_T B[100], real_T C[100]);

void e_mtimes(const real_T A[36], const real_T B_data[],
              const int32_T B_size[2], real_T C_data[], int32_T C_size[2]);

void f_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B_data[], const int32_T B_size[2], real_T C[6]);

void g_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B_data[], int32_T B_size, real_T C[6]);

int32_T h_mtimes(const real_T A_data[], const int32_T A_size[2],
                 const real_T B[6], real_T C_data[]);

void mtimes(const emlrtStack *sp, const emxArray_real_T *A,
            const emxArray_real_T *B, emxArray_real_T *C);

/* End of code generation (mtimes.h) */
