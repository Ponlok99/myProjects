/*
 * File: xgeqp3.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef XGEQP3_H
#define XGEQP3_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int b_xgeqp3(double A_data[], const int A_size[2], double tau_data[],
             int jpvt_data[], int jpvt_size[2]);

void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T *jpvt);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgeqp3.h
 *
 * [EOF]
 */
