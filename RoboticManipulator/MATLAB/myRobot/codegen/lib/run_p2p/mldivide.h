/*
 * File: mldivide.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef MLDIVIDE_H
#define MLDIVIDE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_mldivide(const double A_data[], const int A_size[2], double B_data[],
                int *B_size);

int mldivide(const double A_data[], const int A_size[2], const double B[6],
             double Y_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for mldivide.h
 *
 * [EOF]
 */
