/*
 * File: ppval.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef PPVAL_H
#define PPVAL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_ppval(const double pp_breaks[4], double pp_coefs[6][3][3],
             double v[101][3]);

void c_ppval(const double pp_breaks[4], double pp_coefs[4][3][3],
             double v[101][3]);

void ppval(const double pp_breaks[6], const double pp_coefs_data[],
           const int pp_coefs_size[3], const double x[101], double v_data[],
           int v_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ppval.h
 *
 * [EOF]
 */
