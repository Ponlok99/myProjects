/*
 * File: rottraj.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef ROTTRAJ_H
#define ROTTRAJ_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void rottraj(const double R0[4], const double RF[4], double varargin_2[101][3],
             double R[101][4], double omega[101][3], double alpha[101][3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for rottraj.h
 *
 * [EOF]
 */
