/*
 * File: minjerkpolytraj.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef MINJERKPOLYTRAJ_H
#define MINJERKPOLYTRAJ_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_minjerkpolytraj(double waypoints[2][3], double varargin_2[2][3],
                       double varargin_4[2][3], double varargin_6[2][3],
                       double q[101][3], double qd[101][3], double qdd[101][3]);

void c_minjerkpolytraj(double q[101], double qd[101], double qdd[101]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for minjerkpolytraj.h
 *
 * [EOF]
 */
