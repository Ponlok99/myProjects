/*
 * File: minjerkpolytraj.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "minjerkpolytraj.h"
#include "histcounts.h"
#include "linspace.h"
#include "polyder.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "solvePoly.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double waypoints[2][3]
 *                double varargin_2[2][3]
 *                double varargin_4[2][3]
 *                double varargin_6[2][3]
 *                double q[101][3]
 *                double qd[101][3]
 *                double qdd[101][3]
 * Return Type  : void
 */
void b_minjerkpolytraj(double waypoints[2][3], double varargin_2[2][3],
                       double varargin_4[2][3], double varargin_6[2][3],
                       double q[101][3], double qd[101][3], double qdd[101][3])
{
  emxArray_real_T *a__1;
  emxArray_real_T *a__2;
  double segmentNum[101];
  double tSamples[101];
  double constraints[3][8];
  double ppMatrix[3][8][1];
  double derivative_data[7];
  double accBC[3][2];
  double b_waypoints[3][2];
  double jerkBC[3][2];
  double polyddCoef[6];
  double *a__2_data;
  int i;
  int i1;
  int k;
  int kk;
  int loop_ub;
  for (i = 0; i < 3; i++) {
    b_waypoints[i][0] = waypoints[0][i];
    b_waypoints[i][1] = waypoints[1][i];
  }
  for (i = 0; i < 6; i++) {
    polyddCoef[i] = (&b_waypoints[0][0])[i];
  }
  for (i = 0; i < 3; i++) {
    b_waypoints[i][0] = varargin_2[0][i];
    accBC[i][0] = varargin_4[0][i];
    jerkBC[i][0] = varargin_6[0][i];
    b_waypoints[i][1] = varargin_2[1][i];
    accBC[i][1] = varargin_4[1][i];
    jerkBC[i][1] = varargin_6[1][i];
    for (i1 = 0; i1 < 8; i1++) {
      constraints[i][i1] = 0.0;
    }
  }
  for (k = 0; k < 2; k++) {
    double b_polyddCoef[3][4];
    i = k << 2;
    i1 = (k + 1) << 2;
    if (i + 1 > i1) {
      i = 0;
      i1 = 0;
    }
    b_polyddCoef[0][0] = polyddCoef[k];
    b_polyddCoef[0][1] = b_waypoints[0][k];
    b_polyddCoef[0][2] = accBC[0][k];
    b_polyddCoef[0][3] = jerkBC[0][k];
    b_polyddCoef[1][0] = polyddCoef[k + 2];
    b_polyddCoef[1][1] = b_waypoints[1][k];
    b_polyddCoef[1][2] = accBC[1][k];
    b_polyddCoef[1][3] = jerkBC[1][k];
    b_polyddCoef[2][0] = polyddCoef[k + 4];
    b_polyddCoef[2][1] = b_waypoints[2][k];
    b_polyddCoef[2][2] = accBC[2][k];
    b_polyddCoef[2][3] = jerkBC[2][k];
    loop_ub = i1 - i;
    for (i1 = 0; i1 < 3; i1++) {
      for (kk = 0; kk < loop_ub; kk++) {
        constraints[i1][i + kk] = (&b_polyddCoef[0][0])[kk + loop_ub * i1];
      }
    }
  }
  solvePoly(&constraints[0][0], &ppMatrix[0][0][0]);
  solvePoly(&constraints[1][0], &ppMatrix[1][0][0]);
  solvePoly(&constraints[2][0], &ppMatrix[2][0][0]);
  linspace(0.0, 1.0, tSamples);
  emxInit_real_T(&a__2, 2);
  i = a__2->size[0] * a__2->size[1];
  a__2->size[0] = 1;
  a__2->size[1] = 2;
  emxEnsureCapacity_real_T(a__2, i);
  a__2_data = a__2->data;
  a__2_data[0] = 0.0;
  a__2_data[1] = 1.0;
  emxInit_real_T(&a__1, 2);
  histcounts(tSamples, a__2, a__1, segmentNum);
  emxFree_real_T(&a__2);
  emxFree_real_T(&a__1);
  for (k = 0; k < 3; k++) {
    for (kk = 0; kk < 101; kk++) {
      double polydCoef[7];
      double delT;
      double v;
      int derivative_size[2];
      bool b;
      delT = tSamples[kk] - (double)(signed char)((int)segmentNum[kk] - 1);
      b = rtIsNaN(delT);
      if (b) {
        v = rtNaN;
      } else {
        v = ppMatrix[k][0][0];
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          v = delT * v + ppMatrix[k][loop_ub + 1][0];
        }
      }
      q[kk][k] = v;
      for (i = 0; i < 7; i++) {
        polydCoef[i] = 0.0;
      }
      polyder(&ppMatrix[k][0][0], derivative_data, derivative_size);
      loop_ub = derivative_size[1];
      if (8 - derivative_size[1] > 7) {
        i = 0;
      } else {
        i = 7 - derivative_size[1];
      }
      for (i1 = 0; i1 < loop_ub; i1++) {
        polydCoef[i + i1] = derivative_data[i1];
      }
      if (b) {
        v = rtNaN;
      } else {
        v = polydCoef[0];
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          v = delT * v + polydCoef[loop_ub + 1];
        }
      }
      qd[kk][k] = v;
      for (i = 0; i < 6; i++) {
        polyddCoef[i] = 0.0;
      }
      b_polyder(polydCoef, derivative_data, derivative_size);
      loop_ub = derivative_size[1];
      if (7 - derivative_size[1] > 6) {
        i = 0;
      } else {
        i = 6 - derivative_size[1];
      }
      for (i1 = 0; i1 < loop_ub; i1++) {
        polyddCoef[i + i1] = derivative_data[i1];
      }
      if (b) {
        v = rtNaN;
      } else {
        v = polyddCoef[0];
        for (loop_ub = 0; loop_ub < 5; loop_ub++) {
          v = delT * v + polyddCoef[loop_ub + 1];
        }
      }
      qdd[kk][k] = v;
    }
  }
}

/*
 * Arguments    : double q[101]
 *                double qd[101]
 *                double qdd[101]
 * Return Type  : void
 */
void c_minjerkpolytraj(double q[101], double qd[101], double qdd[101])
{
  emxArray_real_T *a__1;
  emxArray_real_T *a__2;
  double segmentNum[101];
  double tSamples[101];
  double constraints[8];
  double ppMatrix[8];
  double derivative_data[7];
  double *a__2_data;
  int i;
  int i1;
  int ic;
  int kk;
  constraints[0] = 0.0;
  constraints[1] = 0.0;
  constraints[2] = 0.0;
  constraints[3] = 0.0;
  constraints[4] = 1.0;
  constraints[5] = 0.0;
  constraints[6] = 0.0;
  constraints[7] = 0.0;
  solvePoly(constraints, ppMatrix);
  linspace(0.0, 1.0, tSamples);
  emxInit_real_T(&a__2, 2);
  i = a__2->size[0] * a__2->size[1];
  a__2->size[0] = 1;
  a__2->size[1] = 2;
  emxEnsureCapacity_real_T(a__2, i);
  a__2_data = a__2->data;
  a__2_data[0] = 0.0;
  a__2_data[1] = 1.0;
  emxInit_real_T(&a__1, 2);
  histcounts(tSamples, a__2, a__1, segmentNum);
  emxFree_real_T(&a__2);
  emxFree_real_T(&a__1);
  for (kk = 0; kk < 101; kk++) {
    double polydCoef[7];
    double polyddCoef[6];
    double delT;
    double v;
    int derivative_size[2];
    bool b;
    delT = tSamples[kk] - (double)(signed char)((int)segmentNum[kk] - 1);
    b = rtIsNaN(delT);
    if (b) {
      v = rtNaN;
    } else {
      v = ppMatrix[0];
      for (ic = 0; ic < 7; ic++) {
        v = delT * v + ppMatrix[ic + 1];
      }
    }
    q[kk] = v;
    for (i = 0; i < 7; i++) {
      polydCoef[i] = 0.0;
    }
    polyder(ppMatrix, derivative_data, derivative_size);
    ic = derivative_size[1];
    if (8 - derivative_size[1] > 7) {
      i = 0;
    } else {
      i = 7 - derivative_size[1];
    }
    for (i1 = 0; i1 < ic; i1++) {
      polydCoef[i + i1] = derivative_data[i1];
    }
    if (b) {
      v = rtNaN;
    } else {
      v = polydCoef[0];
      for (ic = 0; ic < 6; ic++) {
        v = delT * v + polydCoef[ic + 1];
      }
    }
    qd[kk] = v;
    for (i = 0; i < 6; i++) {
      polyddCoef[i] = 0.0;
    }
    b_polyder(polydCoef, derivative_data, derivative_size);
    ic = derivative_size[1];
    if (7 - derivative_size[1] > 6) {
      i = 0;
    } else {
      i = 6 - derivative_size[1];
    }
    for (i1 = 0; i1 < ic; i1++) {
      polyddCoef[i + i1] = derivative_data[i1];
    }
    if (b) {
      v = rtNaN;
    } else {
      v = polyddCoef[0];
      for (ic = 0; ic < 5; ic++) {
        v = delT * v + polyddCoef[ic + 1];
      }
    }
    qdd[kk] = v;
  }
}

/*
 * File trailer for minjerkpolytraj.c
 *
 * [EOF]
 */
