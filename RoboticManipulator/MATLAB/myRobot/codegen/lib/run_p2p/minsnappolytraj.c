/*
 * File: minsnappolytraj.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "minsnappolytraj.h"
#include "computePolyCoefAndTimeOfArrival.h"
#include "histcounts.h"
#include "linspace.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "rt_nonfinite.h"
#include <string.h>

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
void b_minsnappolytraj(double waypoints[2][3], double varargin_2[2][3],
                       double varargin_4[2][3], double varargin_6[2][3],
                       double q[101][3], double qd[101][3], double qdd[101][3])
{
  emxArray_real_T *a__1;
  emxArray_real_T *timeOfArrival;
  double segmentNum[101];
  double tSamples[101];
  double constraints[3][10];
  double ppMatrix[3][10][1];
  double c_waypoints[3][5];
  double polydCoef[9];
  double polyddCoef[8];
  double accBC[3][2];
  double b_waypoints[3][2];
  double jerkBC[3][2];
  double velBC[3][2];
  double *timeOfArrival_data;
  int b_k;
  int i;
  int k;
  int kk;
  int nlead0;
  int ny_tmp_tmp;
  signed char snapBC[3][2];
  for (k = 0; k < 3; k++) {
    b_waypoints[k][0] = waypoints[0][k];
    velBC[k][0] = varargin_2[0][k];
    accBC[k][0] = varargin_4[0][k];
    jerkBC[k][0] = varargin_6[0][k];
    snapBC[k][0] = 0;
    b_waypoints[k][1] = waypoints[1][k];
    velBC[k][1] = varargin_2[1][k];
    accBC[k][1] = varargin_4[1][k];
    jerkBC[k][1] = varargin_6[1][k];
    snapBC[k][1] = 0;
    for (i = 0; i < 10; i++) {
      constraints[k][i] = 0.0;
    }
  }
  c_waypoints[0][4] = 0.0;
  c_waypoints[1][4] = 0.0;
  for (b_k = 0; b_k < 2; b_k++) {
    k = 5 * b_k;
    i = 5 * (b_k + 1);
    if (k + 1 > i) {
      k = 0;
      i = 0;
    }
    c_waypoints[0][0] = b_waypoints[0][b_k];
    c_waypoints[0][1] = velBC[0][b_k];
    c_waypoints[0][2] = accBC[0][b_k];
    c_waypoints[0][3] = jerkBC[0][b_k];
    c_waypoints[1][0] = b_waypoints[1][b_k];
    c_waypoints[1][1] = velBC[1][b_k];
    c_waypoints[1][2] = accBC[1][b_k];
    c_waypoints[1][3] = jerkBC[1][b_k];
    c_waypoints[2][0] = b_waypoints[2][b_k];
    c_waypoints[2][1] = velBC[2][b_k];
    c_waypoints[2][2] = accBC[2][b_k];
    c_waypoints[2][3] = jerkBC[2][b_k];
    c_waypoints[2][4] = snapBC[2][b_k];
    nlead0 = i - k;
    for (i = 0; i < 3; i++) {
      for (ny_tmp_tmp = 0; ny_tmp_tmp < nlead0; ny_tmp_tmp++) {
        constraints[i][k + ny_tmp_tmp] =
            (&c_waypoints[0][0])[ny_tmp_tmp + nlead0 * i];
      }
    }
  }
  emxInit_real_T(&timeOfArrival, 2);
  computePolyCoefAndTimeOfArrival(constraints, ppMatrix, timeOfArrival);
  timeOfArrival_data = timeOfArrival->data;
  linspace(timeOfArrival_data[0],
           timeOfArrival_data[timeOfArrival->size[1] - 1], tSamples);
  emxInit_real_T(&a__1, 2);
  histcounts(tSamples, timeOfArrival, a__1, segmentNum);
  emxFree_real_T(&a__1);
  for (b_k = 0; b_k < 3; b_k++) {
    double d;
    d = ppMatrix[b_k][9][0];
    for (kk = 0; kk < 101; kk++) {
      double derivative_data[9];
      double a_data[8];
      double delT;
      double v;
      bool b;
      delT = tSamples[kk] - timeOfArrival_data[(int)segmentNum[kk] - 1];
      b = rtIsNaN(delT);
      if (b) {
        v = rtNaN;
      } else {
        v = ppMatrix[b_k][0][0];
        for (nlead0 = 0; nlead0 < 9; nlead0++) {
          v = delT * v + ppMatrix[b_k][nlead0 + 1][0];
        }
      }
      q[kk][b_k] = v;
      memset(&polydCoef[0], 0, 9U * sizeof(double));
      nlead0 = 0;
      k = 0;
      while ((k < 8) && (ppMatrix[b_k][k][0] == 0.0)) {
        nlead0++;
        k++;
      }
      ny_tmp_tmp = 9 - nlead0;
      for (k = 0; k < ny_tmp_tmp; k++) {
        derivative_data[k] = ppMatrix[b_k][k + nlead0][0];
      }
      for (k = 0; k <= ny_tmp_tmp - 2; k++) {
        derivative_data[k] *= (double)(8 - (nlead0 + k)) + 1.0;
      }
      if (rtIsInf(d) || rtIsNaN(d)) {
        derivative_data[8 - nlead0] = rtNaN;
      }
      if (nlead0 + 1 > 9) {
        k = 0;
      } else {
        k = nlead0;
      }
      for (i = 0; i < ny_tmp_tmp; i++) {
        polydCoef[k + i] = derivative_data[i];
      }
      if (b) {
        v = rtNaN;
      } else {
        v = polydCoef[0];
        for (nlead0 = 0; nlead0 < 8; nlead0++) {
          v = delT * v + polydCoef[nlead0 + 1];
        }
      }
      qd[kk][b_k] = v;
      memset(&polyddCoef[0], 0, 8U * sizeof(double));
      nlead0 = 0;
      k = 0;
      while ((k < 7) && (polydCoef[k] == 0.0)) {
        nlead0++;
        k++;
      }
      ny_tmp_tmp = 8 - nlead0;
      for (k = 0; k < ny_tmp_tmp; k++) {
        a_data[k] = polydCoef[k + nlead0];
      }
      for (k = 0; k <= ny_tmp_tmp - 2; k++) {
        a_data[k] *= (double)(7 - (nlead0 + k)) + 1.0;
      }
      if (rtIsInf(polydCoef[8]) || rtIsNaN(polydCoef[8])) {
        a_data[7 - nlead0] = rtNaN;
      }
      if (nlead0 + 1 > 8) {
        k = 0;
      } else {
        k = nlead0;
      }
      for (i = 0; i < ny_tmp_tmp; i++) {
        polyddCoef[k + i] = a_data[i];
      }
      if (b) {
        v = rtNaN;
      } else {
        v = polyddCoef[0];
        for (nlead0 = 0; nlead0 < 7; nlead0++) {
          v = delT * v + polyddCoef[nlead0 + 1];
        }
      }
      qdd[kk][b_k] = v;
    }
  }
  emxFree_real_T(&timeOfArrival);
}

/*
 * File trailer for minsnappolytraj.c
 *
 * [EOF]
 */
