/*
 * File: quinticpolytraj.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "quinticpolytraj.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double wayPoints[2][3]
 *                double varargin_2[2][3]
 *                double varargin_4[2][3]
 *                double q[101][3]
 *                double qd[101][3]
 *                double qdd[101][3]
 * Return Type  : void
 */
void b_quinticpolytraj(double wayPoints[2][3], double varargin_2[2][3],
                       double varargin_4[2][3], double q[101][3],
                       double qd[101][3], double qdd[101][3])
{
  static const double invTMatF[3][3] = {
      {10.0, -15.0, 6.0}, {-4.0, 7.0, -3.0}, {0.5, -1.0, 0.5}};
  static const signed char TMat0[3][3] = {{1, 0, 0}, {1, 1, 0}, {1, 2, 2}};
  static const signed char b_iv[6] = {0, 0, 0, 0, 0, 1};
  double dCoeffs[6][9];
  double ddCoeffs[6][9];
  double coefsWithFlatStart[6][6];
  double coefMat[6][3];
  double coeffMat[6][3];
  double derivativeBreaks[4];
  double xtmp;
  int b_i;
  int i;
  int j;
  for (j = 0; j < 3; j++) {
    double coeffVec[6];
    double y[3];
    double wayPoints_idx_1;
    double wayPoints_idx_2;
    coeffVec[0] = wayPoints[0][j];
    coeffVec[1] = varargin_2[0][j];
    coeffVec[2] = varargin_4[0][j] / 2.0;
    coeffVec[3] = 0.0;
    coeffVec[4] = 0.0;
    coeffVec[5] = 0.0;
    xtmp = coeffVec[0];
    wayPoints_idx_1 = coeffVec[1];
    wayPoints_idx_2 = coeffVec[2];
    for (i = 0; i < 3; i++) {
      y[i] =
          ((double)TMat0[0][i] * xtmp + (double)TMat0[1][i] * wayPoints_idx_1) +
          (double)TMat0[2][i] * wayPoints_idx_2;
    }
    xtmp = wayPoints[1][j] - y[0];
    wayPoints_idx_1 = varargin_2[1][j] - y[1];
    wayPoints_idx_2 = varargin_4[1][j] - y[2];
    for (b_i = 0; b_i < 3; b_i++) {
      coeffVec[b_i + 3] =
          (invTMatF[0][b_i] * xtmp + invTMatF[1][b_i] * wayPoints_idx_1) +
          invTMatF[2][b_i] * wayPoints_idx_2;
    }
    xtmp = coeffVec[0];
    coeffVec[0] = coeffVec[5];
    coeffVec[5] = xtmp;
    xtmp = coeffVec[1];
    coeffVec[1] = coeffVec[4];
    coeffVec[4] = xtmp;
    xtmp = coeffVec[2];
    coeffVec[2] = coeffVec[3];
    coeffVec[3] = xtmp;
    for (i = 0; i < 6; i++) {
      coefMat[i][j] = coeffVec[i];
    }
  }
  for (i = 0; i < 6; i++) {
    coeffMat[i][0] = 0.0;
    coeffMat[i][1] = 0.0;
    coeffMat[i][2] = 0.0;
  }
  for (i = 0; i < 3; i++) {
    xtmp = 0.0;
    for (j = 0; j < 6; j++) {
      xtmp += coefMat[j][i] * (double)b_iv[j];
    }
    coeffMat[5][i] = xtmp;
  }
  for (b_i = 0; b_i < 6; b_i++) {
    for (i = 0; i < 6; i++) {
      coefsWithFlatStart[b_i][i] = 0.0;
    }
    coefsWithFlatStart[b_i][0] = coeffMat[b_i][0];
    coefsWithFlatStart[b_i][3] = coefMat[b_i][0];
    coeffMat[b_i][0] = 0.0;
    coefsWithFlatStart[b_i][1] = coeffMat[b_i][1];
    coefsWithFlatStart[b_i][4] = coefMat[b_i][1];
    coeffMat[b_i][1] = 0.0;
    coefsWithFlatStart[b_i][2] = coeffMat[b_i][2];
    coefsWithFlatStart[b_i][5] = coefMat[b_i][2];
    coeffMat[b_i][2] = 0.0;
  }
  for (i = 0; i < 3; i++) {
    xtmp = 0.0;
    for (j = 0; j < 6; j++) {
      xtmp += coefsWithFlatStart[j][i + 3];
    }
    coeffMat[5][i] = xtmp;
  }
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 9; j++) {
      ddCoeffs[i][j] = 0.0;
    }
    for (j = 0; j < 6; j++) {
      ddCoeffs[i][j] = coefsWithFlatStart[i][j];
    }
    ddCoeffs[i][6] = coeffMat[i][0];
    ddCoeffs[i][7] = coeffMat[i][1];
    ddCoeffs[i][8] = coeffMat[i][2];
  }
  derivativeBreaks[0] = -1.0;
  derivativeBreaks[1] = 0.0;
  derivativeBreaks[2] = 1.0;
  derivativeBreaks[3] = 2.0;
  b_ppval(derivativeBreaks, *(double(*)[6][3][3]) & ddCoeffs[0][0], q);
  derivativeBreaks[0] = -1.0;
  derivativeBreaks[1] = 0.0;
  derivativeBreaks[3] = 2.0;
  derivativeBreaks[2] = 1.01;
  memset(&dCoeffs[0][0], 0, 54U * sizeof(double));
  for (b_i = 0; b_i < 5; b_i++) {
    for (i = 0; i < 9; i++) {
      dCoeffs[b_i + 1][i] =
          ((6.0 - ((double)b_i + 2.0)) + 1.0) * ddCoeffs[b_i][i];
    }
  }
  b_ppval(derivativeBreaks, *(double(*)[6][3][3]) & dCoeffs[0][0], qd);
  memset(&ddCoeffs[0][0], 0, 54U * sizeof(double));
  for (b_i = 0; b_i < 5; b_i++) {
    for (i = 0; i < 9; i++) {
      ddCoeffs[b_i + 1][i] =
          ((6.0 - ((double)b_i + 2.0)) + 1.0) * dCoeffs[b_i][i];
    }
  }
  b_ppval(derivativeBreaks, *(double(*)[6][3][3]) & ddCoeffs[0][0], qdd);
}

/*
 * File trailer for quinticpolytraj.c
 *
 * [EOF]
 */
