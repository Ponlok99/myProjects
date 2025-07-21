/*
 * File: cubicpolytraj.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "cubicpolytraj.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double wayPoints[2][3]
 *                double varargin_2[2][3]
 *                double q[101][3]
 *                double qd[101][3]
 *                double qdd[101][3]
 * Return Type  : void
 */
void b_cubicpolytraj(double wayPoints[2][3], double varargin_2[2][3],
                     double q[101][3], double qd[101][3], double qdd[101][3])
{
  double dCoeffs[4][9];
  double ddCoeffs[4][9];
  double coefsWithFlatStart[4][6];
  double coefMat[4][3];
  double coeffMat[4][3];
  double coeffVec[4];
  int i;
  int j;
  for (j = 0; j < 3; j++) {
    double wayPoints_idx_0;
    double wayPoints_idx_1;
    coeffVec[0] = wayPoints[0][j];
    coeffVec[1] = varargin_2[0][j];
    wayPoints_idx_0 = wayPoints[1][j] - (coeffVec[0] + coeffVec[1]);
    wayPoints_idx_1 = varargin_2[1][j] - (0.0 * coeffVec[0] + coeffVec[1]);
    coeffVec[3] = coeffVec[0];
    coefMat[0][j] = -2.0 * wayPoints_idx_0 + wayPoints_idx_1;
    coefMat[1][j] = 3.0 * wayPoints_idx_0 - wayPoints_idx_1;
    coefMat[2][j] = coeffVec[1];
    coefMat[3][j] = coeffVec[3];
  }
  for (i = 0; i < 4; i++) {
    coeffMat[i][0] = 0.0;
    coeffMat[i][1] = 0.0;
    coeffMat[i][2] = 0.0;
  }
  for (i = 0; i < 3; i++) {
    coeffMat[3][i] =
        ((coefMat[0][i] * 0.0 + coefMat[1][i] * 0.0) + coefMat[2][i] * 0.0) +
        coefMat[3][i];
  }
  for (j = 0; j < 4; j++) {
    for (i = 0; i < 6; i++) {
      coefsWithFlatStart[j][i] = 0.0;
    }
    coefsWithFlatStart[j][0] = coeffMat[j][0];
    coefsWithFlatStart[j][3] = coefMat[j][0];
    coeffMat[j][0] = 0.0;
    coefsWithFlatStart[j][1] = coeffMat[j][1];
    coefsWithFlatStart[j][4] = coefMat[j][1];
    coeffMat[j][1] = 0.0;
    coefsWithFlatStart[j][2] = coeffMat[j][2];
    coefsWithFlatStart[j][5] = coefMat[j][2];
    coeffMat[j][2] = 0.0;
  }
  for (i = 0; i < 3; i++) {
    coeffMat[3][i] =
        ((coefsWithFlatStart[0][i + 3] + coefsWithFlatStart[1][i + 3]) +
         coefsWithFlatStart[2][i + 3]) +
        coefsWithFlatStart[3][i + 3];
  }
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 9; j++) {
      ddCoeffs[i][j] = 0.0;
    }
    for (j = 0; j < 6; j++) {
      ddCoeffs[i][j] = coefsWithFlatStart[i][j];
    }
    ddCoeffs[i][6] = coeffMat[i][0];
    ddCoeffs[i][7] = coeffMat[i][1];
    ddCoeffs[i][8] = coeffMat[i][2];
    coeffVec[i] = (double)i - 1.0;
  }
  c_ppval(coeffVec, *(double(*)[4][3][3]) & ddCoeffs[0][0], q);
  coeffVec[0] = -1.0;
  coeffVec[1] = 0.0;
  coeffVec[3] = 2.0;
  coeffVec[2] = 1.01;
  memset(&dCoeffs[0][0], 0, 36U * sizeof(double));
  for (j = 0; j < 3; j++) {
    for (i = 0; i < 9; i++) {
      dCoeffs[j + 1][i] = ((4.0 - ((double)j + 2.0)) + 1.0) * ddCoeffs[j][i];
    }
  }
  c_ppval(coeffVec, *(double(*)[4][3][3]) & dCoeffs[0][0], qd);
  memset(&ddCoeffs[0][0], 0, 36U * sizeof(double));
  for (j = 0; j < 3; j++) {
    for (i = 0; i < 9; i++) {
      ddCoeffs[j + 1][i] = ((4.0 - ((double)j + 2.0)) + 1.0) * dCoeffs[j][i];
    }
  }
  c_ppval(coeffVec, *(double(*)[4][3][3]) & ddCoeffs[0][0], qdd);
}

/*
 * File trailer for cubicpolytraj.c
 *
 * [EOF]
 */
