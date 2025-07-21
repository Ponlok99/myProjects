/*
 * File: rotm2eul.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "rotm2eul.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double R[3][3]
 *                double eul[3]
 * Return Type  : void
 */
void b_rotm2eul(double R[3][3], double eul[3])
{
  double sy;
  double sySq;
  sySq = R[1][2] * R[1][2] + R[0][2] * R[0][2];
  sy = sqrt(sySq);
  eul[0] = b_atan2(R[1][2], R[0][2]);
  eul[1] = b_atan2(sy, 1.0);
  eul[2] = b_atan2(R[2][1], -R[2][0]);
  if (sySq < 2.2204460492503131E-15) {
    eul[0] = b_atan2(-R[0][1], R[1][1]);
    eul[1] = b_atan2(sy, R[2][2]);
    eul[2] = 0.0;
  }
  eul[0] = -eul[0];
  eul[1] = -eul[1];
  eul[2] = -eul[2];
  sySq = eul[0];
  eul[0] = eul[2];
  eul[2] = sySq;
}

/*
 * Arguments    : double R[3][3]
 *                double eul[3]
 * Return Type  : void
 */
void rotm2eul(double R[3][3], double eul[3])
{
  double sy;
  double sySq;
  sySq = R[1][2] * R[1][2] + R[0][2] * R[0][2];
  sy = sqrt(sySq);
  eul[0] = b_atan2(R[1][2], R[0][2]);
  eul[1] = b_atan2(sy, R[2][2]);
  eul[2] = b_atan2(R[2][1], -R[2][0]);
  if (sySq < 2.2204460492503131E-15) {
    eul[0] = b_atan2(-R[0][1], R[1][1]);
    eul[1] = b_atan2(sy, R[2][2]);
    eul[2] = 0.0;
  }
  eul[0] = -eul[0];
  eul[1] = -eul[1];
  eul[2] = -eul[2];
  sySq = eul[0];
  eul[0] = eul[2];
  eul[2] = sySq;
}

/*
 * File trailer for rotm2eul.c
 *
 * [EOF]
 */
