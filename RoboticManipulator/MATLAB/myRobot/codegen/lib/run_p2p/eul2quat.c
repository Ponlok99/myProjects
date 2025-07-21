/*
 * File: eul2quat.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "eul2quat.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double eul[3]
 *                double q[4]
 * Return Type  : void
 */
void eul2quat(const double eul[3], double q[4])
{
  double cosa;
  double cosb;
  double cosc;
  double ein_idx_1;
  double ein_idx_2;
  double sina;
  double sinb;
  double sinc;
  cosc = eul[0] / 2.0;
  ein_idx_1 = eul[1] / 2.0;
  ein_idx_2 = eul[2] / 2.0;
  sina = sin(cosc);
  sinb = sin(ein_idx_1);
  sinc = sin(ein_idx_2);
  cosa = cos(cosc);
  cosb = cos(ein_idx_1);
  cosc = cos(ein_idx_2);
  ein_idx_1 = cosa * cosb;
  q[0] = ein_idx_1 * cosc + sina * sinb * sinc;
  q[1] = ein_idx_1 * sinc - cosc * sina * sinb;
  q[2] = cosa * cosc * sinb + cosb * sina * sinc;
  q[3] = cosb * cosc * sina - cosa * sinb * sinc;
}

/*
 * File trailer for eul2quat.c
 *
 * [EOF]
 */
