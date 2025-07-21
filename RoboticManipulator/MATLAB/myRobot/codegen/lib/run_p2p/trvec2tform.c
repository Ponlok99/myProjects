/*
 * File: trvec2tform.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "trvec2tform.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double t[3]
 *                double H[4][4]
 * Return Type  : void
 */
void trvec2tform(const double t[3], double H[4][4])
{
  int i;
  for (i = 0; i < 4; i++) {
    H[i][0] = 0.0;
    H[i][1] = 0.0;
    H[i][2] = 0.0;
    H[i][3] = 0.0;
  }
  H[0][0] = 1.0;
  H[1][1] = 1.0;
  H[2][2] = 1.0;
  H[3][3] = 1.0;
  H[3][0] = t[0];
  H[3][1] = t[1];
  H[3][2] = t[2];
}

/*
 * File trailer for trvec2tform.c
 *
 * [EOF]
 */
