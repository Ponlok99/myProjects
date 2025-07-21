/*
 * File: eye.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "eye.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double b_I[4][4]
 * Return Type  : void
 */
void eye(double b_I[4][4])
{
  int i;
  for (i = 0; i < 4; i++) {
    b_I[i][0] = 0.0;
    b_I[i][1] = 0.0;
    b_I[i][2] = 0.0;
    b_I[i][3] = 0.0;
  }
  b_I[0][0] = 1.0;
  b_I[1][1] = 1.0;
  b_I[2][2] = 1.0;
  b_I[3][3] = 1.0;
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
