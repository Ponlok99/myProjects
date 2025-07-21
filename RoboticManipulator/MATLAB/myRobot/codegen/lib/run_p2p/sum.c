/*
 * File: sum.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "sum.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double x[2][2]
 *                double y[2]
 * Return Type  : void
 */
void sum(double x[2][2], double y[2])
{
  y[0] = x[0][0] + x[1][0];
  y[1] = x[0][1] + x[1][1];
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
