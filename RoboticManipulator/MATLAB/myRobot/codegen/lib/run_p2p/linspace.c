/*
 * File: linspace.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "linspace.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double d1
 *                double d2
 *                double y[101]
 * Return Type  : void
 */
void linspace(double d1, double d2, double y[101])
{
  int k;
  y[100] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    double delta1;
    delta1 = d2 / 100.0;
    for (k = 0; k < 99; k++) {
      y[k + 1] = (2.0 * ((double)k + 2.0) - 102.0) * delta1;
    }
    y[50] = 0.0;
  } else if (((d1 < 0.0) != (d2 < 0.0)) &&
             ((fabs(d1) > 8.9884656743115785E+307) ||
              (fabs(d2) > 8.9884656743115785E+307))) {
    double delta1;
    double delta2;
    delta1 = d1 / 100.0;
    delta2 = d2 / 100.0;
    for (k = 0; k < 99; k++) {
      y[k + 1] = (d1 + delta2 * ((double)k + 1.0)) - delta1 * ((double)k + 1.0);
    }
  } else {
    double delta1;
    delta1 = (d2 - d1) / 100.0;
    for (k = 0; k < 99; k++) {
      y[k + 1] = d1 + ((double)k + 1.0) * delta1;
    }
  }
}

/*
 * File trailer for linspace.c
 *
 * [EOF]
 */
