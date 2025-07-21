/*
 * File: abs.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "abs.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const creal_T x
 * Return Type  : double
 */
double b_abs(const creal_T x)
{
  double b;
  double y;
  y = fabs(x.re);
  b = fabs(x.im);
  if (y < b) {
    y /= b;
    y = b * sqrt(y * y + 1.0);
  } else if (y > b) {
    b /= y;
    y *= sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = rtNaN;
  } else {
    y *= 1.4142135623730951;
  }
  return y;
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
