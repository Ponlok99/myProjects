/*
 * File: sqrt.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "sqrt.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : creal_T *x
 * Return Type  : void
 */
void b_sqrt(creal_T *x)
{
  double absxd2;
  double absxi;
  double xi;
  double xr;
  xr = x->re;
  xi = x->im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      absxi = 0.0;
      absxd2 = sqrt(-xr);
    } else {
      absxi = sqrt(xr);
      absxd2 = 0.0;
    }
  } else if (xr == 0.0) {
    if (xi < 0.0) {
      absxi = sqrt(-xi / 2.0);
      absxd2 = -absxi;
    } else {
      absxi = sqrt(xi / 2.0);
      absxd2 = absxi;
    }
  } else if (rtIsNaN(xr)) {
    absxi = rtNaN;
    absxd2 = rtNaN;
  } else if (rtIsNaN(xi)) {
    absxi = rtNaN;
    absxd2 = rtNaN;
  } else if (rtIsInf(xi)) {
    absxi = fabs(xi);
    absxd2 = xi;
  } else if (rtIsInf(xr)) {
    if (xr < 0.0) {
      absxi = 0.0;
      absxd2 = xi * -xr;
    } else {
      absxi = xr;
      absxd2 = 0.0;
    }
  } else {
    double absxr;
    absxr = fabs(xr);
    absxi = fabs(xi);
    if ((absxr > 4.4942328371557893E+307) ||
        (absxi > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      absxd2 = absxi * 0.5;
      if (absxr < absxd2) {
        double a;
        a = absxr / absxd2;
        absxd2 *= sqrt(a * a + 1.0);
      } else if (absxr > absxd2) {
        absxd2 /= absxr;
        absxd2 = absxr * sqrt(absxd2 * absxd2 + 1.0);
      } else {
        absxd2 = absxr * 1.4142135623730951;
      }
      if (absxd2 > absxr) {
        absxi = sqrt(absxd2) * sqrt(absxr / absxd2 + 1.0);
      } else {
        absxi = sqrt(absxd2) * 1.4142135623730951;
      }
    } else {
      if (absxr < absxi) {
        double a;
        a = absxr / absxi;
        absxd2 = absxi * sqrt(a * a + 1.0);
      } else if (absxr > absxi) {
        absxd2 = absxi / absxr;
        absxd2 = absxr * sqrt(absxd2 * absxd2 + 1.0);
      } else {
        absxd2 = absxr * 1.4142135623730951;
      }
      absxi = sqrt((absxd2 + absxr) * 0.5);
    }
    if (xr > 0.0) {
      absxd2 = 0.5 * (xi / absxi);
    } else {
      if (xi < 0.0) {
        absxd2 = -absxi;
      } else {
        absxd2 = absxi;
      }
      absxi = 0.5 * (xi / absxd2);
    }
  }
  x->re = absxi;
  x->im = absxd2;
}

/*
 * File trailer for sqrt.c
 *
 * [EOF]
 */
