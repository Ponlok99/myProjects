/*
 * File: power.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "power.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include "run_p2p_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const creal_T a
 * Return Type  : creal_T
 */
creal_T power(const creal_T a)
{
  creal_T y;
  double b;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 0.33333333333333331);
    y.im = 0.0;
  } else {
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = log(fabs(a.re));
        b = 3.1415926535897931;
      } else {
        r = log(a.re);
        b = 0.0;
      }
    } else {
      bool guard1;
      r = fabs(a.re);
      guard1 = false;
      if (r > 8.9884656743115785E+307) {
        guard1 = true;
      } else {
        b = fabs(a.im);
        if (b > 8.9884656743115785E+307) {
          guard1 = true;
        } else {
          if (r < b) {
            r /= b;
            r = b * sqrt(r * r + 1.0);
          } else if (r > b) {
            b /= r;
            r *= sqrt(b * b + 1.0);
          } else if (rtIsNaN(b)) {
            r = rtNaN;
          } else {
            r *= 1.4142135623730951;
          }
          r = log(r);
          b = b_atan2(a.im, a.re);
        }
      }
      if (guard1) {
        r = fabs(a.re / 2.0);
        b = fabs(a.im / 2.0);
        if (r < b) {
          r /= b;
          r = b * sqrt(r * r + 1.0);
        } else if (r > b) {
          b /= r;
          r *= sqrt(b * b + 1.0);
        } else if (rtIsNaN(b)) {
          r = rtNaN;
        } else {
          r *= 1.4142135623730951;
        }
        r = log(r) + 0.69314718055994529;
        b = b_atan2(a.im, a.re);
      }
    }
    y.re = 0.33333333333333331 * r;
    y.im = 0.33333333333333331 * b;
    if (y.re == 0.0) {
      b = y.im;
      y.re = cos(b);
      y.im = sin(b);
    } else if (y.im == 0.0) {
      b = y.re;
      y.re = exp(b);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      r = exp(y.re / 2.0);
      b = y.im;
      y.re = r * (r * cos(b));
      y.im = r * (r * sin(b));
    }
  }
  return y;
}

/*
 * File trailer for power.c
 *
 * [EOF]
 */
