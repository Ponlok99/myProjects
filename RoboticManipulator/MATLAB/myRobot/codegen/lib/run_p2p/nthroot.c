/*
 * File: nthroot.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "nthroot.h"
#include "rt_nonfinite.h"
#include "run_p2p_rtwutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double x
 * Return Type  : double
 */
double nthroot(double x)
{
  double y;
  if (x < 0.0) {
    y = -rt_powd_snf(-x, 0.33333333333333331);
  } else {
    y = rt_powd_snf(x, 0.33333333333333331);
  }
  if ((!rtIsInf(y)) && (!rtIsNaN(y)) && (x != 0.0)) {
    double d;
    double y2n;
    y2n = rt_powd_snf(y, 3.0);
    d = y2n - x;
    if (d != 0.0) {
      y -= d / (3.0 * (y2n / y));
    }
  }
  return y;
}

/*
 * File trailer for nthroot.c
 *
 * [EOF]
 */
