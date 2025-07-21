/*
 * nthroot.c
 *
 * Code generation for function 'nthroot'
 *
 */

/* Include files */
#include "nthroot.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
real_T nthroot(real_T x)
{
  real_T y;
  if (x < 0.0) {
    y = -muDoubleScalarPower(-x, 0.33333333333333331);
  } else {
    y = muDoubleScalarPower(x, 0.33333333333333331);
  }
  if ((!muDoubleScalarIsInf(y)) && (!muDoubleScalarIsNaN(y)) && (x != 0.0)) {
    real_T d;
    real_T y2n;
    y2n = muDoubleScalarPower(y, 3.0);
    d = y2n - x;
    if (d != 0.0) {
      y -= d / (3.0 * (y2n / y));
    }
  }
  return y;
}

/* End of code generation (nthroot.c) */
