/*
 * power.c
 *
 * Code generation for function 'power'
 *
 */

/* Include files */
#include "power.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
creal_T power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = muDoubleScalarPower(a.re, 0.33333333333333331);
    y.im = 0.0;
  } else {
    real_T b_im;
    real_T r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = muDoubleScalarLog(muDoubleScalarAbs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = muDoubleScalarLog(a.re);
        b_im = 0.0;
      }
    } else if ((muDoubleScalarAbs(a.re) > 8.9884656743115785E+307) ||
               (muDoubleScalarAbs(a.im) > 8.9884656743115785E+307)) {
      r = muDoubleScalarLog(muDoubleScalarHypot(a.re / 2.0, a.im / 2.0)) +
          0.69314718055994529;
      b_im = muDoubleScalarAtan2(a.im, a.re);
    } else {
      r = muDoubleScalarLog(muDoubleScalarHypot(a.re, a.im));
      b_im = muDoubleScalarAtan2(a.im, a.re);
    }
    y.re = 0.33333333333333331 * r;
    y.im = 0.33333333333333331 * b_im;
    if (y.re == 0.0) {
      b_im = y.im;
      y.re = muDoubleScalarCos(b_im);
      y.im = muDoubleScalarSin(b_im);
    } else if (y.im == 0.0) {
      b_im = y.re;
      y.re = muDoubleScalarExp(b_im);
      y.im = 0.0;
    } else if (muDoubleScalarIsInf(y.im) && muDoubleScalarIsInf(y.re) &&
               (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      r = muDoubleScalarExp(y.re / 2.0);
      b_im = y.im;
      y.re = r * (r * muDoubleScalarCos(b_im));
      y.im = r * (r * muDoubleScalarSin(b_im));
    }
  }
  return y;
}

/* End of code generation (power.c) */
