/*
 * File: wrapTo2Pi.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "wrapTo2Pi.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double theta
 * Return Type  : double
 */
double wrapTo2Pi(double theta)
{
  double thetaWrap;
  if (rtIsNaN(theta) || rtIsInf(theta)) {
    thetaWrap = rtNaN;
  } else if (theta == 0.0) {
    thetaWrap = 0.0;
  } else {
    bool rEQ0;
    thetaWrap = fmod(theta, 6.2831853071795862);
    rEQ0 = (thetaWrap == 0.0);
    if (!rEQ0) {
      double q;
      q = fabs(theta / 6.2831853071795862);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      thetaWrap = 0.0;
    } else if (thetaWrap < 0.0) {
      thetaWrap += 6.2831853071795862;
    }
  }
  if ((thetaWrap == 0.0) && (theta > 0.0)) {
    thetaWrap = 6.2831853071795862;
  }
  return thetaWrap;
}

/*
 * File trailer for wrapTo2Pi.c
 *
 * [EOF]
 */
