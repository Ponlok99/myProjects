/*
 * linspace.c
 *
 * Code generation for function 'linspace'
 *
 */

/* Include files */
#include "linspace.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
void linspace(real_T d1, real_T d2, real_T y[101])
{
  int32_T k;
  y[100] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    real_T delta1;
    delta1 = d2 / 100.0;
    for (k = 0; k < 99; k++) {
      y[k + 1] = (2.0 * ((real_T)k + 2.0) - 102.0) * delta1;
    }
    y[50] = 0.0;
  } else if (((d1 < 0.0) != (d2 < 0.0)) &&
             ((muDoubleScalarAbs(d1) > 8.9884656743115785E+307) ||
              (muDoubleScalarAbs(d2) > 8.9884656743115785E+307))) {
    real_T delta1;
    real_T delta2;
    delta1 = d1 / 100.0;
    delta2 = d2 / 100.0;
    for (k = 0; k < 99; k++) {
      y[k + 1] = (d1 + delta2 * ((real_T)k + 1.0)) - delta1 * ((real_T)k + 1.0);
    }
  } else {
    real_T delta1;
    delta1 = (d2 - d1) / 100.0;
    for (k = 0; k < 99; k++) {
      y[k + 1] = d1 + ((real_T)k + 1.0) * delta1;
    }
  }
}

/* End of code generation (linspace.c) */
