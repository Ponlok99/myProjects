/*
 * File: norm.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "norm.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *x
 * Return Type  : double
 */
double vecpnorm(const emxArray_real_T *x)
{
  const double *x_data;
  double y;
  int k;
  x_data = x->data;
  y = 0.0;
  if (x->size[0] == 1) {
    y = fabs(x_data[0]);
  } else {
    double scale;
    int kend;
    scale = 3.3121686421112381E-170;
    kend = x->size[0];
    for (k = 0; k < kend; k++) {
      double absxk;
      absxk = fabs(x_data[k]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }
    y = scale * sqrt(y);
  }
  return y;
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
