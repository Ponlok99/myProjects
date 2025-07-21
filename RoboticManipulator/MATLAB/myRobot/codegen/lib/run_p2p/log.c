/*
 * File: log.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "log.h"
#include "rt_nonfinite.h"
#include "run_p2p_internal_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : quaternion *q
 * Return Type  : void
 */
void quaternionBase_log(quaternion *q)
{
  double qnorm;
  double vnorm;
  double vscale_data;
  int k;
  int trueCount;
  bool b;
  vnorm = sqrt((q->b * q->b + q->c * q->c) + q->d * q->d);
  qnorm = sqrt(q->a * q->a + vnorm * vnorm);
  trueCount = 0;
  if (vnorm != 0.0) {
    trueCount = 1;
  }
  for (k = 0; k < trueCount; k++) {
    vscale_data = q->a / qnorm;
  }
  for (k = 0; k < trueCount; k++) {
    vscale_data = acos(vscale_data);
  }
  for (k = 0; k < trueCount; k++) {
    vscale_data /= vnorm;
  }
  q->a = log(qnorm);
  qnorm = q->b;
  for (k = 0; k < trueCount; k++) {
    qnorm = q->b * vscale_data;
  }
  b = !(vnorm != 0.0);
  if (b) {
    qnorm = 0.0;
  }
  q->b = qnorm;
  qnorm = q->c;
  for (k = 0; k < trueCount; k++) {
    qnorm = q->c * vscale_data;
  }
  if (b) {
    qnorm = 0.0;
  }
  q->c = qnorm;
  qnorm = q->d;
  for (k = 0; k < trueCount; k++) {
    qnorm = q->d * vscale_data;
  }
  if (b) {
    qnorm = 0.0;
  }
  q->d = qnorm;
}

/*
 * File trailer for log.c
 *
 * [EOF]
 */
