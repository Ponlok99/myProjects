/*
 * File: slerp.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "slerp.h"
#include "quaternion.h"
#include "rt_nonfinite.h"
#include "run_p2p_internal_types.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double q1_a
 *                double q1_b
 *                double q1_c
 *                double q1_d
 *                double q2_a
 *                double q2_b
 *                double q2_c
 *                double q2_d
 * Return Type  : quaternion
 */
quaternion quaternionBase_slerp(double q1_a, double q1_b, double q1_c,
                                double q1_d, double q2_a, double q2_b,
                                double q2_c, double q2_d)
{
  quaternion qo;
  double n;
  double q1n_a;
  double q1n_b;
  double q1n_c;
  double q1n_d;
  double q2n_a;
  double q2n_b;
  double q2n_c;
  double q_a;
  double q_c;
  double sinv;
  n = sqrt(((q1_a * q1_a + q1_b * q1_b) + q1_c * q1_c) + q1_d * q1_d);
  q_a = q1_a / n;
  sinv = q1_b / n;
  q_c = q1_c / n;
  n = q1_d / n;
  q1n_a = ((q_a - sinv * 0.0) - q_c * 0.0) - n * 0.0;
  q1n_b = ((q_a * 0.0 + sinv) + q_c * 0.0) - n * 0.0;
  q1n_c = ((q_a * 0.0 - sinv * 0.0) + q_c) + n * 0.0;
  q1n_d = ((q_a * 0.0 + sinv * 0.0) - q_c * 0.0) + n;
  n = sqrt(((q2_a * q2_a + q2_b * q2_b) + q2_c * q2_c) + q2_d * q2_d);
  q_a = q2_a / n;
  sinv = q2_b / n;
  q_c = q2_c / n;
  n = q2_d / n;
  q2n_a = ((q_a - sinv * 0.0) - q_c * 0.0) - n * 0.0;
  q2n_b = ((q_a * 0.0 + sinv) + q_c * 0.0) - n * 0.0;
  q2n_c = ((q_a * 0.0 - sinv * 0.0) + q_c) + n * 0.0;
  q_c = ((q_a * 0.0 + sinv * 0.0) - q_c * 0.0) + n;
  n = ((q1n_a * q2n_a + q1n_b * q2n_b) + q1n_c * q2n_c) + q1n_d * q_c;
  if (n < 0.0) {
    q2n_a = -q2n_a;
    q2n_b = -q2n_b;
    q2n_c = -q2n_c;
    q_c = -q_c;
    n = -n;
  }
  if (n > 1.0) {
    n = 1.0;
  }
  n = acos(n);
  q_a = sin(n);
  sinv = 1.0 / q_a;
  n = sin(0.0 * n);
  qo.a = sinv * (n * q1n_a + q_a * q2n_a);
  qo.b = sinv * (n * q1n_b + q_a * q2n_b);
  qo.c = sinv * (n * q1n_c + q_a * q2n_c);
  qo.d = sinv * (n * q1n_d + q_a * q_c);
  if (rtIsInf(sinv)) {
    quaternion_parenAssign(&qo, (double *)&q1_a, (double *)&q1_b,
                           (double *)&q1_c, (double *)&q1_d);
  }
  n = sqrt(((qo.a * qo.a + qo.b * qo.b) + qo.c * qo.c) + qo.d * qo.d);
  qo.a /= n;
  qo.b /= n;
  qo.c /= n;
  qo.d /= n;
  return qo;
}

/*
 * File trailer for slerp.c
 *
 * [EOF]
 */
