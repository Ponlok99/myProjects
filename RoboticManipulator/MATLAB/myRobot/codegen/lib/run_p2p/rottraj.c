/*
 * File: rottraj.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "rottraj.h"
#include "log.h"
#include "quaternion.h"
#include "rt_nonfinite.h"
#include "run_p2p_internal_types.h"
#include "slerp.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double R0[4]
 *                const double RF[4]
 *                double varargin_2[101][3]
 *                double R[101][4]
 *                double omega[101][3]
 *                double alpha[101][3]
 * Return Type  : void
 */
void rottraj(const double R0[4], const double RF[4], double varargin_2[101][3],
             double R[101][4], double omega[101][3], double alpha[101][3])
{
  quaternion expl_temp;
  quaternion pnCorrected;
  quaternion q;
  double timeScaling[101][3];
  double b_q1n_tmp;
  double b_q2n_a_tmp;
  double c_q1n_tmp;
  double c_q2n_a_tmp;
  double d;
  double d_q1n_tmp;
  double dp;
  double dp_tmp;
  double n;
  double o_a_tmp;
  double o_b_tmp;
  double o_c_tmp;
  double o_d_tmp;
  double pn_a;
  double pn_b;
  double pn_c;
  double pn_d;
  double q1n_tmp;
  double q2n_a;
  double q2n_a_tmp;
  double q2n_b;
  double q2n_b_tmp;
  double q2n_c_tmp;
  double q2n_d_tmp;
  double q_tmp;
  double qn_a;
  double qn_b;
  double qn_c;
  double qn_d;
  double theta0;
  double xd;
  double y;
  int i;
  for (i = 0; i < 101; i++) {
    d = varargin_2[i][0];
    theta0 = d;
    if ((d > 1.0) && (d < 1.0000000149011612)) {
      theta0 = 1.0;
    }
    if ((d < 0.0) && (d > -1.4901161193847656E-8)) {
      theta0 = 0.0;
    }
    timeScaling[i][0] = theta0;
    timeScaling[i][1] = varargin_2[i][1];
    timeScaling[i][2] = varargin_2[i][2];
  }
  n = sqrt(((R0[0] * R0[0] + R0[1] * R0[1]) + R0[2] * R0[2]) + R0[3] * R0[3]);
  pn_a = R0[0] / n;
  pn_b = R0[1] / n;
  pn_c = R0[2] / n;
  pn_d = R0[3] / n;
  n = sqrt(((RF[0] * RF[0] + RF[1] * RF[1]) + RF[2] * RF[2]) + RF[3] * RF[3]);
  qn_a = RF[0] / n;
  qn_b = RF[1] / n;
  qn_c = RF[2] / n;
  qn_d = RF[3] / n;
  theta0 = sqrt(((pn_a * pn_a + pn_b * pn_b) + pn_c * pn_c) + pn_d * pn_d);
  n = pn_a / theta0;
  dp = pn_b / theta0;
  xd = pn_c / theta0;
  theta0 = pn_d / theta0;
  q1n_tmp = ((n - dp * 0.0) - xd * 0.0) - theta0 * 0.0;
  b_q1n_tmp = ((n * 0.0 + dp) + xd * 0.0) - theta0 * 0.0;
  c_q1n_tmp = ((n * 0.0 - dp * 0.0) + xd) + theta0 * 0.0;
  d_q1n_tmp = ((n * 0.0 + dp * 0.0) - xd * 0.0) + theta0;
  theta0 = sqrt(((qn_a * qn_a + qn_b * qn_b) + qn_c * qn_c) + qn_d * qn_d);
  dp = qn_a / theta0;
  xd = qn_b / theta0;
  n = qn_c / theta0;
  q_tmp = qn_d / theta0;
  q2n_a_tmp = n * 0.0;
  theta0 = q_tmp * 0.0;
  b_q2n_a_tmp = xd * 0.0;
  c_q2n_a_tmp = ((dp - b_q2n_a_tmp) - q2n_a_tmp) - theta0;
  q2n_a = c_q2n_a_tmp;
  dp *= 0.0;
  q2n_b_tmp = ((dp + xd) + q2n_a_tmp) - theta0;
  q2n_b = q2n_b_tmp;
  q2n_c_tmp = ((dp - b_q2n_a_tmp) + n) + theta0;
  xd = q2n_c_tmp;
  q2n_d_tmp = ((dp + b_q2n_a_tmp) - q2n_a_tmp) + q_tmp;
  q_tmp = q2n_d_tmp;
  dp_tmp = ((q1n_tmp * c_q2n_a_tmp + b_q1n_tmp * q2n_b_tmp) +
            c_q1n_tmp * q2n_c_tmp) +
           d_q1n_tmp * q2n_d_tmp;
  dp = dp_tmp;
  if (dp_tmp < 0.0) {
    q2n_a = -c_q2n_a_tmp;
    q2n_b = -q2n_b_tmp;
    xd = -q2n_c_tmp;
    q_tmp = -q2n_d_tmp;
    dp = -dp_tmp;
  }
  if (dp > 1.0) {
    dp = 1.0;
  }
  theta0 = acos(dp);
  dp = sin(theta0);
  n = 1.0 / dp;
  y = sin(0.0 * theta0);
  pnCorrected.a = n * (dp * q1n_tmp + y * q2n_a);
  pnCorrected.b = n * (dp * b_q1n_tmp + y * q2n_b);
  pnCorrected.c = n * (dp * c_q1n_tmp + y * xd);
  pnCorrected.d = n * (dp * d_q1n_tmp + y * q_tmp);
  if (rtIsInf(n)) {
    quaternion_parenAssign(&pnCorrected, (double *)&pn_a, (double *)&pn_b,
                           (double *)&pn_c, (double *)&pn_d);
  }
  n = sqrt(((pnCorrected.a * pnCorrected.a + pnCorrected.b * pnCorrected.b) +
            pnCorrected.c * pnCorrected.c) +
           pnCorrected.d * pnCorrected.d);
  pnCorrected.a /= n;
  pnCorrected.b /= n;
  pnCorrected.c /= n;
  pnCorrected.d /= n;
  expl_temp =
      quaternionBase_slerp(pn_a, pn_b, pn_c, pn_d, qn_a, qn_b, qn_c, qn_d);
  o_a_tmp = ((pnCorrected.a * expl_temp.a - -pnCorrected.b * expl_temp.b) -
             -pnCorrected.c * expl_temp.c) -
            -pnCorrected.d * expl_temp.d;
  o_b_tmp = ((pnCorrected.a * expl_temp.b + -pnCorrected.b * expl_temp.a) +
             -pnCorrected.c * expl_temp.d) -
            -pnCorrected.d * expl_temp.c;
  o_c_tmp = ((pnCorrected.a * expl_temp.c - -pnCorrected.b * expl_temp.d) +
             -pnCorrected.c * expl_temp.a) +
            -pnCorrected.d * expl_temp.b;
  o_d_tmp = ((pnCorrected.a * expl_temp.d + -pnCorrected.b * expl_temp.c) -
             -pnCorrected.c * expl_temp.b) +
            -pnCorrected.d * expl_temp.a;
  for (i = 0; i < 101; i++) {
    double xa;
    double xb;
    double xc;
    q2n_a = c_q2n_a_tmp;
    q2n_b = q2n_b_tmp;
    xd = q2n_c_tmp;
    q_tmp = q2n_d_tmp;
    dp = dp_tmp;
    if (dp_tmp < 0.0) {
      q2n_a = -c_q2n_a_tmp;
      q2n_b = -q2n_b_tmp;
      xd = -q2n_c_tmp;
      q_tmp = -q2n_d_tmp;
      dp = -dp_tmp;
    }
    if (dp > 1.0) {
      dp = 1.0;
    }
    theta0 = acos(dp);
    n = 1.0 / sin(theta0);
    d = timeScaling[i][0];
    y = sin((1.0 - d) * theta0);
    dp = sin(d * theta0);
    pnCorrected.a = n * (y * q1n_tmp + dp * q2n_a);
    pnCorrected.b = n * (y * b_q1n_tmp + dp * q2n_b);
    pnCorrected.c = n * (y * c_q1n_tmp + dp * xd);
    pnCorrected.d = n * (y * d_q1n_tmp + dp * q_tmp);
    if (rtIsInf(n)) {
      quaternion_parenAssign(&pnCorrected, (double *)&pn_a, (double *)&pn_b,
                             (double *)&pn_c, (double *)&pn_d);
    }
    n = sqrt(((pnCorrected.a * pnCorrected.a + pnCorrected.b * pnCorrected.b) +
              pnCorrected.c * pnCorrected.c) +
             pnCorrected.d * pnCorrected.d);
    pnCorrected.a /= n;
    pnCorrected.b /= n;
    pnCorrected.c /= n;
    pnCorrected.d /= n;
    expl_temp.a = o_a_tmp;
    expl_temp.b = o_b_tmp;
    expl_temp.c = o_c_tmp;
    expl_temp.d = o_d_tmp;
    quaternionBase_log(&expl_temp);
    d = timeScaling[i][1];
    dp = d * (((pnCorrected.a * expl_temp.a - pnCorrected.b * expl_temp.b) -
               pnCorrected.c * expl_temp.c) -
              pnCorrected.d * expl_temp.d);
    theta0 = d * (((pnCorrected.a * expl_temp.b + pnCorrected.b * expl_temp.a) +
                   pnCorrected.c * expl_temp.d) -
                  pnCorrected.d * expl_temp.c);
    q2n_b = d * (((pnCorrected.a * expl_temp.c - pnCorrected.b * expl_temp.d) +
                  pnCorrected.c * expl_temp.a) +
                 pnCorrected.d * expl_temp.b);
    qn_a = d * (((pnCorrected.a * expl_temp.d + pnCorrected.b * expl_temp.c) -
                 pnCorrected.c * expl_temp.b) +
                pnCorrected.d * expl_temp.a);
    qn_b = 2.0 * dp;
    qn_c = 2.0 * theta0;
    qn_d = 2.0 * q2n_b;
    q2n_a = 2.0 * qn_a;
    omega[i][0] = ((qn_b * -pnCorrected.b + qn_c * pnCorrected.a) +
                   qn_d * -pnCorrected.d) -
                  q2n_a * -pnCorrected.c;
    omega[i][1] = ((qn_b * -pnCorrected.c - qn_c * -pnCorrected.d) +
                   qn_d * pnCorrected.a) +
                  q2n_a * -pnCorrected.b;
    omega[i][2] = ((qn_b * -pnCorrected.d + qn_c * -pnCorrected.c) -
                   qn_d * -pnCorrected.b) +
                  q2n_a * pnCorrected.a;
    expl_temp.a = o_a_tmp;
    expl_temp.b = o_b_tmp;
    expl_temp.c = o_c_tmp;
    expl_temp.d = o_d_tmp;
    quaternionBase_log(&expl_temp);
    q.a = o_a_tmp;
    q.b = o_b_tmp;
    q.c = o_c_tmp;
    q.d = o_d_tmp;
    quaternionBase_log(&q);
    n = ((pnCorrected.a * q.a - pnCorrected.b * q.b) - pnCorrected.c * q.c) -
        pnCorrected.d * q.d;
    q_tmp =
        ((pnCorrected.a * q.b + pnCorrected.b * q.a) + pnCorrected.c * q.d) -
        pnCorrected.d * q.c;
    q2n_a_tmp =
        ((pnCorrected.a * q.c - pnCorrected.b * q.d) + pnCorrected.c * q.a) +
        pnCorrected.d * q.b;
    b_q2n_a_tmp =
        ((pnCorrected.a * q.d + pnCorrected.b * q.c) - pnCorrected.c * q.b) +
        pnCorrected.d * q.a;
    q.a = o_a_tmp;
    q.b = o_b_tmp;
    q.c = o_c_tmp;
    q.d = o_d_tmp;
    quaternionBase_log(&q);
    y = d * d;
    d = timeScaling[i][2];
    xa =
        2.0 *
        (y * (((n * q.a - q_tmp * q.b) - q2n_a_tmp * q.c) - b_q2n_a_tmp * q.d) +
         d * (((pnCorrected.a * expl_temp.a - pnCorrected.b * expl_temp.b) -
               pnCorrected.c * expl_temp.c) -
              pnCorrected.d * expl_temp.d));
    xb =
        2.0 *
        (y * (((n * q.b + q_tmp * q.a) + q2n_a_tmp * q.d) - b_q2n_a_tmp * q.c) +
         d * (((pnCorrected.a * expl_temp.b + pnCorrected.b * expl_temp.a) +
               pnCorrected.c * expl_temp.d) -
              pnCorrected.d * expl_temp.c));
    xc =
        2.0 *
        (y * (((n * q.c - q_tmp * q.d) + q2n_a_tmp * q.a) + b_q2n_a_tmp * q.b) +
         d * (((pnCorrected.a * expl_temp.c - pnCorrected.b * expl_temp.d) +
               pnCorrected.c * expl_temp.a) +
              pnCorrected.d * expl_temp.b));
    xd =
        2.0 *
        (y * (((n * q.d + q_tmp * q.c) - q2n_a_tmp * q.b) + b_q2n_a_tmp * q.a) +
         d * (((pnCorrected.a * expl_temp.d + pnCorrected.b * expl_temp.c) -
               pnCorrected.c * expl_temp.b) +
              pnCorrected.d * expl_temp.a));
    theta0 = -theta0;
    q2n_b = -q2n_b;
    qn_a = -qn_a;
    alpha[i][0] =
        (((xa * -pnCorrected.b + xb * pnCorrected.a) + xc * -pnCorrected.d) -
         xd * -pnCorrected.c) -
        (((qn_b * theta0 + qn_c * dp) + qn_d * qn_a) - q2n_a * q2n_b);
    alpha[i][1] =
        (((xa * -pnCorrected.c - xb * -pnCorrected.d) + xc * pnCorrected.a) +
         xd * -pnCorrected.b) -
        (((qn_b * q2n_b - qn_c * qn_a) + qn_d * dp) + q2n_a * theta0);
    alpha[i][2] =
        (((xa * -pnCorrected.d + xb * -pnCorrected.c) - xc * -pnCorrected.b) +
         xd * pnCorrected.a) -
        (((qn_b * qn_a + qn_c * q2n_b) - qn_d * theta0) + q2n_a * dp);
    R[i][0] = pnCorrected.a;
    R[i][1] = pnCorrected.b;
    R[i][2] = pnCorrected.c;
    R[i][3] = pnCorrected.d;
  }
}

/*
 * File trailer for rottraj.c
 *
 * [EOF]
 */
