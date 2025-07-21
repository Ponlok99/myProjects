/*
 * File: quat2tform.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "quat2tform.h"
#include "rt_nonfinite.h"
#include "run_p2p_rtwutil.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double q[4]
 *                double H[4][4]
 * Return Type  : void
 */
void quat2tform(const double q[4], double H[4][4])
{
  double R[3][3];
  double tempR[9];
  double b;
  double b_tempR_tmp;
  double c_tempR_tmp;
  double d_tempR_tmp;
  double e_tempR_tmp;
  double f_tempR_tmp;
  double g_tempR_tmp;
  double normRowMatrix_idx_0;
  double normRowMatrix_idx_1;
  double normRowMatrix_idx_2;
  double tempR_tmp;
  int k;
  b = 1.0 / sqrt(((rt_powd_snf(q[0], 2.0) + rt_powd_snf(q[1], 2.0)) +
                  rt_powd_snf(q[2], 2.0)) +
                 rt_powd_snf(q[3], 2.0));
  normRowMatrix_idx_0 = q[0] * b;
  normRowMatrix_idx_1 = q[1] * b;
  normRowMatrix_idx_2 = q[2] * b;
  b *= q[3];
  tempR_tmp = b * b;
  b_tempR_tmp = normRowMatrix_idx_2 * normRowMatrix_idx_2;
  c_tempR_tmp = 1.0 - 2.0 * (b_tempR_tmp + tempR_tmp);
  tempR[0] = c_tempR_tmp;
  d_tempR_tmp = normRowMatrix_idx_1 * normRowMatrix_idx_2;
  e_tempR_tmp = normRowMatrix_idx_0 * b;
  c_tempR_tmp = 2.0 * (d_tempR_tmp - e_tempR_tmp);
  tempR[1] = c_tempR_tmp;
  f_tempR_tmp = normRowMatrix_idx_1 * b;
  g_tempR_tmp = normRowMatrix_idx_0 * normRowMatrix_idx_2;
  c_tempR_tmp = 2.0 * (f_tempR_tmp + g_tempR_tmp);
  tempR[2] = c_tempR_tmp;
  d_tempR_tmp = 2.0 * (d_tempR_tmp + e_tempR_tmp);
  tempR[3] = d_tempR_tmp;
  e_tempR_tmp = normRowMatrix_idx_1 * normRowMatrix_idx_1;
  tempR_tmp = 1.0 - 2.0 * (e_tempR_tmp + tempR_tmp);
  tempR[4] = tempR_tmp;
  c_tempR_tmp = normRowMatrix_idx_2 * b;
  b = normRowMatrix_idx_0 * normRowMatrix_idx_1;
  normRowMatrix_idx_2 = 2.0 * (c_tempR_tmp - b);
  tempR[5] = normRowMatrix_idx_2;
  f_tempR_tmp = 2.0 * (f_tempR_tmp - g_tempR_tmp);
  tempR[6] = f_tempR_tmp;
  g_tempR_tmp = 2.0 * (c_tempR_tmp + b);
  tempR[7] = g_tempR_tmp;
  b_tempR_tmp = 1.0 - 2.0 * (e_tempR_tmp + b_tempR_tmp);
  tempR[8] = b_tempR_tmp;
  for (k = 0; k < 3; k++) {
    R[0][k] = tempR[3 * k];
    R[1][k] = tempR[3 * k + 1];
    R[2][k] = tempR[3 * k + 2];
  }
  for (k = 0; k < 4; k++) {
    H[k][0] = 0.0;
    H[k][1] = 0.0;
    H[k][2] = 0.0;
    H[k][3] = 0.0;
  }
  for (k = 0; k < 3; k++) {
    H[k][0] = R[k][0];
    H[k][1] = R[k][1];
    H[k][2] = R[k][2];
  }
  H[3][3] = 1.0;
}

/*
 * File trailer for quat2tform.c
 *
 * [EOF]
 */
