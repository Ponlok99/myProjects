/*
 * File: wrapToPi.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "wrapToPi.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double theta_data[]
 *                const int theta_size[2]
 * Return Type  : void
 */
void b_wrapToPi(double theta_data[], const int theta_size[2])
{
  double z1_data[192];
  int b_k;
  int i;
  int i1;
  int k;
  int z1_size_idx_0;
  int z1_size_idx_1_tmp;
  bool x_data[192];
  bool exitg1;
  bool varargout_1;
  z1_size_idx_0 = (signed char)theta_size[0];
  z1_size_idx_1_tmp = (signed char)theta_size[1];
  if ((theta_size[0] != 0) && (theta_size[1] != 0)) {
    for (k = 0; k < z1_size_idx_1_tmp; k++) {
      for (b_k = 0; b_k < z1_size_idx_0; b_k++) {
        z1_data[b_k + z1_size_idx_0 * k] =
            fabs(theta_data[b_k + theta_size[0] * k]);
      }
    }
  }
  for (i = 0; i < z1_size_idx_1_tmp; i++) {
    for (i1 = 0; i1 < z1_size_idx_0; i1++) {
      x_data[i1 + z1_size_idx_0 * i] =
          (z1_data[i1 + z1_size_idx_0 * i] > 3.1415926535897931);
    }
  }
  z1_size_idx_0 *= (signed char)theta_size[1];
  varargout_1 = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= z1_size_idx_0 - 1)) {
    if (x_data[k]) {
      varargout_1 = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (varargout_1) {
    b_k = theta_size[0];
    k = theta_size[1];
    for (i = 0; i < k; i++) {
      for (i1 = 0; i1 < b_k; i1++) {
        z1_data[i1 + b_k * i] =
            theta_data[i1 + theta_size[0] * i] + 3.1415926535897931;
      }
    }
    for (i = 0; i < k; i++) {
      for (i1 = 0; i1 < b_k; i1++) {
        theta_data[i1 + theta_size[0] * i] = b_mod(z1_data[i1 + b_k * i]);
      }
    }
    z1_size_idx_0 = theta_size[0] * theta_size[1];
    for (z1_size_idx_1_tmp = 0; z1_size_idx_1_tmp < z1_size_idx_0;
         z1_size_idx_1_tmp++) {
      if ((theta_data[z1_size_idx_1_tmp] == 0.0) &&
          (z1_data[z1_size_idx_1_tmp] > 0.0)) {
        theta_data[z1_size_idx_1_tmp] = 6.2831853071795862;
      }
    }
    for (i = 0; i < k; i++) {
      for (i1 = 0; i1 < b_k; i1++) {
        z1_size_idx_0 = i1 + theta_size[0] * i;
        theta_data[z1_size_idx_0] -= 3.1415926535897931;
      }
    }
  }
}

/*
 * Arguments    : double theta[3]
 * Return Type  : void
 */
void wrapToPi(double theta[3])
{
  double z1[3];
  int k;
  bool exitg1;
  bool y;
  z1[0] = fabs(theta[0]);
  z1[1] = fabs(theta[1]);
  z1[2] = fabs(theta[2]);
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (z1[k] > 3.1415926535897931) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (y) {
    double d;
    d = theta[0] + 3.1415926535897931;
    z1[0] = d;
    theta[0] = b_mod(d);
    d = theta[1] + 3.1415926535897931;
    z1[1] = d;
    theta[1] = b_mod(d);
    d = theta[2] + 3.1415926535897931;
    theta[2] = b_mod(d);
    if ((theta[0] == 0.0) && (z1[0] > 0.0)) {
      theta[0] = 6.2831853071795862;
    }
    if ((theta[1] == 0.0) && (z1[1] > 0.0)) {
      theta[1] = 6.2831853071795862;
    }
    if ((theta[2] == 0.0) && (d > 0.0)) {
      theta[2] = 6.2831853071795862;
    }
    theta[0] -= 3.1415926535897931;
    theta[1] -= 3.1415926535897931;
    theta[2] -= 3.1415926535897931;
  }
}

/*
 * File trailer for wrapToPi.c
 *
 * [EOF]
 */
