/*
 * File: RigidBodyTreeUtils.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "RigidBodyTreeUtils.h"
#include "mod.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double config1
 *                const double config2_data[]
 *                const int config2_size[2]
 *                double dist_data[]
 * Return Type  : int
 */
int RigidBodyTreeUtils_distance(double config1, const double config2_data[],
                                const int config2_size[2], double dist_data[])
{
  emxArray_real_T b_xv_data;
  double configDiff_data[192];
  double thetaWrap_data[192];
  double z1_data[192];
  double y_data[32];
  double xv_data[6];
  int b_loop_ub;
  int configDiff_size_idx_0;
  int configDiff_size_idx_1;
  int dist_size;
  int i;
  int k;
  int loop_ub;
  int nx;
  int x;
  signed char b_iv[6];
  bool x_data[192];
  bool exitg1;
  bool varargout_1;
  if (config2_size[0] < 1) {
    configDiff_size_idx_0 = 0;
    configDiff_size_idx_1 = config2_size[1];
  } else if (config2_size[0] > 1) {
    loop_ub = config2_size[0];
    configDiff_size_idx_0 = config2_size[0];
    b_loop_ub = config2_size[1];
    configDiff_size_idx_1 = config2_size[1];
    for (k = 0; k < b_loop_ub; k++) {
      for (i = 0; i < loop_ub; i++) {
        configDiff_data[i + configDiff_size_idx_0 * k] =
            config2_data[i + config2_size[0] * k] - config1;
      }
    }
  } else {
    configDiff_size_idx_0 = 1;
    loop_ub = config2_size[1];
    configDiff_size_idx_1 = config2_size[1];
    for (k = 0; k < loop_ub; k++) {
      configDiff_data[k] = config2_data[k] - config1;
    }
  }
  for (k = 0; k < 6; k++) {
    b_iv[k] = (signed char)k;
    for (i = 0; i < configDiff_size_idx_0; i++) {
      thetaWrap_data[i + configDiff_size_idx_0 * k] =
          configDiff_data[i + configDiff_size_idx_0 * b_iv[k]];
    }
  }
  if (configDiff_size_idx_0 != 0) {
    for (k = 0; k < 6; k++) {
      for (loop_ub = 0; loop_ub < configDiff_size_idx_0; loop_ub++) {
        z1_data[loop_ub + configDiff_size_idx_0 * k] =
            fabs(configDiff_data[loop_ub + configDiff_size_idx_0 * b_iv[k]]);
      }
    }
  }
  for (k = 0; k < 6; k++) {
    for (i = 0; i < configDiff_size_idx_0; i++) {
      x_data[i + configDiff_size_idx_0 * k] =
          (z1_data[i + configDiff_size_idx_0 * k] > 3.1415926535897931);
    }
  }
  b_loop_ub = configDiff_size_idx_0 * 6;
  varargout_1 = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= b_loop_ub - 1)) {
    x = b_loop_ub;
    if (x_data[k]) {
      varargout_1 = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (varargout_1) {
    for (k = 0; k < 6; k++) {
      for (i = 0; i < configDiff_size_idx_0; i++) {
        z1_data[i + configDiff_size_idx_0 * k] =
            configDiff_data[i + configDiff_size_idx_0 * b_iv[k]] +
            3.1415926535897931;
      }
    }
    for (k = 0; k < 6; k++) {
      for (i = 0; i < configDiff_size_idx_0; i++) {
        thetaWrap_data[i + configDiff_size_idx_0 * k] =
            b_mod(z1_data[i + configDiff_size_idx_0 * k]);
      }
    }
    for (loop_ub = 0; loop_ub < b_loop_ub; loop_ub++) {
      if ((thetaWrap_data[loop_ub] == 0.0) && (z1_data[loop_ub] > 0.0)) {
        x = b_loop_ub;
        thetaWrap_data[loop_ub] = 6.2831853071795862;
      }
    }
    for (k = 0; k < 6; k++) {
      for (i = 0; i < configDiff_size_idx_0; i++) {
        loop_ub = i + configDiff_size_idx_0 * k;
        thetaWrap_data[loop_ub] -= 3.1415926535897931;
      }
    }
  }
  for (k = 0; k < 6; k++) {
    for (i = 0; i < configDiff_size_idx_0; i++) {
      configDiff_data[i + configDiff_size_idx_0 * b_iv[k]] =
          thetaWrap_data[i + configDiff_size_idx_0 * k];
    }
  }
  for (k = 0; k < configDiff_size_idx_0; k++) {
    for (i = 0; i < configDiff_size_idx_1; i++) {
      thetaWrap_data[i + configDiff_size_idx_1 * k] =
          configDiff_data[k + configDiff_size_idx_0 * i];
    }
  }
  b_loop_ub = (signed char)configDiff_size_idx_0;
  if (b_loop_ub - 1 >= 0) {
    memset(&y_data[0], 0, (unsigned int)b_loop_ub * sizeof(double));
  }
  if (configDiff_size_idx_0 - 1 >= 0) {
    nx = configDiff_size_idx_1;
    x = configDiff_size_idx_1;
  }
  for (k = 0; k < configDiff_size_idx_0; k++) {
    dist_size = x;
    if (nx - 1 >= 0) {
      memset(&xv_data[0], 0, (unsigned int)nx * sizeof(double));
    }
    for (loop_ub = 0; loop_ub < nx; loop_ub++) {
      xv_data[loop_ub] = thetaWrap_data[loop_ub + configDiff_size_idx_1 * k];
    }
    if (dist_size == 0) {
      y_data[k] = 0.0;
    } else {
      b_xv_data.data = &xv_data[0];
      b_xv_data.size = &dist_size;
      b_xv_data.allocatedSize = 6;
      b_xv_data.numDimensions = 1;
      b_xv_data.canFreeData = false;
      y_data[k] = vecpnorm(&b_xv_data);
    }
  }
  dist_size = (signed char)configDiff_size_idx_0;
  if (b_loop_ub - 1 >= 0) {
    memcpy(&dist_data[0], &y_data[0], (unsigned int)b_loop_ub * sizeof(double));
  }
  return dist_size;
}

/*
 * File trailer for RigidBodyTreeUtils.c
 *
 * [EOF]
 */
