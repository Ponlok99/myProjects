/*
 * File: sortLE.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "sortLE.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double v_data[]
 *                const int v_size[2]
 *                const int dir_data[]
 *                const int dir_size[2]
 *                int idx1
 *                int idx2
 * Return Type  : bool
 */
bool sortLE(const double v_data[], const int v_size[2], const int dir_data[],
            const int dir_size[2], int idx1, int idx2)
{
  int k;
  bool exitg1;
  bool p;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= dir_size[1] - 1)) {
    double v1;
    double v2;
    int v1_tmp;
    v1_tmp = v_size[0] * (dir_data[k] - 1);
    v1 = v_data[(idx1 + v1_tmp) - 1];
    v2 = v_data[(idx2 + v1_tmp) - 1];
    if ((v1 == v2) || (rtIsNaN(v1) && rtIsNaN(v2))) {
      k++;
    } else {
      if ((!(v1 <= v2)) && (!rtIsNaN(v2))) {
        p = false;
      }
      exitg1 = true;
    }
  }
  return p;
}

/*
 * File trailer for sortLE.c
 *
 * [EOF]
 */
