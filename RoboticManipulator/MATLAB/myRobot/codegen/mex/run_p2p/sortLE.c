/*
 * sortLE.c
 *
 * Code generation for function 'sortLE'
 *
 */

/* Include files */
#include "sortLE.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
boolean_T sortLE(const real_T v_data[], const int32_T v_size[2],
                 const int32_T dir_data[], const int32_T dir_size[2],
                 int32_T idx1, int32_T idx2)
{
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= dir_size[1] - 1)) {
    real_T v1;
    real_T v2;
    int32_T v1_tmp;
    v1_tmp = v_size[0] * (dir_data[k] - 1);
    v1 = v_data[(idx1 + v1_tmp) - 1];
    v2 = v_data[(idx2 + v1_tmp) - 1];
    if ((v1 == v2) || (muDoubleScalarIsNaN(v1) && muDoubleScalarIsNaN(v2))) {
      k++;
    } else {
      if ((!(v1 <= v2)) && (!muDoubleScalarIsNaN(v2))) {
        p = false;
      }
      exitg1 = true;
    }
  }
  return p;
}

/* End of code generation (sortLE.c) */
