/*
 * File: any.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "any.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const bool x[2]
 * Return Type  : bool
 */
bool any(const bool x[2])
{
  int k;
  bool exitg1;
  bool y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * File trailer for any.c
 *
 * [EOF]
 */
