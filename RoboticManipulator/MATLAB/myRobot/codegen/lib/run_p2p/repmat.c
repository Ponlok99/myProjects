/*
 * File: repmat.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "repmat.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double a_data[]
 *                const int a_size[2]
 *                double b_data[]
 *                int b_size[2]
 * Return Type  : void
 */
void repmat(const double a_data[], const int a_size[2], double b_data[],
            int b_size[2])
{
  int b_k;
  int i;
  int k;
  int t;
  i = a_size[0] << 1;
  b_size[0] = i;
  b_size[1] = 3;
  if (i != 0) {
    int na;
    na = a_size[0];
    for (k = 0; k < 3; k++) {
      for (t = 0; t < 2; t++) {
        int offset;
        offset = t * na;
        for (b_k = 0; b_k < na; b_k++) {
          b_data[(offset + b_k) + i * k] = a_data[b_k + a_size[0] * k];
        }
      }
    }
  }
}

/*
 * File trailer for repmat.c
 *
 * [EOF]
 */
