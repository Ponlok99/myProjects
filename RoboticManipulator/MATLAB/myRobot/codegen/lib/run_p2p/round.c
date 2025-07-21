/*
 * File: round.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "round.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double x_data[]
 *                const int x_size[2]
 * Return Type  : void
 */
void b_round(double x_data[], const int x_size[2])
{
  int b_k;
  int i;
  int k;
  i = x_size[1];
  for (k = 0; k < i; k++) {
    int i1;
    i1 = x_size[0];
    for (b_k = 0; b_k < i1; b_k++) {
      double x;
      int x_tmp;
      x_tmp = b_k + x_size[0] * k;
      x = x_data[x_tmp];
      if (fabs(x) < 4.503599627370496E+15) {
        if (x >= 0.5) {
          x = floor(x + 0.5);
        } else if (x > -0.5) {
          x *= 0.0;
        } else {
          x = ceil(x - 0.5);
        }
      }
      x_data[x_tmp] = x;
    }
  }
}

/*
 * File trailer for round.c
 *
 * [EOF]
 */
