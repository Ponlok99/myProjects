/*
 * File: xgerc.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "xgerc.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                double alpha1
 *                int ix0
 *                const emxArray_real_T *y
 *                emxArray_real_T *A
 *                int ia0
 *                int lda
 * Return Type  : void
 */
void xgerc(int m, int n, double alpha1, int ix0, const emxArray_real_T *y,
           emxArray_real_T *A, int ia0, int lda)
{
  const double *y_data;
  double *A_data;
  int ijA;
  int j;
  A_data = A->data;
  y_data = y->data;
  if (!(alpha1 == 0.0)) {
    int jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      if (y_data[j] != 0.0) {
        double temp;
        int i;
        temp = y_data[j] * alpha1;
        i = m + jA;
        for (ijA = jA; ijA < i; ijA++) {
          A_data[ijA - 1] += A_data[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += lda;
    }
  }
}

/*
 * File trailer for xgerc.c
 *
 * [EOF]
 */
