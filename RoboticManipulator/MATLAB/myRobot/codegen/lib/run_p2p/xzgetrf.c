/*
 * File: xzgetrf.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "xzgetrf.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                emxArray_real_T *A
 *                int lda
 *                emxArray_int32_T *ipiv
 * Return Type  : int
 */
int xzgetrf(int m, int n, emxArray_real_T *A, int lda, emxArray_int32_T *ipiv)
{
  double *A_data;
  int b_j;
  int b_n;
  int i;
  int info;
  int j;
  int k;
  int yk;
  int *ipiv_data;
  A_data = A->data;
  if (m <= n) {
    yk = m;
  } else {
    yk = n;
  }
  if (yk < 1) {
    b_n = 0;
  } else {
    b_n = yk;
  }
  i = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity_int32_T(ipiv, i);
  ipiv_data = ipiv->data;
  if (b_n > 0) {
    ipiv_data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      ipiv_data[k - 1] = yk;
    }
  }
  info = 0;
  if ((m >= 1) && (n >= 1)) {
    int u0;
    u0 = m - 1;
    if (u0 > n) {
      u0 = n;
    }
    for (j = 0; j < u0; j++) {
      double smax;
      int b_tmp;
      int ipiv_tmp;
      int jA;
      int mmj;
      mmj = m - j;
      b_tmp = j * (lda + 1);
      b_n = b_tmp + 2;
      if (mmj < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj > 1) {
          smax = fabs(A_data[b_tmp]);
          for (k = 2; k <= mmj; k++) {
            double s;
            s = fabs(A_data[(b_tmp + k) - 1]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }
      if (A_data[b_tmp + yk] != 0.0) {
        if (yk != 0) {
          ipiv_tmp = j + yk;
          ipiv_data[j] = ipiv_tmp + 1;
          for (k = 0; k < n; k++) {
            yk = k * lda;
            jA = j + yk;
            smax = A_data[jA];
            i = ipiv_tmp + yk;
            A_data[jA] = A_data[i];
            A_data[i] = smax;
          }
        }
        i = b_tmp + mmj;
        for (yk = b_n; yk <= i; yk++) {
          A_data[yk - 1] /= A_data[b_tmp];
        }
      } else {
        info = j + 1;
      }
      b_n = n - j;
      ipiv_tmp = b_tmp + lda;
      jA = ipiv_tmp + 1;
      for (b_j = 0; b_j <= b_n - 2; b_j++) {
        yk = ipiv_tmp + b_j * lda;
        smax = A_data[yk];
        if (A_data[yk] != 0.0) {
          i = jA + 1;
          yk = mmj + jA;
          for (k = i; k < yk; k++) {
            A_data[k - 1] += A_data[(b_tmp + k) - jA] * -smax;
          }
        }
        jA += lda;
      }
    }
    if ((info == 0) && (m <= n) &&
        (!(A_data[(m + A->size[0] * (m - 1)) - 1] != 0.0))) {
      info = m;
    }
  }
  return info;
}

/*
 * File trailer for xzgetrf.c
 *
 * [EOF]
 */
