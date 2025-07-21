/*
 * File: unique.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "unique.h"
#include "rt_nonfinite.h"
#include "sortLE.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double a_data[]
 *                const int a_size[2]
 *                double b_data[]
 *                int b_size[2]
 *                int ndx_data[]
 * Return Type  : int
 */
int unique_rows(const double a_data[], const int a_size[2], double b_data[],
                int b_size[2], int ndx_data[])
{
  int idx_data[32];
  int iwork_data[32];
  int col_size[2];
  int b_i;
  int i;
  int i1;
  int j;
  int k;
  int n;
  int ndx_size;
  int pEnd;
  int qEnd;
  if (a_size[0] == 0) {
    b_size[0] = 0;
    b_size[1] = a_size[1];
    ndx_size = 0;
  } else {
    double ycol_data[32];
    int col_data[6];
    int i2;
    int loop_ub;
    ndx_size = a_size[0];
    b_size[0] = a_size[0];
    loop_ub = a_size[1];
    b_size[1] = a_size[1];
    col_size[0] = 1;
    col_size[1] = a_size[1];
    if (a_size[1] - 1 >= 0) {
      n = a_size[0];
    }
    for (k = 0; k < loop_ub; k++) {
      for (i = 0; i < n; i++) {
        b_data[i + b_size[0] * k] = a_data[i + a_size[0] * k];
      }
      col_data[k] = k + 1;
    }
    n = a_size[0] + 1;
    memset(&idx_data[0], 0, (unsigned int)ndx_size * sizeof(int));
    if (a_size[1] == 0) {
      for (k = 0; k <= n - 2; k++) {
        idx_data[k] = k + 1;
      }
    } else {
      i = a_size[0] - 1;
      for (k = 1; k <= i; k += 2) {
        if (sortLE(a_data, a_size, col_data, col_size, k, k + 1)) {
          idx_data[k - 1] = k;
          idx_data[k] = k + 1;
        } else {
          idx_data[k - 1] = k + 1;
          idx_data[k] = k;
        }
      }
      if (((unsigned int)a_size[0] & 1U) != 0U) {
        idx_data[a_size[0] - 1] = a_size[0];
      }
      b_i = 2;
      while (b_i < n - 1) {
        i2 = b_i << 1;
        j = 1;
        for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
          int kEnd;
          int p;
          int q;
          p = j;
          q = pEnd;
          qEnd = j + i2;
          if (qEnd > n) {
            qEnd = n;
          }
          k = 0;
          kEnd = qEnd - j;
          while (k + 1 <= kEnd) {
            i = idx_data[q - 1];
            i1 = idx_data[p - 1];
            if (sortLE(a_data, a_size, col_data, col_size, i1, i)) {
              iwork_data[k] = i1;
              p++;
              if (p == pEnd) {
                while (q < qEnd) {
                  k++;
                  iwork_data[k] = idx_data[q - 1];
                  q++;
                }
              }
            } else {
              iwork_data[k] = i;
              q++;
              if (q == qEnd) {
                while (p < pEnd) {
                  k++;
                  iwork_data[k] = idx_data[p - 1];
                  p++;
                }
              }
            }
            k++;
          }
          for (k = 0; k < kEnd; k++) {
            idx_data[(j + k) - 1] = iwork_data[k];
          }
          j = qEnd;
        }
        b_i = i2;
      }
    }
    for (j = 0; j < loop_ub; j++) {
      for (b_i = 0; b_i < ndx_size; b_i++) {
        ycol_data[b_i] = b_data[(idx_data[b_i] + b_size[0] * j) - 1];
      }
      for (b_i = 0; b_i < ndx_size; b_i++) {
        b_data[b_i + b_size[0] * j] = ycol_data[b_i];
      }
    }
    for (i = 0; i < ndx_size; i++) {
      ycol_data[i] = idx_data[i];
    }
    pEnd = -1;
    k = 0;
    while (k + 1 <= ndx_size) {
      i2 = k;
      int exitg1;
      do {
        exitg1 = 0;
        k++;
        if (k + 1 > ndx_size) {
          exitg1 = 1;
        } else {
          bool b_p;
          bool exitg2;
          b_p = false;
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j <= b_size[1] - 1)) {
            i = b_size[0] * j;
            if (b_data[i2 + i] != b_data[k + i]) {
              b_p = true;
              exitg2 = true;
            } else {
              j++;
            }
          }
          if (b_p) {
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
      pEnd++;
      for (j = 0; j < loop_ub; j++) {
        n = b_size[0] * j;
        b_data[pEnd + n] = b_data[i2 + n];
      }
      ycol_data[pEnd] = ycol_data[i2];
    }
    if (pEnd + 1 < 1) {
      ndx_size = 0;
    } else {
      ndx_size = pEnd + 1;
    }
    for (i = 0; i < loop_ub; i++) {
      for (i1 = 0; i1 < ndx_size; i1++) {
        b_data[i1 + ndx_size * i] = b_data[i1 + b_size[0] * i];
      }
    }
    b_size[0] = ndx_size;
    ndx_size = pEnd + 1;
    i = (unsigned char)(pEnd + 1);
    for (k = 0; k < i; k++) {
      ndx_data[k] = (int)ycol_data[k];
    }
  }
  return ndx_size;
}

/*
 * File trailer for unique.c
 *
 * [EOF]
 */
