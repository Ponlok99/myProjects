/*
 * File: sortIdx.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "sortIdx.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void merge(int idx_data[], double x_data[], int offset, int np, int nq,
                  int iwork_data[], double xwork_data[]);

/* Function Definitions */
/*
 * Arguments    : int idx_data[]
 *                double x_data[]
 *                int offset
 *                int np
 *                int nq
 *                int iwork_data[]
 *                double xwork_data[]
 * Return Type  : void
 */
static void merge(int idx_data[], double x_data[], int offset, int np, int nq,
                  int iwork_data[], double xwork_data[])
{
  int j;
  if (nq != 0) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 *                int idx_data[]
 * Return Type  : int
 */
int sortIdx(double x_data[], const int *x_size, int idx_data[])
{
  double b_x_data[32];
  int b_idx_data[32];
  int i;
  int idx_size;
  int k;
  idx_size = *x_size;
  if (idx_size - 1 >= 0) {
    memset(&idx_data[0], 0, (unsigned int)idx_size * sizeof(int));
  }
  if (*x_size != 0) {
    double xwork_data[32];
    double x4[4];
    int iwork_data[32];
    int b_i1;
    int i1;
    int i2;
    int i3;
    int i4;
    int ib;
    int nNaNs;
    signed char idx4[4];
    for (i = 0; i < idx_size; i++) {
      b_idx_data[i] = 0;
      b_x_data[i] = x_data[i];
      iwork_data[i] = 0;
    }
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    nNaNs = 0;
    ib = 0;
    for (k = 0; k < idx_size; k++) {
      if (rtIsNaN(b_x_data[k])) {
        i3 = (idx_size - nNaNs) - 1;
        b_idx_data[i3] = k + 1;
        xwork_data[i3] = b_x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = (signed char)(k + 1);
        x4[ib - 1] = b_x_data[k];
        if (ib == 4) {
          double d;
          double d1;
          ib = k - nNaNs;
          if (x4[0] <= x4[1]) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (x4[2] <= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          d = x4[i1 - 1];
          d1 = x4[i3 - 1];
          if (d <= d1) {
            d = x4[i2 - 1];
            if (d <= d1) {
              i = i1;
              b_i1 = i2;
              i1 = i3;
              i2 = i4;
            } else if (d <= x4[i4 - 1]) {
              i = i1;
              b_i1 = i3;
              i1 = i2;
              i2 = i4;
            } else {
              i = i1;
              b_i1 = i3;
              i1 = i4;
            }
          } else {
            d1 = x4[i4 - 1];
            if (d <= d1) {
              if (x4[i2 - 1] <= d1) {
                i = i3;
                b_i1 = i1;
                i1 = i2;
                i2 = i4;
              } else {
                i = i3;
                b_i1 = i1;
                i1 = i4;
              }
            } else {
              i = i3;
              b_i1 = i4;
            }
          }
          b_idx_data[ib - 3] = idx4[i - 1];
          b_idx_data[ib - 2] = idx4[b_i1 - 1];
          b_idx_data[ib - 1] = idx4[i1 - 1];
          b_idx_data[ib] = idx4[i2 - 1];
          b_x_data[ib - 3] = x4[i - 1];
          b_x_data[ib - 2] = x4[b_i1 - 1];
          b_x_data[ib - 1] = x4[i1 - 1];
          b_x_data[ib] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    i4 = idx_size - nNaNs;
    if (ib > 0) {
      signed char perm[4];
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      switch (ib) {
      case 1:
        perm[0] = 1;
        break;
      case 2:
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
        break;
      default:
        if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }
        break;
      }
      i = (unsigned char)ib;
      for (k = 0; k < i; k++) {
        i3 = (i4 - ib) + k;
        b_i1 = perm[k];
        b_idx_data[i3] = idx4[b_i1 - 1];
        b_x_data[i3] = x4[b_i1 - 1];
      }
    }
    i1 = nNaNs >> 1;
    for (k = 0; k < i1; k++) {
      ib = i4 + k;
      i2 = b_idx_data[ib];
      i3 = (idx_size - k) - 1;
      b_idx_data[ib] = b_idx_data[i3];
      b_idx_data[i3] = i2;
      b_x_data[ib] = xwork_data[i3];
      b_x_data[i3] = xwork_data[ib];
    }
    if (((unsigned int)nNaNs & 1U) != 0U) {
      ib = i4 + i1;
      b_x_data[ib] = xwork_data[ib];
    }
    if (i4 > 1) {
      i3 = i4 >> 2;
      i2 = 4;
      while (i3 > 1) {
        if (((unsigned int)i3 & 1U) != 0U) {
          i3--;
          ib = i2 * i3;
          i1 = i4 - ib;
          if (i1 > i2) {
            merge(b_idx_data, b_x_data, ib, i2, i1 - i2, iwork_data,
                  xwork_data);
          }
        }
        ib = i2 << 1;
        i3 >>= 1;
        for (k = 0; k < i3; k++) {
          merge(b_idx_data, b_x_data, k * ib, i2, i2, iwork_data, xwork_data);
        }
        i2 = ib;
      }
      if (i4 > i2) {
        merge(b_idx_data, b_x_data, 0, i2, i4 - i2, iwork_data, xwork_data);
      }
    }
    if (idx_size - 1 >= 0) {
      memcpy(&idx_data[0], &b_idx_data[0],
             (unsigned int)idx_size * sizeof(int));
      memcpy(&x_data[0], &b_x_data[0], (unsigned int)idx_size * sizeof(double));
    }
  }
  return idx_size;
}

/*
 * File trailer for sortIdx.c
 *
 * [EOF]
 */
