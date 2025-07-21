/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "mldivide.h"
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "xgeqp3.h"
#include "xzgetrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double A_data[]
 *                const int A_size[2]
 *                double B_data[]
 *                int *B_size
 * Return Type  : void
 */
void b_mldivide(const double A_data[], const int A_size[2], double B_data[],
                int *B_size)
{
  emxArray_int32_T *jpvt;
  emxArray_real_T *A;
  emxArray_real_T *tau;
  double b_B_data[10];
  double *b_A_data;
  double *tau_data;
  int b_i;
  int i;
  int i1;
  int j;
  int *jpvt_data;
  emxInit_real_T(&A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  if ((A_size[0] == 0) || (A_size[1] == 0) || (*B_size == 0)) {
    int LDA;
    LDA = A_size[1];
    *B_size = A_size[1];
    if (LDA - 1 >= 0) {
      memset(&B_data[0], 0, (unsigned int)LDA * sizeof(double));
    }
  } else if (A_size[0] == A_size[1]) {
    double wj;
    int LDA;
    int m;
    int n;
    m = A_size[0];
    n = A_size[1];
    if (m <= n) {
      n = m;
    }
    m = *B_size;
    if (m <= n) {
      n = m;
    }
    LDA = A_size[0];
    i = A->size[0] * A->size[1];
    A->size[0] = A_size[0];
    m = A_size[1];
    A->size[1] = A_size[1];
    emxEnsureCapacity_real_T(A, i);
    b_A_data = A->data;
    for (i = 0; i < m; i++) {
      for (i1 = 0; i1 < LDA; i1++) {
        b_A_data[i1 + A->size[0] * i] = A_data[i1 + A_size[0] * i];
      }
    }
    xzgetrf(n, n, A, A_size[0], jpvt);
    jpvt_data = jpvt->data;
    b_A_data = A->data;
    LDA = A->size[0];
    for (b_i = 0; b_i <= n - 2; b_i++) {
      i = jpvt_data[b_i];
      if (i != b_i + 1) {
        wj = B_data[b_i];
        B_data[b_i] = B_data[i - 1];
        B_data[i - 1] = wj;
      }
    }
    for (j = 0; j < n; j++) {
      m = LDA * j;
      if (B_data[j] != 0.0) {
        i = j + 2;
        for (b_i = i; b_i <= n; b_i++) {
          B_data[b_i - 1] -= B_data[j] * b_A_data[(b_i + m) - 1];
        }
      }
    }
    for (j = n; j >= 1; j--) {
      m = LDA * (j - 1);
      wj = B_data[j - 1];
      if (wj != 0.0) {
        wj /= b_A_data[(j + m) - 1];
        B_data[j - 1] = wj;
        for (b_i = 0; b_i <= j - 2; b_i++) {
          B_data[b_i] -= B_data[j - 1] * b_A_data[b_i + m];
        }
      }
    }
  } else {
    int LDA;
    int m;
    int n;
    LDA = A_size[0];
    i = A->size[0] * A->size[1];
    A->size[0] = A_size[0];
    m = A_size[1];
    A->size[1] = A_size[1];
    emxEnsureCapacity_real_T(A, i);
    b_A_data = A->data;
    for (i = 0; i < m; i++) {
      for (i1 = 0; i1 < LDA; i1++) {
        b_A_data[i1 + A->size[0] * i] = A_data[i1 + A_size[0] * i];
      }
    }
    xgeqp3(A, tau, jpvt);
    jpvt_data = jpvt->data;
    tau_data = tau->data;
    b_A_data = A->data;
    n = rankFromQR(A);
    LDA = *B_size;
    if (LDA - 1 >= 0) {
      memcpy(&b_B_data[0], &B_data[0], (unsigned int)LDA * sizeof(double));
    }
    LDA = A->size[1];
    *B_size = A->size[1];
    if (LDA - 1 >= 0) {
      memset(&B_data[0], 0, (unsigned int)LDA * sizeof(double));
    }
    m = A->size[0];
    LDA = A->size[1];
    if (m <= LDA) {
      LDA = m;
    }
    for (j = 0; j < LDA; j++) {
      m = A->size[0];
      if (tau_data[j] != 0.0) {
        double wj;
        wj = b_B_data[j];
        i = j + 2;
        for (b_i = i; b_i <= m; b_i++) {
          wj += b_A_data[(b_i + A->size[0] * j) - 1] * b_B_data[b_i - 1];
        }
        wj *= tau_data[j];
        if (wj != 0.0) {
          b_B_data[j] -= wj;
          for (b_i = i; b_i <= m; b_i++) {
            b_B_data[b_i - 1] -= b_A_data[(b_i + A->size[0] * j) - 1] * wj;
          }
        }
      }
    }
    i = (unsigned char)n;
    for (b_i = 0; b_i < i; b_i++) {
      B_data[jpvt_data[b_i] - 1] = b_B_data[b_i];
    }
    for (j = n; j >= 1; j--) {
      i = jpvt_data[j - 1];
      B_data[i - 1] /= b_A_data[(j + A->size[0] * (j - 1)) - 1];
      i1 = (unsigned char)(j - 1);
      for (b_i = 0; b_i < i1; b_i++) {
        B_data[jpvt_data[b_i] - 1] -=
            B_data[i - 1] * b_A_data[b_i + A->size[0] * (j - 1)];
      }
    }
  }
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&A);
}

/*
 * Arguments    : const double A_data[]
 *                const int A_size[2]
 *                const double B[6]
 *                double Y_data[]
 * Return Type  : int
 */
int mldivide(const double A_data[], const int A_size[2], const double B[6],
             double Y_data[])
{
  double tau_data[6];
  int jpvt_data[49];
  int b_A_size[2];
  int Y_size;
  int i;
  int j;
  int k;
  int maxmn;
  int rankA;
  switch (A_size[1]) {
  case 0:
    Y_size = 0;
    break;
  case 6: {
    double c_A_data[36];
    double b_B[6];
    double tol;
    int minmn;
    signed char ipiv[6];
    maxmn = A_size[1];
    for (i = 0; i < maxmn; i++) {
      for (k = 0; k < 6; k++) {
        minmn = k + 6 * i;
        c_A_data[minmn] = A_data[minmn];
      }
    }
    for (maxmn = 0; maxmn < 6; maxmn++) {
      b_B[maxmn] = B[maxmn];
      ipiv[maxmn] = (signed char)(maxmn + 1);
    }
    for (j = 0; j < 5; j++) {
      int b_tmp;
      signed char i1;
      Y_size = 4 - j;
      b_tmp = j * 7;
      rankA = b_tmp + 2;
      minmn = 7 - j;
      maxmn = 0;
      tol = fabs(c_A_data[b_tmp]);
      for (k = 2; k < minmn; k++) {
        double s;
        s = fabs(c_A_data[(b_tmp + k) - 1]);
        if (s > tol) {
          maxmn = k - 1;
          tol = s;
        }
      }
      if (c_A_data[b_tmp + maxmn] != 0.0) {
        if (maxmn != 0) {
          minmn = j + maxmn;
          ipiv[j] = (signed char)(minmn + 1);
          for (k = 0; k < 6; k++) {
            maxmn = j + k * 6;
            tol = c_A_data[maxmn];
            i = minmn + k * 6;
            c_A_data[maxmn] = c_A_data[i];
            c_A_data[i] = tol;
          }
        }
        i = (b_tmp - j) + 6;
        for (maxmn = rankA; maxmn <= i; maxmn++) {
          c_A_data[maxmn - 1] /= c_A_data[b_tmp];
        }
      }
      minmn = b_tmp;
      for (maxmn = 0; maxmn <= Y_size; maxmn++) {
        tol = c_A_data[(b_tmp + maxmn * 6) + 6];
        if (tol != 0.0) {
          i = minmn + 8;
          k = (minmn - j) + 12;
          for (rankA = i; rankA <= k; rankA++) {
            c_A_data[rankA - 1] +=
                c_A_data[((b_tmp + rankA) - minmn) - 7] * -tol;
          }
        }
        minmn += 6;
      }
      i1 = ipiv[j];
      if (i1 != j + 1) {
        tol = b_B[j];
        b_B[j] = b_B[i1 - 1];
        b_B[i1 - 1] = tol;
      }
    }
    for (k = 0; k < 6; k++) {
      minmn = 6 * k;
      if (b_B[k] != 0.0) {
        i = k + 2;
        for (maxmn = i; maxmn < 7; maxmn++) {
          b_B[maxmn - 1] -= b_B[k] * c_A_data[(maxmn + minmn) - 1];
        }
      }
    }
    for (k = 5; k >= 0; k--) {
      minmn = 6 * k;
      tol = b_B[k];
      if (tol != 0.0) {
        tol /= c_A_data[k + minmn];
        b_B[k] = tol;
        for (maxmn = 0; maxmn < k; maxmn++) {
          b_B[maxmn] -= b_B[k] * c_A_data[maxmn + minmn];
        }
      }
    }
    Y_size = 6;
    for (i = 0; i < 6; i++) {
      Y_data[i] = b_B[i];
    }
  } break;
  default: {
    double b_A_data[294];
    double b_B[6];
    double tol;
    int minmn;
    b_A_size[0] = 6;
    maxmn = A_size[1];
    b_A_size[1] = A_size[1];
    for (i = 0; i < maxmn; i++) {
      for (k = 0; k < 6; k++) {
        minmn = k + 6 * i;
        b_A_data[minmn] = A_data[minmn];
      }
    }
    int jpvt_size[2];
    b_xgeqp3(b_A_data, b_A_size, tau_data, jpvt_data, jpvt_size);
    rankA = 0;
    if (b_A_size[1] > 6) {
      minmn = 6;
      maxmn = b_A_size[1];
    } else {
      minmn = b_A_size[1];
      maxmn = 6;
    }
    if (minmn > 0) {
      tol = 2.2204460492503131E-15 * (double)maxmn * fabs(b_A_data[0]);
      while ((rankA < minmn) && (!(fabs(b_A_data[rankA + 6 * rankA]) <= tol))) {
        rankA++;
      }
    }
    for (maxmn = 0; maxmn < 6; maxmn++) {
      b_B[maxmn] = B[maxmn];
    }
    maxmn = b_A_size[1];
    Y_size = b_A_size[1];
    if (maxmn - 1 >= 0) {
      memset(&Y_data[0], 0, (unsigned int)maxmn * sizeof(double));
    }
    if (b_A_size[1] > 6) {
      i = 6;
    } else {
      i = b_A_size[1];
    }
    for (j = 0; j < i; j++) {
      if (tau_data[j] != 0.0) {
        tol = b_B[j];
        k = j + 2;
        for (maxmn = k; maxmn < 7; maxmn++) {
          tol += b_A_data[(maxmn + 6 * j) - 1] * b_B[maxmn - 1];
        }
        tol *= tau_data[j];
        if (tol != 0.0) {
          b_B[j] -= tol;
          for (maxmn = k; maxmn < 7; maxmn++) {
            b_B[maxmn - 1] -= b_A_data[(maxmn + 6 * j) - 1] * tol;
          }
        }
      }
    }
    for (maxmn = 0; maxmn < rankA; maxmn++) {
      Y_data[jpvt_data[maxmn] - 1] = b_B[maxmn];
    }
    for (j = rankA; j >= 1; j--) {
      i = jpvt_data[j - 1];
      minmn = 6 * (j - 1);
      Y_data[i - 1] /= b_A_data[(j + minmn) - 1];
      for (maxmn = 0; maxmn <= j - 2; maxmn++) {
        k = jpvt_data[maxmn];
        Y_data[k - 1] -= Y_data[i - 1] * b_A_data[maxmn + minmn];
      }
    }
  } break;
  }
  return Y_size;
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
