/*
 * File: solvePoly.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "solvePoly.h"
#include "constructM.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double constraints[8]
 *                double p[8]
 * Return Type  : void
 */
void solvePoly(const double constraints[8], double p[8])
{
  static const double QPrime[8][8] = {
      {1120.0000000007876, 560.00000000075715, 100.00000000018457,
       3.333333333338242, -1120.0000000007876, 560.000000000724,
       -100.0000000000714, 3.3333333333495787},
      {560.00000000040291, 297.142857143245, 58.571428571525047,
       2.0952380952402105, -560.00000000040291, 262.85714285752147,
       -41.42857142860862, 1.2380952381035719},
      {100.00000000008367, 58.571428571509045, 14.285714285735004,
       0.54761904761940183, -100.00000000008367, 41.428571428652958,
       -5.7142857142936236, 0.11904761904934924},
      {3.3333333333412725, 2.095238095245179, 0.54761904762106361,
       0.063492063492081918, -3.3333333333412725, 1.2380952381033552,
       -0.11904761904857253, -0.0079365079363525881},
      {-1120.0000000007876, -560.00000000075715, -100.00000000018457,
       -3.333333333338242, 1120.0000000007876, -560.000000000724,
       100.0000000000714, -3.3333333333495787},
      {560.00000000038472, 262.85714285751214, 41.428571428659524,
       1.2380952380980315, -560.00000000038472, 297.14285714320249,
       -58.571428571462775, 2.0952380952460068},
      {-100.00000000007481, -41.428571428642613, -5.7142857143022354,
       -0.11904761904831229, 100.00000000007481, -58.571428571493016,
       14.285714285720871, -0.54761904762056668},
      {3.3333333333392687, 1.2380952381008115, 0.11904761904883188,
       -0.0079365079364733526, -3.3333333333392687, 2.0952380952430332,
       -0.54761904761955194, 0.063492063492181339}};
  static const double b_AInv[8][8] = {
      {1.0, 0.0, 0.0, 0.0, -34.999999999999467, 83.999999999998792,
       -69.999999999999, 19.999999999999716},
      {0.0, 1.0, 0.0, 0.0, -19.999999999999716, 44.999999999999382,
       -35.9999999999995, 9.9999999999998579},
      {0.0, 0.0, 0.5, 0.0, -4.9999999999999307, 9.9999999999998685,
       -7.4999999999999014, 1.9999999999999718},
      {0.0, 0.0, 0.0, 0.16666666666666666, -0.66666666666665519,
       0.99999999999998335, -0.66666666666665908, 0.16666666666666446},
      {0.0, 0.0, 0.0, 0.0, 34.999999999999467, -83.999999999998792,
       69.999999999999, -19.999999999999716},
      {0.0, 0.0, 0.0, 0.0, -14.999999999999753, 38.99999999999941,
       -33.9999999999995, 9.9999999999998579},
      {0.0, 0.0, 0.0, 0.0, 2.4999999999999503, -6.9999999999998819,
       6.4999999999999014, -1.9999999999999718},
      {0.0, 0.0, 0.0, 0.0, -0.16666666666666266, 0.49999999999999056,
       -0.49999999999999212, 0.16666666666666441}};
  emxArray_real_T *b_y;
  double M[8][8];
  double R_data[64];
  double R_tmp[8][8];
  double b_M[8][8];
  double constraints_data[18];
  double AInv[8];
  double b_R_tmp[8];
  double d;
  double *y_data;
  int R_size[2];
  int b_i;
  int i;
  int i1;
  int i2;
  int i3;
  int k;
  int partialTrueCount;
  int trueCount;
  int y;
  signed char tmp_data[8];
  y = !rtIsNaN(constraints[0]);
  for (k = 0; k < 7; k++) {
    y += !rtIsNaN(constraints[k + 1]);
  }
  constructM(constraints, M);
  for (i = 0; i < 8; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      R_tmp[i][i1] = M[i1][i];
      d = 0.0;
      for (i2 = 0; i2 < 8; i2++) {
        d += M[i2][i] * QPrime[i1][i2];
      }
      b_M[i1][i] = d;
    }
  }
  for (i = 0; i < 8; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      d = 0.0;
      for (i2 = 0; i2 < 8; i2++) {
        d += b_M[i2][i] * R_tmp[i1][i2];
      }
      M[i1][i] = d;
    }
  }
  if (y + 1 > 8) {
    i = 0;
    i1 = 0;
    i2 = 0;
    i3 = 0;
    k = 0;
  } else {
    i = y;
    i1 = 8;
    i2 = y;
    i3 = y;
    k = 8;
  }
  trueCount = 0;
  partialTrueCount = 0;
  for (b_i = 0; b_i < 8; b_i++) {
    if (!rtIsNaN(constraints[b_i])) {
      trueCount++;
      tmp_data[partialTrueCount] = (signed char)b_i;
      partialTrueCount++;
    }
  }
  partialTrueCount = k - i3;
  emxInit_real_T(&b_y, 1);
  k = b_y->size[0];
  b_y->size[0] = partialTrueCount;
  emxEnsureCapacity_real_T(b_y, k);
  y_data = b_y->data;
  for (b_i = 0; b_i < partialTrueCount; b_i++) {
    y_data[b_i] = 0.0;
  }
  for (k = 0; k < y; k++) {
    for (b_i = 0; b_i < partialTrueCount; b_i++) {
      y_data[b_i] += M[i3 + b_i][k] * constraints[tmp_data[k]];
    }
  }
  partialTrueCount = i1 - i;
  R_size[0] = partialTrueCount;
  R_size[1] = partialTrueCount;
  for (i1 = 0; i1 < partialTrueCount; i1++) {
    for (i3 = 0; i3 < partialTrueCount; i3++) {
      R_data[i3 + partialTrueCount * i1] = -M[i2 + i1][i + i3];
    }
  }
  emxReserve_real_T(b_y);
  y_data = b_y->data;
  b_mldivide(R_data, R_size, (double *)b_y->data, &(*(int(*)[1])b_y->size)[0]);
  for (i = 0; i < trueCount; i++) {
    constraints_data[i] = constraints[tmp_data[i]];
  }
  partialTrueCount = b_y->size[0];
  for (i = 0; i < partialTrueCount; i++) {
    constraints_data[i + trueCount] = y_data[i];
  }
  emxFree_real_T(&b_y);
  for (i = 0; i < 8; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d += R_tmp[i1][i] * constraints_data[i1];
    }
    b_R_tmp[i] = d;
  }
  for (i = 0; i < 8; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d += b_AInv[i1][i] * b_R_tmp[i1];
    }
    AInv[i] = d;
  }
  for (i = 0; i < 8; i++) {
    p[i] = AInv[7 - i];
  }
}

/*
 * File trailer for solvePoly.c
 *
 * [EOF]
 */
