/*
 * File: constructM.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "constructM.h"
#include "find.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double constraints[10]
 *                double M[10][10]
 * Return Type  : void
 */
void b_constructM(const double constraints[10], double M[10][10])
{
  int freeBCIdx_data[10];
  int b_i;
  int fixedBCIdx_size;
  int i;
  int k;
  signed char M1[10][10];
  signed char Mcontinuity[10][10];
  bool b_fixedBCIdx_tmp[10];
  bool fixedBCIdx_tmp[10];
  for (i = 0; i < 10; i++) {
    for (b_i = 0; b_i < 10; b_i++) {
      Mcontinuity[i][b_i] = 0;
      M1[i][b_i] = 0;
    }
    bool b;
    b = rtIsNaN(constraints[i]);
    b_fixedBCIdx_tmp[i] = b;
    fixedBCIdx_tmp[i] = !b;
  }
  i = b_eml_find(fixedBCIdx_tmp, freeBCIdx_data);
  fixedBCIdx_size = i;
  for (k = 0; k < i; k++) {
    M1[k][freeBCIdx_data[k] - 1] = 1;
  }
  b_eml_find(b_fixedBCIdx_tmp, freeBCIdx_data);
  b_i = 10 - fixedBCIdx_size;
  for (i = 0; i < b_i; i++) {
    M1[fixedBCIdx_size + i][freeBCIdx_data[i] - 1] = 1;
  }
  for (i = 0; i < 10; i++) {
    Mcontinuity[i][i] = 1;
  }
  for (b_i = 0; b_i < 10; b_i++) {
    for (i = 0; i < 10; i++) {
      double d;
      d = 0.0;
      for (k = 0; k < 10; k++) {
        d += (double)(Mcontinuity[k][i] * M1[b_i][k]);
      }
      M[i][b_i] = d;
    }
  }
}

/*
 * Arguments    : const double constraints[8]
 *                double M[8][8]
 * Return Type  : void
 */
void constructM(const double constraints[8], double M[8][8])
{
  int freeBCIdx_data[8];
  int b_i;
  int fixedBCIdx_size;
  int i;
  int k;
  signed char M1[8][8];
  signed char Mcontinuity[8][8];
  bool b_fixedBCIdx_tmp[8];
  bool fixedBCIdx_tmp[8];
  for (i = 0; i < 8; i++) {
    for (b_i = 0; b_i < 8; b_i++) {
      Mcontinuity[i][b_i] = 0;
      M1[i][b_i] = 0;
    }
    bool b;
    b = rtIsNaN(constraints[i]);
    b_fixedBCIdx_tmp[i] = b;
    fixedBCIdx_tmp[i] = !b;
  }
  i = eml_find(fixedBCIdx_tmp, freeBCIdx_data);
  fixedBCIdx_size = i;
  for (k = 0; k < i; k++) {
    M1[k][freeBCIdx_data[k] - 1] = 1;
  }
  eml_find(b_fixedBCIdx_tmp, freeBCIdx_data);
  b_i = 8 - fixedBCIdx_size;
  for (i = 0; i < b_i; i++) {
    M1[fixedBCIdx_size + i][freeBCIdx_data[i] - 1] = 1;
  }
  for (i = 0; i < 8; i++) {
    Mcontinuity[i][i] = 1;
  }
  for (b_i = 0; b_i < 8; b_i++) {
    for (i = 0; i < 8; i++) {
      double d;
      d = 0.0;
      for (k = 0; k < 8; k++) {
        d += (double)(Mcontinuity[k][i] * M1[b_i][k]);
      }
      M[i][b_i] = d;
    }
  }
}

/*
 * File trailer for constructM.c
 *
 * [EOF]
 */
