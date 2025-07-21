/*
 * File: all.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "all.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const bool x[2]
 * Return Type  : bool
 */
bool all(const bool x[2])
{
  int k;
  bool exitg1;
  bool y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * Arguments    : const bool x[5]
 * Return Type  : bool
 */
bool b_all(const bool x[5])
{
  int k;
  bool exitg1;
  bool y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 5)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * Arguments    : const bool x[3]
 * Return Type  : bool
 */
bool c_all(const bool x[3])
{
  int k;
  bool exitg1;
  bool y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * Arguments    : bool x[3][16]
 *                bool y[16]
 * Return Type  : void
 */
void d_all(bool x[3][16], bool y[16])
{
  int k;
  for (k = 0; k < 16; k++) {
    int b_k;
    bool exitg1;
    y[k] = true;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 3)) {
      if (!x[b_k][k]) {
        y[k] = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }
}

/*
 * Arguments    : const bool x_data[]
 *                const int x_size[2]
 *                bool y_data[]
 * Return Type  : int
 */
int e_all(const bool x_data[], const int x_size[2], bool y_data[])
{
  int k;
  int y_size;
  y_size = x_size[0];
  for (k = 0; k < y_size; k++) {
    int b_k;
    bool exitg1;
    y_data[k] = true;
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k <= x_size[1] - 1)) {
      if (!x_data[k + x_size[0] * b_k]) {
        y_data[k] = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }
  return y_size;
}

/*
 * File trailer for all.c
 *
 * [EOF]
 */
