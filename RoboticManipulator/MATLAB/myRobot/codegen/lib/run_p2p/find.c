/*
 * File: find.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "find.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const bool x[10]
 *                int i_data[]
 * Return Type  : int
 */
int b_eml_find(const bool x[10], int i_data[])
{
  int i_size;
  int ii;
  bool exitg1;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 10)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 10) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  return i_size;
}

/*
 * Arguments    : const bool x[8]
 *                int i_data[]
 * Return Type  : int
 */
int eml_find(const bool x[8], int i_data[])
{
  int i_size;
  int ii;
  bool exitg1;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 8)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 8) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  return i_size;
}

/*
 * File trailer for find.c
 *
 * [EOF]
 */
