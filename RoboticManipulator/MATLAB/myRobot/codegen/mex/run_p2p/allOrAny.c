/*
 * allOrAny.c
 *
 * Code generation for function 'allOrAny'
 *
 */

/* Include files */
#include "allOrAny.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Definitions */
boolean_T allOrAny_anonFcn1(const boolean_T x_data[], int32_T x_size)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T varargout_1;
  varargout_1 = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x_size)) {
    if (x_data[ix - 1]) {
      varargout_1 = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return varargout_1;
}

boolean_T vectorAny(const boolean_T x_data[])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 1)) {
    if (x_data[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/* End of code generation (allOrAny.c) */
