/*
 * File: sort.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "sort.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "rt_nonfinite.h"

/* Type Definitions */
#ifndef struct_emxArray_int32_T_32
#define struct_emxArray_int32_T_32
struct emxArray_int32_T_32 {
  int data[32];
  int size[1];
};
#endif /* struct_emxArray_int32_T_32 */
#ifndef typedef_emxArray_int32_T_32
#define typedef_emxArray_int32_T_32
typedef struct emxArray_int32_T_32 emxArray_int32_T_32;
#endif /* typedef_emxArray_int32_T_32 */

/* Function Definitions */
/*
 * Arguments    : double x_data[]
 *                const int *x_size
 * Return Type  : void
 */
void b_sort(double x_data[], const int *x_size)
{
  emxArray_int32_T_32 b_vwork_data;
  double vwork_data[32];
  int dim;
  int k;
  int vstride;
  int vwork_size;
  dim = 2;
  if (*x_size != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vstride = 1;
  dim -= 2;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (dim = 0; dim < vstride; dim++) {
    for (k = 0; k < vwork_size; k++) {
      vwork_data[k] = x_data[dim + k * vstride];
    }
    b_vwork_data.size[0] = sortIdx(vwork_data, &vwork_size, b_vwork_data.data);
    for (k = 0; k < vwork_size; k++) {
      x_data[dim + k * vstride] = vwork_data[k];
    }
  }
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 *                int idx_data[]
 * Return Type  : int
 */
int c_sort(double x_data[], const int *x_size, int idx_data[])
{
  double vwork_data[32];
  int iidx_data[32];
  int dim;
  int idx_size;
  int j;
  int k;
  int vstride;
  int vwork_size;
  dim = 2;
  if (*x_size != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  idx_size = *x_size;
  vstride = 1;
  dim -= 2;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (j = 0; j < vstride; j++) {
    for (k = 0; k < vwork_size; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, iidx_data);
    for (k = 0; k < vwork_size; k++) {
      dim = j + k * vstride;
      x_data[dim] = vwork_data[k];
      idx_data[dim] = iidx_data[k];
    }
  }
  return idx_size;
}

/*
 * Arguments    : double x[2]
 * Return Type  : void
 */
void sort(double x[2])
{
  double x4[4];
  double xwork[2];
  int ib;
  int k;
  int nNaNs;
  x4[0] = 0.0;
  x4[1] = 0.0;
  x4[2] = 0.0;
  x4[3] = 0.0;
  nNaNs = 0;
  ib = 0;
  if (rtIsNaN(x[0])) {
    xwork[1] = x[0];
    nNaNs = 1;
  } else {
    ib = 1;
    x4[0] = x[0];
  }
  if (rtIsNaN(x[1])) {
    xwork[1 - nNaNs] = x[1];
    nNaNs++;
  } else {
    ib++;
    x4[ib - 1] = x[1];
  }
  if (ib > 0) {
    signed char perm[4];
    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    if (ib == 1) {
      perm[0] = 1;
    } else if (x4[0] <= x4[1]) {
      perm[0] = 1;
      perm[1] = 2;
    } else {
      perm[0] = 2;
      perm[1] = 1;
    }
    for (k = 0; k < ib; k++) {
      x[((k - nNaNs) - ib) + 2] = x4[perm[k] - 1];
    }
  }
  ib = (nNaNs >> 1) + 2;
  for (k = 0; k <= ib - 3; k++) {
    x[2 - nNaNs] = xwork[1];
    x[1] = xwork[2 - nNaNs];
  }
  if (((unsigned int)nNaNs & 1U) != 0U) {
    ib -= nNaNs;
    x[ib] = xwork[ib];
  }
}

/*
 * File trailer for sort.c
 *
 * [EOF]
 */
