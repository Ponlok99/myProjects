/*
 * File: histcounts.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "histcounts.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double x[101]
 *                const emxArray_real_T *varargin_1
 *                emxArray_real_T *n
 *                double bin[101]
 * Return Type  : void
 */
void histcounts(const double x[101], const emxArray_real_T *varargin_1,
                emxArray_real_T *n, double bin[101])
{
  emxArray_int32_T *b_n;
  const double *varargin_1_data;
  double delta;
  double leftEdge;
  double *b_n_data;
  int b_bin[101];
  int high_i;
  int k;
  int unnamed_idx_1_tmp;
  int *n_data;
  varargin_1_data = varargin_1->data;
  unnamed_idx_1_tmp = varargin_1->size[1] - 1;
  emxInit_int32_T(&b_n, 2);
  high_i = b_n->size[0] * b_n->size[1];
  b_n->size[0] = 1;
  b_n->size[1] = unnamed_idx_1_tmp;
  emxEnsureCapacity_int32_T(b_n, high_i);
  n_data = b_n->data;
  for (high_i = 0; high_i < unnamed_idx_1_tmp; high_i++) {
    n_data[high_i] = 0;
  }
  leftEdge = varargin_1_data[0];
  delta = varargin_1_data[1] - varargin_1_data[0];
  for (k = 0; k < 101; k++) {
    double d;
    b_bin[k] = 0;
    d = x[k];
    if ((d >= leftEdge) && (d <= varargin_1_data[unnamed_idx_1_tmp])) {
      double bGuess;
      bool guard1;
      bGuess = ceil((d - leftEdge) / delta);
      guard1 = false;
      if ((bGuess >= 1.0) && (bGuess < varargin_1->size[1])) {
        high_i = (int)bGuess;
        if ((d >= varargin_1_data[high_i - 1]) &&
            (d < varargin_1_data[high_i])) {
          n_data[high_i - 1]++;
          b_bin[k] = high_i;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        int low_i;
        int low_ip1;
        high_i = varargin_1->size[1];
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          int mid_i;
          mid_i = (low_i >> 1) + (high_i >> 1);
          if ((((unsigned int)low_i & 1U) == 1U) &&
              (((unsigned int)high_i & 1U) == 1U)) {
            mid_i++;
          }
          if (x[k] >= varargin_1_data[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }
        n_data[low_i - 1]++;
        b_bin[k] = low_i;
      }
    }
    bin[k] = b_bin[k];
  }
  high_i = n->size[0] * n->size[1];
  n->size[0] = 1;
  n->size[1] = unnamed_idx_1_tmp;
  emxEnsureCapacity_real_T(n, high_i);
  b_n_data = n->data;
  for (high_i = 0; high_i < unnamed_idx_1_tmp; high_i++) {
    b_n_data[high_i] = n_data[high_i];
  }
  emxFree_int32_T(&b_n);
}

/*
 * File trailer for histcounts.c
 *
 * [EOF]
 */
