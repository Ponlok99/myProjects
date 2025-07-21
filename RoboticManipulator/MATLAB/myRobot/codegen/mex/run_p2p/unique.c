/*
 * unique.c
 *
 * Code generation for function 'unique'
 *
 */

/* Include files */
#include "unique.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "sortLE.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo mb_emlrtRTEI = {
    331,           /* lineNo */
    1,             /* colNo */
    "unique_rows", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

/* Function Definitions */
int32_T unique_rows(const emlrtStack *sp, const real_T a_data[],
                    const int32_T a_size[2], real_T b_data[], int32_T b_size[2],
                    int32_T ndx_data[])
{
  int32_T idx_data[32];
  int32_T iwork_data[32];
  int32_T col_size[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T k;
  int32_T ndx_size;
  int32_T pEnd;
  int32_T qEnd;
  if (a_size[0] == 0) {
    b_size[0] = 0;
    b_size[1] = a_size[1];
    ndx_size = 0;
  } else {
    real_T ycol_data[32];
    int32_T col_data[6];
    int32_T b_n;
    int32_T i2;
    int32_T loop_ub;
    int32_T n;
    loop_ub = a_size[0];
    b_size[0] = a_size[0];
    n = a_size[1];
    b_size[1] = a_size[1];
    i2 = a_size[0] * a_size[1];
    if (i2 - 1 >= 0) {
      memcpy(&b_data[0], &a_data[0], (uint32_T)i2 * sizeof(real_T));
    }
    col_size[0] = 1;
    col_size[1] = a_size[1];
    i2 = (a_size[1] / 4) << 2;
    pEnd = i2 - 4;
    for (k = 0; k <= pEnd; k += 4) {
      _mm_storeu_si128(
          (__m128i *)&col_data[0],
          _mm_add_epi32(
              _mm_add_epi32(_mm_set1_epi32(0),
                            _mm_loadu_si128((const __m128i *)&iv2[0])),
              _mm_set1_epi32(1)));
    }
    for (k = i2; k < n; k++) {
      col_data[k] = k + 1;
    }
    b_n = a_size[0] + 1;
    ndx_size = a_size[0];
    memset(&idx_data[0], 0, (uint32_T)loop_ub * sizeof(int32_T));
    if (a_size[1] == 0) {
      i2 = (a_size[0] / 4) << 2;
      pEnd = i2 - 4;
      for (k = 0; k <= pEnd; k += 4) {
        _mm_storeu_si128(
            (__m128i *)&idx_data[k],
            _mm_add_epi32(
                _mm_add_epi32(_mm_set1_epi32(k),
                              _mm_loadu_si128((const __m128i *)&iv2[0])),
                _mm_set1_epi32(1)));
      }
      for (k = i2; k <= b_n - 2; k++) {
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
      if (((uint32_T)a_size[0] & 1U) != 0U) {
        idx_data[a_size[0] - 1] = a_size[0];
      }
      b_i = 2;
      while (b_i < b_n - 1) {
        i2 = b_i << 1;
        j = 1;
        for (pEnd = b_i + 1; pEnd < b_n; pEnd = qEnd + b_i) {
          int32_T kEnd;
          int32_T p;
          int32_T q;
          p = j;
          q = pEnd;
          qEnd = j + i2;
          if (qEnd > b_n) {
            qEnd = b_n;
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
    for (j = 0; j < n; j++) {
      for (b_i = 0; b_i < loop_ub; b_i++) {
        ycol_data[b_i] = b_data[(idx_data[b_i] + b_size[0] * j) - 1];
      }
      for (b_i = 0; b_i < loop_ub; b_i++) {
        b_data[b_i + b_size[0] * j] = ycol_data[b_i];
      }
    }
    for (i = 0; i < ndx_size; i++) {
      ycol_data[i] = idx_data[i];
    }
    ndx_size = 0;
    k = 0;
    while (k + 1 <= loop_ub) {
      pEnd = k;
      int32_T exitg1;
      do {
        exitg1 = 0;
        k++;
        if (k + 1 > loop_ub) {
          exitg1 = 1;
        } else {
          boolean_T b_p;
          boolean_T exitg2;
          b_p = false;
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j <= b_size[1] - 1)) {
            i = b_size[0] * j;
            if (b_data[pEnd + i] != b_data[k + i]) {
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
      ndx_size++;
      for (j = 0; j < n; j++) {
        i2 = b_size[0] * j;
        b_data[(ndx_size + i2) - 1] = b_data[pEnd + i2];
      }
      ycol_data[ndx_size - 1] = ycol_data[pEnd];
    }
    if (ndx_size > a_size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &mb_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
    if (ndx_size < 1) {
      loop_ub = 0;
    } else {
      loop_ub = ndx_size;
    }
    for (i = 0; i < n; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        b_data[i1 + loop_ub * i] = b_data[i1 + b_size[0] * i];
      }
    }
    b_size[0] = loop_ub;
    i = (uint8_T)ndx_size;
    for (k = 0; k < i; k++) {
      ndx_data[k] = (int32_T)ycol_data[k];
    }
  }
  return ndx_size;
}

/* End of code generation (unique.c) */
