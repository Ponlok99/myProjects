/*
 * ppval.c
 *
 * Code generation for function 'ppval'
 *
 */

/* Include files */
#include "ppval.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void b_ppval(const real_T pp_breaks[4], const real_T pp_coefs[54],
             real_T v[303])
{
  int32_T ix;
  int32_T low_ip1;
  for (ix = 0; ix < 101; ix++) {
    real_T xloc;
    int32_T high_i;
    int32_T icp;
    int32_T iv0;
    int32_T low_i;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      icp = (low_i + high_i) >> 1;
      if (0.01 * (real_T)ix >= pp_breaks[icp - 1]) {
        low_i = icp;
        low_ip1 = icp + 1;
      } else {
        high_i = icp;
      }
    }
    icp = (low_i - 1) * 3;
    xloc = 0.01 * (real_T)ix - pp_breaks[low_i - 1];
    v[iv0] = pp_coefs[icp];
    v[iv0 + 1] = pp_coefs[icp + 1];
    v[iv0 + 2] = pp_coefs[icp + 2];
    for (low_ip1 = 0; low_ip1 < 5; low_ip1++) {
      __m128d r;
      high_i = icp + (low_ip1 + 1) * 9;
      r = _mm_loadu_pd(&v[iv0]);
      _mm_storeu_pd(&v[iv0], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(xloc), r),
                                        _mm_loadu_pd(&pp_coefs[high_i])));
      v[iv0 + 2] = xloc * v[iv0 + 2] + pp_coefs[high_i + 2];
    }
  }
}

void c_ppval(const real_T pp_breaks[4], const real_T pp_coefs[36],
             real_T v[303])
{
  int32_T ix;
  for (ix = 0; ix < 101; ix++) {
    int32_T high_i;
    int32_T iv0;
    int32_T low_i;
    int32_T low_ip1;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      int32_T mid_i;
      mid_i = (low_i + high_i) >> 1;
      if (0.01 * (real_T)ix >= pp_breaks[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    __m128d r;
    __m128d r1;
    real_T xloc;
    low_ip1 = (low_i - 1) * 3;
    xloc = 0.01 * (real_T)ix - pp_breaks[low_i - 1];
    v[iv0] = pp_coefs[low_ip1];
    v[iv0 + 1] = pp_coefs[low_ip1 + 1];
    v[iv0 + 2] = pp_coefs[low_ip1 + 2];
    r = _mm_loadu_pd(&v[iv0]);
    r1 = _mm_set1_pd(xloc);
    _mm_storeu_pd(&v[iv0], _mm_add_pd(_mm_mul_pd(r1, r),
                                      _mm_loadu_pd(&pp_coefs[low_ip1 + 9])));
    v[iv0 + 2] = xloc * v[iv0 + 2] + pp_coefs[low_ip1 + 11];
    r = _mm_loadu_pd(&v[iv0]);
    _mm_storeu_pd(&v[iv0], _mm_add_pd(_mm_mul_pd(r1, r),
                                      _mm_loadu_pd(&pp_coefs[low_ip1 + 18])));
    v[iv0 + 2] = xloc * v[iv0 + 2] + pp_coefs[low_ip1 + 20];
    r = _mm_loadu_pd(&v[iv0]);
    _mm_storeu_pd(&v[iv0], _mm_add_pd(_mm_mul_pd(r1, r),
                                      _mm_loadu_pd(&pp_coefs[low_ip1 + 27])));
    v[iv0 + 2] = xloc * v[iv0 + 2] + pp_coefs[low_ip1 + 29];
  }
}

void ppval(const real_T pp_breaks[6], const real_T pp_coefs_data[],
           const int32_T pp_coefs_size[3], const real_T x[101], real_T v_data[],
           int32_T v_size[2])
{
  int32_T coefStride_tmp;
  int32_T elementsPerPage_tmp;
  int32_T ic;
  int32_T ix;
  int32_T j;
  elementsPerPage_tmp = pp_coefs_size[0];
  coefStride_tmp = pp_coefs_size[0] * 5;
  v_size[0] = pp_coefs_size[0];
  v_size[1] = 101;
  if (pp_coefs_size[0] == 1) {
    for (ix = 0; ix < 101; ix++) {
      real_T xloc;
      if (muDoubleScalarIsNaN(x[ix])) {
        xloc = rtNaN;
      } else {
        int32_T high_i;
        int32_T low_i;
        int32_T low_ip1;
        low_i = 0;
        low_ip1 = 2;
        high_i = 6;
        while (high_i > low_ip1) {
          int32_T ic0;
          ic0 = ((low_i + high_i) + 1) >> 1;
          if (x[ix] >= pp_breaks[ic0 - 1]) {
            low_i = ic0 - 1;
            low_ip1 = ic0 + 1;
          } else {
            high_i = ic0;
          }
        }
        xloc = x[ix] - pp_breaks[low_i];
        xloc = xloc * (xloc * pp_coefs_data[low_i] +
                       pp_coefs_data[low_i + coefStride_tmp]) +
               pp_coefs_data[low_i + (coefStride_tmp << 1)];
      }
      v_data[ix] = xloc;
    }
  } else {
    for (ix = 0; ix < 101; ix++) {
      int32_T iv0;
      iv0 = ix * elementsPerPage_tmp;
      if (muDoubleScalarIsNaN(x[ix])) {
        for (j = 0; j < elementsPerPage_tmp; j++) {
          v_data[iv0 + j] = x[ix];
        }
      } else {
        real_T xloc;
        int32_T high_i;
        int32_T ic0;
        int32_T icp;
        int32_T low_i;
        int32_T low_ip1;
        low_i = 1;
        low_ip1 = 2;
        high_i = 6;
        while (high_i > low_ip1) {
          ic0 = (low_i + high_i) >> 1;
          if (x[ix] >= pp_breaks[ic0 - 1]) {
            low_i = ic0;
            low_ip1 = ic0 + 1;
          } else {
            high_i = ic0;
          }
        }
        icp = (low_i - 1) * elementsPerPage_tmp;
        xloc = x[ix] - pp_breaks[low_i - 1];
        for (j = 0; j < elementsPerPage_tmp; j++) {
          v_data[iv0 + j] = pp_coefs_data[icp + j];
        }
        low_ip1 = (elementsPerPage_tmp / 2) << 1;
        high_i = low_ip1 - 2;
        for (ic = 0; ic < 2; ic++) {
          ic0 = icp + (ic + 1) * coefStride_tmp;
          for (j = 0; j <= high_i; j += 2) {
            __m128d r;
            r = _mm_loadu_pd(&v_data[iv0]);
            _mm_storeu_pd(&v_data[iv0],
                          _mm_add_pd(_mm_mul_pd(_mm_set1_pd(xloc), r),
                                     _mm_loadu_pd(&pp_coefs_data[ic0])));
          }
          for (j = low_ip1; j < elementsPerPage_tmp; j++) {
            low_i = iv0 + j;
            v_data[low_i] = xloc * v_data[low_i] + pp_coefs_data[ic0 + j];
          }
        }
      }
    }
  }
}

/* End of code generation (ppval.c) */
