/*
 * solvePoly.c
 *
 * Code generation for function 'solvePoly'
 *
 */

/* Include files */
#include "solvePoly.h"
#include "constructM.h"
#include "inv.h"
#include "mldivide.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void solvePoly(const emlrtStack *sp, const real_T constraints[8], real_T p[8])
{
  static const real_T QPrime[64] = {
      1120.0000000007876,   560.00000000075715,     100.00000000018457,
      3.333333333338242,    -1120.0000000007876,    560.000000000724,
      -100.0000000000714,   3.3333333333495787,     560.00000000040291,
      297.142857143245,     58.571428571525047,     2.0952380952402105,
      -560.00000000040291,  262.85714285752147,     -41.42857142860862,
      1.2380952381035719,   100.00000000008367,     58.571428571509045,
      14.285714285735004,   0.54761904761940183,    -100.00000000008367,
      41.428571428652958,   -5.7142857142936236,    0.11904761904934924,
      3.3333333333412725,   2.095238095245179,      0.54761904762106361,
      0.063492063492081918, -3.3333333333412725,    1.2380952381033552,
      -0.11904761904857253, -0.0079365079363525881, -1120.0000000007876,
      -560.00000000075715,  -100.00000000018457,    -3.333333333338242,
      1120.0000000007876,   -560.000000000724,      100.0000000000714,
      -3.3333333333495787,  560.00000000038472,     262.85714285751214,
      41.428571428659524,   1.2380952380980315,     -560.00000000038472,
      297.14285714320249,   -58.571428571462775,    2.0952380952460068,
      -100.00000000007481,  -41.428571428642613,    -5.7142857143022354,
      -0.11904761904831229, 100.00000000007481,     -58.571428571493016,
      14.285714285720871,   -0.54761904762056668,   3.3333333333392687,
      1.2380952381008115,   0.11904761904883188,    -0.0079365079364733526,
      -3.3333333333392687,  2.0952380952430332,     -0.54761904761955194,
      0.063492063492181339};
  static const real_T b_AInv[64] = {1.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    -34.999999999999467,
                                    83.999999999998792,
                                    -69.999999999999,
                                    19.999999999999716,
                                    0.0,
                                    1.0,
                                    0.0,
                                    0.0,
                                    -19.999999999999716,
                                    44.999999999999382,
                                    -35.9999999999995,
                                    9.9999999999998579,
                                    0.0,
                                    0.0,
                                    0.5,
                                    0.0,
                                    -4.9999999999999307,
                                    9.9999999999998685,
                                    -7.4999999999999014,
                                    1.9999999999999718,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.16666666666666666,
                                    -0.66666666666665519,
                                    0.99999999999998335,
                                    -0.66666666666665908,
                                    0.16666666666666446,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    34.999999999999467,
                                    -83.999999999998792,
                                    69.999999999999,
                                    -19.999999999999716,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    -14.999999999999753,
                                    38.99999999999941,
                                    -33.9999999999995,
                                    9.9999999999998579,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    2.4999999999999503,
                                    -6.9999999999998819,
                                    6.4999999999999014,
                                    -1.9999999999999718,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    -0.16666666666666266,
                                    0.49999999999999056,
                                    -0.49999999999999212,
                                    0.16666666666666441};
  static const real_T b[16] = {
      1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.16666666666666666};
  static const real_T dv[16] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 6.0};
  static const real_T dv1[16] = {1.0, 4.0, 12.0, 24.0,  1.0, 5.0, 20.0, 60.0,
                                 1.0, 6.0, 30.0, 120.0, 1.0, 7.0, 42.0, 210.0};
  static const real_T lowerright[16] = {
      34.999999999999467,   -83.999999999998792, 69.999999999999,
      -19.999999999999716,  -14.999999999999753, 38.99999999999941,
      -33.9999999999995,    9.9999999999998579,  2.4999999999999503,
      -6.9999999999998819,  6.4999999999999014,  -1.9999999999999718,
      -0.16666666666666266, 0.49999999999999056, -0.49999999999999212,
      0.16666666666666441};
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T b_R_data;
  emxArray_real_T constraints_data;
  emxArray_real_T *y;
  real_T M[64];
  real_T R_data[64];
  real_T R_tmp[64];
  real_T D_data[18];
  real_T DP_data[10];
  real_T AInv[8];
  real_T b_R_tmp[8];
  real_T d;
  int32_T R_size[2];
  int32_T R_tmp_tmp;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T k;
  int32_T nz;
  int32_T trueCount;
  int8_T tmp_data[8];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  nz = !muDoubleScalarIsNaN(constraints[0]) + 1;
  for (k = 0; k < 7; k++) {
    nz += !muDoubleScalarIsNaN(constraints[k + 1]);
  }
  st.site = &rd_emlrtRSI;
  constructM(&st, constraints, M);
  st.site = &sd_emlrtRSI;
  b_st.site = &ne_emlrtRSI;
  checkcond(&b_st, dv, b);
  st.site = &td_emlrtRSI;
  b_st.site = &ne_emlrtRSI;
  checkcond(&b_st, dv1, lowerright);
  for (i = 0; i < 8; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      R_tmp_tmp = i1 << 3;
      k = i + R_tmp_tmp;
      R_tmp[i1 + (i << 3)] = M[k];
      d = 0.0;
      for (i2 = 0; i2 < 8; i2++) {
        d += M[i + (i2 << 3)] * QPrime[i2 + R_tmp_tmp];
      }
      R_data[k] = d;
    }
  }
  for (i = 0; i < 8; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      d = 0.0;
      for (i2 = 0; i2 < 8; i2++) {
        d += R_data[i + (i2 << 3)] * R_tmp[i2 + (i1 << 3)];
      }
      M[i + (i1 << 3)] = d;
    }
  }
  if (nz > 8) {
    i = 0;
    i1 = 0;
    i2 = 0;
  } else {
    i = nz - 1;
    i1 = 8;
    i2 = nz - 1;
  }
  R_tmp_tmp = nz - 1;
  if (nz > 8) {
    i3 = -1;
    i4 = -1;
  } else {
    i3 = nz - 2;
    i4 = 7;
  }
  trueCount = 0;
  k = 0;
  for (b_i = 0; b_i < 8; b_i++) {
    if (!muDoubleScalarIsNaN(constraints[b_i])) {
      trueCount++;
      tmp_data[k] = (int8_T)b_i;
      k++;
    }
  }
  emxInit_real_T(sp, &y, 1, &qc_emlrtRTEI);
  st.site = &wd_emlrtRSI;
  b_st.site = &ke_emlrtRSI;
  if (nz - 1 != trueCount) {
    if (((nz - 1 == 1) && (i4 - i3 == 1)) || (trueCount == 1)) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &s_emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
  R_size[0] = nz - 1;
  nz = i4 - i3;
  R_size[1] = nz;
  for (i4 = 0; i4 < nz; i4++) {
    for (k = 0; k < R_tmp_tmp; k++) {
      R_data[k + R_size[0] * i4] = M[k + (((i3 + i4) + 1) << 3)];
    }
  }
  i4 = trueCount;
  for (i3 = 0; i3 < trueCount; i3++) {
    b_R_tmp[i3] = constraints[tmp_data[i3]];
  }
  b_R_data.data = &R_data[0];
  b_R_data.size = &R_size[0];
  b_R_data.allocatedSize = 64;
  b_R_data.numDimensions = 2;
  b_R_data.canFreeData = false;
  constraints_data.data = &b_R_tmp[0];
  constraints_data.size = &i4;
  constraints_data.allocatedSize = 8;
  constraints_data.numDimensions = 1;
  constraints_data.canFreeData = false;
  b_st.site = &je_emlrtRSI;
  mtimes(&b_st, &b_R_data, &constraints_data, y);
  nz = i1 - i;
  R_size[0] = nz;
  R_size[1] = nz;
  for (i1 = 0; i1 < nz; i1++) {
    k = (nz / 2) << 1;
    R_tmp_tmp = k - 2;
    for (i3 = 0; i3 <= R_tmp_tmp; i3 += 2) {
      __m128d r;
      r = _mm_loadu_pd(&M[(i + i3) + ((i2 + i1) << 3)]);
      _mm_storeu_pd(&R_data[i3 + R_size[0] * i1],
                    _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (i3 = k; i3 < nz; i3++) {
      R_data[i3 + R_size[0] * i1] = -M[(i + i3) + ((i2 + i1) << 3)];
    }
  }
  st.site = &wd_emlrtRSI;
  k = mldivide(&st, R_data, R_size, (real_T *)y->data,
               (*(int32_T(*)[1])y->size)[0], DP_data);
  emxFree_real_T(sp, &y);
  R_tmp_tmp = trueCount + k;
  for (i = 0; i < trueCount; i++) {
    D_data[i] = constraints[tmp_data[i]];
  }
  for (i = 0; i < k; i++) {
    D_data[i + trueCount] = DP_data[i];
  }
  st.site = &xd_emlrtRSI;
  b_st.site = &ke_emlrtRSI;
  if (R_tmp_tmp != 8) {
    if (R_tmp_tmp == 1) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &s_emlrtRTEI, "Coder:toolbox:mtimes_noDynamicScalarExpansion",
          "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                    "MATLAB:innerdim", 0);
    }
  }
  for (i = 0; i < 8; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d += R_tmp[i + (i1 << 3)] * D_data[i1];
    }
    b_R_tmp[i] = d;
  }
  for (i = 0; i < 8; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      d += b_AInv[i + (i1 << 3)] * b_R_tmp[i1];
    }
    AInv[i] = d;
  }
  for (i = 0; i < 8; i++) {
    p[i] = AInv[7 - i];
  }
  st.site = &yd_emlrtRSI;
  st.site = &yd_emlrtRSI;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (solvePoly.c) */
