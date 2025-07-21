/*
 * wrapToPi.c
 *
 * Code generation for function 'wrapToPi'
 *
 */

/* Include files */
#include "wrapToPi.h"
#include "allOrAny.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void b_wrapToPi(real_T theta[3])
{
  int32_T k;
  boolean_T x_data[6];
  boolean_T exitg1;
  boolean_T y;
  x_data[0] = (muDoubleScalarAbs(theta[0]) > 3.1415926535897931);
  x_data[1] = (muDoubleScalarAbs(theta[1]) > 3.1415926535897931);
  x_data[2] = (muDoubleScalarAbs(theta[2]) > 3.1415926535897931);
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 2)) {
    if (x_data[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (y) {
    __m128d r;
    real_T y_idx_0;
    real_T y_idx_1;
    real_T y_idx_2;
    y_idx_2 = theta[0] + 3.1415926535897931;
    y_idx_0 = y_idx_2;
    theta[0] = b_mod(y_idx_2);
    y_idx_2 = theta[1] + 3.1415926535897931;
    y_idx_1 = y_idx_2;
    theta[1] = b_mod(y_idx_2);
    y_idx_2 = theta[2] + 3.1415926535897931;
    theta[2] = b_mod(y_idx_2);
    if ((theta[0] == 0.0) && (y_idx_0 > 0.0)) {
      theta[0] = 6.2831853071795862;
    }
    if ((theta[1] == 0.0) && (y_idx_1 > 0.0)) {
      theta[1] = 6.2831853071795862;
    }
    if ((theta[2] == 0.0) && (y_idx_2 > 0.0)) {
      theta[2] = 6.2831853071795862;
    }
    r = _mm_loadu_pd(&theta[0]);
    _mm_storeu_pd(&theta[0], _mm_sub_pd(r, _mm_set1_pd(3.1415926535897931)));
    theta[2] -= 3.1415926535897931;
  }
}

void c_wrapToPi(const emlrtStack *sp, real_T theta_data[],
                const int32_T theta_size[2])
{
  emlrtStack st;
  real_T y_data[192];
  int32_T i;
  int32_T i1;
  int32_T k;
  int32_T loop_ub;
  int32_T y_size_idx_1;
  boolean_T x_data[192];
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  loop_ub = theta_size[0] * theta_size[1];
  for (k = 0; k < loop_ub; k++) {
    y_data[k] = muDoubleScalarAbs(theta_data[k]);
  }
  k = (int8_T)theta_size[0] * (int8_T)theta_size[1];
  for (i = 0; i < k; i++) {
    x_data[i] = (y_data[i] > 3.1415926535897931);
  }
  y = allOrAny_anonFcn1(x_data, k);
  if (y) {
    __m128d r;
    int32_T scalarLB;
    int32_T vectorUB;
    st.site = &dq_emlrtRSI;
    k = theta_size[0];
    y_size_idx_1 = theta_size[1];
    scalarLB = (loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&theta_data[i]);
      _mm_storeu_pd(&y_data[i], _mm_add_pd(r, _mm_set1_pd(3.1415926535897931)));
    }
    for (i = scalarLB; i < loop_ub; i++) {
      y_data[i] = theta_data[i] + 3.1415926535897931;
    }
    for (i = 0; i < y_size_idx_1; i++) {
      for (i1 = 0; i1 < k; i1++) {
        theta_data[i1 + theta_size[0] * i] = b_mod(y_data[i1 + k * i]);
      }
    }
    k = theta_size[0] * theta_size[1];
    for (y_size_idx_1 = 0; y_size_idx_1 < k; y_size_idx_1++) {
      if ((theta_data[y_size_idx_1] == 0.0) && (y_data[y_size_idx_1] > 0.0)) {
        if (y_size_idx_1 > k - 1) {
          emlrtDynamicBoundsCheckR2012b(y_size_idx_1, 0, k - 1, &gc_emlrtBCI,
                                        &st);
        }
        theta_data[y_size_idx_1] = 6.2831853071795862;
      }
    }
    loop_ub = theta_size[1];
    for (i = 0; i < loop_ub; i++) {
      k = theta_size[0];
      scalarLB = (k / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i1 = 0; i1 <= vectorUB; i1 += 2) {
        y_size_idx_1 = i1 + theta_size[0] * i;
        r = _mm_loadu_pd(&theta_data[y_size_idx_1]);
        _mm_storeu_pd(&theta_data[y_size_idx_1],
                      _mm_sub_pd(r, _mm_set1_pd(3.1415926535897931)));
      }
      for (i1 = scalarLB; i1 < k; i1++) {
        y_size_idx_1 = i1 + theta_size[0] * i;
        theta_data[y_size_idx_1] -= 3.1415926535897931;
      }
    }
  }
}

void wrapToPi(real_T *theta)
{
  if (muDoubleScalarAbs(*theta) > 3.1415926535897931) {
    real_T thetaWrap;
    thetaWrap = b_mod(*theta + 3.1415926535897931);
    if ((thetaWrap == 0.0) && (*theta + 3.1415926535897931 > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }
    *theta = thetaWrap - 3.1415926535897931;
  }
}

/* End of code generation (wrapToPi.c) */
