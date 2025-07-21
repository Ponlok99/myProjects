/*
 * svd.c
 *
 * Code generation for function 'svd'
 *
 */

/* Include files */
#include "svd.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "svd1.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo av_emlrtRSI = {
    36,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo bv_emlrtRSI = {
    42,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m" /* pathName
                                                                       */
};

/* Function Definitions */
void svd(const emlrtStack *sp, const real_T A_data[], const int32_T A_size[2],
         real_T U[36], real_T S_data[], int32_T S_size[2], real_T V_data[],
         int32_T V_size[2])
{
  emlrtStack st;
  real_T V1_data[2401];
  real_T tmp_data[294];
  real_T s_data[6];
  int32_T tmp_size[2];
  int32_T i;
  int32_T k;
  int32_T s_size;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  i = 6 * A_size[1];
  p = true;
  for (k = 0; k < i; k++) {
    if (p) {
      real_T d;
      d = A_data[k];
      if (muDoubleScalarIsInf(d) || muDoubleScalarIsNaN(d)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (p) {
    st.site = &av_emlrtRSI;
    s_size = b_svd(&st, A_data, A_size, U, s_data, V_data, V_size);
  } else {
    real_T U1[36];
    tmp_size[0] = 6;
    tmp_size[1] = (int8_T)A_size[1];
    k = 6 * (int8_T)A_size[1];
    if (k - 1 >= 0) {
      memset(&tmp_data[0], 0, (uint32_T)k * sizeof(real_T));
    }
    st.site = &bv_emlrtRSI;
    s_size = b_svd(&st, tmp_data, tmp_size, U1, s_data, V1_data, V_size);
    for (i = 0; i < 36; i++) {
      U[i] = rtNaN;
    }
    for (i = 0; i < s_size; i++) {
      s_data[i] = rtNaN;
    }
    k = V_size[0] * V_size[1];
    for (i = 0; i < k; i++) {
      V_data[i] = rtNaN;
    }
  }
  S_size[0] = 6;
  S_size[1] = V_size[1];
  k = 6 * V_size[1];
  if (k - 1 >= 0) {
    memset(&S_data[0], 0, (uint32_T)k * sizeof(real_T));
  }
  for (k = 0; k < s_size; k++) {
    S_data[k + 6 * k] = s_data[k];
  }
}

/* End of code generation (svd.c) */
