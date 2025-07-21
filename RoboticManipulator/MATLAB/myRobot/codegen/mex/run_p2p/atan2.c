/*
 * atan2.c
 *
 * Code generation for function 'atan2'
 *
 */

/* Include files */
#include "atan2.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo qp_emlrtRSI = {
    13,      /* lineNo */
    "atan2", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elfun\\atan2.m" /* pathName
                                                                        */
};

static emlrtRSInfo rp_emlrtRSI = {
    66,      /* lineNo */
    "ixfun", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m" /* pathName
                                                                            */
};

static emlrtRSInfo sp_emlrtRSI = {
    45,                          /* lineNo */
    "applyBinaryScalarFunction", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\applyBinaryScalarFunction.m" /* pathName */
};

static emlrtRSInfo tp_emlrtRSI = {
    19,             /* lineNo */
    "scalexpAlloc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\scalexpAlloc."
    "m" /* pathName */
};

static emlrtRTEInfo kb_emlrtRTEI = {
    14,             /* lineNo */
    15,             /* colNo */
    "scalexpCheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\scalexpCheck."
    "m" /* pName */
};

/* Function Definitions */
void b_atan2(const emlrtStack *sp, const real_T y_data[],
             const int32_T y_size[3], const real_T x_data[],
             const int32_T x_size[3], real_T r_data[], int32_T r_size[3])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int8_T z_size_idx_2_tmp;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &qp_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_st.site = &rp_emlrtRSI;
  c_st.site = &sp_emlrtRSI;
  if (y_size[2] <= x_size[2]) {
    z_size_idx_2_tmp = (int8_T)y_size[2];
  } else {
    z_size_idx_2_tmp = 0;
  }
  d_st.site = &tp_emlrtRSI;
  p = true;
  if (z_size_idx_2_tmp == y_size[2]) {
    if (z_size_idx_2_tmp != x_size[2]) {
      p = false;
    }
  } else {
    p = false;
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(&d_st, &kb_emlrtRTEI, "MATLAB:dimagree",
                                  "MATLAB:dimagree", 0);
  }
  r_size[0] = 1;
  r_size[1] = 1;
  r_size[2] = z_size_idx_2_tmp;
  if (z_size_idx_2_tmp - 1 >= 0) {
    r_data[0] = muDoubleScalarAtan2(y_data[0], x_data[0]);
  }
}

/* End of code generation (atan2.c) */
