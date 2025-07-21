/*
 * all.c
 *
 * Code generation for function 'all'
 *
 */

/* Include files */
#include "all.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Variable Definitions */
static emlrtRSInfo hp_emlrtRSI =
    {
        16,    /* lineNo */
        "all", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\all.m" /* pathName
                                                                        */
};

static emlrtRSInfo bq_emlrtRSI =
    {
        139,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

static emlrtRTEInfo lb_emlrtRTEI = {
    18,                               /* lineNo */
    27,                               /* colNo */
    "eml_int_forloop_overflow_check", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pName */
};

/* Function Definitions */
boolean_T all(const boolean_T x[2])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 1)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

boolean_T b_all(const boolean_T x[5])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 4)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

boolean_T c_all(const boolean_T x[3])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 2)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

void d_all(const emlrtStack *sp, const boolean_T x[48], boolean_T y[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i;
  int32_T i2;
  int32_T j;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &hp_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  for (i = 0; i < 16; i++) {
    y[i] = true;
  }
  i = 1;
  i2 = 32;
  for (j = 0; j < 16; j++) {
    int32_T i1_tmp;
    int32_T ix;
    boolean_T exitg1;
    i1_tmp = i;
    i++;
    i2++;
    b_st.site = &ig_emlrtRSI;
    if ((i1_tmp <= i2) && (i2 > 2147483631)) {
      c_st.site = &od_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    ix = i1_tmp;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      if (!x[ix - 1]) {
        y[i1_tmp - 1] = false;
        exitg1 = true;
      } else {
        ix += 16;
      }
    }
  }
}

int32_T e_all(const emlrtStack *sp, const boolean_T x_data[],
              const int32_T x_size[2], boolean_T y_data[])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i1;
  int32_T i2;
  int32_T j;
  int32_T y_size;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &hp_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  y_size = x_size[0];
  for (i2 = 0; i2 < y_size; i2++) {
    y_data[i2] = true;
  }
  i2 = (x_size[1] - 1) * x_size[0];
  i1 = 1;
  b_st.site = &bq_emlrtRSI;
  for (j = 0; j < y_size; j++) {
    int32_T i1_tmp;
    int32_T ix;
    boolean_T exitg1;
    boolean_T overflow;
    i1_tmp = i1;
    i1++;
    i2++;
    b_st.site = &ig_emlrtRSI;
    if ((y_size == 0) || (i1_tmp > i2)) {
      overflow = false;
    } else {
      overflow = (i2 > MAX_int32_T - y_size);
    }
    if (y_size == 0) {
      emlrtErrorWithMessageIdR2018a(&b_st, &lb_emlrtRTEI,
                                    "Coder:builtins:VectorStride",
                                    "Coder:builtins:VectorStride", 0);
    }
    if (overflow) {
      c_st.site = &od_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    ix = i1_tmp;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      if (!x_data[ix - 1]) {
        y_data[i1_tmp - 1] = false;
        exitg1 = true;
      } else {
        ix += y_size;
      }
    }
  }
  return y_size;
}

/* End of code generation (all.c) */
