/*
 * find.c
 *
 * Code generation for function 'find'
 *
 */

/* Include files */
#include "find.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Variable Definitions */
static emlrtRSInfo ge_emlrtRSI = {
    138,        /* lineNo */
    "eml_find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

static emlrtRSInfo he_emlrtRSI = {
    396,                  /* lineNo */
    "find_first_indices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

/* Function Definitions */
int32_T b_eml_find(const emlrtStack *sp, const boolean_T x[10],
                   int32_T i_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_iv[2];
  int32_T i_size;
  int32_T ii;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ge_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 10)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 10) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  b_iv[0] = 1;
  b_iv[1] = i_size;
  b_st.site = &he_emlrtRSI;
  indexShapeCheck(&b_st, 10, b_iv);
  return i_size;
}

int32_T eml_find(const emlrtStack *sp, const boolean_T x[8], int32_T i_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_iv[2];
  int32_T i_size;
  int32_T ii;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ge_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 8)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 8) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  b_iv[0] = 1;
  b_iv[1] = i_size;
  b_st.site = &he_emlrtRSI;
  indexShapeCheck(&b_st, 8, b_iv);
  return i_size;
}

/* End of code generation (find.c) */
