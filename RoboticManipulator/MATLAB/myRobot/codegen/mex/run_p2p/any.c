/*
 * any.c
 *
 * Code generation for function 'any'
 *
 */

/* Include files */
#include "any.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"

/* Variable Definitions */
static emlrtRSInfo qg_emlrtRSI =
    {
        13,    /* lineNo */
        "any", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\any.m" /* pathName
                                                                        */
};

/* Function Definitions */
boolean_T any(const emlrtStack *sp, const emxArray_boolean_T *x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T ix;
  const boolean_T *x_data;
  boolean_T exitg1;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  st.site = &qg_emlrtRSI;
  y = false;
  b_st.site = &ig_emlrtRSI;
  if (x->size[0] > 2147483646) {
    c_st.site = &od_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    if (x_data[ix - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

/* End of code generation (any.c) */
