/*
 * sort.c
 *
 * Code generation for function 'sort'
 *
 */

/* Include files */
#include "sort.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "sortIdx.h"
#include "mwmathutil.h"

/* Type Definitions */
#ifndef struct_emxArray_int32_T_32
#define struct_emxArray_int32_T_32
struct emxArray_int32_T_32 {
  int32_T data[32];
};
#endif /* struct_emxArray_int32_T_32 */
#ifndef typedef_emxArray_int32_T_32
#define typedef_emxArray_int32_T_32
typedef struct emxArray_int32_T_32 emxArray_int32_T_32;
#endif /* typedef_emxArray_int32_T_32 */

/* Variable Definitions */
static emlrtRSInfo hr_emlrtRSI = {
    76,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo ir_emlrtRSI = {
    79,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo jr_emlrtRSI = {
    81,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo kr_emlrtRSI = {
    84,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo lr_emlrtRSI = {
    87,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

static emlrtRSInfo mr_emlrtRSI = {
    90,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m" /* pathName
                                                                           */
};

/* Function Definitions */
void b_sort(const emlrtStack *sp, real_T x_data[], const int32_T *x_size)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_int32_T_32 uv_emlrtRSI;
  real_T vwork_data[32];
  int32_T dim;
  int32_T k;
  int32_T vstride;
  int32_T vwork_size;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  dim = 2;
  if (*x_size != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  st.site = &hr_emlrtRSI;
  vstride = 1;
  dim -= 2;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  st.site = &ir_emlrtRSI;
  st.site = &jr_emlrtRSI;
  if (vstride > 2147483646) {
    b_st.site = &od_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (dim = 0; dim < vstride; dim++) {
    st.site = &kr_emlrtRSI;
    for (k = 0; k < vwork_size; k++) {
      vwork_data[k] = x_data[dim + k * vstride];
    }
    st.site = &lr_emlrtRSI;
    sortIdx(&st, vwork_data, &vwork_size, uv_emlrtRSI.data);
    st.site = &mr_emlrtRSI;
    for (k = 0; k < vwork_size; k++) {
      x_data[dim + k * vstride] = vwork_data[k];
    }
  }
}

int32_T c_sort(const emlrtStack *sp, real_T x_data[], const int32_T *x_size,
               int32_T idx_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T vwork_data[32];
  int32_T iidx_data[32];
  int32_T dim;
  int32_T idx_size;
  int32_T j;
  int32_T k;
  int32_T vstride;
  int32_T vwork_size;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  dim = 2;
  if (*x_size != 1) {
    dim = 1;
  }
  if (dim <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  idx_size = *x_size;
  st.site = &hr_emlrtRSI;
  vstride = 1;
  dim -= 2;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  st.site = &ir_emlrtRSI;
  st.site = &jr_emlrtRSI;
  if (vstride > 2147483646) {
    b_st.site = &od_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (j = 0; j < vstride; j++) {
    st.site = &kr_emlrtRSI;
    for (k = 0; k < vwork_size; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    st.site = &lr_emlrtRSI;
    sortIdx(&st, vwork_data, &vwork_size, iidx_data);
    st.site = &mr_emlrtRSI;
    for (k = 0; k < vwork_size; k++) {
      dim = j + k * vstride;
      x_data[dim] = vwork_data[k];
      idx_data[dim] = iidx_data[k];
    }
  }
  return idx_size;
}

void sort(real_T x[2])
{
  if ((!(x[0] <= x[1])) && (!muDoubleScalarIsNaN(x[1]))) {
    real_T tmp;
    tmp = x[0];
    x[0] = x[1];
    x[1] = tmp;
  }
}

/* End of code generation (sort.c) */
