/*
 * histcounts.c
 *
 * Code generation for function 'histcounts'
 *
 */

/* Include files */
#include "histcounts.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ah_emlrtRSI =
    {
        27,           /* lineNo */
        "histcounts", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\histcounts."
        "m" /* pathName */
};

static emlrtRSInfo bh_emlrtRSI =
    {
        87,           /* lineNo */
        "parseinput", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\histcounts."
        "m" /* pathName */
};

static emlrtRTEInfo dc_emlrtRTEI =
    {
        85,           /* lineNo */
        31,           /* colNo */
        "parseinput", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\histcounts."
        "m" /* pName */
};

static emlrtRTEInfo ec_emlrtRTEI = {
    13,                      /* lineNo */
    37,                      /* colNo */
    "validatenondecreasing", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenondecreasing.m" /* pName */
};

static emlrtRTEInfo ad_emlrtRTEI = {
    21,                  /* lineNo */
    1,                   /* colNo */
    "mapElementsToBins", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\mapEle"
    "mentsToBins.m" /* pName */
};

static emlrtRTEInfo bd_emlrtRTEI =
    {
        58,           /* lineNo */
        1,            /* colNo */
        "histcounts", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\histcounts."
        "m" /* pName */
};

static emlrtRTEInfo cd_emlrtRTEI =
    {
        1,            /* lineNo */
        26,           /* colNo */
        "histcounts", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\histcounts."
        "m" /* pName */
};

/* Function Definitions */
void histcounts(const emlrtStack *sp, const real_T x[101],
                const emxArray_real_T *varargin_1, emxArray_real_T *n,
                real_T bin[101])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_int32_T *b_n;
  const real_T *varargin_1_data;
  real_T delta;
  real_T leftEdge;
  real_T *b_n_data;
  int32_T b_bin[101];
  int32_T high_i;
  int32_T k;
  int32_T loop_ub;
  int32_T *n_data;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  varargin_1_data = varargin_1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ah_emlrtRSI;
  if (varargin_1->size[1] < 2) {
    emlrtErrorWithMessageIdR2018a(&st, &dc_emlrtRTEI,
                                  "MATLAB:histcounts:EmptyOrScalarBinEdges",
                                  "MATLAB:histcounts:EmptyOrScalarBinEdges", 0);
  }
  b_st.site = &bh_emlrtRSI;
  c_st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= varargin_1->size[1] - 2)) {
    if (!(varargin_1_data[k] <= varargin_1_data[k + 1])) {
      p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &ec_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonDecreasing",
        "MATLAB:histcounts:expectedNonDecreasing", 3, 4, 22,
        "input number 2, edges,");
  }
  emxInit_int32_T(sp, &b_n, 2, &cd_emlrtRTEI);
  high_i = b_n->size[0] * b_n->size[1];
  b_n->size[0] = 1;
  loop_ub = varargin_1->size[1] - 1;
  b_n->size[1] = loop_ub;
  emxEnsureCapacity_int32_T(sp, b_n, high_i, &ad_emlrtRTEI);
  n_data = b_n->data;
  for (high_i = 0; high_i < loop_ub; high_i++) {
    n_data[high_i] = 0;
  }
  leftEdge = varargin_1_data[0];
  delta = varargin_1_data[1] - varargin_1_data[0];
  for (k = 0; k < 101; k++) {
    real_T d;
    b_bin[k] = 0;
    d = x[k];
    if ((d >= leftEdge) && (d <= varargin_1_data[loop_ub])) {
      real_T bGuess;
      boolean_T guard1;
      bGuess = muDoubleScalarCeil((d - leftEdge) / delta);
      guard1 = false;
      if ((bGuess >= 1.0) && (bGuess < varargin_1->size[1])) {
        high_i = (int32_T)bGuess;
        if ((d >= varargin_1_data[high_i - 1]) &&
            (d < varargin_1_data[high_i])) {
          n_data[high_i - 1]++;
          b_bin[k] = high_i;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        int32_T low_i;
        int32_T low_ip1;
        high_i = varargin_1->size[1];
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          int32_T mid_i;
          mid_i = (low_i >> 1) + (high_i >> 1);
          if ((((uint32_T)low_i & 1U) == 1U) &&
              (((uint32_T)high_i & 1U) == 1U)) {
            mid_i++;
          }
          if (x[k] >= varargin_1_data[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }
        n_data[low_i - 1]++;
        b_bin[k] = low_i;
      }
    }
    bin[k] = b_bin[k];
  }
  high_i = n->size[0] * n->size[1];
  n->size[0] = 1;
  n->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, n, high_i, &bd_emlrtRTEI);
  b_n_data = n->data;
  for (high_i = 0; high_i < loop_ub; high_i++) {
    b_n_data[high_i] = n_data[high_i];
  }
  emxFree_int32_T(sp, &b_n);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (histcounts.c) */
