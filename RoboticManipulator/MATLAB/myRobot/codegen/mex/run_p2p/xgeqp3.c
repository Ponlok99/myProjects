/*
 * xgeqp3.c
 *
 * Code generation for function 'xgeqp3'
 *
 */

/* Include files */
#include "xgeqp3.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo mf_emlrtRSI = {
    98,             /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo pf_emlrtRSI = {
    143,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRTEInfo vc_emlrtRTEI = {
    61,       /* lineNo */
    9,        /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo wc_emlrtRTEI = {
    92,       /* lineNo */
    22,       /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo xc_emlrtRTEI = {
    105,      /* lineNo */
    1,        /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

static emlrtRTEInfo yc_emlrtRTEI = {
    97,       /* lineNo */
    5,        /* colNo */
    "xgeqp3", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pName */
};

/* Function Definitions */
void xgeqp3(const emlrtStack *sp, emxArray_real_T *A, emxArray_real_T *tau,
            emxArray_int32_T *jpvt)
{
  ptrdiff_t *jpvt_t_data;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_ptrdiff_t *jpvt_t;
  real_T *A_data;
  real_T *tau_data;
  int32_T i;
  int32_T loop_ub;
  int32_T minmana;
  int32_T minmn;
  int32_T na;
  int32_T *jpvt_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  A_data = A->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  minmn = A->size[0];
  na = A->size[1] - 1;
  i = jpvt->size[0] * jpvt->size[1];
  jpvt->size[0] = 1;
  loop_ub = A->size[1];
  jpvt->size[1] = loop_ub;
  emxEnsureCapacity_int32_T(sp, jpvt, i, &vc_emlrtRTEI);
  jpvt_data = jpvt->data;
  for (i = 0; i < loop_ub; i++) {
    jpvt_data[i] = 0;
  }
  st.site = &lf_emlrtRSI;
  minmana = muIntScalarMin_sint32(minmn, loop_ub);
  i = tau->size[0];
  tau->size[0] = minmana;
  emxEnsureCapacity_real_T(&st, tau, i, &wc_emlrtRTEI);
  tau_data = tau->data;
  emxInit_ptrdiff_t(&st, &jpvt_t, &xc_emlrtRTEI);
  if ((A->size[0] == 0) || (A->size[1] == 0) || (A->size[0] < 1) ||
      (A->size[1] < 1)) {
    i = tau->size[0];
    tau->size[0] = minmana;
    emxEnsureCapacity_real_T(&st, tau, i, &yc_emlrtRTEI);
    tau_data = tau->data;
    for (i = 0; i < minmana; i++) {
      tau_data[i] = 0.0;
    }
    b_st.site = &mf_emlrtRSI;
    if (A->size[1] > 2147483646) {
      c_st.site = &od_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    loop_ub = ((na + 1) / 4) << 2;
    minmn = loop_ub - 4;
    for (i = 0; i <= minmn; i += 4) {
      _mm_storeu_si128(
          (__m128i *)&jpvt_data[i],
          _mm_add_epi32(
              _mm_add_epi32(_mm_set1_epi32(i),
                            _mm_loadu_si128((const __m128i *)&iv2[0])),
              _mm_set1_epi32(1)));
    }
    for (i = loop_ub; i <= na; i++) {
      jpvt_data[i] = i + 1;
    }
  } else {
    ptrdiff_t info_t;
    boolean_T p;
    i = jpvt_t->size[0];
    jpvt_t->size[0] = loop_ub;
    emxEnsureCapacity_ptrdiff_t(&st, jpvt_t, i, &xc_emlrtRTEI);
    jpvt_t_data = jpvt_t->data;
    for (i = 0; i < loop_ub; i++) {
      jpvt_t_data[i] = (ptrdiff_t)0;
    }
    info_t = LAPACKE_dgeqp3(102, (ptrdiff_t)A->size[0], (ptrdiff_t)A->size[1],
                            &A_data[0], (ptrdiff_t)A->size[0], &jpvt_t_data[0],
                            &tau_data[0]);
    b_st.site = &nf_emlrtRSI;
    if ((int32_T)info_t != 0) {
      p = true;
      if ((int32_T)info_t != -4) {
        if ((int32_T)info_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&b_st, &u_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &v_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &cv2[0], 12, (int32_T)info_t);
        }
      }
    } else {
      p = false;
    }
    if (p) {
      b_st.site = &of_emlrtRSI;
      if (A->size[1] > 2147483646) {
        c_st.site = &od_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (loop_ub = 0; loop_ub <= na; loop_ub++) {
        b_st.site = &pf_emlrtRSI;
        if (minmn > 2147483646) {
          c_st.site = &od_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (i = 0; i < minmn; i++) {
          A_data[loop_ub * minmn + i] = rtNaN;
        }
      }
      i = na + 1;
      minmn = muIntScalarMin_sint32(minmn, i);
      b_st.site = &qf_emlrtRSI;
      if (minmn > 2147483646) {
        c_st.site = &od_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (i = 0; i < minmn; i++) {
        tau_data[i] = rtNaN;
      }
      loop_ub = minmn + 1;
      b_st.site = &rf_emlrtRSI;
      if ((minmn + 1 <= minmana) && (minmana > 2147483646)) {
        c_st.site = &od_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (i = loop_ub; i <= minmana; i++) {
        tau_data[i - 1] = 0.0;
      }
      b_st.site = &sf_emlrtRSI;
      if (A->size[1] > 2147483646) {
        c_st.site = &od_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      loop_ub = ((na + 1) / 4) << 2;
      minmn = loop_ub - 4;
      for (i = 0; i <= minmn; i += 4) {
        _mm_storeu_si128(
            (__m128i *)&jpvt_data[i],
            _mm_add_epi32(
                _mm_add_epi32(_mm_set1_epi32(i),
                              _mm_loadu_si128((const __m128i *)&iv2[0])),
                _mm_set1_epi32(1)));
      }
      for (i = loop_ub; i <= na; i++) {
        jpvt_data[i] = i + 1;
      }
    } else {
      b_st.site = &tf_emlrtRSI;
      if (A->size[1] > 2147483646) {
        c_st.site = &od_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (i = 0; i <= na; i++) {
        jpvt_data[i] = (int32_T)jpvt_t_data[i];
      }
    }
  }
  emxFree_ptrdiff_t(&st, &jpvt_t);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (xgeqp3.c) */
