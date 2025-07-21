/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "eml_int_forloop_overflow_check.h"
#include "infocheck.h"
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_mexutil.h"
#include "run_p2p_types.h"
#include "warning.h"
#include "xgeqp3.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo oe_emlrtRSI = {
    27,       /* lineNo */
    "xgetrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrf.m" /* pathName */
};

static emlrtRSInfo pe_emlrtRSI = {
    91,             /* lineNo */
    "ceval_xgetrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrf.m" /* pathName */
};

static emlrtRSInfo te_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo ue_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo ve_emlrtRSI = {
    44,      /* lineNo */
    "mldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pathName
                                                                         */
};

static emlrtRSInfo
    we_emlrtRSI =
        {
            67,        /* lineNo */
            "lusolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    xe_emlrtRSI =
        {
            109,          /* lineNo */
            "lusolveNxN", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    ye_emlrtRSI =
        {
            112,          /* lineNo */
            "lusolveNxN", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    af_emlrtRSI =
        {
            124,          /* lineNo */
            "InvAtimesX", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo bf_emlrtRSI = {
    19,        /* lineNo */
    "xgetrfs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo cf_emlrtRSI = {
    108,      /* lineNo */
    "cmldiv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo ef_emlrtRSI = {
    18,       /* lineNo */
    "xgetrs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrs.m" /* pathName */
};

static emlrtRSInfo
    hf_emlrtRSI =
        {
            90,              /* lineNo */
            "warn_singular", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\lusolve.m" /* pathName */
};

static emlrtRSInfo
    if_emlrtRSI =
        {
            61,        /* lineNo */
            "qrsolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    jf_emlrtRSI =
        {
            72,        /* lineNo */
            "qrsolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    kf_emlrtRSI =
        {
            85,        /* lineNo */
            "qrsolve", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    wf_emlrtRSI =
        {
            119,         /* lineNo */
            "LSQFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    xf_emlrtRSI =
        {
            128,         /* lineNo */
            "LSQFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo
    yf_emlrtRSI =
        {
            138,         /* lineNo */
            "LSQFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

static emlrtRSInfo ag_emlrtRSI = {
    40,         /* lineNo */
    "xunormqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xunormqr.m" /* pathName */
};

static emlrtRSInfo bg_emlrtRSI = {
    106,              /* lineNo */
    "ceval_xunormqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xunormqr.m" /* pathName */
};

static emlrtRSInfo tt_emlrtRSI = {
    26,        /* lineNo */
    "xgetrfs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRSInfo ut_emlrtRSI = {
    27,        /* lineNo */
    "xgetrfs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrfs.m" /* pathName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    16,         /* lineNo */
    19,         /* colNo */
    "mldivide", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pName
                                                                         */
};

static emlrtRTEInfo
    oc_emlrtRTEI =
        {
            61,        /* lineNo */
            2,         /* colNo */
            "qrsolve", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pName */
};

static emlrtRTEInfo pc_emlrtRTEI = {
    1,          /* lineNo */
    14,         /* colNo */
    "mldivide", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" /* pName
                                                                         */
};

static const char_T cv1[19] = {'L', 'A', 'P', 'A', 'C', 'K', 'E', '_', 'd', 'g',
                               'e', 't', 'r', 'f', '_', 'w', 'o', 'r', 'k'};

/* Function Definitions */
int32_T b_mldivide(const emlrtStack *sp, const real_T A_data[],
                   const int32_T A_size[2], const real_T B[6], real_T Y_data[])
{
  static const int32_T b_iv[2] = {1, 6};
  static const char_T rfmt[6] = {'%', '1', '4', '.', '6', 'e'};
  ptrdiff_t jpvt_t_data[49];
  ptrdiff_t ipiv_t[6];
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  real_T b_A_data[294];
  real_T c_A_data[36];
  real_T b_B[6];
  real_T tau_data[6];
  int32_T jpvt_data[49];
  int32_T Y_size;
  int32_T b_i;
  int32_T i1;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &te_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  if (A_size[1] == 0) {
    Y_size = 0;
  } else if (A_size[1] == 6) {
    ptrdiff_t info_t;
    real_T tol;
    int32_T ipiv_data[6];
    int32_T minmana;
    b_st.site = &ue_emlrtRSI;
    memcpy(&c_A_data[0], &A_data[0], 36U * sizeof(real_T));
    c_st.site = &we_emlrtRSI;
    for (b_i = 0; b_i < 6; b_i++) {
      b_B[b_i] = B[b_i];
    }
    d_st.site = &xe_emlrtRSI;
    e_st.site = &af_emlrtRSI;
    f_st.site = &tt_emlrtRSI;
    g_st.site = &oe_emlrtRSI;
    info_t = LAPACKE_dgetrf_work(102, (ptrdiff_t)6, (ptrdiff_t)6, &c_A_data[0],
                                 (ptrdiff_t)6, &ipiv_t[0]);
    h_st.site = &pe_emlrtRSI;
    if ((int32_T)info_t < 0) {
      if ((int32_T)info_t == -1010) {
        emlrtErrorWithMessageIdR2018a(&h_st, &u_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&h_st, &v_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      19, &cv1[0], 12, (int32_T)info_t);
      }
    }
    for (k = 0; k < 6; k++) {
      ipiv_data[k] = (int32_T)ipiv_t[k];
    }
    f_st.site = &ut_emlrtRSI;
    g_st.site = &ef_emlrtRSI;
    for (b_i = 0; b_i < 5; b_i++) {
      i1 = ipiv_data[b_i];
      if (i1 != b_i + 1) {
        tol = b_B[b_i];
        b_B[b_i] = b_B[i1 - 1];
        b_B[i1 - 1] = tol;
      }
    }
    for (k = 0; k < 6; k++) {
      minmana = 6 * k;
      if (b_B[k] != 0.0) {
        i1 = k + 2;
        for (b_i = i1; b_i < 7; b_i++) {
          b_B[b_i - 1] -= b_B[k] * c_A_data[(b_i + minmana) - 1];
        }
      }
    }
    for (k = 5; k >= 0; k--) {
      minmana = 6 * k;
      tol = b_B[k];
      if (tol != 0.0) {
        tol /= c_A_data[k + minmana];
        b_B[k] = tol;
        for (b_i = 0; b_i < k; b_i++) {
          b_B[b_i] -= b_B[k] * c_A_data[b_i + minmana];
        }
      }
    }
    if ((int32_T)info_t > 0) {
      d_st.site = &ye_emlrtRSI;
      e_st.site = &hf_emlrtRSI;
      warning(&e_st);
    }
    Y_size = 6;
    for (i1 = 0; i1 < 6; i1++) {
      Y_data[i1] = b_B[i1];
    }
  } else {
    ptrdiff_t info_t;
    real_T tol;
    int32_T i;
    int32_T minmana;
    int32_T minmn;
    int32_T na;
    boolean_T p;
    b_st.site = &ve_emlrtRSI;
    c_st.site = &if_emlrtRSI;
    minmn = A_size[1];
    i = A_size[1];
    minmana = 6 * A_size[1];
    memcpy(&b_A_data[0], &A_data[0], (uint32_T)minmana * sizeof(real_T));
    na = A_size[1] - 1;
    memset(&jpvt_data[0], 0, (uint32_T)minmn * sizeof(int32_T));
    d_st.site = &lf_emlrtRSI;
    minmana = muIntScalarMin_sint32(6, A_size[1]);
    for (i1 = 0; i1 < minmn; i1++) {
      jpvt_t_data[i1] = (ptrdiff_t)0;
    }
    info_t =
        LAPACKE_dgeqp3(102, (ptrdiff_t)6, (ptrdiff_t)A_size[1], &b_A_data[0],
                       (ptrdiff_t)6, &jpvt_t_data[0], &tau_data[0]);
    e_st.site = &nf_emlrtRSI;
    if ((int32_T)info_t != 0) {
      p = true;
      if ((int32_T)info_t != -4) {
        if ((int32_T)info_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&e_st, &u_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&e_st, &v_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &cv2[0], 12, (int32_T)info_t);
        }
      }
    } else {
      p = false;
    }
    if (p) {
      e_st.site = &of_emlrtRSI;
      for (k = 0; k <= na; k++) {
        for (b_i = 0; b_i < 6; b_i++) {
          b_A_data[k * 6 + b_i] = rtNaN;
        }
      }
      minmn = muIntScalarMin_sint32(6, A_size[1]) - 1;
      e_st.site = &qf_emlrtRSI;
      for (k = 0; k <= minmn; k++) {
        tau_data[k] = rtNaN;
      }
      i1 = minmn + 2;
      e_st.site = &rf_emlrtRSI;
      if (i1 <= minmana) {
        memset(&tau_data[i1 + -1], 0,
               (uint32_T)((minmana - i1) + 1) * sizeof(real_T));
      }
      e_st.site = &sf_emlrtRSI;
      minmana = (A_size[1] / 4) << 2;
      minmn = minmana - 4;
      for (k = 0; k <= minmn; k += 4) {
        _mm_storeu_si128(
            (__m128i *)&jpvt_data[k],
            _mm_add_epi32(
                _mm_add_epi32(_mm_set1_epi32(k),
                              _mm_loadu_si128((const __m128i *)&iv2[0])),
                _mm_set1_epi32(1)));
      }
      for (k = minmana; k <= na; k++) {
        jpvt_data[k] = k + 1;
      }
    } else {
      e_st.site = &tf_emlrtRSI;
      for (k = 0; k <= na; k++) {
        jpvt_data[k] = (int32_T)jpvt_t_data[k];
      }
    }
    c_st.site = &jf_emlrtRSI;
    na = 0;
    if (A_size[1] > 6) {
      minmn = 6;
      minmana = A_size[1];
    } else {
      minmn = A_size[1];
      minmana = 6;
    }
    tol = 2.2204460492503131E-15 * (real_T)minmana *
          muDoubleScalarAbs(b_A_data[0]);
    while ((na < minmn) &&
           (!(muDoubleScalarAbs(b_A_data[na + 6 * na]) <= tol))) {
      na++;
    }
    if (na < minmn) {
      char_T str[14];
      d_st.site = &uf_emlrtRSI;
      y = NULL;
      m = emlrtCreateCharArray(2, &b_iv[0]);
      emlrtInitCharArrayR2013a(&d_st, 6, m, &rfmt[0]);
      emlrtAssign(&y, m);
      b_y = NULL;
      m = emlrtCreateDoubleScalar(tol);
      emlrtAssign(&b_y, m);
      e_st.site = &rv_emlrtRSI;
      emlrt_marshallIn(&e_st, b_sprintf(&e_st, y, b_y, &b_emlrtMCI),
                       "<output of sprintf>", str);
      d_st.site = &vf_emlrtRSI;
      c_warning(&d_st, na, str);
    }
    c_st.site = &kf_emlrtRSI;
    for (b_i = 0; b_i < 6; b_i++) {
      b_B[b_i] = B[b_i];
    }
    Y_size = A_size[1];
    memset(&Y_data[0], 0, (uint32_T)i * sizeof(real_T));
    d_st.site = &wf_emlrtRSI;
    e_st.site = &ag_emlrtRSI;
    info_t = (ptrdiff_t)6;
    if (A_size[1] > 6) {
      i = 6;
    }
    info_t = LAPACKE_dormqr(102, 'L', 'T', info_t, (ptrdiff_t)1, (ptrdiff_t)i,
                            &b_A_data[0], (ptrdiff_t)6, &tau_data[0], &b_B[0],
                            info_t);
    f_st.site = &bg_emlrtRSI;
    if (infocheck(&f_st, (int32_T)info_t)) {
      for (b_i = 0; b_i < 6; b_i++) {
        b_B[b_i] = rtNaN;
      }
    }
    d_st.site = &xf_emlrtRSI;
    for (b_i = 0; b_i < na; b_i++) {
      Y_data[jpvt_data[b_i] - 1] = b_B[b_i];
    }
    for (k = na; k >= 1; k--) {
      i1 = jpvt_data[k - 1];
      minmana = 6 * (k - 1);
      Y_data[i1 - 1] /= b_A_data[(k + minmana) - 1];
      d_st.site = &yf_emlrtRSI;
      for (b_i = 0; b_i <= k - 2; b_i++) {
        minmn = jpvt_data[b_i];
        Y_data[minmn - 1] -= Y_data[i1 - 1] * b_A_data[b_i + minmana];
      }
    }
  }
  return Y_size;
}

int32_T mldivide(const emlrtStack *sp, const real_T A_data[],
                 const int32_T A_size[2], const real_T B_data[], int32_T B_size,
                 real_T Y_data[])
{
  ptrdiff_t IPIV_data[10];
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_int32_T *jpvt;
  emxArray_real_T *A;
  emxArray_real_T *tau;
  real_T c_A_data[100];
  real_T b_B_data[10];
  real_T *b_A_data;
  real_T *tau_data;
  int32_T Y_size;
  int32_T b_i;
  int32_T i1;
  int32_T j;
  int32_T *jpvt_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  if (B_size != A_size[0]) {
    emlrtErrorWithMessageIdR2018a(sp, &w_emlrtRTEI, "MATLAB:dimagree",
                                  "MATLAB:dimagree", 0);
  }
  st.site = &te_emlrtRSI;
  emxInit_real_T(&st, &A, 2, &pc_emlrtRTEI);
  emxInit_real_T(&st, &tau, 1, &pc_emlrtRTEI);
  emxInit_int32_T(&st, &jpvt, 2, &pc_emlrtRTEI);
  if ((A_size[0] == 0) || (A_size[1] == 0) || (B_size == 0)) {
    int32_T i;
    i = A_size[1];
    Y_size = A_size[1];
    if (i - 1 >= 0) {
      memset(&Y_data[0], 0, (uint32_T)i * sizeof(real_T));
    }
  } else if (A_size[0] == A_size[1]) {
    ptrdiff_t INFO;
    ptrdiff_t LDA;
    ptrdiff_t nrc_t;
    int32_T i;
    b_st.site = &ue_emlrtRSI;
    c_st.site = &we_emlrtRSI;
    Y_size = B_size;
    memcpy(&Y_data[0], &B_data[0], (uint32_T)B_size * sizeof(real_T));
    d_st.site = &xe_emlrtRSI;
    e_st.site = &af_emlrtRSI;
    f_st.site = &bf_emlrtRSI;
    i = A_size[0] * A_size[1];
    memcpy(&c_A_data[0], &A_data[0], (uint32_T)i * sizeof(real_T));
    i = muIntScalarMin_sint32(A_size[0], A_size[1]);
    nrc_t = (ptrdiff_t)muIntScalarMin_sint32(B_size, i);
    LDA = (ptrdiff_t)A_size[0];
    INFO = LAPACKE_dgetrf_work(102, nrc_t, nrc_t, &c_A_data[0], LDA,
                               &IPIV_data[0]);
    g_st.site = &cf_emlrtRSI;
    if ((int32_T)INFO < 0) {
      if ((int32_T)INFO == -1010) {
        emlrtErrorWithMessageIdR2018a(&g_st, &u_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&g_st, &v_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      19, &cv1[0], 12, (int32_T)INFO);
      }
    }
    LAPACKE_dgetrs_work(102, 'N', nrc_t, (ptrdiff_t)1, &c_A_data[0], LDA,
                        &IPIV_data[0], &Y_data[0], (ptrdiff_t)B_size);
    if (((A_size[0] != 1) || (A_size[1] != 1)) && ((int32_T)INFO > 0)) {
      d_st.site = &ye_emlrtRSI;
      e_st.site = &hf_emlrtRSI;
      warning(&e_st);
    }
  } else {
    int32_T i;
    int32_T rankA;
    b_st.site = &ve_emlrtRSI;
    i1 = A->size[0] * A->size[1];
    A->size[0] = A_size[0];
    A->size[1] = A_size[1];
    emxEnsureCapacity_real_T(&b_st, A, i1, &oc_emlrtRTEI);
    b_A_data = A->data;
    i = A_size[0] * A_size[1];
    for (i1 = 0; i1 < i; i1++) {
      b_A_data[i1] = A_data[i1];
    }
    c_st.site = &if_emlrtRSI;
    xgeqp3(&c_st, A, tau, jpvt);
    jpvt_data = jpvt->data;
    tau_data = tau->data;
    b_A_data = A->data;
    c_st.site = &jf_emlrtRSI;
    rankA = rankFromQR(&c_st, A);
    c_st.site = &kf_emlrtRSI;
    memcpy(&b_B_data[0], &B_data[0], (uint32_T)B_size * sizeof(real_T));
    i = A->size[1];
    Y_size = A->size[1];
    if (i - 1 >= 0) {
      memset(&Y_data[0], 0, (uint32_T)i * sizeof(real_T));
    }
    d_st.site = &wf_emlrtRSI;
    e_st.site = &ag_emlrtRSI;
    if ((A->size[0] != 0) && (A->size[1] != 0)) {
      ptrdiff_t nrc_t;
      nrc_t = (ptrdiff_t)B_size;
      nrc_t = LAPACKE_dormqr(
          102, 'L', 'T', nrc_t, (ptrdiff_t)1,
          (ptrdiff_t)muIntScalarMin_sint32(A->size[0], A->size[1]),
          &b_A_data[0], (ptrdiff_t)A->size[0], &tau_data[0], &b_B_data[0],
          nrc_t);
      f_st.site = &bg_emlrtRSI;
      if (infocheck(&f_st, (int32_T)nrc_t)) {
        for (i1 = 0; i1 < B_size; i1++) {
          b_B_data[i1] = rtNaN;
        }
      }
    }
    d_st.site = &xf_emlrtRSI;
    if (rankA > 2147483646) {
      e_st.site = &od_emlrtRSI;
      check_forloop_overflow_error(&e_st);
    }
    i1 = (uint8_T)rankA;
    for (b_i = 0; b_i < i1; b_i++) {
      Y_data[jpvt_data[b_i] - 1] = b_B_data[b_i];
    }
    for (j = rankA; j >= 1; j--) {
      i1 = jpvt_data[j - 1];
      Y_data[i1 - 1] /= b_A_data[(j + A->size[0] * (j - 1)) - 1];
      d_st.site = &yf_emlrtRSI;
      i = (uint8_T)(j - 1);
      for (b_i = 0; b_i < i; b_i++) {
        Y_data[jpvt_data[b_i] - 1] -=
            Y_data[i1 - 1] * b_A_data[b_i + A->size[0] * (j - 1)];
      }
    }
  }
  emxFree_int32_T(&st, &jpvt);
  emxFree_real_T(&st, &tau);
  emxFree_real_T(&st, &A);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return Y_size;
}

/* End of code generation (mldivide.c) */
