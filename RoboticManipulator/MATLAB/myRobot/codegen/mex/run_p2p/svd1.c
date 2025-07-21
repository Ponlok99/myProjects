/*
 * svd1.c
 *
 * Code generation for function 'svd1'
 *
 */

/* Include files */
#include "svd1.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo md_emlrtRSI =
    {
        50,    /* lineNo */
        "eye", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\eye.m" /* pathName
                                                                          */
};

static emlrtRSInfo gg_emlrtRSI = {
    31,       /* lineNo */
    "xgesvd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesvd.m" /* pathName */
};

static emlrtRSInfo hg_emlrtRSI = {
    205,            /* lineNo */
    "ceval_xgesvd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesvd.m" /* pathName */
};

static emlrtRSInfo cv_emlrtRSI = {
    23,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo dv_emlrtRSI = {
    52,    /* lineNo */
    "svd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo ev_emlrtRSI = {
    171,              /* lineNo */
    "getUSVForEmpty", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo fv_emlrtRSI = {
    89,           /* lineNo */
    "callLAPACK", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo gv_emlrtRSI = {
    81,           /* lineNo */
    "callLAPACK", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pathName
                                                                          */
};

static emlrtRSInfo hv_emlrtRSI = {
    209,      /* lineNo */
    "xgesdd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesdd.m" /* pathName */
};

static emlrtRTEInfo x_emlrtRTEI = {
    111,          /* lineNo */
    5,            /* colNo */
    "callLAPACK", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m" /* pName
                                                                          */
};

/* Function Definitions */
int32_T b_svd(const emlrtStack *sp, const real_T A_data[],
              const int32_T A_size[2], real_T U[36], real_T s_data[],
              real_T V_data[], int32_T V_size[2])
{
  static const char_T b_fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                     '_', 'd', 'g', 'e', 's', 'v', 'd'};
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'g', 'e', 's', 'd', 'd'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T Vt_data[2401];
  real_T b_A_data[294];
  real_T c_A_data[294];
  real_T superb_data[5];
  int32_T Vt_size_idx_1;
  int32_T i;
  int32_T i1;
  int32_T s_size;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  if (A_size[1] == 0) {
    st.site = &cv_emlrtRSI;
    memset(&U[0], 0, 36U * sizeof(real_T));
    for (Vt_size_idx_1 = 0; Vt_size_idx_1 < 6; Vt_size_idx_1++) {
      U[Vt_size_idx_1 + 6 * Vt_size_idx_1] = 1.0;
    }
    b_st.site = &ev_emlrtRSI;
    c_st.site = &md_emlrtRSI;
    V_size[0] = 0;
    V_size[1] = 0;
    s_size = 0;
  } else {
    ptrdiff_t info_t;
    int32_T info;
    st.site = &dv_emlrtRSI;
    Vt_size_idx_1 = 6 * A_size[1];
    memcpy(&b_A_data[0], &A_data[0], (uint32_T)Vt_size_idx_1 * sizeof(real_T));
    b_st.site = &gv_emlrtRSI;
    memcpy(&c_A_data[0], &A_data[0], (uint32_T)Vt_size_idx_1 * sizeof(real_T));
    Vt_size_idx_1 = A_size[1];
    s_size = muIntScalarMin_sint32(A_size[1], 6);
    info_t = LAPACKE_dgesdd(102, 'A', (ptrdiff_t)6, (ptrdiff_t)A_size[1],
                            &c_A_data[0], (ptrdiff_t)6, &s_data[0], &U[0],
                            (ptrdiff_t)6, &Vt_data[0], (ptrdiff_t)A_size[1]);
    c_st.site = &hv_emlrtRSI;
    if ((int32_T)info_t < 0) {
      if ((int32_T)info_t == -1010) {
        emlrtErrorWithMessageIdR2018a(&c_st, &u_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &v_emlrtRTEI,
                                      "Coder:toolbox:LAPACKCallErrorInfo",
                                      "Coder:toolbox:LAPACKCallErrorInfo", 5, 4,
                                      14, &fname[0], 12, (int32_T)info_t);
      }
    }
    info = (int32_T)info_t;
    if ((int32_T)info_t > 0) {
      b_st.site = &fv_emlrtRSI;
      c_st.site = &gg_emlrtRSI;
      info_t = LAPACKE_dgesvd(102, 'A', 'A', (ptrdiff_t)6, (ptrdiff_t)A_size[1],
                              &b_A_data[0], (ptrdiff_t)6, &s_data[0], &U[0],
                              (ptrdiff_t)6, &Vt_data[0], (ptrdiff_t)A_size[1],
                              &superb_data[0]);
      V_size[0] = A_size[1];
      V_size[1] = A_size[1];
      for (i = 0; i < Vt_size_idx_1; i++) {
        for (i1 = 0; i1 < Vt_size_idx_1; i1++) {
          V_data[i1 + Vt_size_idx_1 * i] = Vt_data[i + Vt_size_idx_1 * i1];
        }
      }
      d_st.site = &hg_emlrtRSI;
      if ((int32_T)info_t < 0) {
        if ((int32_T)info_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&d_st, &u_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(
              &d_st, &v_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
              "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, &b_fname[0], 12,
              (int32_T)info_t);
        }
      }
      info = (int32_T)info_t;
    } else {
      V_size[0] = A_size[1];
      V_size[1] = A_size[1];
      for (i = 0; i < Vt_size_idx_1; i++) {
        for (i1 = 0; i1 < Vt_size_idx_1; i1++) {
          V_data[i1 + Vt_size_idx_1 * i] = Vt_data[i + Vt_size_idx_1 * i1];
        }
      }
    }
    if (info > 0) {
      emlrtErrorWithMessageIdR2018a(&st, &x_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    }
  }
  return s_size;
}

/* End of code generation (svd1.c) */
