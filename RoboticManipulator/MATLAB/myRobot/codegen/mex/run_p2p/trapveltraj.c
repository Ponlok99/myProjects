/*
 * trapveltraj.c
 *
 * Code generation for function 'trapveltraj'
 *
 */

/* Include files */
#include "trapveltraj.h"
#include "assertValidSizeArg.h"
#include "linspace.h"
#include "mpower.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_46
#define typedef_cell_wrap_46
typedef struct {
  real_T f1[4];
} cell_wrap_46;
#endif /* typedef_cell_wrap_46 */

#ifndef struct_emxArray_real_T_9x3
#define struct_emxArray_real_T_9x3
struct emxArray_real_T_9x3 {
  real_T data[27];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_9x3 */
#ifndef typedef_emxArray_real_T_9x3
#define typedef_emxArray_real_T_9x3
typedef struct emxArray_real_T_9x3 emxArray_real_T_9x3;
#endif /* typedef_emxArray_real_T_9x3 */

#ifndef typedef_cell_wrap_47
#define typedef_cell_wrap_47
typedef struct {
  emxArray_real_T_9x3 f1;
} cell_wrap_47;
#endif /* typedef_cell_wrap_47 */

#ifndef struct_emxArray_real_T_3x5x3
#define struct_emxArray_real_T_3x5x3
struct emxArray_real_T_3x5x3 {
  real_T data[45];
  int32_T size[3];
};
#endif /* struct_emxArray_real_T_3x5x3 */
#ifndef typedef_emxArray_real_T_3x5x3
#define typedef_emxArray_real_T_3x5x3
typedef struct emxArray_real_T_3x5x3 emxArray_real_T_3x5x3;
#endif /* typedef_emxArray_real_T_3x5x3 */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T breaks[6];
  emxArray_real_T_3x5x3 coefs;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_emxArray_struct_T_3
#define typedef_emxArray_struct_T_3
typedef struct {
  struct_T data[3];
} emxArray_struct_T_3;
#endif /* typedef_emxArray_struct_T_3 */

/* Variable Definitions */
static emlrtRSInfo ri_emlrtRSI = {
    82,            /* lineNo */
    "trapveltraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo si_emlrtRSI = {
    93,            /* lineNo */
    "trapveltraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo ti_emlrtRSI = {
    133,           /* lineNo */
    "trapveltraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo ui_emlrtRSI = {
    136,           /* lineNo */
    "trapveltraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo vi_emlrtRSI = {
    143,           /* lineNo */
    "trapveltraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo wi_emlrtRSI = {
    177,           /* lineNo */
    "trapveltraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo aj_emlrtRSI = {
    192,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo bj_emlrtRSI = {
    194,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo cj_emlrtRSI = {
    195,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo dj_emlrtRSI = {
    197,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo ej_emlrtRSI = {
    198,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo fj_emlrtRSI = {
    199,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo gj_emlrtRSI = {
    203,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo hj_emlrtRSI = {
    204,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo ij_emlrtRSI = {
    205,                             /* lineNo */
    "generateTrajectoriesFromCoefs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pathName */
};

static emlrtRSInfo jj_emlrtRSI = {
    18,                             /* lineNo */
    "addFlatSegmentsToPPFormParts", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pathName */
};

static emlrtRSInfo kj_emlrtRSI = {
    37,                  /* lineNo */
    "addSegmentToStart", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pathName */
};

static emlrtRSInfo mj_emlrtRSI = {
    68,                /* lineNo */
    "addSegmentToEnd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pathName */
};

static emlrtRTEInfo db_emlrtRTEI = {
    479,                    /* lineNo */
    9,                      /* colNo */
    "computeProfileParams", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pName */
};

static emlrtRTEInfo eb_emlrtRTEI = {
    478,                    /* lineNo */
    9,                      /* colNo */
    "computeProfileParams", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pName */
};

static emlrtECInfo p_emlrtECI = {
    -1,            /* nDims */
    177,           /* lineNo */
    53,            /* colNo */
    "trapveltraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pName */
};

static emlrtECInfo q_emlrtECI = {
    -1,            /* nDims */
    177,           /* lineNo */
    31,            /* colNo */
    "trapveltraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pName */
};

static emlrtECInfo r_emlrtECI = {
    -1,            /* nDims */
    177,           /* lineNo */
    10,            /* colNo */
    "trapveltraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pName */
};

static emlrtECInfo s_emlrtECI = {
    -1,            /* nDims */
    146,           /* lineNo */
    13,            /* colNo */
    "trapveltraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m" /* pName */
};

static emlrtBCInfo fb_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    168,           /* lineNo */
    43,            /* colNo */
    "",            /* aName */
    "trapveltraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m", /* pName */
    0           /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    177,           /* lineNo */
    76,            /* colNo */
    "",            /* aName */
    "trapveltraj", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\trapv"
    "eltraj.m", /* pName */
    0           /* checkKind */
};

static emlrtRTEInfo fb_emlrtRTEI = {
    48,     /* lineNo */
    19,     /* colNo */
    "mkpp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\mkpp.m" /* pName
                                                                         */
};

static emlrtECInfo t_emlrtECI = {
    -1,                     /* nDims */
    19,                     /* lineNo */
    5,                      /* colNo */
    "polyCoeffsDerivative", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\polyCoeffsDerivative.m" /* pName */
};

static emlrtECInfo u_emlrtECI = {
    -1,                  /* nDims */
    43,                  /* lineNo */
    1,                   /* colNo */
    "addSegmentToStart", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pName */
};

static emlrtBCInfo hb_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    43,                  /* lineNo */
    20,                  /* colNo */
    "",                  /* aName */
    "addSegmentToStart", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m", /* pName */
    0                                           /* checkKind */
};

static emlrtECInfo v_emlrtECI = {
    -1,                  /* nDims */
    42,                  /* lineNo */
    1,                   /* colNo */
    "addSegmentToStart", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pName */
};

static emlrtECInfo w_emlrtECI = {
    -1,                /* nDims */
    74,                /* lineNo */
    1,                 /* colNo */
    "addSegmentToEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pName */
};

static emlrtBCInfo ib_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    74,                /* lineNo */
    19,                /* colNo */
    "",                /* aName */
    "addSegmentToEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m", /* pName */
    0                                           /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    74,                /* lineNo */
    12,                /* colNo */
    "",                /* aName */
    "addSegmentToEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m", /* pName */
    0                                           /* checkKind */
};

static emlrtECInfo x_emlrtECI = {
    -1,                /* nDims */
    73,                /* lineNo */
    1,                 /* colNo */
    "addSegmentToEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pName */
};

static emlrtBCInfo kb_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    73,                /* lineNo */
    12,                /* colNo */
    "",                /* aName */
    "addSegmentToEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m", /* pName */
    0                                           /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    64,                /* lineNo */
    21,                /* colNo */
    "",                /* aName */
    "addSegmentToEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m", /* pName */
    0                                           /* checkKind */
};

static emlrtECInfo y_emlrtECI = {
    -1,                               /* nDims */
    85,                               /* lineNo */
    1,                                /* colNo */
    "createConstantPolynomialCoeffs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pName */
};

static emlrtRTEInfo lc_emlrtRTEI =
    {
        76,                  /* lineNo */
        9,                   /* colNo */
        "eml_mtimes_helper", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo sc_emlrtRTEI = {
    19,                     /* lineNo */
    20,                     /* colNo */
    "polyCoeffsDerivative", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\polyCoeffsDerivative.m" /* pName */
};

/* Function Declarations */
static void generateTrajectoriesFromCoefs(
    const emlrtStack *sp, const real_T breaks[4], const real_T coeffs_data[],
    const int32_T coeffs_size[2], real_T dim, const real_T t[101],
    real_T q_data[], int32_T q_size[2], real_T qd_data[], int32_T qd_size[2],
    real_T qdd_data[], int32_T qdd_size[2], real_T pp_breaks[6],
    real_T pp_coefs_data[], int32_T pp_coefs_size[3]);

/* Function Definitions */
static void generateTrajectoriesFromCoefs(
    const emlrtStack *sp, const real_T breaks[4], const real_T coeffs_data[],
    const int32_T coeffs_size[2], real_T dim, const real_T t[101],
    real_T q_data[], int32_T q_size[2], real_T qd_data[], int32_T qd_size[2],
    real_T qdd_data[], int32_T qdd_size[2], real_T pp_breaks[6],
    real_T pp_coefs_data[], int32_T pp_coefs_size[3])
{
  static const real_T B[3] = {0.0, 0.0, 1.0};
  __m128d r;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *y;
  real_T dCoeffs_data[45];
  real_T ddCoeffs_data[45];
  real_T modCoeffs_data[45];
  real_T ppd_coefs_data[45];
  real_T a_data[36];
  real_T coefsWithFlatStart_data[36];
  real_T valueAtEnd_data[12];
  real_T newSegmentCoeffs_data[9];
  real_T breaksWithFlatStart[5];
  real_T valueAtStart_data[3];
  real_T alpha1;
  real_T beta1;
  real_T *y_data;
  int32_T ppd_coefs_size[3];
  int32_T ppdd_coefs_size[3];
  int32_T b_dim[2];
  int32_T coefsWithFlatStart_size[2];
  int32_T ddCoeffs_size[2];
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T b_loop_ub_tmp;
  int32_T c_loop_ub;
  int32_T c_loop_ub_tmp;
  int32_T dim_tmp;
  int32_T i;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  char_T TRANSA1;
  char_T TRANSB1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &aj_emlrtRSI;
  b_st.site = &jj_emlrtRSI;
  loop_ub_tmp = (int32_T)dim;
  for (i = 0; i < 3; i++) {
    for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
      newSegmentCoeffs_data[b_i + (int32_T)dim * i] =
          coeffs_data[b_i + coeffs_size[0] * i];
    }
  }
  TRANSB1 = 'T';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)(int32_T)dim;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)3;
  lda_t = (ptrdiff_t)(int32_T)dim;
  ldb_t = (ptrdiff_t)1;
  ldc_t = (ptrdiff_t)(int32_T)dim;
  loop_ub = (int32_T)dim * 3;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1,
        &newSegmentCoeffs_data[0], &lda_t, (real_T *)&B[0], &ldb_t, &beta1,
        &valueAtStart_data[0], &ldc_t);
  c_st.site = &kj_emlrtRSI;
  ddCoeffs_size[0] = (int32_T)dim;
  ddCoeffs_size[1] = 3;
  if (loop_ub - 1 >= 0) {
    memset(&newSegmentCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }
  emlrtSubAssignSizeCheckR2012b(&loop_ub_tmp, 1, &loop_ub_tmp, 1, &y_emlrtECI,
                                &c_st);
  for (i = 0; i < loop_ub_tmp; i++) {
    newSegmentCoeffs_data[i + (int32_T)dim * 2] = valueAtStart_data[i];
  }
  b_loop_ub = (int32_T)((real_T)coeffs_size[0] + dim);
  coefsWithFlatStart_size[0] = b_loop_ub;
  coefsWithFlatStart_size[1] = 3;
  b_loop_ub_tmp = b_loop_ub * 3;
  if (b_loop_ub_tmp - 1 >= 0) {
    memset(&coefsWithFlatStart_data[0], 0,
           (uint32_T)b_loop_ub_tmp * sizeof(real_T));
  }
  b_dim[0] = (int32_T)dim;
  b_dim[1] = 3;
  emlrtSubAssignSizeCheckR2012b(&b_dim[0], 2, &ddCoeffs_size[0], 2, &v_emlrtECI,
                                &b_st);
  for (i = 0; i < 3; i++) {
    for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
      coefsWithFlatStart_data[b_i + b_loop_ub * i] =
          newSegmentCoeffs_data[b_i + (int32_T)dim * i];
    }
  }
  if (b_loop_ub < 1) {
    emlrtDynamicBoundsCheckR2012b(b_loop_ub, 1, b_loop_ub, &hb_emlrtBCI, &b_st);
  }
  dim_tmp = (b_loop_ub - (int32_T)(dim + 1.0)) + 1;
  b_dim[0] = dim_tmp;
  b_dim[1] = 3;
  emlrtSubAssignSizeCheckR2012b(&b_dim[0], 2, &coeffs_size[0], 2, &u_emlrtECI,
                                &b_st);
  for (i = 0; i < 3; i++) {
    for (b_i = 0; b_i < dim_tmp; b_i++) {
      coefsWithFlatStart_data[(((int32_T)(dim + 1.0) + b_i) + b_loop_ub * i) -
                              1] = coeffs_data[b_i + coeffs_size[0] * i];
    }
  }
  breaksWithFlatStart[0] = breaks[0] - 1.0;
  breaksWithFlatStart[1] = breaks[0];
  breaksWithFlatStart[2] = breaks[1];
  breaksWithFlatStart[3] = breaks[2];
  breaksWithFlatStart[4] = breaks[3];
  b_st.site = &gi_emlrtRSI;
  alpha1 = breaks[3] - breaks[2];
  c_st.site = &hi_emlrtRSI;
  valueAtStart_data[0] = mpower(&c_st, alpha1, 2.0);
  c_st.site = &hi_emlrtRSI;
  valueAtStart_data[1] = mpower(&c_st, alpha1, 1.0);
  c_st.site = &hi_emlrtRSI;
  valueAtStart_data[2] = mpower(&c_st, alpha1, 0.0);
  alpha1 = ((real_T)b_loop_ub - dim) + 1.0;
  if (alpha1 > b_loop_ub) {
    i = 0;
    b_i = 0;
  } else {
    if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > b_loop_ub)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, b_loop_ub, &lb_emlrtBCI,
                                    &b_st);
    }
    i = (int32_T)alpha1 - 1;
    b_i = b_loop_ub;
  }
  c_loop_ub = b_i - i;
  for (b_i = 0; b_i < 3; b_i++) {
    for (c_loop_ub_tmp = 0; c_loop_ub_tmp < c_loop_ub; c_loop_ub_tmp++) {
      a_data[c_loop_ub_tmp + c_loop_ub * b_i] =
          coefsWithFlatStart_data[(i + c_loop_ub_tmp) + b_loop_ub * b_i];
    }
  }
  if (c_loop_ub == 0) {
    c_loop_ub = 0;
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)c_loop_ub;
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)3;
    lda_t = (ptrdiff_t)c_loop_ub;
    ldb_t = (ptrdiff_t)3;
    ldc_t = (ptrdiff_t)c_loop_ub;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &a_data[0], &lda_t,
          &valueAtStart_data[0], &ldb_t, &beta1, &valueAtEnd_data[0], &ldc_t);
  }
  c_st.site = &mj_emlrtRSI;
  ddCoeffs_size[0] = (int32_T)dim;
  ddCoeffs_size[1] = 3;
  if (loop_ub - 1 >= 0) {
    memset(&newSegmentCoeffs_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }
  emlrtSubAssignSizeCheckR2012b(&loop_ub_tmp, 1, &c_loop_ub, 1, &y_emlrtECI,
                                &c_st);
  for (i = 0; i < loop_ub_tmp; i++) {
    newSegmentCoeffs_data[i + (int32_T)dim * 2] = valueAtEnd_data[i];
  }
  alpha1 = (real_T)b_loop_ub + dim;
  loop_ub_tmp = (int32_T)alpha1;
  b_loop_ub_tmp = (int32_T)alpha1 * 3;
  if (b_loop_ub_tmp - 1 >= 0) {
    memset(&modCoeffs_data[0], 0, (uint32_T)b_loop_ub_tmp * sizeof(real_T));
  }
  if ((b_loop_ub < 1) || (b_loop_ub > (int32_T)alpha1)) {
    emlrtDynamicBoundsCheckR2012b(b_loop_ub, 1, (int32_T)alpha1, &kb_emlrtBCI,
                                  &b_st);
  }
  b_dim[0] = b_loop_ub;
  b_dim[1] = 3;
  emlrtSubAssignSizeCheckR2012b(&b_dim[0], 2, &coefsWithFlatStart_size[0], 2,
                                &x_emlrtECI, &b_st);
  for (i = 0; i < 3; i++) {
    for (b_i = 0; b_i < b_loop_ub; b_i++) {
      modCoeffs_data[b_i + (int32_T)alpha1 * i] =
          coefsWithFlatStart_data[b_i + b_loop_ub * i];
    }
  }
  if ((real_T)b_loop_ub + 1.0 > alpha1) {
    b_loop_ub = 0;
    i = 0;
  } else {
    if (b_loop_ub + 1 > (int32_T)alpha1) {
      emlrtDynamicBoundsCheckR2012b(b_loop_ub + 1, 1, (int32_T)alpha1,
                                    &jb_emlrtBCI, &b_st);
    }
    if ((int32_T)alpha1 < 1) {
      emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, (int32_T)alpha1,
                                    &ib_emlrtBCI, &b_st);
    }
    i = (int32_T)alpha1;
  }
  dim_tmp = i - b_loop_ub;
  b_dim[0] = dim_tmp;
  b_dim[1] = 3;
  emlrtSubAssignSizeCheckR2012b(&b_dim[0], 2, &ddCoeffs_size[0], 2, &w_emlrtECI,
                                &b_st);
  for (i = 0; i < 3; i++) {
    for (b_i = 0; b_i < dim_tmp; b_i++) {
      modCoeffs_data[(b_loop_ub + b_i) + (int32_T)alpha1 * i] =
          newSegmentCoeffs_data[b_i + (int32_T)dim * i];
    }
  }
  for (i = 0; i < 5; i++) {
    pp_breaks[i] = breaksWithFlatStart[i];
  }
  pp_breaks[5] = breaks[3] + 1.0;
  st.site = &bj_emlrtRSI;
  ddCoeffs_size[0] = (int8_T)(int32_T)alpha1;
  ddCoeffs_size[1] = 3;
  c_loop_ub_tmp = (int8_T)(int32_T)alpha1 * 3;
  if (c_loop_ub_tmp - 1 >= 0) {
    memset(&dCoeffs_data[0], 0, (uint32_T)c_loop_ub_tmp * sizeof(real_T));
  }
  emxInit_real_T(&st, &y, 1, &sc_emlrtRTEI);
  c_loop_ub = ((int32_T)alpha1 / 2) << 1;
  b_loop_ub = c_loop_ub - 2;
  dim_tmp = (int8_T)(int32_T)alpha1;
  for (b_i = 0; b_i < 2; b_i++) {
    i = y->size[0];
    y->size[0] = (int32_T)alpha1;
    emxEnsureCapacity_real_T(&st, y, i, &lc_emlrtRTEI);
    y_data = y->data;
    for (i = 0; i <= b_loop_ub; i += 2) {
      r = _mm_loadu_pd(&modCoeffs_data[i + (int32_T)alpha1 * b_i]);
      _mm_storeu_pd(
          &y_data[i],
          _mm_mul_pd(_mm_set1_pd((3.0 - ((real_T)b_i + 2.0)) + 1.0), r));
    }
    for (i = c_loop_ub; i < loop_ub_tmp; i++) {
      y_data[i] = ((3.0 - ((real_T)b_i + 2.0)) + 1.0) *
                  modCoeffs_data[i + (int32_T)alpha1 * b_i];
    }
    emlrtSubAssignSizeCheckR2012b(&ddCoeffs_size[0], 1, &y->size[0], 1,
                                  &t_emlrtECI, &st);
    for (i = 0; i < dim_tmp; i++) {
      dCoeffs_data[i + (int8_T)(int32_T)alpha1 * (b_i + 1)] = y_data[i];
    }
  }
  st.site = &cj_emlrtRSI;
  ddCoeffs_size[0] = (int8_T)(int32_T)alpha1;
  ddCoeffs_size[1] = 3;
  loop_ub_tmp = (int8_T)(int32_T)alpha1 * 3;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&ddCoeffs_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }
  loop_ub = (int8_T)(int32_T)alpha1;
  c_loop_ub = ((int8_T)(int32_T)alpha1 / 2) << 1;
  b_loop_ub = c_loop_ub - 2;
  dim_tmp = (int8_T)(int32_T)alpha1;
  for (b_i = 0; b_i < 2; b_i++) {
    i = y->size[0];
    y->size[0] = (int8_T)(int32_T)alpha1;
    emxEnsureCapacity_real_T(&st, y, i, &lc_emlrtRTEI);
    y_data = y->data;
    for (i = 0; i <= b_loop_ub; i += 2) {
      r = _mm_loadu_pd(&dCoeffs_data[i + (int8_T)(int32_T)alpha1 * b_i]);
      _mm_storeu_pd(
          &y_data[i],
          _mm_mul_pd(_mm_set1_pd((3.0 - ((real_T)b_i + 2.0)) + 1.0), r));
    }
    for (i = c_loop_ub; i < loop_ub; i++) {
      y_data[i] = ((3.0 - ((real_T)b_i + 2.0)) + 1.0) *
                  dCoeffs_data[i + (int8_T)(int32_T)alpha1 * b_i];
    }
    emlrtSubAssignSizeCheckR2012b(&ddCoeffs_size[0], 1, &y->size[0], 1,
                                  &t_emlrtECI, &st);
    for (i = 0; i < dim_tmp; i++) {
      ddCoeffs_data[i + (int8_T)(int32_T)alpha1 * (b_i + 1)] = y_data[i];
    }
  }
  emxFree_real_T(&st, &y);
  st.site = &dj_emlrtRSI;
  if (!(dim == muDoubleScalarFloor(dim))) {
    emlrtErrorWithMessageIdR2018a(&st, &fb_emlrtRTEI, "Coder:toolbox:InvalidD",
                                  "Coder:toolbox:InvalidD", 0);
  }
  valueAtStart_data[0] = dim;
  valueAtStart_data[1] = 5.0;
  valueAtStart_data[2] = 3.0;
  beta1 = dim * 5.0 * 3.0;
  if (!(beta1 == (real_T)(int32_T)alpha1 * 3.0)) {
    emlrtErrorWithMessageIdR2018a(&st, &j_emlrtRTEI,
                                  "Coder:toolbox:MKPPSizeMismatch",
                                  "Coder:toolbox:MKPPSizeMismatch", 0);
  }
  b_st.site = &tg_emlrtRSI;
  c_st.site = &ug_emlrtRSI;
  assertValidSizeArg(&c_st, valueAtStart_data);
  loop_ub = (int32_T)dim * 5 * 3;
  if (loop_ub != b_loop_ub_tmp) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &l_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  pp_coefs_size[0] = (int32_T)dim;
  pp_coefs_size[1] = 5;
  pp_coefs_size[2] = 3;
  if (loop_ub - 1 >= 0) {
    memcpy(&pp_coefs_data[0], &modCoeffs_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  st.site = &ej_emlrtRSI;
  valueAtStart_data[1] = 5.0;
  valueAtStart_data[2] = 3.0;
  if (!(beta1 == (real_T)(int8_T)(int32_T)alpha1 * 3.0)) {
    emlrtErrorWithMessageIdR2018a(&st, &j_emlrtRTEI,
                                  "Coder:toolbox:MKPPSizeMismatch",
                                  "Coder:toolbox:MKPPSizeMismatch", 0);
  }
  b_st.site = &tg_emlrtRSI;
  c_st.site = &ug_emlrtRSI;
  assertValidSizeArg(&c_st, valueAtStart_data);
  if (loop_ub != c_loop_ub_tmp) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &l_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  ppd_coefs_size[0] = (int32_T)dim;
  ppd_coefs_size[1] = 5;
  ppd_coefs_size[2] = 3;
  if (loop_ub - 1 >= 0) {
    memcpy(&ppd_coefs_data[0], &dCoeffs_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  st.site = &fj_emlrtRSI;
  valueAtStart_data[1] = 5.0;
  valueAtStart_data[2] = 3.0;
  if ((int32_T)beta1 != (int8_T)(int32_T)alpha1 * 3) {
    emlrtErrorWithMessageIdR2018a(&st, &j_emlrtRTEI,
                                  "Coder:toolbox:MKPPSizeMismatch",
                                  "Coder:toolbox:MKPPSizeMismatch", 0);
  }
  b_st.site = &tg_emlrtRSI;
  c_st.site = &ug_emlrtRSI;
  assertValidSizeArg(&c_st, valueAtStart_data);
  if (loop_ub != loop_ub_tmp) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &l_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  ppdd_coefs_size[0] = (int32_T)dim;
  ppdd_coefs_size[1] = 5;
  ppdd_coefs_size[2] = 3;
  if (loop_ub - 1 >= 0) {
    memcpy(&modCoeffs_data[0], &ddCoeffs_data[0],
           (uint32_T)loop_ub * sizeof(real_T));
  }
  st.site = &gj_emlrtRSI;
  ppval(pp_breaks, pp_coefs_data, pp_coefs_size, t, q_data, q_size);
  st.site = &hj_emlrtRSI;
  ppval(pp_breaks, ppd_coefs_data, ppd_coefs_size, t, qd_data, qd_size);
  st.site = &ij_emlrtRSI;
  ppval(pp_breaks, modCoeffs_data, ppdd_coefs_size, t, qdd_data, qdd_size);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void b_trapveltraj(const emlrtStack *sp, const real_T wayPoints[6],
                   real_T q[303], real_T qd[303], real_T qdd[303])
{
  cell_wrap_46 breaksCell[3];
  cell_wrap_47 coeffsCell[3];
  emlrtStack b_st;
  emlrtStack st;
  emxArray_struct_T_3 ppCell;
  real_T c_tmp_data[303];
  real_T d_tmp_data[303];
  real_T e_tmp_data[303];
  real_T t[101];
  real_T coeffMat[27];
  real_T parameterMat[18];
  real_T breakMat[12];
  real_T coefs[9];
  real_T varargin_1[3];
  real_T d;
  real_T s0;
  int32_T b_iv[2];
  int32_T b_tmp_size[2];
  int32_T c_tmp_size[2];
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T c_i;
  int32_T deltaSign;
  int32_T i;
  int32_T i1;
  int32_T jj;
  int32_T numComputedPolynomials;
  int8_T b_tmp_data[9];
  int8_T unnamed_idx_0;
  boolean_T exitg1;
  boolean_T hasMultipleBreaks;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ri_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &vc_emlrtRSI;
  hasMultipleBreaks = true;
  numComputedPolynomials = 0;
  exitg1 = false;
  while ((!exitg1) && (numComputedPolynomials < 6)) {
    if ((!muDoubleScalarIsInf(wayPoints[numComputedPolynomials])) &&
        (!muDoubleScalarIsNaN(wayPoints[numComputedPolynomials]))) {
      numComputedPolynomials++;
    } else {
      hasMultipleBreaks = false;
      exitg1 = true;
    }
  }
  if (!hasMultipleBreaks) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:trapveltraj:expectedFinite", 3, 4, 9, "wayPoints");
  }
  st.site = &si_emlrtRSI;
  memset(&q[0], 0, 303U * sizeof(real_T));
  memset(&qd[0], 0, 303U * sizeof(real_T));
  memset(&qdd[0], 0, 303U * sizeof(real_T));
  memset(&coeffMat[0], 0, 27U * sizeof(real_T));
  memset(&breakMat[0], 0, 12U * sizeof(real_T));
  tmp_size[0] = 3;
  tmp_size[1] = 3;
  for (i = 0; i < 3; i++) {
    real_T sF;
    real_T sF_tmp;
    real_T segATime;
    real_T segAcc;
    real_T segVel;
    real_T segVel_tmp;
    int8_T lspbSegIndices_data[4];
    int8_T tmp_data[4];
    boolean_T coefIndex[9];
    st.site = &ti_emlrtRSI;
    d = wayPoints[i];
    s0 = d;
    sF_tmp = wayPoints[i + 3];
    sF = sF_tmp;
    deltaSign = 1;
    if (sF_tmp < d) {
      s0 = sF_tmp;
      sF = d;
      deltaSign = -1;
    }
    segVel_tmp = sF - s0;
    segVel = 1.5 * segVel_tmp;
    segATime = ((s0 - sF) + segVel) / segVel;
    segAcc = segVel / segATime;
    if (s0 == sF) {
      segAcc = 0.0;
      segVel = 0.0;
      segATime = 0.33333333333333331;
    } else {
      if (!(segVel_tmp < segVel)) {
        emlrtErrorWithMessageIdR2018a(
            &st, &eb_emlrtRTEI,
            "shared_robotics:robotcore:utils:TrapVelLowerBoundCondition",
            "shared_robotics:robotcore:utils:TrapVelLowerBoundCondition", 6, 6,
            (real_T)i + 1.0, 6, 1.0, 6, 2.0);
      }
      if (!(segVel <= 2.0 * segVel_tmp)) {
        emlrtErrorWithMessageIdR2018a(
            &st, &db_emlrtRTEI,
            "shared_robotics:robotcore:utils:TrapVelUpperBoundCondition",
            "shared_robotics:robotcore:utils:TrapVelUpperBoundCondition", 6, 6,
            (real_T)i + 1.0, 6, 1.0, 6, 2.0);
      }
    }
    segVel *= (real_T)deltaSign;
    segAcc *= (real_T)deltaSign;
    parameterMat[i] = d;
    parameterMat[i + 3] = sF_tmp;
    parameterMat[i + 6] = segVel;
    parameterMat[i + 9] = segAcc;
    parameterMat[i + 12] = segATime;
    parameterMat[i + 15] = 1.0;
    st.site = &ui_emlrtRSI;
    memset(&coefs[0], 0, 9U * sizeof(real_T));
    if (segVel == 0.0) {
      coefs[6] = d;
      coefs[7] = d;
      coefs[8] = d;
    } else {
      coefs[0] = segAcc / 2.0;
      coefs[3] = 0.0;
      coefs[6] = d;
      coefs[1] = 0.0;
      coefs[4] = segVel;
      s0 = segAcc / 2.0 * (segATime * segATime);
      coefs[7] = s0 + d;
      coefs[2] = -segAcc / 2.0;
      coefs[5] = segVel;
      coefs[8] = (sF_tmp + s0) - segVel * segATime;
    }
    for (c_i = 0; c_i < 9; c_i++) {
      coefIndex[c_i] = false;
    }
    st.site = &vi_emlrtRSI;
    for (b_i = 0; b_i < 3; b_i++) {
      lspbSegIndices_data[b_i] = (int8_T)((i + 3 * b_i) + 1);
    }
    for (b_i = 0; b_i < 3; b_i++) {
      tmp_data[b_i] = lspbSegIndices_data[b_i];
    }
    for (b_i = 0; b_i < 3; b_i++) {
      coefIndex[tmp_data[b_i] - 1] = true;
    }
    numComputedPolynomials = 0;
    deltaSign = 0;
    for (c_i = 0; c_i < 9; c_i++) {
      if (coefIndex[c_i]) {
        numComputedPolynomials++;
        b_tmp_data[deltaSign] = (int8_T)c_i;
        deltaSign++;
      }
    }
    b_iv[0] = numComputedPolynomials;
    b_iv[1] = 3;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &tmp_size[0], 2, &s_emlrtECI,
                                  (emlrtCTX)sp);
    for (b_i = 0; b_i < 3; b_i++) {
      for (i1 = 0; i1 < numComputedPolynomials; i1++) {
        coeffMat[b_tmp_data[i1] + 9 * b_i] =
            coefs[i1 + numComputedPolynomials * b_i];
      }
    }
    s0 = breakMat[i];
    breakMat[i + 3] = segATime + s0;
    breakMat[i + 6] = (1.0 - segATime) + s0;
    breakMat[i + 9] = s0 + 1.0;
  }
  hasMultipleBreaks = false;
  for (i = 0; i < 2; i++) {
    real_T y[4];
    boolean_T b_y;
    y[0] = muDoubleScalarAbs(breakMat[i] - breakMat[i + 1]);
    y[1] = muDoubleScalarAbs(breakMat[i + 3] - breakMat[i + 4]);
    y[2] = muDoubleScalarAbs(breakMat[i + 6] - breakMat[i + 7]);
    y[3] = muDoubleScalarAbs(breakMat[i + 9] - breakMat[i + 10]);
    b_y = false;
    numComputedPolynomials = 0;
    exitg1 = false;
    while ((!exitg1) && (numComputedPolynomials < 4)) {
      if (y[numComputedPolynomials] > 2.2204460492503131E-16) {
        b_y = true;
        exitg1 = true;
      } else {
        numComputedPolynomials++;
      }
    }
    if (b_y || hasMultipleBreaks) {
      hasMultipleBreaks = true;
    } else {
      hasMultipleBreaks = false;
    }
  }
  if (hasMultipleBreaks) {
    coeffsCell[0].f1.size[0] = 3;
    coeffsCell[0].f1.size[1] = 3;
    breaksCell[0].f1[0] = breakMat[0];
    breaksCell[0].f1[1] = breakMat[3];
    breaksCell[0].f1[2] = breakMat[6];
    breaksCell[0].f1[3] = breakMat[9];
    coeffsCell[1].f1.size[0] = 3;
    coeffsCell[1].f1.size[1] = 3;
    breaksCell[1].f1[0] = breakMat[1];
    breaksCell[1].f1[1] = breakMat[4];
    breaksCell[1].f1[2] = breakMat[7];
    breaksCell[1].f1[3] = breakMat[10];
    coeffsCell[2].f1.size[0] = 3;
    coeffsCell[2].f1.size[1] = 3;
    for (b_i = 0; b_i < 3; b_i++) {
      coeffsCell[0].f1.data[3 * b_i] = coeffMat[9 * b_i];
      i1 = 3 * b_i + 1;
      coeffsCell[0].f1.data[i1] = coeffMat[9 * b_i + 3];
      deltaSign = 3 * b_i + 2;
      coeffsCell[0].f1.data[deltaSign] = coeffMat[9 * b_i + 6];
      coeffsCell[1].f1.data[3 * b_i] = coeffMat[9 * b_i + 1];
      coeffsCell[1].f1.data[i1] = coeffMat[9 * b_i + 4];
      coeffsCell[1].f1.data[deltaSign] = coeffMat[9 * b_i + 7];
      coeffsCell[2].f1.data[3 * b_i] = coeffMat[9 * b_i + 2];
      coeffsCell[2].f1.data[i1] = coeffMat[9 * b_i + 5];
      coeffsCell[2].f1.data[deltaSign] = coeffMat[9 * b_i + 8];
    }
    breaksCell[2].f1[0] = breakMat[2];
    breaksCell[2].f1[1] = breakMat[5];
    breaksCell[2].f1[2] = breakMat[8];
    breaksCell[2].f1[3] = breakMat[11];
  } else {
    coeffsCell[0].f1.size[0] = 9;
    coeffsCell[0].f1.size[1] = 3;
    breaksCell[0].f1[0] = breakMat[0];
    breaksCell[0].f1[1] = breakMat[3];
    breaksCell[0].f1[2] = breakMat[6];
    breaksCell[0].f1[3] = breakMat[9];
    coeffsCell[1].f1.size[0] = 9;
    coeffsCell[1].f1.size[1] = 3;
    breaksCell[1].f1[0] = breakMat[0];
    breaksCell[1].f1[1] = breakMat[3];
    breaksCell[1].f1[2] = breakMat[6];
    breaksCell[1].f1[3] = breakMat[9];
    coeffsCell[2].f1.size[0] = 9;
    coeffsCell[2].f1.size[1] = 3;
    memcpy(&coeffsCell[0].f1.data[0], &coeffMat[0], 27U * sizeof(real_T));
    memcpy(&coeffsCell[1].f1.data[0], &coeffMat[0], 27U * sizeof(real_T));
    memcpy(&coeffsCell[2].f1.data[0], &coeffMat[0], 27U * sizeof(real_T));
    breaksCell[2].f1[0] = breakMat[0];
    breaksCell[2].f1[1] = breakMat[3];
    breaksCell[2].f1[2] = breakMat[6];
    breaksCell[2].f1[3] = breakMat[9];
  }
  varargin_1[0] = parameterMat[15];
  varargin_1[1] = parameterMat[16];
  varargin_1[2] = parameterMat[17];
  if (!muDoubleScalarIsNaN(parameterMat[15])) {
    deltaSign = 1;
  } else {
    deltaSign = 0;
    numComputedPolynomials = 2;
    exitg1 = false;
    while ((!exitg1) && (numComputedPolynomials < 4)) {
      if (!muDoubleScalarIsNaN(varargin_1[numComputedPolynomials - 1])) {
        deltaSign = numComputedPolynomials;
        exitg1 = true;
      } else {
        numComputedPolynomials++;
      }
    }
  }
  if (deltaSign == 0) {
    s0 = parameterMat[15];
  } else {
    s0 = varargin_1[deltaSign - 1];
    b_i = deltaSign + 1;
    for (numComputedPolynomials = b_i; numComputedPolynomials < 4;
         numComputedPolynomials++) {
      d = varargin_1[numComputedPolynomials - 1];
      if (s0 < d) {
        s0 = d;
      }
    }
  }
  linspace(0.0, s0, t);
  if (hasMultipleBreaks) {
    numComputedPolynomials = 3;
    c_i = 1;
  } else {
    numComputedPolynomials = 1;
    c_i = 3;
  }
  unnamed_idx_0 = (int8_T)numComputedPolynomials;
  for (b_i = 0; b_i < unnamed_idx_0; b_i++) {
    if (b_i > numComputedPolynomials - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, numComputedPolynomials - 1,
                                    &fb_emlrtBCI, (emlrtConstCTX)sp);
    }
  }
  for (jj = 0; jj < numComputedPolynomials; jj++) {
    int8_T f_tmp_data[3];
    int8_T rowSelection_data[3];
    if (hasMultipleBreaks) {
      i = 1;
      rowSelection_data[0] = (int8_T)(jj + 1);
      deltaSign = jj;
    } else {
      i = 3;
      rowSelection_data[0] = 1;
      rowSelection_data[1] = 2;
      rowSelection_data[2] = 3;
      deltaSign = 0;
    }
    if (jj > numComputedPolynomials - 1) {
      emlrtDynamicBoundsCheckR2012b(jj, 0, numComputedPolynomials - 1,
                                    &gb_emlrtBCI, (emlrtConstCTX)sp);
    }
    st.site = &wi_emlrtRSI;
    generateTrajectoriesFromCoefs(
        &st, breaksCell[deltaSign].f1, coeffsCell[deltaSign].f1.data,
        coeffsCell[deltaSign].f1.size, c_i, t, c_tmp_data, tmp_size, d_tmp_data,
        b_tmp_size, e_tmp_data, c_tmp_size, ppCell.data[jj].breaks,
        ppCell.data[jj].coefs.data, ppCell.data[jj].coefs.size);
    for (b_i = 0; b_i < i; b_i++) {
      f_tmp_data[b_i] = (int8_T)(rowSelection_data[b_i] - 1);
    }
    b_iv[0] = i;
    b_iv[1] = 101;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &tmp_size[0], 2, &r_emlrtECI,
                                  (emlrtCTX)sp);
    for (b_i = 0; b_i < 101; b_i++) {
      for (i1 = 0; i1 < i; i1++) {
        q[f_tmp_data[i1] + 3 * b_i] = c_tmp_data[i1 + tmp_size[0] * b_i];
      }
    }
    b_iv[0] = i;
    b_iv[1] = 101;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &b_tmp_size[0], 2, &q_emlrtECI,
                                  (emlrtCTX)sp);
    for (b_i = 0; b_i < 101; b_i++) {
      for (i1 = 0; i1 < i; i1++) {
        qd[f_tmp_data[i1] + 3 * b_i] = d_tmp_data[i1 + b_tmp_size[0] * b_i];
      }
    }
    b_iv[0] = i;
    b_iv[1] = 101;
    emlrtSubAssignSizeCheckR2012b(&b_iv[0], 2, &c_tmp_size[0], 2, &p_emlrtECI,
                                  (emlrtCTX)sp);
    for (b_i = 0; b_i < 101; b_i++) {
      for (i1 = 0; i1 < i; i1++) {
        qdd[f_tmp_data[i1] + 3 * b_i] = e_tmp_data[i1 + c_tmp_size[0] * b_i];
      }
    }
  }
}

/* End of code generation (trapveltraj.c) */
