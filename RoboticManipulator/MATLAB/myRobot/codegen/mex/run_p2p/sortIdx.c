/*
 * sortIdx.c
 *
 * Code generation for function 'sortIdx'
 *
 */

/* Include files */
#include "sortIdx.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo
    or_emlrtRSI =
        {
            105,       /* lineNo */
            "sortIdx", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    pr_emlrtRSI =
        {
            326,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    qr_emlrtRSI =
        {
            301,                /* lineNo */
            "block_merge_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    rr_emlrtRSI =
        {
            381,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    sr_emlrtRSI =
        {
            409,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    tr_emlrtRSI =
        {
            416,                      /* lineNo */
            "initialize_vector_sort", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    ur_emlrtRSI =
        {
            488,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    vr_emlrtRSI =
        {
            495,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    wr_emlrtRSI =
        {
            496,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    xr_emlrtRSI =
        {
            503,           /* lineNo */
            "merge_block", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    yr_emlrtRSI =
        {
            550,     /* lineNo */
            "merge", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo
    as_emlrtRSI =
        {
            519,     /* lineNo */
            "merge", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

/* Function Declarations */
static void merge(const emlrtStack *sp, int32_T idx_data[], real_T x_data[],
                  int32_T offset, int32_T np, int32_T nq, int32_T iwork_data[],
                  real_T xwork_data[]);

/* Function Definitions */
static void merge(const emlrtStack *sp, int32_T idx_data[], real_T x_data[],
                  int32_T offset, int32_T np, int32_T nq, int32_T iwork_data[],
                  real_T xwork_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T j;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (nq != 0) {
    int32_T iout;
    int32_T n_tmp;
    int32_T p;
    int32_T q;
    n_tmp = np + nq;
    st.site = &as_emlrtRSI;
    if (n_tmp > 2147483646) {
      b_st.site = &od_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          st.site = &yr_emlrtRSI;
          if ((p + 1 <= np) && (np > 2147483646)) {
            b_st.site = &od_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

int32_T sortIdx(const emlrtStack *sp, real_T x_data[], const int32_T *x_size,
                int32_T idx_data[])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int32_T iwork_data[32];
  int32_T idx_size;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  idx_size = *x_size;
  if (idx_size - 1 >= 0) {
    memset(&idx_data[0], 0, (uint32_T)idx_size * sizeof(int32_T));
  }
  if (*x_size != 0) {
    real_T xwork_data[32];
    real_T x4[4];
    int32_T b_i1;
    int32_T i;
    int32_T i1;
    int32_T i2;
    int32_T i3;
    int32_T i4;
    int32_T ib;
    int32_T nNaNs;
    int8_T idx4[4];
    st.site = &or_emlrtRSI;
    if (idx_size - 1 >= 0) {
      memset(&iwork_data[0], 0, (uint32_T)idx_size * sizeof(int32_T));
    }
    b_st.site = &qr_emlrtRSI;
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    nNaNs = 0;
    ib = 0;
    c_st.site = &rr_emlrtRSI;
    for (k = 0; k < idx_size; k++) {
      if (muDoubleScalarIsNaN(x_data[k])) {
        i3 = (idx_size - nNaNs) - 1;
        idx_data[i3] = k + 1;
        xwork_data[i3] = x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = (int8_T)(k + 1);
        x4[ib - 1] = x_data[k];
        if (ib == 4) {
          real_T d;
          real_T d1;
          ib = k - nNaNs;
          if (x4[0] <= x4[1]) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (x4[2] <= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          d = x4[i1 - 1];
          d1 = x4[i3 - 1];
          if (d <= d1) {
            d = x4[i2 - 1];
            if (d <= d1) {
              i = i1;
              b_i1 = i2;
              i1 = i3;
              i2 = i4;
            } else if (d <= x4[i4 - 1]) {
              i = i1;
              b_i1 = i3;
              i1 = i2;
              i2 = i4;
            } else {
              i = i1;
              b_i1 = i3;
              i1 = i4;
            }
          } else {
            d1 = x4[i4 - 1];
            if (d <= d1) {
              if (x4[i2 - 1] <= d1) {
                i = i3;
                b_i1 = i1;
                i1 = i2;
                i2 = i4;
              } else {
                i = i3;
                b_i1 = i1;
                i1 = i4;
              }
            } else {
              i = i3;
              b_i1 = i4;
            }
          }
          idx_data[ib - 3] = idx4[i - 1];
          idx_data[ib - 2] = idx4[b_i1 - 1];
          idx_data[ib - 1] = idx4[i1 - 1];
          idx_data[ib] = idx4[i2 - 1];
          x_data[ib - 3] = x4[i - 1];
          x_data[ib - 2] = x4[b_i1 - 1];
          x_data[ib - 1] = x4[i1 - 1];
          x_data[ib] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    i4 = *x_size - nNaNs;
    if (ib > 0) {
      int8_T perm[4];
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }
      c_st.site = &sr_emlrtRSI;
      if (ib > 2147483646) {
        d_st.site = &od_emlrtRSI;
        check_forloop_overflow_error(&d_st);
      }
      i = (uint8_T)ib;
      for (k = 0; k < i; k++) {
        i3 = (i4 - ib) + k;
        b_i1 = perm[k];
        idx_data[i3] = idx4[b_i1 - 1];
        x_data[i3] = x4[b_i1 - 1];
      }
    }
    i1 = nNaNs >> 1;
    c_st.site = &tr_emlrtRSI;
    for (k = 0; k < i1; k++) {
      ib = i4 + k;
      i2 = idx_data[ib];
      i3 = (idx_size - k) - 1;
      idx_data[ib] = idx_data[i3];
      idx_data[i3] = i2;
      x_data[ib] = xwork_data[i3];
      x_data[i3] = xwork_data[ib];
    }
    if (((uint32_T)nNaNs & 1U) != 0U) {
      i = i4 + i1;
      x_data[i] = xwork_data[i];
    }
    if (i4 > 1) {
      b_st.site = &pr_emlrtRSI;
      i3 = i4 >> 2;
      i2 = 4;
      while (i3 > 1) {
        if (((uint32_T)i3 & 1U) != 0U) {
          i3--;
          ib = i2 * i3;
          i1 = i4 - ib;
          if (i1 > i2) {
            c_st.site = &ur_emlrtRSI;
            merge(&c_st, idx_data, x_data, ib, i2, i1 - i2, iwork_data,
                  xwork_data);
          }
        }
        ib = i2 << 1;
        i3 >>= 1;
        c_st.site = &vr_emlrtRSI;
        for (k = 0; k < i3; k++) {
          c_st.site = &wr_emlrtRSI;
          merge(&c_st, idx_data, x_data, k * ib, i2, i2, iwork_data,
                xwork_data);
        }
        i2 = ib;
      }
      if (i4 > i2) {
        c_st.site = &xr_emlrtRSI;
        merge(&c_st, idx_data, x_data, 0, i2, i4 - i2, iwork_data, xwork_data);
      }
    }
  }
  return idx_size;
}

/* End of code generation (sortIdx.c) */
