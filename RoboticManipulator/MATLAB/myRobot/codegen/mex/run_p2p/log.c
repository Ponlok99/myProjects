/*
 * log.c
 *
 * Code generation for function 'log'
 *
 */

/* Include files */
#include "log.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo uk_emlrtRSI = {
    18,                   /* lineNo */
    "quaternionBase/log", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\log.m" /* pathName */
};

static emlrtRSInfo vk_emlrtRSI = {
    19,                   /* lineNo */
    "quaternionBase/log", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\log.m" /* pathName */
};

static emlrtRSInfo wk_emlrtRSI = {
    24,                   /* lineNo */
    "quaternionBase/log", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\log.m" /* pathName */
};

static emlrtRSInfo xk_emlrtRSI = {
    26,                   /* lineNo */
    "quaternionBase/log", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\log.m" /* pathName */
};

static emlrtECInfo nb_emlrtECI = {
    -1,                   /* nDims */
    29,                   /* lineNo */
    1,                    /* colNo */
    "quaternionBase/log", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\log.m" /* pName */
};

static emlrtECInfo ob_emlrtECI = {
    -1,                   /* nDims */
    31,                   /* lineNo */
    1,                    /* colNo */
    "quaternionBase/log", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\log.m" /* pName */
};

static emlrtRTEInfo fc_emlrtRTEI = {
    14,     /* lineNo */
    9,      /* colNo */
    "acos", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elfun\\acos.m" /* pName
                                                                       */
};

/* Function Definitions */
void quaternionBase_log(const emlrtStack *sp, quaternion *q)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T qnorm;
  real_T unnamed_idx_0;
  real_T vnorm;
  real_T vscale_data;
  int32_T vscale_size[2];
  int32_T i;
  int32_T k;
  int32_T trueCount;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &uk_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &uk_emlrtRSI;
  st.site = &uk_emlrtRSI;
  st.site = &uk_emlrtRSI;
  vnorm = (q->b * q->b + q->c * q->c) + q->d * q->d;
  if (vnorm < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  vnorm = muDoubleScalarSqrt(vnorm);
  st.site = &vk_emlrtRSI;
  st.site = &vk_emlrtRSI;
  st.site = &vk_emlrtRSI;
  qnorm = q->a * q->a + vnorm * vnorm;
  if (qnorm < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &jb_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  qnorm = muDoubleScalarSqrt(qnorm);
  trueCount = 0;
  if (vnorm != 0.0) {
    trueCount = 1;
  }
  st.site = &wk_emlrtRSI;
  if (trueCount - 1 >= 0) {
    vscale_data = q->a / qnorm;
  }
  st.site = &wk_emlrtRSI;
  b_st.site = &wk_emlrtRSI;
  p = false;
  for (k = 0; k < trueCount; k++) {
    if (p || ((vscale_data < -1.0) || (vscale_data > 1.0))) {
      p = true;
    }
  }
  if (p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &fc_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "acos");
  }
  for (k = 0; k < trueCount; k++) {
    vscale_data = muDoubleScalarAcos(vscale_data);
  }
  k = trueCount - 1;
  for (i = 0; i <= k; i++) {
    vscale_data /= vnorm;
  }
  st.site = &xk_emlrtRSI;
  q->a = muDoubleScalarLog(qnorm);
  if (trueCount - 1 >= 0) {
    qnorm = q->b * vscale_data;
  }
  unnamed_idx_0 = q->b;
  if (trueCount - 1 >= 0) {
    unnamed_idx_0 = qnorm;
  }
  p = !(vnorm != 0.0);
  if (p) {
    unnamed_idx_0 = 0.0;
  }
  q->b = unnamed_idx_0;
  vscale_size[1] = trueCount;
  if (trueCount - 1 >= 0) {
    qnorm = q->c * vscale_data;
  }
  if (trueCount != vscale_size[1]) {
    emlrtSubAssignSizeCheck1dR2017a(trueCount, vscale_size[1], &nb_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  unnamed_idx_0 = q->c;
  if (vscale_size[1] - 1 >= 0) {
    unnamed_idx_0 = qnorm;
  }
  if (p) {
    unnamed_idx_0 = 0.0;
  }
  q->c = unnamed_idx_0;
  k = trueCount - 1;
  for (i = 0; i <= k; i++) {
    vscale_data *= q->d;
  }
  if (trueCount != vscale_size[1]) {
    emlrtSubAssignSizeCheck1dR2017a(trueCount, vscale_size[1], &ob_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  unnamed_idx_0 = q->d;
  if (vscale_size[1] - 1 >= 0) {
    unnamed_idx_0 = vscale_data;
  }
  if (p) {
    unnamed_idx_0 = 0.0;
  }
  q->d = unnamed_idx_0;
}

/* End of code generation (log.c) */
