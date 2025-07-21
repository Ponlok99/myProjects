/*
 * slerp.c
 *
 * Code generation for function 'slerp'
 *
 */

/* Include files */
#include "slerp.h"
#include "normalize.h"
#include "quaternion.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo lk_emlrtRSI = {
    60,          /* lineNo */
    "privslerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\privslerp.m" /* pathName */
};

static emlrtRSInfo sk_emlrtRSI = {
    16,          /* lineNo */
    "privslerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\privslerp.m" /* pathName */
};

/* Function Definitions */
quaternion quaternionBase_slerp(const emlrtStack *sp, const quaternion q1,
                                const quaternion q2, real_T t)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  quaternion qo;
  real_T dp;
  real_T q1n_a;
  real_T q1n_a_tmp;
  real_T q1n_b;
  real_T q1n_c;
  real_T q1n_d;
  real_T q2n_a;
  real_T q2n_b;
  real_T q2n_c;
  real_T q2n_d;
  real_T sinv;
  real_T theta0;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ik_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &sk_emlrtRSI;
  c_st.site = &vc_emlrtRSI;
  if (!(t >= 0.0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &gb_emlrtRTEI,
                                  "MATLAB:validateattributes:expectedArray",
                                  "MATLAB:slerp:notGreaterEqual", 9, 4, 18,
                                  "input number 3, t,", 4, 2, ">=", 4, 1, "0");
  }
  c_st.site = &vc_emlrtRSI;
  if (!(t <= 1.0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &hb_emlrtRTEI,
                                  "MATLAB:validateattributes:expectedArray",
                                  "MATLAB:slerp:notLessEqual", 9, 4, 18,
                                  "input number 3, t,", 4, 2, "<=", 4, 1, "1");
  }
  qo = q1;
  b_st.site = &jk_emlrtRSI;
  quaternionBase_normalize(&b_st, &qo);
  theta0 = qo.c * 0.0;
  dp = qo.d * 0.0;
  q1n_a_tmp = qo.b * 0.0;
  q1n_a = ((qo.a - q1n_a_tmp) - theta0) - dp;
  sinv = qo.a * 0.0;
  q1n_b = ((sinv + qo.b) + theta0) - dp;
  q1n_c = ((sinv - q1n_a_tmp) + qo.c) + dp;
  q1n_d = ((sinv + q1n_a_tmp) - theta0) + qo.d;
  qo = q2;
  b_st.site = &kk_emlrtRSI;
  quaternionBase_normalize(&b_st, &qo);
  q1n_a_tmp = qo.c * 0.0;
  theta0 = qo.d * 0.0;
  sinv = qo.b * 0.0;
  q2n_a = ((qo.a - sinv) - q1n_a_tmp) - theta0;
  dp = qo.a * 0.0;
  q2n_b = ((dp + qo.b) + q1n_a_tmp) - theta0;
  q2n_c = ((dp - sinv) + qo.c) + theta0;
  q2n_d = ((dp + sinv) - q1n_a_tmp) + qo.d;
  dp = ((q1n_a * q2n_a + q1n_b * q2n_b) + q1n_c * q2n_c) + q1n_d * q2n_d;
  if (dp < 0.0) {
    q2n_a = -q2n_a;
    q2n_b = -q2n_b;
    q2n_c = -q2n_c;
    q2n_d = -q2n_d;
    dp = -dp;
  }
  if (dp > 1.0) {
    dp = 1.0;
  }
  b_st.site = &lk_emlrtRSI;
  theta0 = muDoubleScalarAcos(dp);
  sinv = 1.0 / muDoubleScalarSin(theta0);
  dp = muDoubleScalarSin((1.0 - t) * theta0);
  q1n_a_tmp = muDoubleScalarSin(t * theta0);
  qo.a = sinv * (dp * q1n_a + q1n_a_tmp * q2n_a);
  qo.b = sinv * (dp * q1n_b + q1n_a_tmp * q2n_b);
  qo.c = sinv * (dp * q1n_c + q1n_a_tmp * q2n_c);
  qo.d = sinv * (dp * q1n_d + q1n_a_tmp * q2n_d);
  if (muDoubleScalarIsInf(sinv)) {
    int32_T t4_a_size[2];
    int32_T t4_b_size[2];
    int32_T t4_c_size[2];
    int32_T t4_d_size[2];
    b_st.site = &mk_emlrtRSI;
    quaternion_parenReference(
        q1.a, q1.b, q1.c, q1.d, (real_T *)&dp, t4_a_size, (real_T *)&theta0,
        t4_b_size, (real_T *)&q1n_a_tmp, t4_c_size, (real_T *)&sinv, t4_d_size);
    b_st.site = &mk_emlrtRSI;
    quaternion_parenAssign(&b_st, &qo, (real_T *)&dp, t4_a_size,
                           (real_T *)&theta0, t4_b_size, (real_T *)&q1n_a_tmp,
                           t4_c_size, (real_T *)&sinv, t4_d_size);
  }
  b_st.site = &nk_emlrtRSI;
  quaternionBase_normalize(&b_st, &qo);
  return qo;
}

/* End of code generation (slerp.c) */
