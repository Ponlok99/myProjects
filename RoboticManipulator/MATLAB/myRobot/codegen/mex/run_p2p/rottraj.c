/*
 * rottraj.c
 *
 * Code generation for function 'rottraj'
 *
 */

/* Include files */
#include "rottraj.h"
#include "log.h"
#include "normalize.h"
#include "quaternion.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "slerp.h"
#include "validateTimeScaling.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo rj_emlrtRSI = {
    105,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo sj_emlrtRSI = {
    106,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo tj_emlrtRSI = {
    107,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo uj_emlrtRSI = {
    108,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo vj_emlrtRSI = {
    111,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo wj_emlrtRSI = {
    125,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo xj_emlrtRSI = {
    126,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo yj_emlrtRSI = {
    129,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo ak_emlrtRSI = {
    130,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo bk_emlrtRSI = {
    132,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo ck_emlrtRSI = {
    136,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo dk_emlrtRSI = {
    143,       /* lineNo */
    "rottraj", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo tk_emlrtRSI = {
    214,                          /* lineNo */
    "computeFirstQuatDerivative", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo al_emlrtRSI = {
    229,                           /* lineNo */
    "computeSecondQuatDerivative", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo bl_emlrtRSI = {
    233,                           /* lineNo */
    "computeSecondQuatDerivative", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

static emlrtRSInfo cl_emlrtRSI = {
    235,                           /* lineNo */
    "computeSecondQuatDerivative", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\rottr"
    "aj.m" /* pathName */
};

/* Function Definitions */
void rottraj(const emlrtStack *sp, const real_T R0[4], const real_T RF[4],
             const real_T varargin_2[303], real_T R[404], real_T omega[303],
             real_T alpha[303])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  quaternion pnCorrected;
  quaternion q0;
  quaternion q1n;
  quaternion qF;
  quaternion x;
  real_T timeScaling[303];
  real_T qCalc_a[101];
  real_T qCalc_b[101];
  real_T qCalc_c[101];
  real_T qCalc_d[101];
  real_T dp;
  real_T q1n_a;
  real_T q1n_b;
  real_T q1n_c;
  real_T q1n_d;
  real_T q2n_a;
  real_T q2n_b;
  real_T q2n_c;
  real_T q2n_d;
  real_T sinv;
  real_T theta0;
  real_T xd;
  real_T y;
  int32_T t5_a_size[2];
  int32_T t5_b_size[2];
  int32_T t5_c_size[2];
  int32_T t5_d_size[2];
  int32_T b_i;
  int32_T i;
  int32_T omega_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  q0.a = R0[0];
  q0.b = R0[1];
  q0.c = R0[2];
  q0.d = R0[3];
  qF.a = RF[0];
  qF.b = RF[1];
  qF.c = RF[2];
  qF.d = RF[3];
  st.site = &rj_emlrtRSI;
  st.site = &sj_emlrtRSI;
  b_st.site = &gd_emlrtRSI;
  st.site = &tj_emlrtRSI;
  st.site = &uj_emlrtRSI;
  st.site = &vj_emlrtRSI;
  validateTimeScaling(&st, varargin_2, timeScaling);
  st.site = &wj_emlrtRSI;
  quaternionBase_normalize(&st, &q0);
  st.site = &xj_emlrtRSI;
  quaternionBase_normalize(&st, &qF);
  st.site = &yj_emlrtRSI;
  b_st.site = &ik_emlrtRSI;
  x = q0;
  c_st.site = &jk_emlrtRSI;
  quaternionBase_normalize(&c_st, &x);
  dp = x.c * 0.0;
  theta0 = x.d * 0.0;
  xd = x.b * 0.0;
  q1n.a = ((x.a - xd) - dp) - theta0;
  sinv = x.a * 0.0;
  q1n.b = ((sinv + x.b) + dp) - theta0;
  q1n.c = ((sinv - xd) + x.c) + theta0;
  q1n.d = ((sinv + xd) - dp) + x.d;
  x = qF;
  c_st.site = &kk_emlrtRSI;
  quaternionBase_normalize(&c_st, &x);
  xd = x.c * 0.0;
  theta0 = x.d * 0.0;
  sinv = x.b * 0.0;
  q2n_a = ((x.a - sinv) - xd) - theta0;
  dp = x.a * 0.0;
  q2n_b = ((dp + x.b) + xd) - theta0;
  q2n_c = ((dp - sinv) + x.c) + theta0;
  q2n_d = ((dp + sinv) - xd) + x.d;
  dp = ((q1n.a * q2n_a + q1n.b * q2n_b) + q1n.c * q2n_c) + q1n.d * q2n_d;
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
  theta0 = muDoubleScalarAcos(dp);
  xd = muDoubleScalarSin(theta0);
  sinv = 1.0 / xd;
  y = muDoubleScalarSin(0.0 * theta0);
  pnCorrected.a = sinv * (xd * q1n.a + y * q2n_a);
  pnCorrected.b = sinv * (xd * q1n.b + y * q2n_b);
  pnCorrected.c = sinv * (xd * q1n.c + y * q2n_c);
  pnCorrected.d = sinv * (xd * q1n.d + y * q2n_d);
  if (muDoubleScalarIsInf(sinv)) {
    quaternion_parenReference(q0.a, q0.b, q0.c, q0.d, (real_T *)&dp, t5_a_size,
                              (real_T *)&theta0, t5_b_size, (real_T *)&xd,
                              t5_c_size, (real_T *)&sinv, t5_d_size);
    c_st.site = &mk_emlrtRSI;
    quaternion_parenAssign(&c_st, &pnCorrected, (real_T *)&dp, t5_a_size,
                           (real_T *)&theta0, t5_b_size, (real_T *)&xd,
                           t5_c_size, (real_T *)&sinv, t5_d_size);
  }
  c_st.site = &nk_emlrtRSI;
  quaternionBase_normalize(&c_st, &pnCorrected);
  st.site = &ak_emlrtRSI;
  b_st.site = &ik_emlrtRSI;
  x = q0;
  c_st.site = &jk_emlrtRSI;
  quaternionBase_normalize(&c_st, &x);
  theta0 = x.c * 0.0;
  dp = x.d * 0.0;
  xd = x.b * 0.0;
  q1n_a = ((x.a - xd) - theta0) - dp;
  sinv = x.a * 0.0;
  q1n_b = ((sinv + x.b) + theta0) - dp;
  q1n_c = ((sinv - xd) + x.c) + dp;
  q1n_d = ((sinv + xd) - theta0) + x.d;
  x = qF;
  c_st.site = &kk_emlrtRSI;
  quaternionBase_normalize(&c_st, &x);
  xd = x.c * 0.0;
  theta0 = x.d * 0.0;
  sinv = x.b * 0.0;
  q2n_a = ((x.a - sinv) - xd) - theta0;
  dp = x.a * 0.0;
  q2n_b = ((dp + x.b) + xd) - theta0;
  q2n_c = ((dp - sinv) + x.c) + theta0;
  q2n_d = ((dp + sinv) - xd) + x.d;
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
  theta0 = muDoubleScalarAcos(dp);
  xd = muDoubleScalarSin(theta0);
  sinv = 1.0 / xd;
  y = muDoubleScalarSin(0.0 * theta0);
  q1n.a = sinv * (y * q1n_a + xd * q2n_a);
  q1n.b = sinv * (y * q1n_b + xd * q2n_b);
  q1n.c = sinv * (y * q1n_c + xd * q2n_c);
  q1n.d = sinv * (y * q1n_d + xd * q2n_d);
  if (muDoubleScalarIsInf(sinv)) {
    quaternion_parenReference(q0.a, q0.b, q0.c, q0.d, (real_T *)&dp, t5_a_size,
                              (real_T *)&theta0, t5_b_size, (real_T *)&xd,
                              t5_c_size, (real_T *)&sinv, t5_d_size);
    c_st.site = &mk_emlrtRSI;
    quaternion_parenAssign(&c_st, &q1n, (real_T *)&dp, t5_a_size,
                           (real_T *)&theta0, t5_b_size, (real_T *)&xd,
                           t5_c_size, (real_T *)&sinv, t5_d_size);
  }
  c_st.site = &nk_emlrtRSI;
  quaternionBase_normalize(&c_st, &q1n);
  q2n_a = ((pnCorrected.a * q1n.a - -pnCorrected.b * q1n.b) -
           -pnCorrected.c * q1n.c) -
          -pnCorrected.d * q1n.d;
  q2n_b = ((pnCorrected.a * q1n.b + -pnCorrected.b * q1n.a) +
           -pnCorrected.c * q1n.d) -
          -pnCorrected.d * q1n.c;
  q1n_d = ((pnCorrected.a * q1n.c - -pnCorrected.b * q1n.d) +
           -pnCorrected.c * q1n.a) +
          -pnCorrected.d * q1n.b;
  q1n_c = ((pnCorrected.a * q1n.d + -pnCorrected.b * q1n.c) -
           -pnCorrected.c * q1n.b) +
          -pnCorrected.d * q1n.a;
  for (i = 0; i < 101; i++) {
    real_T W_tmp_a;
    real_T W_tmp_b;
    real_T W_tmp_c;
    real_T W_tmp_d;
    real_T d;
    real_T d1;
    real_T qdCalc_a;
    real_T qdCalc_b;
    real_T qdCalc_c;
    real_T qdCalc_d;
    real_T xa;
    real_T xb;
    real_T xc;
    st.site = &bk_emlrtRSI;
    x = quaternionBase_slerp(&st, q0, qF, timeScaling[3 * i]);
    q1n_a = x.a;
    qCalc_a[i] = x.a;
    q1n_b = x.b;
    qCalc_b[i] = x.b;
    d = x.c;
    qCalc_c[i] = x.c;
    d1 = x.d;
    qCalc_d[i] = x.d;
    st.site = &ck_emlrtRSI;
    x.a = q2n_a;
    x.b = q2n_b;
    x.c = q1n_d;
    x.d = q1n_c;
    b_st.site = &tk_emlrtRSI;
    quaternionBase_log(&b_st, &x);
    b_i = 3 * i + 1;
    dp = timeScaling[b_i];
    qdCalc_a = dp * (((q1n_a * x.a - q1n_b * x.b) - d * x.c) - d1 * x.d);
    qdCalc_b = dp * (((q1n_a * x.b + q1n_b * x.a) + d * x.d) - d1 * x.c);
    qdCalc_c = dp * (((q1n_a * x.c - q1n_b * x.d) + d * x.a) + d1 * x.b);
    qdCalc_d = dp * (((q1n_a * x.d + q1n_b * x.c) - d * x.b) + d1 * x.a);
    W_tmp_a = 2.0 * qdCalc_a;
    W_tmp_b = 2.0 * qdCalc_b;
    W_tmp_c = 2.0 * qdCalc_c;
    W_tmp_d = 2.0 * qdCalc_d;
    omega[3 * i] =
        ((W_tmp_a * -q1n_b + W_tmp_b * q1n_a) + W_tmp_c * -d1) - W_tmp_d * -d;
    omega[b_i] =
        ((W_tmp_a * -d - W_tmp_b * -d1) + W_tmp_c * q1n_a) + W_tmp_d * -q1n_b;
    omega_tmp = 3 * i + 2;
    omega[omega_tmp] =
        ((W_tmp_a * -d1 + W_tmp_b * -d) - W_tmp_c * -q1n_b) + W_tmp_d * q1n_a;
    st.site = &dk_emlrtRSI;
    b_st.site = &al_emlrtRSI;
    x.a = q2n_a;
    x.b = q2n_b;
    x.c = q1n_d;
    x.d = q1n_c;
    c_st.site = &tk_emlrtRSI;
    quaternionBase_log(&c_st, &x);
    q1n.a = q2n_a;
    q1n.b = q2n_b;
    q1n.c = q1n_d;
    q1n.d = q1n_c;
    b_st.site = &bl_emlrtRSI;
    quaternionBase_log(&b_st, &q1n);
    theta0 = ((q1n_a * q1n.a - q1n_b * q1n.b) - d * q1n.c) - d1 * q1n.d;
    sinv = ((q1n_a * q1n.b + q1n_b * q1n.a) + d * q1n.d) - d1 * q1n.c;
    q2n_d = ((q1n_a * q1n.c - q1n_b * q1n.d) + d * q1n.a) + d1 * q1n.b;
    q2n_c = ((q1n_a * q1n.d + q1n_b * q1n.c) - d * q1n.b) + d1 * q1n.a;
    q1n.a = q2n_a;
    q1n.b = q2n_b;
    q1n.c = q1n_d;
    q1n.d = q1n_c;
    b_st.site = &bl_emlrtRSI;
    quaternionBase_log(&b_st, &q1n);
    b_st.site = &cl_emlrtRSI;
    y = dp * dp;
    dp = timeScaling[omega_tmp];
    xa = 2.0 * (y * (((theta0 * q1n.a - sinv * q1n.b) - q2n_d * q1n.c) -
                     q2n_c * q1n.d) +
                dp * (((q1n_a * x.a - q1n_b * x.b) - d * x.c) - d1 * x.d));
    xb = 2.0 * (y * (((theta0 * q1n.b + sinv * q1n.a) + q2n_d * q1n.d) -
                     q2n_c * q1n.c) +
                dp * (((q1n_a * x.b + q1n_b * x.a) + d * x.d) - d1 * x.c));
    xc = 2.0 * (y * (((theta0 * q1n.c - sinv * q1n.d) + q2n_d * q1n.a) +
                     q2n_c * q1n.b) +
                dp * (((q1n_a * x.c - q1n_b * x.d) + d * x.a) + d1 * x.b));
    xd = 2.0 * (y * (((theta0 * q1n.d + sinv * q1n.c) - q2n_d * q1n.b) +
                     q2n_c * q1n.a) +
                dp * (((q1n_a * x.d + q1n_b * x.c) - d * x.b) + d1 * x.a));
    qdCalc_b = -qdCalc_b;
    qdCalc_c = -qdCalc_c;
    qdCalc_d = -qdCalc_d;
    alpha[3 * i] =
        (((xa * -q1n_b + xb * q1n_a) + xc * -d1) - xd * -d) -
        (((W_tmp_a * qdCalc_b + W_tmp_b * qdCalc_a) + W_tmp_c * qdCalc_d) -
         W_tmp_d * qdCalc_c);
    alpha[b_i] =
        (((xa * -d - xb * -d1) + xc * q1n_a) + xd * -q1n_b) -
        (((W_tmp_a * qdCalc_c - W_tmp_b * qdCalc_d) + W_tmp_c * qdCalc_a) +
         W_tmp_d * qdCalc_b);
    alpha[omega_tmp] =
        (((xa * -d1 + xb * -d) - xc * -q1n_b) + xd * q1n_a) -
        (((W_tmp_a * qdCalc_d + W_tmp_b * qdCalc_c) - W_tmp_c * qdCalc_b) +
         W_tmp_d * qdCalc_a);
  }
  for (b_i = 0; b_i < 101; b_i++) {
    omega_tmp = b_i << 2;
    R[omega_tmp] = qCalc_a[b_i];
    R[omega_tmp + 1] = qCalc_b[b_i];
    R[omega_tmp + 2] = qCalc_c[b_i];
    R[omega_tmp + 3] = qCalc_d[b_i];
  }
}

/* End of code generation (rottraj.c) */
