/*
 * polyder.c
 *
 * Code generation for function 'polyder'
 *
 */

/* Include files */
#include "polyder.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
void b_polyder(const emlrtStack *sp, const real_T u[7], real_T a_data[],
               int32_T a_size[2])
{
  emlrtStack st;
  int32_T k;
  int32_T nlead0;
  int32_T ny;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ch_emlrtRSI;
  nlead0 = 0;
  k = 0;
  while ((k < 5) && (u[k] == 0.0)) {
    nlead0++;
    k++;
  }
  ny = 6 - nlead0;
  if (6 - nlead0 > 6) {
    emlrtErrorWithMessageIdR2018a(&st, &cb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  a_size[0] = 1;
  if (6 - nlead0 < 0) {
    emlrtNonNegativeCheckR2012b(6 - nlead0, &i_emlrtDCI, &st);
  }
  a_size[1] = 6 - nlead0;
  for (k = 0; k < ny; k++) {
    a_data[k] = u[k + nlead0];
  }
  ny = 6 - nlead0;
  for (k = 0; k <= ny - 2; k++) {
    a_data[k] *= (real_T)(5 - (nlead0 + k)) + 1.0;
  }
  if (muDoubleScalarIsInf(u[6]) || muDoubleScalarIsNaN(u[6])) {
    a_data[5 - nlead0] = rtNaN;
  }
}

void c_polyder(const emlrtStack *sp, const real_T u[6], real_T a_data[],
               int32_T a_size[2])
{
  emlrtStack st;
  int32_T k;
  int32_T nlead0;
  int32_T ny;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ch_emlrtRSI;
  nlead0 = 0;
  k = 0;
  while ((k < 4) && (u[k] == 0.0)) {
    nlead0++;
    k++;
  }
  ny = 5 - nlead0;
  if (5 - nlead0 > 5) {
    emlrtErrorWithMessageIdR2018a(&st, &cb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  a_size[0] = 1;
  if (5 - nlead0 < 0) {
    emlrtNonNegativeCheckR2012b(5 - nlead0, &i_emlrtDCI, &st);
  }
  a_size[1] = 5 - nlead0;
  for (k = 0; k < ny; k++) {
    a_data[k] = u[k + nlead0];
  }
  ny = 5 - nlead0;
  for (k = 0; k <= ny - 2; k++) {
    a_data[k] *= (real_T)(4 - (nlead0 + k)) + 1.0;
  }
  if (muDoubleScalarIsInf(u[5]) || muDoubleScalarIsNaN(u[5])) {
    a_data[4 - nlead0] = rtNaN;
  }
}

void polyder(const emlrtStack *sp, const real_T u[8], real_T a_data[],
             int32_T a_size[2])
{
  emlrtStack st;
  int32_T k;
  int32_T nlead0;
  int32_T ny;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ch_emlrtRSI;
  nlead0 = 0;
  k = 0;
  while ((k < 6) && (u[k] == 0.0)) {
    nlead0++;
    k++;
  }
  ny = 7 - nlead0;
  if (7 - nlead0 > 7) {
    emlrtErrorWithMessageIdR2018a(&st, &cb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  a_size[0] = 1;
  if (7 - nlead0 < 0) {
    emlrtNonNegativeCheckR2012b(7 - nlead0, &i_emlrtDCI, &st);
  }
  a_size[1] = 7 - nlead0;
  for (k = 0; k < ny; k++) {
    a_data[k] = u[k + nlead0];
  }
  ny = 7 - nlead0;
  for (k = 0; k <= ny - 2; k++) {
    a_data[k] *= (real_T)(6 - (nlead0 + k)) + 1.0;
  }
  if (muDoubleScalarIsInf(u[7]) || muDoubleScalarIsNaN(u[7])) {
    a_data[6 - nlead0] = rtNaN;
  }
}

/* End of code generation (polyder.c) */
