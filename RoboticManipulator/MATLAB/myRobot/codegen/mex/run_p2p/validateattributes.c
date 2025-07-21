/*
 * validateattributes.c
 *
 * Code generation for function 'validateattributes'
 *
 */

/* Include files */
#include "validateattributes.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
void validateattributes(const emlrtStack *sp, const real_T a_data[],
                        const int32_T a_size[2],
                        const real_T attributesMixed_f5[2])
{
  emlrtStack st;
  real_T d;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= a_size[1] - 1)) {
    if (!muDoubleScalarIsNaN(a_data[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &ib_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:validateDynamicsFunctionInputs:expectedNonNaN", 3, 4, 25,
        "joint position vector (q)");
  }
  st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= a_size[1] - 1)) {
    if ((!muDoubleScalarIsInf(a_data[k])) &&
        (!muDoubleScalarIsNaN(a_data[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:validateDynamicsFunctionInputs:expectedFinite", 3, 4, 25,
        "joint position vector (q)");
  }
  st.site = &vc_emlrtRSI;
  p = true;
  for (k = 0; k < 2; k++) {
    if (p) {
      d = attributesMixed_f5[k];
      if ((!(d != d)) && (muDoubleScalarIsInf(d) || (!(d >= 0.0)) ||
                          (!(d == muDoubleScalarFloor(d))))) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(&st, &rb_emlrtRTEI,
                                  "MATLAB:validateattributes:badSizeArray",
                                  "MATLAB:validateattributes:badSizeArray", 0);
  }
  p = true;
  for (k = 0; k < 2; k++) {
    if (p) {
      d = attributesMixed_f5[k];
      if ((!(d != d)) && (!(d == a_size[k]))) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &st, &sb_emlrtRTEI, "Coder:toolbox:ValidateattributesincorrectSize",
        "MATLAB:validateDynamicsFunctionInputs:incorrectSize", 3, 4, 25,
        "joint position vector (q)");
  }
}

/* End of code generation (validateattributes.c) */
