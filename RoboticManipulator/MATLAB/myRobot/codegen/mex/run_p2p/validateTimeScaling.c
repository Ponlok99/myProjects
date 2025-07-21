/*
 * validateTimeScaling.c
 *
 * Code generation for function 'validateTimeScaling'
 *
 */

/* Include files */
#include "validateTimeScaling.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ek_emlrtRSI = {
    10,                    /* lineNo */
    "validateTimeScaling", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\+"
    "robotics\\+core\\+internal\\validateTimeScalin"
    "g.m" /* pathName */
};

static emlrtRSInfo fk_emlrtRSI = {
    17,                    /* lineNo */
    "validateTimeScaling", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\+"
    "robotics\\+core\\+internal\\validateTimeScalin"
    "g.m" /* pathName */
};

static emlrtRSInfo gk_emlrtRSI = {
    29,                    /* lineNo */
    "validateTimeScaling", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotcore\\matlab\\+"
    "robotics\\+core\\+internal\\validateTimeScalin"
    "g.m" /* pathName */
};

/* Function Definitions */
void validateTimeScaling(const emlrtStack *sp,
                         const real_T timeScalingMatrix[303],
                         real_T validatedTimeScaling[303])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T timeScalingPos[101];
  int32_T i;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ek_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 303)) {
    if ((!muDoubleScalarIsInf(timeScalingMatrix[k])) &&
        (!muDoubleScalarIsNaN(timeScalingMatrix[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:rottraj:expectedFinite", 3, 4, 11, "TimeScaling");
  }
  st.site = &fk_emlrtRSI;
  for (k = 0; k < 101; k++) {
    real_T d;
    d = timeScalingMatrix[3 * k];
    timeScalingPos[k] = d;
    if ((d > 1.0) && (d < 1.0000000149011612)) {
      timeScalingPos[k] = 1.0;
    }
    if ((d < 0.0) && (d > -1.4901161193847656E-8)) {
      timeScalingPos[k] = 0.0;
    }
  }
  st.site = &gk_emlrtRSI;
  b_st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 101)) {
    if (timeScalingPos[k] >= 0.0) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(&b_st, &gb_emlrtRTEI,
                                  "MATLAB:validateattributes:expectedArray",
                                  "MATLAB:rottraj:notGreaterEqual", 9, 4, 16,
                                  "TimeScaling(1,:)", 4, 2, ">=", 4, 1, "0");
  }
  b_st.site = &vc_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 101)) {
    if (timeScalingPos[k] <= 1.0) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(&b_st, &hb_emlrtRTEI,
                                  "MATLAB:validateattributes:expectedArray",
                                  "MATLAB:rottraj:notLessEqual", 9, 4, 16,
                                  "TimeScaling(1,:)", 4, 2, "<=", 4, 1, "1");
  }
  for (i = 0; i < 101; i++) {
    validatedTimeScaling[3 * i] = timeScalingPos[i];
    k = 3 * i + 1;
    validatedTimeScaling[k] = timeScalingMatrix[k];
    k = 3 * i + 2;
    validatedTimeScaling[k] = timeScalingMatrix[k];
  }
}

/* End of code generation (validateTimeScaling.c) */
