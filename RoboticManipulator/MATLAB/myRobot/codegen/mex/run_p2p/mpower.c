/*
 * mpower.c
 *
 * Code generation for function 'mpower'
 *
 */

/* Include files */
#include "mpower.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ae_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRSInfo be_emlrtRSI =
    {
        71,      /* lineNo */
        "power", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pathName
                                                                          */
};

static emlrtRTEInfo r_emlrtRTEI =
    {
        82,         /* lineNo */
        5,          /* colNo */
        "fltpower", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pName
                                                                          */
};

/* Function Definitions */
real_T mpower(const emlrtStack *sp, real_T a, real_T b)
{
  emlrtStack b_st;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ae_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &be_emlrtRSI;
  if ((a < 0.0) && (!muDoubleScalarIsNaN(b)) && (muDoubleScalarFloor(b) != b)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &r_emlrtRTEI,
                                  "Coder:toolbox:power_domainError",
                                  "Coder:toolbox:power_domainError", 0);
  }
  return muDoubleScalarPower(a, b);
}

/* End of code generation (mpower.c) */
