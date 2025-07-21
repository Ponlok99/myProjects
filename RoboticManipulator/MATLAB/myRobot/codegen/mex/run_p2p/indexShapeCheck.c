/*
 * indexShapeCheck.c
 *
 * Code generation for function 'indexShapeCheck'
 *
 */

/* Include files */
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Variable Definitions */
static emlrtRSInfo ie_emlrtRSI = {
    42,                /* lineNo */
    "indexShapeCheck", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\indexShapeCheck.m" /* pathName */
};

static emlrtRTEInfo t_emlrtRTEI = {
    122,           /* lineNo */
    5,             /* colNo */
    "errOrWarnIf", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\indexShapeCheck.m" /* pName */
};

/* Function Definitions */
void indexShapeCheck(const emlrtStack *sp, int32_T matrixSize,
                     const int32_T indexSize[2])
{
  emlrtStack st;
  boolean_T c;
  st.prev = sp;
  st.tls = sp->tls;
  if ((matrixSize == 1) && (indexSize[1] != 1)) {
    c = true;
  } else {
    c = false;
  }
  st.site = &ie_emlrtRSI;
  if (c) {
    emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI,
                                  "Coder:FE:PotentialVectorVector",
                                  "Coder:FE:PotentialVectorVector", 0);
  }
}

/* End of code generation (indexShapeCheck.c) */
