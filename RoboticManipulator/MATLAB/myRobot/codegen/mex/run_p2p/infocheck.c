/*
 * infocheck.c
 *
 * Code generation for function 'infocheck'
 *
 */

/* Include files */
#include "infocheck.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Definitions */
boolean_T infocheck(const emlrtStack *sp, int32_T info)
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'o', 'r', 'm', 'q', 'r'};
  boolean_T p;
  if (info != 0) {
    boolean_T b_p;
    p = true;
    b_p = false;
    if (info == -7) {
      b_p = true;
    } else if (info == -9) {
      b_p = true;
    } else if (info == -10) {
      b_p = true;
    }
    if (!b_p) {
      if (info == -1010) {
        emlrtErrorWithMessageIdR2018a(sp, &u_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(
            sp, &v_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
            "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, &fname[0], 12, info);
      }
    }
  } else {
    p = false;
  }
  return p;
}

/* End of code generation (infocheck.c) */
