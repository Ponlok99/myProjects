/*
 * wrapTo2Pi.c
 *
 * Code generation for function 'wrapTo2Pi'
 *
 */

/* Include files */
#include "wrapTo2Pi.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Definitions */
real_T wrapTo2Pi(real_T theta)
{
  real_T thetaWrap;
  thetaWrap = b_mod(theta);
  if ((thetaWrap == 0.0) && (theta > 0.0)) {
    thetaWrap = 6.2831853071795862;
  }
  return thetaWrap;
}

/* End of code generation (wrapTo2Pi.c) */
