/*
 * eul2quat.c
 *
 * Code generation for function 'eul2quat'
 *
 */

/* Include files */
#include "eul2quat.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void eul2quat(const real_T eul[3], real_T q[4])
{
  real_T ein[3];
  real_T cosa;
  real_T cosb;
  real_T cosc;
  real_T q_tmp;
  real_T sina;
  real_T sinb;
  real_T sinc;
  _mm_storeu_pd(&ein[0], _mm_div_pd(_mm_loadu_pd(&eul[0]), _mm_set1_pd(2.0)));
  ein[2] = eul[2] / 2.0;
  sina = muDoubleScalarSin(ein[0]);
  sinb = muDoubleScalarSin(ein[1]);
  sinc = muDoubleScalarSin(ein[2]);
  cosa = muDoubleScalarCos(ein[0]);
  cosb = muDoubleScalarCos(ein[1]);
  cosc = muDoubleScalarCos(ein[2]);
  q_tmp = cosa * cosb;
  q[0] = q_tmp * cosc + sina * sinb * sinc;
  q[1] = q_tmp * sinc - cosc * sina * sinb;
  q[2] = cosa * cosc * sinb + cosb * sina * sinc;
  q[3] = cosb * cosc * sina - cosa * sinb * sinc;
}

/* End of code generation (eul2quat.c) */
