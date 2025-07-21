/*
 * round.c
 *
 * Code generation for function 'round'
 *
 */

/* Include files */
#include "round.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "mwmathutil.h"

/* Function Definitions */
void b_round(real_T x_data[], const int32_T x_size[2])
{
  int32_T i;
  int32_T k;
  i = x_size[0] * x_size[1];
  for (k = 0; k < i; k++) {
    x_data[k] = muDoubleScalarRound(x_data[k]);
  }
}

/* End of code generation (round.c) */
