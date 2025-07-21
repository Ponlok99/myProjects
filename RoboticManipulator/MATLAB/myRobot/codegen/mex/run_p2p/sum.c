/*
 * sum.c
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include <emmintrin.h>

/* Function Definitions */
void sum(const real_T x[4], real_T y[2])
{
  _mm_storeu_pd(&y[0], _mm_add_pd(_mm_loadu_pd(&x[0]), _mm_loadu_pd(&x[2])));
}

/* End of code generation (sum.c) */
