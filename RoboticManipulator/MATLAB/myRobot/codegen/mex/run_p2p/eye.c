/*
 * eye.c
 *
 * Code generation for function 'eye'
 *
 */

/* Include files */
#include "eye.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include <string.h>

/* Function Definitions */
void eye(real_T b_I[16])
{
  memset(&b_I[0], 0, 16U * sizeof(real_T));
  b_I[0] = 1.0;
  b_I[5] = 1.0;
  b_I[10] = 1.0;
  b_I[15] = 1.0;
}

/* End of code generation (eye.c) */
