/*
 * File: run_p2p_initialize.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "run_p2p_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"

/* Function Definitions */
/*
 * Arguments    : run_p2pStackData *SD
 * Return Type  : void
 */
void run_p2p_initialize(run_p2pStackData *SD)
{
  c_eml_rand_mt19937ar_stateful_i(SD);
}

/*
 * File trailer for run_p2p_initialize.c
 *
 * [EOF]
 */
