/*
 * File: quaternion.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "quaternion.h"
#include "rt_nonfinite.h"
#include "run_p2p_internal_types.h"

/* Function Definitions */
/*
 * Arguments    : quaternion *obj
 *                const double rhs_a_data[]
 *                const double rhs_b_data[]
 *                const double rhs_c_data[]
 *                const double rhs_d_data[]
 * Return Type  : void
 */
void quaternion_parenAssign(quaternion *obj, const double rhs_a_data[],
                            const double rhs_b_data[],
                            const double rhs_c_data[],
                            const double rhs_d_data[])
{
  obj->a = rhs_a_data[0];
  obj->b = rhs_b_data[0];
  obj->c = rhs_c_data[0];
  obj->d = rhs_d_data[0];
}

/*
 * File trailer for quaternion.c
 *
 * [EOF]
 */
