/*
 * File: handle.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "handle.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"
#include "collisioncodegen_api.hpp"

/* Function Definitions */
/*
 * Arguments    : d_robotics_manip_internal_Colli *obj
 * Return Type  : void
 */
void handle_matlabCodegenDestructor(d_robotics_manip_internal_Colli *obj)
{
  c_robotics_manip_internal_Colli b_obj;
  int b_i;
  if (!obj->matlabCodegenIsDeleted) {
    double d;
    int i;
    obj->matlabCodegenIsDeleted = true;
    d = obj->Size;
    i = (int)d;
    for (b_i = 0; b_i < i; b_i++) {
      b_obj = obj->CollisionGeometries->data[b_i];
      collisioncodegen_destructGeometry(&b_obj.CollisionPrimitive);
      obj->CollisionGeometries->data[b_i] = b_obj;
    }
  }
}

/*
 * File trailer for handle.c
 *
 * [EOF]
 */
