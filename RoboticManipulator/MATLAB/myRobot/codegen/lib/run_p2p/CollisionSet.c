/*
 * File: CollisionSet.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "CollisionSet.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"

/* Function Definitions */
/*
 * Arguments    : d_robotics_manip_internal_Colli *obj
 *                double maxElements
 * Return Type  : d_robotics_manip_internal_Colli *
 */
d_robotics_manip_internal_Colli *
CollisionSet_CollisionSet(d_robotics_manip_internal_Colli *obj,
                          double maxElements)
{
  static const void *t0_GeometryInternal = NULL;
  c_robotics_manip_internal_Colli expl_temp;
  d_robotics_manip_internal_Colli *b_obj;
  double d;
  int i;
  int i1;
  int size_tmp_idx_1;
  b_obj = obj;
  b_obj->Size = 0.0;
  b_obj->MaxElements = maxElements;
  size_tmp_idx_1 = (int)b_obj->MaxElements;
  i = b_obj->CollisionGeometries->size[0] * b_obj->CollisionGeometries->size[1];
  b_obj->CollisionGeometries->size[0] = 1;
  b_obj->CollisionGeometries->size[1] = size_tmp_idx_1;
  c_emxEnsureCapacity_robotics_ma(b_obj->CollisionGeometries, i);
  d = b_obj->MaxElements;
  i = (int)d;
  for (size_tmp_idx_1 = 0; size_tmp_idx_1 < i; size_tmp_idx_1++) {
    expl_temp.CollisionPrimitive = (void *)t0_GeometryInternal;
    for (i1 = 0; i1 < 4; i1++) {
      expl_temp.LocalPose[i1][0] = iv[i1][0];
      expl_temp.LocalPose[i1][1] = iv[i1][1];
      expl_temp.LocalPose[i1][2] = iv[i1][2];
      expl_temp.LocalPose[i1][3] = iv[i1][3];
    }
    for (i1 = 0; i1 < 4; i1++) {
      expl_temp.WorldPose[i1][0] = iv[i1][0];
      expl_temp.WorldPose[i1][1] = iv[i1][1];
      expl_temp.WorldPose[i1][2] = iv[i1][2];
      expl_temp.WorldPose[i1][3] = iv[i1][3];
    }
    expl_temp.MeshScale[0] = 1.0;
    expl_temp.MeshScale[1] = 1.0;
    expl_temp.MeshScale[2] = 1.0;
    b_obj->CollisionGeometries->data[size_tmp_idx_1] = expl_temp;
  }
  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

/*
 * File trailer for CollisionSet.c
 *
 * [EOF]
 */
