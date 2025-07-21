/*
 * File: run_p2p_internal_types.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RUN_P2P_INTERNAL_TYPES_H
#define RUN_P2P_INTERNAL_TYPES_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_types.h"

/* Type Definitions */
#ifndef typedef_quaternion
#define typedef_quaternion
typedef struct {
  double a;
  double b;
  double c;
  double d;
} quaternion;
#endif /* typedef_quaternion */

#ifndef typedef_rigidBodyTree
#define typedef_rigidBodyTree
typedef struct {
  bool matlabCodegenIsDeleted;
  d_robotics_manip_internal_Rigid *TreeInternal;
} rigidBodyTree;
#endif /* typedef_rigidBodyTree */

#endif
/*
 * File trailer for run_p2p_internal_types.h
 *
 * [EOF]
 */
