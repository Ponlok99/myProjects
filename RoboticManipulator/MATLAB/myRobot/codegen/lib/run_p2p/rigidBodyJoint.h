/*
 * File: rigidBodyJoint.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RIGIDBODYJOINT_H
#define RIGIDBODYJOINT_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_rigidBodyJoint_get_MotionSubs(const rigidBodyJoint *obj,
                                     double msubspace_data[],
                                     int msubspace_size[2]);

void c_rigidBodyJoint_set_MotionSubs(rigidBodyJoint *obj,
                                     const double msubspace_data[]);

void c_rigidBodyJoint_transformBodyT(const rigidBodyJoint *obj,
                                     const double q_data[], int q_size,
                                     double T[4][4]);

void d_rigidBodyJoint_transformBodyT(const rigidBodyJoint *obj, double T[4][4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for rigidBodyJoint.h
 *
 * [EOF]
 */
