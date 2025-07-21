/*
 * File: RigidBodyTreeDynamics.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RIGIDBODYTREEDYNAMICS_H
#define RIGIDBODYTREEDYNAMICS_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int c_RigidBodyTreeDynamics_inverse(d_robotics_manip_internal_Rigid *robot,
                                    const double q_data[],
                                    const double qdot_data[],
                                    const double qddot_data[],
                                    const double fext_data[],
                                    double tau_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for RigidBodyTreeDynamics.h
 *
 * [EOF]
 */
