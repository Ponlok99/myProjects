/*
 * File: rigidBodyTree.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RIGIDBODYTREE_H
#define RIGIDBODYTREE_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_internal_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void rigidBodyTree_geometricJacobian(rigidBodyTree *obj, const double Q_data[],
                                     double Jac_data[], int Jac_size[2]);

void rigidBodyTree_inverseDynamics(rigidBodyTree *obj,
                                   const double varargin_1_data[],
                                   const int varargin_1_size[2],
                                   const double varargin_2[6],
                                   const double varargin_3[6],
                                   double tau_data[], int tau_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for rigidBodyTree.h
 *
 * [EOF]
 */
