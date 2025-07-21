/*
 * File: ikine_myRobot.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef IKINE_MYROBOT_H
#define IKINE_MYROBOT_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void applyJointLimits(const double inputConfig[3], double validConfig[3]);

void solveFirstThreeDHJoints(const double jt5Pos[3], double outputThetas_data[],
                             int outputThetas_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ikine_myRobot.h
 *
 * [EOF]
 */
