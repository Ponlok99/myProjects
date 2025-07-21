/*
 * File: run_p2p.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RUN_P2P_H
#define RUN_P2P_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void run_p2p(run_p2pStackData *SD, double wpts[2][3],
                    double orientations[2][3], p2pMode runP2pMode,
                    trajMode runTrajMode, double waypointVels[2][3],
                    double waypointAccels[2][3], double waypointJerks[2][3],
                    double pose[3][101], double qInterp[101][6],
                    double qdInterp[101][6], double jointTorq[101][6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for run_p2p.h
 *
 * [EOF]
 */
