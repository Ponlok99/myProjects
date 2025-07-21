/*
 * run_p2p.h
 *
 * Code generation for function 'run_p2p'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void run_p2p(run_p2pStackData *SD, const emlrtStack *sp, const real_T wpts[6],
             const real_T orientations[6], p2pMode runP2pMode,
             trajMode runTrajMode, const real_T waypointVels[6],
             const real_T waypointAccels[6], const real_T waypointJerks[6],
             real_T pose[303], real_T qInterp[606], real_T qdInterp[606],
             real_T jointTorq[606]);

/* End of code generation (run_p2p.h) */
