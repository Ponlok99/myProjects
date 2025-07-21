/*
 * File: _coder_run_p2p_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef _CODER_RUN_P2P_API_H
#define _CODER_RUN_P2P_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef enum_p2pMode
#define enum_p2pMode
enum p2pMode
{
  moveJ = 0, /* Default value */
  moveJI,
  moveL,
  moveJB,
  moveLB,
  moveJL,
  moveA,
  moveC
};
#endif /* enum_p2pMode */
#ifndef typedef_p2pMode
#define typedef_p2pMode
typedef enum p2pMode p2pMode;
#endif /* typedef_p2pMode */

#ifndef enum_trajMode
#define enum_trajMode
enum trajMode
{
  trapveltraj = 0, /* Default value */
  cubicpolytraj,
  quinticpolytraj,
  minjerkpolytraj,
  minsnappolytraj
};
#endif /* enum_trajMode */
#ifndef typedef_trajMode
#define typedef_trajMode
typedef enum trajMode trajMode;
#endif /* typedef_trajMode */

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void run_p2p(real_T wpts[2][3], real_T orientations[2][3], p2pMode runP2pMode,
             trajMode runTrajMode, real_T waypointVels[2][3],
             real_T waypointAccels[2][3], real_T waypointJerks[2][3],
             real_T pose[3][101], real_T qInterp[101][6],
             real_T qdInterp[101][6], real_T jointTorq[101][6]);

void run_p2p_api(const mxArray *const prhs[7], int32_T nlhs,
                 const mxArray *plhs[4]);

void run_p2p_atexit(void);

void run_p2p_initialize(void);

void run_p2p_terminate(void);

void run_p2p_xil_shutdown(void);

void run_p2p_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_run_p2p_api.h
 *
 * [EOF]
 */
