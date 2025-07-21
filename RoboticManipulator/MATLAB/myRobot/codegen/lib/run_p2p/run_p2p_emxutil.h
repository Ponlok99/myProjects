/*
 * File: run_p2p_emxutil.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RUN_P2P_EMXUTIL_H
#define RUN_P2P_EMXUTIL_H

/* Include Files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void
c_emxEnsureCapacity_robotics_ma(c_emxArray_robotics_manip_inter *emxArray,
                                int oldNumel);

extern void
c_emxFreeMatrix_robotics_manip_(d_robotics_manip_internal_Colli pMatrix[22]);

extern void
c_emxFreeStruct_robotics_core_i(c_robotics_core_internal_Damped *pStruct);

extern void
c_emxFreeStruct_robotics_manip_(d_robotics_manip_internal_Colli *pStruct);

extern void
c_emxFree_robotics_manip_intern(c_emxArray_robotics_manip_inter **pEmxArray);

extern void
c_emxInitMatrix_robotics_manip_(d_robotics_manip_internal_Colli pMatrix[22]);

extern void
c_emxInitStruct_robotics_core_i(c_robotics_core_internal_Damped *pStruct);

extern void
c_emxInitStruct_robotics_manip_(d_robotics_manip_internal_Rigid *pStruct);

extern void
c_emxInit_robotics_manip_intern(c_emxArray_robotics_manip_inter **pEmxArray);

extern void
d_emxFreeStruct_robotics_manip_(d_robotics_manip_internal_Rigid *pStruct);

extern void
d_emxInitStruct_robotics_manip_(d_robotics_manip_internal_Colli *pStruct);

extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFreeStruct_struct_T(struct_T *pStruct);

extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInitStruct_struct_T(struct_T *pStruct);

extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

extern void emxReserve_real_T(emxArray_real_T *emxArray);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for run_p2p_emxutil.h
 *
 * [EOF]
 */
