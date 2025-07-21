/*
 * run_p2p_emxutil.h
 *
 * Code generation for function 'run_p2p_emxutil'
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
void c_emxEnsureCapacity_robotics_ma(const emlrtStack *sp,
                                     c_emxArray_robotics_manip_inter *emxArray,
                                     int32_T oldNumel,
                                     const emlrtRTEInfo *srcLocation);

void c_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[22]);

void c_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct);

void c_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct);

void c_emxFree_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray);

void c_emxInitMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[22],
    const emlrtRTEInfo *srcLocation);

void c_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void c_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void c_emxInit_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray,
    const emlrtRTEInfo *srcLocation);

void d_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct);

void d_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct);

void d_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void d_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void e_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct);

void e_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_ptrdiff_t(const emlrtStack *sp,
                                 emxArray_ptrdiff_t *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxFreeStruct_struct_T(const emlrtStack *sp, b_struct_T *pStruct);

void emxFreeStruct_struct_T1(const emlrtStack *sp, b_struct_T *pStruct);

void emxFreeStruct_struct_T2(const emlrtStack *sp, b_struct_T *pStruct);

void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray);

void emxFree_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray);

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

void emxInitStruct_struct_T(const emlrtStack *sp, b_struct_T *pStruct,
                            const emlrtRTEInfo *srcLocation);

void emxInitStruct_struct_T1(const emlrtStack *sp, b_struct_T *pStruct,
                             const emlrtRTEInfo *srcLocation);

void emxInitStruct_struct_T3(const emlrtStack *sp, b_struct_T *pStruct,
                             const emlrtRTEInfo *srcLocation);

void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
                     int32_T numDimensions, const emlrtRTEInfo *srcLocation);

void emxInit_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray,
                       const emlrtRTEInfo *srcLocation);

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T numDimensions, const emlrtRTEInfo *srcLocation);

/* End of code generation (run_p2p_emxutil.h) */
