/*
 * run_p2p_emxutil.c
 *
 * Code generation for function 'run_p2p_emxutil'
 *
 */

/* Include files */
#include "run_p2p_emxutil.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_types.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void c_emxEnsureCapacity_robotics_ma(const emlrtStack *sp,
                                     c_emxArray_robotics_manip_inter *emxArray,
                                     int32_T oldNumel,
                                     const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData =
        emlrtMallocMex((uint32_T)i * sizeof(c_robotics_manip_internal_Colli));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(c_robotics_manip_internal_Colli) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (c_robotics_manip_internal_Colli *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void c_emxFreeMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[22])
{
  int32_T i;
  for (i = 0; i < 22; i++) {
    c_emxFreeStruct_robotics_manip_(sp, &pMatrix[i]);
  }
}

void c_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct)
{
  emxFree_real_T(sp, &pStruct->ConstraintMatrix);
  emxFree_real_T(sp, &pStruct->ConstraintBound);
  emxFreeStruct_struct_T(sp, &pStruct->ExtraArgs);
  emxFree_real_T(sp, &pStruct->SeedInternal);
}

void c_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct)
{
  c_emxFree_robotics_manip_intern(sp, &pStruct->CollisionGeometries);
}

void c_emxFree_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray)
{
  if (*pEmxArray != (c_emxArray_robotics_manip_inter *)NULL) {
    if (((*pEmxArray)->data != (c_robotics_manip_internal_Colli *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (c_emxArray_robotics_manip_inter *)NULL;
  }
}

void c_emxInitMatrix_robotics_manip_(
    const emlrtStack *sp, d_robotics_manip_internal_Colli pMatrix[22],
    const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  for (i = 0; i < 22; i++) {
    d_emxInitStruct_robotics_manip_(sp, &pMatrix[i], srcLocation);
  }
}

void c_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->ConstraintMatrix, 2, srcLocation);
  emxInit_real_T(sp, &pStruct->ConstraintBound, 1, srcLocation);
  emxInitStruct_struct_T(sp, &pStruct->ExtraArgs, srcLocation);
  emxInit_real_T(sp, &pStruct->SeedInternal, 1, srcLocation);
}

void c_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  c_emxInitMatrix_robotics_manip_(sp, pStruct->_pobj0, srcLocation);
}

void c_emxInit_robotics_manip_intern(
    const emlrtStack *sp, c_emxArray_robotics_manip_inter **pEmxArray,
    const emlrtRTEInfo *srcLocation)
{
  c_emxArray_robotics_manip_inter *emxArray;
  int32_T i;
  *pEmxArray = (c_emxArray_robotics_manip_inter *)emlrtMallocEmxArray(
      sizeof(c_emxArray_robotics_manip_inter));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&c_emxFree_robotics_manip_intern,
                                      NULL, NULL, NULL);
  emxArray = *pEmxArray;
  emxArray->data = (c_robotics_manip_internal_Colli *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * 2U);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void d_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct)
{
  emxFree_real_T(sp, &pStruct->ConstraintMatrix);
  emxFree_real_T(sp, &pStruct->ConstraintBound);
  emxFreeStruct_struct_T1(sp, &pStruct->ExtraArgs);
  emxFree_real_T(sp, &pStruct->SeedInternal);
}

void d_emxFreeStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Rigid *pStruct)
{
  c_emxFreeMatrix_robotics_manip_(sp, pStruct->_pobj0);
}

void d_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->ConstraintMatrix, 2, srcLocation);
  emxInit_real_T(sp, &pStruct->ConstraintBound, 1, srcLocation);
  emxInitStruct_struct_T1(sp, &pStruct->ExtraArgs, srcLocation);
  emxInit_real_T(sp, &pStruct->SeedInternal, 1, srcLocation);
}

void d_emxInitStruct_robotics_manip_(const emlrtStack *sp,
                                     d_robotics_manip_internal_Colli *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  c_emxInit_robotics_manip_intern(sp, &pStruct->CollisionGeometries,
                                  srcLocation);
}

void e_emxFreeStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct)
{
  emxFree_real_T(sp, &pStruct->ConstraintMatrix);
  emxFree_real_T(sp, &pStruct->ConstraintBound);
  emxFreeStruct_struct_T2(sp, &pStruct->ExtraArgs);
  emxFree_real_T(sp, &pStruct->SeedInternal);
}

void e_emxInitStruct_robotics_core_i(const emlrtStack *sp,
                                     c_robotics_core_internal_Damped *pStruct,
                                     const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->ConstraintMatrix, 2, srcLocation);
  emxInit_real_T(sp, &pStruct->ConstraintBound, 1, srcLocation);
  emxInitStruct_struct_T3(sp, &pStruct->ExtraArgs, srcLocation);
  emxInit_real_T(sp, &pStruct->SeedInternal, 1, srcLocation);
}

void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T *emxArray,
                               int32_T oldNumel,
                               const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtMallocMex((uint32_T)i * sizeof(int32_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_ptrdiff_t(const emlrtStack *sp,
                                 emxArray_ptrdiff_t *emxArray, int32_T oldNumel,
                                 const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtMallocMex((uint32_T)i * sizeof(ptrdiff_t));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(ptrdiff_t) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (ptrdiff_t *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel, const emlrtRTEInfo *srcLocation)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel = (int32_T)emlrtSizeMulR2012b((size_t)(uint32_T)newNumel,
                                           (size_t)(uint32_T)emxArray->size[i],
                                           srcLocation, (emlrtCTX)sp);
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = emlrtMallocMex((uint32_T)i * sizeof(real_T));
    if (newData == NULL) {
      emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
    }
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxFreeStruct_struct_T(const emlrtStack *sp, b_struct_T *pStruct)
{
  emxFree_real_T(sp, &pStruct->grads);
}

void emxFreeStruct_struct_T1(const emlrtStack *sp, b_struct_T *pStruct)
{
  emxFree_real_T(sp, &pStruct->grads);
}

void emxFreeStruct_struct_T2(const emlrtStack *sp, b_struct_T *pStruct)
{
  emxFree_real_T(sp, &pStruct->grads);
}

void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

void emxFree_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray)
{
  if (*pEmxArray != (emxArray_ptrdiff_t *)NULL) {
    if (((*pEmxArray)->data != (ptrdiff_t *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_ptrdiff_t *)NULL;
  }
}

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference((emlrtCTX)sp, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void emxInitStruct_struct_T(const emlrtStack *sp, b_struct_T *pStruct,
                            const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->grads, 2, srcLocation);
}

void emxInitStruct_struct_T1(const emlrtStack *sp, b_struct_T *pStruct,
                             const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->grads, 2, srcLocation);
}

void emxInitStruct_struct_T3(const emlrtStack *sp, b_struct_T *pStruct,
                             const emlrtRTEInfo *srcLocation)
{
  emxInit_real_T(sp, &pStruct->grads, 2, srcLocation);
}

void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
                     int32_T numDimensions, const emlrtRTEInfo *srcLocation)
{
  emxArray_int32_T *emxArray;
  int32_T i;
  *pEmxArray =
      (emxArray_int32_T *)emlrtMallocEmxArray(sizeof(emxArray_int32_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_int32_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size =
      (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)numDimensions);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_ptrdiff_t(const emlrtStack *sp, emxArray_ptrdiff_t **pEmxArray,
                       const emlrtRTEInfo *srcLocation)
{
  emxArray_ptrdiff_t *emxArray;
  *pEmxArray =
      (emxArray_ptrdiff_t *)emlrtMallocEmxArray(sizeof(emxArray_ptrdiff_t));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_ptrdiff_t, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (ptrdiff_t *)NULL;
  emxArray->numDimensions = 1;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T));
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  emxArray->size[0] = 0;
}

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T numDimensions, const emlrtRTEInfo *srcLocation)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocEmxArray(sizeof(emxArray_real_T));
  if ((void *)*pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emlrtPushHeapReferenceStackEmxArray((emlrtCTX)sp, true, (void *)pEmxArray,
                                      (void *)&emxFree_real_T, NULL, NULL,
                                      NULL);
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size =
      (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)numDimensions);
  if ((void *)emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(srcLocation, (emlrtCTX)sp);
  }
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/* End of code generation (run_p2p_emxutil.c) */
