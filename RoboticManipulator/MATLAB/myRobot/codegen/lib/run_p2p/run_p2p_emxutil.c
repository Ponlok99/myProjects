/*
 * File: run_p2p_emxutil.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "run_p2p_emxutil.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"
#include <stdlib.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : c_emxArray_robotics_manip_inter *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void c_emxEnsureCapacity_robotics_ma(c_emxArray_robotics_manip_inter *emxArray,
                                     int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
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
    newData = malloc((unsigned int)i * sizeof(c_robotics_manip_internal_Colli));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(c_robotics_manip_internal_Colli) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (c_robotics_manip_internal_Colli *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : d_robotics_manip_internal_Colli pMatrix[22]
 * Return Type  : void
 */
void c_emxFreeMatrix_robotics_manip_(
    d_robotics_manip_internal_Colli pMatrix[22])
{
  int i;
  for (i = 0; i < 22; i++) {
    c_emxFreeStruct_robotics_manip_(&pMatrix[i]);
  }
}

/*
 * Arguments    : c_robotics_core_internal_Damped *pStruct
 * Return Type  : void
 */
void c_emxFreeStruct_robotics_core_i(c_robotics_core_internal_Damped *pStruct)
{
  emxFree_real_T(&pStruct->ConstraintMatrix);
  emxFree_real_T(&pStruct->ConstraintBound);
  emxFreeStruct_struct_T(&pStruct->ExtraArgs);
  emxFree_real_T(&pStruct->SeedInternal);
}

/*
 * Arguments    : d_robotics_manip_internal_Colli *pStruct
 * Return Type  : void
 */
void c_emxFreeStruct_robotics_manip_(d_robotics_manip_internal_Colli *pStruct)
{
  c_emxFree_robotics_manip_intern(&pStruct->CollisionGeometries);
}

/*
 * Arguments    : c_emxArray_robotics_manip_inter **pEmxArray
 * Return Type  : void
 */
void c_emxFree_robotics_manip_intern(
    c_emxArray_robotics_manip_inter **pEmxArray)
{
  if (*pEmxArray != (c_emxArray_robotics_manip_inter *)NULL) {
    if (((*pEmxArray)->data != (c_robotics_manip_internal_Colli *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (c_emxArray_robotics_manip_inter *)NULL;
  }
}

/*
 * Arguments    : d_robotics_manip_internal_Colli pMatrix[22]
 * Return Type  : void
 */
void c_emxInitMatrix_robotics_manip_(
    d_robotics_manip_internal_Colli pMatrix[22])
{
  int i;
  for (i = 0; i < 22; i++) {
    d_emxInitStruct_robotics_manip_(&pMatrix[i]);
  }
}

/*
 * Arguments    : c_robotics_core_internal_Damped *pStruct
 * Return Type  : void
 */
void c_emxInitStruct_robotics_core_i(c_robotics_core_internal_Damped *pStruct)
{
  emxInit_real_T(&pStruct->ConstraintMatrix, 2);
  emxInit_real_T(&pStruct->ConstraintBound, 1);
  emxInitStruct_struct_T(&pStruct->ExtraArgs);
  emxInit_real_T(&pStruct->SeedInternal, 1);
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void c_emxInitStruct_robotics_manip_(d_robotics_manip_internal_Rigid *pStruct)
{
  c_emxInitMatrix_robotics_manip_(pStruct->_pobj0);
}

/*
 * Arguments    : c_emxArray_robotics_manip_inter **pEmxArray
 * Return Type  : void
 */
void c_emxInit_robotics_manip_intern(
    c_emxArray_robotics_manip_inter **pEmxArray)
{
  c_emxArray_robotics_manip_inter *emxArray;
  int i;
  *pEmxArray = (c_emxArray_robotics_manip_inter *)malloc(
      sizeof(c_emxArray_robotics_manip_inter));
  emxArray = *pEmxArray;
  emxArray->data = (c_robotics_manip_internal_Colli *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int *)malloc(sizeof(int) * 2U);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void d_emxFreeStruct_robotics_manip_(d_robotics_manip_internal_Rigid *pStruct)
{
  c_emxFreeMatrix_robotics_manip_(pStruct->_pobj0);
}

/*
 * Arguments    : d_robotics_manip_internal_Colli *pStruct
 * Return Type  : void
 */
void d_emxInitStruct_robotics_manip_(d_robotics_manip_internal_Colli *pStruct)
{
  c_emxInit_robotics_manip_intern(&pStruct->CollisionGeometries);
}

/*
 * Arguments    : emxArray_int32_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
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
    newData = malloc((unsigned int)i * sizeof(int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_real_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
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
    newData = malloc((unsigned int)i * sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : struct_T *pStruct
 * Return Type  : void
 */
void emxFreeStruct_struct_T(struct_T *pStruct)
{
  emxFree_real_T(&pStruct->grads);
}

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 * Return Type  : void
 */
void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : struct_T *pStruct
 * Return Type  : void
 */
void emxInitStruct_struct_T(struct_T *pStruct)
{
  emxInit_real_T(&pStruct->grads, 2);
}

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real_T *emxArray
 * Return Type  : void
 */
void emxReserve_real_T(emxArray_real_T *emxArray)
{
  int i;
  void *newData;
  if (emxArray->allocatedSize < 10) {
    int numel;
    numel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
      numel *= emxArray->size[i];
    }
    newData = malloc(10U * sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * (unsigned int)numel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (double *)newData;
    emxArray->allocatedSize = 10;
    emxArray->canFreeData = true;
  }
}

/*
 * File trailer for run_p2p_emxutil.c
 *
 * [EOF]
 */
