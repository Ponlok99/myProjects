/*
 * run_p2p_internal_types.h
 *
 * Code generation for function 'run_p2p'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "run_p2p_types.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_quaternion
#define typedef_quaternion
typedef struct {
  real_T a;
  real_T b;
  real_T c;
  real_T d;
} quaternion;
#endif /* typedef_quaternion */

#ifndef typedef_cell_wrap_63
#define typedef_cell_wrap_63
typedef struct {
  real_T f1[16];
} cell_wrap_63;
#endif /* typedef_cell_wrap_63 */

#ifndef typedef_rigidBodyTree
#define typedef_rigidBodyTree
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  d_robotics_manip_internal_Rigid *TreeInternal;
} rigidBodyTree;
#endif /* typedef_rigidBodyTree */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef typedef_rtDesignRangeCheckInfo
#define typedef_rtDesignRangeCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtDesignRangeCheckInfo;
#endif /* typedef_rtDesignRangeCheckInfo */

#ifndef typedef_rtRunTimeErrorInfo
#define typedef_rtRunTimeErrorInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtRunTimeErrorInfo;
#endif /* typedef_rtRunTimeErrorInfo */

/* End of code generation (run_p2p_internal_types.h) */
