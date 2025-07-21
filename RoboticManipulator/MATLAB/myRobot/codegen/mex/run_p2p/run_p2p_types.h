/*
 * run_p2p_types.h
 *
 * Code generation for function 'run_p2p'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include <stddef.h>

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

#ifndef c_typedef_c_robotics_manip_inte
#define c_typedef_c_robotics_manip_inte
typedef struct {
  void *CollisionPrimitive;
} c_robotics_manip_internal_Colli;
#endif /* c_typedef_c_robotics_manip_inte */

#ifndef d_typedef_c_robotics_manip_inte
#define d_typedef_c_robotics_manip_inte
typedef struct {
  real_T Length;
  char_T Vector[200];
} c_robotics_manip_internal_Chara;
#endif /* d_typedef_c_robotics_manip_inte */

#ifndef typedef_rigidBodyJoint
#define typedef_rigidBodyJoint
typedef struct {
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  c_robotics_manip_internal_Chara NameInternal;
  real_T HomePositionInternal[7];
  real_T JointAxisInternal[3];
  real_T MotionSubspaceInternal[36];
  c_robotics_manip_internal_Chara TypeInternal;
} rigidBodyJoint;
#endif /* typedef_rigidBodyJoint */

#ifndef c_typedef_c_emxArray_robotics_m
#define c_typedef_c_emxArray_robotics_m
typedef struct {
  c_robotics_manip_internal_Colli *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} c_emxArray_robotics_manip_inter;
#endif /* c_typedef_c_emxArray_robotics_m */

#ifndef c_typedef_d_robotics_manip_inte
#define c_typedef_d_robotics_manip_inte
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  c_emxArray_robotics_manip_inter *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
} d_robotics_manip_internal_Colli;
#endif /* c_typedef_d_robotics_manip_inte */

#ifndef e_typedef_c_robotics_manip_inte
#define e_typedef_c_robotics_manip_inte
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  c_robotics_manip_internal_Chara NameInternal;
  real_T Index;
  rigidBodyJoint JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
  d_robotics_manip_internal_Colli *CollisionsInternal;
} c_robotics_manip_internal_Rigid;
#endif /* e_typedef_c_robotics_manip_inte */

#ifndef d_typedef_d_robotics_manip_inte
#define d_typedef_d_robotics_manip_inte
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  c_robotics_manip_internal_Rigid Base;
  real_T Gravity[3];
  c_robotics_manip_internal_Rigid *Bodies[7];
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[14];
  real_T VelocityDoFMap[14];
  d_robotics_manip_internal_Colli _pobj0[22];
  c_robotics_manip_internal_Rigid _pobj1[14];
} d_robotics_manip_internal_Rigid;
#endif /* d_typedef_d_robotics_manip_inte */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_ptrdiff_t
#define struct_emxArray_ptrdiff_t
struct emxArray_ptrdiff_t {
  ptrdiff_t *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_ptrdiff_t */
#ifndef typedef_emxArray_ptrdiff_t
#define typedef_emxArray_ptrdiff_t
typedef struct emxArray_ptrdiff_t emxArray_ptrdiff_t;
#endif /* typedef_emxArray_ptrdiff_t */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  emxArray_real_T *grads;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef c_typedef_c_robotics_core_inter
#define c_typedef_c_robotics_core_inter
typedef struct {
  emxArray_real_T *ConstraintMatrix;
  emxArray_real_T *ConstraintBound;
  b_struct_T ExtraArgs;
  emxArray_real_T *SeedInternal;
} c_robotics_core_internal_Damped;
#endif /* c_typedef_c_robotics_core_inter */

#ifndef typedef_j_RigidBody_RigidBody
#define typedef_j_RigidBody_RigidBody
typedef struct {
  real_T vertices[19914];
} j_RigidBody_RigidBody;
#endif /* typedef_j_RigidBody_RigidBody */

#ifndef typedef_k_RigidBody_RigidBody
#define typedef_k_RigidBody_RigidBody
typedef struct {
  real_T vertices[26496];
} k_RigidBody_RigidBody;
#endif /* typedef_k_RigidBody_RigidBody */

#ifndef typedef_l_RigidBody_RigidBody
#define typedef_l_RigidBody_RigidBody
typedef struct {
  real_T vertices[40830];
} l_RigidBody_RigidBody;
#endif /* typedef_l_RigidBody_RigidBody */

#ifndef typedef_m_RigidBody_RigidBody
#define typedef_m_RigidBody_RigidBody
typedef struct {
  real_T vertices[37593];
} m_RigidBody_RigidBody;
#endif /* typedef_m_RigidBody_RigidBody */

#ifndef typedef_n_RigidBody_RigidBody
#define typedef_n_RigidBody_RigidBody
typedef struct {
  real_T vertices[49965];
} n_RigidBody_RigidBody;
#endif /* typedef_n_RigidBody_RigidBody */

#ifndef typedef_o_RigidBody_RigidBody
#define typedef_o_RigidBody_RigidBody
typedef struct {
  real_T vertices[46134];
} o_RigidBody_RigidBody;
#endif /* typedef_o_RigidBody_RigidBody */

#ifndef typedef_run_p2pStackData
#define typedef_run_p2pStackData
typedef struct {
  union {
    j_RigidBody_RigidBody f0;
    k_RigidBody_RigidBody f1;
    l_RigidBody_RigidBody f2;
    m_RigidBody_RigidBody f3;
    n_RigidBody_RigidBody f4;
    o_RigidBody_RigidBody f5;
  } u1;
} run_p2pStackData;
#endif /* typedef_run_p2pStackData */

/* End of code generation (run_p2p_types.h) */
