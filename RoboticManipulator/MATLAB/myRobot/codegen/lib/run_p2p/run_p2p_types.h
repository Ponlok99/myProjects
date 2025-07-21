/*
 * File: run_p2p_types.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

#ifndef RUN_P2P_TYPES_H
#define RUN_P2P_TYPES_H

/* Include Files */
#include "rtwtypes.h"
#include "coder_posix_time.h"

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

#ifndef c_typedef_c_robotics_core_inter
#define c_typedef_c_robotics_core_inter
typedef struct {
  coderTimespec StartTime;
} c_robotics_core_internal_System;
#endif /* c_typedef_c_robotics_core_inter */

#ifndef c_typedef_c_robotics_manip_inte
#define c_typedef_c_robotics_manip_inte
typedef struct {
  double Length;
  char Vector[200];
} c_robotics_manip_internal_Chara;
#endif /* c_typedef_c_robotics_manip_inte */

#ifndef typedef_rigidBodyJoint
#define typedef_rigidBodyJoint
typedef struct {
  double VelocityNumber;
  double PositionNumber;
  bool InTree;
  double JointToParentTransform[4][4];
  double ChildToJointTransform[4][4];
  c_robotics_manip_internal_Chara NameInternal;
  double PositionLimitsInternal[2][7];
  double HomePositionInternal[7];
  double JointAxisInternal[3];
  double MotionSubspaceInternal[6][6];
  c_robotics_manip_internal_Chara TypeInternal;
} rigidBodyJoint;
#endif /* typedef_rigidBodyJoint */

#ifndef d_typedef_c_robotics_manip_inte
#define d_typedef_c_robotics_manip_inte
typedef struct {
  void *CollisionPrimitive;
  double LocalPose[4][4];
  double WorldPose[4][4];
  double MeshScale[3];
} c_robotics_manip_internal_Colli;
#endif /* d_typedef_c_robotics_manip_inte */

#ifndef e_typedef_c_robotics_manip_inte
#define e_typedef_c_robotics_manip_inte
typedef struct {
  char ShowTag[17];
  bool IsMaxReachUpToDate;
} c_robotics_manip_internal_Visua;
#endif /* e_typedef_c_robotics_manip_inte */

#ifndef c_typedef_c_emxArray_robotics_m
#define c_typedef_c_emxArray_robotics_m
typedef struct {
  c_robotics_manip_internal_Colli *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
} c_emxArray_robotics_manip_inter;
#endif /* c_typedef_c_emxArray_robotics_m */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  double constraints[3][10];
  double kt;
  double costWeightIdx[10];
  double stateSize;
  double numSegments;
  double segmentNumCoefficient;
  double segmentOrder;
  double cost;
  emxArray_real_T *grads;
} struct_T;
#endif /* typedef_struct_T */

#ifndef c_typedef_d_robotics_manip_inte
#define c_typedef_d_robotics_manip_inte
typedef struct {
  bool matlabCodegenIsDeleted;
  c_emxArray_robotics_manip_inter *CollisionGeometries;
  double MaxElements;
  double Size;
} d_robotics_manip_internal_Colli;
#endif /* c_typedef_d_robotics_manip_inte */

#ifndef f_typedef_c_robotics_manip_inte
#define f_typedef_c_robotics_manip_inte
typedef struct {
  bool matlabCodegenIsDeleted;
  c_robotics_manip_internal_Chara NameInternal;
  double Index;
  rigidBodyJoint JointInternal;
  double ParentIndex;
  double ChildrenIndices[7];
  double MassInternal;
  double CenterOfMassInternal[3];
  double InertiaInternal[3][3];
  double SpatialInertia[6][6];
  d_robotics_manip_internal_Colli *CollisionsInternal;
} c_robotics_manip_internal_Rigid;
#endif /* f_typedef_c_robotics_manip_inte */

#ifndef d_typedef_d_robotics_manip_inte
#define d_typedef_d_robotics_manip_inte
typedef struct {
  bool matlabCodegenIsDeleted;
  double NumBodies;
  c_robotics_manip_internal_Rigid Base;
  double Gravity[3];
  c_robotics_manip_internal_Rigid *Bodies[7];
  double NumNonFixedBodies;
  double PositionNumber;
  double VelocityNumber;
  double PositionDoFMap[2][7];
  double VelocityDoFMap[2][7];
  c_robotics_manip_internal_Visua VisualizationInfo;
  d_robotics_manip_internal_Colli _pobj0[22];
  c_robotics_manip_internal_Rigid _pobj1[14];
} d_robotics_manip_internal_Rigid;
#endif /* d_typedef_d_robotics_manip_inte */

#ifndef d_typedef_c_robotics_core_inter
#define d_typedef_c_robotics_core_inter
typedef struct {
  char Name[22];
  emxArray_real_T *ConstraintMatrix;
  emxArray_real_T *ConstraintBound;
  bool ConstraintsOn;
  double SolutionTolerance;
  bool RandomRestart;
  struct_T ExtraArgs;
  double MaxNumIteration;
  double MaxTime;
  emxArray_real_T *SeedInternal;
  double MaxTimeInternal;
  double MaxNumIterationInternal;
  double StepTolerance;
  c_robotics_core_internal_System TimeObj;
  double GradientTolerance;
  double ArmijoRuleBeta;
  double ArmijoRuleSigma;
  c_robotics_core_internal_System TimeObjInternal;
} c_robotics_core_internal_Damped;
#endif /* d_typedef_c_robotics_core_inter */

#ifndef typedef_i_RigidBody_RigidBody
#define typedef_i_RigidBody_RigidBody
typedef struct {
  double vertices[3][6638];
} i_RigidBody_RigidBody;
#endif /* typedef_i_RigidBody_RigidBody */

#ifndef typedef_j_RigidBody_RigidBody
#define typedef_j_RigidBody_RigidBody
typedef struct {
  double vertices[3][8832];
} j_RigidBody_RigidBody;
#endif /* typedef_j_RigidBody_RigidBody */

#ifndef typedef_k_RigidBody_RigidBody
#define typedef_k_RigidBody_RigidBody
typedef struct {
  double vertices[3][13610];
} k_RigidBody_RigidBody;
#endif /* typedef_k_RigidBody_RigidBody */

#ifndef typedef_l_RigidBody_RigidBody
#define typedef_l_RigidBody_RigidBody
typedef struct {
  double vertices[3][12531];
} l_RigidBody_RigidBody;
#endif /* typedef_l_RigidBody_RigidBody */

#ifndef typedef_m_RigidBody_RigidBody
#define typedef_m_RigidBody_RigidBody
typedef struct {
  double vertices[3][16655];
} m_RigidBody_RigidBody;
#endif /* typedef_m_RigidBody_RigidBody */

#ifndef typedef_n_RigidBody_RigidBody
#define typedef_n_RigidBody_RigidBody
typedef struct {
  double vertices[3][15378];
} n_RigidBody_RigidBody;
#endif /* typedef_n_RigidBody_RigidBody */

#ifndef typedef_run_p2pPersistentData
#define typedef_run_p2pPersistentData
typedef struct {
  unsigned int state[625];
} run_p2pPersistentData;
#endif /* typedef_run_p2pPersistentData */

#ifndef typedef_run_p2pStackData
#define typedef_run_p2pStackData
typedef struct {
  union {
    i_RigidBody_RigidBody f0;
    j_RigidBody_RigidBody f1;
    k_RigidBody_RigidBody f2;
    l_RigidBody_RigidBody f3;
    m_RigidBody_RigidBody f4;
    n_RigidBody_RigidBody f5;
  } u1;
  run_p2pPersistentData *pd;
} run_p2pStackData;
#endif /* typedef_run_p2pStackData */

#endif
/*
 * File trailer for run_p2p_types.h
 *
 * [EOF]
 */
