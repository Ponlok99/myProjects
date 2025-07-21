/*
 * File: importrobot.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "importrobot.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "rand.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : run_p2pStackData *SD
 *                d_robotics_manip_internal_Rigid *iobj_0
 *                rigidBodyTree **iobj_1
 * Return Type  : void
 */
void importrobot(run_p2pStackData *SD, d_robotics_manip_internal_Rigid *iobj_0,
                 rigidBodyTree **iobj_1)
{
  static const double dv[4][4] = {{1.0, 0.0, -0.0, 0.0},
                                  {0.0, 1.0, 0.0, 0.0},
                                  {0.0, 0.0, 1.0, 0.0},
                                  {0.046, 0.0, 0.0, 1.0}};
  static const char cv1[62] = {
      'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
      'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
      '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C',
      'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
      'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};
  static const signed char iv3[2][7] = {{1, 2, 3, 4, 5, 6, 0},
                                        {1, 2, 3, 4, 5, 6, -1}};
  static const char b_cv[12] = {'D', 'O', '_', 'N', 'O', 'T',
                                '_', 'E', 'D', 'I', 'T', '_'};
  static const char cv10[12] = {'j', 'o', 'i', 'n', 't', '6',
                                '-', 't', 'o', 'o', 'l', '0'};
  static const char cv2[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '1'};
  static const char cv3[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '2'};
  static const char cv4[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '3'};
  static const char cv5[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '4'};
  static const char cv6[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '5'};
  static const char cv7[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '6'};
  static const char cv8[10] = {'d', 'u', 'm', 'm', 'y',
                               'b', 'o', 'd', 'y', '7'};
  static const char cv12[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char cv13[8] = {'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7] = {1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6] = {0, 0, 1, 0, 0, 0};
  static const signed char iv1[6] = {0, 0, 0, 0, 0, 1};
  static const char cv11[5] = {'f', 'i', 'x', 'e', 'd'};
  static const char cv14[5] = {'f', 'i', 'x', 'e', 'd'};
  static const char cv9[5] = {'t', 'o', 'o', 'l', '0'};
  c_robotics_manip_internal_Chara s;
  double msubspace_data[36];
  double poslim_data[14];
  double idx[5];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int i1;
  int k;
  int poslim_size_idx_0;
  char obj_ShowTag[17];
  signed char homepos_data[7];
  bool result;
  b_rand(SD, idx);
  for (k = 0; k < 5; k++) {
    idx[k] = floor(idx[k] * 62.0) + 1.0;
  }
  for (i = 0; i < 12; i++) {
    obj_ShowTag[i] = b_cv[i];
  }
  for (i = 0; i < 5; i++) {
    obj_ShowTag[i + 12] = cv1[(int)idx[i] - 1];
  }
  for (i = 0; i < 17; i++) {
    iobj_0->VisualizationInfo.ShowTag[i] = obj_ShowTag[i];
  }
  iobj_0->VisualizationInfo.IsMaxReachUpToDate = false;
  iobj_0->NumBodies = 7.0;
  iobj_0->Bodies[0] =
      RigidBody_RigidBody(&iobj_0->_pobj1[0], cv2, &iobj_0->_pobj0[0]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[0]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[1] =
      RigidBody_RigidBody(&iobj_0->_pobj1[1], cv3, &iobj_0->_pobj0[1]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[1]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[2] =
      RigidBody_RigidBody(&iobj_0->_pobj1[2], cv4, &iobj_0->_pobj0[2]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[2]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[3] =
      RigidBody_RigidBody(&iobj_0->_pobj1[3], cv5, &iobj_0->_pobj0[3]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[3]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[4] =
      RigidBody_RigidBody(&iobj_0->_pobj1[4], cv6, &iobj_0->_pobj0[4]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[4]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[5] =
      RigidBody_RigidBody(&iobj_0->_pobj1[5], cv7, &iobj_0->_pobj0[5]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[5]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[6] =
      RigidBody_RigidBody(&iobj_0->_pobj1[6], cv8, &iobj_0->_pobj0[6]);
  for (i = 0; i < 7; i++) {
    iobj_0->Bodies[6]->ChildrenIndices[i] = 0.0;
  }
  iobj_0->Bodies[0] =
      b_RigidBody_RigidBody(SD, &iobj_0->_pobj1[7], &iobj_0->_pobj0[7]);
  iobj_0->Bodies[0]->Index = 1.0;
  iobj_0->Bodies[1] =
      c_RigidBody_RigidBody(SD, &iobj_0->_pobj1[8], &iobj_0->_pobj0[9]);
  iobj_0->Bodies[1]->Index = 2.0;
  iobj_0->Bodies[2] =
      d_RigidBody_RigidBody(SD, &iobj_0->_pobj1[9], &iobj_0->_pobj0[11]);
  iobj_0->Bodies[2]->Index = 3.0;
  iobj_0->Bodies[3] =
      e_RigidBody_RigidBody(SD, &iobj_0->_pobj1[10], &iobj_0->_pobj0[13]);
  iobj_0->Bodies[3]->Index = 4.0;
  iobj_0->Bodies[4] =
      f_RigidBody_RigidBody(SD, &iobj_0->_pobj1[11], &iobj_0->_pobj0[15]);
  iobj_0->Bodies[4]->Index = 5.0;
  iobj_0->Bodies[5] =
      g_RigidBody_RigidBody(&iobj_0->_pobj1[12], &iobj_0->_pobj0[17]);
  iobj_0->Bodies[5]->Index = 6.0;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_0->_pobj1[13].NameInternal = s;
  s = iobj_0->_pobj1[13].NameInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = cv9[i];
  }
  iobj_0->_pobj1[13].NameInternal = s;
  iobj_0->_pobj1[13].ParentIndex = 6.0;
  for (i = 0; i < 7; i++) {
    iobj_0->_pobj1[13].ChildrenIndices[i] = 0.0;
  }
  iobj_0->_pobj1[13].MassInternal = 0.0;
  iobj_0->_pobj1[13].CenterOfMassInternal[0] = 0.0;
  iobj_0->_pobj1[13].CenterOfMassInternal[1] = 0.0;
  iobj_0->_pobj1[13].CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 3; i++) {
    iobj_0->_pobj1[13].InertiaInternal[i][0] = 0.0;
    iobj_0->_pobj1[13].InertiaInternal[i][1] = 0.0;
    iobj_0->_pobj1[13].InertiaInternal[i][2] = 0.0;
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      iobj_0->_pobj1[13].SpatialInertia[i][i1] = 0.0;
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      iobj_0->_pobj1[13].JointInternal.PositionLimitsInternal[i][i1] = 0.0;
    }
  }
  for (i = 0; i < 7; i++) {
    iobj_0->_pobj1[13].JointInternal.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      iobj_0->_pobj1[13].JointInternal.MotionSubspaceInternal[i][i1] = 0.0;
    }
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_0->_pobj1[13].JointInternal.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_0->_pobj1[13].JointInternal.TypeInternal = s;
  s = iobj_0->_pobj1[13].JointInternal.NameInternal;
  s.Length = 12.0;
  for (i = 0; i < 12; i++) {
    s.Vector[i] = cv10[i];
  }
  iobj_0->_pobj1[13].JointInternal.NameInternal = s;
  s = iobj_0->_pobj1[13].JointInternal.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = cv11[i];
  }
  iobj_0->_pobj1[13].JointInternal.TypeInternal = s;
  s = iobj_0->_pobj1[13].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = (int)s.Length;
  }
  result = false;
  if (i == 8) {
    k = 0;
    do {
      exitg1 = 0;
      if (k < 8) {
        if (cv12[k] != s.Vector[k]) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    k = 0;
  } else {
    result = false;
    if (i == 9) {
      k = 0;
      do {
        exitg1 = 0;
        if (k < 9) {
          if (cv[k] != s.Vector[k]) {
            exitg1 = 1;
          } else {
            k++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      k = 1;
    } else {
      result = false;
      if (i == 8) {
        k = 0;
        do {
          exitg1 = 0;
          if (k < 8) {
            if (cv13[k] != s.Vector[k]) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        k = 2;
      } else {
        k = -1;
      }
    }
  }
  switch (k) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_0->_pobj1[13].JointInternal.VelocityNumber = 1.0;
    iobj_0->_pobj1[13].JointInternal.PositionNumber = 1.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv1[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_0->_pobj1[13].JointInternal.VelocityNumber = 1.0;
    iobj_0->_pobj1[13].JointInternal.PositionNumber = 1.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[6][6];
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        b_I[i][i1] = 0;
      }
    }
    for (k = 0; k < 6; k++) {
      b_I[k][k] = 1;
    }
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        msubspace_data[i1 + 6 * i] = b_I[i][i1];
      }
    }
    poslim_size_idx_0 = 7;
    for (k = 0; k < 2; k++) {
      signed char b_tmp;
      b_tmp = (signed char)(10 * k - 5);
      poslim_data[7 * k] = rtNaN;
      poslim_data[7 * k + 1] = rtNaN;
      poslim_data[7 * k + 2] = rtNaN;
      poslim_data[7 * k + 3] = rtNaN;
      poslim_data[7 * k + 4] = b_tmp;
      poslim_data[7 * k + 5] = b_tmp;
      poslim_data[7 * k + 6] = b_tmp;
    }
    homepos_size_idx_1 = 7;
    for (i = 0; i < 7; i++) {
      homepos_data[i] = iv2[i];
    }
    iobj_0->_pobj1[13].JointInternal.VelocityNumber = 6.0;
    iobj_0->_pobj1[13].JointInternal.PositionNumber = 7.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[0] = rtNaN;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[1] = rtNaN;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[2] = rtNaN;
  } break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0.0;
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_0->_pobj1[13].JointInternal.VelocityNumber = 0.0;
    iobj_0->_pobj1[13].JointInternal.PositionNumber = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0->_pobj1[13].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }
  c_rigidBodyJoint_set_MotionSubs(&iobj_0->_pobj1[13].JointInternal,
                                  msubspace_data);
  s = iobj_0->_pobj1[13].JointInternal.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = (int)s.Length;
  }
  result = false;
  if (i == 5) {
    k = 0;
    do {
      exitg1 = 0;
      if (k < 5) {
        if (s.Vector[k] != cv14[k]) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!result) {
    double d;
    d = iobj_0->_pobj1[13].JointInternal.PositionNumber;
    if (d < 1.0) {
      k = 0;
    } else {
      k = (int)d;
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < k; i1++) {
        iobj_0->_pobj1[13].JointInternal.PositionLimitsInternal[i][i1] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_0->_pobj1[13].JointInternal.HomePositionInternal[i] =
          homepos_data[i];
    }
  } else {
    iobj_0->_pobj1[13].JointInternal.PositionLimitsInternal[0][0] =
        poslim_data[0];
    iobj_0->_pobj1[13].JointInternal.PositionLimitsInternal[1][0] =
        poslim_data[1];
    iobj_0->_pobj1[13].JointInternal.HomePositionInternal[0] = homepos_data[0];
  }
  for (i = 0; i < 4; i++) {
    iobj_0->_pobj1[13].JointInternal.JointToParentTransform[i][0] = dv[i][0];
    iobj_0->_pobj1[13].JointInternal.JointToParentTransform[i][1] = dv[i][1];
    iobj_0->_pobj1[13].JointInternal.JointToParentTransform[i][2] = dv[i][2];
    iobj_0->_pobj1[13].JointInternal.JointToParentTransform[i][3] = dv[i][3];
  }
  for (i = 0; i < 4; i++) {
    iobj_0->_pobj1[13].JointInternal.ChildToJointTransform[i][0] = iv[i][0];
    iobj_0->_pobj1[13].JointInternal.ChildToJointTransform[i][1] = iv[i][1];
    iobj_0->_pobj1[13].JointInternal.ChildToJointTransform[i][2] = iv[i][2];
    iobj_0->_pobj1[13].JointInternal.ChildToJointTransform[i][3] = iv[i][3];
  }
  int msubspace_size[2];
  c_rigidBodyJoint_get_MotionSubs(&iobj_0->_pobj1[13].JointInternal,
                                  msubspace_data, msubspace_size);
  for (i = 0; i < 6; i++) {
    msubspace_data[i] = 0.0;
  }
  c_rigidBodyJoint_set_MotionSubs(&iobj_0->_pobj1[13].JointInternal,
                                  msubspace_data);
  iobj_0->_pobj1[13].JointInternal.InTree = true;
  iobj_0->_pobj1[13].JointInternal.PositionLimitsInternal[0][0] = 0.0;
  iobj_0->_pobj1[13].JointInternal.PositionLimitsInternal[1][0] = 0.0;
  iobj_0->_pobj1[13].JointInternal.JointAxisInternal[0] = 0.0;
  iobj_0->_pobj1[13].JointInternal.JointAxisInternal[1] = 0.0;
  iobj_0->_pobj1[13].JointInternal.JointAxisInternal[2] = 0.0;
  iobj_0->_pobj1[13].JointInternal.HomePositionInternal[0] = 0.0;
  iobj_0->_pobj1[13].CollisionsInternal =
      CollisionSet_CollisionSet(&iobj_0->_pobj0[19], 0.0);
  iobj_0->_pobj1[13].matlabCodegenIsDeleted = false;
  iobj_0->Bodies[6] = &iobj_0->_pobj1[13];
  iobj_0->Bodies[6]->Index = 7.0;
  iobj_0->Gravity[0] = 0.0;
  iobj_0->Gravity[1] = 0.0;
  iobj_0->Gravity[2] = 0.0;
  iobj_0->NumNonFixedBodies = 6.0;
  iobj_0->PositionNumber = 6.0;
  iobj_0->VelocityNumber = 6.0;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      iobj_0->PositionDoFMap[i][i1] = iv3[i][i1];
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      iobj_0->VelocityDoFMap[i][i1] = iv3[i][i1];
    }
  }
  h_RigidBody_RigidBody(SD, &iobj_0->Base, &iobj_0->_pobj0[20]);
  iobj_0->Base.Index = 0.0;
  iobj_0->matlabCodegenIsDeleted = false;
  (*iobj_1)->TreeInternal = iobj_0;
  (*iobj_1)->matlabCodegenIsDeleted = false;
}

/*
 * File trailer for importrobot.c
 *
 * [EOF]
 */
