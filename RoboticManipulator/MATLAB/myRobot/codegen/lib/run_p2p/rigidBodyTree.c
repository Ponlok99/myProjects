/*
 * File: rigidBodyTree.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "rigidBodyTree.h"
#include "RigidBodyTreeDynamics.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_73
#define typedef_cell_wrap_73
typedef struct {
  double f1[4][4];
} cell_wrap_73;
#endif /* typedef_cell_wrap_73 */

/* Function Definitions */
/*
 * Arguments    : rigidBodyTree *obj
 *                const double Q_data[]
 *                double Jac_data[]
 *                int Jac_size[2]
 * Return Type  : void
 */
void rigidBodyTree_geometricJacobian(rigidBodyTree *obj, const double Q_data[],
                                     double Jac_data[], int Jac_size[2])
{
  static const char b_cv[5] = {'t', 'o', 'o', 'l', '0'};
  static const char cv1[5] = {'f', 'i', 'x', 'e', 'd'};
  c_robotics_manip_internal_Rigid *body;
  cell_wrap_73 Ttree_data[7];
  d_robotics_manip_internal_Rigid *b_obj;
  double B_data[294];
  double X[6][6];
  double b_data[36];
  double T2[4][4];
  double T2inv[4][4];
  double Tdh[4][4];
  double R[3][3];
  double b_R[3][3];
  double d;
  double d1;
  double d2;
  double k;
  double n;
  double velnum;
  int b_i;
  int b_k;
  int c_i;
  int endeffectorIndex;
  int exitg1;
  int i;
  int loop_ub;
  int loop_ub_tmp;
  char obj_Vector[200];
  signed char chainmask[7];
  bool b_bool;
  b_obj = obj->TreeInternal;
  n = b_obj->NumBodies;
  i = (int)n;
  if ((int)n != 0) {
    for (endeffectorIndex = 0; endeffectorIndex < i; endeffectorIndex++) {
      for (b_k = 0; b_k < 4; b_k++) {
        Ttree_data[endeffectorIndex].f1[b_k][0] = iv[b_k][0];
        Ttree_data[endeffectorIndex].f1[b_k][1] = iv[b_k][1];
        Ttree_data[endeffectorIndex].f1[b_k][2] = iv[b_k][2];
        Ttree_data[endeffectorIndex].f1[b_k][3] = iv[b_k][3];
      }
    }
  }
  k = 1.0;
  for (b_i = 0; b_i < i; b_i++) {
    double b_Q_data[6];
    body = b_obj->Bodies[b_i];
    n = body->JointInternal.PositionNumber;
    d = k + n;
    if (k > d - 1.0) {
      b_k = 0;
      endeffectorIndex = 0;
    } else {
      b_k = (int)k - 1;
      endeffectorIndex = (int)(d - 1.0);
    }
    loop_ub = endeffectorIndex - b_k;
    for (endeffectorIndex = 0; endeffectorIndex < loop_ub; endeffectorIndex++) {
      b_Q_data[endeffectorIndex] = Q_data[b_k + endeffectorIndex];
    }
    c_rigidBodyJoint_transformBodyT(&body->JointInternal, b_Q_data, loop_ub,
                                    Ttree_data[b_i].f1);
    k = d;
    if (body->ParentIndex > 0.0) {
      for (b_k = 0; b_k < 4; b_k++) {
        T2[b_k][0] = Ttree_data[(int)body->ParentIndex - 1].f1[b_k][0];
        T2[b_k][1] = Ttree_data[(int)body->ParentIndex - 1].f1[b_k][1];
        T2[b_k][2] = Ttree_data[(int)body->ParentIndex - 1].f1[b_k][2];
        T2[b_k][3] = Ttree_data[(int)body->ParentIndex - 1].f1[b_k][3];
      }
      for (b_k = 0; b_k < 4; b_k++) {
        d = T2[0][b_k];
        d1 = T2[1][b_k];
        d2 = T2[2][b_k];
        n = T2[3][b_k];
        for (endeffectorIndex = 0; endeffectorIndex < 4; endeffectorIndex++) {
          Tdh[endeffectorIndex][b_k] =
              ((d * Ttree_data[b_i].f1[endeffectorIndex][0] +
                d1 * Ttree_data[b_i].f1[endeffectorIndex][1]) +
               d2 * Ttree_data[b_i].f1[endeffectorIndex][2]) +
              n * Ttree_data[b_i].f1[endeffectorIndex][3];
        }
      }
      for (b_k = 0; b_k < 4; b_k++) {
        Ttree_data[b_i].f1[b_k][0] = Tdh[b_k][0];
        Ttree_data[b_i].f1[b_k][1] = Tdh[b_k][1];
        Ttree_data[b_i].f1[b_k][2] = Tdh[b_k][2];
        Ttree_data[b_i].f1[b_k][3] = Tdh[b_k][3];
      }
    }
  }
  velnum = b_obj->VelocityNumber;
  loop_ub_tmp = (int)velnum;
  for (i = 0; i < loop_ub_tmp; i++) {
    for (b_k = 0; b_k < 6; b_k++) {
      Jac_data[b_k + 6 * i] = 0.0;
    }
  }
  for (i = 0; i < 7; i++) {
    chainmask[i] = 0;
  }
  n = b_obj->Base.NameInternal.Length;
  for (i = 0; i < 200; i++) {
    obj_Vector[i] = b_obj->Base.NameInternal.Vector[i];
  }
  if (n < 1.0) {
    i = 0;
  } else {
    i = (int)n;
  }
  b_bool = false;
  if (i == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (b_cv[loop_ub] != obj_Vector[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    for (i = 0; i < 4; i++) {
      T2inv[i][0] = 0.0;
      T2inv[i][1] = 0.0;
      T2inv[i][2] = 0.0;
      T2inv[i][3] = 0.0;
    }
    T2inv[0][0] = 1.0;
    T2inv[1][1] = 1.0;
    T2inv[2][2] = 1.0;
    T2inv[3][3] = 1.0;
    for (i = 0; i < 4; i++) {
      T2[i][0] = T2inv[i][0];
      T2[i][1] = T2inv[i][1];
      T2[i][2] = T2inv[i][2];
      T2[i][3] = T2inv[i][3];
    }
  } else {
    endeffectorIndex = -2;
    n = b_obj->Base.NameInternal.Length;
    for (i = 0; i < 200; i++) {
      obj_Vector[i] = b_obj->Base.NameInternal.Vector[i];
    }
    if (n < 1.0) {
      i = 0;
    } else {
      i = (int)n;
    }
    b_bool = false;
    if (i == 5) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 5) {
          if (obj_Vector[loop_ub] != b_cv[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      endeffectorIndex = -1;
    } else {
      bool exitg2;
      d = b_obj->NumBodies;
      b_i = 0;
      exitg2 = false;
      while ((!exitg2) && (b_i <= (int)d - 1)) {
        body = b_obj->Bodies[b_i];
        n = body->NameInternal.Length;
        for (i = 0; i < 200; i++) {
          obj_Vector[i] = body->NameInternal.Vector[i];
        }
        if (n < 1.0) {
          i = 0;
        } else {
          i = (int)n;
        }
        b_bool = false;
        if (i == 5) {
          loop_ub = 0;
          do {
            exitg1 = 0;
            if (loop_ub < 5) {
              if (obj_Vector[loop_ub] != b_cv[loop_ub]) {
                exitg1 = 1;
              } else {
                loop_ub++;
              }
            } else {
              b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }
        if (b_bool) {
          endeffectorIndex = b_i;
          exitg2 = true;
        } else {
          b_i++;
        }
      }
    }
    body = b_obj->Bodies[endeffectorIndex];
    for (i = 0; i < 4; i++) {
      T2[i][0] = Ttree_data[endeffectorIndex].f1[i][0];
      T2[i][1] = Ttree_data[endeffectorIndex].f1[i][1];
      T2[i][2] = Ttree_data[endeffectorIndex].f1[i][2];
      T2[i][3] = Ttree_data[endeffectorIndex].f1[i][3];
    }
    for (i = 0; i < 3; i++) {
      d = Ttree_data[endeffectorIndex].f1[0][i];
      R[i][0] = d;
      b_R[i][0] = -d;
      d = Ttree_data[endeffectorIndex].f1[1][i];
      R[i][1] = d;
      b_R[i][1] = -d;
      d = Ttree_data[endeffectorIndex].f1[2][i];
      R[i][2] = d;
      b_R[i][2] = -d;
    }
    for (i = 0; i < 3; i++) {
      T2inv[i][0] = R[i][0];
      T2inv[i][1] = R[i][1];
      T2inv[i][2] = R[i][2];
      T2inv[3][i] = (b_R[0][i] * Ttree_data[endeffectorIndex].f1[3][0] +
                     b_R[1][i] * Ttree_data[endeffectorIndex].f1[3][1]) +
                    b_R[2][i] * Ttree_data[endeffectorIndex].f1[3][2];
    }
    T2inv[0][3] = 0.0;
    T2inv[1][3] = 0.0;
    T2inv[2][3] = 0.0;
    T2inv[3][3] = 1.0;
    chainmask[endeffectorIndex] = 1;
    while (body->ParentIndex > 0.0) {
      body = b_obj->Bodies[(int)body->ParentIndex - 1];
      chainmask[(int)body->Index - 1] = 1;
    }
  }
  d = b_obj->NumBodies;
  i = (int)d;
  for (b_i = 0; b_i < i; b_i++) {
    body = b_obj->Bodies[b_i];
    n = body->JointInternal.TypeInternal.Length;
    for (b_k = 0; b_k < 200; b_k++) {
      obj_Vector[b_k] = body->JointInternal.TypeInternal.Vector[b_k];
    }
    if (n < 1.0) {
      b_k = 0;
    } else {
      b_k = (int)n;
    }
    b_bool = false;
    if (b_k == 5) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub < 5) {
          if (obj_Vector[loop_ub] != cv1[loop_ub]) {
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if ((!b_bool) && (chainmask[b_i] != 0)) {
      double JacSlice_data[36];
      double T1[4][4];
      double b_T2inv[4][4];
      double c_R[3];
      double idx_idx_1;
      for (b_k = 0; b_k < 4; b_k++) {
        T1[b_k][0] = Ttree_data[(int)body->Index - 1].f1[b_k][0];
        T1[b_k][1] = Ttree_data[(int)body->Index - 1].f1[b_k][1];
        T1[b_k][2] = Ttree_data[(int)body->Index - 1].f1[b_k][2];
        T1[b_k][3] = Ttree_data[(int)body->Index - 1].f1[b_k][3];
      }
      for (b_k = 0; b_k < 4; b_k++) {
        Tdh[b_k][0] = body->JointInternal.ChildToJointTransform[b_k][0];
        Tdh[b_k][1] = body->JointInternal.ChildToJointTransform[b_k][1];
        Tdh[b_k][2] = body->JointInternal.ChildToJointTransform[b_k][2];
        Tdh[b_k][3] = body->JointInternal.ChildToJointTransform[b_k][3];
      }
      for (b_k = 0; b_k < 3; b_k++) {
        d = Tdh[0][b_k];
        R[b_k][0] = d;
        b_R[b_k][0] = -d;
        d = Tdh[1][b_k];
        R[b_k][1] = d;
        b_R[b_k][1] = -d;
        d = Tdh[2][b_k];
        R[b_k][2] = d;
        b_R[b_k][2] = -d;
      }
      d = Tdh[3][0];
      d1 = Tdh[3][1];
      d2 = Tdh[3][2];
      for (b_k = 0; b_k < 3; b_k++) {
        c_R[b_k] = (b_R[0][b_k] * d + b_R[1][b_k] * d1) + b_R[2][b_k] * d2;
      }
      for (b_k = 0; b_k < 4; b_k++) {
        d = T2inv[0][b_k];
        d1 = T2inv[1][b_k];
        d2 = T2inv[2][b_k];
        n = T2inv[3][b_k];
        for (endeffectorIndex = 0; endeffectorIndex < 4; endeffectorIndex++) {
          b_T2inv[endeffectorIndex][b_k] =
              ((d * T1[endeffectorIndex][0] + d1 * T1[endeffectorIndex][1]) +
               d2 * T1[endeffectorIndex][2]) +
              n * T1[endeffectorIndex][3];
        }
      }
      for (b_k = 0; b_k < 3; b_k++) {
        Tdh[b_k][0] = R[b_k][0];
        Tdh[b_k][1] = R[b_k][1];
        Tdh[b_k][2] = R[b_k][2];
        Tdh[3][b_k] = c_R[b_k];
      }
      Tdh[0][3] = 0.0;
      Tdh[1][3] = 0.0;
      Tdh[2][3] = 0.0;
      Tdh[3][3] = 1.0;
      for (b_k = 0; b_k < 4; b_k++) {
        d = b_T2inv[0][b_k];
        d1 = b_T2inv[1][b_k];
        d2 = b_T2inv[2][b_k];
        n = b_T2inv[3][b_k];
        for (endeffectorIndex = 0; endeffectorIndex < 4; endeffectorIndex++) {
          T1[endeffectorIndex][b_k] =
              ((d * Tdh[endeffectorIndex][0] + d1 * Tdh[endeffectorIndex][1]) +
               d2 * Tdh[endeffectorIndex][2]) +
              n * Tdh[endeffectorIndex][3];
        }
      }
      k = b_obj->VelocityDoFMap[0][b_i];
      idx_idx_1 = b_obj->VelocityDoFMap[1][b_i];
      R[0][0] = 0.0;
      R[1][0] = -T1[3][2];
      R[2][0] = T1[3][1];
      R[0][1] = T1[3][2];
      R[1][1] = 0.0;
      R[2][1] = -T1[3][0];
      R[0][2] = -T1[3][1];
      R[1][2] = T1[3][0];
      R[2][2] = 0.0;
      for (b_k = 0; b_k < 3; b_k++) {
        d = R[0][b_k];
        d1 = R[1][b_k];
        d2 = R[2][b_k];
        for (endeffectorIndex = 0; endeffectorIndex < 3; endeffectorIndex++) {
          b_R[endeffectorIndex][b_k] =
              (d * T1[endeffectorIndex][0] + d1 * T1[endeffectorIndex][1]) +
              d2 * T1[endeffectorIndex][2];
          X[b_k][endeffectorIndex] = T1[b_k][endeffectorIndex];
          X[b_k + 3][endeffectorIndex] = 0.0;
        }
      }
      for (b_k = 0; b_k < 3; b_k++) {
        X[b_k][3] = b_R[b_k][0];
        X[b_k + 3][3] = T1[b_k][0];
        X[b_k][4] = b_R[b_k][1];
        X[b_k + 3][4] = T1[b_k][1];
        X[b_k][5] = b_R[b_k][2];
        X[b_k + 3][5] = T1[b_k][2];
      }
      int b_size[2];
      c_rigidBodyJoint_get_MotionSubs(&body->JointInternal, b_data, b_size);
      endeffectorIndex = b_size[1];
      for (loop_ub = 0; loop_ub < endeffectorIndex; loop_ub++) {
        for (c_i = 0; c_i < 6; c_i++) {
          n = 0.0;
          for (b_k = 0; b_k < 6; b_k++) {
            n += X[b_k][c_i] * b_data[b_k + 6 * loop_ub];
          }
          JacSlice_data[c_i + 6 * loop_ub] = n;
        }
      }
      if (k > idx_idx_1) {
        b_k = 0;
        endeffectorIndex = 0;
      } else {
        b_k = (int)k - 1;
        endeffectorIndex = (int)idx_idx_1;
      }
      loop_ub = endeffectorIndex - b_k;
      for (endeffectorIndex = 0; endeffectorIndex < loop_ub;
           endeffectorIndex++) {
        for (c_i = 0; c_i < 6; c_i++) {
          Jac_data[c_i + 6 * (b_k + endeffectorIndex)] =
              JacSlice_data[c_i + 6 * endeffectorIndex];
        }
      }
    }
  }
  for (i = 0; i < 3; i++) {
    d = T2[i][0];
    X[i][0] = d;
    X[i + 3][0] = 0.0;
    X[i][3] = 0.0;
    X[i + 3][3] = d;
    d = T2[i][1];
    X[i][1] = d;
    X[i + 3][1] = 0.0;
    X[i][4] = 0.0;
    X[i + 3][4] = d;
    d = T2[i][2];
    X[i][2] = d;
    X[i + 3][2] = 0.0;
    X[i][5] = 0.0;
    X[i + 3][5] = d;
  }
  for (i = 0; i < loop_ub_tmp; i++) {
    for (b_k = 0; b_k < 6; b_k++) {
      loop_ub = b_k + 6 * i;
      B_data[loop_ub] = Jac_data[loop_ub];
    }
  }
  Jac_size[0] = 6;
  Jac_size[1] = (int)velnum;
  for (loop_ub = 0; loop_ub < loop_ub_tmp; loop_ub++) {
    for (b_i = 0; b_i < 6; b_i++) {
      n = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        n += X[b_k][b_i] * B_data[b_k + 6 * loop_ub];
      }
      Jac_data[b_i + 6 * loop_ub] = n;
    }
  }
}

/*
 * Arguments    : rigidBodyTree *obj
 *                const double varargin_1_data[]
 *                const int varargin_1_size[2]
 *                const double varargin_2[6]
 *                const double varargin_3[6]
 *                double tau_data[]
 *                int tau_size[2]
 * Return Type  : void
 */
void rigidBodyTree_inverseDynamics(rigidBodyTree *obj,
                                   const double varargin_1_data[],
                                   const int varargin_1_size[2],
                                   const double varargin_2[6],
                                   const double varargin_3[6],
                                   double tau_data[], int tau_size[2])
{
  static const char b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  d_robotics_manip_internal_Rigid *b_obj;
  rigidBodyJoint *c_obj;
  double q_data[49];
  double tmp_data[42];
  double b_tau_data[6];
  double nb;
  double posnum;
  int b_i;
  int i;
  int i1;
  int i2;
  int loop_ub;
  b_obj = obj->TreeInternal;
  nb = b_obj->NumBodies;
  posnum = b_obj->PositionNumber;
  loop_ub = (int)posnum;
  if (loop_ub - 1 >= 0) {
    memset(&q_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  posnum = b_obj->NumBodies;
  i = (int)posnum;
  for (b_i = 0; b_i < i; b_i++) {
    double p_idx_0;
    double p_idx_1;
    p_idx_0 = b_obj->PositionDoFMap[0][b_i];
    p_idx_1 = b_obj->PositionDoFMap[1][b_i];
    if (p_idx_0 <= p_idx_1) {
      double qi_data[7];
      char obj_Vector[200];
      bool b_bool;
      c_obj = &b_obj->Bodies[b_i]->JointInternal;
      posnum = c_obj->TypeInternal.Length;
      for (i1 = 0; i1 < 200; i1++) {
        obj_Vector[i1] = c_obj->TypeInternal.Vector[i1];
      }
      if (posnum < 1.0) {
        i1 = 0;
      } else {
        i1 = (int)posnum;
      }
      b_bool = false;
      if (i1 == 5) {
        loop_ub = 0;
        int exitg1;
        do {
          exitg1 = 0;
          if (loop_ub < 5) {
            if (obj_Vector[loop_ub] != b_cv[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (!b_bool) {
        posnum = c_obj->PositionNumber;
        if (posnum < 1.0) {
          loop_ub = 0;
        } else {
          loop_ub = (int)posnum;
        }
        for (i1 = 0; i1 < loop_ub; i1++) {
          qi_data[i1] = c_obj->HomePositionInternal[i1];
        }
      } else {
        qi_data[0] = 0.0;
      }
      if (p_idx_0 > p_idx_1) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = (int)p_idx_0 - 1;
        i2 = (int)p_idx_1;
      }
      loop_ub = i2 - i1;
      for (i2 = 0; i2 < loop_ub; i2++) {
        q_data[i1 + i2] = qi_data[i2];
      }
    }
  }
  if (varargin_1_size[1] != 0) {
    loop_ub = varargin_1_size[1];
    memcpy(&q_data[0], &varargin_1_data[0],
           (unsigned int)loop_ub * sizeof(double));
  }
  loop_ub = (int)nb;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      tmp_data[i1 + 6 * i] = 0.0;
    }
  }
  c_RigidBodyTreeDynamics_inverse(obj->TreeInternal, q_data, varargin_2,
                                  varargin_3, tmp_data, b_tau_data);
  tau_size[0] = 1;
  tau_size[1] = 6;
  for (i = 0; i < 6; i++) {
    tau_data[i] = b_tau_data[i];
  }
}

/*
 * File trailer for rigidBodyTree.c
 *
 * [EOF]
 */
