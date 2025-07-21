/*
 * File: RigidBodyTreeDynamics.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "RigidBodyTreeDynamics.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "run_p2p_types.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_76
#define typedef_cell_wrap_76
typedef struct {
  double f1[6][6];
} cell_wrap_76;
#endif /* typedef_cell_wrap_76 */

/* Function Definitions */
/*
 * Arguments    : d_robotics_manip_internal_Rigid *robot
 *                const double q_data[]
 *                const double qdot_data[]
 *                const double qddot_data[]
 *                const double fext_data[]
 *                double tau_data[]
 * Return Type  : int
 */
int c_RigidBodyTreeDynamics_inverse(d_robotics_manip_internal_Rigid *robot,
                                    const double q_data[],
                                    const double qdot_data[],
                                    const double qddot_data[],
                                    const double fext_data[], double tau_data[])
{
  static const char b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  c_robotics_manip_internal_Rigid *obj;
  cell_wrap_76 X_data[7];
  cell_wrap_76 Xtree_data[7];
  double aB_data[42];
  double f_data[42];
  double vB_data[42];
  double vJ_data[42];
  double S_data[36];
  double XDHOffset[6][6];
  double d_R[6][6];
  double y_data[36];
  double TDHOffset[4][4];
  double R[3][3];
  double b_R[3][3];
  double a0[6];
  double qddoti_data[6];
  double y[6];
  double c_R[3];
  double a_idx_0;
  double a_idx_1;
  double b_idx_0;
  double b_idx_1;
  double nb;
  int S_size[2];
  int b_i;
  int b_loop_ub;
  int d_i;
  int e_i;
  int i;
  int i1;
  int k;
  int loop_ub;
  int tau_size;
  int vJ_data_tmp;
  a0[0] = 0.0;
  a0[1] = 0.0;
  a0[2] = 0.0;
  a0[3] = -robot->Gravity[0];
  a0[4] = -robot->Gravity[1];
  a0[5] = -robot->Gravity[2];
  nb = robot->NumBodies;
  loop_ub = (int)nb;
  for (i = 0; i < loop_ub; i++) {
    for (k = 0; k < 6; k++) {
      vJ_data_tmp = k + 6 * i;
      vJ_data[vJ_data_tmp] = 0.0;
      vB_data[vJ_data_tmp] = 0.0;
      aB_data[vJ_data_tmp] = 0.0;
    }
  }
  tau_size = 6;
  for (i = 0; i < 6; i++) {
    tau_data[i] = 0.0;
  }
  for (k = 0; k < loop_ub; k++) {
    memset(&XDHOffset[0][0], 0, 36U * sizeof(double));
    for (vJ_data_tmp = 0; vJ_data_tmp < 6; vJ_data_tmp++) {
      XDHOffset[vJ_data_tmp][vJ_data_tmp] = 1.0;
    }
    memcpy(&Xtree_data[k].f1[0][0], &XDHOffset[0][0], 36U * sizeof(double));
    memcpy(&X_data[k].f1[0][0], &XDHOffset[0][0], 36U * sizeof(double));
  }
  for (b_i = 0; b_i < loop_ub; b_i++) {
    double T[4][4];
    obj = robot->Bodies[b_i];
    c_rigidBodyJoint_get_MotionSubs(&obj->JointInternal, S_data, S_size);
    a_idx_0 = robot->PositionDoFMap[0][b_i];
    a_idx_1 = robot->PositionDoFMap[1][b_i];
    b_idx_0 = robot->VelocityDoFMap[0][b_i];
    b_idx_1 = robot->VelocityDoFMap[1][b_i];
    memset(&XDHOffset[0][0], 0, 36U * sizeof(double));
    for (k = 0; k < 6; k++) {
      XDHOffset[k][k] = 1.0;
    }
    if (a_idx_1 < a_idx_0) {
      obj = robot->Bodies[b_i];
      d_rigidBodyJoint_transformBodyT(&obj->JointInternal, T);
      qddoti_data[0] = 0.0;
      for (i = 0; i < 6; i++) {
        vJ_data[i + 6 * b_i] = 0.0;
      }
    } else {
      double b_q_data[49];
      if (a_idx_0 > a_idx_1) {
        i = 0;
        k = 0;
      } else {
        i = (int)a_idx_0 - 1;
        k = (int)a_idx_1;
      }
      if (b_idx_0 > b_idx_1) {
        i1 = 0;
        vJ_data_tmp = 0;
        d_i = 0;
      } else {
        i1 = (int)b_idx_0 - 1;
        vJ_data_tmp = (int)b_idx_0 - 1;
        d_i = (int)b_idx_1;
      }
      b_loop_ub = d_i - vJ_data_tmp;
      for (d_i = 0; d_i < b_loop_ub; d_i++) {
        qddoti_data[d_i] = qddot_data[vJ_data_tmp + d_i];
      }
      obj = robot->Bodies[b_i];
      b_loop_ub = k - i;
      for (k = 0; k < b_loop_ub; k++) {
        b_q_data[k] = q_data[i + k];
      }
      c_rigidBodyJoint_transformBodyT(&obj->JointInternal, b_q_data, b_loop_ub,
                                      T);
      obj = robot->Bodies[b_i];
      for (i = 0; i < 4; i++) {
        TDHOffset[i][0] = obj->JointInternal.ChildToJointTransform[i][0];
        TDHOffset[i][1] = obj->JointInternal.ChildToJointTransform[i][1];
        TDHOffset[i][2] = obj->JointInternal.ChildToJointTransform[i][2];
        TDHOffset[i][3] = obj->JointInternal.ChildToJointTransform[i][3];
      }
      for (i = 0; i < 3; i++) {
        a_idx_0 = TDHOffset[0][i];
        R[i][0] = a_idx_0;
        b_R[i][0] = -a_idx_0;
        a_idx_0 = TDHOffset[1][i];
        R[i][1] = a_idx_0;
        b_R[i][1] = -a_idx_0;
        a_idx_0 = TDHOffset[2][i];
        R[i][2] = a_idx_0;
        b_R[i][2] = -a_idx_0;
      }
      for (i = 0; i < 3; i++) {
        a_idx_0 = b_R[0][i] * TDHOffset[3][0];
        TDHOffset[i][0] = R[i][0];
        a_idx_0 += b_R[1][i] * TDHOffset[3][1];
        TDHOffset[i][1] = R[i][1];
        a_idx_0 += b_R[2][i] * TDHOffset[3][2];
        TDHOffset[i][2] = R[i][2];
        c_R[i] = a_idx_0;
      }
      TDHOffset[3][0] = c_R[0];
      TDHOffset[3][1] = c_R[1];
      TDHOffset[3][2] = c_R[2];
      TDHOffset[0][3] = 0.0;
      TDHOffset[1][3] = 0.0;
      TDHOffset[2][3] = 0.0;
      TDHOffset[3][3] = 1.0;
      R[0][0] = 0.0;
      R[1][0] = -c_R[2];
      R[2][0] = c_R[1];
      R[0][1] = c_R[2];
      R[1][1] = 0.0;
      R[2][1] = -c_R[0];
      R[0][2] = -c_R[1];
      R[1][2] = c_R[0];
      R[2][2] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[0][i];
        a_idx_1 = R[1][i];
        b_idx_0 = R[2][i];
        for (k = 0; k < 3; k++) {
          b_R[k][i] = (a_idx_0 * TDHOffset[k][0] + a_idx_1 * TDHOffset[k][1]) +
                      b_idx_0 * TDHOffset[k][2];
          XDHOffset[i][k] = TDHOffset[i][k];
          XDHOffset[i + 3][k] = 0.0;
        }
      }
      for (i = 0; i < 3; i++) {
        XDHOffset[i][3] = b_R[i][0];
        XDHOffset[i + 3][3] = TDHOffset[i][0];
        XDHOffset[i][4] = b_R[i][1];
        XDHOffset[i + 3][4] = TDHOffset[i][1];
        XDHOffset[i][5] = b_R[i][2];
        XDHOffset[i + 3][5] = TDHOffset[i][2];
      }
      vJ_data_tmp = S_size[1];
      for (b_loop_ub = 0; b_loop_ub < vJ_data_tmp; b_loop_ub++) {
        for (d_i = 0; d_i < 6; d_i++) {
          a_idx_0 = 0.0;
          for (k = 0; k < 6; k++) {
            a_idx_0 += XDHOffset[k][d_i] * S_data[k + 6 * b_loop_ub];
          }
          y_data[d_i + 6 * b_loop_ub] = a_idx_0;
        }
      }
      for (d_i = 0; d_i < 6; d_i++) {
        y[d_i] = 0.0;
      }
      for (k = 0; k < vJ_data_tmp; k++) {
        a_idx_0 = qdot_data[i1 + k];
        for (d_i = 0; d_i < 6; d_i++) {
          y[d_i] += y_data[d_i + 6 * k] * a_idx_0;
        }
      }
      for (i = 0; i < 6; i++) {
        vJ_data[i + 6 * b_i] = y[i];
      }
    }
    for (i = 0; i < 3; i++) {
      a_idx_0 = T[0][i];
      R[i][0] = a_idx_0;
      b_R[i][0] = -a_idx_0;
      a_idx_0 = T[1][i];
      R[i][1] = a_idx_0;
      b_R[i][1] = -a_idx_0;
      a_idx_0 = T[2][i];
      R[i][2] = a_idx_0;
      b_R[i][2] = -a_idx_0;
    }
    a_idx_0 = T[3][0];
    a_idx_1 = T[3][1];
    b_idx_0 = T[3][2];
    for (i = 0; i < 3; i++) {
      TDHOffset[i][0] = R[i][0];
      TDHOffset[i][1] = R[i][1];
      TDHOffset[i][2] = R[i][2];
      TDHOffset[3][i] =
          (b_R[0][i] * a_idx_0 + b_R[1][i] * a_idx_1) + b_R[2][i] * b_idx_0;
    }
    TDHOffset[0][3] = 0.0;
    TDHOffset[1][3] = 0.0;
    TDHOffset[2][3] = 0.0;
    TDHOffset[3][3] = 1.0;
    R[0][0] = 0.0;
    R[1][0] = -TDHOffset[3][2];
    R[2][0] = TDHOffset[3][1];
    R[0][1] = TDHOffset[3][2];
    R[1][1] = 0.0;
    R[2][1] = -TDHOffset[3][0];
    R[0][2] = -TDHOffset[3][1];
    R[1][2] = TDHOffset[3][0];
    R[2][2] = 0.0;
    for (i = 0; i < 3; i++) {
      a_idx_0 = R[0][i];
      a_idx_1 = R[1][i];
      b_idx_0 = R[2][i];
      for (k = 0; k < 3; k++) {
        b_R[k][i] = (a_idx_0 * TDHOffset[k][0] + a_idx_1 * TDHOffset[k][1]) +
                    b_idx_0 * TDHOffset[k][2];
        X_data[b_i].f1[i][k] = TDHOffset[i][k];
        X_data[b_i].f1[i + 3][k] = 0.0;
      }
    }
    for (i = 0; i < 3; i++) {
      X_data[b_i].f1[i][3] = b_R[i][0];
      X_data[b_i].f1[i + 3][3] = TDHOffset[i][0];
      X_data[b_i].f1[i][4] = b_R[i][1];
      X_data[b_i].f1[i + 3][4] = TDHOffset[i][1];
      X_data[b_i].f1[i][5] = b_R[i][2];
      X_data[b_i].f1[i + 3][5] = TDHOffset[i][2];
    }
    b_idx_1 = robot->Bodies[b_i]->ParentIndex;
    if (b_idx_1 > 0.0) {
      double e_R[6];
      for (i = 0; i < 6; i++) {
        a_idx_0 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += X_data[b_i].f1[k][i] * vB_data[k + 6 * ((int)b_idx_1 - 1)];
        }
        y[i] = vJ_data[i + 6 * b_i] + a_idx_0;
      }
      for (i = 0; i < 6; i++) {
        vB_data[i + 6 * b_i] = y[i];
      }
      vJ_data_tmp = S_size[1];
      for (b_loop_ub = 0; b_loop_ub < vJ_data_tmp; b_loop_ub++) {
        for (d_i = 0; d_i < 6; d_i++) {
          a_idx_0 = 0.0;
          for (k = 0; k < 6; k++) {
            a_idx_0 += XDHOffset[k][d_i] * S_data[k + 6 * b_loop_ub];
          }
          y_data[d_i + 6 * b_loop_ub] = a_idx_0;
        }
      }
      for (d_i = 0; d_i < 6; d_i++) {
        y[d_i] = 0.0;
      }
      for (k = 0; k < vJ_data_tmp; k++) {
        for (d_i = 0; d_i < 6; d_i++) {
          y[d_i] += y_data[d_i + 6 * k] * qddoti_data[k];
        }
      }
      R[0][0] = 0.0;
      a_idx_0 = vB_data[6 * b_i + 2];
      R[1][0] = -a_idx_0;
      a_idx_1 = vB_data[6 * b_i + 1];
      R[2][0] = a_idx_1;
      R[0][1] = a_idx_0;
      R[1][1] = 0.0;
      a_idx_0 = vB_data[6 * b_i];
      R[2][1] = -a_idx_0;
      R[0][2] = -a_idx_1;
      R[1][2] = a_idx_0;
      R[2][2] = 0.0;
      d_R[0][3] = 0.0;
      a_idx_0 = vB_data[6 * b_i + 5];
      d_R[1][3] = -a_idx_0;
      a_idx_1 = vB_data[6 * b_i + 4];
      d_R[2][3] = a_idx_1;
      d_R[0][4] = a_idx_0;
      d_R[1][4] = 0.0;
      a_idx_0 = vB_data[6 * b_i + 3];
      d_R[2][4] = -a_idx_0;
      d_R[0][5] = -a_idx_1;
      d_R[1][5] = a_idx_0;
      d_R[2][5] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[i][0];
        d_R[i][0] = a_idx_0;
        d_R[i + 3][0] = 0.0;
        d_R[i + 3][3] = a_idx_0;
        a_idx_0 = R[i][1];
        d_R[i][1] = a_idx_0;
        d_R[i + 3][1] = 0.0;
        d_R[i + 3][4] = a_idx_0;
        a_idx_0 = R[i][2];
        d_R[i][2] = a_idx_0;
        d_R[i + 3][2] = 0.0;
        d_R[i + 3][5] = a_idx_0;
      }
      for (i = 0; i < 6; i++) {
        a_idx_0 = 0.0;
        a_idx_1 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += X_data[b_i].f1[k][i] * aB_data[k + 6 * ((int)b_idx_1 - 1)];
          a_idx_1 += d_R[k][i] * vJ_data[k + 6 * b_i];
        }
        e_R[i] = a_idx_1;
        qddoti_data[i] = a_idx_0 + y[i];
      }
      for (i = 0; i < 6; i++) {
        aB_data[i + 6 * b_i] = qddoti_data[i] + e_R[i];
      }
      R[0][0] = 0.0;
      R[1][0] = -T[3][2];
      R[2][0] = T[3][1];
      R[0][1] = T[3][2];
      R[1][1] = 0.0;
      R[2][1] = -T[3][0];
      R[0][2] = -T[3][1];
      R[1][2] = T[3][0];
      R[2][2] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[0][i];
        a_idx_1 = R[1][i];
        b_idx_0 = R[2][i];
        for (k = 0; k < 3; k++) {
          b_R[k][i] =
              (a_idx_0 * T[k][0] + a_idx_1 * T[k][1]) + b_idx_0 * T[k][2];
          XDHOffset[i][k] = T[i][k];
          XDHOffset[i + 3][k] = 0.0;
        }
      }
      for (i = 0; i < 3; i++) {
        XDHOffset[i][3] = b_R[i][0];
        XDHOffset[i + 3][3] = T[i][0];
        XDHOffset[i][4] = b_R[i][1];
        XDHOffset[i + 3][4] = T[i][1];
        XDHOffset[i][5] = b_R[i][2];
        XDHOffset[i + 3][5] = T[i][2];
      }
      for (i = 0; i < 6; i++) {
        for (k = 0; k < 6; k++) {
          a_idx_0 = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            a_idx_0 +=
                Xtree_data[(int)b_idx_1 - 1].f1[i1][i] * XDHOffset[k][i1];
          }
          d_R[k][i] = a_idx_0;
        }
      }
      memcpy(&Xtree_data[b_i].f1[0][0], &d_R[0][0], 36U * sizeof(double));
    } else {
      for (i = 0; i < 6; i++) {
        vJ_data_tmp = i + 6 * b_i;
        vB_data[vJ_data_tmp] = vJ_data[vJ_data_tmp];
      }
      vJ_data_tmp = S_size[1];
      for (b_loop_ub = 0; b_loop_ub < vJ_data_tmp; b_loop_ub++) {
        for (d_i = 0; d_i < 6; d_i++) {
          a_idx_0 = 0.0;
          for (k = 0; k < 6; k++) {
            a_idx_0 += XDHOffset[k][d_i] * S_data[k + 6 * b_loop_ub];
          }
          y_data[d_i + 6 * b_loop_ub] = a_idx_0;
        }
      }
      for (d_i = 0; d_i < 6; d_i++) {
        y[d_i] = 0.0;
      }
      for (k = 0; k < vJ_data_tmp; k++) {
        for (d_i = 0; d_i < 6; d_i++) {
          y[d_i] += y_data[d_i + 6 * k] * qddoti_data[k];
        }
      }
      for (i = 0; i < 6; i++) {
        a_idx_0 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += X_data[b_i].f1[k][i] * a0[k];
        }
        aB_data[i + 6 * b_i] = a_idx_0 + y[i];
      }
      R[0][0] = 0.0;
      R[1][0] = -T[3][2];
      R[2][0] = T[3][1];
      R[0][1] = T[3][2];
      R[1][1] = 0.0;
      R[2][1] = -T[3][0];
      R[0][2] = -T[3][1];
      R[1][2] = T[3][0];
      R[2][2] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[0][i];
        a_idx_1 = R[1][i];
        b_idx_0 = R[2][i];
        for (k = 0; k < 3; k++) {
          b_R[k][i] =
              (a_idx_0 * T[k][0] + a_idx_1 * T[k][1]) + b_idx_0 * T[k][2];
          Xtree_data[b_i].f1[i][k] = T[i][k];
          Xtree_data[b_i].f1[i + 3][k] = 0.0;
        }
      }
      for (i = 0; i < 3; i++) {
        Xtree_data[b_i].f1[i][3] = b_R[i][0];
        Xtree_data[b_i].f1[i + 3][3] = T[i][0];
        Xtree_data[b_i].f1[i][4] = b_R[i][1];
        Xtree_data[b_i].f1[i + 3][4] = T[i][1];
        Xtree_data[b_i].f1[i][5] = b_R[i][2];
        Xtree_data[b_i].f1[i + 3][5] = T[i][2];
      }
    }
    for (i = 0; i < 6; i++) {
      for (k = 0; k < 6; k++) {
        XDHOffset[i][k] = robot->Bodies[b_i]->SpatialInertia[i][k];
      }
    }
    R[0][0] = 0.0;
    a_idx_0 = vB_data[6 * b_i + 2];
    R[1][0] = -a_idx_0;
    a_idx_1 = vB_data[6 * b_i + 1];
    R[2][0] = a_idx_1;
    R[0][1] = a_idx_0;
    R[1][1] = 0.0;
    a_idx_0 = vB_data[6 * b_i];
    R[2][1] = -a_idx_0;
    R[0][2] = -a_idx_1;
    R[1][2] = a_idx_0;
    R[2][2] = 0.0;
    d_R[3][0] = 0.0;
    a_idx_0 = vB_data[6 * b_i + 5];
    d_R[4][0] = -a_idx_0;
    a_idx_1 = vB_data[6 * b_i + 4];
    d_R[5][0] = a_idx_1;
    d_R[3][1] = a_idx_0;
    d_R[4][1] = 0.0;
    a_idx_0 = vB_data[6 * b_i + 3];
    d_R[5][1] = -a_idx_0;
    d_R[3][2] = -a_idx_1;
    d_R[4][2] = a_idx_0;
    d_R[5][2] = 0.0;
    for (i = 0; i < 3; i++) {
      a_idx_0 = R[i][0];
      d_R[i][0] = a_idx_0;
      d_R[i][3] = 0.0;
      d_R[i + 3][3] = a_idx_0;
      a_idx_0 = R[i][1];
      d_R[i][1] = a_idx_0;
      d_R[i][4] = 0.0;
      d_R[i + 3][4] = a_idx_0;
      a_idx_0 = R[i][2];
      d_R[i][2] = a_idx_0;
      d_R[i][5] = 0.0;
      d_R[i + 3][5] = a_idx_0;
    }
    for (i = 0; i < 6; i++) {
      a_idx_0 = 0.0;
      a_idx_1 = 0.0;
      for (k = 0; k < 6; k++) {
        b_idx_0 = XDHOffset[k][i];
        i1 = k + 6 * b_i;
        a_idx_0 += b_idx_0 * vB_data[i1];
        a_idx_1 += b_idx_0 * aB_data[i1];
      }
      qddoti_data[i] = a_idx_1;
      y[i] = a_idx_0;
    }
    for (i = 0; i < 6; i++) {
      a_idx_0 = 0.0;
      a_idx_1 = 0.0;
      for (k = 0; k < 6; k++) {
        a_idx_1 += d_R[k][i] * y[k];
        a_idx_0 += Xtree_data[b_i].f1[i][k] * fext_data[k + 6 * b_i];
      }
      f_data[i + 6 * b_i] = (qddoti_data[i] + a_idx_1) - a_idx_0;
    }
  }
  i = (int)-((-1.0 - nb) + 1.0);
  for (b_i = 0; b_i < i; b_i++) {
    double c_i;
    char obj_Vector[200];
    bool b_bool;
    c_i = nb - (double)b_i;
    obj = robot->Bodies[(int)c_i - 1];
    a_idx_0 = obj->JointInternal.TypeInternal.Length;
    for (k = 0; k < 200; k++) {
      obj_Vector[k] = obj->JointInternal.TypeInternal.Vector[k];
    }
    if (a_idx_0 < 1.0) {
      k = 0;
    } else {
      k = (int)a_idx_0;
    }
    b_bool = false;
    if (k == 5) {
      vJ_data_tmp = 0;
      int exitg1;
      do {
        exitg1 = 0;
        if (vJ_data_tmp < 5) {
          if (obj_Vector[vJ_data_tmp] != b_cv[vJ_data_tmp]) {
            exitg1 = 1;
          } else {
            vJ_data_tmp++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (!b_bool) {
      obj = robot->Bodies[(int)c_i - 1];
      for (k = 0; k < 4; k++) {
        TDHOffset[k][0] = obj->JointInternal.ChildToJointTransform[k][0];
        TDHOffset[k][1] = obj->JointInternal.ChildToJointTransform[k][1];
        TDHOffset[k][2] = obj->JointInternal.ChildToJointTransform[k][2];
        TDHOffset[k][3] = obj->JointInternal.ChildToJointTransform[k][3];
      }
      for (k = 0; k < 3; k++) {
        a_idx_0 = TDHOffset[0][k];
        R[k][0] = a_idx_0;
        b_R[k][0] = -a_idx_0;
        a_idx_0 = TDHOffset[1][k];
        R[k][1] = a_idx_0;
        b_R[k][1] = -a_idx_0;
        a_idx_0 = TDHOffset[2][k];
        R[k][2] = a_idx_0;
        b_R[k][2] = -a_idx_0;
      }
      for (k = 0; k < 3; k++) {
        a_idx_0 = b_R[0][k] * TDHOffset[3][0];
        TDHOffset[k][0] = R[k][0];
        a_idx_0 += b_R[1][k] * TDHOffset[3][1];
        TDHOffset[k][1] = R[k][1];
        a_idx_0 += b_R[2][k] * TDHOffset[3][2];
        TDHOffset[k][2] = R[k][2];
        c_R[k] = a_idx_0;
      }
      TDHOffset[3][0] = c_R[0];
      TDHOffset[3][1] = c_R[1];
      TDHOffset[3][2] = c_R[2];
      TDHOffset[0][3] = 0.0;
      TDHOffset[1][3] = 0.0;
      TDHOffset[2][3] = 0.0;
      TDHOffset[3][3] = 1.0;
      obj = robot->Bodies[(int)c_i - 1];
      c_rigidBodyJoint_get_MotionSubs(&obj->JointInternal, y_data, S_size);
      R[0][0] = 0.0;
      R[1][0] = -c_R[2];
      R[2][0] = c_R[1];
      R[0][1] = c_R[2];
      R[1][1] = 0.0;
      R[2][1] = -c_R[0];
      R[0][2] = -c_R[1];
      R[1][2] = c_R[0];
      R[2][2] = 0.0;
      for (k = 0; k < 3; k++) {
        a_idx_0 = R[0][k];
        a_idx_1 = R[1][k];
        b_idx_0 = R[2][k];
        for (i1 = 0; i1 < 3; i1++) {
          b_R[i1][k] =
              (a_idx_0 * TDHOffset[i1][0] + a_idx_1 * TDHOffset[i1][1]) +
              b_idx_0 * TDHOffset[i1][2];
          XDHOffset[k][i1] = TDHOffset[k][i1];
          XDHOffset[k + 3][i1] = 0.0;
        }
      }
      for (k = 0; k < 3; k++) {
        XDHOffset[k][3] = b_R[k][0];
        XDHOffset[k + 3][3] = TDHOffset[k][0];
        XDHOffset[k][4] = b_R[k][1];
        XDHOffset[k + 3][4] = TDHOffset[k][1];
        XDHOffset[k][5] = b_R[k][2];
        XDHOffset[k + 3][5] = TDHOffset[k][2];
      }
      vJ_data_tmp = S_size[1];
      for (b_loop_ub = 0; b_loop_ub < vJ_data_tmp; b_loop_ub++) {
        for (d_i = 0; d_i < 6; d_i++) {
          a_idx_0 = 0.0;
          for (k = 0; k < 6; k++) {
            a_idx_0 += XDHOffset[k][d_i] * y_data[k + 6 * b_loop_ub];
          }
          S_data[d_i + 6 * b_loop_ub] = a_idx_0;
        }
      }
      if (S_size[1] - 1 >= 0) {
        e_i = (int)c_i;
      }
      for (d_i = 0; d_i < vJ_data_tmp; d_i++) {
        a_idx_0 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += S_data[k + 6 * d_i] * f_data[k + 6 * (e_i - 1)];
        }
        qddoti_data[d_i] = a_idx_0;
      }
      b_idx_0 = robot->VelocityDoFMap[0][(int)c_i - 1];
      b_idx_1 = robot->VelocityDoFMap[1][(int)c_i - 1];
      if (b_idx_0 > b_idx_1) {
        k = 0;
        i1 = 0;
      } else {
        k = (int)b_idx_0 - 1;
        i1 = (int)b_idx_1;
      }
      loop_ub = i1 - k;
      for (i1 = 0; i1 < loop_ub; i1++) {
        tau_data[k + i1] = qddoti_data[i1];
      }
    }
    b_idx_1 = robot->Bodies[(int)c_i - 1]->ParentIndex;
    if (b_idx_1 > 0.0) {
      for (k = 0; k < 6; k++) {
        a_idx_0 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          a_idx_0 +=
              X_data[(int)c_i - 1].f1[k][i1] * f_data[i1 + 6 * ((int)c_i - 1)];
        }
        y[k] = f_data[k + 6 * ((int)b_idx_1 - 1)] + a_idx_0;
      }
      for (k = 0; k < 6; k++) {
        f_data[k + 6 * ((int)b_idx_1 - 1)] = y[k];
      }
    }
  }
  return tau_size;
}

/*
 * File trailer for RigidBodyTreeDynamics.c
 *
 * [EOF]
 */
