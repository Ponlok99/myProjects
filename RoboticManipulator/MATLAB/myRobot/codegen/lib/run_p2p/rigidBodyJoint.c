/*
 * File: rigidBodyJoint.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "rigidBodyJoint.h"
#include "quat2tform.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_rtwutil.h"
#include "run_p2p_types.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void rigidBodyJoint_get_JointAxis(const rigidBodyJoint *obj,
                                         double ax[3]);

/* Function Definitions */
/*
 * Arguments    : const rigidBodyJoint *obj
 *                double ax[3]
 * Return Type  : void
 */
static void rigidBodyJoint_get_JointAxis(const rigidBodyJoint *obj,
                                         double ax[3])
{
  static const char b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int exitg1;
  int i;
  int kstr;
  bool b_bool;
  bool guard1;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int)obj->TypeInternal.Length;
  }
  b_bool = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (obj->TypeInternal.Vector[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    b_bool = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (obj->TypeInternal.Vector[kstr] != cv[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = rtNaN;
      ax[1] = rtNaN;
      ax[2] = rtNaN;
    }
  }
  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

/*
 * Arguments    : const rigidBodyJoint *obj
 *                double msubspace_data[]
 *                int msubspace_size[2]
 * Return Type  : void
 */
void c_rigidBodyJoint_get_MotionSubs(const rigidBodyJoint *obj,
                                     double msubspace_data[],
                                     int msubspace_size[2])
{
  static const char b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  int i;
  int i1;
  int kstr;
  bool b_bool;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int)obj->TypeInternal.Length;
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (obj->TypeInternal.Vector[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    double d;
    d = obj->VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      kstr = (int)d;
    }
    msubspace_size[0] = 6;
    msubspace_size[1] = kstr;
    for (i = 0; i < kstr; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        msubspace_data[i1 + 6 * i] = obj->MotionSubspaceInternal[i][i1];
      }
    }
  } else {
    msubspace_size[0] = 6;
    msubspace_size[1] = 1;
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0.0;
    }
  }
}

/*
 * Arguments    : rigidBodyJoint *obj
 *                const double msubspace_data[]
 * Return Type  : void
 */
void c_rigidBodyJoint_set_MotionSubs(rigidBodyJoint *obj,
                                     const double msubspace_data[])
{
  static const char b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  int i;
  int i1;
  int kstr;
  bool b_bool;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int)obj->TypeInternal.Length;
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (obj->TypeInternal.Vector[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    double d;
    d = obj->VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      kstr = (int)d;
    }
    for (i = 0; i < kstr; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        obj->MotionSubspaceInternal[i][i1] = msubspace_data[i1 + 6 * i];
      }
    }
  } else {
    for (i = 0; i < 6; i++) {
      obj->MotionSubspaceInternal[0][i] = 0.0;
    }
  }
}

/*
 * Arguments    : const rigidBodyJoint *obj
 *                const double q_data[]
 *                int q_size
 *                double T[4][4]
 * Return Type  : void
 */
void c_rigidBodyJoint_transformBodyT(const rigidBodyJoint *obj,
                                     const double q_data[], int q_size,
                                     double T[4][4])
{
  static const char b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char cv1[8] = {'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  double b[4][4];
  double b_I[4][4];
  double tempR[9];
  double b_tempR_tmp;
  double result_data_idx_2;
  double result_data_idx_3;
  double sth;
  double tempR_tmp;
  int exitg1;
  int i;
  int kstr;
  bool result;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int)obj->TypeInternal.Length;
  }
  result = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != obj->TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    result = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (cv[kstr] != obj->TypeInternal.Vector[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      kstr = 1;
    } else {
      result = false;
      if (i == 8) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 8) {
            if (cv1[kstr] != obj->TypeInternal.Vector[kstr]) {
              exitg1 = 1;
            } else {
              kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        kstr = 2;
      } else {
        kstr = -1;
      }
    }
  }
  switch (kstr) {
  case 0: {
    double R[3][3];
    double v[3];
    rigidBodyJoint_get_JointAxis(obj, v);
    sth = v[0];
    tempR_tmp = v[1];
    result_data_idx_2 = v[2];
    kstr = (q_size != 0);
    for (i = 0; i < kstr; i++) {
      result_data_idx_3 = q_data[0];
    }
    double c_tempR_tmp;
    double cth;
    double d_tempR_tmp;
    b_tempR_tmp =
        1.0 / sqrt((rt_powd_snf(sth, 2.0) + rt_powd_snf(tempR_tmp, 2.0)) +
                   rt_powd_snf(result_data_idx_2, 2.0));
    v[0] = sth * b_tempR_tmp;
    v[1] = tempR_tmp * b_tempR_tmp;
    v[2] = result_data_idx_2 * b_tempR_tmp;
    cth = cos(result_data_idx_3);
    sth = sin(result_data_idx_3);
    b_tempR_tmp = v[0] * v[0] * (1.0 - cth) + cth;
    tempR[0] = b_tempR_tmp;
    result_data_idx_3 = v[0] * v[1] * (1.0 - cth);
    tempR_tmp = v[2] * sth;
    b_tempR_tmp = result_data_idx_3 - tempR_tmp;
    tempR[1] = b_tempR_tmp;
    c_tempR_tmp = v[0] * v[2] * (1.0 - cth);
    d_tempR_tmp = v[1] * sth;
    b_tempR_tmp = c_tempR_tmp + d_tempR_tmp;
    tempR[2] = b_tempR_tmp;
    result_data_idx_3 += tempR_tmp;
    tempR[3] = result_data_idx_3;
    tempR_tmp = v[1] * v[1] * (1.0 - cth) + cth;
    tempR[4] = tempR_tmp;
    b_tempR_tmp = v[1] * v[2] * (1.0 - cth);
    tempR_tmp = v[0] * sth;
    result_data_idx_2 = b_tempR_tmp - tempR_tmp;
    tempR[5] = result_data_idx_2;
    c_tempR_tmp -= d_tempR_tmp;
    tempR[6] = c_tempR_tmp;
    d_tempR_tmp = b_tempR_tmp + tempR_tmp;
    tempR[7] = d_tempR_tmp;
    b_tempR_tmp = v[2] * v[2] * (1.0 - cth) + cth;
    tempR[8] = b_tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[0][kstr] = tempR[3 * kstr];
      R[1][kstr] = tempR[3 * kstr + 1];
      R[2][kstr] = tempR[3 * kstr + 2];
    }
    for (i = 0; i < 4; i++) {
      b[i][0] = 0.0;
      b[i][1] = 0.0;
      b[i][2] = 0.0;
      b[i][3] = 0.0;
    }
    for (i = 0; i < 3; i++) {
      b[i][0] = R[i][0];
      b[i][1] = R[i][1];
      b[i][2] = R[i][2];
    }
    b[3][3] = 1.0;
  } break;
  case 1: {
    double v[3];
    rigidBodyJoint_get_JointAxis(obj, v);
    memset(&tempR[0], 0, 9U * sizeof(double));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      b[i][0] = tempR[3 * i];
      b[i][1] = tempR[3 * i + 1];
      b[i][2] = tempR[3 * i + 2];
      b[3][i] = v[i] * q_data[0];
    }
    b[0][3] = 0.0;
    b[1][3] = 0.0;
    b[2][3] = 0.0;
    b[3][3] = 1.0;
  } break;
  case 2: {
    double dv[4][4];
    for (i = 0; i < 4; i++) {
      b_I[i][0] = 0.0;
      b_I[i][1] = 0.0;
      b_I[i][2] = 0.0;
      b_I[i][3] = 0.0;
    }
    double q[4];
    b_I[0][0] = 1.0;
    b_I[1][1] = 1.0;
    b_I[2][2] = 1.0;
    b_I[3][3] = 1.0;
    b_I[3][0] = q_data[4];
    b_I[3][1] = q_data[5];
    b_I[3][2] = q_data[6];
    q[0] = q_data[0];
    q[1] = q_data[1];
    q[2] = q_data[2];
    q[3] = q_data[3];
    quat2tform(q, dv);
    for (i = 0; i < 4; i++) {
      sth = b_I[0][i];
      tempR_tmp = b_I[1][i];
      result_data_idx_2 = b_I[2][i];
      b_tempR_tmp = b_I[3][i];
      for (kstr = 0; kstr < 4; kstr++) {
        b[kstr][i] = ((sth * dv[kstr][0] + tempR_tmp * dv[kstr][1]) +
                      result_data_idx_2 * dv[kstr][2]) +
                     b_tempR_tmp * dv[kstr][3];
      }
    }
  } break;
  default:
    for (i = 0; i < 4; i++) {
      b[i][0] = 0.0;
      b[i][1] = 0.0;
      b[i][2] = 0.0;
      b[i][3] = 0.0;
    }
    b[0][0] = 1.0;
    b[1][1] = 1.0;
    b[2][2] = 1.0;
    b[3][3] = 1.0;
    break;
  }
  for (i = 0; i < 4; i++) {
    sth = obj->JointToParentTransform[0][i];
    tempR_tmp = obj->JointToParentTransform[1][i];
    result_data_idx_2 = obj->JointToParentTransform[2][i];
    b_tempR_tmp = obj->JointToParentTransform[3][i];
    for (kstr = 0; kstr < 4; kstr++) {
      b_I[kstr][i] = ((sth * b[kstr][0] + tempR_tmp * b[kstr][1]) +
                      result_data_idx_2 * b[kstr][2]) +
                     b_tempR_tmp * b[kstr][3];
    }
    sth = b_I[0][i];
    tempR_tmp = b_I[1][i];
    result_data_idx_2 = b_I[2][i];
    b_tempR_tmp = b_I[3][i];
    for (kstr = 0; kstr < 4; kstr++) {
      T[kstr][i] = ((sth * obj->ChildToJointTransform[kstr][0] +
                     tempR_tmp * obj->ChildToJointTransform[kstr][1]) +
                    result_data_idx_2 * obj->ChildToJointTransform[kstr][2]) +
                   b_tempR_tmp * obj->ChildToJointTransform[kstr][3];
    }
  }
}

/*
 * Arguments    : const rigidBodyJoint *obj
 *                double T[4][4]
 * Return Type  : void
 */
void d_rigidBodyJoint_transformBodyT(const rigidBodyJoint *obj, double T[4][4])
{
  static const char b_cv[8] = {'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char cv1[8] = {'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  double b[4][4];
  double b_obj[4][4];
  double tempR[9];
  double axang_idx_1;
  double b_b;
  double b_tempR_tmp;
  double tempR_tmp;
  int exitg1;
  int i;
  int kstr;
  bool result;
  if (obj->TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = (int)obj->TypeInternal.Length;
  }
  result = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != obj->TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    result = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (cv[kstr] != obj->TypeInternal.Vector[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      kstr = 1;
    } else {
      result = false;
      if (i == 8) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 8) {
            if (cv1[kstr] != obj->TypeInternal.Vector[kstr]) {
              exitg1 = 1;
            } else {
              kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        kstr = 2;
      } else {
        kstr = -1;
      }
    }
  }
  switch (kstr) {
  case 0: {
    double R[3][3];
    double v[3];
    rigidBodyJoint_get_JointAxis(obj, v);
    tempR_tmp = v[0];
    axang_idx_1 = v[1];
    b_tempR_tmp = v[2];
    b_b = 1.0 / sqrt((rt_powd_snf(v[0], 2.0) + rt_powd_snf(v[1], 2.0)) +
                     rt_powd_snf(v[2], 2.0));
    v[0] = tempR_tmp * b_b;
    v[1] = axang_idx_1 * b_b;
    v[2] = b_tempR_tmp * b_b;
    tempR_tmp = v[0] * v[0] * 0.0 + 1.0;
    tempR[0] = tempR_tmp;
    axang_idx_1 = v[0] * v[1] * 0.0;
    b_tempR_tmp = axang_idx_1 - v[2] * 0.0;
    tempR[1] = b_tempR_tmp;
    b_b = v[0] * v[2] * 0.0;
    tempR_tmp = b_b + v[1] * 0.0;
    tempR[2] = tempR_tmp;
    axang_idx_1 += v[2] * 0.0;
    tempR[3] = axang_idx_1;
    tempR_tmp = v[1] * v[1] * 0.0 + 1.0;
    tempR[4] = tempR_tmp;
    tempR_tmp = v[1] * v[2] * 0.0;
    b_tempR_tmp = tempR_tmp - v[0] * 0.0;
    tempR[5] = b_tempR_tmp;
    b_b -= v[1] * 0.0;
    tempR[6] = b_b;
    tempR_tmp += v[0] * 0.0;
    tempR[7] = tempR_tmp;
    tempR_tmp = v[2] * v[2] * 0.0 + 1.0;
    tempR[8] = tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[0][kstr] = tempR[3 * kstr];
      R[1][kstr] = tempR[3 * kstr + 1];
      R[2][kstr] = tempR[3 * kstr + 2];
    }
    for (i = 0; i < 4; i++) {
      b[i][0] = 0.0;
      b[i][1] = 0.0;
      b[i][2] = 0.0;
      b[i][3] = 0.0;
    }
    for (i = 0; i < 3; i++) {
      b[i][0] = R[i][0];
      b[i][1] = R[i][1];
      b[i][2] = R[i][2];
    }
    b[3][3] = 1.0;
  } break;
  case 1: {
    double v[3];
    rigidBodyJoint_get_JointAxis(obj, v);
    memset(&tempR[0], 0, 9U * sizeof(double));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      b[i][0] = tempR[3 * i];
      b[i][1] = tempR[3 * i + 1];
      b[i][2] = tempR[3 * i + 2];
      b[3][i] = v[i] * 0.0;
    }
    b[0][3] = 0.0;
    b[1][3] = 0.0;
    b[2][3] = 0.0;
    b[3][3] = 1.0;
  } break;
  case 2:
    /* A check that is always false is detected at compile-time. Eliminating
     * code that follows. */
    break;
  default:
    for (i = 0; i < 4; i++) {
      b[i][0] = 0.0;
      b[i][1] = 0.0;
      b[i][2] = 0.0;
      b[i][3] = 0.0;
    }
    b[0][0] = 1.0;
    b[1][1] = 1.0;
    b[2][2] = 1.0;
    b[3][3] = 1.0;
    break;
  }
  for (i = 0; i < 4; i++) {
    tempR_tmp = obj->JointToParentTransform[0][i];
    axang_idx_1 = obj->JointToParentTransform[1][i];
    b_tempR_tmp = obj->JointToParentTransform[2][i];
    b_b = obj->JointToParentTransform[3][i];
    for (kstr = 0; kstr < 4; kstr++) {
      b_obj[kstr][i] = ((tempR_tmp * b[kstr][0] + axang_idx_1 * b[kstr][1]) +
                        b_tempR_tmp * b[kstr][2]) +
                       b_b * b[kstr][3];
    }
    tempR_tmp = b_obj[0][i];
    axang_idx_1 = b_obj[1][i];
    b_tempR_tmp = b_obj[2][i];
    b_b = b_obj[3][i];
    for (kstr = 0; kstr < 4; kstr++) {
      T[kstr][i] = ((tempR_tmp * obj->ChildToJointTransform[kstr][0] +
                     axang_idx_1 * obj->ChildToJointTransform[kstr][1]) +
                    b_tempR_tmp * obj->ChildToJointTransform[kstr][2]) +
                   b_b * obj->ChildToJointTransform[kstr][3];
    }
  }
}

/*
 * File trailer for rigidBodyJoint.c
 *
 * [EOF]
 */
