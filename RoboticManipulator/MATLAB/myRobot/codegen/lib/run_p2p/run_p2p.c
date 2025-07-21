/*
 * File: run_p2p.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "run_p2p.h"
#include "RigidBodyTreeUtils.h"
#include "all.h"
#include "any.h"
#include "cubicpolytraj.h"
#include "eul2quat.h"
#include "eye.h"
#include "handle.h"
#include "ikine_myRobot.h"
#include "importrobot.h"
#include "minjerkpolytraj.h"
#include "minsnappolytraj.h"
#include "mldivide.h"
#include "quat2tform.h"
#include "quinticpolytraj.h"
#include "repmat.h"
#include "rigidBodyTree.h"
#include "rotm2eul.h"
#include "rottraj.h"
#include "round.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "sort.h"
#include "sum.h"
#include "trapveltraj.h"
#include "trvec2tform.h"
#include "unique.h"
#include "wrapTo2Pi.h"
#include "wrapToPi.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Load robot URDF
 *
 * Arguments    : run_p2pStackData *SD
 *                double wpts[2][3]
 *                double orientations[2][3]
 *                p2pMode runP2pMode
 *                trajMode runTrajMode
 *                double waypointVels[2][3]
 *                double waypointAccels[2][3]
 *                double waypointJerks[2][3]
 *                double pose[3][101]
 *                double qInterp[101][6]
 *                double qdInterp[101][6]
 *                double jointTorq[101][6]
 * Return Type  : void
 */
void run_p2p(run_p2pStackData *SD, double wpts[2][3], double orientations[2][3],
             p2pMode runP2pMode, trajMode runTrajMode,
             double waypointVels[2][3], double waypointAccels[2][3],
             double waypointJerks[2][3], double pose[3][101],
             double qInterp[101][6], double qdInterp[101][6],
             double jointTorq[101][6])
{
  static const double dhParams[4][6] = {{0.047, 0.11, 0.026, 0.0, 0.0, 0.0},
                                        {-1.5707963267949, 0.0,
                                         -1.5707963267949, 1.5707963267949,
                                         -1.5707963267949, -1.5707963267949},
                                        {0.135, 0.0, 0.0, 0.1175, 0.0, 0.06665},
                                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  static const double b[4][4] = {{6.12323399573677E-17, 1.0, 0.0, 0.0},
                                 {-1.0, 6.12323399573677E-17, 0.0, 0.0},
                                 {0.0, 0.0, 1.0, 0.0},
                                 {0.0, 0.0, 0.0, 1.0}};
  static const double b_b[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                   {0.0, -1.0, 1.22464679914735E-16, 0.0},
                                   {0.0, -1.22464679914735E-16, -1.0, 0.0},
                                   {0.0, 0.06665, -4.08113545815855E-18, 1.0}};
  static const double dv2[3][3] = {{1.0, 0.0, 0.0},
                                   {0.0, -3.4914813388431334E-15, 1.0},
                                   {0.0, -1.0, -3.4914813388431334E-15}};
  static const double jointLimits[2][3] = {{-2.87979, -1.91986, -1.91986},
                                           {2.87979, 1.91986, 1.22173}};
  static const double jointLim[2][2] = {{-2.79253, -8.5521163267949},
                                        {2.79253, 5.4105236732051}};
  static const signed char c_b[3][3] = {{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
  static const bool bv[2] = {false, true};
  c_robotics_manip_internal_Rigid *obj;
  d_robotics_manip_internal_Rigid lobj_1;
  emxArray_real_T *result;
  rigidBodyTree robot;
  rigidBodyTree *r;
  double qddInterp[101][6];
  double R[101][4];
  double alpha[101][3];
  double b_s[101][3];
  double omega[101][3];
  double p[101][3];
  double pd[101][3];
  double pdd[101][3];
  double jacobian_data[294];
  double q0_data[192];
  double s[101];
  double sd[101];
  double sdd[101];
  double varargin_1_data[96];
  double d_tmp_data[49];
  double q123Opts_data[48];
  double a[4][4];
  double f_tmp_data[6];
  double *result_data;
  int iidx_data[32];
  int indx_data[32];
  int b_i;
  int c_i;
  int i;
  int i1;
  int j;
  int jtIdx;
  int limIdx;
  signed char c_tmp_data[32];
  signed char e_tmp_data[2];
  signed char g_tmp_data[2];
  bool b_tmp_data[32];
  (void)runP2pMode;
  c_emxInitStruct_robotics_manip_(&lobj_1);
  for (i = 0; i < 22; i++) {
    lobj_1._pobj0[i].matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 14; i++) {
    lobj_1._pobj1[i].matlabCodegenIsDeleted = true;
  }
  lobj_1.Base.matlabCodegenIsDeleted = true;
  lobj_1.matlabCodegenIsDeleted = true;
  robot.matlabCodegenIsDeleted = true;
  r = &robot;
  importrobot(SD, &lobj_1, &r);
  robot.TreeInternal->Gravity[0] = 0.0;
  robot.TreeInternal->Gravity[1] = 0.0;
  robot.TreeInternal->Gravity[2] = -9.81;
  /*  Define trajectory parameters */
  /*  Time vector */
  /*  Parse optional inputs */
  /*     %% */
  /*  Trajectory following loop */
  /*     %% Initialize arrays */
  memset(&qInterp[0][0], 0, 606U * sizeof(double));
  /*  Get the initial and final rotations and times for the segment */
  switch (runTrajMode) {
  case minjerkpolytraj:
    b_minjerkpolytraj(wpts, waypointVels, waypointAccels, waypointJerks, p, pd,
                      pdd);
    break;
  case minsnappolytraj:
    b_minsnappolytraj(wpts, waypointVels, waypointAccels, waypointJerks, p, pd,
                      pdd);
    break;
  case quinticpolytraj:
    b_quinticpolytraj(wpts, waypointVels, waypointAccels, p, pd, pdd);
    break;
  case cubicpolytraj:
    b_cubicpolytraj(wpts, waypointVels, p, pd, pdd);
    break;
  default:
    b_trapveltraj(wpts, p, pd, pdd);
    break;
  }
  /*  Find the quaternions from trajectory generation */
  c_minjerkpolytraj(s, sd, sdd);
  /*  [s, sd, sdd] = quinticpolytraj([0 1], [0 1], tvec); */
  for (i = 0; i < 101; i++) {
    b_s[i][0] = s[i];
    b_s[i][1] = sd[i];
    b_s[i][2] = sdd[i];
  }
  double dv[4];
  double dv1[4];
  eul2quat(&(&orientations[0][0])[0], dv);
  eul2quat(&(&orientations[0][0])[3], dv1);
  rottraj(dv, dv1, b_s, R, omega, alpha);
  emxInit_real_T(&result, 2);
  for (b_i = 0; b_i < 101; b_i++) {
    double a_data[192];
    double allSolnOpts_data[192];
    double q456Opts_data[96];
    double dist_data[32];
    double eePose[4][4];
    double jt4ZeroPose[4][4];
    double tfInterp[4][4];
    double b_omega[6];
    double d;
    double d1;
    double diffRotation;
    double wrappedTotalRotation;
    int a_size[2];
    int q123Opts_size[2];
    int b_q456Opts_data_tmp;
    int q456Opts_data_tmp;
    int result_tmp;
    int trueCount;
    signed char i2;
    signed char input_sizes_idx_1;
    signed char sizes_idx_1;
    bool tmp_data[192];
    bool empty_non_axis_sizes;
    trvec2tform(&p[b_i][0], jt4ZeroPose);
    quat2tform(&R[b_i][0], eePose);
    for (i = 0; i < 4; i++) {
      d = jt4ZeroPose[0][i];
      diffRotation = jt4ZeroPose[1][i];
      wrappedTotalRotation = jt4ZeroPose[2][i];
      d1 = jt4ZeroPose[3][i];
      for (i1 = 0; i1 < 4; i1++) {
        tfInterp[i1][i] = ((d * eePose[i1][0] + diffRotation * eePose[i1][1]) +
                           wrappedTotalRotation * eePose[i1][2]) +
                          d1 * eePose[i1][3];
      }
    }
    pose[0][b_i] = tfInterp[3][0] / tfInterp[3][3];
    pose[1][b_i] = tfInterp[3][1] / tfInterp[3][3];
    pose[2][b_i] = tfInterp[3][2] / tfInterp[3][3];
    /* ikine_myRobot Function for generating closed-form inverse kinematics
     * solutions to the DH robot given by the parameters specified below */
    /*    $Revision: $ $Date: $ */
    /*  */
    /*    Generated on 04-Jul-2025 14:57:48 */
    /*  Compute the shifted joint limits, which are the limits during solution,
     * where theta offsets are not yet in play */
    /*  Convert the end effector pose in the global frame to the end effector
     * described by the DH parameters relative to the DH-described origin */
    /*  Parse optional inputs */
    /*  If joint limits are not enforced, set the shifted joint limits to have
     * infinite range */
    /*  Map the desired end effector pose to the pose of the central
     * intersecting joint. */
    /*  Solve for the position of the first three joints from the pose of joint
     * 5 */
    for (i = 0; i < 4; i++) {
      signed char i3;
      i2 = iv[0][i];
      input_sizes_idx_1 = iv[1][i];
      sizes_idx_1 = iv[2][i];
      i3 = iv[3][i];
      for (i1 = 0; i1 < 4; i1++) {
        a[i1][i] = (((double)i2 * tfInterp[i1][0] +
                     (double)input_sizes_idx_1 * tfInterp[i1][1]) +
                    (double)sizes_idx_1 * tfInterp[i1][2]) +
                   (double)i3 * tfInterp[i1][3];
      }
      d = a[0][i];
      diffRotation = a[1][i];
      wrappedTotalRotation = a[2][i];
      d1 = a[3][i];
      for (i1 = 0; i1 < 4; i1++) {
        eePose[i1][i] = ((d * b[i1][0] + diffRotation * b[i1][1]) +
                         wrappedTotalRotation * b[i1][2]) +
                        d1 * b[i1][3];
      }
      d = eePose[0][i];
      diffRotation = eePose[1][i];
      wrappedTotalRotation = eePose[2][i];
      d1 = eePose[3][i];
      for (i1 = 0; i1 < 4; i1++) {
        a[i1][i] = ((d * b_b[i1][0] + diffRotation * b_b[i1][1]) +
                    wrappedTotalRotation * b_b[i1][2]) +
                   d1 * b_b[i1][3];
      }
    }
    solveFirstThreeDHJoints(&a[3][0], q123Opts_data, q123Opts_size);
    /*  Solve for the positions of the intersecting axes */
    /*  For each position solution, this configuration of the last three axes
     * produces at least two possible orientation solutions */
    result_tmp = q123Opts_size[0] << 1;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < result_tmp; i1++) {
        q456Opts_data[i1 + result_tmp * i] = 0.0;
      }
    }
    /*  The next step seeks to find the orientation, which is entirely governed
     * by the last three joints. This means that rotation from the fourth joint
     * to the end effector can be mapped to three rotations in-place about the
     * fifth joint. Since the rotations are in-place, they can be defined
     * relative to the fourth joint axes, assuming a fixed pose rotation at the
     * end to align with the end effector. The fixed rotation is found using the
     * DH parameters, and corresponds to the rotation of the end effector
     * relative to the fourth joint when the last three joints are all zero. */
    i = q123Opts_size[0];
    if (i - 1 >= 0) {
    }
    for (jtIdx = 0; jtIdx < i; jtIdx++) {
      double b_jt4ZeroPose[3][3];
      double jt4ToEERot[3][3];
      double orientationSolns[3][2];
      double eulAltUnwrapped[3];
      double eulAngles[3];
      bool exitg1;
      /*  Get the position of the fourth joint at its zero position when the
       * first three joints are positioned for IK */
      /* getJoint4PoseFromDH Get the pose of the fourth joint when the first
       * three joints set to q123 and joint4 angle is zero */
      /*  Initialize output */
      eye(jt4ZeroPose);
      for (c_i = 0; c_i < 3; c_i++) {
        double TFixed_tmp;
        double b_TFixed_tmp;
        d = q123Opts_data[jtIdx + q123Opts_size[0] * c_i];
        diffRotation = sin(d);
        wrappedTotalRotation = cos(d);
        d = dhParams[1][c_i];
        TFixed_tmp = sin(d);
        b_TFixed_tmp = cos(d);
        a[0][0] = wrappedTotalRotation;
        a[1][0] = -diffRotation;
        a[2][0] = 0.0;
        a[3][0] = 0.0;
        a[0][1] = diffRotation;
        a[1][1] = wrappedTotalRotation;
        a[2][1] = 0.0;
        a[3][1] = 0.0;
        a[0][2] = 0.0;
        a[0][3] = 0.0;
        a[1][2] = 0.0;
        a[1][3] = 0.0;
        a[2][2] = 1.0;
        a[2][3] = 0.0;
        a[3][2] = 0.0;
        a[3][3] = 1.0;
        for (i1 = 0; i1 < 4; i1++) {
          d = jt4ZeroPose[0][i1];
          diffRotation = jt4ZeroPose[1][i1];
          wrappedTotalRotation = jt4ZeroPose[2][i1];
          d1 = jt4ZeroPose[3][i1];
          for (limIdx = 0; limIdx < 4; limIdx++) {
            tfInterp[limIdx][i1] =
                ((d * a[limIdx][0] + diffRotation * a[limIdx][1]) +
                 wrappedTotalRotation * a[limIdx][2]) +
                d1 * a[limIdx][3];
          }
        }
        a[0][0] = 1.0;
        a[1][0] = 0.0;
        a[2][0] = 0.0;
        a[3][0] = dhParams[0][c_i];
        a[0][1] = 0.0;
        a[1][1] = b_TFixed_tmp;
        a[2][1] = -TFixed_tmp;
        a[3][1] = 0.0;
        a[0][2] = 0.0;
        a[1][2] = TFixed_tmp;
        a[2][2] = b_TFixed_tmp;
        a[3][2] = dhParams[2][c_i];
        a[0][3] = 0.0;
        a[1][3] = 0.0;
        a[2][3] = 0.0;
        a[3][3] = 1.0;
        for (i1 = 0; i1 < 4; i1++) {
          d = tfInterp[0][i1];
          diffRotation = tfInterp[1][i1];
          wrappedTotalRotation = tfInterp[2][i1];
          d1 = tfInterp[3][i1];
          for (limIdx = 0; limIdx < 4; limIdx++) {
            jt4ZeroPose[limIdx][i1] =
                ((d * a[limIdx][0] + diffRotation * a[limIdx][1]) +
                 wrappedTotalRotation * a[limIdx][2]) +
                d1 * a[limIdx][3];
          }
        }
      }
      /*  Compute the rotation matrix needed to get to the end */
      /*  The orientation of the end effector in the world frame can be written:
       */
      /*     eeRot = jt4ZeroRot*(Rotation about axes 4-6)*eeFixedRotation */
      /*  Then the goal is to solve for the rotation about the axes and relate
       * them to he known form from the DH parameters, if a valid solution
       * exists: */
      /*     (Rotation about axes 4-6) = jt4ZeroRot'*eeRot*eeFixedRotation' */
      for (i1 = 0; i1 < 3; i1++) {
        d = jt4ZeroPose[i1][0];
        diffRotation = jt4ZeroPose[i1][1];
        wrappedTotalRotation = jt4ZeroPose[i1][2];
        for (limIdx = 0; limIdx < 3; limIdx++) {
          b_jt4ZeroPose[limIdx][i1] =
              (d * eePose[limIdx][0] + diffRotation * eePose[limIdx][1]) +
              wrappedTotalRotation * eePose[limIdx][2];
        }
        d = b_jt4ZeroPose[0][i1];
        diffRotation = b_jt4ZeroPose[1][i1];
        wrappedTotalRotation = b_jt4ZeroPose[2][i1];
        for (limIdx = 0; limIdx < 3; limIdx++) {
          jt4ToEERot[limIdx][i1] =
              (d * dv2[limIdx][0] + diffRotation * dv2[limIdx][1]) +
              wrappedTotalRotation * dv2[limIdx][2];
        }
      }
      /*  This orientation produces at least two configurations for every
       * solution, when joint limits allow */
      /* convertRotationToZYZAxesAngles Convert desired orientation to rotation
       * about Z-Y-Z */
      /*    This function is used to three angles of rotation corresponding to
       */
      /*    consecutive joint angles whose joint axes intersect at a single,
       * common */
      /*    point. This function addresses the case where the first joint
       * rotation */
      /*    is about Z, and the subsequent angles, in order and defined relative
       * to */
      /*    the first joint frame, are about Y and then Z. The function accepts
       * the */
      /*    rotation of the last joint relative to the origin of the first one,
       * as */
      /*    well as the sign of each axes. The second output indicates joints
       * that */
      /*    are in gimbal lock, where 1 indicates that they are, and zero
       * indicates */
      /*    that they are not. When joints are in gimbal lock, the affected
       * joint */
      /*    axes align and an infinite combination of joint angles are possible
       * (as */
      /*    long as the total rotation still matches the target). The default */
      /*    assumption is that rotation is divided over the joint along the */
      /*    affected axis. */
      /*    Copyright 2020 The MathWorks, Inc. */
      rotm2eul(jt4ToEERot, eulAngles);
      /*  The jointsInGimalLock variable indicates redundant joints, i.e. joints
       */
      /*  that complement each other and can have an infinite pair of values in
       * the */
      /*  directJointAngleMaps output. Initialize this value to zeros (no joints
       * in */
      /*  gimbal lock). This is a flag that is consistent across rotation
       * functions */
      /*  and may be used by the caller function. */
      /*  When the middle angle is zero, the first and last joints are co-axial,
       */
      /*  meaning there are an infinite set of solutions. Use a helper function
       * to */
      /*  distribute the values consistently given knowledge of the joint
       * limits. */
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (fabs(eulAngles[1]) < 1.0E-6) {
        double jointAngles[2];
        bool isRevJointFullRange[2];
        bool guard1;
        jt4ToEERot[2][0] = 0.0;
        jt4ToEERot[0][2] = 0.0;
        jt4ToEERot[2][1] = 0.0;
        jt4ToEERot[1][2] = 0.0;
        jt4ToEERot[2][2] = 1.0;
        b_rotm2eul(jt4ToEERot, eulAngles);
        wrappedTotalRotation = eulAngles[0] + eulAngles[2];
        /* distributeRotationOverJoints Distribute a rotation over several
         * in-line revolute joints */
        /*    When revolute joints are co-axial, the total rotation can be
         * distributed */
        /*    over the joints in a number of ways. This function assigns a
         * default */
        /*    behavior that respects the joint limits. For the case where no
         * joint */
        /*    limits are required, they should be provided as infinite, i.e
         * [-inf */
        /*    inf] for each joint. The default behaviors are as follows: */
        /*  */
        /*       - If any joint limits have a range of at minimum 2*pi, all
         * total */
        /*         rotation amounts are achievable and the rotation is
         * distributed */
        /*         evenly over the joints with infinite range, assuming the
         * other */
        /*         joints are centered within their range. */
        /*  */
        /*       - If all joints have finite ranges with total range less than
         * 2*pi, */
        /*         some total rotation amounts may not be feasible and the
         * rotation */
        /*         is distributed as much as possible on the distal joints
         * (unused */
        /*         more proximal joints are centered). If the solution is
         * infeasible, */
        /*         a NaN-vector is returned. */
        /*  */
        /*    The goal of these distributions is to favor solutions that are */
        /*    efficient. This function accepts the total rotation (in radians)
         * to be */
        /*    divided over N joints, the signs of those N joints (whether
         * rotation is */
        /*    positive or negative about the axes), and the joint limits, given
         * as an */
        /*    Nx2 matrix. */
        /*  */
        /*    If joint limits are ignored, they can be provided as infinite; the
         */
        /*    behavior is equivalent. This function returns an N-element row
         * vector. */
        /*    Copyright 2020 The MathWorks, Inc. */
        /*  Get the total number of joints from the joint limit input */
        /*  Initialize the output */
        /*  Remap the joint limits to fit the assumption that all axes are
         * positive. */
        /*  Since the joint limits can contain infinite elements, it is
         * necessary to */
        /*  use element-wise multiplication, as matrix multiplication can result
         * in */
        /*  NaNs when it causes sums of infinities. */
        /*  Re-order the joint limits to ensure the lower limit always comes
         * first */
        /*  (in case the of a sign flip in the previous line) */
        /*  Determine the total ranges of each joint. Since all joints are
         * revolute, */
        /*  a range of 2*pi or greater is equivalent to an infinite range as the
         * IK */
        /*  problem does not distinguish between periodic equivalents. Note that
         * a */
        /*  downstream helper in the IK solution, applyJointLimits, includes a
         * check */
        /*  that wraps periodic equivalents back to their values given the joint
         */
        /*  limits. */
        for (limIdx = 0; limIdx < 2; limIdx++) {
          jointAngles[limIdx] = 0.0;
          empty_non_axis_sizes = bv[limIdx];
          isRevJointFullRange[limIdx] = empty_non_axis_sizes;
          /*  Use a tolerance check on the equality. Since
           * isEqualWithinTolerance */
          /*  returns a scalar, it is necessary to do this inside a for-loop */
          if (empty_non_axis_sizes ||
              (fabs((8.37758 * (double)limIdx + 5.58506) - 6.2831853071795862) <
               1.0E-6)) {
            isRevJointFullRange[limIdx] = true;
          } else {
            /* isEqualWithinTolerance Check if two matrices are equal within a
             * set tolerance */
            /*    This is a convenience function designed for inputs with up to
             * two */
            /*    dimensions. If the input has 3+ dimensions, a non-scalar
             * output will be */
            /*    returned. */
            isRevJointFullRange[limIdx] = false;
          }
        }
        /*  There are two primary cases: when some joints have full range, any
         */
        /*  solution is feasible and the variable values are distributed over
         * these */
        /*  joints. When all of the joints have range of less than 2*pi, the
         * problem */
        /*  is more complex, as some solutions may not be feasible. */
        guard1 = false;
        if (any(isRevJointFullRange)) {
          bool d_b;
          /*  If any of the joint have infinite ranges, use that to distribute
           * the */
          /*  total rotation. First, place the joints with finite ranges in the
           */
          /*  middle of their respective ranges, then distribute the remaining
           */
          /*  joint rotation evenly over the joints with at least 2*pi range. */
          trueCount = 0;
          empty_non_axis_sizes = !isRevJointFullRange[0];
          if (empty_non_axis_sizes) {
            trueCount = 1;
          }
          d_b = !isRevJointFullRange[1];
          if (d_b) {
            trueCount++;
          }
          limIdx = 0;
          if (empty_non_axis_sizes) {
            e_tmp_data[0] = 0;
            limIdx = 1;
          }
          if (d_b) {
            e_tmp_data[limIdx] = 1;
          }
          for (c_i = 0; c_i < trueCount; c_i++) {
            i2 = e_tmp_data[c_i];
            jointAngles[i2] = (jointLim[0][i2] + jointLim[1][i2]) / 2.0;
          }
          /*  Compute the remaining rotation and wrap it to the interval [-pi,
           * pi], */
          /*  then distribute over the joints with complete range */
          diffRotation =
              wrappedTotalRotation - (jointAngles[0] + jointAngles[1]);
          if (fabs(diffRotation) > 3.1415926535897931) {
            diffRotation = wrapTo2Pi(diffRotation + 3.1415926535897931) -
                           3.1415926535897931;
          }
          trueCount = 0;
          if (isRevJointFullRange[0]) {
            trueCount = 1;
          }
          if (isRevJointFullRange[1]) {
            trueCount++;
          }
          limIdx = 0;
          if (isRevJointFullRange[0]) {
            g_tmp_data[0] = 0;
            limIdx = 1;
          }
          if (isRevJointFullRange[1]) {
            g_tmp_data[limIdx] = 1;
          }
          for (j = 0; j < trueCount; j++) {
            jointAngles[g_tmp_data[j]] = diffRotation / (double)trueCount;
          }
          guard1 = true;
        } else {
          /*  Use an algorithm that favors loading distal joints, which are */
          /*  typically easier to change: first set all the joints to their */
          /*  mid-range values. Then iterate over the joints from first to last,
           */
          /*  moving each joint up or down based on the difference in the
           * current */
          /*  total from the desired total, until the desired total is reached.
           */
          /*  This is essentially a cascaded bang-bang controller. */
          /*  Initialize the joint angles to their mid-range values */
          sum(*(double(*)[2][2]) & jointLim[0][0], jointAngles);
          jointAngles[0] /= 2.0;
          jointAngles[1] /= 2.0;
          /*  Iterate over the joints, using a feedback law to move them closer
           */
          /*  to the desired total */
          if (fabs(wrappedTotalRotation) > 3.1415926535897931) {
            wrappedTotalRotation =
                wrapTo2Pi(wrappedTotalRotation + 3.1415926535897931) -
                3.1415926535897931;
          }
          diffRotation =
              wrappedTotalRotation - (jointAngles[0] + jointAngles[1]);
          if (fabs(diffRotation) > 3.1415926535897931) {
            diffRotation = wrapTo2Pi(diffRotation + 3.1415926535897931) -
                           3.1415926535897931;
          }
          if (rtIsNaN(diffRotation)) {
            d = rtNaN;
          } else if (diffRotation < 0.0) {
            d = -1.0;
          } else {
            d = (diffRotation > 0.0);
          }
          jointAngles[0] += d * fmin(fabs(diffRotation), 2.79253);
          diffRotation =
              wrappedTotalRotation - (jointAngles[0] + jointAngles[1]);
          if (fabs(diffRotation) > 3.1415926535897931) {
            diffRotation = wrapTo2Pi(diffRotation + 3.1415926535897931) -
                           3.1415926535897931;
          }
          if (rtIsNaN(diffRotation)) {
            d = rtNaN;
          } else if (diffRotation < 0.0) {
            d = -1.0;
          } else {
            d = (diffRotation > 0.0);
          }
          jointAngles[1] += d * fmin(fabs(diffRotation), 6.98132);
          /*  Check if the sum of the joint angles reaches the desired total. If
           */
          /*  not, the solution is infeasible and a vector of NaNs is returned.
           */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          d = jointAngles[0] + jointAngles[1];
          if (fabs(d) > 3.1415926535897931) {
            d = wrapTo2Pi(d + 3.1415926535897931) - 3.1415926535897931;
          }
          if (!(fabs(d - wrappedTotalRotation) < 1.0E-6)) {
            eulAngles[0] = rtNaN;
            eulAngles[2] = rtNaN;
          } else {
            guard1 = true;
          }
        }
        if (guard1) {
          /*  Factor back in the axes signs. Since all valid joint angles are
           * finite, */
          /*  matrix multiplication is the most efficient approach. */
          eulAngles[0] = jointAngles[0] + jointAngles[1] * 0.0;
          eulAngles[2] = jointAngles[0] * 0.0 + jointAngles[1];
        }
        /*  In this case the alternate Euler angles aren't required, as they
         * will */
        /*  also result in a set of co-axial joints. However, to ensure codegen
         */
        /*  compatibility, the size must stay the same Therefore, return a set
         * of */
        /*  NaN angles (so the second solution may be thrown out). Note that the
         */
        /*  axes sign is handled inside the distributeRotationOverJoints helper
         */
        /*  function. */
        orientationSolns[0][0] = eulAngles[0];
        orientationSolns[0][1] = rtNaN;
        orientationSolns[1][0] = eulAngles[1];
        orientationSolns[1][1] = rtNaN;
        orientationSolns[2][0] = eulAngles[2];
        orientationSolns[2][1] = rtNaN;
      } else {
        double b_eulAngles[3][2];
        /*  For finite solutions when the middle angle is non-zero, there are
         * two possible solutions to this problem */
        /*  that can be derived from the first solution set */
        eulAltUnwrapped[0] = eulAngles[0] + 3.1415926535897931;
        eulAltUnwrapped[2] = eulAngles[2] + 3.1415926535897931;
        eulAltUnwrapped[1] =
            (-eulAngles[1] + 3.1415926535897931) - 3.1415926535897931;
        /*  Output the angles given the axes signs */
        wrapToPi(eulAltUnwrapped);
        b_eulAngles[0][0] = eulAngles[0];
        b_eulAngles[0][1] = eulAltUnwrapped[0];
        b_eulAngles[1][0] = eulAngles[1];
        b_eulAngles[1][1] = eulAltUnwrapped[1];
        b_eulAngles[2][0] = eulAngles[2];
        b_eulAngles[2][1] = eulAltUnwrapped[2];
        for (i1 = 0; i1 < 2; i1++) {
          d = b_eulAngles[0][i1];
          diffRotation = b_eulAngles[1][i1];
          wrappedTotalRotation = b_eulAngles[2][i1];
          for (limIdx = 0; limIdx < 3; limIdx++) {
            orientationSolns[limIdx][i1] =
                (d * (double)c_b[limIdx][0] +
                 diffRotation * (double)c_b[limIdx][1]) +
                wrappedTotalRotation * (double)c_b[limIdx][2];
          }
        }
      }
      limIdx = jtIdx + q123Opts_size[0];
      /*  Offset theta to reflect the source robot configuration */
      q456Opts_data[limIdx] = orientationSolns[0][1];
      q456Opts_data[jtIdx] = orientationSolns[0][0];
      j = limIdx + result_tmp;
      q456Opts_data[j] = orientationSolns[1][1];
      q123Opts_data[limIdx] -= -1.5707963267949;
      b_q456Opts_data_tmp = jtIdx + result_tmp;
      q456Opts_data[b_q456Opts_data_tmp] = orientationSolns[1][0];
      trueCount = limIdx + result_tmp * 2;
      q456Opts_data[trueCount] = orientationSolns[2][1];
      q456Opts_data_tmp = jtIdx + result_tmp * 2;
      q456Opts_data[q456Opts_data_tmp] =
          orientationSolns[2][0] - -1.5707963267949;
      /*  Remove solutions that violate joint limits */
      /* applyJointLimits Convert solutions with invalid joint limits to NaNs */
      /*    Given an N-element configuration, an Nx2 set of lower and upper
       * joint */
      /*    limits, and an N-element vector indicating the joint type (revolute
       * or */
      /*    prismatic), this function checks whether the configuration is within
       */
      /*    the joint limits. If not, the configuration is converted to NaNs. */
      /*    Copyright 2020-2021 The MathWorks, Inc. */
      /*  Initialize output */
      eulAngles[0] = q123Opts_data[jtIdx];
      eulAngles[1] = q123Opts_data[limIdx];
      q456Opts_data[trueCount] -= -1.5707963267949;
      eulAngles[2] = q123Opts_data[jtIdx + q123Opts_size[0] * 2];
      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 3)) {
        d = q123Opts_data[jtIdx + q123Opts_size[0] * c_i];
        if ((jointLimits[0][c_i] > d) || (jointLimits[1][c_i] < d)) {
          /*  Compute the offset from the lower joint limit and compare that to
           */
          /*  the total range */
          diffRotation = wrapTo2Pi(d - jointLimits[0][c_i]);
          /*  If the wrapped value is 2*pi, make sure it is instead registered
           */
          /*  as zero to ensure this doesn't fall outside the range */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          if (fabs(diffRotation - 6.2831853071795862) < 1.0E-6) {
            diffRotation = 0.0;
          }
          wrappedTotalRotation = jointLimits[1][c_i] - jointLimits[0][c_i];
          if ((diffRotation < wrappedTotalRotation) ||
              (fabs(diffRotation - wrappedTotalRotation) < 1.0E-6)) {
            /*  Make sure the final value is definitively inside the joint */
            /*  limits if it was on the bound */
            diffRotation = fmin(diffRotation, wrappedTotalRotation);
            /*  Update the configuration */
            eulAngles[c_i] = jointLimits[0][c_i] + diffRotation;
            c_i++;
          } else {
            /* isEqualWithinTolerance Check if two matrices are equal within a
             * set tolerance */
            /*    This is a convenience function designed for inputs with up to
             * two */
            /*    dimensions. If the input has 3+ dimensions, a non-scalar
             * output will be */
            /*    returned. */
            /*  If any element is NaN, the whole array will be thrown out so */
            /*  there is no need to continue */
            eulAngles[0] = rtNaN;
            eulAngles[1] = rtNaN;
            eulAngles[2] = rtNaN;
            exitg1 = true;
          }
        } else {
          c_i++;
        }
      }
      q123Opts_data[jtIdx] = eulAngles[0];
      eulAngles[0] = q456Opts_data[jtIdx];
      q123Opts_data[jtIdx + q123Opts_size[0]] = eulAngles[1];
      eulAngles[1] = q456Opts_data[b_q456Opts_data_tmp];
      q123Opts_data[jtIdx + q123Opts_size[0] * 2] = eulAngles[2];
      eulAngles[2] = q456Opts_data[q456Opts_data_tmp];
      applyJointLimits(eulAngles, eulAltUnwrapped);
      q456Opts_data[jtIdx] = eulAltUnwrapped[0];
      q456Opts_data[b_q456Opts_data_tmp] = eulAltUnwrapped[1];
      q456Opts_data[q456Opts_data_tmp] = eulAltUnwrapped[2];
      eulAngles[0] = q456Opts_data[limIdx];
      eulAngles[1] = q456Opts_data[j];
      eulAngles[2] = q456Opts_data[trueCount];
      applyJointLimits(eulAngles, eulAltUnwrapped);
      q456Opts_data[limIdx] = eulAltUnwrapped[0];
      q456Opts_data[j] = eulAltUnwrapped[1];
      q456Opts_data[trueCount] = eulAltUnwrapped[2];
    }
    /*  Filter out any remaining rows with NaNs in them by getting the index of
     * the valid rows and only assembling those in the final output */
    repmat(q123Opts_data, q123Opts_size, varargin_1_data, a_size);
    if (a_size[0] != 0) {
      q456Opts_data_tmp = a_size[0];
    } else if (result_tmp != 0) {
      q456Opts_data_tmp = result_tmp;
    } else {
      q456Opts_data_tmp = 0;
    }
    empty_non_axis_sizes = (q456Opts_data_tmp == 0);
    if (empty_non_axis_sizes || (a_size[0] != 0)) {
      i2 = 3;
      input_sizes_idx_1 = 3;
    } else {
      i2 = 0;
      input_sizes_idx_1 = 0;
    }
    if (empty_non_axis_sizes || (result_tmp != 0)) {
      sizes_idx_1 = 3;
    } else {
      sizes_idx_1 = 0;
    }
    i = result->size[0] * result->size[1];
    result->size[0] = q456Opts_data_tmp;
    result_tmp = i2 + sizes_idx_1;
    result->size[1] = result_tmp;
    emxEnsureCapacity_real_T(result, i);
    result_data = result->data;
    limIdx = i2;
    for (i = 0; i < limIdx; i++) {
      for (i1 = 0; i1 < q456Opts_data_tmp; i1++) {
        result_data[i1 + result->size[0] * i] =
            varargin_1_data[i1 + q456Opts_data_tmp * i];
      }
    }
    j = sizes_idx_1;
    for (i = 0; i < j; i++) {
      for (i1 = 0; i1 < q456Opts_data_tmp; i1++) {
        result_data[i1 + result->size[0] * (i + i2)] =
            q456Opts_data[i1 + q456Opts_data_tmp * i];
      }
    }
    for (i = 0; i < limIdx; i++) {
      for (i1 = 0; i1 < q456Opts_data_tmp; i1++) {
        allSolnOpts_data[i1 + q456Opts_data_tmp * i] =
            varargin_1_data[i1 + q456Opts_data_tmp * i];
      }
    }
    for (i = 0; i < j; i++) {
      for (i1 = 0; i1 < q456Opts_data_tmp; i1++) {
        allSolnOpts_data[i1 + q456Opts_data_tmp * (i + input_sizes_idx_1)] =
            q456Opts_data[i1 + q456Opts_data_tmp * i];
      }
    }
    j = result->size[0];
    a_size[0] = result->size[0];
    a_size[1] = result_tmp;
    for (i = 0; i < result_tmp; i++) {
      for (i1 = 0; i1 < j; i1++) {
        tmp_data[i1 + a_size[0] * i] =
            !rtIsNaN(result_data[i1 + result->size[0] * i]);
      }
    }
    b_q456Opts_data_tmp = e_all(tmp_data, a_size, b_tmp_data);
    trueCount = 0;
    limIdx = 0;
    for (c_i = 0; c_i < b_q456Opts_data_tmp; c_i++) {
      if (b_tmp_data[c_i]) {
        trueCount++;
        c_tmp_data[limIdx] = (signed char)c_i;
        limIdx++;
      }
    }
    /*  Create a copy of the solutions that wraps all revolute joints to pi,
     * then round within solution tolerance. */
    a_size[0] = trueCount;
    for (i = 0; i < result_tmp; i++) {
      for (i1 = 0; i1 < trueCount; i1++) {
        a_data[i1 + trueCount * i] =
            result_data[c_tmp_data[i1] + result->size[0] * i];
      }
    }
    b_wrapToPi(a_data, a_size);
    /*  Find the indices of all unique values after wrapping to pi */
    j = a_size[1];
    for (i = 0; i < j; i++) {
      limIdx = a_size[0];
      for (i1 = 0; i1 < limIdx; i1++) {
        b_q456Opts_data_tmp = i1 + a_size[0] * i;
        a_data[b_q456Opts_data_tmp] *= 1.0E+6;
      }
    }
    b_round(a_data, a_size);
    j = a_size[1];
    for (i = 0; i < j; i++) {
      limIdx = a_size[0];
      for (i1 = 0; i1 < limIdx; i1++) {
        b_q456Opts_data_tmp = i1 + a_size[0] * i;
        a_data[b_q456Opts_data_tmp] /= 1.0E+6;
      }
    }
    trueCount = unique_rows(a_data, a_size, q0_data, q123Opts_size, indx_data);
    /*  Select only unique solutions from the original set of solutions */
    j = trueCount;
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    a_size[0] = j;
    a_size[1] = result_tmp;
    for (i = 0; i < result_tmp; i++) {
      for (i1 = 0; i1 < j; i1++) {
        a_data[i1 + j * i] =
            allSolnOpts_data[c_tmp_data[(int)dist_data[i1] - 1] +
                             q456Opts_data_tmp * i];
      }
    }
    /*  Sort results using a distance metric */
    /* sortByEuclideanDistance Sort a matrix of solution configurations relative
     * to a reference configuration by Euclidean norm */
    /*    This function sorts a matrix of configurations using a pre-defined */
    /*    distance metric. The computed distance between any state and the */
    /*    reference state, referenceConfig, is a Euclidean norm of difference */
    /*    between a revolute joint's values which is then wrapped to [-pi, pi],
     */
    /*    and a displacement between a prismatic joint's values. */
    /*  Helper functions */
    /*    Copyright 2020 The MathWorks, Inc. */
    /*  Compute the distances between each configuration and the reference */
    j = RigidBodyTreeUtils_distance(qInterp[b_i][5], a_data, a_size, dist_data);
    /* , */
    /*  Sort the outputs */
    limIdx = c_sort(dist_data, &j, iidx_data);
    for (i = 0; i < result_tmp; i++) {
      for (i1 = 0; i1 < limIdx; i1++) {
        q0_data[i1 + limIdx * i] = a_data[(iidx_data[i1] + a_size[0] * i) - 1];
      }
    }
    j = trueCount;
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < result_tmp; i++) {
      b_omega[i] = q0_data[limIdx * i];
    }
    rigidBodyTree_geometricJacobian(&robot, b_omega, jacobian_data,
                                    q123Opts_size);
    b_omega[0] = omega[b_i][0];
    b_omega[3] = pd[b_i][0];
    b_omega[1] = omega[b_i][1];
    b_omega[4] = pd[b_i][1];
    b_omega[2] = omega[b_i][2];
    b_omega[5] = pd[b_i][2];
    mldivide(jacobian_data, q123Opts_size, b_omega, d_tmp_data);
    for (i = 0; i < 6; i++) {
      qdInterp[b_i][i] = d_tmp_data[i];
    }
    b_omega[0] = alpha[b_i][0];
    b_omega[3] = pdd[b_i][0];
    b_omega[1] = alpha[b_i][1];
    b_omega[4] = pdd[b_i][1];
    b_omega[2] = alpha[b_i][2];
    b_omega[5] = pdd[b_i][2];
    mldivide(jacobian_data, q123Opts_size, b_omega, d_tmp_data);
    for (i = 0; i < 6; i++) {
      qddInterp[b_i][i] = d_tmp_data[i];
    }
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < result_tmp; i++) {
      b_omega[i] = q0_data[limIdx * i];
    }
    rigidBodyTree_inverseDynamics(&robot, b_omega, a_size, &qdInterp[b_i][0],
                                  &qddInterp[b_i][0], f_tmp_data,
                                  q123Opts_size);
    for (i = 0; i < 6; i++) {
      jointTorq[b_i][i] = f_tmp_data[i];
    }
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < trueCount; i++) {
      dist_data[i] = indx_data[i];
    }
    b_sort(dist_data, &j);
    for (i = 0; i < 6; i++) {
      qInterp[b_i][i] = q0_data[limIdx * i];
    }
  }
  emxFree_real_T(&result);
  if (!robot.matlabCodegenIsDeleted) {
    robot.matlabCodegenIsDeleted = true;
  }
  if (!lobj_1.matlabCodegenIsDeleted) {
    lobj_1.matlabCodegenIsDeleted = true;
  }
  if (!lobj_1.Base.matlabCodegenIsDeleted) {
    lobj_1.Base.matlabCodegenIsDeleted = true;
  }
  for (i = 0; i < 14; i++) {
    obj = &lobj_1._pobj1[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  for (i = 0; i < 22; i++) {
    handle_matlabCodegenDestructor(&lobj_1._pobj0[i]);
  }
  d_emxFreeStruct_robotics_manip_(&lobj_1);
}

/*
 * File trailer for run_p2p.c
 *
 * [EOF]
 */
