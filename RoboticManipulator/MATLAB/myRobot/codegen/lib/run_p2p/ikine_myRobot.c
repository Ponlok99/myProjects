/*
 * File: ikine_myRobot.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "ikine_myRobot.h"
#include "abs.h"
#include "all.h"
#include "asin.h"
#include "atan2.h"
#include "nthroot.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "run_p2p_rtwutil.h"
#include "sort.h"
#include "sqrt.h"
#include "wrapTo2Pi.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void solveTrigEquations(double a, double b, double c, double theta[2]);

/* Function Definitions */
/*
 * solveTrigEquations Solve equations of the form a*cos(theta) + b*sin(theta) =
 * c for theta This function solves the common trigonometric equality by
 * equating the solution to cos(phi)sin(theta) + sin(phi)cos(theta) = sin(phi +
 * theta). The function returns two possible solutions for theta.
 *
 * Arguments    : double a
 *                double b
 *                double c
 *                double theta[2]
 * Return Type  : void
 */
static void solveTrigEquations(double a, double b, double c, double theta[2])
{
  creal_T dc;
  bool b_b;
  /*    Copyright 2020 The MathWorks, Inc. */
  theta[0] = rtNaN;
  theta[1] = rtNaN;
  /*  Handle the trivial case */
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  b_b = (fabs(a) < 1.0E-6);
  if (b_b && (fabs(b) < 1.0E-6) && (fabs(c) < 1.0E-6)) {
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    theta[0] = 0.0;
  } else if ((!b_b) || (!(fabs(b) < 1.0E-6)) || (fabs(c) < 1.0E-6)) {
    double cPrime;
    /*  As long as a or b are nonzero, a set of general solutions may be found
     */
    cPrime = c / sqrt(a * a + b * b);
    if ((cPrime < 1.0) || (cPrime - 1.0 < 1.0E-6)) {
      /*  Throw out the imaginary solutions, which occur when cPrime > 1 */
      dc.re = cPrime;
      dc.im = 0.0;
      b_asin(&dc);
      theta[0] = dc.re - b_atan2(a, b);
      theta[1] = -dc.re - b_atan2(-a, -b);
    } else {
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
    }
  } else {
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
  }
}

/*
 * applyJointLimits Convert solutions with invalid joint limits to NaNs
 *    Given an N-element configuration, an Nx2 set of lower and upper joint
 *    limits, and an N-element vector indicating the joint type (revolute or
 *    prismatic), this function checks whether the configuration is within
 *    the joint limits. If not, the configuration is converted to NaNs.
 *
 * Arguments    : const double inputConfig[3]
 *                double validConfig[3]
 * Return Type  : void
 */
void applyJointLimits(const double inputConfig[3], double validConfig[3])
{
  static const double jointLimits[2][3] = {{-2.79253, -2.094395, -6.98132},
                                           {2.79253, 2.094395, 6.98132}};
  int i;
  bool exitg1;
  /*    Copyright 2020-2021 The MathWorks, Inc. */
  /*  Initialize output */
  validConfig[0] = inputConfig[0];
  validConfig[1] = inputConfig[1];
  validConfig[2] = inputConfig[2];
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 3)) {
    if ((jointLimits[0][i] > inputConfig[i]) ||
        (jointLimits[1][i] < inputConfig[i])) {
      double jointRange;
      double wrappedJointValueOffset;
      /*  Compute the offset from the lower joint limit and compare that to */
      /*  the total range */
      wrappedJointValueOffset = wrapTo2Pi(inputConfig[i] - jointLimits[0][i]);
      /*  If the wrapped value is 2*pi, make sure it is instead registered */
      /*  as zero to ensure this doesn't fall outside the range */
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (fabs(wrappedJointValueOffset - 6.2831853071795862) < 1.0E-6) {
        wrappedJointValueOffset = 0.0;
      }
      jointRange = jointLimits[1][i] - jointLimits[0][i];
      if ((wrappedJointValueOffset < jointRange) ||
          (fabs(wrappedJointValueOffset - jointRange) < 1.0E-6)) {
        /*  Make sure the final value is definitively inside the joint */
        /*  limits if it was on the bound */
        wrappedJointValueOffset = fmin(wrappedJointValueOffset, jointRange);
        /*  Update the configuration */
        validConfig[i] = jointLimits[0][i] + wrappedJointValueOffset;
        i++;
      } else {
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        /*  If any element is NaN, the whole array will be thrown out so */
        /*  there is no need to continue */
        validConfig[0] = rtNaN;
        validConfig[1] = rtNaN;
        validConfig[2] = rtNaN;
        exitg1 = true;
      }
    } else {
      i++;
    }
  }
}

/*
 * solveFirstThreeDHJoints Solve for the first three joint angles of a
 * DH-parameterized robot This function computes the first three joint angles of
 * a robot parameterized using Denavit-Hartenberg parameters. The function
 * accepts a matrix of the fixed DH parameters, as well as the position of the
 *    fifth joint. The matrix of DH parameters is of size 6x4 for the 6
 *    non-fixed joints, where each row has the order [a alpha d 0], where a
 *    is a translation along x, alpha is a rotation about x, and d is a
 *    translation along z. The last value, which typically refers to theta
 *    (the rotation about z) for that joint, is not yet known; this function
 *    will solve for theta for the first three joints. When a robot has the
 *    last three axes intersecting, the position and orientation of the end
 *    effector can be split up: the position is entirely determined by the
 *    first three joints, while the orientation is governed by the last three
 *    joints (provided the first three are known). Furthermore, the position
 *    of any end effector can be related to the position of the fifth joint
 *    frame, which corresponds to the joint frame at the midpoint of the
 *    three intersecting joints. This frame will have the same position in
 *    any valid solution to the particular IK problem (though its orientation
 *    may differ), and its translation relative to the base frame is entirely
 *    defined by the first three joints. This function solves for those first
 *    three joints given the position of the joint 5 frame relative to the
 *    base. This solution method and notation follows Chp.3 of Pieper's 1968
 *    thesis, but adds two corrections, as well as minor notation changes and
 *    the addition of constraints to ensure only feasible solutions are
 *    output:
 *
 *    Pieper, D. The Kinematics Of Manipulators Under Computer Control.
 *    Stanford University (1968).
 *
 * Arguments    : const double jt5Pos[3]
 *                double outputThetas_data[]
 *                int outputThetas_size[2]
 * Return Type  : void
 */
void solveFirstThreeDHJoints(const double jt5Pos[3], double outputThetas_data[],
                             int outputThetas_size[2])
{
  creal_T hSolns_data[4];
  creal_T cubicRoots[3];
  creal_T D;
  creal_T cubicS;
  creal_T cubicT;
  double possThetas[3][16];
  double mat1_tmp[5];
  double R3;
  double R3_tmp;
  double b_mat1_tmp_tmp;
  double cubicD;
  double cubicR;
  double mat1_tmp_tmp;
  double polyA0;
  double polyA1;
  double t112;
  double t127;
  double t127_tmp;
  double t133;
  double t140;
  double t4;
  double t44;
  double t58;
  double t59;
  double t60;
  double t61_tmp;
  double t63_tmp;
  double t78;
  double t78_tmp;
  double t8;
  double t92;
  int hSolns_size;
  int k;
  int solIdx;
  int trueCount;
  signed char tmp_data[16];
  bool bv[3][16];
  bool bv1[16];
  bool isCoeffZero[5];
  bool hasPiSoln;
  /*  Extract DH parameters from matrix */
  /*  Note that Pieper uses "s" instead of "d" in his solutions */
  /*  Three variables derived from jt5Pos */
  R3_tmp = (jt5Pos[2] - 0.135) * (jt5Pos[2] - 0.135);
  R3 = (jt5Pos[0] * jt5Pos[0] + jt5Pos[1] * jt5Pos[1]) + R3_tmp;
  /*  The first step is to solve for theta3. This is achieved by eliminating */
  /*  theta1 and theta2 through a number of substitutions and sum-of-squares */
  /*  operations. The resultant equation for theta3, is a function of */
  /*  sin(theta3) and cos(theta3), but this can be further mapped to a */
  /*  polynomial in h, where h = tan(theta3/2). This substitutions is made */
  /*  possible by the Weierstrass transformation, which maps sin(theta3) to */
  /*  2*h/(1 + h^2) and cos(theta3) to (1-h^2)/(1+h^2). The goal is then to */
  /*  solve the polynomial for h and map the solutions back to theta3. */
  /*  As a1 and sin(alpha1) are both nonzero, use sum of squares to eliminate
   * theta2, which produces a quartic in h */
  /* solveForHGeneralCase Solve for h when a1 and alpha1 are nonzero */
  /*    To solve for theta3, it is necessary to reparameterize a trigonometric
   */
  /*    equation in terms of a new parameter h = tan(that3/2) using the */
  /*    Weierstrass equation. This function solves equation 3.39 in the Pieper
   */
  /*    inverse kinematics solution for h in the case where DH parameters a1 */
  /*    and alpha1 are both nonzero. The equation arises from the sum of */
  /*    squares of the following two equations: */
  /*       3.25: R3 = F1cos(theta2) + F2sin(theta2)2a1 + F3 */
  /*       3.26: z3 = F1sin(theta2) - F2cos(theta2)sin(alpha1) + F4 */
  /*    Here F1 to F4 are functions of theta3 (and the constant DH parameters),
   */
  /*    and R3 and z3 are functions of P, a known position input from the IK */
  /*    problem, and DH parameter d1: */
  /*       R3 = P(1)^2 + P(2)^2 + (P(3) – d1)^2 */
  /*       z3 = P(3) - d1 */
  /*    The sum of squares produces a single equation that may be */
  /*    reparameterized in h, producing a quartic polynomial in h. This */
  /*    function solves that polynomial for the values of h given R3, z3, and */
  /*    the DH parameters of the associated serial manipulator. */
  /*    Copyright 2020 The MathWorks, Inc. */
  /*  Compute the polynomial coefficients */
  /* getQuarticPolynomialCoeffs Compute the coefficients of the quartic
   * polynomial */
  /*    The first part of the solution is a fourth-order polynomial in h = */
  /*    tan(theta3/2). This function computes the coefficients of that */
  /*    polynomial, A*h^4 + B*h^3 + C*h^2 + D*h + E = 0. */
  /*  Helper functions */
  /* Polynomial coefficients for DH robot */
  t8 = R3 * R3;
  t112 = 0.0 * (jt5Pos[2] - 0.135) * 8.0;
  t44 = R3 * 0.11 * 0.026 * 4.0;
  cubicD = -0.0 * (jt5Pos[2] - 0.135);
  t63_tmp = -(cubicD * 8.0);
  t4 = R3 * 0.0 * 0.1175;
  t92 = -(t4 * 0.0) * 4.0;
  polyA1 = cubicD * 16.0;
  t127_tmp = 3.1641206337091337E-33 * (jt5Pos[2] - 0.135);
  t127 = -(t127_tmp * 8.0);
  t58 = -(R3 * 0.002209 * 2.0);
  t59 = -(R3 * 0.0121 * 2.0);
  t60 = -(R3 * 0.000676 * 2.0);
  t61_tmp = -(R3 * 0.0 * 2.0);
  t78_tmp = R3 * 0.0 * 0.0;
  t78 = -(t78_tmp * 4.0);
  t4 *= -3.4914813388431334E-15;
  cubicD = -(t4 * 4.0);
  cubicR = R3 * 0.013806249999999999 * 1.2190441939489839E-29;
  t133 = -(cubicR * 2.0);
  t140 = -(R3 * 0.013806249999999999 * 2.0);
  /*  Add exception to handle the trivial case */
  mat1_tmp_tmp = (((t8 + 4.879681E-6) + 0.00014641) + 4.5697599999999995E-7) +
                 0.002209 * R3_tmp * 4.0;
  mat1_tmp[0] = ((((((((((((((((((((((((((((((((((mat1_tmp_tmp + t44) -
                                                 7.7334399999999989E-6) -
                                                0.00013842399999999997) +
                                               2.5270959999999996E-5) +
                                              t58) +
                                             t59) +
                                            t60) +
                                           t61_tmp) +
                                          t61_tmp) +
                                         t63_tmp) +
                                        4.9077599999999992E-5) +
                                       2.8326333704911415E-62) +
                                      t78) +
                                     cubicD) +
                                    0.00019061253906249998) -
                                   5.34578E-5) -
                                  2.986568E-6) -
                                 t92) +
                                t63_tmp) +
                               cubicD) -
                              t112) -
                             1.9254010664698131E-33) +
                            t127) -
                           0.00015794349999999997) +
                          t133) +
                         t140) +
                        7.4356834892164643E-34) +
                       4.0729637944553745E-33) +
                      2.2754739876461427E-34) +
                     6.0996012499999994E-5) +
                    0.00033411125) +
                   1.8666049999999997E-5) +
                  4.64730218076029E-33) +
                 1.8128853571143305E-62) -
                0.00012199202499999999;
  b_mat1_tmp_tmp =
      (((-(R3 * 0.026 * 0.0 * 0.0 * 8.0) - -(R3 * 0.11 * 0.1175) * 8.0) -
        polyA1) -
       0.0012511399999999998) -
      6.9898399999999988E-5;
  mat1_tmp[1] =
      (((((b_mat1_tmp_tmp - 0.00010797591999999999) - -0.00059144799999999984) -
         0.0014275662499999998) +
        0.00022841059999999998) -
       1.7402663485400234E-32) -
      -0.00010797591999999999;
  polyA0 = R3 * 0.0 * 4.0;
  t4 *= 8.0;
  mat1_tmp[2] =
      (((((((((((((((((((((((((((((((t8 * 2.0 + 9.759362E-6) + 0.00029282) +
                                   9.139519999999999E-7) +
                                  5.6652667409822829E-62) +
                                 0.00038122507812499997) +
                                0.002209 * R3_tmp * 8.0) -
                               R3 * 0.002209 * 4.0) -
                              R3 * 0.0121 * 4.0) -
                             R3 * 0.000676 * 4.0) -
                            polyA0) -
                           polyA0) -
                          0.0001069156) +
                         1.7919408E-5) -
                        3.2718399999999995E-5) -
                       cubicR * 4.0) -
                      R3 * 0.013806249999999999 * 4.0) -
                     2.3892544E-5) +
                    1.4871366978432929E-33) +
                   8.1459275889107491E-33) +
                  4.5509479752922853E-34) -
                 0.000365976075) +
                0.0033411125) +
               3.7332099999999994E-5) +
              9.29460436152058E-33) -
             polyA1) -
            t78_tmp * 8.0) -
           t4) +
          3.6257707142286611E-62) +
         0.00024398404999999998) -
        polyA1) -
       t127_tmp * 16.0) -
      t4;
  mat1_tmp[3] =
      (((((b_mat1_tmp_tmp - -0.00010797591999999999) - 0.00059144799999999984) -
         0.0014275662499999998) +
        0.00022841059999999998) -
       1.7402663485400234E-32) -
      0.00010797591999999999;
  mat1_tmp[4] = ((((((((((((((((((((((((((((((((((mat1_tmp_tmp - t44) +
                                                 7.7334399999999989E-6) +
                                                0.00013842399999999997) -
                                               2.5270959999999996E-5) +
                                              t58) +
                                             t59) +
                                            t60) +
                                           t61_tmp) +
                                          t61_tmp) +
                                         t63_tmp) +
                                        4.9077599999999992E-5) +
                                       2.8326333704911415E-62) +
                                      t78) +
                                     cubicD) +
                                    0.00019061253906249998) -
                                   5.34578E-5) -
                                  2.986568E-6) +
                                 t92) +
                                t63_tmp) +
                               cubicD) +
                              t112) +
                             1.9254010664698131E-33) +
                            t127) +
                           0.00015794349999999997) +
                          t133) +
                         t140) +
                        7.4356834892164643E-34) +
                       4.0729637944553745E-33) +
                      2.2754739876461427E-34) +
                     6.0996012499999994E-5) +
                    0.00033411125) +
                   1.8666049999999997E-5) +
                  4.64730218076029E-33) +
                 1.8128853571143305E-62) -
                0.00012199202499999999;
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  for (k = 0; k < 5; k++) {
    isCoeffZero[k] = (fabs(mat1_tmp[k]) < 1.0E-6);
  }
  if (b_all(isCoeffZero)) {
    /*  The trivial case happens when the rotation of theta3 has no impact on */
    /*  the end effector position (only on the orientation) because the next */
    /*  joint lies on the axis of rotation. Since this equation is derived */
    /*  from the position solution, any real-valued orientation solution */
    /*  would work. Default to zero. */
    hSolns_size = 1;
    hSolns_data[0].re = 0.0;
    hSolns_data[0].im = 0.0;
    hasPiSoln = true;
  } else {
    /*  Solve polynomial. While there are four solutions to the quartic, there
     */
    /*  can be at most two solutions for this variable -- the others are false
     */
    /*  solutions that arise from the sum of squares. These will be eliminated
     */
    /*  below by using constraint equations. */
    /* solveQuarticPolynomial Solve 4th order polynomial */
    /*    This function accepts a vector [A B C D E] of coefficients and solves
     */
    /*    the equation of the form Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 using an */
    /*    analytical formulation. */
    /*  */
    /*    Reference: */
    /*        Weisstein, Eric W. "Quartic Equation." From MathWorld--A */
    /*        Wolfram Web Resource.
     * https://mathworld.wolfram.com/QuarticEquation.html */
    /*    Copyright 2020 The MathWorks, Inc. */
    /*  Analytical methods are not robust to division by zero, so filter out */
    /*  cases that are actually linear, quadratic, or cubic */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    isCoeffZero[0] = (fabs(mat1_tmp[0]) < 1.0E-6);
    isCoeffZero[1] = (fabs((((((b_mat1_tmp_tmp - 0.00010797591999999999) -
                               -0.00059144799999999984) -
                              0.0014275662499999998) +
                             0.00022841059999999998) -
                            1.7402663485400234E-32) -
                           -0.00010797591999999999) < 1.0E-6);
    isCoeffZero[2] = (fabs(mat1_tmp[2]) < 1.0E-6);
    isCoeffZero[3] = (fabs((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                               0.00059144799999999984) -
                              0.0014275662499999998) +
                             0.00022841059999999998) -
                            1.7402663485400234E-32) -
                           0.00010797591999999999) < 1.0E-6);
    isCoeffZero[4] = (fabs(mat1_tmp[4]) < 1.0E-6);
    if (b_all(isCoeffZero)) {
      /*  All coefficients are zero. This is a trivial solution; output zero */
      hSolns_size = 1;
      hSolns_data[0].re = 0.0;
      hSolns_data[0].im = 0.0;
    } else if (c_all(&isCoeffZero[0])) {
      /*  The first three coefficients are zero; this problem is linear */
      hSolns_size = 1;
      hSolns_data[0].re =
          -mat1_tmp[4] / ((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                              0.00059144799999999984) -
                             0.0014275662499999998) +
                            0.00022841059999999998) -
                           1.7402663485400234E-32) -
                          0.00010797591999999999);
      hSolns_data[0].im = 0.0;
    } else if (all(&isCoeffZero[0])) {
      /*  The first two coefficients are zero; this problem is quadratic */
      polyA1 = ((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                    0.00059144799999999984) -
                   0.0014275662499999998) +
                  0.00022841059999999998) -
                 1.7402663485400234E-32) -
                0.00010797591999999999) *
                   ((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                        0.00059144799999999984) -
                       0.0014275662499999998) +
                      0.00022841059999999998) -
                     1.7402663485400234E-32) -
                    0.00010797591999999999) -
               4.0 * mat1_tmp[2] * mat1_tmp[4];
      D.re = polyA1;
      D.im = 0.0;
      b_sqrt(&D);
      cubicT.re = polyA1;
      cubicT.im = 0.0;
      b_sqrt(&cubicT);
      hSolns_size = 2;
      cubicD = -((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                     0.00059144799999999984) -
                    0.0014275662499999998) +
                   0.00022841059999999998) -
                  1.7402663485400234E-32) -
                 0.00010797591999999999) +
               D.re;
      t4 = 2.0 * mat1_tmp[2];
      if (D.im == 0.0) {
        hSolns_data[0].re = cubicD / t4;
        hSolns_data[0].im = 0.0;
      } else if (cubicD == 0.0) {
        hSolns_data[0].re = 0.0;
        hSolns_data[0].im = D.im / t4;
      } else {
        hSolns_data[0].re = cubicD / t4;
        hSolns_data[0].im = D.im / t4;
      }
      cubicD = -((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                     0.00059144799999999984) -
                    0.0014275662499999998) +
                   0.00022841059999999998) -
                  1.7402663485400234E-32) -
                 0.00010797591999999999) -
               cubicT.re;
      if (0.0 - cubicT.im == 0.0) {
        hSolns_data[1].re = cubicD / t4;
        hSolns_data[1].im = 0.0;
      } else if (cubicD == 0.0) {
        hSolns_data[1].re = 0.0;
        hSolns_data[1].im = (0.0 - cubicT.im) / t4;
      } else {
        hSolns_data[1].re = cubicD / t4;
        hSolns_data[1].im = (0.0 - cubicT.im) / t4;
      }
    } else if (isCoeffZero[0]) {
      /*  The first coefficient are zero; this problem is cubic. Solve this */
      /*  using the cubic solver subroutine, which accepts inputs in standard */
      /*  polynomial form. */
      polyA0 = mat1_tmp[2] / ((((((b_mat1_tmp_tmp - 0.00010797591999999999) -
                                  -0.00059144799999999984) -
                                 0.0014275662499999998) +
                                0.00022841059999999998) -
                               1.7402663485400234E-32) -
                              -0.00010797591999999999);
      t4 = ((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                0.00059144799999999984) -
               0.0014275662499999998) +
              0.00022841059999999998) -
             1.7402663485400234E-32) -
            0.00010797591999999999) /
           ((((((b_mat1_tmp_tmp - 0.00010797591999999999) -
                -0.00059144799999999984) -
               0.0014275662499999998) +
              0.00022841059999999998) -
             1.7402663485400234E-32) -
            -0.00010797591999999999);
      /* solveCubicPolynomial Solve for a real-valued root to a cubic polynomial
       */
      /*    This function solves for a real root of the cubic polynomial in */
      /*    the form x^3 + b2*x^2 + b1*x + b0 = 0. This type of polynomial */
      /*    has three roots, two of which may be complex. */
      /*  Use Cardano's formula */
      cubicR = ((9.0 * polyA0 * t4 -
                 27.0 * (mat1_tmp[4] /
                         ((((((b_mat1_tmp_tmp - 0.00010797591999999999) -
                              -0.00059144799999999984) -
                             0.0014275662499999998) +
                            0.00022841059999999998) -
                           1.7402663485400234E-32) -
                          -0.00010797591999999999))) -
                2.0 * rt_powd_snf(polyA0, 3.0)) /
               54.0;
      cubicD = rt_powd_snf((3.0 * t4 - polyA0 * polyA0) / 9.0, 3.0) +
               cubicR * cubicR;
      /*  Make sure to call square roots with sqrt(complex()) to ensure */
      /*  code generation support when numbers are negative */
      if (cubicD < 0.0) {
        cubicT.re = cubicD;
        cubicT.im = 0.0;
        b_sqrt(&cubicT);
        D.re = cubicR + cubicT.re;
        D.im = cubicT.im;
        cubicS = power(D);
        D.re = cubicR - cubicT.re;
        D.im = 0.0 - cubicT.im;
        cubicT = power(D);
      } else {
        /*  When D is greater than zero, use nthroot, which ensures the */
        /*  real-valued root is returned */
        t4 = sqrt(cubicD);
        cubicS.re = nthroot(cubicR + t4);
        cubicS.im = 0.0;
        cubicT.re = nthroot(cubicR - t4);
        cubicT.im = 0.0;
      }
      cubicR = cubicS.re + cubicT.re;
      t133 = cubicS.im + cubicT.im;
      D.im = 0.5 * t133;
      hSolns_size = 3;
      hSolns_data[0].re = -0.33333333333333331 * polyA0 + cubicR;
      hSolns_data[0].im = t133;
      cubicD = cubicS.re - cubicT.re;
      t133 = cubicS.im - cubicT.im;
      t140 = 0.0 * cubicD - 0.8660254037844386 * t133;
      t4 = 0.0 * t133 + 0.8660254037844386 * cubicD;
      mat1_tmp_tmp = -0.33333333333333331 * polyA0 - 0.5 * cubicR;
      hSolns_data[1].re = mat1_tmp_tmp + t140;
      hSolns_data[1].im = (0.0 - D.im) + t4;
      hSolns_data[2].re = mat1_tmp_tmp - t140;
      hSolns_data[2].im = (0.0 - D.im) - t4;
    } else {
      /*  This problem is quartic */
      /*  Rewrite in standard polynomial form. Be sure to use different */
      /*  variables than are used elsewhere in code, as this may otherwise */
      /*  create global variables that corrupt data in other parts of the */
      /*  generated solution. */
      polyA0 = mat1_tmp[4] / mat1_tmp[0];
      polyA1 = ((((((b_mat1_tmp_tmp - -0.00010797591999999999) -
                    0.00059144799999999984) -
                   0.0014275662499999998) +
                  0.00022841059999999998) -
                 1.7402663485400234E-32) -
                0.00010797591999999999) /
               mat1_tmp[0];
      t127 = mat1_tmp[2] / mat1_tmp[0];
      t58 = ((((((b_mat1_tmp_tmp - 0.00010797591999999999) -
                 -0.00059144799999999984) -
                0.0014275662499999998) +
               0.00022841059999999998) -
              1.7402663485400234E-32) -
             -0.00010797591999999999) /
            mat1_tmp[0];
      /*  Compute a real solution to the resolvent cubic polynomial */
      t4 = polyA1 * t58 - 4.0 * polyA0;
      /* solveCubicPolynomial Solve for a real-valued root to a cubic polynomial
       */
      /*    This function solves for a real root of the cubic polynomial in */
      /*    the form x^3 + b2*x^2 + b1*x + b0 = 0. This type of polynomial */
      /*    has three roots, two of which may be complex. */
      /*  Use Cardano's formula */
      t127_tmp = t58 * t58;
      cubicR =
          ((9.0 * -t127 * t4 - 27.0 * ((4.0 * t127 * polyA0 - polyA1 * polyA1) -
                                       t127_tmp * polyA0)) -
           2.0 * rt_powd_snf(-t127, 3.0)) /
          54.0;
      cubicD =
          rt_powd_snf((3.0 * t4 - -t127 * -t127) / 9.0, 3.0) + cubicR * cubicR;
      /*  Make sure to call square roots with sqrt(complex()) to ensure */
      /*  code generation support when numbers are negative */
      if (cubicD < 0.0) {
        cubicT.re = cubicD;
        cubicT.im = 0.0;
        b_sqrt(&cubicT);
        D.re = cubicR + cubicT.re;
        D.im = cubicT.im;
        cubicS = power(D);
        D.re = cubicR - cubicT.re;
        D.im = 0.0 - cubicT.im;
        cubicT = power(D);
      } else {
        /*  When D is greater than zero, use nthroot, which ensures the */
        /*  real-valued root is returned */
        t4 = sqrt(cubicD);
        cubicS.re = nthroot(cubicR + t4);
        cubicS.im = 0.0;
        cubicT.re = nthroot(cubicR - t4);
        cubicT.im = 0.0;
      }
      cubicR = cubicS.re + cubicT.re;
      t133 = cubicS.im + cubicT.im;
      D.im = 0.5 * t133;
      mat1_tmp_tmp = -0.33333333333333331 * -t127;
      cubicRoots[0].re = mat1_tmp_tmp + cubicR;
      cubicRoots[0].im = t133;
      cubicD = cubicS.re - cubicT.re;
      t4 = cubicS.im - cubicT.im;
      t140 = 0.0 * cubicD - 0.8660254037844386 * t4;
      t4 = 0.0 * t4 + 0.8660254037844386 * cubicD;
      mat1_tmp_tmp -= 0.5 * cubicR;
      cubicRoots[1].re = mat1_tmp_tmp + t140;
      cubicRoots[1].im = (0.0 - D.im) + t4;
      cubicRoots[2].re = mat1_tmp_tmp - t140;
      cubicRoots[2].im = (0.0 - D.im) - t4;
      /*  Select a real-valued root */
      cubicT = cubicRoots[0];
      if (t133 == 0.0) {
        cubicT = cubicRoots[0];
      }
      if (cubicRoots[1].im == 0.0) {
        cubicT = cubicRoots[1];
      }
      if (cubicRoots[2].im == 0.0) {
        cubicT = cubicRoots[2];
      }
      /*  To minimize code generation issues, declare contents of the square */
      /*  roots to be complex to avoid unexpected complex terms */
      /*  Compute supporting elements */
      cubicS.re = (0.25 * t127_tmp - t127) + cubicT.re;
      cubicS.im = cubicT.im;
      b_sqrt(&cubicS);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      D.re = cubicS.re;
      D.im = cubicS.im;
      if (b_abs(D) < 1.0E-6) {
        cubicD = cubicT.re * cubicT.im;
        D.re = (cubicT.re * cubicT.re - cubicT.im * cubicT.im) - 4.0 * polyA0;
        D.im = cubicD + cubicD;
        b_sqrt(&D);
        cubicT.re = 2.0 * D.re;
        cubicT.im = 2.0 * D.im;
        polyA1 = 0.75 * t127_tmp - 2.0 * t127;
        D.re = polyA1 + cubicT.re;
        D.im = cubicT.im;
        b_sqrt(&D);
        cubicT.re = polyA1 - cubicT.re;
        cubicT.im = 0.0 - cubicT.im;
        b_sqrt(&cubicT);
      } else {
        cubicT.re = cubicS.re * cubicS.re - cubicS.im * cubicS.im;
        cubicD = cubicS.re * cubicS.im;
        cubicT.im = cubicD + cubicD;
        mat1_tmp_tmp =
            0.25 * ((4.0 * t58 * t127 - 8.0 * polyA1) - rt_powd_snf(t58, 3.0));
        if (cubicS.im == 0.0) {
          t140 = mat1_tmp_tmp / cubicS.re;
          cubicD = 0.0;
        } else if (cubicS.re == 0.0) {
          if (mat1_tmp_tmp == 0.0) {
            t140 = 0.0 / cubicS.im;
            cubicD = 0.0;
          } else {
            t140 = 0.0;
            cubicD = -(mat1_tmp_tmp / cubicS.im);
          }
        } else {
          t133 = fabs(cubicS.re);
          cubicD = fabs(cubicS.im);
          if (t133 > cubicD) {
            t4 = cubicS.im / cubicS.re;
            cubicR = cubicS.re + t4 * cubicS.im;
            t140 = (mat1_tmp_tmp + t4 * 0.0) / cubicR;
            cubicD = (0.0 - t4 * mat1_tmp_tmp) / cubicR;
          } else if (cubicD == t133) {
            if (cubicS.re > 0.0) {
              t4 = 0.5;
            } else {
              t4 = -0.5;
            }
            if (cubicS.im > 0.0) {
              polyA0 = 0.5;
            } else {
              polyA0 = -0.5;
            }
            t140 = (mat1_tmp_tmp * t4 + 0.0 * polyA0) / t133;
            cubicD = (0.0 * t4 - mat1_tmp_tmp * polyA0) / t133;
          } else {
            t4 = cubicS.re / cubicS.im;
            cubicR = cubicS.im + t4 * cubicS.re;
            t140 = t4 * mat1_tmp_tmp / cubicR;
            cubicD = (t4 * 0.0 - mat1_tmp_tmp) / cubicR;
          }
        }
        polyA1 = (0.75 * t127_tmp - cubicT.re) - 2.0 * t127;
        D.re = polyA1 + t140;
        D.im = (0.0 - cubicT.im) + cubicD;
        b_sqrt(&D);
        if (cubicS.im == 0.0) {
          t140 = mat1_tmp_tmp / cubicS.re;
          cubicD = 0.0;
        } else if (cubicS.re == 0.0) {
          if (mat1_tmp_tmp == 0.0) {
            t140 = 0.0 / cubicS.im;
            cubicD = 0.0;
          } else {
            t140 = 0.0;
            cubicD = -(mat1_tmp_tmp / cubicS.im);
          }
        } else {
          t133 = fabs(cubicS.re);
          cubicD = fabs(cubicS.im);
          if (t133 > cubicD) {
            t4 = cubicS.im / cubicS.re;
            cubicR = cubicS.re + t4 * cubicS.im;
            t140 = (mat1_tmp_tmp + t4 * 0.0) / cubicR;
            cubicD = (0.0 - t4 * mat1_tmp_tmp) / cubicR;
          } else if (cubicD == t133) {
            if (cubicS.re > 0.0) {
              t4 = 0.5;
            } else {
              t4 = -0.5;
            }
            if (cubicS.im > 0.0) {
              polyA0 = 0.5;
            } else {
              polyA0 = -0.5;
            }
            t140 = (mat1_tmp_tmp * t4 + 0.0 * polyA0) / t133;
            cubicD = (0.0 * t4 - mat1_tmp_tmp * polyA0) / t133;
          } else {
            t4 = cubicS.re / cubicS.im;
            cubicR = cubicS.im + t4 * cubicS.re;
            t140 = t4 * mat1_tmp_tmp / cubicR;
            cubicD = (t4 * 0.0 - mat1_tmp_tmp) / cubicR;
          }
        }
        cubicT.re = polyA1 - t140;
        cubicT.im = (0.0 - cubicT.im) - cubicD;
        b_sqrt(&cubicT);
      }
      /*  Assemble the four solutions */
      cubicS.re *= 0.5;
      cubicS.im *= 0.5;
      D.re *= 0.5;
      D.im *= 0.5;
      cubicT.re *= 0.5;
      cubicT.im *= 0.5;
      hSolns_size = 4;
      mat1_tmp_tmp = -0.25 * t58 + cubicS.re;
      hSolns_data[0].re = mat1_tmp_tmp + D.re;
      hSolns_data[0].im = cubicS.im + D.im;
      hSolns_data[1].re = mat1_tmp_tmp - D.re;
      hSolns_data[1].im = cubicS.im - D.im;
      mat1_tmp_tmp = -0.25 * t58 - cubicS.re;
      hSolns_data[2].re = mat1_tmp_tmp + cubicT.re;
      hSolns_data[2].im = (0.0 - cubicS.im) + cubicT.im;
      hSolns_data[3].re = mat1_tmp_tmp - cubicT.re;
      hSolns_data[3].im = (0.0 - cubicS.im) - cubicT.im;
    }
    /*  Check if there is a solution at theta3 = pi, for which h is undefined,
     * by */
    /*  checking if R3 = F3 (eq 3.25) is satisfied for that solution. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    hasPiSoln = (fabs(((1.432376927890056E-30 - (jt5Pos[2] - 0.135)) *
                           (1.432376927890056E-30 - (jt5Pos[2] - 0.135)) +
                       (0.023071249999999995 - R3) *
                           (0.023071249999999995 - R3) / 0.008836) -
                      0.02086225) < 1.0E-6);
  }
  /*  Initialize the matrix of possible solutions */
  memset(&possThetas[0][0], 0, 48U * sizeof(double));
  /*  After all solutions are processed, rows with NaNs will be removed */
  /*  Initialize theta3 to NaN and replace based on actual solutions */
  for (solIdx = 0; solIdx < 16; solIdx++) {
    possThetas[2][solIdx] = rtNaN;
  }
  for (k = 0; k < hSolns_size; k++) {
    /*  Ensure only real solutions to h are converted */
    /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
    /*    This function replaces imaginary values with NaNs. This is useful when
     */
    /*    the element is part of a matrix, and rendering one element of the */
    /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
     * it */
    /*    may be used to filter invalid solutions. */
    /*    Copyright 2020 The MathWorks, Inc. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    if (!(fabs(hSolns_data[k].im) < 1.0E-6)) {
      cubicD = rtNaN;
    } else {
      cubicD = hSolns_data[k].re;
    }
    if (rtIsNaN(cubicD)) {
      /*  When h3 is imaginary, theta3 = NaN */
      possThetas[2][k] = rtNaN;
      possThetas[2][k + 4] = rtNaN;
    } else {
      /*  When h3 is real, there are two possible equivalent values of theta3 */
      possThetas[2][k] = 2.0 * b_atan2(cubicD, 1.0);
      possThetas[2][k + 4] = 2.0 * b_atan2(-cubicD, -1.0);
    }
  }
  if (hasPiSoln) {
    possThetas[2][hSolns_size] = 3.1415926535897931;
  }
  for (k = 0; k < 8; k++) {
    /*  If theta3 is NaN or imaginary, replace whole row with NaNs and skip to
     * next row */
    mat1_tmp_tmp = possThetas[2][k];
    /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
    /*    This function replaces imaginary values with NaNs. This is useful when
     */
    /*    the element is part of a matrix, and rendering one element of the */
    /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
     * it */
    /*    may be used to filter invalid solutions. */
    /*    Copyright 2020 The MathWorks, Inc. */
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    if (rtIsNaN(mat1_tmp_tmp)) {
      possThetas[0][k] = rtNaN;
      possThetas[1][k] = rtNaN;
      possThetas[2][k] = rtNaN;
    } else {
      double realSolutionPair1[2];
      double theta2[2];
      double theta2Constraint[2];
      double theta2Opts[2];
      /*  Compute key subexpressions f1 to f3 and F1 to F4, which are functions
       * of theta3 */
      /* computef13SupportingEquations Compute f1 to f3 supporting equations */
      /*    This function computes f1 to f3, which are functions of theta3. For
       * a */
      /*    given robot with three consecutive revolute axes, the position of a
       */
      /*    joint a distance s4 along the joint 3 axis can be described as: */
      /*       P = T1*T2*T3*[0; 0; s4; 1], */
      /*    where Ti represent transformation matrices associated with links.
       * Then */
      /*    this equation may be rewritten as P = T1*T2*f. This function
       * computes */
      /*    the values of f that satisfy the rewritten equation. */
      /*  Helper functions */
      /*    Copyright 2020 The MathWorks, Inc. */
      /*  Initialize output */
      /*  Compute component terms */
      cubicD = cos(mat1_tmp_tmp);
      t4 = sin(mat1_tmp_tmp);
      /*  Assemble outputs. Note that there is technically a fourth output, f(4)
       * = */
      /*  1, but its value is unused, so it is not computed or returned. */
      t133 = 0.026 * cubicD + -0.1175 * t4;
      cubicR = 0.026 * t4 - -0.1175 * cubicD;
      /* computeF14SupportingEquations Compute intermediate variables F1 to F4
       */
      /*    This function computes F1 to F4, which are intermediate variables in
       */
      /*    Pieper's derivation that are functions of the theta3 joint position
       */
      /*    (and constant parameters). The function accepts several DH
       * parameters, */
      /*    as well as the intermediate variables f1 to f3, which are functions
       * of */
      /*    theta3, and outputs the four F1 to F4 intermediate variables. */
      /*    Copyright 2020 The MathWorks, Inc. */
      /*  Initialize output */
      cubicD = cubicR * 0.0;
      /*  Compute theta2. The exact approach depends on the DH */
      /*  parameters, but the gist is the same: since the equations */
      /*  output multiple solutions, but some are actually just results */
      /*  of the sum of squares, i.e., they solve the local problem, */
      /*  but do not actually solve the overlying problem. Rather than */
      /*  compute all solutions and filter at the end, we filter here */
      /*  by always solving using two different equations. Then we */
      /*  choose only the solution that satisfies both equations. */
      /*  Since a1 and sin(alpha1) are both nonzero, solve for theta2 using
       * equation 3.25 and 3.26 */
      solveTrigEquations(
          (t133 + 0.11) * 2.0 * 0.047, -cubicR * 2.0 * 0.047,
          R3 - ((((((0.0 * (cubicD - 4.1024905731406813E-16) * 2.0 +
                     0.11 * t133 * 2.0) +
                    0.002209) +
                   0.0121) +
                  t133 * t133) +
                 cubicR * cubicR) +
                1.6830428902708156E-31),
          theta2Opts);
      solveTrigEquations(-cubicR, -(t133 + 0.11),
                         (jt5Pos[2] - 0.135) -
                             -3.4914813388431334E-15 *
                                 (cubicD - 4.1024905731406813E-16),
                         theta2Constraint);
      /*  Choose the solution(s) that solve both equations */
      /* chooseCorrectSolution Choose the solution that appears in both
       * solutionPair1 and solutionPair2 */
      /*    This helper function is used to choose a correct solution when two
       */
      /*    solutions are provided, e.g. as the result of a sum of squares. The
       */
      /*    function accepts two 2-element vectors, solutionPair1 and */
      /*    solutionPair2, which represent the solution options from the source
       * and */
      /*    constraint equations, respectively. The correct solution will be the
       */
      /*    solution both of the source equation, as well as a constraint
       * equation */
      /*    for the same problem. This helper simply chooses the value that
       * occurs */
      /*    in both the original and constraint solutions, within a tolerance.
       */
      /*    Copyright 2020 The MathWorks, Inc. */
      /*  Filter any imaginary values out of the solution pairs by replacing
       * them */
      /*  with NaNs */
      /*  To check equivalence, it's insufficient to just check whether the
       * values */
      /*  are equal, because they are periodic. For example, -pi and pi are both
       */
      /*  valid outcomes of wrapToPi that fail a basic equality test but are */
      /*  equivalent in this context. Therefore, it's necessary to check that
       * the */
      /*  difference of the two values, when wrapped to pi, is inside the
       * expected tolerance. */
      /*  Have to wrap to pi so that the two values are comparable */
      /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
      /*    This function replaces imaginary values with NaNs. This is useful
       * when */
      /*    the element is part of a matrix, and rendering one element of the */
      /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
       * it */
      /*    may be used to filter invalid solutions. */
      /*    Copyright 2020 The MathWorks, Inc. */
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      realSolutionPair1[0] = theta2Opts[0];
      if (fabs(realSolutionPair1[0]) > 3.1415926535897931) {
        realSolutionPair1[0] =
            wrapTo2Pi(realSolutionPair1[0] + 3.1415926535897931) -
            3.1415926535897931;
      }
      /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
      /*    This function replaces imaginary values with NaNs. This is useful
       * when */
      /*    the element is part of a matrix, and rendering one element of the */
      /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
       * it */
      /*    may be used to filter invalid solutions. */
      /*    Copyright 2020 The MathWorks, Inc. */
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (fabs(theta2Constraint[0]) > 3.1415926535897931) {
        theta2Constraint[0] =
            wrapTo2Pi(theta2Constraint[0] + 3.1415926535897931) -
            3.1415926535897931;
      }
      theta2[0] = rtNaN;
      /*  Have to wrap to pi so that the two values are comparable */
      /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
      /*    This function replaces imaginary values with NaNs. This is useful
       * when */
      /*    the element is part of a matrix, and rendering one element of the */
      /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
       * it */
      /*    may be used to filter invalid solutions. */
      /*    Copyright 2020 The MathWorks, Inc. */
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      realSolutionPair1[1] = theta2Opts[1];
      if (fabs(realSolutionPair1[1]) > 3.1415926535897931) {
        realSolutionPair1[1] =
            wrapTo2Pi(realSolutionPair1[1] + 3.1415926535897931) -
            3.1415926535897931;
      }
      /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
      /*    This function replaces imaginary values with NaNs. This is useful
       * when */
      /*    the element is part of a matrix, and rendering one element of the */
      /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
       * it */
      /*    may be used to filter invalid solutions. */
      /*    Copyright 2020 The MathWorks, Inc. */
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (fabs(theta2Constraint[1]) > 3.1415926535897931) {
        theta2Constraint[1] =
            wrapTo2Pi(theta2Constraint[1] + 3.1415926535897931) -
            3.1415926535897931;
      }
      theta2[1] = rtNaN;
      polyA0 = realSolutionPair1[0] - theta2Constraint[0];
      if (fabs(polyA0) > 3.1415926535897931) {
        polyA0 = wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
      }
      if (fabs(polyA0) < 1.0E-6) {
        theta2[0] = realSolutionPair1[0];
      }
      polyA0 = realSolutionPair1[0] - theta2Constraint[1];
      if (fabs(polyA0) > 3.1415926535897931) {
        polyA0 = wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
      }
      if (fabs(polyA0) < 1.0E-6) {
        theta2[0] = realSolutionPair1[0];
      }
      polyA0 = realSolutionPair1[1] - theta2Constraint[0];
      if (fabs(polyA0) > 3.1415926535897931) {
        polyA0 = wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
      }
      if (fabs(polyA0) < 1.0E-6) {
        theta2[1] = realSolutionPair1[1];
      }
      polyA0 = realSolutionPair1[1] - theta2Constraint[1];
      if (fabs(polyA0) > 3.1415926535897931) {
        polyA0 = wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
      }
      if (fabs(polyA0) < 1.0E-6) {
        theta2[1] = realSolutionPair1[1];
      }
      /*  Sort the output so that if there is one correct solution it is always
       * in */
      /*  the first element slot */
      sort(theta2);
      /*  Theta2 is a 2-element vector with up to two valid solutions (invalid
       */
      /*  solutions are represented by NaNs). Iterate over the possible values
       */
      /*  and add the second solution set in the latter half of the matrix (so
       */
      /*  they aren't overwritten by subsequent loops). */
      for (hSolns_size = 0; hSolns_size < 2; hSolns_size++) {
        /*  Update the local index so it's reflective of the indexed value of
         * theta2 */
        solIdx = k + (hSolns_size << 3);
        /*  Update the value of theta3 in case it was previously set to NaN, */
        /*  and replace any invalid values of theta2 with NaN */
        possThetas[2][solIdx] = mat1_tmp_tmp;
        polyA0 = theta2[hSolns_size];
        possThetas[1][solIdx] = polyA0;
        /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
        /*    This function replaces imaginary values with NaNs. This is useful
         * when */
        /*    the element is part of a matrix, and rendering one element of the
         */
        /*    matrix imaginary will make the entire matrix imaginary.
         * Furthermore, it */
        /*    may be used to filter invalid solutions. */
        /*    Copyright 2020 The MathWorks, Inc. */
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        /*  If any of the joint variables in NaN, replace it and all the */
        /*  remaining joints to solve with NaNs and move on to the next loop */
        if (rtIsNaN(polyA0)) {
          possThetas[0][solIdx] = rtNaN;
          possThetas[1][solIdx] = rtNaN;
        } else {
          /*  Compute theta1 from the first two elements of eq 3.20 */
          /* computeg12SupportingEquations Compute g1 and g2 supporting
           * equations */
          /*    This function computes g1 and g2, which are functions of theta2
           * and */
          /*    theta3. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /*  Initialize output */
          /*  Compute component terms */
          cubicD = cos(polyA0);
          t4 = sin(polyA0);
          /*  Assemble outputs */
          polyA0 = (cubicD * (t133 + 0.11) + 0.047) - t4 * cubicR;
          cubicD = (-(cubicR * 0.0 - 4.1024905731406813E-16) -
                    -3.4914813388431334E-15 * t4 * (t133 + 0.11)) -
                   -3.4914813388431334E-15 * cubicD * cubicR;
          solveTrigEquations(polyA0, cubicD, jt5Pos[0], theta2Opts);
          solveTrigEquations(-cubicD, polyA0, jt5Pos[1], theta2Constraint);
          /* chooseCorrectSolution Choose the solution that appears in both
           * solutionPair1 and solutionPair2 */
          /*    This helper function is used to choose a correct solution when
           * two */
          /*    solutions are provided, e.g. as the result of a sum of squares.
           * The */
          /*    function accepts two 2-element vectors, solutionPair1 and */
          /*    solutionPair2, which represent the solution options from the
           * source and */
          /*    constraint equations, respectively. The correct solution will be
           * the */
          /*    solution both of the source equation, as well as a constraint
           * equation */
          /*    for the same problem. This helper simply chooses the value that
           * occurs */
          /*    in both the original and constraint solutions, within a
           * tolerance. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /*  Filter any imaginary values out of the solution pairs by replacing
           * them */
          /*  with NaNs */
          /*  To check equivalence, it's insufficient to just check whether the
           * values */
          /*  are equal, because they are periodic. For example, -pi and pi are
           * both */
          /*  valid outcomes of wrapToPi that fail a basic equality test but are
           */
          /*  equivalent in this context. Therefore, it's necessary to check
           * that the */
          /*  difference of the two values, when wrapped to pi, is inside the
           * expected tolerance. */
          /*  Have to wrap to pi so that the two values are comparable */
          /* replaceImagWithNaN Replace imaginary and empty elements with NaNs
           */
          /*    This function replaces imaginary values with NaNs. This is
           * useful when */
          /*    the element is part of a matrix, and rendering one element of
           * the */
          /*    matrix imaginary will make the entire matrix imaginary.
           * Furthermore, it */
          /*    may be used to filter invalid solutions. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          realSolutionPair1[0] = theta2Opts[0];
          if (fabs(realSolutionPair1[0]) > 3.1415926535897931) {
            realSolutionPair1[0] =
                wrapTo2Pi(realSolutionPair1[0] + 3.1415926535897931) -
                3.1415926535897931;
          }
          /* replaceImagWithNaN Replace imaginary and empty elements with NaNs
           */
          /*    This function replaces imaginary values with NaNs. This is
           * useful when */
          /*    the element is part of a matrix, and rendering one element of
           * the */
          /*    matrix imaginary will make the entire matrix imaginary.
           * Furthermore, it */
          /*    may be used to filter invalid solutions. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          if (fabs(theta2Constraint[0]) > 3.1415926535897931) {
            theta2Constraint[0] =
                wrapTo2Pi(theta2Constraint[0] + 3.1415926535897931) -
                3.1415926535897931;
          }
          theta2Opts[0] = rtNaN;
          /*  Have to wrap to pi so that the two values are comparable */
          /* replaceImagWithNaN Replace imaginary and empty elements with NaNs
           */
          /*    This function replaces imaginary values with NaNs. This is
           * useful when */
          /*    the element is part of a matrix, and rendering one element of
           * the */
          /*    matrix imaginary will make the entire matrix imaginary.
           * Furthermore, it */
          /*    may be used to filter invalid solutions. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          realSolutionPair1[1] = theta2Opts[1];
          if (fabs(realSolutionPair1[1]) > 3.1415926535897931) {
            realSolutionPair1[1] =
                wrapTo2Pi(realSolutionPair1[1] + 3.1415926535897931) -
                3.1415926535897931;
          }
          /* replaceImagWithNaN Replace imaginary and empty elements with NaNs
           */
          /*    This function replaces imaginary values with NaNs. This is
           * useful when */
          /*    the element is part of a matrix, and rendering one element of
           * the */
          /*    matrix imaginary will make the entire matrix imaginary.
           * Furthermore, it */
          /*    may be used to filter invalid solutions. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          if (fabs(theta2Constraint[1]) > 3.1415926535897931) {
            theta2Constraint[1] =
                wrapTo2Pi(theta2Constraint[1] + 3.1415926535897931) -
                3.1415926535897931;
          }
          theta2Opts[1] = rtNaN;
          polyA0 = realSolutionPair1[0] - theta2Constraint[0];
          if (fabs(polyA0) > 3.1415926535897931) {
            polyA0 =
                wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
          }
          if (fabs(polyA0) < 1.0E-6) {
            theta2Opts[0] = realSolutionPair1[0];
          }
          polyA0 = realSolutionPair1[0] - theta2Constraint[1];
          if (fabs(polyA0) > 3.1415926535897931) {
            polyA0 =
                wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
          }
          if (fabs(polyA0) < 1.0E-6) {
            theta2Opts[0] = realSolutionPair1[0];
          }
          polyA0 = realSolutionPair1[1] - theta2Constraint[0];
          if (fabs(polyA0) > 3.1415926535897931) {
            polyA0 =
                wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
          }
          if (fabs(polyA0) < 1.0E-6) {
            theta2Opts[1] = realSolutionPair1[1];
          }
          polyA0 = realSolutionPair1[1] - theta2Constraint[1];
          if (fabs(polyA0) > 3.1415926535897931) {
            polyA0 =
                wrapTo2Pi(polyA0 + 3.1415926535897931) - 3.1415926535897931;
          }
          if (fabs(polyA0) < 1.0E-6) {
            theta2Opts[1] = realSolutionPair1[1];
          }
          /*  Sort the output so that if there is one correct solution it is
           * always in */
          /*  the first element slot */
          /*  Since theta1 is the last value that is solved for, only one */
          /*  of the solutions will be valid, and chooseCorrectSolution */
          /*  sorts the results so that if there is only one solution, it */
          /*  is always the first element (and the other element is nan) */
          sort(theta2Opts);
          possThetas[0][solIdx] = theta2Opts[0];
          /*  Update the array of possible theta values */
          /* replaceImagWithNaN Replace imaginary and empty elements with NaNs
           */
          /*    This function replaces imaginary values with NaNs. This is
           * useful when */
          /*    the element is part of a matrix, and rendering one element of
           * the */
          /*    matrix imaginary will make the entire matrix imaginary.
           * Furthermore, it */
          /*    may be used to filter invalid solutions. */
          /*    Copyright 2020 The MathWorks, Inc. */
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
        }
      }
    }
  }
  /*  Now we are left with an 8x3 matrix where some values are NaN. The */
  /*  function will only output the rows where all elements are non-NaN. */
  for (solIdx = 0; solIdx < 3; solIdx++) {
    for (k = 0; k < 16; k++) {
      bv[solIdx][k] = !rtIsNaN(possThetas[solIdx][k]);
    }
  }
  d_all(bv, bv1);
  trueCount = 0;
  k = 0;
  for (hSolns_size = 0; hSolns_size < 16; hSolns_size++) {
    if (bv1[hSolns_size]) {
      trueCount++;
      tmp_data[k] = (signed char)hSolns_size;
      k++;
    }
  }
  outputThetas_size[0] = trueCount;
  outputThetas_size[1] = 3;
  for (solIdx = 0; solIdx < 3; solIdx++) {
    for (k = 0; k < trueCount; k++) {
      outputThetas_data[k + trueCount * solIdx] =
          possThetas[solIdx][tmp_data[k]];
    }
  }
}

/*
 * File trailer for ikine_myRobot.c
 *
 * [EOF]
 */
