/*
 * ikine_myRobot.c
 *
 * Code generation for function 'ikine_myRobot'
 *
 */

/* Include files */
#include "ikine_myRobot.h"
#include "RigidBodyTreeUtils.h"
#include "all.h"
#include "allOrAny.h"
#include "asin.h"
#include "eye.h"
#include "nthroot.h"
#include "power.h"
#include "repmat.h"
#include "rotm2eul.h"
#include "round.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "sort.h"
#include "sqrt.h"
#include "sqrt1.h"
#include "sum.h"
#include "sumMatrixIncludeNaN.h"
#include "unique.h"
#include "wrapTo2Pi.h"
#include "wrapToPi.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo om_emlrtRSI = {
    43,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo pm_emlrtRSI = {
    55,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo qm_emlrtRSI = {
    65,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo rm_emlrtRSI = {
    76,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo sm_emlrtRSI = {
    77,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo tm_emlrtRSI = {
    78,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo um_emlrtRSI = {
    84,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo vm_emlrtRSI = {
    85,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo wm_emlrtRSI = {
    89,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo xm_emlrtRSI = {
    92,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo ym_emlrtRSI = {
    95,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo an_emlrtRSI = {
    98,                                            /* lineNo */
    "ikine_myRobot",                               /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo cn_emlrtRSI = {
    401,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo dn_emlrtRSI = {
    409,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo hn_emlrtRSI = {
    444,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo in_emlrtRSI = {
    445,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo mn_emlrtRSI = {
    472,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo nn_emlrtRSI = {
    473,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo qn_emlrtRSI = {
    491,                                           /* lineNo */
    "solveFirstThreeDHJoints",                     /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo rn_emlrtRSI = {
    692,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo sn_emlrtRSI = {
    698,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo tn_emlrtRSI = {
    712,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo un_emlrtRSI = {
    715,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo vn_emlrtRSI = {
    716,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo wn_emlrtRSI = {
    720,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo xn_emlrtRSI = {
    722,                                            /* lineNo */
    "solveFirstThreeDHJoints/solveForHGeneralCase", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo go_emlrtRSI = {
    989,                                              /* lineNo */
    "solveFirstThreeDHJoints/solveQuarticPolynomial", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"    /* pathName */
};

static emlrtRSInfo ho_emlrtRSI = {
    1003,                                             /* lineNo */
    "solveFirstThreeDHJoints/solveQuarticPolynomial", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"    /* pathName */
};

static emlrtRSInfo ro_emlrtRSI = {
    1056, /* lineNo */
    "solveFirstThreeDHJoints/solveQuarticPolynomial/solveCubicPolynomial", /* fcnName
                                                                            */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo so_emlrtRSI = {
    1057, /* lineNo */
    "solveFirstThreeDHJoints/solveQuarticPolynomial/solveCubicPolynomial", /* fcnName
                                                                            */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo dp_emlrtRSI = {
    594,                                           /* lineNo */
    "solveFirstThreeDHJoints/solveTrigEquations",  /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo ip_emlrtRSI = {
    184,                                            /* lineNo */
    "ikine_myRobot/convertRotationToZYZAxesAngles", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo kp_emlrtRSI = {
    201,                                            /* lineNo */
    "ikine_myRobot/convertRotationToZYZAxesAngles", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo lp_emlrtRSI = {
    205,                                            /* lineNo */
    "ikine_myRobot/convertRotationToZYZAxesAngles", /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m"  /* pathName */
};

static emlrtRSInfo xp_emlrtRSI = {
    143,                                           /* lineNo */
    "ikine_myRobot/applyJointLimits",              /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo yp_emlrtRSI = {
    149,                                           /* lineNo */
    "ikine_myRobot/applyJointLimits",              /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo mq_emlrtRSI = {
    34,       /* lineNo */
    "unique", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo gr_emlrtRSI = {
    42,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\sort.m" /* pathName
                                                                         */
};

static emlrtRSInfo bs_emlrtRSI = {
    113,                                           /* lineNo */
    "ikine_myRobot/sortByEuclideanDistance",       /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo cs_emlrtRSI = {
    117,                                           /* lineNo */
    "ikine_myRobot/sortByEuclideanDistance",       /* fcnName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m" /* pathName */
};

static emlrtRSInfo hs_emlrtRSI = {
    37,     /* lineNo */
    "sort", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\sort.m" /* pathName
                                                                         */
};

static emlrtBCInfo mb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    118,                                            /* lineNo */
    37,                                             /* colNo */
    "solutions",                                    /* aName */
    "ikine_myRobot/sortByEuclideanDistance",        /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    95,                                             /* lineNo */
    23,                                             /* colNo */
    "qOptsAllSolns",                                /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo ob_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    86,                                             /* lineNo */
    29,                                             /* colNo */
    "allSolnOpts",                                  /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo pb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    55,                                             /* lineNo */
    48,                                             /* colNo */
    "q123Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo qb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    70,                                             /* lineNo */
    34,                                             /* colNo */
    "q123Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo rb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    66,                                             /* lineNo */
    14,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo sb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    67,                                             /* lineNo */
    14,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo tb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    71,                                             /* lineNo */
    34,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    72,                                             /* lineNo */
    53,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    76,                                             /* lineNo */
    55,                                             /* colNo */
    "q123Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    76,                                             /* lineNo */
    18,                                             /* colNo */
    "q123Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo xb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    77,                                             /* lineNo */
    55,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    77,                                             /* lineNo */
    18,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo ac_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    78,                                             /* lineNo */
    74,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo bc_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    78,                                             /* lineNo */
    18,                                             /* colNo */
    "q456Opts",                                     /* aName */
    "ikine_myRobot",                                /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo cc_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    409,                                            /* lineNo */
    36,                                             /* colNo */
    "hSolns",                                       /* aName */
    "solveFirstThreeDHJoints",                      /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    300,                                            /* lineNo */
    54,                                             /* colNo */
    "jointsWithIncompleteRange",                    /* aName */
    "ikine_myRobot/distributeRotationOverJoints",   /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo ec_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    309,                                            /* lineNo */
    52,                                             /* colNo */
    "jointsWithCompleteRange",                      /* aName */
    "ikine_myRobot/distributeRotationOverJoints",   /* fName */
    "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", /* pName */
    0                                               /* checkKind */
};

static emlrtRTEInfo tc_emlrtRTEI = {
    247,   /* lineNo */
    5,     /* colNo */
    "cat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

static emlrtRTEInfo uc_emlrtRTEI = {
    80,    /* lineNo */
    10,    /* colNo */
    "cat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

/* Function Declarations */
static void applyJointLimits(const emlrtStack *sp, const real_T inputConfig[3],
                             real_T validConfig[3]);

static void chooseCorrectSolution(const real_T solutionPair1[2],
                                  const real_T solutionPair2[2],
                                  real_T correctSolution[2]);

static void computeF14SupportingEquations(real_T f1, real_T f2, real_T F[4]);

static void convertRotationToZYZAxesAngles(const emlrtStack *sp,
                                           const real_T tgtRotation[9],
                                           real_T actAngles[6]);

static real_T getQuarticPolynomialCoeffs(real_T R3s, real_T z3s, real_T *h3Coef,
                                         real_T *h2Coef, real_T *h1Coef,
                                         real_T *h0Coef);

static void solveCubicPolynomial(const emlrtStack *sp, real_T b2, real_T b1,
                                 real_T b0, creal_T cubicRoots[3]);

static void solveFirstThreeDHJoints(const emlrtStack *sp,
                                    const real_T jt5Pos[3],
                                    real_T outputThetas_data[],
                                    int32_T outputThetas_size[2]);

static int32_T solveForHGeneralCase(const emlrtStack *sp, real_T R3, real_T z3,
                                    creal_T hSolns_data[],
                                    boolean_T *hasFiniteNumSol,
                                    boolean_T *hasPiSoln);

static void solveTrigEquations(const emlrtStack *sp, real_T a, real_T b,
                               real_T c, real_T theta[2]);

/* Function Definitions */
static void applyJointLimits(const emlrtStack *sp, const real_T inputConfig[3],
                             real_T validConfig[3])
{
  static const real_T jointLimits[6] = {-2.79253, -2.094395, -6.98132,
                                        2.79253,  2.094395,  6.98132};
  real_T jointRange;
  real_T wrappedJointValueOffset;
  int32_T i;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 2U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 11U);
  /* applyJointLimits Convert solutions with invalid joint limits to NaNs */
  /*    Given an N-element configuration, an Nx2 set of lower and upper joint */
  /*    limits, and an N-element vector indicating the joint type (revolute or
   */
  /*    prismatic), this function checks whether the configuration is within */
  /*    the joint limits. If not, the configuration is converted to NaNs. */
  /*    Copyright 2020-2021 The MathWorks, Inc. */
  /*  Initialize output */
  validConfig[0] = inputConfig[0];
  validConfig[1] = inputConfig[1];
  validConfig[2] = inputConfig[2];
  i = 0;
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (i < 3) {
      boolean_T guard1;
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 1, 1);
      guard1 = false;
      if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 1,
                       jointLimits[i] > inputConfig[i]) ||
          covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 2,
                       jointLimits[i + 3] < inputConfig[i])) {
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 1, true);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 6, true);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 12U);
        /*  Compute the offset from the lower joint limit and compare that to */
        /*  the total range */
        wrappedJointValueOffset = wrapTo2Pi(inputConfig[i] - jointLimits[i]);
        /*  If the wrapped value is 2*pi, make sure it is instead registered */
        /*  as zero to ensure this doesn't fall outside the range */
        covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 7,
                       muDoubleScalarAbs(wrappedJointValueOffset -
                                         6.2831853071795862) < 1.0E-6)) {
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 13U);
          wrappedJointValueOffset = 0.0;
        }
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 14U);
        jointRange = jointLimits[i + 3] - jointLimits[i];
        covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 3, true);
        if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 4,
                         wrappedJointValueOffset < jointRange)) {
          guard1 = true;
        } else {
          covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 5,
                           muDoubleScalarAbs(wrappedJointValueOffset -
                                             jointRange) < 1.0E-6)) {
            guard1 = true;
          } else {
            covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 2, false);
            covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 8, false);
            covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 16U);
            /*  If any element is NaN, the whole array will be thrown out so */
            /*  there is no need to continue */
            validConfig[0] = rtNaN;
            validConfig[1] = rtNaN;
            validConfig[2] = rtNaN;
            exitg1 = 1;
          }
        }
      } else {
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 1, false);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 6, false);
        i++;
      }
      if (guard1) {
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 2, true);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 8, true);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 15U);
        /*  Make sure the final value is definitively inside the joint */
        /*  limits if it was on the bound */
        wrappedJointValueOffset =
            muDoubleScalarMin(wrappedJointValueOffset, jointRange);
        /*  Update the configuration */
        validConfig[i] = jointLimits[i] + wrappedJointValueOffset;
        i++;
      }
    } else {
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 1, 0);
      exitg1 = 1;
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  } while (exitg1 == 0);
}

static void chooseCorrectSolution(const real_T solutionPair1[2],
                                  const real_T solutionPair2[2],
                                  real_T correctSolution[2])
{
  real_T realSolutionPair1[2];
  real_T realSolutionPair2[2];
  real_T d;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 10U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 50U);
  /* chooseCorrectSolution Choose the solution that appears in both
   * solutionPair1 and solutionPair2 */
  /*    This helper function is used to choose a correct solution when two */
  /*    solutions are provided, e.g. as the result of a sum of squares. The */
  /*    function accepts two 2-element vectors, solutionPair1 and */
  /*    solutionPair2, which represent the solution options from the source and
   */
  /*    constraint equations, respectively. The correct solution will be the */
  /*    solution both of the source equation, as well as a constraint equation
   */
  /*    for the same problem. This helper simply chooses the value that occurs
   */
  /*    in both the original and constraint solutions, within a tolerance. */
  /*    Copyright 2020 The MathWorks, Inc. */
  /*  Filter any imaginary values out of the solution pairs by replacing them */
  /*  with NaNs */
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 9, 1);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 51U);
  /*  Have to wrap to pi so that the two values are comparable */
  covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
  /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
  /*    This function replaces imaginary values with NaNs. This is useful when
   */
  /*    the element is part of a matrix, and rendering one element of the */
  /*    matrix imaginary will make the entire matrix imaginary. Furthermore, it
   */
  /*    may be used to filter invalid solutions. */
  /*    Copyright 2020 The MathWorks, Inc. */
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
  covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
  realSolutionPair1[0] = solutionPair1[0];
  wrapToPi(&realSolutionPair1[0]);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
  /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
  /*    This function replaces imaginary values with NaNs. This is useful when
   */
  /*    the element is part of a matrix, and rendering one element of the */
  /*    matrix imaginary will make the entire matrix imaginary. Furthermore, it
   */
  /*    may be used to filter invalid solutions. */
  /*    Copyright 2020 The MathWorks, Inc. */
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
  covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
  realSolutionPair2[0] = solutionPair2[0];
  wrapToPi(&realSolutionPair2[0]);
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 9, 1);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 51U);
  /*  Have to wrap to pi so that the two values are comparable */
  covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
  /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
  /*    This function replaces imaginary values with NaNs. This is useful when
   */
  /*    the element is part of a matrix, and rendering one element of the */
  /*    matrix imaginary will make the entire matrix imaginary. Furthermore, it
   */
  /*    may be used to filter invalid solutions. */
  /*    Copyright 2020 The MathWorks, Inc. */
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
  covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
  realSolutionPair1[1] = solutionPair1[1];
  wrapToPi(&realSolutionPair1[1]);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
  /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
  /*    This function replaces imaginary values with NaNs. This is useful when
   */
  /*    the element is part of a matrix, and rendering one element of the */
  /*    matrix imaginary will make the entire matrix imaginary. Furthermore, it
   */
  /*    may be used to filter invalid solutions. */
  /*    Copyright 2020 The MathWorks, Inc. */
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
  covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
  realSolutionPair2[1] = solutionPair2[1];
  wrapToPi(&realSolutionPair2[1]);
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 9, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 52U);
  /*  To check equivalence, it's insufficient to just check whether the values
   */
  /*  are equal, because they are periodic. For example, -pi and pi are both */
  /*  valid outcomes of wrapToPi that fail a basic equality test but are */
  /*  equivalent in this context. Therefore, it's necessary to check that the */
  /*  difference of the two values, when wrapped to pi, is inside the expected
   * tolerance. */
  correctSolution[0] = rtNaN;
  correctSolution[1] = rtNaN;
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 10, 1);
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 11, 1);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 53U);
  d = realSolutionPair1[0] - realSolutionPair2[0];
  wrapToPi(&d);
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 19,
                 muDoubleScalarAbs(d) < 1.0E-6)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 54U);
    correctSolution[0] = realSolutionPair1[0];
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 11, 1);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 53U);
  d = realSolutionPair1[0] - realSolutionPair2[1];
  wrapToPi(&d);
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 19,
                 muDoubleScalarAbs(d) < 1.0E-6)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 54U);
    correctSolution[0] = realSolutionPair1[0];
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 11, 0);
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 10, 1);
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 11, 1);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 53U);
  d = realSolutionPair1[1] - realSolutionPair2[0];
  wrapToPi(&d);
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 19,
                 muDoubleScalarAbs(d) < 1.0E-6)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 54U);
    correctSolution[1] = realSolutionPair1[1];
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 11, 1);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 53U);
  d = realSolutionPair1[1] - realSolutionPair2[1];
  wrapToPi(&d);
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 19,
                 muDoubleScalarAbs(d) < 1.0E-6)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 54U);
    correctSolution[1] = realSolutionPair1[1];
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 11, 0);
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 10, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 55U);
  /*  Sort the output so that if there is one correct solution it is always in
   */
  /*  the first element slot */
  sort(correctSolution);
}

static void computeF14SupportingEquations(real_T f1, real_T f2, real_T F[4])
{
  real_T t5;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 7U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 43U);
  /* computeF14SupportingEquations Compute intermediate variables F1 to F4 */
  /*    This function computes F1 to F4, which are intermediate variables in */
  /*    Pieper's derivation that are functions of the theta3 joint position */
  /*    (and constant parameters). The function accepts several DH parameters,
   */
  /*    as well as the intermediate variables f1 to f3, which are functions of
   */
  /*    theta3, and outputs the four F1 to F4 intermediate variables. */
  /*    Copyright 2020 The MathWorks, Inc. */
  /*  Initialize output */
  F[0] = f1 + 0.11;
  F[1] = -f2;
  t5 = f2 * 0.0;
  F[2] = (((((0.0 * (t5 - 4.1024905731406813E-16) * 2.0 + 0.11 * f1 * 2.0) +
             0.002209) +
            0.0121) +
           f1 * f1) +
          f2 * f2) +
         1.6830428902708156E-31;
  F[3] = -3.4914813388431334E-15 * (t5 - 4.1024905731406813E-16);
}

static void convertRotationToZYZAxesAngles(const emlrtStack *sp,
                                           const real_T tgtRotation[9],
                                           real_T actAngles[6])
{
  static const real_T jointLim[4] = {-2.79253, -8.5521163267949, 2.79253,
                                     5.4105236732051};
  static const int8_T b[9] = {1, 0, 0, 0, -1, 0, 0, 0, 1};
  emlrtStack st;
  real_T newTgtRotation[9];
  real_T eulAltUnwrapped[3];
  real_T eulAngles[3];
  real_T jointAngles[2];
  real_T d;
  real_T diffRotation;
  real_T wrappedTotalRotation;
  int32_T limIdx;
  int32_T trueCount;
  int8_T b_tmp_data[2];
  int8_T tmp_data[2];
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 3U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 17U);
  /* convertRotationToZYZAxesAngles Convert desired orientation to rotation
   * about Z-Y-Z */
  /*    This function is used to three angles of rotation corresponding to */
  /*    consecutive joint angles whose joint axes intersect at a single, common
   */
  /*    point. This function addresses the case where the first joint rotation
   */
  /*    is about Z, and the subsequent angles, in order and defined relative to
   */
  /*    the first joint frame, are about Y and then Z. The function accepts the
   */
  /*    rotation of the last joint relative to the origin of the first one, as
   */
  /*    well as the sign of each axes. The second output indicates joints that
   */
  /*    are in gimbal lock, where 1 indicates that they are, and zero indicates
   */
  /*    that they are not. When joints are in gimbal lock, the affected joint */
  /*    axes align and an infinite combination of joint angles are possible (as
   */
  /*    long as the total rotation still matches the target). The default */
  /*    assumption is that rotation is divided over the joint along the */
  /*    affected axis. */
  /*    Copyright 2020 The MathWorks, Inc. */
  st.site = &ip_emlrtRSI;
  rotm2eul(&st, tgtRotation, eulAngles);
  /*  The jointsInGimalLock variable indicates redundant joints, i.e. joints */
  /*  that complement each other and can have an infinite pair of values in the
   */
  /*  directJointAngleMaps output. Initialize this value to zeros (no joints in
   */
  /*  gimbal lock). This is a flag that is consistent across rotation functions
   */
  /*  and may be used by the caller function. */
  /*  When the middle angle is zero, the first and last joints are co-axial, */
  /*  meaning there are an infinite set of solutions. Use a helper function to
   */
  /*  distribute the values consistently given knowledge of the joint limits. */
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 9,
                 muDoubleScalarAbs(eulAngles[1]) < 1.0E-6)) {
    real_T b_eulAngles[2];
    boolean_T isRevJointFullRange[2];
    boolean_T guard1;
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 18U);
    memcpy(&newTgtRotation[0], &tgtRotation[0], 9U * sizeof(real_T));
    newTgtRotation[6] = 0.0;
    newTgtRotation[2] = 0.0;
    newTgtRotation[7] = 0.0;
    newTgtRotation[5] = 0.0;
    newTgtRotation[8] = 1.0;
    st.site = &kp_emlrtRSI;
    b_rotm2eul(&st, newTgtRotation, eulAngles);
    b_eulAngles[0] = eulAngles[0];
    b_eulAngles[1] = eulAngles[2];
    wrappedTotalRotation = b_sumColumnB(b_eulAngles);
    st.site = &lp_emlrtRSI;
    covrtLogFcn(&emlrtCoverageInstance, 1U, 4U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 20U);
    /* distributeRotationOverJoints Distribute a rotation over several in-line
     * revolute joints */
    /*    When revolute joints are co-axial, the total rotation can be
     * distributed */
    /*    over the joints in a number of ways. This function assigns a default
     */
    /*    behavior that respects the joint limits. For the case where no joint
     */
    /*    limits are required, they should be provided as infinite, i.e [-inf */
    /*    inf] for each joint. The default behaviors are as follows: */
    /*  */
    /*       - If any joint limits have a range of at minimum 2*pi, all total */
    /*         rotation amounts are achievable and the rotation is distributed
     */
    /*         evenly over the joints with infinite range, assuming the other */
    /*         joints are centered within their range. */
    /*  */
    /*       - If all joints have finite ranges with total range less than 2*pi,
     */
    /*         some total rotation amounts may not be feasible and the rotation
     */
    /*         is distributed as much as possible on the distal joints (unused
     */
    /*         more proximal joints are centered). If the solution is
     * infeasible, */
    /*         a NaN-vector is returned. */
    /*  */
    /*    The goal of these distributions is to favor solutions that are */
    /*    efficient. This function accepts the total rotation (in radians) to be
     */
    /*    divided over N joints, the signs of those N joints (whether rotation
     * is */
    /*    positive or negative about the axes), and the joint limits, given as
     * an */
    /*    Nx2 matrix. */
    /*  */
    /*    If joint limits are ignored, they can be provided as infinite; the */
    /*    behavior is equivalent. This function returns an N-element row vector.
     */
    /*    Copyright 2020 The MathWorks, Inc. */
    /*  Get the total number of joints from the joint limit input */
    /*  Initialize the output */
    /*  Remap the joint limits to fit the assumption that all axes are positive.
     */
    /*  Since the joint limits can contain infinite elements, it is necessary to
     */
    /*  use element-wise multiplication, as matrix multiplication can result in
     */
    /*  NaNs when it causes sums of infinities. */
    /*  Re-order the joint limits to ensure the lower limit always comes first
     */
    /*  (in case the of a sign flip in the previous line) */
    /*  Determine the total ranges of each joint. Since all joints are revolute,
     */
    /*  a range of 2*pi or greater is equivalent to an infinite range as the IK
     */
    /*  problem does not distinguish between periodic equivalents. Note that a
     */
    /*  downstream helper in the IK solution, applyJointLimits, includes a check
     */
    /*  that wraps periodic equivalents back to their values given the joint */
    /*  limits. */
    jointAngles[0] = 0.0;
    isRevJointFullRange[0] = false;
    jointAngles[1] = 0.0;
    isRevJointFullRange[1] = true;
    for (limIdx = 0; limIdx < 2; limIdx++) {
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 2, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 21U);
      /*  Use a tolerance check on the equality. Since isEqualWithinTolerance */
      /*  returns a scalar, it is necessary to do this inside a for-loop */
      if (isRevJointFullRange[limIdx]) {
        isRevJointFullRange[limIdx] = true;
      } else {
        covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        if (muDoubleScalarAbs((8.37758 * (real_T)limIdx + 5.58506) -
                              6.2831853071795862) < 1.0E-6) {
          isRevJointFullRange[limIdx] = true;
        } else {
          isRevJointFullRange[limIdx] = false;
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&st);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 2, 0);
    /*  There are two primary cases: when some joints have full range, any */
    /*  solution is feasible and the variable values are distributed over these
     */
    /*  joints. When all of the joints have range of less than 2*pi, the problem
     */
    /*  is more complex, as some solutions may not be feasible. */
    guard1 = false;
    if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 10,
                   vectorAny(isRevJointFullRange))) {
      boolean_T b1;
      boolean_T b_b;
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 22U);
      /*  If any of the joint have infinite ranges, use that to distribute the
       */
      /*  total rotation. First, place the joints with finite ranges in the */
      /*  middle of their respective ranges, then distribute the remaining */
      /*  joint rotation evenly over the joints with at least 2*pi range. */
      trueCount = 0;
      b_b = !isRevJointFullRange[0];
      if (b_b) {
        trueCount = 1;
      }
      b1 = !isRevJointFullRange[1];
      if (b1) {
        trueCount++;
      }
      limIdx = 0;
      if (b_b) {
        tmp_data[0] = 0;
        limIdx = 1;
      }
      if (b1) {
        tmp_data[limIdx] = 1;
      }
      for (limIdx = 0; limIdx < trueCount; limIdx++) {
        int8_T i;
        covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 3, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 23U);
        if (limIdx + 1 > trueCount) {
          emlrtDynamicBoundsCheckR2012b(limIdx + 1, 1, trueCount, &dc_emlrtBCI,
                                        &st);
        }
        i = tmp_data[limIdx];
        b_eulAngles[0] = jointLim[i];
        b_eulAngles[1] = jointLim[i + 2];
        jointAngles[i] = b_sumColumnB(b_eulAngles) / 2.0;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 3, 0);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 24U);
      /*  Compute the remaining rotation and wrap it to the interval [-pi, pi],
       */
      /*  then distribute over the joints with complete range */
      diffRotation = wrappedTotalRotation - b_sumColumnB(jointAngles);
      wrapToPi(&diffRotation);
      trueCount = 0;
      if (isRevJointFullRange[0]) {
        trueCount = 1;
      }
      if (isRevJointFullRange[1]) {
        trueCount++;
      }
      limIdx = 0;
      if (isRevJointFullRange[0]) {
        b_tmp_data[0] = 0;
        limIdx = 1;
      }
      if (isRevJointFullRange[1]) {
        b_tmp_data[limIdx] = 1;
      }
      for (limIdx = 0; limIdx < trueCount; limIdx++) {
        covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 4, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 25U);
        if (limIdx + 1 > trueCount) {
          emlrtDynamicBoundsCheckR2012b(limIdx + 1, 1, trueCount, &ec_emlrtBCI,
                                        &st);
        }
        jointAngles[b_tmp_data[limIdx]] = diffRotation / (real_T)trueCount;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 4, 0);
      guard1 = true;
    } else {
      __m128d r;
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 26U);
      /*  Use an algorithm that favors loading distal joints, which are */
      /*  typically easier to change: first set all the joints to their */
      /*  mid-range values. Then iterate over the joints from first to last, */
      /*  moving each joint up or down based on the difference in the current */
      /*  total from the desired total, until the desired total is reached. */
      /*  This is essentially a cascaded bang-bang controller. */
      /*  Initialize the joint angles to their mid-range values */
      sum(jointLim, jointAngles);
      r = _mm_loadu_pd(&jointAngles[0]);
      _mm_storeu_pd(&jointAngles[0], _mm_div_pd(r, _mm_set1_pd(2.0)));
      /*  Iterate over the joints, using a feedback law to move them closer */
      /*  to the desired total */
      wrapToPi(&wrappedTotalRotation);
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 5, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 27U);
      diffRotation = wrappedTotalRotation - b_sumColumnB(jointAngles);
      wrapToPi(&diffRotation);
      jointAngles[0] +=
          muDoubleScalarSign(diffRotation) *
          muDoubleScalarMin(muDoubleScalarAbs(diffRotation), 2.79253);
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 5, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 27U);
      diffRotation = wrappedTotalRotation - b_sumColumnB(jointAngles);
      wrapToPi(&diffRotation);
      jointAngles[1] +=
          muDoubleScalarSign(diffRotation) *
          muDoubleScalarMin(muDoubleScalarAbs(diffRotation), 6.98132);
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 5, 0);
      /*  Check if the sum of the joint angles reaches the desired total. If */
      /*  not, the solution is infeasible and a vector of NaNs is returned. */
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      d = b_sumColumnB(jointAngles);
      wrapToPi(&d);
      if (covrtLogIf(
              &emlrtCoverageInstance, 1U, 0U, 11,
              covrtLogMcdc(
                  &emlrtCoverageInstance, 1U, 0U, 3,
                  !covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 6,
                                muDoubleScalarAbs(d - wrappedTotalRotation) <
                                    1.0E-6)))) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 28U);
        eulAngles[0] = rtNaN;
        eulAngles[2] = rtNaN;
      } else {
        guard1 = true;
      }
    }
    if (guard1) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 29U);
      /*  Factor back in the axes signs. Since all valid joint angles are
       * finite, */
      /*  matrix multiplication is the most efficient approach. */
      eulAngles[0] = jointAngles[0] + jointAngles[1] * 0.0;
      eulAngles[2] = jointAngles[0] * 0.0 + jointAngles[1];
    }
    /*  In this case the alternate Euler angles aren't required, as they will */
    /*  also result in a set of co-axial joints. However, to ensure codegen */
    /*  compatibility, the size must stay the same Therefore, return a set of */
    /*  NaN angles (so the second solution may be thrown out). Note that the */
    /*  axes sign is handled inside the distributeRotationOverJoints helper */
    /*  function. */
    actAngles[0] = eulAngles[0];
    actAngles[1] = rtNaN;
    actAngles[2] = eulAngles[1];
    actAngles[3] = rtNaN;
    actAngles[4] = eulAngles[2];
    actAngles[5] = rtNaN;
  } else {
    __m128d r;
    real_T c_eulAngles[6];
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 19U);
    /*  For finite solutions when the middle angle is non-zero, there are two
     * possible solutions to this problem */
    /*  that can be derived from the first solution set */
    eulAltUnwrapped[0] = eulAngles[0];
    eulAltUnwrapped[2] = eulAngles[2];
    eulAltUnwrapped[1] = -eulAngles[1];
    r = _mm_loadu_pd(&eulAltUnwrapped[0]);
    _mm_storeu_pd(&eulAltUnwrapped[0],
                  _mm_add_pd(r, _mm_set1_pd(3.1415926535897931)));
    eulAltUnwrapped[2] += 3.1415926535897931;
    eulAltUnwrapped[1] -= 3.1415926535897931;
    /*  Output the angles given the axes signs */
    b_wrapToPi(eulAltUnwrapped);
    c_eulAngles[0] = eulAngles[0];
    c_eulAngles[1] = eulAltUnwrapped[0];
    c_eulAngles[2] = eulAngles[1];
    c_eulAngles[3] = eulAltUnwrapped[1];
    c_eulAngles[4] = eulAngles[2];
    c_eulAngles[5] = eulAltUnwrapped[2];
    for (limIdx = 0; limIdx < 2; limIdx++) {
      d = c_eulAngles[limIdx];
      diffRotation = c_eulAngles[limIdx + 2];
      wrappedTotalRotation = c_eulAngles[limIdx + 4];
      for (trueCount = 0; trueCount < 3; trueCount++) {
        actAngles[limIdx + (trueCount << 1)] =
            (d * (real_T)b[3 * trueCount] +
             diffRotation * (real_T)b[3 * trueCount + 1]) +
            wrappedTotalRotation * (real_T)b[3 * trueCount + 2];
      }
    }
  }
}

static real_T getQuarticPolynomialCoeffs(real_T R3s, real_T z3s, real_T *h3Coef,
                                         real_T *h2Coef, real_T *h1Coef,
                                         real_T *h0Coef)
{
  real_T b_t57_tmp_tmp;
  real_T h2Coef_tmp;
  real_T h3Coef_tmp;
  real_T h4Coef;
  real_T h4Coef_tmp;
  real_T t110;
  real_T t112;
  real_T t114;
  real_T t114_tmp;
  real_T t115;
  real_T t26;
  real_T t44;
  real_T t45;
  real_T t46;
  real_T t47;
  real_T t50_tmp;
  real_T t52_tmp;
  real_T t56;
  real_T t56_tmp;
  real_T t57_tmp;
  real_T t57_tmp_tmp;
  real_T t8;
  real_T t80_tmp;
  real_T t92;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 13U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 62U);
  /*  Helper functions */
  /* getQuarticPolynomialCoeffs Compute the coefficients of the quartic
   * polynomial */
  /*    The first part of the solution is a fourth-order polynomial in h = */
  /*    tan(theta3/2). This function computes the coefficients of that */
  /*    polynomial, A*h^4 + B*h^3 + C*h^2 + D*h + E = 0. */
  /* Polynomial coefficients for DH robot */
  t8 = R3s * R3s;
  t26 = z3s * z3s;
  t52_tmp = -0.0 * z3s * 8.0;
  t80_tmp = -0.0 * z3s * 16.0;
  t110 = 3.1641206337091337E-33 * z3s * 8.0;
  t112 = 0.0 * z3s * 8.0;
  t44 = R3s * 0.11 * 0.026 * 4.0;
  t45 = R3s * 0.002209 * 2.0;
  t46 = R3s * 0.0121 * 2.0;
  t47 = R3s * 0.000676 * 2.0;
  t50_tmp = R3s * 0.0 * 2.0;
  t56_tmp = R3s * 0.0 * 0.0;
  t56 = t56_tmp * 4.0;
  t57_tmp_tmp = R3s * 0.0 * 0.1175;
  b_t57_tmp_tmp = t57_tmp_tmp * -3.4914813388431334E-15;
  t57_tmp = b_t57_tmp_tmp * 4.0;
  t92 = -(t57_tmp_tmp * 0.0) * 4.0;
  t114_tmp = R3s * 0.013806249999999999 * 1.2190441939489839E-29;
  t114 = t114_tmp * 2.0;
  t115 = R3s * 0.013806249999999999 * 2.0;
  h4Coef_tmp = (((t8 + 4.879681E-6) + 0.00014641) + 4.5697599999999995E-7) +
               0.002209 * t26 * 4.0;
  h4Coef = ((((((((((((((((((((((((((((((((((h4Coef_tmp + t44) -
                                            7.7334399999999989E-6) -
                                           0.00013842399999999997) +
                                          2.5270959999999996E-5) -
                                         t45) -
                                        t46) -
                                       t47) -
                                      t50_tmp) -
                                     t50_tmp) -
                                    t52_tmp) +
                                   4.9077599999999992E-5) +
                                  2.8326333704911415E-62) -
                                 t56) -
                                t57_tmp) +
                               0.00019061253906249998) -
                              5.34578E-5) -
                             2.986568E-6) -
                            t92) -
                           t52_tmp) -
                          t57_tmp) -
                         t112) -
                        1.9254010664698131E-33) -
                       t110) -
                      0.00015794349999999997) -
                     t114) -
                    t115) +
                   7.4356834892164643E-34) +
                  4.0729637944553745E-33) +
                 2.2754739876461427E-34) +
                6.0996012499999994E-5) +
               0.00033411125) +
              1.8666049999999997E-5) +
             4.64730218076029E-33) +
            1.8128853571143305E-62) -
           0.00012199202499999999;
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 23, true);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 63U);
  h3Coef_tmp =
      (((-(R3s * 0.026 * 0.0 * 0.0 * 8.0) - -(R3s * 0.11 * 0.1175) * 8.0) -
        t80_tmp) -
       0.0012511399999999998) -
      6.9898399999999988E-5;
  *h3Coef =
      (((((h3Coef_tmp - 0.00010797591999999999) - -0.00059144799999999984) -
         0.0014275662499999998) +
        0.00022841059999999998) -
       1.7402663485400234E-32) -
      -0.00010797591999999999;
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 24, true);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 64U);
  h2Coef_tmp = R3s * 0.0 * 4.0;
  t57_tmp_tmp = b_t57_tmp_tmp * 8.0;
  *h2Coef =
      (((((((((((((((((((((((((((((((t8 * 2.0 + 9.759362E-6) + 0.00029282) +
                                   9.139519999999999E-7) +
                                  5.6652667409822829E-62) +
                                 0.00038122507812499997) +
                                0.002209 * t26 * 8.0) -
                               R3s * 0.002209 * 4.0) -
                              R3s * 0.0121 * 4.0) -
                             R3s * 0.000676 * 4.0) -
                            h2Coef_tmp) -
                           h2Coef_tmp) -
                          0.0001069156) +
                         1.7919408E-5) -
                        3.2718399999999995E-5) -
                       t114_tmp * 4.0) -
                      R3s * 0.013806249999999999 * 4.0) -
                     2.3892544E-5) +
                    1.4871366978432929E-33) +
                   8.1459275889107491E-33) +
                  4.5509479752922853E-34) -
                 0.000365976075) +
                0.0033411125) +
               3.7332099999999994E-5) +
              9.29460436152058E-33) -
             t80_tmp) -
            t56_tmp * 8.0) -
           t57_tmp_tmp) +
          3.6257707142286611E-62) +
         0.00024398404999999998) -
        t80_tmp) -
       3.1641206337091337E-33 * z3s * 16.0) -
      t57_tmp_tmp;
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 25, true);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 65U);
  *h1Coef =
      (((((h3Coef_tmp - -0.00010797591999999999) - 0.00059144799999999984) -
         0.0014275662499999998) +
        0.00022841059999999998) -
       1.7402663485400234E-32) -
      0.00010797591999999999;
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 26, true);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 66U);
  *h0Coef = ((((((((((((((((((((((((((((((((((h4Coef_tmp - t44) +
                                             7.7334399999999989E-6) +
                                            0.00013842399999999997) -
                                           2.5270959999999996E-5) -
                                          t45) -
                                         t46) -
                                        t47) -
                                       t50_tmp) -
                                      t50_tmp) -
                                     t52_tmp) +
                                    4.9077599999999992E-5) +
                                   2.8326333704911415E-62) -
                                  t56) -
                                 t57_tmp) +
                                0.00019061253906249998) -
                               5.34578E-5) -
                              2.986568E-6) +
                             t92) -
                            t52_tmp) -
                           t57_tmp) +
                          t112) +
                         1.9254010664698131E-33) -
                        t110) +
                       0.00015794349999999997) -
                      t114) -
                     t115) +
                    7.4356834892164643E-34) +
                   4.0729637944553745E-33) +
                  2.2754739876461427E-34) +
                 6.0996012499999994E-5) +
                0.00033411125) +
               1.8666049999999997E-5) +
              4.64730218076029E-33) +
             1.8128853571143305E-62) -
            0.00012199202499999999;
  return h4Coef;
}

static void solveCubicPolynomial(const emlrtStack *sp, real_T b2, real_T b1,
                                 real_T b0, creal_T cubicRoots[3])
{
  emlrtStack st;
  creal_T cubicS;
  creal_T cubicT;
  real_T cubicD;
  real_T cubicR;
  real_T cubicRoots_tmp_im;
  real_T d;
  real_T re;
  real_T re_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 15U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 78U);
  /* solveCubicPolynomial Solve for a real-valued root to a cubic polynomial */
  /*    This function solves for a real root of the cubic polynomial in */
  /*    the form x^3 + b2*x^2 + b1*x + b0 = 0. This type of polynomial */
  /*    has three roots, two of which may be complex. */
  /*  Use Cardano's formula */
  cubicR =
      ((9.0 * b2 * b1 - 27.0 * b0) - 2.0 * muDoubleScalarPower(b2, 3.0)) / 54.0;
  cubicD =
      muDoubleScalarPower((3.0 * b1 - b2 * b2) / 9.0, 3.0) + cubicR * cubicR;
  /*  Make sure to call square roots with sqrt(complex()) to ensure */
  /*  code generation support when numbers are negative */
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 33, cubicD < 0.0)) {
    creal_T b_cubicR;
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 79U);
    cubicT.re = cubicD;
    cubicT.im = 0.0;
    c_sqrt(&cubicT);
    b_cubicR.re = cubicR + cubicT.re;
    b_cubicR.im = cubicT.im;
    cubicS = power(b_cubicR);
    b_cubicR.re = cubicR - cubicT.re;
    b_cubicR.im = 0.0 - cubicT.im;
    cubicT = power(b_cubicR);
  } else {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 80U);
    /*  When D is greater than zero, use nthroot, which ensures the */
    /*  real-valued root is returned */
    d = cubicD;
    st.site = &ro_emlrtRSI;
    b_sqrt(&st, &d);
    re = nthroot(cubicR + d);
    cubicS.re = re;
    cubicS.im = 0.0;
    st.site = &so_emlrtRSI;
    b_sqrt(&st, &cubicD);
    re = nthroot(cubicR - cubicD);
    cubicT.re = re;
    cubicT.im = 0.0;
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 81U);
  cubicD = cubicS.re + cubicT.re;
  re = cubicS.im + cubicT.im;
  cubicRoots_tmp_im = 0.5 * re;
  cubicRoots[0].re = -0.33333333333333331 * b2 + cubicD;
  cubicRoots[0].im = re;
  cubicR = cubicS.re - cubicT.re;
  re = cubicS.im - cubicT.im;
  re_tmp = 0.0 * cubicR - 0.8660254037844386 * re;
  re = 0.0 * re + 0.8660254037844386 * cubicR;
  d = -0.33333333333333331 * b2 - 0.5 * cubicD;
  cubicRoots[1].re = d + re_tmp;
  cubicRoots[1].im = (0.0 - cubicRoots_tmp_im) + re;
  cubicRoots[2].re = d - re_tmp;
  cubicRoots[2].im = (0.0 - cubicRoots_tmp_im) - re;
}

static void solveFirstThreeDHJoints(const emlrtStack *sp,
                                    const real_T jt5Pos[3],
                                    real_T outputThetas_data[],
                                    int32_T outputThetas_size[2])
{
  emlrtStack st;
  creal_T hSolns_data[4];
  real_T possThetas[48];
  real_T R3;
  real_T h3;
  int32_T hIdx;
  int32_T hSolns_size;
  int32_T i;
  int8_T tmp_data[16];
  boolean_T bv[48];
  boolean_T bv1[16];
  boolean_T a__3;
  boolean_T hasPiSoln;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 5U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 30U);
  /* solveFirstThreeDHJoints Solve for the first three joint angles of a
   * DH-parameterized robot */
  /*    This function computes the first three joint angles of a robot */
  /*    parameterized using Denavit-Hartenberg parameters. The function accepts
   */
  /*    a matrix of the fixed DH parameters, as well as the position of the */
  /*    fifth joint. The matrix of DH parameters is of size 6x4 for the 6 */
  /*    non-fixed joints, where each row has the order [a alpha d 0], where a */
  /*    is a translation along x, alpha is a rotation about x, and d is a */
  /*    translation along z. The last value, which typically refers to theta */
  /*    (the rotation about z) for that joint, is not yet known; this function
   */
  /*    will solve for theta for the first three joints. When a robot has the */
  /*    last three axes intersecting, the position and orientation of the end */
  /*    effector can be split up: the position is entirely determined by the */
  /*    first three joints, while the orientation is governed by the last three
   */
  /*    joints (provided the first three are known). Furthermore, the position
   */
  /*    of any end effector can be related to the position of the fifth joint */
  /*    frame, which corresponds to the joint frame at the midpoint of the */
  /*    three intersecting joints. This frame will have the same position in */
  /*    any valid solution to the particular IK problem (though its orientation
   */
  /*    may differ), and its translation relative to the base frame is entirely
   */
  /*    defined by the first three joints. This function solves for those first
   */
  /*    three joints given the position of the joint 5 frame relative to the */
  /*    base. This solution method and notation follows Chp.3 of Pieper's 1968
   */
  /*    thesis, but adds two corrections, as well as minor notation changes and
   */
  /*    the addition of constraints to ensure only feasible solutions are */
  /*    output: */
  /*  */
  /*    Pieper, D. The Kinematics Of Manipulators Under Computer Control. */
  /*    Stanford University (1968). */
  /*  Extract DH parameters from matrix */
  /*  Note that Pieper uses "s" instead of "d" in his solutions */
  /*  Three variables derived from jt5Pos */
  R3 = (jt5Pos[0] * jt5Pos[0] + jt5Pos[1] * jt5Pos[1]) +
       (jt5Pos[2] - 0.135) * (jt5Pos[2] - 0.135);
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
  st.site = &cn_emlrtRSI;
  hSolns_size = solveForHGeneralCase(&st, R3, jt5Pos[2] - 0.135, hSolns_data,
                                     &a__3, &hasPiSoln);
  /*  Initialize the matrix of possible solutions */
  memset(&possThetas[0], 0, 48U * sizeof(real_T));
  /*  After all solutions are processed, rows with NaNs will be removed */
  /*  Initialize theta3 to NaN and replace based on actual solutions */
  for (i = 0; i < 16; i++) {
    possThetas[i + 32] = rtNaN;
  }
  for (hIdx = 0; hIdx < hSolns_size; hIdx++) {
    covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 6, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 31U);
    /*  Ensure only real solutions to h are converted */
    st.site = &dn_emlrtRSI;
    if (hIdx + 1 > hSolns_size) {
      emlrtDynamicBoundsCheckR2012b(hIdx + 1, 1, hSolns_size, &cc_emlrtBCI,
                                    &st);
    }
    covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
    /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
    /*    This function replaces imaginary values with NaNs. This is useful when
     */
    /*    the element is part of a matrix, and rendering one element of the */
    /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
     * it */
    /*    may be used to filter invalid solutions. */
    /*    Copyright 2020 The MathWorks, Inc. */
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    if (covrtLogIf(
            &emlrtCoverageInstance, 1U, 0U, 21,
            covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7,
                         !covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15,
                                       muDoubleScalarAbs(hSolns_data[hIdx].im) <
                                           1.0E-6)))) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 57U);
      h3 = rtNaN;
    } else {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
      h3 = hSolns_data[hIdx].re;
    }
    if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 12,
                   muDoubleScalarIsNaN(h3))) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 32U);
      /*  When h3 is imaginary, theta3 = NaN */
      possThetas[hIdx + 32] = rtNaN;
      possThetas[hIdx + 36] = rtNaN;
    } else {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 33U);
      /*  When h3 is real, there are two possible equivalent values of theta3 */
      possThetas[hIdx + 32] = 2.0 * muDoubleScalarAtan2(h3, 1.0);
      possThetas[hIdx + 36] = 2.0 * muDoubleScalarAtan2(-h3, -1.0);
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 6, 0);
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 13, hasPiSoln)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 34U);
    possThetas[hSolns_size + 32] = 3.1415926535897931;
  }
  for (hSolns_size = 0; hSolns_size < 8; hSolns_size++) {
    real_T d;
    covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 7, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 35U);
    /*  If theta3 is NaN or imaginary, replace whole row with NaNs and skip to
     * next row */
    d = possThetas[hSolns_size + 32];
    covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
    /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
    /*    This function replaces imaginary values with NaNs. This is useful when
     */
    /*    the element is part of a matrix, and rendering one element of the */
    /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
     * it */
    /*    may be used to filter invalid solutions. */
    /*    Copyright 2020 The MathWorks, Inc. */
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
    covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
    if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 14,
                   muDoubleScalarIsNaN(d))) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 36U);
      possThetas[hSolns_size] = rtNaN;
      possThetas[hSolns_size + 16] = rtNaN;
      possThetas[hSolns_size + 32] = rtNaN;
    } else {
      real_T F[4];
      real_T b_theta2Opts[2];
      real_T theta2[2];
      real_T theta2Constraint[2];
      real_T theta2Opts[2];
      real_T d1;
      real_T d2;
      real_T d3;
      real_T t4;
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 37U);
      /*  Compute key subexpressions f1 to f3 and F1 to F4, which are functions
       * of theta3 */
      covrtLogFcn(&emlrtCoverageInstance, 1U, 6U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 42U);
      /*  Helper functions */
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
      /*    Copyright 2020 The MathWorks, Inc. */
      /*  Initialize output */
      /*  Compute component terms */
      h3 = muDoubleScalarCos(d);
      t4 = muDoubleScalarSin(d);
      /*  Assemble outputs. Note that there is technically a fourth output, f(4)
       * = */
      /*  1, but its value is unused, so it is not computed or returned. */
      d1 = 0.026 * h3 + -0.1175 * t4;
      d2 = 0.026 * t4 - -0.1175 * h3;
      computeF14SupportingEquations(d1, d2, F);
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
      st.site = &hn_emlrtRSI;
      solveTrigEquations(&st, F[0] * 2.0 * 0.047, F[1] * 2.0 * 0.047, R3 - F[2],
                         theta2Opts);
      st.site = &in_emlrtRSI;
      solveTrigEquations(&st, F[1], -F[0], (jt5Pos[2] - 0.135) - F[3],
                         theta2Constraint);
      /*  Choose the solution(s) that solve both equations */
      chooseCorrectSolution(theta2Opts, theta2Constraint, theta2);
      /*  Theta2 is a 2-element vector with up to two valid solutions (invalid
       */
      /*  solutions are represented by NaNs). Iterate over the possible values
       */
      /*  and add the second solution set in the latter half of the matrix (so
       */
      /*  they aren't overwritten by subsequent loops). */
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 8, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 38U);
      /*  Update the local index so it's reflective of the indexed value of
       * theta2 */
      /*  Update the value of theta3 in case it was previously set to NaN, */
      /*  and replace any invalid values of theta2 with NaN */
      possThetas[hSolns_size + 16] = theta2[0];
      covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
      /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
      /*    This function replaces imaginary values with NaNs. This is useful
       * when */
      /*    the element is part of a matrix, and rendering one element of the */
      /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
       * it */
      /*    may be used to filter invalid solutions. */
      /*    Copyright 2020 The MathWorks, Inc. */
      covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
      covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
      covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
      /*  If any of the joint variables in NaN, replace it and all the */
      /*  remaining joints to solve with NaNs and move on to the next loop */
      if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 15,
                     muDoubleScalarIsNaN(theta2[0]))) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 39U);
        possThetas[hSolns_size] = rtNaN;
        possThetas[hSolns_size + 16] = rtNaN;
      } else {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 40U);
        /*  Compute theta1 from the first two elements of eq 3.20 */
        covrtLogFcn(&emlrtCoverageInstance, 1U, 8U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 44U);
        /* computeg12SupportingEquations Compute g1 and g2 supporting equations
         */
        /*    This function computes g1 and g2, which are functions of theta2
         * and */
        /*    theta3. */
        /*    Copyright 2020 The MathWorks, Inc. */
        /*  Initialize output */
        /*  Compute component terms */
        h3 = muDoubleScalarCos(theta2[0]);
        t4 = muDoubleScalarSin(theta2[0]);
        /*  Assemble outputs */
        d3 = (h3 * (d1 + 0.11) + 0.047) - t4 * d2;
        h3 = (-(d2 * 0.0 - 4.1024905731406813E-16) -
              -3.4914813388431334E-15 * t4 * (d1 + 0.11)) -
             -3.4914813388431334E-15 * h3 * d2;
        st.site = &mn_emlrtRSI;
        solveTrigEquations(&st, d3, h3, jt5Pos[0], theta2Opts);
        st.site = &nn_emlrtRSI;
        solveTrigEquations(&st, -h3, d3, jt5Pos[1], theta2Constraint);
        for (i = 0; i < 2; i++) {
          b_theta2Opts[i] = theta2Opts[i];
        }
        chooseCorrectSolution(b_theta2Opts, theta2Constraint, theta2Opts);
        /*  Since theta1 is the last value that is solved for, only one */
        /*  of the solutions will be valid, and chooseCorrectSolution */
        /*  sorts the results so that if there is only one solution, it */
        /*  is always the first element (and the other element is nan) */
        possThetas[hSolns_size] = theta2Opts[0];
        /*  Update the array of possible theta values */
        covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
        /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
        /*    This function replaces imaginary values with NaNs. This is useful
         * when */
        /*    the element is part of a matrix, and rendering one element of the
         */
        /*    matrix imaginary will make the entire matrix imaginary.
         * Furthermore, it */
        /*    may be used to filter invalid solutions. */
        /*    Copyright 2020 The MathWorks, Inc. */
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
        covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 8, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 38U);
      /*  Update the local index so it's reflective of the indexed value of
       * theta2 */
      /*  Update the value of theta3 in case it was previously set to NaN, */
      /*  and replace any invalid values of theta2 with NaN */
      possThetas[hSolns_size + 40] = d;
      possThetas[hSolns_size + 24] = theta2[1];
      covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
      /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
      /*    This function replaces imaginary values with NaNs. This is useful
       * when */
      /*    the element is part of a matrix, and rendering one element of the */
      /*    matrix imaginary will make the entire matrix imaginary. Furthermore,
       * it */
      /*    may be used to filter invalid solutions. */
      /*    Copyright 2020 The MathWorks, Inc. */
      covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
      covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
      covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
      /*  If any of the joint variables in NaN, replace it and all the */
      /*  remaining joints to solve with NaNs and move on to the next loop */
      if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 15,
                     muDoubleScalarIsNaN(theta2[1]))) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 39U);
        possThetas[hSolns_size + 8] = rtNaN;
        possThetas[hSolns_size + 24] = rtNaN;
      } else {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 40U);
        /*  Compute theta1 from the first two elements of eq 3.20 */
        covrtLogFcn(&emlrtCoverageInstance, 1U, 8U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 44U);
        /* computeg12SupportingEquations Compute g1 and g2 supporting equations
         */
        /*    This function computes g1 and g2, which are functions of theta2
         * and */
        /*    theta3. */
        /*    Copyright 2020 The MathWorks, Inc. */
        /*  Initialize output */
        /*  Compute component terms */
        h3 = muDoubleScalarCos(theta2[1]);
        t4 = muDoubleScalarSin(theta2[1]);
        /*  Assemble outputs */
        d3 = (h3 * (d1 + 0.11) + 0.047) - t4 * d2;
        h3 = (-(d2 * 0.0 - 4.1024905731406813E-16) -
              -3.4914813388431334E-15 * t4 * (d1 + 0.11)) -
             -3.4914813388431334E-15 * h3 * d2;
        st.site = &mn_emlrtRSI;
        solveTrigEquations(&st, d3, h3, jt5Pos[0], theta2Opts);
        st.site = &nn_emlrtRSI;
        solveTrigEquations(&st, -h3, d3, jt5Pos[1], theta2Constraint);
        for (i = 0; i < 2; i++) {
          b_theta2Opts[i] = theta2Opts[i];
        }
        chooseCorrectSolution(b_theta2Opts, theta2Constraint, theta2Opts);
        /*  Since theta1 is the last value that is solved for, only one */
        /*  of the solutions will be valid, and chooseCorrectSolution */
        /*  sorts the results so that if there is only one solution, it */
        /*  is always the first element (and the other element is nan) */
        possThetas[hSolns_size + 8] = theta2Opts[0];
        /*  Update the array of possible theta values */
        covrtLogFcn(&emlrtCoverageInstance, 1U, 11U);
        /* replaceImagWithNaN Replace imaginary and empty elements with NaNs */
        /*    This function replaces imaginary values with NaNs. This is useful
         * when */
        /*    the element is part of a matrix, and rendering one element of the
         */
        /*    matrix imaginary will make the entire matrix imaginary.
         * Furthermore, it */
        /*    may be used to filter invalid solutions. */
        /*    Copyright 2020 The MathWorks, Inc. */
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 20, false);
        covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 15, true);
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 7, false);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 21, false);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 58U);
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 8, 0);
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 7, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 41U);
  /*  Now we are left with an 8x3 matrix where some values are NaN. The */
  /*  function will only output the rows where all elements are non-NaN. */
  for (i = 0; i < 48; i++) {
    bv[i] = !muDoubleScalarIsNaN(possThetas[i]);
  }
  st.site = &qn_emlrtRSI;
  d_all(&st, bv, bv1);
  hIdx = 0;
  hSolns_size = 0;
  for (i = 0; i < 16; i++) {
    if (bv1[i]) {
      hIdx++;
      tmp_data[hSolns_size] = (int8_T)i;
      hSolns_size++;
    }
  }
  outputThetas_size[0] = hIdx;
  outputThetas_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (hSolns_size = 0; hSolns_size < hIdx; hSolns_size++) {
      outputThetas_data[hSolns_size + hIdx * i] =
          possThetas[tmp_data[hSolns_size] + (i << 4)];
    }
  }
}

static int32_T solveForHGeneralCase(const emlrtStack *sp, real_T R3, real_T z3,
                                    creal_T hSolns_data[],
                                    boolean_T *hasFiniteNumSol,
                                    boolean_T *hasPiSoln)
{
  emlrtStack b_st;
  emlrtStack st;
  creal_T cubicRoots[3];
  creal_T R;
  creal_T b_D;
  real_T diffMat[5];
  real_T mat1_tmp[5];
  real_T unusedExpr[4];
  real_T A;
  real_T B;
  real_T C;
  real_T D;
  real_T polyA1;
  int32_T hSolns_size;
  int32_T k;
  boolean_T isCoeffZero[5];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 12U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 59U);
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
  st.site = &rn_emlrtRSI;
  A = getQuarticPolynomialCoeffs(R3, z3, &B, &C, &D, &polyA1);
  /*  Add exception to handle the trivial case */
  st.site = &sn_emlrtRSI;
  mat1_tmp[0] = A;
  mat1_tmp[1] = B;
  mat1_tmp[2] = C;
  mat1_tmp[3] = D;
  mat1_tmp[4] = polyA1;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  for (k = 0; k < 5; k++) {
    diffMat[k] = muDoubleScalarAbs(mat1_tmp[k]);
  }
  for (k = 0; k < 5; k++) {
    isCoeffZero[k] = (diffMat[k] < 1.0E-6);
  }
  if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 22, b_all(isCoeffZero))) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 60U);
    /*  The trivial case happens when the rotation of theta3 has no impact on */
    /*  the end effector position (only on the orientation) because the next */
    /*  joint lies on the axis of rotation. Since this equation is derived */
    /*  from the position solution, any real-valued orientation solution */
    /*  would work. Default to zero. */
    hSolns_size = 1;
    hSolns_data[0].re = 0.0;
    hSolns_data[0].im = 0.0;
    *hasFiniteNumSol = false;
    *hasPiSoln = true;
  } else {
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 61U);
    /*  Solve polynomial. While there are four solutions to the quartic, there
     */
    /*  can be at most two solutions for this variable -- the others are false
     */
    /*  solutions that arise from the sum of squares. These will be eliminated
     */
    /*  below by using constraint equations. */
    st.site = &tn_emlrtRSI;
    covrtLogFcn(&emlrtCoverageInstance, 1U, 14U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 67U);
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
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    isCoeffZero[0] = (muDoubleScalarAbs(A) < 1.0E-6);
    isCoeffZero[1] = (muDoubleScalarAbs(B) < 1.0E-6);
    isCoeffZero[2] = (muDoubleScalarAbs(C) < 1.0E-6);
    isCoeffZero[3] = (muDoubleScalarAbs(D) < 1.0E-6);
    isCoeffZero[4] = (muDoubleScalarAbs(polyA1) < 1.0E-6);
    if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 27, b_all(isCoeffZero))) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 68U);
      /*  All coefficients are zero. This is a trivial solution; output zero */
      hSolns_size = 1;
      hSolns_data[0].re = 0.0;
      hSolns_data[0].im = 0.0;
    } else if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 28,
                          c_all(&isCoeffZero[0]))) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 69U);
      /*  The first three coefficients are zero; this problem is linear */
      hSolns_size = 1;
      hSolns_data[0].re = -polyA1 / D;
      hSolns_data[0].im = 0.0;
    } else if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 29,
                          all(&isCoeffZero[0]))) {
      real_T cubicRoots_tmp;
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 70U);
      /*  The first two coefficients are zero; this problem is quadratic */
      cubicRoots_tmp = D * D - 4.0 * C * polyA1;
      b_D.re = cubicRoots_tmp;
      b_D.im = 0.0;
      c_sqrt(&b_D);
      cubicRoots[0].re = cubicRoots_tmp;
      cubicRoots[0].im = 0.0;
      c_sqrt(&cubicRoots[0]);
      hSolns_size = 2;
      A = -D + b_D.re;
      polyA1 = 2.0 * C;
      if (b_D.im == 0.0) {
        hSolns_data[0].re = A / polyA1;
        hSolns_data[0].im = 0.0;
      } else if (A == 0.0) {
        hSolns_data[0].re = 0.0;
        hSolns_data[0].im = b_D.im / polyA1;
      } else {
        hSolns_data[0].re = A / polyA1;
        hSolns_data[0].im = b_D.im / polyA1;
      }
      A = -D - cubicRoots[0].re;
      if (0.0 - cubicRoots[0].im == 0.0) {
        hSolns_data[1].re = A / polyA1;
        hSolns_data[1].im = 0.0;
      } else if (A == 0.0) {
        hSolns_data[1].re = 0.0;
        hSolns_data[1].im = (0.0 - cubicRoots[0].im) / polyA1;
      } else {
        hSolns_data[1].re = A / polyA1;
        hSolns_data[1].im = (0.0 - cubicRoots[0].im) / polyA1;
      }
    } else if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 30, isCoeffZero[0])) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 71U);
      /*  The first coefficient are zero; this problem is cubic. Solve this */
      /*  using the cubic solver subroutine, which accepts inputs in standard */
      /*  polynomial form. */
      b_st.site = &go_emlrtRSI;
      solveCubicPolynomial(&b_st, C / B, D / B, polyA1 / B, cubicRoots);
      hSolns_size = 3;
      hSolns_data[0] = cubicRoots[0];
      hSolns_data[1] = cubicRoots[1];
      hSolns_data[2] = cubicRoots[2];
    } else {
      real_T cubicRoots_tmp;
      real_T polyA0;
      real_T polyA2;
      real_T polyA3;
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 72U);
      /*  This problem is quartic */
      /*  Rewrite in standard polynomial form. Be sure to use different */
      /*  variables than are used elsewhere in code, as this may otherwise */
      /*  create global variables that corrupt data in other parts of the */
      /*  generated solution. */
      polyA0 = polyA1 / A;
      polyA1 = D / A;
      polyA2 = C / A;
      polyA3 = B / A;
      /*  Compute a real solution to the resolvent cubic polynomial */
      cubicRoots_tmp = polyA3 * polyA3;
      b_st.site = &ho_emlrtRSI;
      solveCubicPolynomial(&b_st, -polyA2, polyA1 * polyA3 - 4.0 * polyA0,
                           (4.0 * polyA2 * polyA0 - polyA1 * polyA1) -
                               cubicRoots_tmp * polyA0,
                           cubicRoots);
      /*  Select a real-valued root */
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 12, 1);
      if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 31,
                     cubicRoots[0].im == 0.0)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 73U);
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 12, 1);
      if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 31,
                     cubicRoots[1].im == 0.0)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 73U);
        cubicRoots[0] = cubicRoots[1];
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 12, 1);
      if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 31,
                     cubicRoots[2].im == 0.0)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 73U);
        cubicRoots[0] = cubicRoots[2];
      }
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 12, 0);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 74U);
      /*  To minimize code generation issues, declare contents of the square */
      /*  roots to be complex to avoid unexpected complex terms */
      /*  Compute supporting elements */
      R.re = (0.25 * cubicRoots_tmp - polyA2) + cubicRoots[0].re;
      R.im = cubicRoots[0].im;
      c_sqrt(&R);
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 32,
                     muDoubleScalarHypot(R.re, R.im) < 1.0E-6)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 75U);
        polyA1 = cubicRoots[0].re * cubicRoots[0].im;
        b_D.re = (cubicRoots[0].re * cubicRoots[0].re -
                  cubicRoots[0].im * cubicRoots[0].im) -
                 4.0 * polyA0;
        b_D.im = polyA1 + polyA1;
        c_sqrt(&b_D);
        cubicRoots[0].re = 2.0 * b_D.re;
        cubicRoots[0].im = 2.0 * b_D.im;
        cubicRoots_tmp = 0.75 * cubicRoots_tmp - 2.0 * polyA2;
        b_D.re = cubicRoots_tmp + cubicRoots[0].re;
        b_D.im = cubicRoots[0].im;
        c_sqrt(&b_D);
        cubicRoots[0].re = cubicRoots_tmp - cubicRoots[0].re;
        cubicRoots[0].im = 0.0 - cubicRoots[0].im;
        c_sqrt(&cubicRoots[0]);
      } else {
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 76U);
        cubicRoots[0].re = R.re * R.re - R.im * R.im;
        A = R.re * R.im;
        cubicRoots[0].im = A + A;
        polyA0 = 0.25 * ((4.0 * polyA3 * polyA2 - 8.0 * polyA1) -
                         muDoubleScalarPower(polyA3, 3.0));
        if (R.im == 0.0) {
          D = polyA0 / R.re;
          polyA1 = 0.0;
        } else if (R.re == 0.0) {
          if (polyA0 == 0.0) {
            D = 0.0 / R.im;
            polyA1 = 0.0;
          } else {
            D = 0.0;
            polyA1 = -(polyA0 / R.im);
          }
        } else {
          C = muDoubleScalarAbs(R.re);
          polyA1 = muDoubleScalarAbs(R.im);
          if (C > polyA1) {
            polyA1 = R.im / R.re;
            A = R.re + polyA1 * R.im;
            D = (polyA0 + polyA1 * 0.0) / A;
            polyA1 = (0.0 - polyA1 * polyA0) / A;
          } else if (polyA1 == C) {
            if (R.re > 0.0) {
              A = 0.5;
            } else {
              A = -0.5;
            }
            if (R.im > 0.0) {
              B = 0.5;
            } else {
              B = -0.5;
            }
            D = (polyA0 * A + 0.0 * B) / C;
            polyA1 = (0.0 * A - polyA0 * B) / C;
          } else {
            polyA1 = R.re / R.im;
            A = R.im + polyA1 * R.re;
            D = polyA1 * polyA0 / A;
            polyA1 = (polyA1 * 0.0 - polyA0) / A;
          }
        }
        cubicRoots_tmp =
            (0.75 * cubicRoots_tmp - cubicRoots[0].re) - 2.0 * polyA2;
        b_D.re = cubicRoots_tmp + D;
        b_D.im = (0.0 - cubicRoots[0].im) + polyA1;
        c_sqrt(&b_D);
        if (R.im == 0.0) {
          D = polyA0 / R.re;
          polyA1 = 0.0;
        } else if (R.re == 0.0) {
          if (polyA0 == 0.0) {
            D = 0.0 / R.im;
            polyA1 = 0.0;
          } else {
            D = 0.0;
            polyA1 = -(polyA0 / R.im);
          }
        } else {
          C = muDoubleScalarAbs(R.re);
          polyA1 = muDoubleScalarAbs(R.im);
          if (C > polyA1) {
            polyA1 = R.im / R.re;
            A = R.re + polyA1 * R.im;
            D = (polyA0 + polyA1 * 0.0) / A;
            polyA1 = (0.0 - polyA1 * polyA0) / A;
          } else if (polyA1 == C) {
            if (R.re > 0.0) {
              A = 0.5;
            } else {
              A = -0.5;
            }
            if (R.im > 0.0) {
              B = 0.5;
            } else {
              B = -0.5;
            }
            D = (polyA0 * A + 0.0 * B) / C;
            polyA1 = (0.0 * A - polyA0 * B) / C;
          } else {
            polyA1 = R.re / R.im;
            A = R.im + polyA1 * R.re;
            D = polyA1 * polyA0 / A;
            polyA1 = (polyA1 * 0.0 - polyA0) / A;
          }
        }
        cubicRoots[0].re = cubicRoots_tmp - D;
        cubicRoots[0].im = (0.0 - cubicRoots[0].im) - polyA1;
        c_sqrt(&cubicRoots[0]);
      }
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 77U);
      /*  Assemble the four solutions */
      R.re *= 0.5;
      R.im *= 0.5;
      b_D.re *= 0.5;
      b_D.im *= 0.5;
      cubicRoots[0].re *= 0.5;
      cubicRoots[0].im *= 0.5;
      hSolns_size = 4;
      polyA1 = -0.25 * polyA3 + R.re;
      hSolns_data[0].re = polyA1 + b_D.re;
      hSolns_data[0].im = R.im + b_D.im;
      hSolns_data[1].re = polyA1 - b_D.re;
      hSolns_data[1].im = R.im - b_D.im;
      polyA1 = -0.25 * polyA3 - R.re;
      hSolns_data[2].re = polyA1 + cubicRoots[0].re;
      hSolns_data[2].im = (0.0 - R.im) + cubicRoots[0].im;
      hSolns_data[3].re = polyA1 - cubicRoots[0].re;
      hSolns_data[3].im = (0.0 - R.im) - cubicRoots[0].im;
    }
    *hasFiniteNumSol = true;
    st.site = &un_emlrtRSI;
    covrtLogFcn(&emlrtCoverageInstance, 1U, 6U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 42U);
    /*  Helper functions */
    /* computef13SupportingEquations Compute f1 to f3 supporting equations */
    /*    This function computes f1 to f3, which are functions of theta3. For a
     */
    /*    given robot with three consecutive revolute axes, the position of a */
    /*    joint a distance s4 along the joint 3 axis can be described as: */
    /*       P = T1*T2*T3*[0; 0; s4; 1], */
    /*    where Ti represent transformation matrices associated with links. Then
     */
    /*    this equation may be rewritten as P = T1*T2*f. This function computes
     */
    /*    the values of f that satisfy the rewritten equation. */
    /*    Copyright 2020 The MathWorks, Inc. */
    /*  Initialize output */
    /*  Compute component terms */
    /*  Assemble outputs. Note that there is technically a fourth output, f(4) =
     */
    /*  1, but its value is unused, so it is not computed or returned. */
    st.site = &vn_emlrtRSI;
    computeF14SupportingEquations(-0.026000000000000013, -0.1175, unusedExpr);
    /*  Check if there is a solution at theta3 = pi, for which h is undefined,
     * by */
    /*  checking if R3 = F3 (eq 3.25) is satisfied for that solution. */
    st.site = &wn_emlrtRSI;
    st.site = &wn_emlrtRSI;
    st.site = &xn_emlrtRSI;
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    *hasPiSoln =
        (muDoubleScalarAbs(
             ((1.432376927890056E-30 - z3) * (1.432376927890056E-30 - z3) +
              (0.023071249999999995 - R3) * (0.023071249999999995 - R3) /
                  0.008836) -
             0.02086225) < 1.0E-6);
  }
  return hSolns_size;
}

static void solveTrigEquations(const emlrtStack *sp, real_T a, real_T b,
                               real_T c, real_T theta[2])
{
  emlrtStack st;
  creal_T dc;
  real_T d;
  boolean_T b_b;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 9U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 45U);
  /* solveTrigEquations Solve equations of the form a*cos(theta) + b*sin(theta)
   * = c for theta */
  /*    This function solves the common trigonometric equality by equating the
   */
  /*    solution to cos(phi)sin(theta) + sin(phi)cos(theta) = sin(phi + theta).
   */
  /*    The function returns two possible solutions for theta. */
  /*    Copyright 2020 The MathWorks, Inc. */
  theta[0] = rtNaN;
  theta[1] = rtNaN;
  /*  Handle the trivial case */
  covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
  /* isEqualWithinTolerance Check if two matrices are equal within a set
   * tolerance */
  /*    This is a convenience function designed for inputs with up to two */
  /*    dimensions. If the input has 3+ dimensions, a non-scalar output will be
   */
  /*    returned. */
  b_b = covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 7,
                     muDoubleScalarAbs(a) < 1.0E-6);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (b_b) {
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 8,
                     muDoubleScalarAbs(b) < 1.0E-6)) {
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 9,
                       muDoubleScalarAbs(c) < 1.0E-6)) {
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 4, true);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 16, true);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 46U);
        theta[0] = 0.0;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }
  if (guard3) {
    covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 4, false);
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 16, false);
    covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
    /* isEqualWithinTolerance Check if two matrices are equal within a set
     * tolerance */
    /*    This is a convenience function designed for inputs with up to two */
    /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
     * be */
    /*    returned. */
    if (b_b) {
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 11,
                       muDoubleScalarAbs(b) < 1.0E-6)) {
        covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
        covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
        /* isEqualWithinTolerance Check if two matrices are equal within a set
         * tolerance */
        /*    This is a convenience function designed for inputs with up to two
         */
        /*    dimensions. If the input has 3+ dimensions, a non-scalar output
         * will be */
        /*    returned. */
        if (!covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 12,
                          muDoubleScalarAbs(c) < 1.0E-6)) {
          covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 5, true);
          covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 17, true);
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 47U);
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }
  if (guard2) {
    covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 5, false);
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 17, false);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 48U);
    /*  As long as a or b are nonzero, a set of general solutions may be found
     */
    d = a * a + b * b;
    st.site = &dp_emlrtRSI;
    b_sqrt(&st, &d);
    d = c / d;
    if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 13, d < 1.0)) {
      guard1 = true;
    } else {
      covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
      /* isEqualWithinTolerance Check if two matrices are equal within a set
       * tolerance */
      /*    This is a convenience function designed for inputs with up to two */
      /*    dimensions. If the input has 3+ dimensions, a non-scalar output will
       * be */
      /*    returned. */
      if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 14, d - 1.0 < 1.0E-6)) {
        guard1 = true;
      } else {
        covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 6, false);
        covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 18, false);
      }
    }
  }
  if (guard1) {
    covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 6, true);
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 18, true);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 49U);
    /*  Throw out the imaginary solutions, which occur when cPrime > 1 */
    dc.re = d;
    dc.im = 0.0;
    b_asin(&dc);
    theta[0] = dc.re - muDoubleScalarAtan2(a, b);
    theta[1] = -dc.re - muDoubleScalarAtan2(-a, -b);
  }
}

void ikine_myRobot(const emlrtStack *sp, const real_T eeTform[16],
                   real_T referenceConfig, real_T qOpts_data[],
                   int32_T qOpts_size[2])
{
  static const real_T dhParams[24] = {0.047,
                                      0.11,
                                      0.026,
                                      0.0,
                                      0.0,
                                      0.0,
                                      -1.5707963267949,
                                      0.0,
                                      -1.5707963267949,
                                      1.5707963267949,
                                      -1.5707963267949,
                                      -1.5707963267949,
                                      0.135,
                                      0.0,
                                      0.0,
                                      0.1175,
                                      0.0,
                                      0.06665,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0};
  static const real_T b[16] = {6.12323399573677E-17,
                               1.0,
                               0.0,
                               0.0,
                               -1.0,
                               6.12323399573677E-17,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               1.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               1.0};
  static const real_T b_b[16] = {1.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 -1.0,
                                 1.22464679914735E-16,
                                 0.0,
                                 0.0,
                                 -1.22464679914735E-16,
                                 -1.0,
                                 0.0,
                                 0.0,
                                 0.06665,
                                 -4.08113545815855E-18,
                                 1.0};
  static const real_T dv[9] = {1.0,
                               0.0,
                               0.0,
                               0.0,
                               -3.4914813388431334E-15,
                               1.0,
                               0.0,
                               -1.0,
                               -3.4914813388431334E-15};
  static const real_T jointLimits[6] = {-2.87979, -1.91986, -1.91986,
                                        2.87979,  1.91986,  1.22173};
  __m128d r;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *result;
  real_T allSolnOpts_data[192];
  real_T b_data[192];
  real_T qOptsWrappedAndRounded_data[192];
  real_T q456Opts_data[96];
  real_T varargin_1_data[96];
  real_T q123Opts_data[48];
  real_T dist_data[32];
  real_T a[16];
  real_T eePose[16];
  real_T Ttheta_tmp;
  real_T d;
  real_T jointRange;
  real_T wrappedJointValueOffset;
  real_T *result_data;
  int32_T indx_data[32];
  int32_T q123Opts_size[2];
  int32_T qOptsWrappedAndRounded_size[2];
  int32_T b_i;
  int32_T b_q456Opts_data_tmp;
  int32_T c_q456Opts_data_tmp;
  int32_T d_q456Opts_data_tmp;
  int32_T i;
  int32_T i3;
  int32_T i4;
  int32_T jtIdx;
  int32_T loop_ub_tmp;
  int32_T q456Opts_data_tmp;
  int32_T q456Opts_size_idx_0;
  int8_T b_tmp_data[32];
  int8_T input_sizes_idx_1;
  int8_T sizes_idx_1;
  boolean_T tmp_data[192];
  boolean_T isValidRowIdx_data[32];
  boolean_T empty_non_axis_sizes;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  covrtLogFcn(&emlrtCoverageInstance, 1U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 0U);
  /* ikine_myRobot Function for generating closed-form inverse kinematics
   * solutions to the DH robot given by the parameters specified below */
  /*    $Revision: $ $Date: $ */
  /*  */
  /*    Generated on 04-Jul-2025 14:57:48 */
  /*  Compute the shifted joint limits, which are the limits during solution,
   * where theta offsets are not yet in play */
  /*  Convert the end effector pose in the global frame to the end effector
   * described by the DH parameters relative to the DH-described origin */
  for (i = 0; i < 4; i++) {
    int8_T i1;
    int8_T i2;
    input_sizes_idx_1 = iv[i];
    sizes_idx_1 = iv[i + 4];
    i1 = iv[i + 8];
    i2 = iv[i + 12];
    for (i3 = 0; i3 < 4; i3++) {
      i4 = i3 << 2;
      a[i + i4] = (((real_T)input_sizes_idx_1 * eeTform[i4] +
                    (real_T)sizes_idx_1 * eeTform[i4 + 1]) +
                   (real_T)i1 * eeTform[i4 + 2]) +
                  (real_T)i2 * eeTform[i4 + 3];
    }
    d = a[i];
    wrappedJointValueOffset = a[i + 4];
    jointRange = a[i + 8];
    Ttheta_tmp = a[i + 12];
    for (i3 = 0; i3 < 4; i3++) {
      i4 = i3 << 2;
      eePose[i + i4] = ((d * b[i4] + wrappedJointValueOffset * b[i4 + 1]) +
                        jointRange * b[i4 + 2]) +
                       Ttheta_tmp * b[i4 + 3];
    }
  }
  /*  Parse optional inputs */
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 0, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 1, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 2, false);
  /*  If joint limits are not enforced, set the shifted joint limits to have
   * infinite range */
  covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 0, true);
  covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 0, false);
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 3, false);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 5U);
  /*  Map the desired end effector pose to the pose of the central intersecting
   * joint. */
  /*  Solve for the position of the first three joints from the pose of joint 5
   */
  for (i = 0; i < 4; i++) {
    d = eePose[i];
    wrappedJointValueOffset = eePose[i + 4];
    jointRange = eePose[i + 8];
    Ttheta_tmp = eePose[i + 12];
    for (i3 = 0; i3 < 4; i3++) {
      i4 = i3 << 2;
      a[i + i4] = ((d * b_b[i4] + wrappedJointValueOffset * b_b[i4 + 1]) +
                   jointRange * b_b[i4 + 2]) +
                  Ttheta_tmp * b_b[i4 + 3];
    }
  }
  st.site = &om_emlrtRSI;
  solveFirstThreeDHJoints(&st, &a[12], q123Opts_data, q123Opts_size);
  /*  Solve for the positions of the intersecting axes */
  /*  For each position solution, this configuration of the last three axes
   * produces at least two possible orientation solutions */
  q456Opts_size_idx_0 = q123Opts_size[0] << 1;
  loop_ub_tmp = q456Opts_size_idx_0 * 3;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&q456Opts_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }
  /*  The next step seeks to find the orientation, which is entirely governed by
   * the last three joints. This means that rotation from the fourth joint to
   * the end effector can be mapped to three rotations in-place about the fifth
   * joint. Since the rotations are in-place, they can be defined relative to
   * the fourth joint axes, assuming a fixed pose rotation at the end to align
   * with the end effector. The fixed rotation is found using the DH parameters,
   * and corresponds to the rotation of the end effector relative to the fourth
   * joint when the last three joints are all zero. */
  i = q123Opts_size[0];
  for (jtIdx = 0; jtIdx < i; jtIdx++) {
    real_T jt4ZeroPose[16];
    real_T c_jt4ZeroPose[9];
    real_T d_jt4ZeroPose[9];
    real_T orientationSolns[6];
    real_T dv1[3];
    real_T q123Opts[3];
    covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 6U);
    /*  Get the position of the fourth joint at its zero position when the first
     * three joints are positioned for IK */
    st.site = &pm_emlrtRSI;
    if (jtIdx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, i, &pb_emlrtBCI, &st);
    }
    covrtLogFcn(&emlrtCoverageInstance, 1U, 16U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 82U);
    /* getJoint4PoseFromDH Get the pose of the fourth joint when the first three
     * joints set to q123 and joint4 angle is zero */
    /*  Initialize output */
    eye(jt4ZeroPose);
    for (b_i = 0; b_i < 3; b_i++) {
      real_T b_jt4ZeroPose[16];
      real_T TFixed_tmp;
      real_T b_TFixed_tmp;
      covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 13, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 83U);
      d = q123Opts_data[jtIdx + q123Opts_size[0] * b_i];
      Ttheta_tmp = muDoubleScalarSin(d);
      jointRange = muDoubleScalarCos(d);
      wrappedJointValueOffset = dhParams[b_i + 6];
      TFixed_tmp = muDoubleScalarSin(wrappedJointValueOffset);
      b_TFixed_tmp = muDoubleScalarCos(wrappedJointValueOffset);
      a[0] = jointRange;
      a[4] = -Ttheta_tmp;
      a[8] = 0.0;
      a[12] = 0.0;
      a[1] = Ttheta_tmp;
      a[5] = jointRange;
      a[9] = 0.0;
      a[13] = 0.0;
      a[2] = 0.0;
      a[3] = 0.0;
      a[6] = 0.0;
      a[7] = 0.0;
      a[10] = 1.0;
      a[11] = 0.0;
      a[14] = 0.0;
      a[15] = 1.0;
      for (i3 = 0; i3 < 4; i3++) {
        d = jt4ZeroPose[i3];
        wrappedJointValueOffset = jt4ZeroPose[i3 + 4];
        jointRange = jt4ZeroPose[i3 + 8];
        Ttheta_tmp = jt4ZeroPose[i3 + 12];
        for (i4 = 0; i4 < 4; i4++) {
          b_q456Opts_data_tmp = i4 << 2;
          b_jt4ZeroPose[i3 + b_q456Opts_data_tmp] =
              ((d * a[b_q456Opts_data_tmp] +
                wrappedJointValueOffset * a[b_q456Opts_data_tmp + 1]) +
               jointRange * a[b_q456Opts_data_tmp + 2]) +
              Ttheta_tmp * a[b_q456Opts_data_tmp + 3];
        }
      }
      a[0] = 1.0;
      a[4] = 0.0;
      a[8] = 0.0;
      a[12] = dhParams[b_i];
      a[1] = 0.0;
      a[5] = b_TFixed_tmp;
      a[9] = -TFixed_tmp;
      a[13] = 0.0;
      a[2] = 0.0;
      a[6] = TFixed_tmp;
      a[10] = b_TFixed_tmp;
      a[14] = dhParams[b_i + 12];
      a[3] = 0.0;
      a[7] = 0.0;
      a[11] = 0.0;
      a[15] = 1.0;
      for (i3 = 0; i3 < 4; i3++) {
        d = b_jt4ZeroPose[i3];
        wrappedJointValueOffset = b_jt4ZeroPose[i3 + 4];
        jointRange = b_jt4ZeroPose[i3 + 8];
        Ttheta_tmp = b_jt4ZeroPose[i3 + 12];
        for (i4 = 0; i4 < 4; i4++) {
          b_q456Opts_data_tmp = i4 << 2;
          jt4ZeroPose[i3 + b_q456Opts_data_tmp] =
              ((d * a[b_q456Opts_data_tmp] +
                wrappedJointValueOffset * a[b_q456Opts_data_tmp + 1]) +
               jointRange * a[b_q456Opts_data_tmp + 2]) +
              Ttheta_tmp * a[b_q456Opts_data_tmp + 3];
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&st);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 13, 0);
    /*  Compute the rotation matrix needed to get to the end */
    /*  The orientation of the end effector in the world frame can be written:
     */
    /*     eeRot = jt4ZeroRot*(Rotation about axes 4-6)*eeFixedRotation */
    /*  Then the goal is to solve for the rotation about the axes and relate
     * them to he known form from the DH parameters, if a valid solution exists:
     */
    /*     (Rotation about axes 4-6) = jt4ZeroRot'*eeRot*eeFixedRotation' */
    /*  This orientation produces at least two configurations for every
     * solution, when joint limits allow */
    for (i3 = 0; i3 < 3; i3++) {
      i4 = i3 << 2;
      for (b_q456Opts_data_tmp = 0; b_q456Opts_data_tmp < 3;
           b_q456Opts_data_tmp++) {
        c_q456Opts_data_tmp = b_q456Opts_data_tmp << 2;
        c_jt4ZeroPose[i3 + 3 * b_q456Opts_data_tmp] =
            (jt4ZeroPose[i4] * eePose[c_q456Opts_data_tmp] +
             jt4ZeroPose[i4 + 1] * eePose[c_q456Opts_data_tmp + 1]) +
            jt4ZeroPose[i4 + 2] * eePose[c_q456Opts_data_tmp + 2];
      }
      d = c_jt4ZeroPose[i3];
      wrappedJointValueOffset = c_jt4ZeroPose[i3 + 3];
      jointRange = c_jt4ZeroPose[i3 + 6];
      for (i4 = 0; i4 < 3; i4++) {
        d_jt4ZeroPose[i3 + 3 * i4] =
            (d * dv[3 * i4] + wrappedJointValueOffset * dv[3 * i4 + 1]) +
            jointRange * dv[3 * i4 + 2];
      }
    }
    st.site = &qm_emlrtRSI;
    convertRotationToZYZAxesAngles(&st, d_jt4ZeroPose, orientationSolns);
    if (jtIdx + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, q456Opts_size_idx_0,
                                    &rb_emlrtBCI, (emlrtConstCTX)sp);
    }
    q456Opts_data[jtIdx] = orientationSolns[0];
    c_q456Opts_data_tmp = jtIdx + q456Opts_size_idx_0;
    q456Opts_data[c_q456Opts_data_tmp] = orientationSolns[2];
    b_q456Opts_data_tmp = jtIdx + q456Opts_size_idx_0 * 2;
    q456Opts_data[b_q456Opts_data_tmp] = orientationSolns[4];
    loop_ub_tmp = jtIdx + q123Opts_size[0];
    if (loop_ub_tmp + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(loop_ub_tmp + 1, 1, q456Opts_size_idx_0,
                                    &sb_emlrtBCI, (emlrtConstCTX)sp);
    }
    q456Opts_data[loop_ub_tmp] = orientationSolns[1];
    d_q456Opts_data_tmp = loop_ub_tmp + q456Opts_size_idx_0;
    q456Opts_data[d_q456Opts_data_tmp] = orientationSolns[3];
    q456Opts_data_tmp = loop_ub_tmp + q456Opts_size_idx_0 * 2;
    q456Opts_data[q456Opts_data_tmp] = orientationSolns[5];
    /*  Offset theta to reflect the source robot configuration */
    if (jtIdx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, i, &qb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    q123Opts_data[loop_ub_tmp] -= -1.5707963267949;
    if (jtIdx + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, q456Opts_size_idx_0,
                                    &tb_emlrtBCI, (emlrtConstCTX)sp);
    }
    q456Opts_data[b_q456Opts_data_tmp] -= -1.5707963267949;
    if (loop_ub_tmp + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(loop_ub_tmp + 1, 1, q456Opts_size_idx_0,
                                    &ub_emlrtBCI, (emlrtConstCTX)sp);
    }
    q456Opts_data[q456Opts_data_tmp] -= -1.5707963267949;
    /*  Remove solutions that violate joint limits */
    covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 4, true);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 7U);
    st.site = &rm_emlrtRSI;
    if (jtIdx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, i, &vb_emlrtBCI, &st);
    }
    covrtLogFcn(&emlrtCoverageInstance, 1U, 2U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 11U);
    /* applyJointLimits Convert solutions with invalid joint limits to NaNs */
    /*    Given an N-element configuration, an Nx2 set of lower and upper joint
     */
    /*    limits, and an N-element vector indicating the joint type (revolute or
     */
    /*    prismatic), this function checks whether the configuration is within
     */
    /*    the joint limits. If not, the configuration is converted to NaNs. */
    /*    Copyright 2020-2021 The MathWorks, Inc. */
    /*  Initialize output */
    q123Opts[0] = q123Opts_data[jtIdx];
    q123Opts[1] = q123Opts_data[loop_ub_tmp];
    q123Opts[2] = q123Opts_data[jtIdx + q123Opts_size[0] * 2];
    b_i = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_i < 3) {
        boolean_T guard1;
        covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 1, 1);
        d = q123Opts_data[jtIdx + q123Opts_size[0] * b_i];
        guard1 = false;
        if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 1,
                         jointLimits[b_i] > d) ||
            covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 2,
                         jointLimits[b_i + 3] < d)) {
          covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 1, true);
          covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 6, true);
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 12U);
          /*  Compute the offset from the lower joint limit and compare that to
           */
          /*  the total range */
          wrappedJointValueOffset = wrapTo2Pi(d - jointLimits[b_i]);
          /*  If the wrapped value is 2*pi, make sure it is instead registered
           */
          /*  as zero to ensure this doesn't fall outside the range */
          b_st.site = &xp_emlrtRSI;
          covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
          /* isEqualWithinTolerance Check if two matrices are equal within a set
           * tolerance */
          /*    This is a convenience function designed for inputs with up to
           * two */
          /*    dimensions. If the input has 3+ dimensions, a non-scalar output
           * will be */
          /*    returned. */
          if (covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 7,
                         muDoubleScalarAbs(wrappedJointValueOffset -
                                           6.2831853071795862) < 1.0E-6)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 13U);
            wrappedJointValueOffset = 0.0;
          }
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 14U);
          jointRange = jointLimits[b_i + 3] - jointLimits[b_i];
          covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 3, true);
          if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 4,
                           wrappedJointValueOffset < jointRange)) {
            guard1 = true;
          } else {
            b_st.site = &yp_emlrtRSI;
            covrtLogFcn(&emlrtCoverageInstance, 1U, 17U);
            covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 84U);
            /* isEqualWithinTolerance Check if two matrices are equal within a
             * set tolerance */
            /*    This is a convenience function designed for inputs with up to
             * two */
            /*    dimensions. If the input has 3+ dimensions, a non-scalar
             * output will be */
            /*    returned. */
            if (covrtLogCond(&emlrtCoverageInstance, 1U, 0U, 5,
                             muDoubleScalarAbs(wrappedJointValueOffset -
                                               jointRange) < 1.0E-6)) {
              guard1 = true;
            } else {
              covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 2, false);
              covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 8, false);
              covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 16U);
              /*  If any element is NaN, the whole array will be thrown out so
               */
              /*  there is no need to continue */
              q123Opts[0] = rtNaN;
              q123Opts[1] = rtNaN;
              q123Opts[2] = rtNaN;
              exitg1 = 1;
            }
          }
        } else {
          covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 1, false);
          covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 6, false);
          b_i++;
        }
        if (guard1) {
          covrtLogMcdc(&emlrtCoverageInstance, 1U, 0U, 2, true);
          covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 8, true);
          covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 15U);
          /*  Make sure the final value is definitively inside the joint */
          /*  limits if it was on the bound */
          wrappedJointValueOffset =
              muDoubleScalarMin(wrappedJointValueOffset, jointRange);
          /*  Update the configuration */
          q123Opts[b_i] = jointLimits[b_i] + wrappedJointValueOffset;
          b_i++;
        }
      } else {
        covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 1, 0);
        exitg1 = 1;
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&st);
      }
    } while (exitg1 == 0);
    if (jtIdx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, i, &wb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    q123Opts_data[jtIdx] = q123Opts[0];
    q123Opts_data[jtIdx + q123Opts_size[0]] = q123Opts[1];
    q123Opts_data[jtIdx + q123Opts_size[0] * 2] = q123Opts[2];
    if (jtIdx + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, q456Opts_size_idx_0,
                                    &xb_emlrtBCI, (emlrtConstCTX)sp);
    }
    q123Opts[0] = q456Opts_data[jtIdx];
    q123Opts[1] = q456Opts_data[c_q456Opts_data_tmp];
    q123Opts[2] = q456Opts_data[b_q456Opts_data_tmp];
    st.site = &sm_emlrtRSI;
    applyJointLimits(&st, q123Opts, dv1);
    if (jtIdx + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(jtIdx + 1, 1, q456Opts_size_idx_0,
                                    &yb_emlrtBCI, (emlrtConstCTX)sp);
    }
    q456Opts_data[jtIdx] = dv1[0];
    q456Opts_data[c_q456Opts_data_tmp] = dv1[1];
    q456Opts_data[b_q456Opts_data_tmp] = dv1[2];
    if (loop_ub_tmp + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(loop_ub_tmp + 1, 1, q456Opts_size_idx_0,
                                    &ac_emlrtBCI, (emlrtConstCTX)sp);
    }
    q123Opts[0] = q456Opts_data[loop_ub_tmp];
    q123Opts[1] = q456Opts_data[d_q456Opts_data_tmp];
    q123Opts[2] = q456Opts_data[q456Opts_data_tmp];
    st.site = &tm_emlrtRSI;
    applyJointLimits(&st, q123Opts, dv1);
    if (loop_ub_tmp + 1 > q456Opts_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(loop_ub_tmp + 1, 1, q456Opts_size_idx_0,
                                    &bc_emlrtBCI, (emlrtConstCTX)sp);
    }
    q456Opts_data[loop_ub_tmp] = dv1[0];
    q456Opts_data[d_q456Opts_data_tmp] = dv1[1];
    q456Opts_data[q456Opts_data_tmp] = dv1[2];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 1U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 8U);
  /*  Filter out any remaining rows with NaNs in them by getting the index of
   * the valid rows and only assembling those in the final output */
  st.site = &um_emlrtRSI;
  b_st.site = &um_emlrtRSI;
  repmat(&b_st, q123Opts_data, q123Opts_size, varargin_1_data,
         qOptsWrappedAndRounded_size);
  b_st.site = &pd_emlrtRSI;
  if (qOptsWrappedAndRounded_size[0] != 0) {
    q456Opts_data_tmp = qOptsWrappedAndRounded_size[0];
  } else if (q456Opts_size_idx_0 != 0) {
    q456Opts_data_tmp = q456Opts_size_idx_0;
  } else {
    q456Opts_data_tmp = 0;
  }
  c_st.site = &qd_emlrtRSI;
  if ((qOptsWrappedAndRounded_size[0] != q456Opts_data_tmp) &&
      (qOptsWrappedAndRounded_size[0] != 0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &p_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if ((q456Opts_size_idx_0 != q456Opts_data_tmp) &&
      (q456Opts_size_idx_0 != 0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &p_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  empty_non_axis_sizes = (q456Opts_data_tmp == 0);
  if (empty_non_axis_sizes || (qOptsWrappedAndRounded_size[0] != 0)) {
    input_sizes_idx_1 = 3;
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes || (q456Opts_size_idx_0 != 0)) {
    sizes_idx_1 = 3;
  } else {
    sizes_idx_1 = 0;
  }
  emxInit_real_T(&b_st, &result, 2, &uc_emlrtRTEI);
  i = result->size[0] * result->size[1];
  result->size[0] = q456Opts_data_tmp;
  q456Opts_size_idx_0 = input_sizes_idx_1 + sizes_idx_1;
  result->size[1] = q456Opts_size_idx_0;
  emxEnsureCapacity_real_T(&b_st, result, i, &tc_emlrtRTEI);
  result_data = result->data;
  loop_ub_tmp = input_sizes_idx_1;
  for (i = 0; i < loop_ub_tmp; i++) {
    for (i3 = 0; i3 < q456Opts_data_tmp; i3++) {
      result_data[i3 + result->size[0] * i] =
          varargin_1_data[i3 + q456Opts_data_tmp * i];
    }
  }
  c_q456Opts_data_tmp = sizes_idx_1;
  for (i = 0; i < c_q456Opts_data_tmp; i++) {
    for (i3 = 0; i3 < q456Opts_data_tmp; i3++) {
      result_data[i3 + result->size[0] * (i + input_sizes_idx_1)] =
          q456Opts_data[i3 + q456Opts_data_tmp * i];
    }
  }
  for (i = 0; i < loop_ub_tmp; i++) {
    for (i3 = 0; i3 < q456Opts_data_tmp; i3++) {
      allSolnOpts_data[i3 + q456Opts_data_tmp * i] =
          varargin_1_data[i3 + q456Opts_data_tmp * i];
    }
  }
  for (i = 0; i < c_q456Opts_data_tmp; i++) {
    for (i3 = 0; i3 < q456Opts_data_tmp; i3++) {
      allSolnOpts_data[i3 + q456Opts_data_tmp * (i + input_sizes_idx_1)] =
          q456Opts_data[i3 + q456Opts_data_tmp * i];
    }
  }
  qOptsWrappedAndRounded_size[0] = q456Opts_data_tmp;
  qOptsWrappedAndRounded_size[1] = q456Opts_size_idx_0;
  loop_ub_tmp = q456Opts_data_tmp * q456Opts_size_idx_0;
  for (i = 0; i < loop_ub_tmp; i++) {
    tmp_data[i] = !muDoubleScalarIsNaN(allSolnOpts_data[i]);
  }
  st.site = &vm_emlrtRSI;
  c_q456Opts_data_tmp =
      e_all(&st, tmp_data, qOptsWrappedAndRounded_size, isValidRowIdx_data);
  d_q456Opts_data_tmp = 0;
  b_q456Opts_data_tmp = 0;
  for (b_i = 0; b_i < c_q456Opts_data_tmp; b_i++) {
    if (isValidRowIdx_data[b_i]) {
      d_q456Opts_data_tmp++;
      b_tmp_data[b_q456Opts_data_tmp] = (int8_T)b_i;
      b_q456Opts_data_tmp++;
    }
  }
  for (i = 0; i < d_q456Opts_data_tmp; i++) {
    input_sizes_idx_1 = b_tmp_data[i];
    if (input_sizes_idx_1 > result->size[0] - 1) {
      emlrtDynamicBoundsCheckR2012b(input_sizes_idx_1, 0, result->size[0] - 1,
                                    &ob_emlrtBCI, (emlrtConstCTX)sp);
    }
  }
  /*  Create a copy of the solutions that wraps all revolute joints to pi, then
   * round within solution tolerance. */
  qOptsWrappedAndRounded_size[0] = d_q456Opts_data_tmp;
  for (i = 0; i < q456Opts_size_idx_0; i++) {
    for (i3 = 0; i3 < d_q456Opts_data_tmp; i3++) {
      qOptsWrappedAndRounded_data[i3 + d_q456Opts_data_tmp * i] =
          result_data[b_tmp_data[i3] + result->size[0] * i];
    }
  }
  emxFree_real_T(sp, &result);
  st.site = &wm_emlrtRSI;
  c_wrapToPi(&st, qOptsWrappedAndRounded_data, qOptsWrappedAndRounded_size);
  loop_ub_tmp = qOptsWrappedAndRounded_size[0] * qOptsWrappedAndRounded_size[1];
  c_q456Opts_data_tmp = (loop_ub_tmp / 2) << 1;
  b_q456Opts_data_tmp = c_q456Opts_data_tmp - 2;
  for (i = 0; i <= b_q456Opts_data_tmp; i += 2) {
    r = _mm_loadu_pd(&qOptsWrappedAndRounded_data[i]);
    _mm_storeu_pd(&qOptsWrappedAndRounded_data[i],
                  _mm_mul_pd(r, _mm_set1_pd(1.0E+6)));
  }
  for (i = c_q456Opts_data_tmp; i < loop_ub_tmp; i++) {
    qOptsWrappedAndRounded_data[i] *= 1.0E+6;
  }
  st.site = &wm_emlrtRSI;
  b_round(qOptsWrappedAndRounded_data, qOptsWrappedAndRounded_size);
  loop_ub_tmp = qOptsWrappedAndRounded_size[0] * qOptsWrappedAndRounded_size[1];
  c_q456Opts_data_tmp = (loop_ub_tmp / 2) << 1;
  b_q456Opts_data_tmp = c_q456Opts_data_tmp - 2;
  for (i = 0; i <= b_q456Opts_data_tmp; i += 2) {
    r = _mm_loadu_pd(&qOptsWrappedAndRounded_data[i]);
    _mm_storeu_pd(&qOptsWrappedAndRounded_data[i],
                  _mm_div_pd(r, _mm_set1_pd(1.0E+6)));
  }
  for (i = c_q456Opts_data_tmp; i < loop_ub_tmp; i++) {
    qOptsWrappedAndRounded_data[i] /= 1.0E+6;
  }
  /*  Find the indices of all unique values after wrapping to pi */
  st.site = &xm_emlrtRSI;
  b_st.site = &mq_emlrtRSI;
  b_q456Opts_data_tmp = unique_rows(&b_st, qOptsWrappedAndRounded_data,
                                    qOptsWrappedAndRounded_size, b_data,
                                    q123Opts_size, indx_data);
  /*  Select only unique solutions from the original set of solutions */
  st.site = &ym_emlrtRSI;
  loop_ub_tmp = b_q456Opts_data_tmp;
  for (i = 0; i < b_q456Opts_data_tmp; i++) {
    dist_data[i] = indx_data[i];
  }
  b_st.site = &gr_emlrtRSI;
  b_sort(&b_st, dist_data, &loop_ub_tmp);
  qOptsWrappedAndRounded_size[0] = loop_ub_tmp;
  qOptsWrappedAndRounded_size[1] = q456Opts_size_idx_0;
  for (i = 0; i < q456Opts_size_idx_0; i++) {
    for (i3 = 0; i3 < loop_ub_tmp; i3++) {
      i4 = (int32_T)dist_data[i3];
      if ((i4 < 1) || (i4 > d_q456Opts_data_tmp)) {
        emlrtDynamicBoundsCheckR2012b(i4, 1, d_q456Opts_data_tmp, &nb_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      qOptsWrappedAndRounded_data[i3 + loop_ub_tmp * i] =
          allSolnOpts_data[b_tmp_data[i4 - 1] + q456Opts_data_tmp * i];
    }
  }
  /*  Sort results using a distance metric */
  covrtLogIf(&emlrtCoverageInstance, 1U, 0U, 5, true);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 9U);
  st.site = &an_emlrtRSI;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 1U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 10U);
  /*  Helper functions */
  /* sortByEuclideanDistance Sort a matrix of solution configurations relative
   * to a reference configuration by Euclidean norm */
  /*    This function sorts a matrix of configurations using a pre-defined */
  /*    distance metric. The computed distance between any state and the */
  /*    reference state, referenceConfig, is a Euclidean norm of difference */
  /*    between a revolute joint's values which is then wrapped to [-pi, pi], */
  /*    and a displacement between a prismatic joint's values. */
  /*    Copyright 2020 The MathWorks, Inc. */
  /*  Compute the distances between each configuration and the reference */
  b_st.site = &bs_emlrtRSI;
  c_q456Opts_data_tmp = RigidBodyTreeUtils_distance(
      &b_st, referenceConfig, qOptsWrappedAndRounded_data,
      qOptsWrappedAndRounded_size, dist_data);
  /* , */
  /*  Sort the outputs */
  b_st.site = &cs_emlrtRSI;
  c_st.site = &hs_emlrtRSI;
  b_q456Opts_data_tmp =
      c_sort(&c_st, dist_data, &c_q456Opts_data_tmp, indx_data);
  qOpts_size[0] = b_q456Opts_data_tmp;
  qOpts_size[1] = q456Opts_size_idx_0;
  for (i = 0; i < q456Opts_size_idx_0; i++) {
    for (i3 = 0; i3 < b_q456Opts_data_tmp; i3++) {
      i4 = indx_data[i3];
      if ((i4 < 1) || (i4 > loop_ub_tmp)) {
        emlrtDynamicBoundsCheckR2012b(i4, 1, loop_ub_tmp, &mb_emlrtBCI, &st);
      }
      qOpts_data[i3 + b_q456Opts_data_tmp * i] =
          qOptsWrappedAndRounded_data[(i4 + loop_ub_tmp * i) - 1];
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (ikine_myRobot.c) */
