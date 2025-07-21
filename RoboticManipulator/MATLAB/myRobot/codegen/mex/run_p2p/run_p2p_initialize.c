/*
 * run_p2p_initialize.c
 *
 * Code generation for function 'run_p2p_initialize'
 *
 */

/* Include files */
#include "run_p2p_initialize.h"
#include "_coder_run_p2p_mex.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Declarations */
static void run_p2p_once(void);

/* Function Definitions */
static void run_p2p_once(void)
{
  int32_T postfix_exprs_1_5[6] = {0, 1, -3, 2, -1, -3};
  int32_T caseExprEnds_0_0[5] = {2216, 1267, 1560, 1853, 2079};
  int32_T caseStarts_0_0[5] = {2207, 1238, 1531, 1824, 2052};
  int32_T postfix_exprs_1_2[5] = {0, 1, 2, -2, -3};
  int32_T postfix_exprs_1_4[5] = {0, 1, -3, 2, -3};
  int32_T caseExprEnds_0_1[4] = {2401, 2328, 2355, 2383};
  int32_T caseStarts_0_1[4] = {2392, 2310, 2337, 2364};
  int32_T cond_ends_1_2[3] = {8022, 8064, 8128};
  int32_T cond_ends_1_4[3] = {29782, 29813, 29844};
  int32_T cond_ends_1_5[3] = {29933, 29964, 29996};
  int32_T cond_starts_1_2[3] = {8004, 8028, 8069};
  int32_T cond_starts_1_4[3] = {29755, 29786, 29817};
  int32_T cond_starts_1_5[3] = {29906, 29937, 29969};
  int32_T postfix_exprs_1_1[3] = {0, 1, -2};
  int32_T postfix_exprs_1_6[3] = {0, 1, -2};
  int32_T cond_ends_1_1[2] = {7356, 7393};
  int32_T cond_ends_1_6[2] = {30182, 30218};
  int32_T cond_starts_1_1[2] = {7323, 7360};
  int32_T cond_starts_1_6[2] = {30172, 30186};
  int32_T postfix_exprs_1_0[2] = {0, -1};
  int32_T postfix_exprs_1_3[2] = {0, -1};
  int32_T postfix_exprs_1_7[2] = {0, -1};
  int32_T cond_ends_1_0 = 1540;
  int32_T cond_ends_1_3 = 18740;
  int32_T cond_ends_1_7 = 33387;
  int32_T cond_starts_1_0 = 1522;
  int32_T cond_starts_1_3 = 18650;
  int32_T cond_starts_1_7 = 33346;
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "D:\\6R_robotic_arm\\myRobot\\run_p2p.m", 0U, 1U, 16U, 6U, 0U,
                  0U, 2U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "run_p2p", 0, -1, 3397);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 15U, 3361, -1, 3385);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 14U, 3301, -1, 3340);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 13U, 2713, -1, 3230);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 12U, 2473, -1, 2675);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 10U, 2092, -1, 2198);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 9U, 1866, -1, 2043);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 8U, 1573, -1, 1815);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 7U, 1280, -1, 1522);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 11U, 2229, -1, 2271);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 6U, 804, -1, 1206);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 5U, 696, -1, 723);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 4U, 626, -1, 661);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 3U, 565, -1, 591);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 2U, 502, -1, 530);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 440, -1, 467);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 165, -1, 413);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 418, 431, -1, 475);
  covrtIfInit(&emlrtCoverageInstance, 0U, 1U, 480, 493, -1, 538);
  covrtIfInit(&emlrtCoverageInstance, 0U, 2U, 543, 556, -1, 599);
  covrtIfInit(&emlrtCoverageInstance, 0U, 3U, 604, 617, -1, 669);
  covrtIfInit(&emlrtCoverageInstance, 0U, 4U, 674, 687, -1, 731);
  covrtIfInit(&emlrtCoverageInstance, 0U, 5U, 3239, 3288, -1, 3352);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 2685, 2704, 3393);
  /* Initialize While Information */
  /* Initialize Switch Information */
  covrtSwitchInit(&emlrtCoverageInstance, 0U, 0U, 1211, 1229, 2279, 5U,
                  caseStarts_0_0, caseExprEnds_0_0);
  covrtSwitchInit(&emlrtCoverageInstance, 0U, 1U, 2284, 2301, 2409, 4U,
                  caseStarts_0_1, caseExprEnds_0_1);
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "D:\\6R_robotic_arm\\myRobot\\ikine_myRobot.m", 1U, 18U, 85U,
                  34U, 0U, 0U, 0U, 14U, 0U, 16U, 8U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 1U, 0U, "ikine_myRobot", 0, -1, 19058);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 5U, "solveFirstThreeDHJoints", 19060,
               -1, 54341);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 16U, "getJoint4PoseFromDH", 54343,
               -1, 55038);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 17U, "isEqualWithinTolerance", 55040,
               -1, 55421);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 1U, "sortByEuclideanDistance", 5672,
               -1, 6662);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 2U, "applyJointLimits", 6668, -1,
               8778);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 3U, "convertRotationToZYZAxesAngles",
               8784, -1, 12415);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 4U, "distributeRotationOverJoints",
               12421, -1, 19053);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 6U, "computef13SupportingEquations",
               26394, -1, 27533);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 7U, "computeF14SupportingEquations",
               27540, -1, 28477);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 8U, "computeg12SupportingEquations",
               28483, -1, 29253);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 9U, "solveTrigEquations", 29260, -1,
               30499);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 10U, "chooseCorrectSolution", 30505,
               -1, 32810);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 11U, "replaceImagWithNaN", 32816, -1,
               33474);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 12U, "solveForHGeneralCase", 33480,
               -1, 48810);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 13U, "getQuarticPolynomialCoeffs",
               36712, -1, 48802);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 14U, "solveQuarticPolynomial", 48816,
               -1, 54336);
  covrtFcnInit(&emlrtCoverageInstance, 1U, 15U, "solveCubicPolynomial", 52935,
               -1, 54328);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 9U, 5566, -1, 5643);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 8U, 4892, -1, 5504);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 7U, 4392, -1, 4742);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 6U, 3133, -1, 4307);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 5U, 1698, -1, 2983);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 4U, 1545, -1, 1610);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 3U, 1398, -1, 1424);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 2U, 1352, -1, 1375);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 1U, 1300, -1, 1329);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 0U, 297, -1, 1281);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 41U, 26315, -1, 26370);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 40U, 25418, -1, 26162);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 39U, 25274, -1, 25332);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 38U, 24792, -1, 25074);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 37U, 23345, -1, 24398);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 36U, 23188, -1, 23246);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 35U, 23088, -1, 23141);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 34U, 22938, -1, 22973);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 33U, 22825, -1, 22908);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 32U, 22668, -1, 22729);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 31U, 22559, -1, 22597);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 30U, 21060, -1, 22475);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 83U, 54725, -1, 55029);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 82U, 54508, -1, 54708);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 84U, 55334, -1, 55416);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 10U, 6417, -1, 6653);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 15U, 8284, -1, 8480);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 16U, 8656, -1, 8721);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 14U, 7934, -1, 7983);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 13U, 7868, -1, 7896);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 12U, 7529, -1, 7618);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 11U, 7243, -1, 7269);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 19U, 12003, -1, 12395);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 18U, 10823, -1, 11796);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 17U, 10049, -1, 10512);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 25U, 17203, -1, 17330);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 24U, 16978, -1, 17133);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 23U, 16689, -1, 16798);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 22U, 16518, -1, 16617);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 28U, 18757, -1, 18818);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 29U, 19002, -1, 19044);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 27U, 18241, -1, 18455);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 26U, 17904, -1, 18173);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 21U, 15718, -1, 15828);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 20U, 14352, -1, 15504);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 42U, 27134, -1, 27524);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 43U, 28181, -1, 28468);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 44U, 28828, -1, 29244);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 46U, 29857, -1, 29890);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 47U, 30009, -1, 30016);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 49U, 30308, -1, 30478);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 48U, 30118, -1, 30160);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 45U, 29692, -1, 29709);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 55U, 32761, -1, 32801);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 54U, 32545, -1, 32587);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 53U, 32391, -1, 32495);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 52U, 32303, -1, 32330);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 51U, 31675, -1, 31864);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 50U, 31500, -1, 31571);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 58U, 33434, -1, 33453);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 57U, 33400, -1, 33408);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 56U, 33321, -1, 33329);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 61U, 36022, -1, 36663);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 60U, 35612, -1, 35690);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 59U, 34954, -1, 35118);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 66U, 48267, -1, 48774);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 65U, 48029, -1, 48207);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 64U, 46007, -1, 47969);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 63U, 45741, -1, 45947);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 62U, 37220, -1, 45697);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 77U, 52703, -1, 52912);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 76U, 52420, -1, 52631);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 75U, 52203, -1, 52386);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 74U, 52081, -1, 52141);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 73U, 51776, -1, 51839);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 72U, 51213, -1, 51687);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 71U, 50761, -1, 50881);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 70U, 50270, -1, 50525);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 69U, 50100, -1, 50141);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 68U, 49956, -1, 49970);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 67U, 49520, -1, 49833);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 81U, 54055, -1, 54316);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 80U, 53920, -1, 54025);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 79U, 53648, -1, 53761);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 78U, 53345, -1, 53464);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 1U, 0U, 1282, 1295, -1, 1333);
  covrtIfInit(&emlrtCoverageInstance, 1U, 1U, 1334, 1347, -1, 1379);
  covrtIfInit(&emlrtCoverageInstance, 1U, 2U, 1380, 1393, -1, 1428);
  covrtIfInit(&emlrtCoverageInstance, 1U, 3U, 1518, 1540, -1, 1614);
  covrtIfInit(&emlrtCoverageInstance, 1U, 4U, 4362, 4383, -1, 4750);
  covrtIfInit(&emlrtCoverageInstance, 1U, 5U, 5544, 5561, -1, 5647);
  covrtIfInit(&emlrtCoverageInstance, 1U, 12U, 22602, 22614, 22734, 22916);
  covrtIfInit(&emlrtCoverageInstance, 1U, 13U, 22921, 22933, -1, 22977);
  covrtIfInit(&emlrtCoverageInstance, 1U, 14U, 23146, 23179, 24681, 26171);
  covrtIfInit(&emlrtCoverageInstance, 1U, 15U, 25230, 25261, 26170, 26171);
  covrtIfInit(&emlrtCoverageInstance, 1U, 6U, 7320, 7393, -1, 8757);
  covrtIfInit(&emlrtCoverageInstance, 1U, 7U, 7791, 7847, -1, 7916);
  covrtIfInit(&emlrtCoverageInstance, 1U, 8U, 8001, 8129, 8497, 8741);
  covrtIfInit(&emlrtCoverageInstance, 1U, 9U, 10767, 10809, 11805, 12407);
  covrtIfInit(&emlrtCoverageInstance, 1U, 10U, 16158, 16185, 17356, 18846);
  covrtIfInit(&emlrtCoverageInstance, 1U, 11U, 18646, 18740, 18845, 18846);
  covrtIfInit(&emlrtCoverageInstance, 1U, 16U, 29752, 29844, 29899, 30017);
  covrtIfInit(&emlrtCoverageInstance, 1U, 17U, 29899, 29996, 30169, 30490);
  covrtIfInit(&emlrtCoverageInstance, 1U, 18U, 30169, 30218, -1, 30490);
  covrtIfInit(&emlrtCoverageInstance, 1U, 19U, 32512, 32524, -1, 32607);
  covrtIfInit(&emlrtCoverageInstance, 1U, 20U, 33288, 33308, 33338, 33465);
  covrtIfInit(&emlrtCoverageInstance, 1U, 21U, 33338, 33387, 33417, 33465);
  covrtIfInit(&emlrtCoverageInstance, 1U, 22U, 35178, 35229, 35699, 36675);
  covrtIfInit(&emlrtCoverageInstance, 1U, 23U, 45710, 45724, -1, 45963);
  covrtIfInit(&emlrtCoverageInstance, 1U, 24U, 45976, 45990, -1, 47985);
  covrtIfInit(&emlrtCoverageInstance, 1U, 25U, 47998, 48012, -1, 48223);
  covrtIfInit(&emlrtCoverageInstance, 1U, 26U, 48236, 48250, -1, 48790);
  covrtIfInit(&emlrtCoverageInstance, 1U, 27U, 49843, 49862, 49979, 52924);
  covrtIfInit(&emlrtCoverageInstance, 1U, 28U, 49979, 50011, 50150, 52924);
  covrtIfInit(&emlrtCoverageInstance, 1U, 29U, 50150, 50180, 50534, 52924);
  covrtIfInit(&emlrtCoverageInstance, 1U, 30U, 50534, 50555, 50890, 52924);
  covrtIfInit(&emlrtCoverageInstance, 1U, 31U, 51728, 51755, 51874, 51875);
  covrtIfInit(&emlrtCoverageInstance, 1U, 32U, 52155, 52186, 52399, 52647);
  covrtIfInit(&emlrtCoverageInstance, 1U, 33U, 53618, 53631, 53774, 54041);
  /* Initialize MCDC Information */
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 0U, 1521, 1540, 1, 0,
                &cond_starts_1_0, &cond_ends_1_0, 2, postfix_exprs_1_0);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 1U, 7323, 7393, 2, 1,
                cond_starts_1_1, cond_ends_1_1, 3, postfix_exprs_1_1);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 2U, 8004, 8129, 3, 3,
                cond_starts_1_2, cond_ends_1_2, 5, postfix_exprs_1_2);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 3U, 18649, 18740, 1, 6,
                &cond_starts_1_3, &cond_ends_1_3, 2, postfix_exprs_1_3);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 4U, 29755, 29844, 3, 7,
                cond_starts_1_4, cond_ends_1_4, 5, postfix_exprs_1_4);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 5U, 29906, 29996, 3, 10,
                cond_starts_1_5, cond_ends_1_5, 6, postfix_exprs_1_5);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 6U, 30172, 30218, 2, 13,
                cond_starts_1_6, cond_ends_1_6, 3, postfix_exprs_1_6);
  covrtMcdcInit(&emlrtCoverageInstance, 1U, 7U, 33345, 33387, 1, 15,
                &cond_starts_1_7, &cond_ends_1_7, 2, postfix_exprs_1_7);
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 1U, 0U, 2984, 3014, 4755);
  covrtForInit(&emlrtCoverageInstance, 1U, 6U, 22476, 22502, 22920);
  covrtForInit(&emlrtCoverageInstance, 1U, 7U, 22978, 22997, 26176);
  covrtForInit(&emlrtCoverageInstance, 1U, 8U, 24681, 24700, 26171);
  covrtForInit(&emlrtCoverageInstance, 1U, 13U, 54709, 54720, 55033);
  covrtForInit(&emlrtCoverageInstance, 1U, 1U, 7279, 7307, 8769);
  covrtForInit(&emlrtCoverageInstance, 1U, 2U, 15513, 15546, 15840);
  covrtForInit(&emlrtCoverageInstance, 1U, 3U, 16630, 16672, 16814);
  covrtForInit(&emlrtCoverageInstance, 1U, 4U, 17146, 17186, 17346);
  covrtForInit(&emlrtCoverageInstance, 1U, 5U, 18186, 18224, 18471);
  covrtForInit(&emlrtCoverageInstance, 1U, 9U, 31580, 31591, 31876);
  covrtForInit(&emlrtCoverageInstance, 1U, 10U, 32339, 32350, 32635);
  covrtForInit(&emlrtCoverageInstance, 1U, 11U, 32363, 32374, 32623);
  covrtForInit(&emlrtCoverageInstance, 1U, 12U, 51700, 51711, 51875);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 1U);
}

void run_p2p_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtLicenseCheckR2022a(&st, "EMLRT:runTime:MexFunctionNeedsLicense",
                          "robotics_system_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    run_p2p_once();
  }
}

/* End of code generation (run_p2p_initialize.c) */
