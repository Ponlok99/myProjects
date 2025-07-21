/*
 * computePolyCoefAndTimeOfArrival.c
 *
 * Code generation for function 'computePolyCoefAndTimeOfArrival'
 *
 */

/* Include files */
#include "computePolyCoefAndTimeOfArrival.h"
#include "constructM.h"
#include "inv.h"
#include "mldivide.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "solvePoly.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo ld_emlrtRSI = {
    52,                                /* lineNo */
    "computePolyCoefAndTimeOfArrival", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\computePolyCoefAndTimeOf"
    "Arrival.m" /* pathName */
};

static emlrtRSInfo ud_emlrtRSI = {
    69,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

static emlrtRSInfo vd_emlrtRSI = {
    70,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

static emlrtBCInfo bb_emlrtBCI = {
    1,           /* iFirst */
    10,          /* iLast */
    74,          /* lineNo */
    22,          /* colNo */
    "",          /* aName */
    "solvePoly", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m", /* pName */
    0                                     /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = {
    1,           /* iFirst */
    10,          /* iLast */
    74,          /* lineNo */
    15,          /* colNo */
    "",          /* aName */
    "solvePoly", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m", /* pName */
    0                                     /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = {
    1,           /* iFirst */
    10,          /* iLast */
    73,          /* lineNo */
    13,          /* colNo */
    "",          /* aName */
    "solvePoly", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m", /* pName */
    0                                     /* checkKind */
};

static emlrtRTEInfo jc_emlrtRTEI = {
    57,                                /* lineNo */
    9,                                 /* colNo */
    "computePolyCoefAndTimeOfArrival", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\computePolyCoefAndTimeOf"
    "Arrival.m" /* pName */
};

static emlrtRTEInfo kc_emlrtRTEI = {
    27,         /* lineNo */
    5,          /* colNo */
    "optimize", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\optimize.m" /* pName */
};

/* Function Definitions */
void b_computePolyCoefAndTimeOfArriv(const emlrtStack *sp,
                                     const real_T constraints[30],
                                     real_T pp[30],
                                     emxArray_real_T *timeOfArrival)
{
  static const real_T b_AInv[100] = {1.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     -126.00000000005578,
                                     420.00000000019594,
                                     -540.00000000026034,
                                     315.0000000001553,
                                     -70.000000000035072,
                                     0.0,
                                     1.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     -70.000000000028734,
                                     224.00000000010124,
                                     -280.00000000013472,
                                     160.00000000008041,
                                     -35.000000000018161,
                                     0.0,
                                     0.0,
                                     0.5,
                                     0.0,
                                     0.0,
                                     -17.500000000006349,
                                     52.500000000022553,
                                     -63.000000000030141,
                                     35.000000000018019,
                                     -7.5000000000040723,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.16666666666666666,
                                     0.0,
                                     -2.5000000000007154,
                                     6.6666666666692667,
                                     -7.5000000000035358,
                                     4.000000000002129,
                                     -0.83333333333381487,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.041666666666666664,
                                     -0.20833333333336226,
                                     0.41666666666678787,
                                     -0.41666666666685243,
                                     0.20833333333344822,
                                     -0.041666666666693275,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     126.00000000005578,
                                     -420.00000000019594,
                                     540.00000000026034,
                                     -315.0000000001553,
                                     70.000000000035072,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     -56.000000000027043,
                                     196.0000000000947,
                                     -260.00000000012562,
                                     155.00000000007489,
                                     -35.000000000016911,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     10.500000000005503,
                                     -38.500000000019284,
                                     53.000000000025594,
                                     -32.500000000015262,
                                     7.500000000003447,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     -1.0000000000005647,
                                     3.833333333335323,
                                     -5.5000000000026406,
                                     3.5000000000015747,
                                     -0.83333333333368909,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.041666666666691769,
                                     -0.16666666666675478,
                                     0.25000000000011707,
                                     -0.16666666666673657,
                                     0.041666666666682471};
  static const real_T b_b[100] = {
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      0.0,
      0.0,     0.0,      0.0,      0.0,      576.0,
      1440.0,  2880.0,   5040.0,   8064.0,   12096.0,
      0.0,     0.0,      0.0,      0.0,      1440.0,
      4800.0,  10800.0,  20160.0,  33600.0,  51840.0,
      0.0,     0.0,      0.0,      0.0,      2880.0,
      10800.0, 25920.0,  50400.0,  86400.0,  136080.0,
      0.0,     0.0,      0.0,      0.0,      5040.0,
      20160.0, 50400.0,  100800.0, 176400.0, 282240.0,
      0.0,     0.0,      0.0,      0.0,      8064.0,
      33600.0, 86400.0,  176400.0, 313600.0, 508032.0,
      0.0,     0.0,      0.0,      0.0,      12096.0,
      51840.0, 136080.0, 282240.0, 508032.0, 831325.09090909094};
  static const real_T b[25] = {1.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               1.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.5,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.16666666666666666,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.041666666666666664};
  static const real_T dv[25] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 24.0};
  static const real_T dv1[25] = {
      1.0,   5.0,    20.0, 60.0, 120.0, 1.0,   6.0,   30.0, 120.0,
      360.0, 1.0,    7.0,  42.0, 210.0, 840.0, 1.0,   8.0,  56.0,
      336.0, 1680.0, 1.0,  9.0,  72.0,  504.0, 3024.0};
  static const real_T lowerright[25] = {
      126.00000000005578,   -420.00000000019594,  540.00000000026034,
      -315.0000000001553,   70.000000000035072,   -56.000000000027043,
      196.0000000000947,    -260.00000000012562,  155.00000000007489,
      -35.000000000016911,  10.500000000005503,   -38.500000000019284,
      53.000000000025594,   -32.500000000015262,  7.500000000003447,
      -1.0000000000005647,  3.833333333335323,    -5.5000000000026406,
      3.5000000000015747,   -0.83333333333368909, 0.041666666666691769,
      -0.16666666666675478, 0.25000000000011707,  -0.16666666666673657,
      0.041666666666682471};
  c_robotics_core_internal_Damped solver;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T b_DP_data;
  emxArray_real_T b_QPrime_data;
  emxArray_real_T *y;
  real_T QPrime[100];
  real_T QPrime_data[100];
  real_T DP_data[10];
  real_T *timeOfArrival_data;
  int32_T QPrime_size[2];
  int32_T DP_size;
  int32_T dimIdx;
  int32_T i;
  int32_T i1;
  int32_T i3;
  int32_T i4;
  int32_T i5;
  int32_T k;
  int32_T nz;
  int8_T tmp_data[10];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &y, 1, &qc_emlrtRTEI);
  d_emxInitStruct_robotics_core_i(sp, &solver, &kc_emlrtRTEI);
  for (dimIdx = 0; dimIdx < 3; dimIdx++) {
    real_T M[100];
    real_T D_data[20];
    real_T AInv[10];
    real_T b_M[10];
    real_T d;
    int32_T i2;
    int32_T innerDimA;
    int32_T loop_ub;
    int32_T trueCount;
    boolean_T numConstraints_tmp[10];
    st.site = &ld_emlrtRSI;
    for (i = 0; i < 10; i++) {
      numConstraints_tmp[i] =
          !muDoubleScalarIsNaN(constraints[i + 10 * dimIdx]);
    }
    nz = numConstraints_tmp[0];
    for (k = 0; k < 9; k++) {
      nz += numConstraints_tmp[k + 1];
    }
    real_T b_y[100];
    b_st.site = &rd_emlrtRSI;
    b_constructM(&b_st, &constraints[10 * dimIdx], M);
    b_st.site = &sd_emlrtRSI;
    c_st.site = &ne_emlrtRSI;
    b_checkcond(&c_st, dv, b);
    b_st.site = &td_emlrtRSI;
    c_st.site = &ne_emlrtRSI;
    b_checkcond(&c_st, dv1, lowerright);
    b_st.site = &ud_emlrtRSI;
    c_mtimes(b_AInv, b_b, b_y);
    b_st.site = &ud_emlrtRSI;
    b_mtimes(b_y, b_AInv, QPrime);
    b_st.site = &vd_emlrtRSI;
    b_mtimes(M, QPrime, b_y);
    b_st.site = &vd_emlrtRSI;
    d_mtimes(b_y, M, QPrime);
    if ((real_T)nz + 1.0 > 10.0) {
      i = 0;
      i1 = 0;
      i2 = 0;
      i3 = 0;
    } else {
      if ((nz + 1 < 1) || (nz + 1 > 10)) {
        emlrtDynamicBoundsCheckR2012b(nz + 1, 1, 10, &db_emlrtBCI, &st);
      }
      i = nz;
      i1 = 10;
      i2 = nz;
      i3 = 10;
    }
    if (nz < 1) {
      innerDimA = 0;
    } else {
      if (nz > 10) {
        emlrtDynamicBoundsCheckR2012b(nz, 1, 10, &cb_emlrtBCI, &st);
      }
      innerDimA = nz;
    }
    if (nz + 1 > 10) {
      i4 = -1;
      i5 = -1;
    } else {
      if ((nz + 1 < 1) || (nz + 1 > 10)) {
        emlrtDynamicBoundsCheckR2012b(nz + 1, 1, 10, &bb_emlrtBCI, &st);
      }
      i4 = nz - 1;
      i5 = 9;
    }
    trueCount = 0;
    nz = 0;
    for (k = 0; k < 10; k++) {
      if (numConstraints_tmp[k]) {
        trueCount++;
        tmp_data[nz] = (int8_T)k;
        nz++;
      }
    }
    b_st.site = &wd_emlrtRSI;
    c_st.site = &ke_emlrtRSI;
    if (innerDimA != trueCount) {
      if (((innerDimA == 1) && (i5 - i4 == 1)) || (trueCount == 1)) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &s_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    QPrime_size[0] = innerDimA;
    loop_ub = i5 - i4;
    QPrime_size[1] = loop_ub;
    for (i5 = 0; i5 < loop_ub; i5++) {
      for (nz = 0; nz < innerDimA; nz++) {
        QPrime_data[nz + QPrime_size[0] * i5] =
            QPrime[nz + 10 * ((i4 + i5) + 1)];
      }
    }
    DP_size = trueCount;
    for (i4 = 0; i4 < trueCount; i4++) {
      DP_data[i4] = constraints[tmp_data[i4] + 10 * dimIdx];
    }
    b_QPrime_data.data = &QPrime_data[0];
    b_QPrime_data.size = &QPrime_size[0];
    b_QPrime_data.allocatedSize = 100;
    b_QPrime_data.numDimensions = 2;
    b_QPrime_data.canFreeData = false;
    b_DP_data.data = &DP_data[0];
    b_DP_data.size = &DP_size;
    b_DP_data.allocatedSize = 10;
    b_DP_data.numDimensions = 1;
    b_DP_data.canFreeData = false;
    c_st.site = &je_emlrtRSI;
    mtimes(&c_st, &b_QPrime_data, &b_DP_data, y);
    loop_ub = i1 - i;
    QPrime_size[0] = loop_ub;
    nz = i3 - i2;
    QPrime_size[1] = nz;
    for (i1 = 0; i1 < nz; i1++) {
      k = (loop_ub / 2) << 1;
      innerDimA = k - 2;
      for (i3 = 0; i3 <= innerDimA; i3 += 2) {
        __m128d r;
        r = _mm_loadu_pd(&QPrime[(i + i3) + 10 * (i2 + i1)]);
        _mm_storeu_pd(&QPrime_data[i3 + QPrime_size[0] * i1],
                      _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (i3 = k; i3 < loop_ub; i3++) {
        QPrime_data[i3 + QPrime_size[0] * i1] =
            -QPrime[(i + i3) + 10 * (i2 + i1)];
      }
    }
    b_st.site = &wd_emlrtRSI;
    DP_size = mldivide(&b_st, QPrime_data, QPrime_size, (real_T *)y->data,
                       (*(int32_T(*)[1])y->size)[0], DP_data);
    nz = trueCount + DP_size;
    for (i = 0; i < trueCount; i++) {
      D_data[i] = constraints[tmp_data[i] + 10 * dimIdx];
    }
    loop_ub = DP_size;
    for (i = 0; i < loop_ub; i++) {
      D_data[i + trueCount] = DP_data[i];
    }
    b_st.site = &xd_emlrtRSI;
    c_st.site = &ke_emlrtRSI;
    if (nz != 10) {
      if (nz == 1) {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &s_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    for (i = 0; i < 10; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 10; i1++) {
        d += M[i1 + 10 * i] * D_data[i1];
      }
      b_M[i] = d;
    }
    for (i = 0; i < 10; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 10; i1++) {
        d += b_AInv[i + 10 * i1] * b_M[i1];
      }
      AInv[i] = d;
    }
    for (i = 0; i < 10; i++) {
      pp[i + 10 * dimIdx] = AInv[9 - i];
    }
    b_st.site = &yd_emlrtRSI;
    b_st.site = &yd_emlrtRSI;
  }
  i = timeOfArrival->size[0] * timeOfArrival->size[1];
  timeOfArrival->size[0] = 1;
  timeOfArrival->size[1] = 2;
  emxEnsureCapacity_real_T(sp, timeOfArrival, i, &jc_emlrtRTEI);
  timeOfArrival_data = timeOfArrival->data;
  timeOfArrival_data[0] = 0.0;
  timeOfArrival_data[1] = 1.0;
  emxFree_real_T(sp, &y);
  d_emxFreeStruct_robotics_core_i(sp, &solver);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_computePolyCoefAndTimeOfArriv(const emlrtStack *sp,
                                     const real_T constraints[8],
                                     emxArray_real_T *timeOfArrival,
                                     real_T pp[8])
{
  c_robotics_core_internal_Damped solver;
  emlrtStack st;
  real_T *timeOfArrival_data;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  e_emxInitStruct_robotics_core_i(sp, &solver, &kc_emlrtRTEI);
  st.site = &ld_emlrtRSI;
  solvePoly(&st, constraints, pp);
  i = timeOfArrival->size[0] * timeOfArrival->size[1];
  timeOfArrival->size[0] = 1;
  timeOfArrival->size[1] = 2;
  emxEnsureCapacity_real_T(sp, timeOfArrival, i, &jc_emlrtRTEI);
  timeOfArrival_data = timeOfArrival->data;
  timeOfArrival_data[0] = 0.0;
  timeOfArrival_data[1] = 1.0;
  e_emxFreeStruct_robotics_core_i(sp, &solver);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void computePolyCoefAndTimeOfArrival(const emlrtStack *sp,
                                     const real_T constraints[24],
                                     real_T pp[24],
                                     emxArray_real_T *timeOfArrival)
{
  c_robotics_core_internal_Damped solver;
  emlrtStack st;
  real_T *timeOfArrival_data;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  c_emxInitStruct_robotics_core_i(sp, &solver, &kc_emlrtRTEI);
  st.site = &ld_emlrtRSI;
  solvePoly(&st, &constraints[0], &pp[0]);
  st.site = &ld_emlrtRSI;
  solvePoly(&st, &constraints[8], &pp[8]);
  st.site = &ld_emlrtRSI;
  solvePoly(&st, &constraints[16], &pp[16]);
  i = timeOfArrival->size[0] * timeOfArrival->size[1];
  timeOfArrival->size[0] = 1;
  timeOfArrival->size[1] = 2;
  emxEnsureCapacity_real_T(sp, timeOfArrival, i, &jc_emlrtRTEI);
  timeOfArrival_data = timeOfArrival->data;
  timeOfArrival_data[0] = 0.0;
  timeOfArrival_data[1] = 1.0;
  c_emxFreeStruct_robotics_core_i(sp, &solver);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (computePolyCoefAndTimeOfArrival.c) */
