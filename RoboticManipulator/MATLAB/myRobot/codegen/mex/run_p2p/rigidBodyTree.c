/*
 * rigidBodyTree.c
 *
 * Code generation for function 'rigidBodyTree'
 *
 */

/* Include files */
#include "rigidBodyTree.h"
#include "RigidBodyTree1.h"
#include "RigidBodyTreeDynamics.h"
#include "any.h"
#include "mtimes.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_internal_types.h"
#include "run_p2p_types.h"
#include "validateattributes.h"
#include "warning.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo
    is_emlrtRSI =
        {
            743,                               /* lineNo */
            "rigidBodyTree/geometricJacobian", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyTre"
            "e.m" /* pathName */
};

static emlrtRSInfo js_emlrtRSI = {
    1102,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ks_emlrtRSI = {
    1104,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ls_emlrtRSI = {
    1106,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ms_emlrtRSI = {
    1110,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ns_emlrtRSI = {
    1114,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo os_emlrtRSI = {
    1133,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ps_emlrtRSI = {
    1138,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo qs_emlrtRSI = {
    1146,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo rs_emlrtRSI = {
    1145,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ss_emlrtRSI = {
    1152,                              /* lineNo */
    "RigidBodyTree/geometricJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo ts_emlrtRSI = {
    1675,                                  /* lineNo */
    "RigidBodyTree/validateConfiguration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo nt_emlrtRSI = {
    464,                  /* lineNo */
    "RigidBody/get.Name", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBody.m" /* pathName */
};

static emlrtRSInfo ot_emlrtRSI = {
    1723,                                  /* lineNo */
    "RigidBodyTree/validateInputBodyName", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo pt_emlrtRSI = {
    1727,                                  /* lineNo */
    "RigidBodyTree/validateInputBodyName", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo qt_emlrtRSI = {
    1428,                                /* lineNo */
    "RigidBodyTree/findBodyIndexByName", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo rt_emlrtRSI = {
    1434,                                /* lineNo */
    "RigidBodyTree/findBodyIndexByName", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo
    vt_emlrtRSI =
        {
            990,                             /* lineNo */
            "rigidBodyTree/inverseDynamics", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyTre"
            "e.m" /* pathName */
};

static emlrtRSInfo
    wt_emlrtRSI =
        {
            991,                             /* lineNo */
            "rigidBodyTree/inverseDynamics", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyTre"
            "e.m" /* pathName */
};

static emlrtRSInfo
    xt_emlrtRSI =
        {
            992,                             /* lineNo */
            "rigidBodyTree/inverseDynamics", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyTre"
            "e.m" /* pathName */
};

static emlrtRSInfo yt_emlrtRSI = {
    1552,                                           /* lineNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo au_emlrtRSI = {
    1553,                                           /* lineNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo bu_emlrtRSI = {
    1554,                                           /* lineNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo cu_emlrtRSI = {
    1574,                                           /* lineNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo du_emlrtRSI = {
    1585,                                           /* lineNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo eu_emlrtRSI = {
    1601,                                           /* lineNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo fu_emlrtRSI = {
    647,                                /* lineNo */
    "RigidBodyTree/homeJointPositions", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo gu_emlrtRSI = {
    653,                                /* lineNo */
    "RigidBodyTree/homeJointPositions", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo
    hu_emlrtRSI =
        {
            288,                               /* lineNo */
            "rigidBodyJoint/get.HomePosition", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

static emlrtRSInfo wu_emlrtRSI = {
    1533,                              /* lineNo */
    "RigidBodyTree/resultPostProcess", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo xu_emlrtRSI = {
    1534,                              /* lineNo */
    "RigidBodyTree/resultPostProcess", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pathName */
};

static emlrtRSInfo yu_emlrtRSI = {
    21,        /* lineNo */
    "warning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\warning.m" /* pathName */
};

static emlrtRTEInfo ob_emlrtRTEI = {
    1433,                                /* lineNo */
    21,                                  /* colNo */
    "RigidBodyTree/findBodyIndexByName", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo pb_emlrtRTEI = {
    2417,                                             /* lineNo */
    13,                                               /* colNo */
    "RigidBodyTree/assertUpperBoundOnVelocityNumber", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtBCInfo hc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1135,                              /* lineNo */
    32,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ic_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1116,                              /* lineNo */
    28,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo cb_emlrtECI = {
    -1,                                /* nDims */
    1147,                              /* lineNo */
    21,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtBCInfo jc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1147,                              /* lineNo */
    34,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = {
    1147,                              /* lineNo */
    34,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo kc_emlrtBCI = {
    -1,                                /* iFirst */
    -1,                                /* iLast */
    1147,                              /* lineNo */
    27,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = {
    1147,                              /* lineNo */
    27,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo l_emlrtDCI = {
    1135,                              /* lineNo */
    32,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtRTEInfo qb_emlrtRTEI = {
    1129,                              /* lineNo */
    21,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtDCInfo m_emlrtDCI = {
    1107,                              /* lineNo */
    28,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = {
    1107,                              /* lineNo */
    28,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtBCInfo lc_emlrtBCI = {
    0,                                 /* iFirst */
    6,                                 /* iLast */
    1130,                              /* lineNo */
    35,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo mc_emlrtBCI = {
    0,                                   /* iFirst */
    6,                                   /* iLast */
    1434,                                /* lineNo */
    38,                                  /* colNo */
    "",                                  /* aName */
    "RigidBodyTree/findBodyIndexByName", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo nc_emlrtBCI = {
    0,                                 /* iFirst */
    6,                                 /* iLast */
    1115,                              /* lineNo */
    35,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo oc_emlrtBCI = {
    0,                                 /* iFirst */
    6,                                 /* iLast */
    1124,                              /* lineNo */
    39,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo o_emlrtDCI = {
    1124,                              /* lineNo */
    39,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo pc_emlrtBCI = {
    1,                                 /* iFirst */
    7,                                 /* iLast */
    1125,                              /* lineNo */
    31,                                /* colNo */
    "",                                /* aName */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    3                            /* checkKind */
};

static emlrtDCInfo p_emlrtDCI = {
    1125,                              /* lineNo */
    31,                                /* colNo */
    "RigidBodyTree/geometricJacobian", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo u_emlrtDCI = {
    1555,                                           /* lineNo */
    26,                                             /* colNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo v_emlrtDCI = {
    1555,                                           /* lineNo */
    26,                                             /* colNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo w_emlrtDCI = {
    1557,                                           /* lineNo */
    29,                                             /* colNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

static emlrtDCInfo x_emlrtDCI = {
    1557,                                           /* lineNo */
    29,                                             /* colNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtRTEInfo wb_emlrtRTEI = {
    650,                                /* lineNo */
    21,                                 /* colNo */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtBCInfo xc_emlrtBCI = {
    1,                                  /* iFirst */
    7,                                  /* iLast */
    651,                                /* lineNo */
    40,                                 /* colNo */
    "",                                 /* aName */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo y_emlrtDCI = {
    654,                                /* lineNo */
    23,                                 /* colNo */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo yc_emlrtBCI = {
    -1,                                 /* iFirst */
    -1,                                 /* iLast */
    654,                                /* lineNo */
    23,                                 /* colNo */
    "",                                 /* aName */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtDCInfo ab_emlrtDCI = {
    654,                                /* lineNo */
    28,                                 /* colNo */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtBCInfo ad_emlrtBCI = {
    -1,                                 /* iFirst */
    -1,                                 /* iLast */
    654,                                /* lineNo */
    28,                                 /* colNo */
    "",                                 /* aName */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    0                            /* checkKind */
};

static emlrtECInfo db_emlrtECI = {
    -1,                                 /* nDims */
    654,                                /* lineNo */
    21,                                 /* colNo */
    "RigidBodyTree/homeJointPositions", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtRTEInfo xb_emlrtRTEI = {
    2425,                                             /* lineNo */
    13,                                               /* colNo */
    "RigidBodyTree/assertUpperBoundOnPositionNumber", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

static emlrtDCInfo bb_emlrtDCI =
    {
        289,                               /* lineNo */
        51,                                /* colNo */
        "rigidBodyJoint/get.HomePosition", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        1    /* checkKind */
};

static emlrtBCInfo bd_emlrtBCI =
    {
        1,                                 /* iFirst */
        7,                                 /* iLast */
        289,                               /* lineNo */
        51,                                /* colNo */
        "",                                /* aName */
        "rigidBodyJoint/get.HomePosition", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoint."
        "m", /* pName */
        0    /* checkKind */
};

static emlrtDCInfo cb_emlrtDCI = {
    1554,                                           /* lineNo */
    13,                                             /* colNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    1                            /* checkKind */
};

static emlrtDCInfo db_emlrtDCI = {
    1554,                                           /* lineNo */
    13,                                             /* colNo */
    "RigidBodyTree/validateDynamicsFunctionInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m", /* pName */
    4                            /* checkKind */
};

/* Function Definitions */
void rigidBodyTree_geometricJacobian(const emlrtStack *sp, rigidBodyTree *obj,
                                     const real_T Q_data[],
                                     const int32_T Q_size[2], real_T Jac_data[],
                                     int32_T Jac_size[2])
{
  static const char_T a[5] = {'t', 'o', 'o', 'l', '0'};
  static const char_T b_cv[5] = {'t', 'o', 'o', 'l', '0'};
  static const char_T b_cv1[5] = {'f', 'i', 'x', 'e', 'd'};
  static const char_T varargin_1[5] = {'J', 'o', 'i', 'n', 't'};
  __m128d r;
  __m128d r1;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  c_robotics_manip_internal_Rigid *body;
  cell_wrap_63 Ttree_data[7];
  d_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T B_data[294];
  real_T JacSlice_data[36];
  real_T X[36];
  real_T b_data[36];
  real_T T1[16];
  real_T T2[16];
  real_T T2inv[16];
  real_T R[9];
  real_T b_R[9];
  real_T c_R[3];
  real_T idx[2];
  real_T alpha1;
  real_T beta1;
  real_T d;
  real_T velnum;
  int32_T JacSlice_size[2];
  int32_T Ttree_size[2];
  int32_T b_size[2];
  int32_T b_i;
  int32_T endeffectorIndex;
  int32_T exitg1;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T kstr;
  int32_T loop_ub_tmp;
  char_T obj_Vector[200];
  char_T TRANSA1;
  char_T TRANSB1;
  int8_T chainmask[7];
  boolean_T b_bool;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &is_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_obj = obj->TreeInternal;
  b_st.site = &js_emlrtRSI;
  idx[0] = 1.0;
  idx[1] = b_obj->PositionNumber;
  c_st.site = &ts_emlrtRSI;
  validateattributes(&c_st, Q_data, Q_size, idx);
  b_st.site = &ks_emlrtRSI;
  RigidBodyTree_forwardKinematics(&b_st, b_obj, Q_data, Q_size[1], Ttree_data,
                                  Ttree_size);
  b_st.site = &ls_emlrtRSI;
  velnum = b_obj->VelocityNumber;
  if (!(velnum <= 49.0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &pb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(velnum >= 0.0)) {
    emlrtNonNegativeCheckR2012b(velnum, &n_emlrtDCI, &st);
  }
  if (velnum != muDoubleScalarFloor(velnum)) {
    emlrtIntegerCheckR2012b(velnum, &m_emlrtDCI, &st);
  }
  loop_ub_tmp = 6 * (int32_T)velnum;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&Jac_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }
  for (i = 0; i < 7; i++) {
    chainmask[i] = 0;
  }
  b_st.site = &ms_emlrtRSI;
  c_st.site = &nt_emlrtRSI;
  alpha1 = b_obj->Base.NameInternal.Length;
  for (i = 0; i < 200; i++) {
    obj_Vector[i] = b_obj->Base.NameInternal.Vector[i];
  }
  if (alpha1 < 1.0) {
    i = 0;
  } else {
    if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
      emlrtIntegerCheckR2012b(alpha1, &c_emlrtDCI, &c_st);
    }
    if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > 200)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, 200, &f_emlrtBCI,
                                    &c_st);
    }
    i = (int32_T)alpha1;
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (b_cv[kstr] != obj_Vector[kstr]) {
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
    memset(&T2inv[0], 0, 16U * sizeof(real_T));
    T2inv[0] = 1.0;
    T2inv[5] = 1.0;
    T2inv[10] = 1.0;
    T2inv[15] = 1.0;
    memcpy(&T2[0], &T2inv[0], 16U * sizeof(real_T));
  } else {
    b_st.site = &ns_emlrtRSI;
    c_st.site = &ot_emlrtRSI;
    endeffectorIndex = -2;
    d_st.site = &qt_emlrtRSI;
    e_st.site = &nt_emlrtRSI;
    alpha1 = b_obj->Base.NameInternal.Length;
    for (i = 0; i < 200; i++) {
      obj_Vector[i] = b_obj->Base.NameInternal.Vector[i];
    }
    if (alpha1 < 1.0) {
      i = 0;
    } else {
      if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
        emlrtIntegerCheckR2012b(alpha1, &c_emlrtDCI, &e_st);
      }
      if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > 200)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, 200, &f_emlrtBCI,
                                      &e_st);
      }
      i = (int32_T)alpha1;
    }
    b_bool = false;
    if (i == 5) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 5) {
          if (obj_Vector[kstr] != b_cv[kstr]) {
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
      endeffectorIndex = -1;
    } else {
      boolean_T exitg2;
      d = b_obj->NumBodies;
      emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                    &ob_emlrtRTEI, &c_st);
      b_i = 0;
      exitg2 = false;
      while ((!exitg2) && (b_i <= (int32_T)d - 1)) {
        d_st.site = &rt_emlrtRSI;
        if (b_i > 6) {
          emlrtDynamicBoundsCheckR2012b(7, 0, 6, &mc_emlrtBCI, &d_st);
        }
        body = b_obj->Bodies[b_i];
        e_st.site = &nt_emlrtRSI;
        alpha1 = body->NameInternal.Length;
        for (i = 0; i < 200; i++) {
          obj_Vector[i] = body->NameInternal.Vector[i];
        }
        if (alpha1 < 1.0) {
          i = 0;
        } else {
          if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
            emlrtIntegerCheckR2012b(alpha1, &c_emlrtDCI, &e_st);
          }
          if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > 200)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, 200, &f_emlrtBCI,
                                          &e_st);
          }
          i = (int32_T)alpha1;
        }
        b_bool = false;
        if (i == 5) {
          kstr = 0;
          do {
            exitg1 = 0;
            if (kstr < 5) {
              if (obj_Vector[kstr] != b_cv[kstr]) {
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
          endeffectorIndex = b_i;
          exitg2 = true;
        } else {
          b_i++;
        }
      }
    }
    if (endeffectorIndex + 1 == -1) {
      c_st.site = &pt_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "robotics:robotmanip:rigidbodytree:BodyNotFound",
          "robotics:robotmanip:rigidbodytree:BodyNotFound", 3, 4, 5, &a[0]);
    }
    if (endeffectorIndex < 0) {
      emlrtDynamicBoundsCheckR2012b(endeffectorIndex, 0, 6, &nc_emlrtBCI, &st);
    }
    body = b_obj->Bodies[endeffectorIndex];
    if (endeffectorIndex > Ttree_size[1] - 1) {
      emlrtDynamicBoundsCheckR2012b(endeffectorIndex, 0, Ttree_size[1] - 1,
                                    &ic_emlrtBCI, &st);
    }
    memcpy(&T2[0], &Ttree_data[endeffectorIndex].f1[0], 16U * sizeof(real_T));
    for (i = 0; i < 3; i++) {
      R[3 * i] = Ttree_data[endeffectorIndex].f1[i];
      R[3 * i + 1] = Ttree_data[endeffectorIndex].f1[i + 4];
      R[3 * i + 2] = Ttree_data[endeffectorIndex].f1[i + 8];
    }
    r = _mm_loadu_pd(&R[0]);
    r1 = _mm_set1_pd(-1.0);
    _mm_storeu_pd(&b_R[0], _mm_mul_pd(r, r1));
    r = _mm_loadu_pd(&R[2]);
    _mm_storeu_pd(&b_R[2], _mm_mul_pd(r, r1));
    r = _mm_loadu_pd(&R[4]);
    _mm_storeu_pd(&b_R[4], _mm_mul_pd(r, r1));
    r = _mm_loadu_pd(&R[6]);
    _mm_storeu_pd(&b_R[6], _mm_mul_pd(r, r1));
    b_R[8] = -R[8];
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      T2inv[kstr] = R[3 * i];
      T2inv[kstr + 1] = R[3 * i + 1];
      T2inv[kstr + 2] = R[3 * i + 2];
      T2inv[i + 12] = (b_R[i] * Ttree_data[endeffectorIndex].f1[12] +
                       b_R[i + 3] * Ttree_data[endeffectorIndex].f1[13]) +
                      b_R[i + 6] * Ttree_data[endeffectorIndex].f1[14];
    }
    T2inv[3] = 0.0;
    T2inv[7] = 0.0;
    T2inv[11] = 0.0;
    T2inv[15] = 1.0;
    chainmask[endeffectorIndex] = 1;
    while (body->ParentIndex > 0.0) {
      if (body->ParentIndex !=
          (int32_T)muDoubleScalarFloor(body->ParentIndex)) {
        emlrtIntegerCheckR2012b(body->ParentIndex, &o_emlrtDCI, &st);
      }
      i = (int32_T)body->ParentIndex - 1;
      if ((i < 0) || (i > 6)) {
        emlrtDynamicBoundsCheckR2012b(i, 0, 6, &oc_emlrtBCI, &st);
      }
      body = b_obj->Bodies[i];
      if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
        emlrtIntegerCheckR2012b(body->Index, &p_emlrtDCI, &st);
      }
      i = (int32_T)body->Index;
      if ((i < 1) || (i > 7)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 7, &pc_emlrtBCI, &st);
      }
      chainmask[i - 1] = 1;
    }
  }
  d = b_obj->NumBodies;
  i = (int32_T)d;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, d, mxDOUBLE_CLASS, (int32_T)d,
                                &qb_emlrtRTEI, &st);
  for (b_i = 0; b_i < i; b_i++) {
    if (b_i > 6) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, 6, &lc_emlrtBCI, &st);
    }
    body = b_obj->Bodies[b_i];
    b_st.site = &os_emlrtRSI;
    c_st.site = &pb_emlrtRSI;
    alpha1 = body->JointInternal.TypeInternal.Length;
    for (i1 = 0; i1 < 200; i1++) {
      obj_Vector[i1] = body->JointInternal.TypeInternal.Vector[i1];
    }
    if (alpha1 < 1.0) {
      i1 = 0;
    } else {
      if (alpha1 != (int32_T)muDoubleScalarFloor(alpha1)) {
        emlrtIntegerCheckR2012b(alpha1, &c_emlrtDCI, &c_st);
      }
      if (((int32_T)alpha1 < 1) || ((int32_T)alpha1 > 200)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)alpha1, 1, 200, &f_emlrtBCI,
                                      &c_st);
      }
      i1 = (int32_T)alpha1;
    }
    b_bool = false;
    if (i1 == 5) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 5) {
          if (obj_Vector[kstr] != b_cv1[kstr]) {
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
      b_st.site = &os_emlrtRSI;
      if (chainmask[b_i] != 0) {
        real_T Tdh[16];
        real_T b_T2inv[16];
        real_T d1;
        if (body->Index != (int32_T)muDoubleScalarFloor(body->Index)) {
          emlrtIntegerCheckR2012b(body->Index, &l_emlrtDCI, &st);
        }
        i1 = (int32_T)body->Index - 1;
        if ((i1 < 0) || (i1 > Ttree_size[1] - 1)) {
          emlrtDynamicBoundsCheckR2012b(i1, 0, Ttree_size[1] - 1, &hc_emlrtBCI,
                                        &st);
        }
        memcpy(&T1[0], &Ttree_data[i1].f1[0], 16U * sizeof(real_T));
        b_st.site = &ps_emlrtRSI;
        if (body->Index == 0.0) {
          c_st.site = &st_emlrtRSI;
          emlrtErrorWithMessageIdR2018a(
              &c_st, &d_emlrtRTEI,
              "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
              "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
              &varargin_1[0]);
        }
        for (i1 = 0; i1 < 16; i1++) {
          Tdh[i1] = body->JointInternal.ChildToJointTransform[i1];
        }
        for (i1 = 0; i1 < 3; i1++) {
          R[3 * i1] = Tdh[i1];
          R[3 * i1 + 1] = Tdh[i1 + 4];
          R[3 * i1 + 2] = Tdh[i1 + 8];
        }
        r = _mm_loadu_pd(&R[0]);
        r1 = _mm_set1_pd(-1.0);
        _mm_storeu_pd(&b_R[0], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[2]);
        _mm_storeu_pd(&b_R[2], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[4]);
        _mm_storeu_pd(&b_R[4], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[6]);
        _mm_storeu_pd(&b_R[6], _mm_mul_pd(r, r1));
        b_R[8] = -R[8];
        d = Tdh[12];
        alpha1 = Tdh[13];
        beta1 = Tdh[14];
        r = _mm_loadu_pd(&b_R[0]);
        r = _mm_mul_pd(r, _mm_set1_pd(d));
        r1 = _mm_loadu_pd(&b_R[3]);
        r1 = _mm_mul_pd(r1, _mm_set1_pd(alpha1));
        r = _mm_add_pd(r, r1);
        r1 = _mm_loadu_pd(&b_R[6]);
        r1 = _mm_mul_pd(r1, _mm_set1_pd(beta1));
        r = _mm_add_pd(r, r1);
        _mm_storeu_pd(&c_R[0], r);
        c_R[2] = (b_R[2] * d + b_R[5] * alpha1) + b_R[8] * beta1;
        for (i1 = 0; i1 < 4; i1++) {
          d = T2inv[i1];
          alpha1 = T2inv[i1 + 4];
          beta1 = T2inv[i1 + 8];
          d1 = T2inv[i1 + 12];
          for (endeffectorIndex = 0; endeffectorIndex < 4; endeffectorIndex++) {
            i2 = endeffectorIndex << 2;
            b_T2inv[i1 + i2] =
                ((d * T1[i2] + alpha1 * T1[i2 + 1]) + beta1 * T1[i2 + 2]) +
                d1 * T1[i2 + 3];
          }
        }
        for (i1 = 0; i1 < 3; i1++) {
          kstr = i1 << 2;
          Tdh[kstr] = R[3 * i1];
          Tdh[kstr + 1] = R[3 * i1 + 1];
          Tdh[kstr + 2] = R[3 * i1 + 2];
          Tdh[i1 + 12] = c_R[i1];
        }
        Tdh[3] = 0.0;
        Tdh[7] = 0.0;
        Tdh[11] = 0.0;
        Tdh[15] = 1.0;
        for (i1 = 0; i1 < 4; i1++) {
          d = b_T2inv[i1];
          alpha1 = b_T2inv[i1 + 4];
          beta1 = b_T2inv[i1 + 8];
          d1 = b_T2inv[i1 + 12];
          for (endeffectorIndex = 0; endeffectorIndex < 4; endeffectorIndex++) {
            i2 = endeffectorIndex << 2;
            T1[i1 + i2] =
                ((d * Tdh[i2] + alpha1 * Tdh[i2 + 1]) + beta1 * Tdh[i2 + 2]) +
                d1 * Tdh[i2 + 3];
          }
        }
        idx[0] = b_obj->VelocityDoFMap[b_i];
        idx[1] = b_obj->VelocityDoFMap[b_i + 7];
        b_st.site = &rs_emlrtRSI;
        c_st.site = &qs_emlrtRSI;
        if (body->Index == 0.0) {
          d_st.site = &st_emlrtRSI;
          emlrtErrorWithMessageIdR2018a(
              &d_st, &d_emlrtRTEI,
              "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
              "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
              &varargin_1[0]);
        }
        c_st.site = &qs_emlrtRSI;
        c_rigidBodyJoint_get_MotionSubs(&c_st, &body->JointInternal, b_data,
                                        b_size);
        R[0] = 0.0;
        R[3] = -T1[14];
        R[6] = T1[13];
        R[1] = T1[14];
        R[4] = 0.0;
        R[7] = -T1[12];
        R[2] = -T1[13];
        R[5] = T1[12];
        R[8] = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          d = R[i1];
          alpha1 = R[i1 + 3];
          beta1 = R[i1 + 6];
          for (endeffectorIndex = 0; endeffectorIndex < 3; endeffectorIndex++) {
            i2 = endeffectorIndex << 2;
            b_R[i1 + 3 * endeffectorIndex] =
                (d * T1[i2] + alpha1 * T1[i2 + 1]) + beta1 * T1[i2 + 2];
            X[endeffectorIndex + 6 * i1] = T1[endeffectorIndex + (i1 << 2)];
            X[endeffectorIndex + 6 * (i1 + 3)] = 0.0;
          }
        }
        for (i1 = 0; i1 < 3; i1++) {
          X[6 * i1 + 3] = b_R[3 * i1];
          kstr = i1 << 2;
          endeffectorIndex = 6 * (i1 + 3);
          X[endeffectorIndex + 3] = T1[kstr];
          X[6 * i1 + 4] = b_R[3 * i1 + 1];
          X[endeffectorIndex + 4] = T1[kstr + 1];
          X[6 * i1 + 5] = b_R[3 * i1 + 2];
          X[endeffectorIndex + 5] = T1[kstr + 2];
        }
        c_st.site = &je_emlrtRSI;
        e_mtimes(X, b_data, b_size, JacSlice_data, JacSlice_size);
        if (idx[0] > idx[1]) {
          i1 = 0;
          endeffectorIndex = 0;
        } else {
          if (idx[0] != (int32_T)muDoubleScalarFloor(idx[0])) {
            emlrtIntegerCheckR2012b(idx[0], &k_emlrtDCI, &st);
          }
          if (((int32_T)idx[0] < 1) || ((int32_T)idx[0] > (int32_T)velnum)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)idx[0], 1, (int32_T)velnum,
                                          &kc_emlrtBCI, &st);
          }
          i1 = (int32_T)idx[0] - 1;
          if (idx[1] != (int32_T)muDoubleScalarFloor(idx[1])) {
            emlrtIntegerCheckR2012b(idx[1], &j_emlrtDCI, &st);
          }
          if (((int32_T)idx[1] < 1) || ((int32_T)idx[1] > (int32_T)velnum)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)idx[1], 1, (int32_T)velnum,
                                          &jc_emlrtBCI, &st);
          }
          endeffectorIndex = (int32_T)idx[1];
        }
        b_size[0] = 6;
        kstr = endeffectorIndex - i1;
        b_size[1] = kstr;
        emlrtSubAssignSizeCheckR2012b(&b_size[0], 2, &JacSlice_size[0], 2,
                                      &cb_emlrtECI, &st);
        for (endeffectorIndex = 0; endeffectorIndex < kstr;
             endeffectorIndex++) {
          for (i2 = 0; i2 < 6; i2++) {
            Jac_data[i2 + 6 * (i1 + endeffectorIndex)] =
                JacSlice_data[i2 + 6 * endeffectorIndex];
          }
        }
      }
    }
  }
  for (i = 0; i < 3; i++) {
    i1 = i << 2;
    d = T2[i1];
    X[6 * i] = d;
    kstr = 6 * (i + 3);
    X[kstr] = 0.0;
    X[6 * i + 3] = 0.0;
    X[kstr + 3] = d;
    d = T2[i1 + 1];
    X[6 * i + 1] = d;
    X[kstr + 1] = 0.0;
    X[6 * i + 4] = 0.0;
    X[kstr + 4] = d;
    d = T2[i1 + 2];
    X[6 * i + 2] = d;
    X[kstr + 2] = 0.0;
    X[6 * i + 5] = 0.0;
    X[kstr + 5] = d;
  }
  b_st.site = &ss_emlrtRSI;
  c_st.site = &je_emlrtRSI;
  if ((int32_T)velnum == 0) {
    Jac_size[0] = 6;
    Jac_size[1] = 0;
  } else {
    d_st.site = &le_emlrtRSI;
    memcpy(&B_data[0], &Jac_data[0], (uint32_T)loop_ub_tmp * sizeof(real_T));
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)6;
    n_t = (ptrdiff_t)(int32_T)velnum;
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)6;
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)6;
    Jac_size[0] = 6;
    Jac_size[1] = (int32_T)velnum;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &X[0], &lda_t,
          &B_data[0], &ldb_t, &beta1, &Jac_data[0], &ldc_t);
  }
}

void rigidBodyTree_inverseDynamics(const emlrtStack *sp, rigidBodyTree *obj,
                                   const real_T varargin_1_data[],
                                   const int32_T varargin_1_size[2],
                                   const real_T varargin_2[6],
                                   const real_T varargin_3[6],
                                   real_T tau_data[], int32_T tau_size[2])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  d_robotics_manip_internal_Rigid *b_obj;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_boolean_T c_tmp_data;
  emxArray_boolean_T d_tmp_data;
  rigidBodyJoint *c_obj;
  real_T q_data[49];
  real_T tmp_data[42];
  real_T b_tau_data[6];
  real_T p[2];
  real_T sz2[2];
  real_T nb;
  real_T obj_Length;
  real_T posnum;
  real_T vnum;
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T b_tmp_size;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T q_size;
  int32_T q_size_tmp;
  boolean_T b_tmp_data[6];
  boolean_T b_bool;
  boolean_T exitg2;
  boolean_T guard1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &vt_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_obj = obj->TreeInternal;
  b_st.site = &yt_emlrtRSI;
  vnum = b_obj->VelocityNumber;
  if (!(vnum <= 49.0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &pb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  b_st.site = &au_emlrtRSI;
  nb = b_obj->NumBodies;
  if (!(nb <= 7.0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  b_st.site = &bu_emlrtRSI;
  c_st.site = &fu_emlrtRSI;
  posnum = b_obj->PositionNumber;
  if (!(posnum <= 49.0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &xb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(posnum >= 0.0)) {
    emlrtNonNegativeCheckR2012b(posnum, &db_emlrtDCI, &b_st);
  }
  obj_Length = muDoubleScalarFloor(posnum);
  if (posnum != obj_Length) {
    emlrtIntegerCheckR2012b(posnum, &cb_emlrtDCI, &b_st);
  }
  q_size_tmp = (int32_T)posnum;
  q_size = (int32_T)posnum;
  if (posnum != obj_Length) {
    emlrtIntegerCheckR2012b(posnum, &cb_emlrtDCI, &b_st);
  }
  if (q_size_tmp - 1 >= 0) {
    memset(&q_data[0], 0, (uint32_T)q_size_tmp * sizeof(real_T));
  }
  obj_Length = b_obj->NumBodies;
  i = (int32_T)obj_Length;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, obj_Length, mxDOUBLE_CLASS,
                                (int32_T)obj_Length, &wb_emlrtRTEI, &b_st);
  for (b_i = 0; b_i < i; b_i++) {
    if (((int32_T)((uint32_T)b_i + 1U) < 1) ||
        ((int32_T)((uint32_T)b_i + 1U) > 7)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1, 7,
                                    &xc_emlrtBCI, &b_st);
    }
    p[0] = b_obj->PositionDoFMap[b_i];
    p[1] = b_obj->PositionDoFMap[b_i + 7];
    if (p[0] <= p[1]) {
      real_T qi_data[7];
      int32_T loop_ub;
      char_T obj_Vector[200];
      c_st.site = &gu_emlrtRSI;
      c_obj = &b_obj->Bodies[b_i]->JointInternal;
      d_st.site = &hu_emlrtRSI;
      e_st.site = &pb_emlrtRSI;
      obj_Length = c_obj->TypeInternal.Length;
      for (i1 = 0; i1 < 200; i1++) {
        obj_Vector[i1] = c_obj->TypeInternal.Vector[i1];
      }
      if (obj_Length < 1.0) {
        i1 = 0;
      } else {
        if (obj_Length != (int32_T)muDoubleScalarFloor(obj_Length)) {
          emlrtIntegerCheckR2012b(obj_Length, &c_emlrtDCI, &e_st);
        }
        if (((int32_T)obj_Length < 1) || ((int32_T)obj_Length > 200)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)obj_Length, 1, 200,
                                        &f_emlrtBCI, &e_st);
        }
        i1 = (int32_T)obj_Length;
      }
      b_bool = false;
      if (i1 == 5) {
        q_size_tmp = 0;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (q_size_tmp < 5) {
            if (obj_Vector[q_size_tmp] != b_cv[q_size_tmp]) {
              exitg1 = 1;
            } else {
              q_size_tmp++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (!b_bool) {
        obj_Length = c_obj->PositionNumber;
        if (obj_Length < 1.0) {
          q_size_tmp = 0;
        } else {
          if (obj_Length != (int32_T)muDoubleScalarFloor(obj_Length)) {
            emlrtIntegerCheckR2012b(obj_Length, &bb_emlrtDCI, &c_st);
          }
          if (((int32_T)obj_Length < 1) || ((int32_T)obj_Length > 7)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)obj_Length, 1, 7,
                                          &bd_emlrtBCI, &c_st);
          }
          q_size_tmp = (int32_T)obj_Length;
        }
        for (i1 = 0; i1 < q_size_tmp; i1++) {
          qi_data[i1] = c_obj->HomePositionInternal[i1];
        }
      } else {
        q_size_tmp = 1;
        qi_data[0] = 0.0;
      }
      if (p[0] > p[1]) {
        i1 = 0;
        i2 = 0;
      } else {
        if (p[0] != (int32_T)muDoubleScalarFloor(p[0])) {
          emlrtIntegerCheckR2012b(p[0], &y_emlrtDCI, &b_st);
        }
        if (((int32_T)p[0] < 1) || ((int32_T)p[0] > (int32_T)posnum)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)p[0], 1, (int32_T)posnum,
                                        &yc_emlrtBCI, &b_st);
        }
        i1 = (int32_T)p[0] - 1;
        if (p[1] != (int32_T)muDoubleScalarFloor(p[1])) {
          emlrtIntegerCheckR2012b(p[1], &ab_emlrtDCI, &b_st);
        }
        if (((int32_T)p[1] < 1) || ((int32_T)p[1] > (int32_T)posnum)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)p[1], 1, (int32_T)posnum,
                                        &ad_emlrtBCI, &b_st);
        }
        i2 = (int32_T)p[1];
      }
      loop_ub = i2 - i1;
      if (loop_ub != q_size_tmp) {
        emlrtSubAssignSizeCheck1dR2017a(loop_ub, q_size_tmp, &db_emlrtECI,
                                        &b_st);
      }
      for (i2 = 0; i2 < loop_ub; i2++) {
        q_data[i1 + i2] = qi_data[i2];
      }
    }
  }
  if (!(vnum >= 0.0)) {
    emlrtNonNegativeCheckR2012b(vnum, &u_emlrtDCI, &st);
  }
  if (vnum != muDoubleScalarFloor(vnum)) {
    emlrtIntegerCheckR2012b(vnum, &v_emlrtDCI, &st);
  }
  if (!(nb >= 0.0)) {
    emlrtNonNegativeCheckR2012b(nb, &w_emlrtDCI, &st);
  }
  if (nb != muDoubleScalarFloor(nb)) {
    emlrtIntegerCheckR2012b(nb, &x_emlrtDCI, &st);
  }
  p[0] = 1.0;
  p[1] = b_obj->PositionNumber;
  sz2[0] = 1.0;
  sz2[1] = b_obj->VelocityNumber;
  if (varargin_1_size[1] != 0) {
    b_st.site = &cu_emlrtRSI;
    validateattributes(&b_st, varargin_1_data, varargin_1_size, p);
    q_size_tmp = varargin_1_size[1];
    q_size = varargin_1_size[1];
    memcpy(&q_data[0], &varargin_1_data[0],
           (uint32_T)q_size_tmp * sizeof(real_T));
  }
  b_st.site = &du_emlrtRSI;
  c_st.site = &vc_emlrtRSI;
  b_bool = true;
  q_size_tmp = 0;
  exitg2 = false;
  while ((!exitg2) && (q_size_tmp < 6)) {
    if (!muDoubleScalarIsNaN(varargin_2[q_size_tmp])) {
      q_size_tmp++;
    } else {
      b_bool = false;
      exitg2 = true;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &ib_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:validateDynamicsFunctionInputs:expectedNonNaN", 3, 4, 28,
        "joint velocity vector (qdot)");
  }
  c_st.site = &vc_emlrtRSI;
  b_bool = true;
  q_size_tmp = 0;
  exitg2 = false;
  while ((!exitg2) && (q_size_tmp < 6)) {
    if ((!muDoubleScalarIsInf(varargin_2[q_size_tmp])) &&
        (!muDoubleScalarIsNaN(varargin_2[q_size_tmp]))) {
      q_size_tmp++;
    } else {
      b_bool = false;
      exitg2 = true;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:validateDynamicsFunctionInputs:expectedFinite", 3, 4, 28,
        "joint velocity vector (qdot)");
  }
  c_st.site = &vc_emlrtRSI;
  b_bool = true;
  for (q_size_tmp = 0; q_size_tmp < 2; q_size_tmp++) {
    if (b_bool) {
      obj_Length = sz2[q_size_tmp];
      if ((!(obj_Length != obj_Length)) &&
          (muDoubleScalarIsInf(obj_Length) || (!(obj_Length >= 0.0)) ||
           (!(obj_Length == muDoubleScalarFloor(obj_Length))))) {
        b_bool = false;
      }
    } else {
      b_bool = false;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
                                  "MATLAB:validateattributes:badSizeArray",
                                  "MATLAB:validateattributes:badSizeArray", 0);
  }
  b_bool = true;
  for (q_size_tmp = 0; q_size_tmp < 2; q_size_tmp++) {
    if (b_bool) {
      obj_Length = sz2[q_size_tmp];
      if ((!(obj_Length != obj_Length)) &&
          (!(obj_Length == 5.0 * (real_T)q_size_tmp + 1.0))) {
        b_bool = false;
      }
    } else {
      b_bool = false;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &sb_emlrtRTEI, "Coder:toolbox:ValidateattributesincorrectSize",
        "MATLAB:validateDynamicsFunctionInputs:incorrectSize", 3, 4, 28,
        "joint velocity vector (qdot)");
  }
  b_st.site = &eu_emlrtRSI;
  c_st.site = &vc_emlrtRSI;
  b_bool = true;
  q_size_tmp = 0;
  exitg2 = false;
  while ((!exitg2) && (q_size_tmp < 6)) {
    if (!muDoubleScalarIsNaN(varargin_3[q_size_tmp])) {
      q_size_tmp++;
    } else {
      b_bool = false;
      exitg2 = true;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &ib_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:validateDynamicsFunctionInputs:expectedNonNaN", 3, 4, 33,
        "joint acceleration vector (qddot)");
  }
  c_st.site = &vc_emlrtRSI;
  b_bool = true;
  q_size_tmp = 0;
  exitg2 = false;
  while ((!exitg2) && (q_size_tmp < 6)) {
    if ((!muDoubleScalarIsInf(varargin_3[q_size_tmp])) &&
        (!muDoubleScalarIsNaN(varargin_3[q_size_tmp]))) {
      q_size_tmp++;
    } else {
      b_bool = false;
      exitg2 = true;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &m_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:validateDynamicsFunctionInputs:expectedFinite", 3, 4, 33,
        "joint acceleration vector (qddot)");
  }
  c_st.site = &vc_emlrtRSI;
  b_bool = true;
  for (q_size_tmp = 0; q_size_tmp < 2; q_size_tmp++) {
    if (b_bool) {
      obj_Length = sz2[q_size_tmp];
      if ((!(obj_Length != obj_Length)) &&
          (muDoubleScalarIsInf(obj_Length) || (!(obj_Length >= 0.0)) ||
           (!(obj_Length == muDoubleScalarFloor(obj_Length))))) {
        b_bool = false;
      }
    } else {
      b_bool = false;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
                                  "MATLAB:validateattributes:badSizeArray",
                                  "MATLAB:validateattributes:badSizeArray", 0);
  }
  b_bool = true;
  for (q_size_tmp = 0; q_size_tmp < 2; q_size_tmp++) {
    if (b_bool) {
      obj_Length = sz2[q_size_tmp];
      if ((!(obj_Length != obj_Length)) &&
          (!(obj_Length == 5.0 * (real_T)q_size_tmp + 1.0))) {
        b_bool = false;
      }
    } else {
      b_bool = false;
    }
  }
  if (!b_bool) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &sb_emlrtRTEI, "Coder:toolbox:ValidateattributesincorrectSize",
        "MATLAB:validateDynamicsFunctionInputs:incorrectSize", 3, 4, 33,
        "joint acceleration vector (qddot)");
  }
  tmp_size[0] = 6;
  tmp_size[1] = (int32_T)nb;
  q_size_tmp = 6 * (int32_T)nb;
  if (q_size_tmp - 1 >= 0) {
    memset(&tmp_data[0], 0, (uint32_T)q_size_tmp * sizeof(real_T));
  }
  st.site = &wt_emlrtRSI;
  c_RigidBodyTreeDynamics_inverse(&st, obj->TreeInternal, q_data, q_size,
                                  varargin_2, varargin_3, tmp_data, tmp_size,
                                  b_tau_data);
  st.site = &xt_emlrtRSI;
  b_tmp_size = 6;
  for (i = 0; i < 6; i++) {
    b_tmp_data[i] = muDoubleScalarIsNaN(b_tau_data[i]);
  }
  c_tmp_data.data = &b_tmp_data[0];
  c_tmp_data.size = &b_tmp_size;
  c_tmp_data.allocatedSize = 6;
  c_tmp_data.numDimensions = 1;
  c_tmp_data.canFreeData = false;
  guard1 = false;
  b_st.site = &wu_emlrtRSI;
  if (any(&b_st, &c_tmp_data)) {
    guard1 = true;
  } else {
    b_tmp_size = 6;
    for (i = 0; i < 6; i++) {
      b_tmp_data[i] = muDoubleScalarIsInf(b_tau_data[i]);
    }
    d_tmp_data.data = &b_tmp_data[0];
    d_tmp_data.size = &b_tmp_size;
    d_tmp_data.allocatedSize = 6;
    d_tmp_data.numDimensions = 1;
    d_tmp_data.canFreeData = false;
    b_st.site = &wu_emlrtRSI;
    if (any(&b_st, &d_tmp_data)) {
      guard1 = true;
    }
  }
  if (guard1) {
    b_st.site = &xu_emlrtRSI;
    c_st.site = &yu_emlrtRSI;
    d_warning(&c_st);
  }
  tau_size[0] = 1;
  tau_size[1] = 6;
  for (i = 0; i < 6; i++) {
    tau_data[i] = b_tau_data[i];
  }
}

void rigidBodyTree_set_Gravity(rigidBodyTree *obj)
{
  d_robotics_manip_internal_Rigid *b_obj;
  b_obj = obj->TreeInternal;
  b_obj->Gravity[0] = 0.0;
  b_obj->Gravity[1] = 0.0;
  b_obj->Gravity[2] = -9.81;
}

/* End of code generation (rigidBodyTree.c) */
