/*
 * run_p2p_data.c
 *
 * Code generation for function 'run_p2p_data'
 *
 */

/* Include files */
#include "run_p2p_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                              /* bFirstTime */
    false,                                             /* bInitialized */
    131659U,                                           /* fVersionInfo */
    NULL,                                              /* fErrorFunction */
    "run_p2p",                                         /* fFunctionName */
    NULL,                                              /* fRTCallStack */
    false,                                             /* bDebugMode */
    {497771824U, 2490594622U, 369384464U, 266412326U}, /* fSigWrd */
    NULL                                               /* fSigMem */
};

emlrtRSInfo t_emlrtRSI = {
    1,                               /* lineNo */
    "InternalAccess/InternalAccess", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\InternalAccess.m" /* pathName */
};

emlrtRSInfo
    pb_emlrtRSI =
        {
            155,                       /* lineNo */
            "rigidBodyJoint/get.Type", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\rigidBodyJoi"
            "nt.m" /* pathName */
};

emlrtRSInfo uc_emlrtRSI = {
    8,                          /* lineNo */
    "validateTrajectoryInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\validateTrajectoryInputs"
    ".m" /* pathName */
};

emlrtRSInfo vc_emlrtRSI = {
    93,                   /* lineNo */
    "validateattributes", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\lang\\validateattributes"
    ".m" /* pathName */
};

emlrtRSInfo wc_emlrtRSI = {
    7,             /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo xc_emlrtRSI = {
    8,             /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo yc_emlrtRSI = {
    9,             /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo ad_emlrtRSI = {
    10,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo bd_emlrtRSI = {
    11,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo cd_emlrtRSI = {
    12,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo dd_emlrtRSI = {
    13,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo ed_emlrtRSI = {
    14,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo fd_emlrtRSI = {
    15,            /* lineNo */
    "parseInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\parseInputs.m" /* pathName */
};

emlrtRSInfo gd_emlrtRSI = {
    31,                                /* lineNo */
    "NameValueParser/NameValueParser", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+core\\+internal\\+codegen\\NameValuePar"
    "ser.m" /* pathName */
};

emlrtRSInfo od_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pathName */
};

emlrtRSInfo pd_emlrtRSI = {
    41,    /* lineNo */
    "cat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

emlrtRSInfo qd_emlrtRSI = {
    65,         /* lineNo */
    "cat_impl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

emlrtRSInfo rd_emlrtRSI = {
    53,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

emlrtRSInfo sd_emlrtRSI = {
    64,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

emlrtRSInfo td_emlrtRSI = {
    65,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

emlrtRSInfo wd_emlrtRSI = {
    77,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

emlrtRSInfo xd_emlrtRSI = {
    80,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

emlrtRSInfo yd_emlrtRSI = {
    89,          /* lineNo */
    "solvePoly", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pathName */
};

emlrtRSInfo je_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

emlrtRSInfo ke_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

emlrtRSInfo le_emlrtRSI = {
    142,      /* lineNo */
    "mtimes", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pathName */
};

emlrtRSInfo ne_emlrtRSI = {
    22,    /* lineNo */
    "inv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m" /* pathName
                                                                       */
};

emlrtRSInfo lf_emlrtRSI = {
    63,       /* lineNo */
    "xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo nf_emlrtRSI = {
    138,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo of_emlrtRSI = {
    141,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo qf_emlrtRSI = {
    148,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo rf_emlrtRSI = {
    151,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo sf_emlrtRSI = {
    154,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo tf_emlrtRSI = {
    158,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

emlrtRSInfo
    uf_emlrtRSI =
        {
            173,          /* lineNo */
            "rankFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

emlrtRSInfo
    vf_emlrtRSI =
        {
            172,          /* lineNo */
            "rankFromQR", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\qrsolve.m" /* pathName */
};

emlrtRSInfo ig_emlrtRSI =
    {
        143,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

emlrtRSInfo sg_emlrtRSI = {
    26,                /* lineNo */
    "convertToPPForm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\convertToPPForm.m" /* pathName */
};

emlrtRSInfo tg_emlrtRSI = {
    78,     /* lineNo */
    "mkpp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\mkpp.m" /* pathName
                                                                         */
};

emlrtRSInfo ug_emlrtRSI = {
    40,                  /* lineNo */
    "reshapeSizeChecks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

emlrtRSInfo ch_emlrtRSI = {
    21,        /* lineNo */
    "polyder", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\polyder.m" /* pathName
                                                                            */
};

emlrtRSInfo gi_emlrtRSI = {
    19,                             /* lineNo */
    "addFlatSegmentsToPPFormParts", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pathName */
};

emlrtRSInfo hi_emlrtRSI = {
    62,                /* lineNo */
    "addSegmentToEnd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\polytraj\\+robotics\\+core\\+"
    "internal\\addFlatSegmentsToPPFormParts.m" /* pathName */
};

emlrtRSInfo ik_emlrtRSI = {
    29,                     /* lineNo */
    "quaternionBase/slerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\@quaternionBa"
    "se\\slerp.m" /* pathName */
};

emlrtRSInfo jk_emlrtRSI = {
    26,          /* lineNo */
    "privslerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\privslerp.m" /* pathName */
};

emlrtRSInfo kk_emlrtRSI = {
    27,          /* lineNo */
    "privslerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\privslerp.m" /* pathName */
};

emlrtRSInfo mk_emlrtRSI = {
    78,          /* lineNo */
    "privslerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\privslerp.m" /* pathName */
};

emlrtRSInfo nk_emlrtRSI = {
    81,          /* lineNo */
    "privslerp", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\rotations\\rotationslib\\+"
    "matlabshared\\+rotations\\+internal\\privslerp.m" /* pathName */
};

emlrtRSInfo jl_emlrtRSI = {
    15,              /* lineNo */
    "normalizeRows", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\normalizeRows.m" /* pathName */
};

emlrtRSInfo dq_emlrtRSI = {
    18,         /* lineNo */
    "wrapToPi", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\wrapToPi.m" /* pathName */
};

emlrtRSInfo st_emlrtRSI = {
    443,                   /* lineNo */
    "RigidBody/get.Joint", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBody.m" /* pathName */
};

emlrtMCInfo b_emlrtMCI = {
    53,        /* lineNo */
    19,        /* colNo */
    "flt2str", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\flt2str.m" /* pName */
};

emlrtDCInfo c_emlrtDCI = {
    30,                          /* lineNo */
    32,                          /* colNo */
    "CharacterVector/getVector", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CharacterVector.m", /* pName */
    1                              /* checkKind */
};

emlrtBCInfo f_emlrtBCI = {
    1,                           /* iFirst */
    200,                         /* iLast */
    30,                          /* lineNo */
    32,                          /* colNo */
    "",                          /* aName */
    "CharacterVector/getVector", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\CharacterVector.m", /* pName */
    0                              /* checkKind */
};

emlrtRTEInfo d_emlrtRTEI = {
    28,      /* lineNo */
    9,       /* colNo */
    "error", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\error.m" /* pName */
};

emlrtRTEInfo e_emlrtRTEI = {
    14,                          /* lineNo */
    1,                           /* colNo */
    "validateBoundaryCondition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\validateBoundaryConditio"
    "n.m" /* pName */
};

emlrtRTEInfo f_emlrtRTEI = {
    15,                          /* lineNo */
    1,                           /* colNo */
    "validateBoundaryCondition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\validateBoundaryConditio"
    "n.m" /* pName */
};

emlrtRTEInfo g_emlrtRTEI = {
    16,                          /* lineNo */
    1,                           /* colNo */
    "validateBoundaryCondition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\validateBoundaryConditio"
    "n.m" /* pName */
};

emlrtRTEInfo h_emlrtRTEI = {
    41,     /* lineNo */
    15,     /* colNo */
    "mkpp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\mkpp.m" /* pName
                                                                         */
};

emlrtRTEInfo i_emlrtRTEI = {
    62,     /* lineNo */
    5,      /* colNo */
    "mkpp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\mkpp.m" /* pName
                                                                         */
};

emlrtRTEInfo j_emlrtRTEI = {
    76,     /* lineNo */
    15,     /* colNo */
    "mkpp", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\mkpp.m" /* pName
                                                                         */
};

emlrtRTEInfo k_emlrtRTEI = {
    80,                  /* lineNo */
    13,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

emlrtRTEInfo l_emlrtRTEI = {
    87,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

emlrtRTEInfo m_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatefinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatefinite.m" /* pName */
};

emlrtRTEInfo p_emlrtRTEI = {
    225,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

emlrtRTEInfo q_emlrtRTEI =
    {
        138,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

emlrtRTEInfo s_emlrtRTEI =
    {
        133,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

emlrtRTEInfo u_emlrtRTEI = {
    45,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

emlrtRTEInfo v_emlrtRTEI = {
    48,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

emlrtRTEInfo cb_emlrtRTEI = {
    112,                  /* lineNo */
    9,                    /* colNo */
    "trim_leading_zeros", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\polyder.m" /* pName
                                                                            */
};

emlrtDCInfo
    i_emlrtDCI =
        {
            113,                  /* lineNo */
            36,                   /* colNo */
            "trim_leading_zeros", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\polyfun\\polyder"
            ".m", /* pName */
            4     /* checkKind */
};

emlrtRTEInfo gb_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatege", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatege.m" /* pName */
};

emlrtRTEInfo hb_emlrtRTEI = {
    28,           /* lineNo */
    27,           /* colNo */
    "validatele", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatele.m" /* pName */
};

emlrtRTEInfo ib_emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatenonnan", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonnan.m" /* pName */
};

emlrtRTEInfo jb_emlrtRTEI = {
    13,     /* lineNo */
    9,      /* colNo */
    "sqrt", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m" /* pName
                                                                       */
};

emlrtBCInfo gc_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    23,          /* lineNo */
    15,          /* colNo */
    "",          /* aName */
    "wrapTo2Pi", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\wrapTo2Pi.m", /* pName */
    0                                   /* checkKind */
};

emlrtRTEInfo rb_emlrtRTEI = {
    10,             /* lineNo */
    23,             /* colNo */
    "validatesize", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatesize.m" /* pName */
};

emlrtRTEInfo sb_emlrtRTEI = {
    15,             /* lineNo */
    19,             /* colNo */
    "validatesize", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatesize.m" /* pName */
};

emlrtRTEInfo ub_emlrtRTEI = {
    2405,                                        /* lineNo */
    13,                                          /* colNo */
    "RigidBodyTree/assertUpperBoundOnNumBodies", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTree.m" /* pName */
};

emlrtRTEInfo qc_emlrtRTEI = {
    77,          /* lineNo */
    16,          /* colNo */
    "solvePoly", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\uav_rst\\matlab\\+shared_uav_"
    "rst\\+internal\\+traj\\solvePoly.m" /* pName */
};

const char_T cv[9] = {'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c'};

const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

const int32_T iv2[4] = {0, 1, 2, 3};

const char_T cv2[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                        '_', 'd', 'g', 'e', 'q', 'p', '3'};

emlrtRSInfo rv_emlrtRSI = {
    53,        /* lineNo */
    "flt2str", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\flt2str.m" /* pathName */
};

covrtInstance emlrtCoverageInstance;

/* End of code generation (run_p2p_data.c) */
