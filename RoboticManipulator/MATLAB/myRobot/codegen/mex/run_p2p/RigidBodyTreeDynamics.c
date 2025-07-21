/*
 * RigidBodyTreeDynamics.c
 *
 * Code generation for function 'RigidBodyTreeDynamics'
 *
 */

/* Include files */
#include "RigidBodyTreeDynamics.h"
#include "eml_mtimes_helper.h"
#include "indexShapeCheck.h"
#include "mtimes.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "run_p2p_types.h"
#include "strcmp.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_66
#define typedef_cell_wrap_66
typedef struct {
  real_T f1[36];
} cell_wrap_66;
#endif /* typedef_cell_wrap_66 */

/* Variable Definitions */
static emlrtRSInfo iu_emlrtRSI = {
    158,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo ju_emlrtRSI = {
    164,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo ku_emlrtRSI = {
    168,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo lu_emlrtRSI = {
    169,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo mu_emlrtRSI = {
    170,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo nu_emlrtRSI = {
    171,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo ou_emlrtRSI = {
    172,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo pu_emlrtRSI = {
    174,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo qu_emlrtRSI = {
    184,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo ru_emlrtRSI = {
    189,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo su_emlrtRSI = {
    200,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo tu_emlrtRSI = {
    201,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo uu_emlrtRSI = {
    203,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRSInfo vu_emlrtRSI = {
    204,                                     /* lineNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pathName */
};

static emlrtRTEInfo yb_emlrtRTEI = {
    140,                                     /* lineNo */
    13,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pName */
};

static emlrtDCInfo eb_emlrtDCI = {
    168,                                     /* lineNo */
    28,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo cd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    168,                                     /* lineNo */
    28,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo fb_emlrtDCI = {
    168,                                     /* lineNo */
    33,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo dd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    168,                                     /* lineNo */
    33,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo gb_emlrtDCI = {
    169,                                     /* lineNo */
    34,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo ed_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    169,                                     /* lineNo */
    34,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo hb_emlrtDCI = {
    169,                                     /* lineNo */
    39,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo fd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    169,                                     /* lineNo */
    39,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo ib_emlrtDCI = {
    170,                                     /* lineNo */
    36,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo gd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    170,                                     /* lineNo */
    36,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo jb_emlrtDCI = {
    170,                                     /* lineNo */
    41,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo hd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    170,                                     /* lineNo */
    41,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo id_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    166,                                     /* lineNo */
    26,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo jd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    183,                                     /* lineNo */
    36,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo kb_emlrtDCI = {
    183,                                     /* lineNo */
    51,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo kd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    183,                                     /* lineNo */
    51,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo ld_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    184,                                     /* lineNo */
    41,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo md_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    185,                                     /* lineNo */
    72,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo nd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    184,                                     /* lineNo */
    26,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo od_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    188,                                     /* lineNo */
    36,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo pd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    189,                                     /* lineNo */
    26,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo qd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    193,                                     /* lineNo */
    28,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo rd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    194,                                     /* lineNo */
    33,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo sd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    194,                                     /* lineNo */
    78,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo td_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    195,                                     /* lineNo */
    45,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtRTEInfo ac_emlrtRTEI = {
    199,                                     /* lineNo */
    21,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pName */
};

static emlrtBCInfo ud_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    194,                                     /* lineNo */
    21,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo vd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    204,                                     /* lineNo */
    35,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo wd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    210,                                     /* lineNo */
    36,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo xd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    210,                                     /* lineNo */
    53,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo lb_emlrtDCI = {
    210,                                     /* lineNo */
    25,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo yd_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    210,                                     /* lineNo */
    25,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo mb_emlrtDCI = {
    206,                                     /* lineNo */
    25,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo ae_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    206,                                     /* lineNo */
    25,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo nb_emlrtDCI = {
    206,                                     /* lineNo */
    30,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtBCInfo be_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    206,                                     /* lineNo */
    30,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtECInfo eb_emlrtECI = {
    -1,                                      /* nDims */
    206,                                     /* lineNo */
    21,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m" /* pName */
};

static emlrtBCInfo ce_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    152,                                     /* lineNo */
    23,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo de_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    153,                                     /* lineNo */
    19,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo ee_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    210,                                     /* lineNo */
    45,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    183,                                     /* lineNo */
    43,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo ge_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    189,                                     /* lineNo */
    33,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo he_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    184,                                     /* lineNo */
    33,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo ie_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    186,                                     /* lineNo */
    38,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo je_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    195,                                     /* lineNo */
    34,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo ke_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    186,                                     /* lineNo */
    27,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtDCInfo ob_emlrtDCI = {
    142,                                     /* lineNo */
    24,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    1                                    /* checkKind */
};

static emlrtDCInfo pb_emlrtDCI = {
    142,                                     /* lineNo */
    24,                                      /* colNo */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    4                                    /* checkKind */
};

static emlrtBCInfo le_emlrtBCI = {
    0,                                       /* iFirst */
    6,                                       /* iLast */
    200,                                     /* lineNo */
    41,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo me_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    174,                                     /* lineNo */
    26,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo ne_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    190,                                     /* lineNo */
    21,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo oe_emlrtBCI = {
    -1,                                      /* iFirst */
    -1,                                      /* iLast */
    178,                                     /* lineNo */
    17,                                      /* colNo */
    "",                                      /* aName */
    "RigidBodyTreeDynamics/inverseDynamics", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeDynamics.m", /* pName */
    0                                    /* checkKind */
};

/* Function Definitions */
int32_T c_RigidBodyTreeDynamics_inverse(
    const emlrtStack *sp, d_robotics_manip_internal_Rigid *robot,
    const real_T q_data[], int32_T q_size, const real_T qdot_data[],
    const real_T qddot_data[], const real_T fext_data[],
    const int32_T fext_size[2], real_T tau_data[])
{
  static const char_T varargin_1[5] = {'J', 'o', 'i', 'n', 't'};
  __m128d r;
  __m128d r1;
  c_robotics_manip_internal_Rigid *obj;
  cell_wrap_66 X_data[7];
  cell_wrap_66 Xtree_data[7];
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T aB_data[42];
  real_T f_data[42];
  real_T vB_data[42];
  real_T vJ_data[42];
  real_T S_data[36];
  real_T XDHOffset[36];
  real_T y_data[36];
  real_T TDHOffset[16];
  real_T R[9];
  real_T b_R[9];
  real_T a0[6];
  real_T qddoti_data[6];
  real_T tmp_data[6];
  real_T y[6];
  real_T c_R[3];
  real_T a_idx_0;
  real_T a_idx_1;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T nb;
  int32_T obj_Vector_size[2];
  int32_T tmp_size[2];
  int32_T y_size[2];
  int32_T X_size_idx_1_tmp;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T k;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  int32_T tau_size;
  char_T obj_Vector[200];
  char_T obj_Vector_data[200];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  a0[0] = 0.0;
  a0[1] = 0.0;
  a0[2] = 0.0;
  a0[3] = -robot->Gravity[0];
  a0[4] = -robot->Gravity[1];
  a0[5] = -robot->Gravity[2];
  nb = robot->NumBodies;
  if (!(nb <= 7.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &yb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(nb >= 0.0)) {
    emlrtNonNegativeCheckR2012b(nb, &pb_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (nb != muDoubleScalarFloor(nb)) {
    emlrtIntegerCheckR2012b(nb, &ob_emlrtDCI, (emlrtConstCTX)sp);
  }
  X_size_idx_1_tmp = (int32_T)nb;
  loop_ub_tmp = 6 * (int32_T)nb;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&vJ_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
    memset(&vB_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
    memset(&aB_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }
  tau_size = 6;
  for (i = 0; i < 6; i++) {
    tau_data[i] = 0.0;
  }
  for (k = 0; k < X_size_idx_1_tmp; k++) {
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (loop_ub_tmp = 0; loop_ub_tmp < 6; loop_ub_tmp++) {
      XDHOffset[loop_ub_tmp + 6 * loop_ub_tmp] = 1.0;
    }
    if (k > (int32_T)nb - 1) {
      emlrtDynamicBoundsCheckR2012b(k, 0, (int32_T)nb - 1, &ce_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    memcpy(&Xtree_data[k].f1[0], &XDHOffset[0], 36U * sizeof(real_T));
    if (k > (int32_T)nb - 1) {
      emlrtDynamicBoundsCheckR2012b(k, 0, (int32_T)nb - 1, &de_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    memcpy(&X_data[k].f1[0], &XDHOffset[0], 36U * sizeof(real_T));
  }
  for (b_i = 0; b_i < X_size_idx_1_tmp; b_i++) {
    real_T T[16];
    st.site = &iu_emlrtRSI;
    obj = robot->Bodies[b_i];
    if (obj->Index == 0.0) {
      b_st.site = &st_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &b_st, &d_emlrtRTEI,
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
          &varargin_1[0]);
    }
    st.site = &iu_emlrtRSI;
    c_rigidBodyJoint_get_MotionSubs(&st, &obj->JointInternal, S_data,
                                    obj_Vector_size);
    a_idx_0 = robot->PositionDoFMap[b_i];
    a_idx_1 = robot->PositionDoFMap[b_i + 7];
    b_idx_0 = robot->VelocityDoFMap[b_i];
    b_idx_1 = robot->VelocityDoFMap[b_i + 7];
    memset(&XDHOffset[0], 0, 36U * sizeof(real_T));
    for (k = 0; k < 6; k++) {
      XDHOffset[k + 6 * k] = 1.0;
    }
    if (a_idx_1 < a_idx_0) {
      st.site = &ju_emlrtRSI;
      obj = robot->Bodies[b_i];
      if (obj->Index == 0.0) {
        b_st.site = &st_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &b_st, &d_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      st.site = &ju_emlrtRSI;
      d_rigidBodyJoint_transformBodyT(&st, &obj->JointInternal, T);
      b_loop_ub = 1;
      qddoti_data[0] = 0.0;
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &id_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      for (i = 0; i < 6; i++) {
        vJ_data[i + 6 * b_i] = 0.0;
      }
    } else {
      real_T b_q_data[49];
      if (a_idx_0 > a_idx_1) {
        i = 0;
        k = 0;
      } else {
        if (a_idx_0 != (int32_T)muDoubleScalarFloor(a_idx_0)) {
          emlrtIntegerCheckR2012b(a_idx_0, &eb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)a_idx_0 < 1) || ((int32_T)a_idx_0 > q_size)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)a_idx_0, 1, q_size,
                                        &cd_emlrtBCI, (emlrtConstCTX)sp);
        }
        i = (int32_T)a_idx_0 - 1;
        if (a_idx_1 != (int32_T)muDoubleScalarFloor(a_idx_1)) {
          emlrtIntegerCheckR2012b(a_idx_1, &fb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)a_idx_1 < 1) || ((int32_T)a_idx_1 > q_size)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)a_idx_1, 1, q_size,
                                        &dd_emlrtBCI, (emlrtConstCTX)sp);
        }
        k = (int32_T)a_idx_1;
      }
      y_size[0] = 1;
      loop_ub_tmp = k - i;
      y_size[1] = loop_ub_tmp;
      st.site = &ku_emlrtRSI;
      indexShapeCheck(&st, q_size, y_size);
      if (b_idx_0 > b_idx_1) {
        k = 0;
        i1 = 0;
      } else {
        if (b_idx_0 != (int32_T)muDoubleScalarFloor(b_idx_0)) {
          emlrtIntegerCheckR2012b(b_idx_0, &gb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)b_idx_0 < 1) || ((int32_T)b_idx_0 > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_0, 1, 6, &ed_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        k = (int32_T)b_idx_0 - 1;
        if (b_idx_1 != (int32_T)muDoubleScalarFloor(b_idx_1)) {
          emlrtIntegerCheckR2012b(b_idx_1, &hb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, 6, &fd_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        i1 = (int32_T)b_idx_1;
      }
      y_size[0] = 1;
      loop_ub = i1 - k;
      y_size[1] = loop_ub;
      st.site = &lu_emlrtRSI;
      indexShapeCheck(&st, 6, y_size);
      tmp_size[0] = 1;
      tmp_size[1] = loop_ub;
      for (i1 = 0; i1 < loop_ub; i1++) {
        tmp_data[i1] = qdot_data[k + i1];
      }
      if (b_idx_0 > b_idx_1) {
        k = 0;
        i1 = 0;
      } else {
        if (b_idx_0 != (int32_T)muDoubleScalarFloor(b_idx_0)) {
          emlrtIntegerCheckR2012b(b_idx_0, &ib_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)b_idx_0 < 1) || ((int32_T)b_idx_0 > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_0, 1, 6, &gd_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        k = (int32_T)b_idx_0 - 1;
        if (b_idx_1 != (int32_T)muDoubleScalarFloor(b_idx_1)) {
          emlrtIntegerCheckR2012b(b_idx_1, &jb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, 6, &hd_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        i1 = (int32_T)b_idx_1;
      }
      y_size[0] = 1;
      b_loop_ub = i1 - k;
      y_size[1] = b_loop_ub;
      st.site = &mu_emlrtRSI;
      indexShapeCheck(&st, 6, y_size);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        qddoti_data[i1] = qddot_data[k + i1];
      }
      st.site = &nu_emlrtRSI;
      obj = robot->Bodies[b_i];
      if (obj->Index == 0.0) {
        b_st.site = &st_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &b_st, &d_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      for (k = 0; k < loop_ub_tmp; k++) {
        b_q_data[k] = q_data[i + k];
      }
      st.site = &nu_emlrtRSI;
      c_rigidBodyJoint_transformBodyT(&st, &obj->JointInternal, b_q_data,
                                      loop_ub_tmp, T);
      st.site = &ou_emlrtRSI;
      obj = robot->Bodies[b_i];
      if (obj->Index == 0.0) {
        b_st.site = &st_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &b_st, &d_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      for (i = 0; i < 16; i++) {
        TDHOffset[i] = obj->JointInternal.ChildToJointTransform[i];
      }
      for (i = 0; i < 3; i++) {
        R[3 * i] = TDHOffset[i];
        R[3 * i + 1] = TDHOffset[i + 4];
        R[3 * i + 2] = TDHOffset[i + 8];
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
        a_idx_0 = b_R[i] * TDHOffset[12];
        loop_ub_tmp = i << 2;
        TDHOffset[loop_ub_tmp] = R[3 * i];
        a_idx_0 += b_R[i + 3] * TDHOffset[13];
        TDHOffset[loop_ub_tmp + 1] = R[3 * i + 1];
        a_idx_0 += b_R[i + 6] * TDHOffset[14];
        TDHOffset[loop_ub_tmp + 2] = R[3 * i + 2];
        c_R[i] = a_idx_0;
      }
      TDHOffset[12] = c_R[0];
      TDHOffset[13] = c_R[1];
      TDHOffset[14] = c_R[2];
      TDHOffset[3] = 0.0;
      TDHOffset[7] = 0.0;
      TDHOffset[11] = 0.0;
      TDHOffset[15] = 1.0;
      R[0] = 0.0;
      R[3] = -c_R[2];
      R[6] = c_R[1];
      R[1] = c_R[2];
      R[4] = 0.0;
      R[7] = -c_R[0];
      R[2] = -c_R[1];
      R[5] = c_R[0];
      R[8] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[i];
        a_idx_1 = R[i + 3];
        b_idx_0 = R[i + 6];
        for (k = 0; k < 3; k++) {
          i1 = k << 2;
          b_R[i + 3 * k] =
              (a_idx_0 * TDHOffset[i1] + a_idx_1 * TDHOffset[i1 + 1]) +
              b_idx_0 * TDHOffset[i1 + 2];
          XDHOffset[k + 6 * i] = TDHOffset[k + (i << 2)];
          XDHOffset[k + 6 * (i + 3)] = 0.0;
        }
      }
      for (i = 0; i < 3; i++) {
        XDHOffset[6 * i + 3] = b_R[3 * i];
        loop_ub_tmp = i << 2;
        k = 6 * (i + 3);
        XDHOffset[k + 3] = TDHOffset[loop_ub_tmp];
        XDHOffset[6 * i + 4] = b_R[3 * i + 1];
        XDHOffset[k + 4] = TDHOffset[loop_ub_tmp + 1];
        XDHOffset[6 * i + 5] = b_R[3 * i + 2];
        XDHOffset[k + 5] = TDHOffset[loop_ub_tmp + 2];
      }
      st.site = &pu_emlrtRSI;
      b_st.site = &je_emlrtRSI;
      e_mtimes(XDHOffset, S_data, obj_Vector_size, y_data, y_size);
      st.site = &pu_emlrtRSI;
      b_st.site = &ke_emlrtRSI;
      dynamic_size_checks(&b_st, tmp_size, y_size[1], loop_ub);
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &me_emlrtBCI,
                                      &st);
      }
      b_st.site = &je_emlrtRSI;
      f_mtimes(y_data, y_size, tmp_data, tmp_size, &vJ_data[6 * b_i]);
    }
    for (i = 0; i < 3; i++) {
      R[3 * i] = T[i];
      R[3 * i + 1] = T[i + 4];
      R[3 * i + 2] = T[i + 8];
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
    a_idx_0 = T[12];
    a_idx_1 = T[13];
    b_idx_0 = T[14];
    for (i = 0; i < 3; i++) {
      loop_ub_tmp = i << 2;
      TDHOffset[loop_ub_tmp] = R[3 * i];
      TDHOffset[loop_ub_tmp + 1] = R[3 * i + 1];
      TDHOffset[loop_ub_tmp + 2] = R[3 * i + 2];
      TDHOffset[i + 12] =
          (b_R[i] * a_idx_0 + b_R[i + 3] * a_idx_1) + b_R[i + 6] * b_idx_0;
    }
    TDHOffset[3] = 0.0;
    TDHOffset[7] = 0.0;
    TDHOffset[11] = 0.0;
    TDHOffset[15] = 1.0;
    R[0] = 0.0;
    R[3] = -TDHOffset[14];
    R[6] = TDHOffset[13];
    R[1] = TDHOffset[14];
    R[4] = 0.0;
    R[7] = -TDHOffset[12];
    R[2] = -TDHOffset[13];
    R[5] = TDHOffset[12];
    R[8] = 0.0;
    for (i = 0; i < 3; i++) {
      a_idx_0 = R[i];
      a_idx_1 = R[i + 3];
      b_idx_0 = R[i + 6];
      for (k = 0; k < 3; k++) {
        i1 = k << 2;
        b_R[i + 3 * k] =
            (a_idx_0 * TDHOffset[i1] + a_idx_1 * TDHOffset[i1 + 1]) +
            b_idx_0 * TDHOffset[i1 + 2];
      }
    }
    for (i = 0; i < 3; i++) {
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      k = i << 2;
      X_data[b_i].f1[6 * i] = TDHOffset[k];
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[6 * i + 1] = TDHOffset[k + 1];
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[6 * i + 2] = TDHOffset[k + 2];
    }
    for (i = 0; i < 3; i++) {
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      k = 6 * (i + 3);
      X_data[b_i].f1[k] = 0.0;
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[k + 1] = 0.0;
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[k + 2] = 0.0;
    }
    for (i = 0; i < 3; i++) {
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[6 * i + 3] = b_R[3 * i];
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[6 * i + 4] = b_R[3 * i + 1];
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[6 * i + 5] = b_R[3 * i + 2];
    }
    for (i = 0; i < 3; i++) {
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      k = i << 2;
      i1 = 6 * (i + 3);
      X_data[b_i].f1[i1 + 3] = TDHOffset[k];
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[i1 + 4] = TDHOffset[k + 1];
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &oe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      X_data[b_i].f1[i1 + 5] = TDHOffset[k + 2];
    }
    b_idx_1 = robot->Bodies[b_i]->ParentIndex;
    if (b_idx_1 > 0.0) {
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &fe_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (b_idx_1 != (int32_T)muDoubleScalarFloor(b_idx_1)) {
        emlrtIntegerCheckR2012b(b_idx_1, &kb_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > (int32_T)nb)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, (int32_T)nb,
                                      &kd_emlrtBCI, (emlrtConstCTX)sp);
      }
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &jd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      for (i = 0; i < 6; i++) {
        a_idx_0 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += X_data[b_i].f1[i + 6 * k] *
                     vB_data[k + 6 * ((int32_T)b_idx_1 - 1)];
        }
        y[i] = vJ_data[i + 6 * b_i] + a_idx_0;
      }
      for (i = 0; i < 6; i++) {
        vB_data[i + 6 * b_i] = y[i];
      }
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &he_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > (int32_T)nb)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, (int32_T)nb,
                                      &ld_emlrtBCI, (emlrtConstCTX)sp);
      }
      st.site = &qu_emlrtRSI;
      b_st.site = &je_emlrtRSI;
      e_mtimes(XDHOffset, S_data, obj_Vector_size, y_data, y_size);
      st.site = &qu_emlrtRSI;
      b_st.site = &ke_emlrtRSI;
      if (b_loop_ub != y_size[1]) {
        if (b_loop_ub == 1) {
          emlrtErrorWithMessageIdR2018a(
              &b_st, &s_emlrtRTEI,
              "Coder:toolbox:mtimes_noDynamicScalarExpansion",
              "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
      }
      b_st.site = &je_emlrtRSI;
      g_mtimes(y_data, y_size, qddoti_data, b_loop_ub, y);
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &md_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      R[0] = 0.0;
      i = 6 * b_i + 2;
      a_idx_0 = vB_data[i];
      R[3] = -a_idx_0;
      a_idx_1 = vB_data[6 * b_i + 1];
      R[6] = a_idx_1;
      R[1] = a_idx_0;
      R[4] = 0.0;
      a_idx_0 = vB_data[6 * b_i];
      R[7] = -a_idx_0;
      R[2] = -a_idx_1;
      R[5] = a_idx_0;
      R[8] = 0.0;
      S_data[3] = 0.0;
      a_idx_0 = vB_data[6 * b_i + 5];
      S_data[9] = -a_idx_0;
      k = 6 * b_i + 4;
      a_idx_1 = vB_data[k];
      S_data[15] = a_idx_1;
      S_data[4] = a_idx_0;
      S_data[10] = 0.0;
      a_idx_0 = vB_data[6 * b_i + 3];
      S_data[16] = -a_idx_0;
      S_data[5] = -a_idx_1;
      S_data[11] = a_idx_0;
      S_data[17] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        a_idx_0 = R[3 * i1];
        S_data[6 * i1] = a_idx_0;
        loop_ub_tmp = 6 * (i1 + 3);
        S_data[loop_ub_tmp] = 0.0;
        S_data[loop_ub_tmp + 3] = a_idx_0;
        a_idx_0 = R[3 * i1 + 1];
        S_data[6 * i1 + 1] = a_idx_0;
        S_data[loop_ub_tmp + 1] = 0.0;
        S_data[loop_ub_tmp + 4] = a_idx_0;
        a_idx_0 = R[3 * i1 + 2];
        S_data[6 * i1 + 2] = a_idx_0;
        S_data[loop_ub_tmp + 2] = 0.0;
        S_data[loop_ub_tmp + 5] = a_idx_0;
      }
      for (i1 = 0; i1 < 6; i1++) {
        a_idx_0 = 0.0;
        a_idx_1 = 0.0;
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          loop_ub_tmp = i1 + 6 * loop_ub;
          a_idx_0 += X_data[b_i].f1[loop_ub_tmp] *
                     aB_data[loop_ub + 6 * ((int32_T)b_idx_1 - 1)];
          a_idx_1 += S_data[loop_ub_tmp] * vJ_data[loop_ub + 6 * b_i];
        }
        qddoti_data[i1] = a_idx_1;
        tmp_data[i1] = a_idx_0 + y[i1];
      }
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &nd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      r = _mm_loadu_pd(&tmp_data[0]);
      r1 = _mm_loadu_pd(&qddoti_data[0]);
      _mm_storeu_pd(&aB_data[6 * b_i], _mm_add_pd(r, r1));
      r = _mm_loadu_pd(&tmp_data[2]);
      r1 = _mm_loadu_pd(&qddoti_data[2]);
      _mm_storeu_pd(&aB_data[i], _mm_add_pd(r, r1));
      r = _mm_loadu_pd(&tmp_data[4]);
      r1 = _mm_loadu_pd(&qddoti_data[4]);
      _mm_storeu_pd(&aB_data[k], _mm_add_pd(r, r1));
      if (((int32_T)b_idx_1 - 1 < 0) ||
          ((int32_T)b_idx_1 - 1 > (int32_T)nb - 1)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1 - 1, 0, (int32_T)nb - 1,
                                      &ie_emlrtBCI, (emlrtConstCTX)sp);
      }
      R[0] = 0.0;
      R[3] = -T[14];
      R[6] = T[13];
      R[1] = T[14];
      R[4] = 0.0;
      R[7] = -T[12];
      R[2] = -T[13];
      R[5] = T[12];
      R[8] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[i];
        a_idx_1 = R[i + 3];
        b_idx_0 = R[i + 6];
        for (k = 0; k < 3; k++) {
          i1 = k << 2;
          b_R[i + 3 * k] =
              (a_idx_0 * T[i1] + a_idx_1 * T[i1 + 1]) + b_idx_0 * T[i1 + 2];
          XDHOffset[k + 6 * i] = T[k + (i << 2)];
          XDHOffset[k + 6 * (i + 3)] = 0.0;
        }
      }
      for (i = 0; i < 3; i++) {
        XDHOffset[6 * i + 3] = b_R[3 * i];
        loop_ub_tmp = i << 2;
        k = 6 * (i + 3);
        XDHOffset[k + 3] = T[loop_ub_tmp];
        XDHOffset[6 * i + 4] = b_R[3 * i + 1];
        XDHOffset[k + 4] = T[loop_ub_tmp + 1];
        XDHOffset[6 * i + 5] = b_R[3 * i + 2];
        XDHOffset[k + 5] = T[loop_ub_tmp + 2];
      }
      for (i = 0; i < 6; i++) {
        for (k = 0; k < 6; k++) {
          a_idx_0 = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            a_idx_0 += Xtree_data[(int32_T)b_idx_1 - 1].f1[i + 6 * i1] *
                       XDHOffset[i1 + 6 * k];
          }
          S_data[i + 6 * k] = a_idx_0;
        }
      }
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ke_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      memcpy(&Xtree_data[b_i].f1[0], &S_data[0], 36U * sizeof(real_T));
    } else {
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &od_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      for (i = 0; i < 6; i++) {
        loop_ub_tmp = i + 6 * b_i;
        vB_data[loop_ub_tmp] = vJ_data[loop_ub_tmp];
      }
      if (b_i > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ge_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      st.site = &ru_emlrtRSI;
      b_st.site = &je_emlrtRSI;
      e_mtimes(XDHOffset, S_data, obj_Vector_size, y_data, y_size);
      st.site = &ru_emlrtRSI;
      b_st.site = &ke_emlrtRSI;
      if (b_loop_ub != y_size[1]) {
        if (b_loop_ub == 1) {
          emlrtErrorWithMessageIdR2018a(
              &b_st, &s_emlrtRTEI,
              "Coder:toolbox:mtimes_noDynamicScalarExpansion",
              "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &q_emlrtRTEI, "MATLAB:innerdim",
                                        "MATLAB:innerdim", 0);
        }
      }
      b_st.site = &je_emlrtRSI;
      g_mtimes(y_data, y_size, qddoti_data, b_loop_ub, y);
      if (b_i + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &pd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      for (i = 0; i < 6; i++) {
        a_idx_0 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += X_data[b_i].f1[i + 6 * k] * a0[k];
        }
        aB_data[i + 6 * b_i] = a_idx_0 + y[i];
      }
      R[0] = 0.0;
      R[3] = -T[14];
      R[6] = T[13];
      R[1] = T[14];
      R[4] = 0.0;
      R[7] = -T[12];
      R[2] = -T[13];
      R[5] = T[12];
      R[8] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[i];
        a_idx_1 = R[i + 3];
        b_idx_0 = R[i + 6];
        for (k = 0; k < 3; k++) {
          i1 = k << 2;
          b_R[i + 3 * k] =
              (a_idx_0 * T[i1] + a_idx_1 * T[i1 + 1]) + b_idx_0 * T[i1 + 2];
        }
      }
      for (i = 0; i < 3; i++) {
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        k = i << 2;
        Xtree_data[b_i].f1[6 * i] = T[k];
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[6 * i + 1] = T[k + 1];
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[6 * i + 2] = T[k + 2];
      }
      for (i = 0; i < 3; i++) {
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        k = 6 * (i + 3);
        Xtree_data[b_i].f1[k] = 0.0;
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[k + 1] = 0.0;
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[k + 2] = 0.0;
      }
      for (i = 0; i < 3; i++) {
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[6 * i + 3] = b_R[3 * i];
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[6 * i + 4] = b_R[3 * i + 1];
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[6 * i + 5] = b_R[3 * i + 2];
      }
      for (i = 0; i < 3; i++) {
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        k = i << 2;
        i1 = 6 * (i + 3);
        Xtree_data[b_i].f1[i1 + 3] = T[k];
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[i1 + 4] = T[k + 1];
        if (b_i > (int32_T)nb - 1) {
          emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &ne_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        Xtree_data[b_i].f1[i1 + 5] = T[k + 2];
      }
    }
    for (i = 0; i < 36; i++) {
      XDHOffset[i] = robot->Bodies[b_i]->SpatialInertia[i];
    }
    if (b_i + 1 > (int32_T)nb) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &qd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (b_i > (int32_T)nb - 1) {
      emlrtDynamicBoundsCheckR2012b(b_i, 0, (int32_T)nb - 1, &je_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (b_i + 1 > (int32_T)nb) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &rd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (b_i + 1 > fext_size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, fext_size[1], &td_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (b_i + 1 > (int32_T)nb) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &sd_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    R[0] = 0.0;
    a_idx_0 = vB_data[6 * b_i + 2];
    R[3] = -a_idx_0;
    a_idx_1 = vB_data[6 * b_i + 1];
    R[6] = a_idx_1;
    R[1] = a_idx_0;
    R[4] = 0.0;
    a_idx_0 = vB_data[6 * b_i];
    R[7] = -a_idx_0;
    R[2] = -a_idx_1;
    R[5] = a_idx_0;
    R[8] = 0.0;
    S_data[18] = 0.0;
    a_idx_0 = vB_data[6 * b_i + 5];
    S_data[24] = -a_idx_0;
    a_idx_1 = vB_data[6 * b_i + 4];
    S_data[30] = a_idx_1;
    S_data[19] = a_idx_0;
    S_data[25] = 0.0;
    a_idx_0 = vB_data[6 * b_i + 3];
    S_data[31] = -a_idx_0;
    S_data[20] = -a_idx_1;
    S_data[26] = a_idx_0;
    S_data[32] = 0.0;
    for (i = 0; i < 3; i++) {
      a_idx_0 = R[3 * i];
      S_data[6 * i] = a_idx_0;
      S_data[6 * i + 3] = 0.0;
      loop_ub_tmp = 6 * (i + 3);
      S_data[loop_ub_tmp + 3] = a_idx_0;
      a_idx_0 = R[3 * i + 1];
      S_data[6 * i + 1] = a_idx_0;
      S_data[6 * i + 4] = 0.0;
      S_data[loop_ub_tmp + 4] = a_idx_0;
      a_idx_0 = R[3 * i + 2];
      S_data[6 * i + 2] = a_idx_0;
      S_data[6 * i + 5] = 0.0;
      S_data[loop_ub_tmp + 5] = a_idx_0;
    }
    for (i = 0; i < 6; i++) {
      a_idx_0 = 0.0;
      a_idx_1 = 0.0;
      for (k = 0; k < 6; k++) {
        b_idx_0 = XDHOffset[i + 6 * k];
        i1 = k + 6 * b_i;
        a_idx_0 += b_idx_0 * vB_data[i1];
        a_idx_1 += b_idx_0 * aB_data[i1];
      }
      tmp_data[i] = a_idx_1;
      y[i] = a_idx_0;
    }
    for (i = 0; i < 6; i++) {
      a_idx_0 = 0.0;
      for (k = 0; k < 6; k++) {
        a_idx_0 += S_data[i + 6 * k] * y[k];
      }
      qddoti_data[i] = a_idx_0;
    }
    if (b_i + 1 > (int32_T)nb) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, (int32_T)nb, &ud_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    for (i = 0; i < 6; i++) {
      a_idx_0 = 0.0;
      for (k = 0; k < 6; k++) {
        a_idx_0 += Xtree_data[b_i].f1[k + 6 * i] * fext_data[k + 6 * b_i];
      }
      f_data[i + 6 * b_i] = (tmp_data[i] + qddoti_data[i]) - a_idx_0;
    }
  }
  emlrtForLoopVectorCheckR2021a(nb, -1.0, 1.0, mxDOUBLE_CLASS, (int32_T)nb,
                                &ac_emlrtRTEI, (emlrtConstCTX)sp);
  if (X_size_idx_1_tmp - 1 >= 0) {
    obj_Vector_size[0] = 1;
  }
  for (b_i = 0; b_i < X_size_idx_1_tmp; b_i++) {
    loop_ub = ((int32_T)nb - b_i) - 1;
    st.site = &su_emlrtRSI;
    if (loop_ub < 0) {
      emlrtDynamicBoundsCheckR2012b(loop_ub, 0, 6, &le_emlrtBCI, &st);
    }
    obj = robot->Bodies[loop_ub];
    if (obj->Index == 0.0) {
      b_st.site = &st_emlrtRSI;
      emlrtErrorWithMessageIdR2018a(
          &b_st, &d_emlrtRTEI,
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
          "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
          &varargin_1[0]);
    }
    st.site = &su_emlrtRSI;
    b_st.site = &pb_emlrtRSI;
    a_idx_0 = obj->JointInternal.TypeInternal.Length;
    for (i = 0; i < 200; i++) {
      obj_Vector[i] = obj->JointInternal.TypeInternal.Vector[i];
    }
    if (a_idx_0 < 1.0) {
      loop_ub_tmp = 0;
    } else {
      if (a_idx_0 != (int32_T)muDoubleScalarFloor(a_idx_0)) {
        emlrtIntegerCheckR2012b(a_idx_0, &c_emlrtDCI, &b_st);
      }
      if (((int32_T)a_idx_0 < 1) || ((int32_T)a_idx_0 > 200)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)a_idx_0, 1, 200, &f_emlrtBCI,
                                      &b_st);
      }
      loop_ub_tmp = (int32_T)a_idx_0;
    }
    obj_Vector_size[1] = loop_ub_tmp;
    if (loop_ub_tmp - 1 >= 0) {
      memcpy(&obj_Vector_data[0], &obj_Vector[0],
             (uint32_T)loop_ub_tmp * sizeof(char_T));
    }
    if (!b_strcmp(obj_Vector_data, obj_Vector_size)) {
      st.site = &tu_emlrtRSI;
      obj = robot->Bodies[loop_ub];
      if (obj->Index == 0.0) {
        b_st.site = &st_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &b_st, &d_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      for (i = 0; i < 16; i++) {
        TDHOffset[i] = obj->JointInternal.ChildToJointTransform[i];
      }
      for (i = 0; i < 3; i++) {
        R[3 * i] = TDHOffset[i];
        R[3 * i + 1] = TDHOffset[i + 4];
        R[3 * i + 2] = TDHOffset[i + 8];
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
        a_idx_0 = b_R[i] * TDHOffset[12];
        loop_ub_tmp = i << 2;
        TDHOffset[loop_ub_tmp] = R[3 * i];
        a_idx_0 += b_R[i + 3] * TDHOffset[13];
        TDHOffset[loop_ub_tmp + 1] = R[3 * i + 1];
        a_idx_0 += b_R[i + 6] * TDHOffset[14];
        TDHOffset[loop_ub_tmp + 2] = R[3 * i + 2];
        c_R[i] = a_idx_0;
      }
      TDHOffset[12] = c_R[0];
      TDHOffset[13] = c_R[1];
      TDHOffset[14] = c_R[2];
      TDHOffset[3] = 0.0;
      TDHOffset[7] = 0.0;
      TDHOffset[11] = 0.0;
      TDHOffset[15] = 1.0;
      st.site = &uu_emlrtRSI;
      b_st.site = &uu_emlrtRSI;
      obj = robot->Bodies[loop_ub];
      if (obj->Index == 0.0) {
        c_st.site = &st_emlrtRSI;
        emlrtErrorWithMessageIdR2018a(
            &c_st, &d_emlrtRTEI,
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase",
            "robotics:robotmanip:rigidbody:NoSuchPropertyForBase", 3, 4, 5,
            &varargin_1[0]);
      }
      b_st.site = &uu_emlrtRSI;
      c_rigidBodyJoint_get_MotionSubs(&b_st, &obj->JointInternal, y_data,
                                      y_size);
      R[0] = 0.0;
      R[3] = -c_R[2];
      R[6] = c_R[1];
      R[1] = c_R[2];
      R[4] = 0.0;
      R[7] = -c_R[0];
      R[2] = -c_R[1];
      R[5] = c_R[0];
      R[8] = 0.0;
      for (i = 0; i < 3; i++) {
        a_idx_0 = R[i];
        a_idx_1 = R[i + 3];
        b_idx_0 = R[i + 6];
        for (k = 0; k < 3; k++) {
          i1 = k << 2;
          b_R[i + 3 * k] =
              (a_idx_0 * TDHOffset[i1] + a_idx_1 * TDHOffset[i1 + 1]) +
              b_idx_0 * TDHOffset[i1 + 2];
          XDHOffset[k + 6 * i] = TDHOffset[k + (i << 2)];
          XDHOffset[k + 6 * (i + 3)] = 0.0;
        }
      }
      for (i = 0; i < 3; i++) {
        XDHOffset[6 * i + 3] = b_R[3 * i];
        loop_ub_tmp = i << 2;
        k = 6 * (i + 3);
        XDHOffset[k + 3] = TDHOffset[loop_ub_tmp];
        XDHOffset[6 * i + 4] = b_R[3 * i + 1];
        XDHOffset[k + 4] = TDHOffset[loop_ub_tmp + 1];
        XDHOffset[6 * i + 5] = b_R[3 * i + 2];
        XDHOffset[k + 5] = TDHOffset[loop_ub_tmp + 2];
      }
      b_st.site = &je_emlrtRSI;
      e_mtimes(XDHOffset, y_data, y_size, S_data, obj_Vector_size);
      st.site = &vu_emlrtRSI;
      if (loop_ub + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, (int32_T)nb, &vd_emlrtBCI,
                                      &st);
      }
      b_st.site = &je_emlrtRSI;
      b_loop_ub =
          h_mtimes(S_data, obj_Vector_size, &f_data[6 * loop_ub], qddoti_data);
      b_idx_0 = robot->VelocityDoFMap[loop_ub];
      b_idx_1 = robot->VelocityDoFMap[loop_ub + 7];
      if (b_idx_0 > b_idx_1) {
        i = 0;
        k = 0;
      } else {
        if (b_idx_0 != (int32_T)muDoubleScalarFloor(b_idx_0)) {
          emlrtIntegerCheckR2012b(b_idx_0, &mb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)b_idx_0 < 1) || ((int32_T)b_idx_0 > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_0, 1, 6, &ae_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        i = (int32_T)b_idx_0 - 1;
        if (b_idx_1 != (int32_T)muDoubleScalarFloor(b_idx_1)) {
          emlrtIntegerCheckR2012b(b_idx_1, &nb_emlrtDCI, (emlrtConstCTX)sp);
        }
        if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > 6)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, 6, &be_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        k = (int32_T)b_idx_1;
      }
      loop_ub_tmp = k - i;
      if (loop_ub_tmp != b_loop_ub) {
        emlrtSubAssignSizeCheck1dR2017a(loop_ub_tmp, b_loop_ub, &eb_emlrtECI,
                                        (emlrtConstCTX)sp);
      }
      for (k = 0; k < loop_ub_tmp; k++) {
        tau_data[i + k] = qddoti_data[k];
      }
    }
    b_idx_1 = robot->Bodies[loop_ub]->ParentIndex;
    if (b_idx_1 > 0.0) {
      if (loop_ub > (int32_T)nb - 1) {
        emlrtDynamicBoundsCheckR2012b(loop_ub, 0, (int32_T)nb - 1, &ee_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (loop_ub + 1 > (int32_T)nb) {
        emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, (int32_T)nb, &xd_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > (int32_T)nb)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, (int32_T)nb,
                                      &wd_emlrtBCI, (emlrtConstCTX)sp);
      }
      if (b_idx_1 != (int32_T)muDoubleScalarFloor(b_idx_1)) {
        emlrtIntegerCheckR2012b(b_idx_1, &lb_emlrtDCI, (emlrtConstCTX)sp);
      }
      if (((int32_T)b_idx_1 < 1) || ((int32_T)b_idx_1 > (int32_T)nb)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)b_idx_1, 1, (int32_T)nb,
                                      &yd_emlrtBCI, (emlrtConstCTX)sp);
      }
      for (i = 0; i < 6; i++) {
        a_idx_0 = 0.0;
        for (k = 0; k < 6; k++) {
          a_idx_0 += X_data[loop_ub].f1[k + 6 * i] * f_data[k + 6 * loop_ub];
        }
        y[i] = f_data[i + 6 * ((int32_T)b_idx_1 - 1)] + a_idx_0;
      }
      for (i = 0; i < 6; i++) {
        f_data[i + 6 * ((int32_T)b_idx_1 - 1)] = y[i];
      }
    }
  }
  return tau_size;
}

/* End of code generation (RigidBodyTreeDynamics.c) */
