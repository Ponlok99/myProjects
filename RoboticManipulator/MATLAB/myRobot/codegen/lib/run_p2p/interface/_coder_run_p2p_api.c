/*
 * File: _coder_run_p2p_api.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "_coder_run_p2p_api.h"
#include "_coder_run_p2p_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131659U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "run_p2p",                                            /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2][3];

static const mxArray *b_emlrt_marshallOut(real_T u[101][6]);

static p2pMode c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier);

static p2pMode d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static trajMode e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[2][3];

static const mxArray *emlrt_marshallOut(real_T u[3][101]);

static trajMode f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2][3];

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[2][3]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2][3]
{
  real_T(*y)[2][3];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : real_T u[101][6]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(real_T u[101][6])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {6, 101};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : p2pMode
 */
static p2pMode c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  p2pMode y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : p2pMode
 */
static p2pMode d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId)
{
  static const int32_T enumValues[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  static const int32_T dims = 0;
  static const char_T *enumNames[8] = {"moveJ",  "moveJI", "moveL", "moveJB",
                                       "moveLB", "moveJL", "moveA", "moveC"};
  p2pMode y;
  emlrtCheckEnumR2012b((emlrtConstCTX)sp, "p2pMode", 8,
                       (const char_T **)&enumNames[0], &enumValues[0]);
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, parentId, u, "p2pMode", false, 0U,
                          (const void *)&dims);
  y = (p2pMode)emlrtGetEnumElementR2009a(u, 0);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : trajMode
 */
static trajMode e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  trajMode y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const void *r
 * Return Type  : void
 */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[2][3]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[2][3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[2][3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : real_T u[3][101]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(real_T u[3][101])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {101, 3};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : trajMode
 */
static trajMode f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  static const int32_T enumValues[5] = {0, 1, 2, 3, 4};
  static const int32_T dims = 0;
  static const char_T *enumNames[5] = {"trapveltraj", "cubicpolytraj",
                                       "quinticpolytraj", "minjerkpolytraj",
                                       "minsnappolytraj"};
  trajMode y;
  emlrtCheckEnumR2012b((emlrtConstCTX)sp, "trajMode", 5,
                       (const char_T **)&enumNames[0], &enumValues[0]);
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, parentId, u, "trajMode", false, 0U,
                          (const void *)&dims);
  y = (trajMode)emlrtGetEnumElementR2009a(u, 0);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[2][3]
 */
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2][3]
{
  static const int32_T dims[2] = {3, 2};
  real_T(*ret)[2][3];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[2][3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[7]
 *                int32_T nlhs
 *                const mxArray *plhs[4]
 * Return Type  : void
 */
void run_p2p_api(const mxArray *const prhs[7], int32_T nlhs,
                 const mxArray *plhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*jointTorq)[101][6];
  real_T(*qInterp)[101][6];
  real_T(*qdInterp)[101][6];
  real_T(*pose)[3][101];
  real_T(*orientations)[2][3];
  real_T(*waypointAccels)[2][3];
  real_T(*waypointJerks)[2][3];
  real_T(*waypointVels)[2][3];
  real_T(*wpts)[2][3];
  p2pMode runP2pMode;
  trajMode runTrajMode;
  st.tls = emlrtRootTLSGlobal;
  pose = (real_T(*)[3][101])mxMalloc(sizeof(real_T[3][101]));
  qInterp = (real_T(*)[101][6])mxMalloc(sizeof(real_T[101][6]));
  qdInterp = (real_T(*)[101][6])mxMalloc(sizeof(real_T[101][6]));
  jointTorq = (real_T(*)[101][6])mxMalloc(sizeof(real_T[101][6]));
  /* Marshall function inputs */
  wpts = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "wpts");
  orientations = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "orientations");
  runP2pMode = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "runP2pMode");
  runTrajMode = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "runTrajMode");
  waypointVels = emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "waypointVels");
  waypointAccels = emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "waypointAccels");
  waypointJerks = emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "waypointJerks");
  /* Invoke the target function */
  run_p2p(*wpts, *orientations, runP2pMode, runTrajMode, *waypointVels,
          *waypointAccels, *waypointJerks, *pose, *qInterp, *qdInterp,
          *jointTorq);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*pose);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*qInterp);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(*qdInterp);
  }
  if (nlhs > 3) {
    plhs[3] = b_emlrt_marshallOut(*jointTorq);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void run_p2p_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  run_p2p_xil_terminate();
  run_p2p_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void run_p2p_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void run_p2p_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_run_p2p_api.c
 *
 * [EOF]
 */
