/*
 * _coder_run_p2p_api.c
 *
 * Code generation for function '_coder_run_p2p_api'
 *
 */

/* Include files */
#include "_coder_run_p2p_api.h"
#include "rt_nonfinite.h"
#include "run_p2p.h"
#include "run_p2p_data.h"
#include "run_p2p_types.h"

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(real_T u[303]);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[6];

static const mxArray *c_emlrt_marshallOut(real_T u[606]);

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6];

static p2pMode e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier);

static p2pMode f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static trajMode g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier);

static trajMode h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(real_T u[303])
{
  static const int32_T b_iv[2] = {0, 0};
  static const int32_T b_iv1[2] = {101, 3};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&b_iv[0], mxDOUBLE_CLASS,
                              mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &b_iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[6];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *c_emlrt_marshallOut(real_T u[606])
{
  static const int32_T b_iv[2] = {0, 0};
  static const int32_T b_iv1[2] = {6, 101};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&b_iv[0], mxDOUBLE_CLASS,
                              mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &b_iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static p2pMode e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  p2pMode y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static p2pMode f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
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

static trajMode g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  trajMode y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static trajMode h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
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

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6]
{
  static const int32_T dims[2] = {3, 2};
  real_T(*ret)[6];
  int32_T b_iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &b_iv[0]);
  ret = (real_T(*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void run_p2p_api(run_p2pStackData *SD, const mxArray *const prhs[7],
                 int32_T nlhs, const mxArray *plhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*jointTorq)[606];
  real_T(*qInterp)[606];
  real_T(*qdInterp)[606];
  real_T(*pose)[303];
  real_T(*orientations)[6];
  real_T(*waypointAccels)[6];
  real_T(*waypointJerks)[6];
  real_T(*waypointVels)[6];
  real_T(*wpts)[6];
  p2pMode runP2pMode;
  trajMode runTrajMode;
  st.tls = emlrtRootTLSGlobal;
  pose = (real_T(*)[303])mxMalloc(sizeof(real_T[303]));
  qInterp = (real_T(*)[606])mxMalloc(sizeof(real_T[606]));
  qdInterp = (real_T(*)[606])mxMalloc(sizeof(real_T[606]));
  jointTorq = (real_T(*)[606])mxMalloc(sizeof(real_T[606]));
  /* Marshall function inputs */
  wpts = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "wpts");
  orientations = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "orientations");
  runP2pMode = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "runP2pMode");
  runTrajMode = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "runTrajMode");
  waypointVels = c_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "waypointVels");
  waypointAccels =
      c_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "waypointAccels");
  waypointJerks = c_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "waypointJerks");
  /* Invoke the target function */
  run_p2p(SD, &st, *wpts, *orientations, runP2pMode, runTrajMode, *waypointVels,
          *waypointAccels, *waypointJerks, *pose, *qInterp, *qdInterp,
          *jointTorq);
  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*pose);
  if (nlhs > 1) {
    plhs[1] = c_emlrt_marshallOut(*qInterp);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(*qdInterp);
  }
  if (nlhs > 3) {
    plhs[3] = c_emlrt_marshallOut(*jointTorq);
  }
}

/* End of code generation (_coder_run_p2p_api.c) */
