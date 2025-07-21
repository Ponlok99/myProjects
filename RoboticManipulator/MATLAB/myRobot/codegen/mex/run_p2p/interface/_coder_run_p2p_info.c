/*
 * _coder_run_p2p_info.c
 *
 * Code generation for function 'run_p2p'
 *
 */

/* Include files */
#include "_coder_run_p2p_info.h"
#include "run_p2p_data.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[7] = {
      "789ced583f6fd3401cbda016588048081626a40a8482221144088c75f3c72d49d3a4a555"
      "e22a75ec0b7162fbaccba549374616041f80994fc0c037a06c2c9d60"
      "66ca8a04031271ed7312c3c516491da5ca6fb9fcfcea7bef5e7acfce81109f0d0100ae02"
      "ab7edeb2c62b761fb6c70b60b4dc78c81e975d3d70ae2f8ddc47f137",
      "f628219dc02eb11a5dd4a073a78c34451775b27d644080610ba987503e456a8a0ab7150d"
      "16879b9cd969a921c8694cc8fcccd5a1d42cb63580ebad814275b871"
      "fc78c558ef924f3fee30fc08bbf072727feda9102f5430aa22a24815116b827654303b01"
      "b7f58a1133a2da40d7c184ba2e33755908c162238b6438299fbb583e",
      "d09a96ef773df8285ece8df19d9ad037decb876b3e75b9c7c1df5bbebffef5f5140a8aaf"
      "77fbdbe720f968cd8aafcb98cfefffd54d065fd8851778b293e73a19"
      "bcc7c552d9cca33ad73ce4b9818ebc078f970ec0e8839aff0be37ebf3e6a8cf9c32ebccc"
      "73fb2b9a4854b18a11222b024148ada2aed0aa8b18ca82bd6d5bd687",
      "3651d49610712e462484a11051facf16ac8baab0266a06945753e962278d4559813ac963"
      "d4801251906ec6abb3beb713aeef9ec7fa284ea5464da5512a349a7b"
      "962f9a0f3a9cec2a24a58a2fe8b36956797cfc9f7c74feaa071fc5cb7c6e8adff7df3646"
      "b5d1751d30744f2b7f4e3e9e049ae72f6bd79783e4a335af797e83c1",
      "1776e1b1865a2f7661a9192fae27e23b5bbb0f0a994d707ef23cb8fd3dcd3c77f6376f5e"
      "a9891274efef458e8ff22d72fcdfeb5ae4b8558b1c1fcfe3a50330fa"
      "a0e63fafe7159798ba2ca44f67fe529f59ae06e6fbb8f30adb043fbecfeb79c571efddef"
      "20f968cd6b2efa3daf586de613a5cee33d231ddf7ab82e77a58ddda6",
      "9a5ce4e259e7a231a12ef779ba5b17c595a6a2c38a2d044c2f972f32f92d4446edaa0aa7"
      "f73e7e9fc9378a8ffd1e46ccb0de2483dadf9f7ac1e6e5f7f71f7e04"
      "c9476b5ef3d2ef7ba49e2a243663d527f1ce066fc47252b654c83f07f39f977f009110b6"
      "bd",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 7184U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 7);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("run_p2p"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(7.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString("D:\\6R_robotic_arm\\myRobot\\run_p2p.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739818.97832175926));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2712019 (R2024b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("rGQ5sFka4A2lc0PUFsThr"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  emlrtSetField(xResult, 0, "CoverageInfo",
                covrtSerializeInstanceData(&emlrtCoverageInstance));
  return xResult;
}

/* End of code generation (_coder_run_p2p_info.c) */
