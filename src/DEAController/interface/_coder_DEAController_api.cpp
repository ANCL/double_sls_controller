//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_DEAController_api.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 25-Aug-2024 22:36:55
//

// Include Files
#include "_coder_DEAController_api.h"
#include "_coder_DEAController_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131626U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "DEAController",                                      // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *k,
                                   const char_T *identifier))[24];

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[24];

static const mxArray *b_emlrt_marshallOut(const real_T u[4]);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
                                   const char_T *identifier))[4];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static const mxArray *c_emlrt_marshallOut(const real_T u[24]);

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *ref,
                                   const char_T *identifier))[13];

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[13];

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *t,
                                 const char_T *identifier);

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *z,
                                 const char_T *identifier))[22];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[22];

static const mxArray *emlrt_marshallOut(const real_T u[3]);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[22];

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[24];

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[13];

static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

// Function Definitions
//
// Arguments    : const emlrtStack *sp
//                const mxArray *k
//                const char_T *identifier
// Return Type  : real_T (*)[24]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *k,
                                   const char_T *identifier))[24]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[24];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(k), &thisId);
  emlrtDestroyArray(&k);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[24]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[24]
{
  real_T(*y)[24];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[4]
// Return Type  : const mxArray *
//
static const mxArray *b_emlrt_marshallOut(const real_T u[4])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{1, 4};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *param
//                const char_T *identifier
// Return Type  : real_T (*)[4]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
                                   const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(param), &thisId);
  emlrtDestroyArray(&param);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[4]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[24]
// Return Type  : const mxArray *
//
static const mxArray *c_emlrt_marshallOut(const real_T u[24])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{6, 4};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *ref
//                const char_T *identifier
// Return Type  : real_T (*)[13]
//
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *ref,
                                   const char_T *identifier))[13]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[13];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(ref), &thisId);
  emlrtDestroyArray(&ref);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[13]
//
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[13]
{
  real_T(*y)[13];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *t
//                const char_T *identifier
// Return Type  : real_T
//
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *t,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(t), &thisId);
  emlrtDestroyArray(&t);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *z
//                const char_T *identifier
// Return Type  : real_T (*)[22]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *z,
                                 const char_T *identifier))[22]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[22];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(z), &thisId);
  emlrtDestroyArray(&z);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[22]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[22]
{
  real_T(*y)[22];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[3]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[3])
{
  static const int32_T i{0};
  static const int32_T i1{3};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[22]
//
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[22]
{
  static const int32_T dims[2]{1, 22};
  real_T(*ret)[22];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[22])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[24]
//
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[24]
{
  static const int32_T dims[2]{6, 4};
  real_T(*ret)[24];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[24])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[4]
//
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims[2]{1, 4};
  real_T(*ret)[4];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[13]
//
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[13]
{
  static const int32_T dims[2]{1, 13};
  real_T(*ret)[13];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[13])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const mxArray * const prhs[5]
//                int32_T nlhs
//                const mxArray *plhs[4]
// Return Type  : void
//
void DEAController_api(const mxArray *const prhs[5], int32_T nlhs,
                       const mxArray *plhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*k)[24];
  real_T(*sys_output)[24];
  real_T(*z)[22];
  real_T(*ref)[13];
  real_T(*param)[4];
  real_T(*xi_dot)[4];
  real_T(*F1)[3];
  real_T(*F2)[3];
  real_T t;
  st.tls = emlrtRootTLSGlobal;
  F1 = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  F2 = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  xi_dot = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  sys_output = (real_T(*)[24])mxMalloc(sizeof(real_T[24]));
  // Marshall function inputs
  z = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "z");
  k = b_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "k");
  param = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "param");
  ref = d_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "ref");
  t = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "t");
  // Invoke the target function
  DEAController(*z, *k, *param, *ref, t, *F1, *F2, *xi_dot, *sys_output);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*F1);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(*F2);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(*xi_dot);
  }
  if (nlhs > 3) {
    plhs[3] = c_emlrt_marshallOut(*sys_output);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void DEAController_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  DEAController_xil_terminate();
  DEAController_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void DEAController_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void DEAController_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_DEAController_api.cpp
//
// [EOF]
//
