//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_DEAController_mex.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 25-Aug-2024 22:36:55
//

// Include Files
#include "_coder_DEAController_mex.h"
#include "_coder_DEAController_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&DEAController_atexit);
  // Module initialization.
  DEAController_initialize();
  // Dispatch the entry-point.
  unsafe_DEAController_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  DEAController_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, (const char_T *)"windows-1252", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[4]
//                int32_T nrhs
//                const mxArray *prhs[5]
// Return Type  : void
//
void unsafe_DEAController_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                      int32_T nrhs, const mxArray *prhs[5])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs[4];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4,
                        13, "DEAController");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "DEAController");
  }
  // Call the function.
  DEAController_api(prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_DEAController_mex.cpp
//
// [EOF]
//
