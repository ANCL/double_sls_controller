//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_QuadSLS_PT_Controller_QSF_mex.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 09-Aug-2024 12:52:54
//

// Include Files
#include "_coder_QuadSLS_PT_Controller_QSF_mex.h"
#include "_coder_QuadSLS_PT_Controller_QSF_api.h"

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
  mexAtExit(&QuadSLS_PT_Controller_QSF_atexit);
  // Module initialization.
  QuadSLS_PT_Controller_QSF_initialize();
  // Dispatch the entry-point.
  unsafe_QuadSLS_PT_Controller_QSF_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  QuadSLS_PT_Controller_QSF_terminate();
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
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[4]
// Return Type  : void
//
void unsafe_QuadSLS_PT_Controller_QSF_mexFunction(int32_T nlhs,
                                                  mxArray *plhs[1],
                                                  int32_T nrhs,
                                                  const mxArray *prhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        25, "QuadSLS_PT_Controller_QSF");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 25,
                        "QuadSLS_PT_Controller_QSF");
  }
  // Call the function.
  QuadSLS_PT_Controller_QSF_api(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// File trailer for _coder_QuadSLS_PT_Controller_QSF_mex.cpp
//
// [EOF]
//
