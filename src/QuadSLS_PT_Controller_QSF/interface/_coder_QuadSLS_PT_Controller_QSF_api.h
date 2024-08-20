//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_QuadSLS_PT_Controller_QSF_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 09-Aug-2024 12:52:54
//

#ifndef _CODER_QUADSLS_PT_CONTROLLER_QSF_API_H
#define _CODER_QUADSLS_PT_CONTROLLER_QSF_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void QuadSLS_PT_Controller_QSF(real_T x[10], real_T Kv[12], real_T param[3],
                               real_T ref[12], real_T u[3]);

void QuadSLS_PT_Controller_QSF_api(const mxArray *const prhs[4],
                                   const mxArray **plhs);

void QuadSLS_PT_Controller_QSF_atexit();

void QuadSLS_PT_Controller_QSF_initialize();

void QuadSLS_PT_Controller_QSF_terminate();

void QuadSLS_PT_Controller_QSF_xil_shutdown();

void QuadSLS_PT_Controller_QSF_xil_terminate();

#endif
//
// File trailer for _coder_QuadSLS_PT_Controller_QSF_api.h
//
// [EOF]
//
