//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_DSLSDEAController_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 22-Sep-2024 04:35:15
//

#ifndef _CODER_DSLSDEACONTROLLER_API_H
#define _CODER_DSLSDEACONTROLLER_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void DSLSDEAController(real_T z[22], real_T k[24], real_T param[4],
                       real_T ref[15], real_T t, real_T F1[3], real_T F2[3],
                       real_T xi_dot[4]);

void DSLSDEAController_api(const mxArray *const prhs[5], int32_T nlhs,
                           const mxArray *plhs[3]);

void DSLSDEAController_atexit();

void DSLSDEAController_initialize();

void DSLSDEAController_terminate();

void DSLSDEAController_xil_shutdown();

void DSLSDEAController_xil_terminate();

#endif
//
// File trailer for _coder_DSLSDEAController_api.h
//
// [EOF]
//
