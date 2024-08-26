//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_DEAController_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 25-Aug-2024 22:36:55
//

#ifndef _CODER_DEACONTROLLER_API_H
#define _CODER_DEACONTROLLER_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void DEAController(real_T z[22], real_T k[24], real_T param[4], real_T ref[13],
                   real_T t, real_T F1[3], real_T F2[3], real_T xi_dot[4],
                   real_T sys_output[24]);

void DEAController_api(const mxArray *const prhs[5], int32_T nlhs,
                       const mxArray *plhs[4]);

void DEAController_atexit();

void DEAController_initialize();

void DEAController_terminate();

void DEAController_xil_shutdown();

void DEAController_xil_terminate();

#endif
//
// File trailer for _coder_DEAController_api.h
//
// [EOF]
//
