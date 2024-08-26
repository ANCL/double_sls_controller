//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DEAController.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 25-Aug-2024 22:36:55
//

#ifndef DEACONTROLLER_H
#define DEACONTROLLER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void DEAController(const double z[22], const double k[24],
                          const double param[4], const double ref[13], double t,
                          double F1[3], double F2[3], double xi_dot[4],
                          double sys_output[24]);

#endif
//
// File trailer for DEAController.h
//
// [EOF]
//
