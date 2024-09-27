//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DSLSDEAController.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 22-Sep-2024 04:35:15
//

#ifndef DSLSDEACONTROLLER_H
#define DSLSDEACONTROLLER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void DSLSDEAController(const double z[22], const double k[24],
                              const double param[4], const double ref[15],
                              double t, double F1[3], double F2[3],
                              double xi_dot[4]);


int test(int k){return k;}

#endif
//
// File trailer for DSLSDEAController.h
//
// [EOF]
//

