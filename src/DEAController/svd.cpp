//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 25-Aug-2024 22:36:55
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A[3]
//                double U[9]
//                double S[3]
//                double *V
// Return Type  : void
//
namespace coder {
void svd(const double A[3], double U[9], double S[3], double *V)
{
  double s;
  boolean_T p;
  p = true;
  if (std::isinf(A[0]) || std::isnan(A[0]) ||
      (std::isinf(A[1]) || std::isnan(A[1]))) {
    p = false;
  }
  if ((!p) || (std::isinf(A[2]) || std::isnan(A[2]))) {
    p = false;
  }
  if (p) {
    double A_idx_0;
    double A_idx_1;
    double A_idx_2;
    double a;
    double absxk;
    double nrm;
    double scale;
    double t;
    int qjj;
    A_idx_0 = A[0];
    A_idx_1 = A[1];
    A_idx_2 = A[2];
    std::memset(&U[0], 0, 9U * sizeof(double));
    scale = 3.3121686421112381E-170;
    absxk = std::abs(A[0]);
    if (absxk > 3.3121686421112381E-170) {
      nrm = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      nrm = t * t;
    }
    absxk = std::abs(A[1]);
    if (absxk > scale) {
      t = scale / absxk;
      nrm = nrm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      nrm += t * t;
    }
    absxk = std::abs(A[2]);
    if (absxk > scale) {
      t = scale / absxk;
      nrm = nrm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      nrm += t * t;
    }
    nrm = scale * std::sqrt(nrm);
    if (nrm > 0.0) {
      if (A[0] < 0.0) {
        s = -nrm;
      } else {
        s = nrm;
      }
      if (std::abs(s) >= 1.0020841800044864E-292) {
        a = 1.0 / s;
        A_idx_0 = a * A[0];
        A_idx_1 = a * A[1];
        A_idx_2 = a * A[2];
      } else {
        A_idx_0 = A[0] / s;
        A_idx_1 = A[1] / s;
        A_idx_2 = A[2] / s;
      }
      A_idx_0++;
      s = -s;
    } else {
      s = 0.0;
    }
    U[0] = A_idx_0;
    U[1] = A_idx_1;
    U[2] = A_idx_2;
    for (int jj{0}; jj < 2; jj++) {
      qjj = 3 * (jj + 1);
      U[qjj] = 0.0;
      U[qjj + 1] = 0.0;
      U[qjj + 2] = 0.0;
      U[(jj + qjj) + 1] = 1.0;
    }
    if (s != 0.0) {
      for (int jj{0}; jj < 2; jj++) {
        qjj = 3 * (jj + 1);
        scale = U[qjj];
        absxk = U[1];
        t = U[qjj + 1];
        nrm = U[2];
        A_idx_2 = U[qjj + 2];
        a = -(((U[0] * scale + U[1] * t) + U[2] * A_idx_2) / U[0]);
        if (!(a == 0.0)) {
          scale += a * U[0];
          U[qjj] = scale;
          t += a * absxk;
          U[qjj + 1] = t;
          A_idx_2 += a * nrm;
          U[qjj + 2] = A_idx_2;
        }
      }
      U[0] = -U[0];
      U[1] = -U[1];
      U[2] = -U[2];
      U[0]++;
      scale = std::abs(s);
      a = s / scale;
      s = scale;
      U[0] *= a;
      U[1] *= a;
      U[2] *= a;
    } else {
      U[1] = 0.0;
      U[2] = 0.0;
      U[0] = 1.0;
    }
    *V = 1.0;
  } else {
    for (int qjj{0}; qjj < 9; qjj++) {
      U[qjj] = rtNaN;
    }
    s = rtNaN;
    *V = rtNaN;
  }
  S[1] = 0.0;
  S[2] = 0.0;
  S[0] = s;
}

} // namespace coder

//
// File trailer for svd.cpp
//
// [EOF]
//
