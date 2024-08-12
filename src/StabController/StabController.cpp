//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: StabController.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 12-Sep-2023 17:59:47
//

// Include Files
#include "StabController.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : const double x[10]
//                const double Kv[12]
//                const double param[4]
//                const double setpoint[3]
//                double u[3]
// Return Type  : void
//
void StabController(const double x[10], const double Kv[12],
                    const double param[4], const double setpoint[3],
                    double u[3])
{
  double Ts1_idx_1;
  double Ts1_idx_1_tmp;
  double Ts1_tmp;
  double Ts1_tmp_tmp;
  double a_tmp;
  double b_Ts1_idx_1_tmp;
  double b_Ts1_tmp;
  double b_Ts1_tmp_tmp;
  double b_n1_tmp;
  double b_u_tmp;
  double c_n1_tmp;
  double c_u_tmp;
  double d_u_tmp;
  double e_u_tmp;
  double f_u_tmp;
  double n1;
  double n1_tmp;
  double n2;
  double n3;
  double n3_tmp;
  double n3p;
  double n3pp;
  double u_tmp;
  double u_tmp_tmp;
  Ts1_tmp_tmp = std::cos(x[4]);
  b_Ts1_tmp_tmp = std::cos(x[3]);
  Ts1_tmp = std::sin(x[4]);
  b_Ts1_tmp = std::sin(x[3]);
  Ts1_idx_1_tmp = param[2] * x[9];
  b_Ts1_idx_1_tmp = param[2] * x[8];
  Ts1_idx_1 = (x[7] - b_Ts1_idx_1_tmp * b_Ts1_tmp * Ts1_tmp_tmp) -
              Ts1_idx_1_tmp * b_Ts1_tmp_tmp * Ts1_tmp;
  n3_tmp = param[2] * b_Ts1_tmp_tmp;
  n3 = -Kv[0] * Ts1_idx_1 -
       Kv[3] * ((x[2] + n3_tmp * Ts1_tmp_tmp) - setpoint[2]);
  n3p = -Kv[0] * n3 - Ts1_idx_1 * Kv[3];
  n3pp = -Kv[0] * n3p - Kv[3] * n3;
  Ts1_idx_1 = param[3] - n3;
  n1_tmp = b_Ts1_tmp_tmp * b_Ts1_tmp_tmp;
  b_n1_tmp = Ts1_tmp_tmp * Ts1_tmp_tmp;
  c_n1_tmp = b_Ts1_tmp * x[8];
  n1 =
      ((-Kv[1] *
            (((n3p * Ts1_tmp_tmp * Ts1_tmp - x[9] * Ts1_idx_1) * b_Ts1_tmp_tmp -
              c_n1_tmp * Ts1_tmp_tmp * Ts1_tmp * Ts1_idx_1) /
             n1_tmp / b_n1_tmp) -
        -Ts1_tmp * Ts1_idx_1 / Ts1_tmp_tmp / b_Ts1_tmp_tmp * Kv[4]) -
       (x[5] + Ts1_idx_1_tmp * Ts1_tmp_tmp) * Kv[7]) -
      Kv[10] * ((x[0] + param[2] * Ts1_tmp) - setpoint[0]);
  n2 = ((-Kv[2] *
             ((-n3p * b_Ts1_tmp * b_Ts1_tmp_tmp + x[8] * Ts1_idx_1) / n1_tmp) -
         Kv[5] * (b_Ts1_tmp * Ts1_idx_1 / b_Ts1_tmp_tmp)) -
        ((x[6] - b_Ts1_idx_1_tmp * b_Ts1_tmp_tmp * Ts1_tmp_tmp) +
         Ts1_idx_1_tmp * b_Ts1_tmp * Ts1_tmp) *
            Kv[8]) -
       Kv[11] * ((x[1] - param[2] * b_Ts1_tmp * Ts1_tmp_tmp) - setpoint[1]);
  a_tmp = -param[3] + n3;
  u_tmp = x[8] * x[8];
  b_u_tmp = rt_powd_snf(Ts1_tmp_tmp, 3.0);
  c_u_tmp = x[9] * x[9];
  d_u_tmp = param[0] + param[1];
  e_u_tmp = u_tmp * a_tmp;
  u[0] =
      (((n3_tmp * ((n1_tmp * n3pp - b_Ts1_tmp_tmp * b_Ts1_tmp * n2) + e_u_tmp) *
             param[0] * b_u_tmp +
         3.0 * param[2] * c_u_tmp * b_Ts1_tmp_tmp * param[0] * a_tmp *
             Ts1_tmp_tmp) +
        a_tmp * a_tmp * d_u_tmp) *
           Ts1_tmp +
       2.0 * param[2] * param[0] *
           (-b_n1_tmp * n1_tmp * n1 / 2.0 +
            (b_Ts1_tmp_tmp * n3p + c_n1_tmp * a_tmp) * x[9]) *
           b_n1_tmp) /
      a_tmp / Ts1_tmp_tmp / b_Ts1_tmp_tmp;
  c_n1_tmp = rt_powd_snf(Ts1_tmp_tmp, 4.0);
  b_u_tmp *= param[2];
  n3_tmp = rt_powd_snf(b_Ts1_tmp_tmp, 3.0);
  u_tmp_tmp = 0.66666666666666663 * u_tmp;
  f_u_tmp = c_u_tmp + u_tmp_tmp;
  Ts1_idx_1 = Ts1_idx_1_tmp * x[8] * Ts1_tmp;
  u[1] =
      ((((-b_Ts1_tmp_tmp * param[2] * param[0] *
              (((n3_tmp * n2 + n1_tmp * b_Ts1_tmp * n3pp) -
                n2 * b_Ts1_tmp_tmp) +
               u_tmp * b_Ts1_tmp * a_tmp) *
              c_n1_tmp -
          b_u_tmp * n1_tmp * Ts1_tmp * b_Ts1_tmp * param[0] * n1) -
         3.0 * b_Ts1_tmp_tmp * param[2] * param[0] *
             ((0.66666666666666663 * x[8] * n3p + n2 / 3.0) * b_Ts1_tmp_tmp +
              f_u_tmp * b_Ts1_tmp * a_tmp) *
             b_n1_tmp) +
        ((-4.0 * param[2] * x[9] * x[8] * Ts1_tmp * param[0] * a_tmp * n1_tmp +
          2.0 * param[2] * x[9] * Ts1_tmp * b_Ts1_tmp_tmp * b_Ts1_tmp *
              param[0] * n3p) +
         2.0 * (-d_u_tmp * a_tmp * b_Ts1_tmp / 2.0 + Ts1_idx_1 * param[0]) *
             a_tmp) *
            Ts1_tmp_tmp) +
       2.0 * param[2] * c_u_tmp * b_Ts1_tmp_tmp * b_Ts1_tmp * param[0] *
           a_tmp) /
      a_tmp / Ts1_tmp_tmp / b_Ts1_tmp_tmp;
  u[2] =
      ((((param[2] * c_n1_tmp * rt_powd_snf(b_Ts1_tmp_tmp, 4.0) * param[0] *
              n3pp -
          b_u_tmp * param[0] * (Ts1_tmp_tmp * b_Ts1_tmp * n2 - Ts1_tmp * n1) *
              n3_tmp) +
         3.0 *
             (((e_u_tmp * c_n1_tmp / 3.0 +
                ((f_u_tmp * n3 + (-c_u_tmp - u_tmp_tmp) * param[3]) -
                 n3pp / 3.0) *
                    b_n1_tmp) -
               0.66666666666666663 * x[9] * Ts1_tmp_tmp * Ts1_tmp * n3p) -
              0.66666666666666663 * c_u_tmp * a_tmp) *
             param[2] * param[0] * n1_tmp) -
        4.0 *
            (b_Ts1_idx_1_tmp * Ts1_tmp_tmp * b_Ts1_tmp * param[0] * n3p / 2.0 +
             (Ts1_idx_1 * b_Ts1_tmp * param[0] - d_u_tmp * a_tmp / 4.0) *
                 a_tmp) *
            Ts1_tmp_tmp * b_Ts1_tmp_tmp) -
       2.0 * param[2] * u_tmp * b_n1_tmp * param[0] * a_tmp) /
      a_tmp / Ts1_tmp_tmp / b_Ts1_tmp_tmp;
}

//
// File trailer for StabController.cpp
//
// [EOF]
//
