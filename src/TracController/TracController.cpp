//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: TracController.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 11-Sep-2023 17:17:59
//

// Include Files
#include "TracController.h"
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
//                double t
//                double u[3]
// Return Type  : void
//
void TracController(const double x[10], const double Kv[12],
                    const double param[4], double t, double u[3])
{
  double Ts1_idx_1_tmp;
  double Ts1_tmp;
  double Ts1_tmp_tmp;
  double b_Ts1_idx_1_tmp;
  double b_Ts1_tmp;
  double b_Ts1_tmp_tmp;
  double b_n1_tmp;
  double b_n3_tmp;
  double b_u_tmp;
  double b_yr_tmp;
  double c_u_tmp;
  double c_yr_tmp;
  double d_u_tmp;
  double e_u_tmp;
  double n1;
  double n1_tmp;
  double n2;
  double n3;
  double n3_tmp;
  double n3p;
  double n3pp;
  double u_tmp;
  double yr_idx_12;
  double yr_idx_13;
  double yr_tmp;
  Ts1_tmp_tmp = std::cos(x[4]);
  b_Ts1_tmp_tmp = std::cos(x[3]);
  Ts1_tmp = std::sin(x[4]);
  b_Ts1_tmp = std::sin(x[3]);
  Ts1_idx_1_tmp = param[2] * x[9];
  b_Ts1_idx_1_tmp = param[2] * x[8];
  yr_idx_12 = 3.1415926535897931 * t / 8.0;
  yr_tmp = std::sin(yr_idx_12);
  b_yr_tmp = std::cos(yr_idx_12);
  yr_idx_12 = 3.1415926535897931 * t / 4.0;
  c_yr_tmp = std::sin(yr_idx_12);
  n2 = std::cos(yr_idx_12);
  yr_idx_12 = -0.077106284383510609 * yr_tmp;
  yr_idx_13 = -0.030279567070605289 * b_yr_tmp;
  n3_tmp = ((x[7] - b_Ts1_idx_1_tmp * b_Ts1_tmp * Ts1_tmp_tmp) -
            Ts1_idx_1_tmp * b_Ts1_tmp_tmp * Ts1_tmp) -
           0.19634954084936207 * b_yr_tmp;
  b_n3_tmp = param[2] * b_Ts1_tmp_tmp;
  n3 = (yr_idx_12 - Kv[0] * n3_tmp) -
       Kv[3] * ((x[2] + b_n3_tmp * Ts1_tmp_tmp) - (0.5 * yr_tmp - 0.6));
  yr_idx_12 = n3 - yr_idx_12;
  n3p = (yr_idx_13 - Kv[0] * yr_idx_12) - Kv[3] * n3_tmp;
  n3pp = (0.011890758182861623 * yr_tmp - Kv[0] * (n3p - yr_idx_13)) -
         Kv[3] * yr_idx_12;
  yr_idx_12 = param[3] - n3;
  n1_tmp = b_Ts1_tmp_tmp * b_Ts1_tmp_tmp;
  b_n1_tmp = Ts1_tmp_tmp * Ts1_tmp_tmp;
  n3_tmp = b_Ts1_tmp * x[8];
  n1 = (((0.035672274548584869 * yr_tmp -
          Kv[1] * (((n3p * Ts1_tmp_tmp * Ts1_tmp - x[9] * yr_idx_12) *
                        b_Ts1_tmp_tmp -
                    n3_tmp * Ts1_tmp_tmp * Ts1_tmp * yr_idx_12) /
                       n1_tmp / b_n1_tmp -
                   -0.090838701211815864 * b_yr_tmp)) -
         Kv[4] * (-Ts1_tmp * yr_idx_12 / Ts1_tmp_tmp / b_Ts1_tmp_tmp -
                  -0.23131885315053183 * yr_tmp)) -
        Kv[7] * ((x[5] + Ts1_idx_1_tmp * Ts1_tmp_tmp) -
                 0.58904862254808621 * b_yr_tmp)) -
       Kv[10] * ((x[0] + param[2] * Ts1_tmp) - 1.5 * yr_tmp);
  n2 = (((0.28537819638867895 * c_yr_tmp -
          Kv[2] *
              ((-n3p * b_Ts1_tmp * b_Ts1_tmp_tmp + x[8] * yr_idx_12) / n1_tmp -
               -0.36335480484726346 * n2)) -
         Kv[5] * (b_Ts1_tmp * yr_idx_12 / b_Ts1_tmp_tmp -
                  -0.46263770630106366 * c_yr_tmp)) -
        Kv[8] * (((x[6] - b_Ts1_idx_1_tmp * b_Ts1_tmp_tmp * Ts1_tmp_tmp) +
                  Ts1_idx_1_tmp * b_Ts1_tmp * Ts1_tmp) -
                 0.58904862254808621 * n2)) -
       Kv[11] * ((x[1] - param[2] * b_Ts1_tmp * Ts1_tmp_tmp) - 0.75 * c_yr_tmp);
  c_yr_tmp = -param[3] + n3;
  u_tmp = x[8] * x[8];
  b_u_tmp = rt_powd_snf(Ts1_tmp_tmp, 3.0);
  c_u_tmp = x[9] * x[9];
  d_u_tmp = param[0] + param[1];
  e_u_tmp = u_tmp * c_yr_tmp;
  u[0] = (((b_n3_tmp *
                ((n1_tmp * n3pp - b_Ts1_tmp_tmp * b_Ts1_tmp * n2) + e_u_tmp) *
                param[0] * b_u_tmp +
            3.0 * param[2] * c_u_tmp * b_Ts1_tmp_tmp * param[0] * c_yr_tmp *
                Ts1_tmp_tmp) +
           c_yr_tmp * c_yr_tmp * d_u_tmp) *
              Ts1_tmp +
          2.0 * param[2] * param[0] *
              (-b_n1_tmp * n1_tmp * n1 / 2.0 +
               (b_Ts1_tmp_tmp * n3p + n3_tmp * c_yr_tmp) * x[9]) *
              b_n1_tmp) /
         c_yr_tmp / Ts1_tmp_tmp / b_Ts1_tmp_tmp;
  n3_tmp = rt_powd_snf(Ts1_tmp_tmp, 4.0);
  b_u_tmp *= param[2];
  yr_idx_13 = rt_powd_snf(b_Ts1_tmp_tmp, 3.0);
  yr_tmp = 0.66666666666666663 * u_tmp;
  b_yr_tmp = c_u_tmp + yr_tmp;
  yr_idx_12 = Ts1_idx_1_tmp * x[8] * Ts1_tmp;
  u[1] =
      ((((-b_Ts1_tmp_tmp * param[2] * param[0] *
              (((yr_idx_13 * n2 + n1_tmp * b_Ts1_tmp * n3pp) -
                n2 * b_Ts1_tmp_tmp) +
               u_tmp * b_Ts1_tmp * c_yr_tmp) *
              n3_tmp -
          b_u_tmp * n1_tmp * Ts1_tmp * b_Ts1_tmp * param[0] * n1) -
         3.0 * b_Ts1_tmp_tmp * param[2] * param[0] *
             ((0.66666666666666663 * x[8] * n3p + n2 / 3.0) * b_Ts1_tmp_tmp +
              b_yr_tmp * b_Ts1_tmp * c_yr_tmp) *
             b_n1_tmp) +
        ((-4.0 * param[2] * x[9] * x[8] * Ts1_tmp * param[0] * c_yr_tmp *
              n1_tmp +
          2.0 * param[2] * x[9] * Ts1_tmp * b_Ts1_tmp_tmp * b_Ts1_tmp *
              param[0] * n3p) +
         2.0 * (-d_u_tmp * c_yr_tmp * b_Ts1_tmp / 2.0 + yr_idx_12 * param[0]) *
             c_yr_tmp) *
            Ts1_tmp_tmp) +
       2.0 * param[2] * c_u_tmp * b_Ts1_tmp_tmp * b_Ts1_tmp * param[0] *
           c_yr_tmp) /
      c_yr_tmp / Ts1_tmp_tmp / b_Ts1_tmp_tmp;
  u[2] =
      ((((param[2] * n3_tmp * rt_powd_snf(b_Ts1_tmp_tmp, 4.0) * param[0] *
              n3pp -
          b_u_tmp * param[0] * (Ts1_tmp_tmp * b_Ts1_tmp * n2 - Ts1_tmp * n1) *
              yr_idx_13) +
         3.0 *
             (((e_u_tmp * n3_tmp / 3.0 +
                ((b_yr_tmp * n3 + (-c_u_tmp - yr_tmp) * param[3]) -
                 n3pp / 3.0) *
                    b_n1_tmp) -
               0.66666666666666663 * x[9] * Ts1_tmp_tmp * Ts1_tmp * n3p) -
              0.66666666666666663 * c_u_tmp * c_yr_tmp) *
             param[2] * param[0] * n1_tmp) -
        4.0 *
            (b_Ts1_idx_1_tmp * Ts1_tmp_tmp * b_Ts1_tmp * param[0] * n3p / 2.0 +
             (yr_idx_12 * b_Ts1_tmp * param[0] - d_u_tmp * c_yr_tmp / 4.0) *
                 c_yr_tmp) *
            Ts1_tmp_tmp * b_Ts1_tmp_tmp) -
       2.0 * param[2] * u_tmp * b_n1_tmp * param[0] * c_yr_tmp) /
      c_yr_tmp / Ts1_tmp_tmp / b_Ts1_tmp_tmp;
}

//
// File trailer for TracController.cpp
//
// [EOF]
//
