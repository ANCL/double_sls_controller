//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QuadSLS_PT_Controller_QSF.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 09-Aug-2024 12:52:54
//

// Include Files
#include "QuadSLS_PT_Controller_QSF.h"
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
//                const double param[3]
//                const double ref[12]
//                double u[3]
// Return Type  : void
//
void QuadSLS_PT_Controller_QSF(const double x[10], const double Kv[12],
                               const double param[3], const double ref[12],
                               double u[3])
{
  double Ts_x_tmp;
  double Ts_x_tmp_tmp;
  double b_Ts_x_tmp;
  double b_u_tmp;
  double b_v1_tmp;
  double c_u_tmp;
  double d_u_tmp;
  double e_u_tmp;
  double f_u_tmp;
  double g_u_tmp;
  double h_u_tmp;
  double i_u_tmp;
  double j_u_tmp;
  double u_tmp;
  double v1;
  double v1_tmp;
  double v2;
  double v3;
  double v3d;
  double v3d2;
  double v3d_tmp;
  v3d2 = x[7] - ref[5];
  v3 = -Kv[2] * (x[2] - ref[2]) - Kv[5] * v3d2;
  v3d_tmp = v3 - ref[8];
  v3d = -Kv[2] * v3d2 - Kv[5] * v3d_tmp;
  v3d2 = -Kv[2] * v3d_tmp - Kv[5] * (v3d - ref[11]);
  v3d_tmp = std::cos(x[4]);
  Ts_x_tmp = std::sin(x[4]);
  Ts_x_tmp_tmp = std::cos(x[3]);
  b_Ts_x_tmp = std::sin(x[3]);
  v1_tmp = Ts_x_tmp_tmp * Ts_x_tmp_tmp;
  b_v1_tmp = v3d_tmp * v3d_tmp;
  v1 =
      ((-Kv[0] * (x[0] - ref[0]) - Kv[3] * (x[5] - ref[3])) -
       Kv[6] * (-(9.81 - v3) * Ts_x_tmp / v3d_tmp / Ts_x_tmp_tmp - ref[6])) -
      Kv[9] * (((v3d * v3d_tmp * Ts_x_tmp - (9.81 - v3) * x[9]) * Ts_x_tmp_tmp -
                b_Ts_x_tmp * x[8] * v3d_tmp * Ts_x_tmp * (9.81 - v3)) /
                   v1_tmp / b_v1_tmp -
               ref[9]);
  v2 = ((-Kv[1] * (x[1] - ref[1]) - Kv[4] * (x[6] - ref[4])) -
        Kv[7] * ((9.81 - v3) * b_Ts_x_tmp / Ts_x_tmp_tmp - ref[7])) -
       Kv[10] *
           ((-v3d * b_Ts_x_tmp * Ts_x_tmp_tmp + (9.81 - v3) * x[8]) / v1_tmp -
            ref[10]);
  u_tmp = x[8] * x[8];
  b_u_tmp = rt_powd_snf(v3d_tmp, 3.0);
  c_u_tmp = x[9] * x[9];
  d_u_tmp = param[0] + param[1];
  e_u_tmp = u_tmp * (9.81 - v3);
  f_u_tmp = 2.0 * param[0] * param[2];
  g_u_tmp = v3d2 * v1_tmp;
  u[0] =
      (((-Ts_x_tmp_tmp * param[0] * param[2] *
             ((g_u_tmp - Ts_x_tmp_tmp * b_Ts_x_tmp * v2) - e_u_tmp) * b_u_tmp +
         3.0 * param[2] * c_u_tmp * Ts_x_tmp_tmp * param[0] * (9.81 - v3) *
             v3d_tmp) -
        (9.81 - v3) * (9.81 - v3) * d_u_tmp) *
           Ts_x_tmp -
       f_u_tmp *
           (-b_v1_tmp * v1_tmp * v1 / 2.0 +
            (v3d * Ts_x_tmp_tmp - x[8] * std::sin(x[3]) * (9.81 - v3)) * x[9]) *
           b_v1_tmp) /
      (9.81 - v3) / v3d_tmp / Ts_x_tmp_tmp;
  h_u_tmp = rt_powd_snf(v3d_tmp, 4.0);
  i_u_tmp = rt_powd_snf(Ts_x_tmp_tmp, 3.0);
  d_u_tmp *= 9.81 - v3;
  j_u_tmp = 2.0 * (9.81 - v3);
  u[1] =
      ((((Ts_x_tmp_tmp * param[0] * param[2] *
              (((i_u_tmp * v2 + g_u_tmp * b_Ts_x_tmp) - Ts_x_tmp_tmp * v2) -
               u_tmp * b_Ts_x_tmp * (9.81 - v3)) *
              h_u_tmp +
          param[2] * b_Ts_x_tmp * b_u_tmp * v1_tmp * Ts_x_tmp * param[0] * v1) +
         2.0 * Ts_x_tmp_tmp *
             ((v3d * x[8] + v2 / 2.0) * Ts_x_tmp_tmp -
              1.5 * b_Ts_x_tmp * (c_u_tmp + 0.66666666666666663 * u_tmp) *
                  (9.81 - v3)) *
             param[0] * param[2] * b_v1_tmp) +
        ((-4.0 * param[2] * x[9] * x[8] * Ts_x_tmp * param[0] * (9.81 - v3) *
              v1_tmp -
          2.0 * param[2] * v3d * x[9] * Ts_x_tmp * Ts_x_tmp_tmp * b_Ts_x_tmp *
              param[0]) +
         j_u_tmp * (d_u_tmp * b_Ts_x_tmp / 2.0 +
                    param[2] * x[9] * x[8] * Ts_x_tmp * param[0])) *
            v3d_tmp) +
       2.0 * param[2] * c_u_tmp * Ts_x_tmp_tmp * b_Ts_x_tmp * param[0] *
           (9.81 - v3)) /
      (9.81 - v3) / v3d_tmp / Ts_x_tmp_tmp;
  u[2] = ((((-param[2] * h_u_tmp * rt_powd_snf(Ts_x_tmp_tmp, 4.0) * param[0] *
                 v3d2 +
             param[2] * b_u_tmp * param[0] *
                 (v3d_tmp * b_Ts_x_tmp * v2 - Ts_x_tmp * v1) * i_u_tmp) +
            f_u_tmp *
                (((e_u_tmp * h_u_tmp / 2.0 + (((-1.5 * c_u_tmp - u_tmp) * v3 +
                                               (1.5 * c_u_tmp + u_tmp) * 9.81) +
                                              v3d2 / 2.0) *
                                                 b_v1_tmp) +
                  v3d * x[9] * v3d_tmp * Ts_x_tmp) -
                 c_u_tmp * (9.81 - v3)) *
                v1_tmp) +
           2.0 *
               (param[2] * v3d * x[8] * v3d_tmp * b_Ts_x_tmp * param[0] -
                j_u_tmp * (param[2] * x[9] * x[8] * std::sin(x[4]) *
                               b_Ts_x_tmp * param[0] +
                           d_u_tmp / 4.0)) *
               v3d_tmp * Ts_x_tmp_tmp) -
          2.0 * param[2] * u_tmp * b_v1_tmp * param[0] * (9.81 - v3)) /
         (9.81 - v3) / v3d_tmp / Ts_x_tmp_tmp;
}

//
// File trailer for QuadSLS_PT_Controller_QSF.cpp
//
// [EOF]
//
