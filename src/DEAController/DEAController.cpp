//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DEAController.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 25-Aug-2024 22:36:55
//

// Include Files
#include "DEAController.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include <cfloat>
#include <cmath>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

static double rt_remd_snf(double u0, double u1);

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
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_remd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
    y = rtNaN;
  } else if (std::isinf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
    double q;
    q = std::abs(u0 / u1);
    if (!(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = std::fmod(u0, u1);
    }
  } else {
    y = std::fmod(u0, u1);
  }
  return y;
}

//
// Arguments    : const double z[22]
//                const double k[24]
//                const double param[4]
//                const double ref[13]
//                double t
//                double F1[3]
//                double F2[3]
//                double xi_dot[4]
//                double sys_output[24]
// Return Type  : void
//
void DEAController(const double z[22], const double k[24],
                   const double param[4], const double ref[13], double t,
                   double F1[3], double F2[3], double xi_dot[4],
                   double sys_output[24])
{
  static const signed char iv[6]{0, 0, 1, 0, 0, 0};
  double dv[36];
  double U[9];
  double q1_matriax[9];
  double q2_matriax[9];
  double Ud[6];
  double b_z[6];
  double h[6];
  double h_dddt[6];
  double h_ddt[6];
  double h_dt[6];
  double S[3];
  double q1[3];
  double Delta;
  double Mt;
  double a_tmp;
  double absxk;
  double b_h_dddt_tmp;
  double b_h_dddt_tmp_tmp;
  double b_h_dt_tmp;
  double b_h_tmp;
  double b_t;
  double b_z_tmp;
  double c_h_dddt_tmp;
  double c_h_dt_tmp;
  double c_h_tmp;
  double c_z_tmp;
  double d;
  double d1;
  double d10;
  double d11;
  double d12;
  double d13;
  double d14;
  double d15;
  double d16;
  double d17;
  double d18;
  double d19;
  double d2;
  double d20;
  double d21;
  double d22;
  double d23;
  double d24;
  double d25;
  double d26;
  double d27;
  double d28;
  double d29;
  double d3;
  double d30;
  double d31;
  double d32;
  double d33;
  double d34;
  double d35;
  double d36;
  double d37;
  double d38;
  double d39;
  double d4;
  double d40;
  double d41;
  double d42;
  double d43;
  double d44;
  double d45;
  double d46;
  double d47;
  double d48;
  double d49;
  double d5;
  double d50;
  double d51;
  double d52;
  double d53;
  double d54;
  double d55;
  double d56;
  double d6;
  double d7;
  double d8;
  double d9;
  double d_h_dddt_tmp;
  double d_h_dt_tmp;
  double d_h_tmp;
  double d_z_tmp;
  double e_h_dt_tmp;
  double e_h_tmp;
  double e_z_tmp;
  double f_h_dt_tmp;
  double f_z_tmp;
  double g_h_dt_tmp;
  double g_z_tmp;
  double h_dddt_tmp;
  double h_dddt_tmp_tmp;
  double h_dt_tmp;
  double h_tmp;
  double h_z_tmp;
  double i_z_tmp;
  double j_z_tmp;
  double k_z_tmp;
  double l_z_tmp;
  double m_z_tmp;
  double n_z_tmp;
  double o_z_tmp;
  double p_z_tmp;
  double q1__3;
  double q1__3_tmp;
  double q1_idx_0;
  double q1_idx_1;
  double q1_idx_2;
  double q2__3;
  double q2__3_tmp;
  double q2_idx_0;
  double q2_idx_1;
  double q_z_tmp;
  double r_z_tmp;
  double ref_1_dt_tmp;
  double ref_1_tmp;
  double ref_2_dt_tmp;
  double ref_2_tmp;
  double ref_3_dt_tmp;
  double ref_3_tmp;
  double s_z_tmp;
  double x;
  double z_tmp;
  //  Double SLS Controller
  //  Projecting q1 and q2 on S2
  q1[0] = z[3];
  q1[1] = z[4];
  q1[2] = z[5];
  coder::svd(q1, U, S, &q1_idx_2);
  for (int i{0}; i < 9; i++) {
    q1_matriax[i] = U[i] * q1_idx_2;
  }
  q1_idx_0 = U[0] * q1_idx_2;
  q1_idx_1 = U[1] * q1_idx_2;
  q1_idx_2 *= U[2];
  // q1 corrected
  x = q1_matriax[2];
  if (!std::isnan(q1_idx_2)) {
    if (q1_idx_2 < 0.0) {
      x = -1.0;
    } else {
      x = (q1_idx_2 > 0.0);
    }
  }
  q1__3_tmp = q1_matriax[0] * q1_matriax[0];
  q1__3 = x * std::sqrt((1.0 - q1__3_tmp) - q1_matriax[1] * q1_matriax[1]);
  q1[0] = z[6];
  q1[1] = z[7];
  q1[2] = z[8];
  coder::svd(q1, U, S, &q1_idx_2);
  for (int i{0}; i < 9; i++) {
    q2_matriax[i] = U[i] * q1_idx_2;
  }
  q2_idx_0 = U[0] * q1_idx_2;
  q2_idx_1 = U[1] * q1_idx_2;
  q1_idx_2 *= U[2];
  // q2 corrected
  x = q2_matriax[2];
  if (!std::isnan(q1_idx_2)) {
    if (q1_idx_2 < 0.0) {
      x = -1.0;
    } else {
      x = (q1_idx_2 > 0.0);
    }
  }
  q2__3_tmp = q2_matriax[0] * q2_matriax[0];
  q2__3 = x * std::sqrt((1.0 - q2__3_tmp) - q2_matriax[1] * q2_matriax[1]);
  //  output & derivatives
  h[0] = z[0];
  h[1] = z[1];
  h[2] = z[2];
  h_tmp = q2_idx_0 * q1_idx_0;
  q1_idx_2 = q1_idx_1 * q2_idx_1;
  b_h_tmp = h_tmp + q1_idx_2;
  c_h_tmp = q1__3 * q2__3;
  h[3] = b_h_tmp + c_h_tmp;
  h[4] = q1__3 - q2__3;
  d_h_tmp = q2_idx_0 * q1_idx_1;
  e_h_tmp = q1_idx_0 * q2_idx_1;
  h[5] = e_h_tmp - d_h_tmp;
  h_dt[0] = z[9];
  h_dt[1] = z[10];
  h_dt[2] = z[11];
  h_dt_tmp = -z[12] + z[15];
  absxk = -z[14] + z[17];
  b_h_dt_tmp = q2__3 * (-z[13] + z[16]);
  c_h_dt_tmp = z[13] - z[16];
  h_dt[3] = (((z[14] - z[17]) * q2_idx_1 + b_h_dt_tmp) * q1_idx_0 +
             (absxk * q2_idx_0 - h_dt_tmp * q2__3) * q1_idx_1) +
            q1__3 * (c_h_dt_tmp * q2_idx_0 + q2_idx_1 * h_dt_tmp);
  h_dt_tmp = q2_idx_0 * z[16];
  d_h_dt_tmp = q2_idx_1 * z[15];
  Delta = q1_idx_1 * z[12];
  h_dt[4] = ((-q1_idx_0 * z[13] + Delta) + h_dt_tmp) - d_h_dt_tmp;
  b_t = q1__3 * q2_idx_0;
  e_h_dt_tmp = q1__3 * q2_idx_1;
  f_h_dt_tmp = q1_idx_0 * q2__3;
  g_h_dt_tmp = q1_idx_1 * q2__3;
  h_dt[5] = ((((h_tmp * absxk - f_h_dt_tmp * z[15]) + q1_idx_2 * absxk) -
              g_h_dt_tmp * z[16]) +
             b_t * z[12]) +
            e_h_dt_tmp * z[13];
  q1_idx_2 = -q2__3 * z[19] + z[18];
  h_ddt[0] = (q1_idx_2 * q1_idx_0 + b_t * z[19]) / q1__3;
  h_ddt[1] = (q1_idx_2 * q1_idx_1 + e_h_dt_tmp * z[19]) / q1__3;
  h_ddt[2] = param[3] + z[18];
  h_ddt[3] = 0.0;
  h_ddt[4] = 0.0;
  h_ddt[5] = 0.0;
  h_dddt_tmp_tmp = q2__3 * z[19];
  h_dddt_tmp = h_dddt_tmp_tmp - z[18];
  b_h_dddt_tmp = q1__3 * q1__3;
  q1_idx_2 = z[21] * q2__3;
  h_dddt[0] =
      (((((b_h_dt_tmp - q2_idx_1 * z[17]) * z[19] + q2_idx_0 * z[21]) +
         z[13] * z[18]) *
            b_h_dddt_tmp +
        ((((h_dt_tmp - d_h_dt_tmp) * z[19] - q1_idx_2) + z[20]) * q1_idx_0 +
         q1_idx_1 * z[14] * h_dddt_tmp) *
            q1__3) -
       (q1_idx_0 * z[13] - Delta) * q1_idx_0 * h_dddt_tmp) /
      b_h_dddt_tmp;
  c_h_dddt_tmp = -q2_idx_0 * z[16] + d_h_dt_tmp;
  b_h_dddt_tmp_tmp = q1_idx_0 * q1_idx_1;
  d_h_dddt_tmp = (q1__3_tmp * z[12] + b_h_dddt_tmp_tmp * z[13]) - z[12];
  h_dddt[1] =
      ((((q2_idx_0 * z[17] - q2__3 * z[15]) * z[19] + q2_idx_1 * z[21]) *
            b_h_dddt_tmp +
        (((-q1_idx_0 * z[14] * q2__3 - c_h_dddt_tmp * q1_idx_1) * z[19] +
          q1_idx_0 * z[14] * z[18]) -
         q1_idx_1 * (q1_idx_2 - z[20])) *
            q1__3) -
       h_dddt_tmp * d_h_dddt_tmp) /
      b_h_dddt_tmp;
  h_dddt[2] = z[20];
  h_dddt[3] = 0.0;
  h_dddt[4] = 0.0;
  h_dddt[5] = 0.0;
  //  reference & derivatives
  //  Ref 1
  q1_idx_2 = ref[1] * t + ref[3];
  ref_1_tmp = std::sin(q1_idx_2);
  ref_1_dt_tmp = std::cos(q1_idx_2);
  //  Ref 2
  q1_idx_2 = ref[5] * t + ref[7];
  ref_2_tmp = std::sin(q1_idx_2);
  ref_2_dt_tmp = std::cos(q1_idx_2);
  //  Ref 3
  q1_idx_2 = ref[9] * t + ref[11];
  ref_3_tmp = std::sin(q1_idx_2);
  ref_3_dt_tmp = std::cos(q1_idx_2);
  //  Ref 4
  //  Ref 5
  //  Ref 6
  //  linear subsystems
  //  1st linear subsystem
  //  2nd linear subsystem
  //  3rd linear subsystem
  //  4th linear subsystem
  //  5th linear subsystem
  //  6th linear subsystem
  //  friend
  //  auxiliary input vv
  //  Physical Controller U
  if (std::isinf(ref[12]) || std::isnan(ref[12])) {
    x = rtNaN;
  } else {
    signed char n;
    x = rt_remd_snf(ref[12], 360.0);
    q1_idx_2 = std::abs(x);
    if (q1_idx_2 > 180.0) {
      if (x > 0.0) {
        x -= 360.0;
      } else {
        x += 360.0;
      }
      q1_idx_2 = std::abs(x);
    }
    if (q1_idx_2 <= 45.0) {
      x *= 0.017453292519943295;
      n = 0;
    } else if (q1_idx_2 <= 135.0) {
      if (x > 0.0) {
        x = 0.017453292519943295 * (x - 90.0);
        n = 1;
      } else {
        x = 0.017453292519943295 * (x + 90.0);
        n = -1;
      }
    } else if (x > 0.0) {
      x = 0.017453292519943295 * (x - 180.0);
      n = 2;
    } else {
      x = 0.017453292519943295 * (x + 180.0);
      n = -2;
    }
    if (n == 0) {
      x = std::cos(x);
    } else if (n == 1) {
      x = -std::sin(x);
    } else if (n == -1) {
      x = std::sin(x);
    } else {
      x = -std::cos(x);
    }
  }
  absxk = z[12] * z[14];
  b_t = z[16] * z[16];
  Delta = z[12] * z[12];
  e_h_dt_tmp = z[13] * z[13];
  Mt = 2.0 * e_h_dt_tmp;
  h_dt_tmp = z[14] * z[14];
  b_h_dt_tmp = z[12] * z[13];
  z_tmp = 3.0 * z[12] * z[14];
  b_z_tmp = 2.0 * z[20] * z[13];
  d_h_dt_tmp = 1.5 * z[13] * z[14];
  c_z_tmp = 2.0 * z[12] * z[14];
  d_z_tmp = rt_powd_snf(q1__3, 3.0);
  e_z_tmp = z[15] * z[15];
  f_z_tmp = z[17] * z[17];
  g_z_tmp = z[15] * z[16];
  h_z_tmp = 2.0 * z[21] * z[17];
  i_z_tmp = 2.0 * z[14] * c_h_dddt_tmp;
  j_z_tmp = q2_idx_0 * z[15] + q2_idx_1 * z[16];
  k_z_tmp = q2__3 * q2__3;
  l_z_tmp = z[19] * h_dddt_tmp;
  m_z_tmp = Delta - e_h_dt_tmp;
  n_z_tmp = 2.0 * z[12] * z[13];
  o_z_tmp = 2.0 * h_dddt_tmp;
  p_z_tmp = z[15] * z[17];
  q_z_tmp = z[17] * j_z_tmp;
  b_z[0] = -(
      ((((((((((((absxk + p_z_tmp) * q2__3 +
                 (2.0 * q2_idx_0 * z[16] - 2.0 * q2_idx_1 * z[15]) * z[13]) +
                (-b_t - f_z_tmp) * q2_idx_0) +
               g_z_tmp * q2_idx_1) *
                  z[19] -
              2.0 * z[21] * c_h_dt_tmp * q2__3) -
             h_z_tmp * q2_idx_1) -
            absxk * z[18]) +
           b_z_tmp) *
              param[2] +
          q2_idx_0 * q2__3 * z[19] * h_dddt_tmp) *
             d_z_tmp +
         (((((((((Delta - Mt) + h_dt_tmp) + e_z_tmp) + b_t) * q2__3 - q_z_tmp) *
                 q1_idx_0 +
             q1_idx_1 * (b_h_dt_tmp * q2__3 + i_z_tmp)) *
                z[19] +
            ((-Delta * z[18] + (Mt - h_dt_tmp) * z[18]) -
             2.0 * z[21] * c_h_dddt_tmp) *
                q1_idx_0) -
           2.0 * q1_idx_1 *
               ((-(z[14] * z[21] * q2__3) + b_h_dt_tmp * z[18] / 2.0) +
                z[14] * z[20])) *
              param[2] +
          l_z_tmp * (((q1_idx_0 * q2__3_tmp - q1_idx_0 * k_z_tmp) +
                      d_h_tmp * q2_idx_1) -
                     q2_idx_0)) *
             b_h_dddt_tmp) +
        (((((((z_tmp * q2__3 - 2.0 * z[13] * c_h_dddt_tmp) * q1__3_tmp +
              2.0 * q1_idx_1 * (d_h_dt_tmp * q2__3 + z[12] * c_h_dddt_tmp) *
                  q1_idx_0) -
             c_z_tmp * q2__3) *
                z[19] +
            ((-2.0 * q2__3 * z[21] * z[13] - z_tmp * z[18]) + b_z_tmp) *
                q1__3_tmp) -
           2.0 * q1_idx_1 *
               ((-(z[12] * z[21] * q2__3) + d_h_dt_tmp * z[18]) +
                z[12] * z[20]) *
               q1_idx_0) +
          c_z_tmp * z[18]) *
             param[2] -
         f_h_dt_tmp * z[19] * h_dddt_tmp * (b_h_tmp - 1.0)) *
            q1__3) +
       o_z_tmp *
           ((m_z_tmp * q1__3_tmp + n_z_tmp * q1_idx_1 * q1_idx_0) - Delta) *
           param[2] * q1_idx_0) /
      d_z_tmp / param[2]);
  Mt = 2.0 * z[20] * z[12];
  z_tmp = z[16] * z[17];
  d_h_dt_tmp = z[13] * z[14];
  b_z[1] = -(
      ((((((((((-2.0 * z[13] * z[14] + z_tmp) * q2__3 +
               (-e_z_tmp - f_z_tmp) * q2_idx_1) +
              g_z_tmp * q2_idx_0) *
                 z[19] -
             2.0 * z[21] * z[15] * q2__3) +
            h_z_tmp * q2_idx_0) +
           2.0 * z[13] * z[14] * z[18]) *
              param[2] +
          2.0 * q2_idx_1 * q2__3 * z[19] * h_dddt_tmp) *
             d_z_tmp +
         (((((((-3.0 * z[12] * z[13] * q1_idx_0 -
                q1_idx_1 * (((e_h_dt_tmp - h_dt_tmp) - e_z_tmp) - b_t)) *
                   q2__3 -
               i_z_tmp * q1_idx_0) -
              q1_idx_1 * z[17] * j_z_tmp) *
                 z[19] -
             2.0 * z[21] * z[14] * q1_idx_0 * q2__3) +
            (3.0 * z[12] * z[13] * z[18] + 2.0 * z[20] * z[14]) * q1_idx_0) -
           2.0 *
               ((-e_h_dt_tmp + h_dt_tmp) * z[18] / 2.0 + z[21] * c_h_dddt_tmp) *
               q1_idx_1) *
              param[2] +
          l_z_tmp * ((((h_tmp * q2_idx_1 - q1_idx_1 * q2__3_tmp) -
                       2.0 * q1_idx_1 * k_z_tmp) +
                      q1_idx_1) -
                     q2_idx_1)) *
             b_h_dddt_tmp) +
        (((((((3.0 * z[14] *
                   ((q1_idx_0 * z[12] * q1_idx_1 - z[13] * q1__3_tmp) +
                    z[13] / 3.0) *
                   q2__3 -
               2.0 * c_h_dddt_tmp * d_h_dddt_tmp) *
                  z[19] -
              2.0 * z[21] * d_h_dddt_tmp * q2__3) +
             (3.0 * z[13] * z[14] * z[18] + Mt) * q1__3_tmp) +
            q1_idx_1 * (-3.0 * z[12] * z[14] * z[18] + b_z_tmp) * q1_idx_0) -
           d_h_dt_tmp * z[18]) -
          Mt) *
             param[2] +
         h_dddt_tmp_tmp * h_dddt_tmp *
             (((q1__3_tmp * q2_idx_1 - d_h_tmp * q1_idx_0) + q1_idx_1) -
              q2_idx_1)) *
            q1__3) +
       o_z_tmp *
           (((-2.0 * z[12] * z[13] * rt_powd_snf(q1_matriax[0], 3.0) +
              q1_idx_1 * m_z_tmp * q1__3_tmp) +
             n_z_tmp * q1_idx_0) -
            Delta * q1_idx_1) *
           param[2]) /
      d_z_tmp / param[2]);
  b_z[2] = -0.0;
  Mt = z[14] - 2.0 * z[17];
  b_z_tmp = 2.0 * k_z_tmp;
  c_z_tmp = z[14] - z[17] / 2.0;
  h_z_tmp = 2.0 * z[13] * z[16];
  i_z_tmp = 2.0 * z[12] * z[15];
  j_z_tmp = 2.0 * z[14] * z[17];
  l_z_tmp = q2__3_tmp / 2.0;
  m_z_tmp = (q2__3_tmp - 0.5) * q1__3_tmp;
  n_z_tmp = b_h_dddt_tmp_tmp * q2_idx_0 * q2_idx_1;
  r_z_tmp = rt_powd_snf(q2__3, 3.0);
  q1_idx_2 = Mt * z[12];
  c_h_dddt_tmp = b_z_tmp * z[18];
  s_z_tmp = 2.0 * r_z_tmp * z[19];
  b_z[3] =
      -((((-(z[19] * ((q2__3_tmp + b_z_tmp) - 1.0) * d_z_tmp) +
           ((((((((((((-(z[12] * z[12]) + i_z_tmp) - e_h_dt_tmp) + h_z_tmp) -
                    e_z_tmp) -
                   b_t) *
                      q2__3 +
                  (q1_idx_2 + p_z_tmp) * q2_idx_0) +
                 (Mt * z[13] + z_tmp) * q2_idx_1) *
                    param[2] +
                s_z_tmp) -
               c_h_dddt_tmp) -
              2.0 * ((b_h_tmp - l_z_tmp) + 0.5) * z[19] * q2__3) -
             z[18] * q2__3_tmp) +
            z[18]) *
               b_h_dddt_tmp) +
          (((((((absxk - 2.0 * z[15] * c_z_tmp) * q1_idx_0 +
                (d_h_dt_tmp - 2.0 * z[16] * c_z_tmp) * q1_idx_1) *
                   q2__3 +
               ((((((-(z[13] * z[13]) + h_z_tmp) - h_dt_tmp) + j_z_tmp) - b_t) -
                 f_z_tmp) *
                    q2_idx_0 +
                (b_h_dt_tmp - 2.0 * z[15] * (z[13] - z[16] / 2.0)) * q2_idx_1) *
                   q1_idx_0) -
              (((-z[13] + 2.0 * z[16]) * z[12] - g_z_tmp) * q2_idx_0 +
               q2_idx_1 *
                   (((((Delta - i_z_tmp) + h_dt_tmp) - j_z_tmp) + e_z_tmp) +
                    f_z_tmp)) *
                  q1_idx_1) *
                 param[2] -
             z[19] *
                 (((q1__3_tmp - 2.0 * q2_idx_0 * q1_idx_0) -
                   2.0 * q1_idx_1 * q2_idx_1) -
                  1.0) *
                 k_z_tmp) -
            2.0 * z[18] * b_h_tmp * q2__3) -
           2.0 * ((m_z_tmp + n_z_tmp) - l_z_tmp) * z[19]) *
              q1__3) +
         o_z_tmp * ((((q1__3_tmp - 1.0) * k_z_tmp / 2.0 + m_z_tmp) + n_z_tmp) -
                    l_z_tmp)) /
        param[2] / q1__3);
  z_tmp = q2__3 * h_dddt_tmp;
  h_z_tmp = k_z_tmp * z[18];
  b_z[4] =
      -((((-d_z_tmp * q2__3 * z[19] +
           ((-(z[12] * z[12]) - e_h_dt_tmp) * param[2] - z[19] * b_h_tmp) *
               b_h_dddt_tmp) +
          ((((-r_z_tmp * z[19] + h_z_tmp) +
             ((e_z_tmp + b_t) * param[2] + 2.0 * z[19]) * q2__3) +
            ((absxk * q1_idx_0 + d_h_dt_tmp * q1_idx_1) - q_z_tmp) * param[2]) -
           z[18]) *
              q1__3) -
         z_tmp * b_h_tmp) /
        param[2] / q1__3);
  b_z[5] = -(
      (((-(d_z_tmp * q2_idx_0 * q2_idx_1 * z[19]) +
         ((((-Mt * z[13] * q2_idx_0 + q1_idx_2 * q2_idx_1) +
            2.0 * q2__3 * (z[12] * z[16] - z[13] * z[15])) *
               param[2] -
           e_h_tmp * q2__3 * z[19]) +
          (g_h_dt_tmp * z[19] + q2_idx_1 * h_dddt_tmp) * q2_idx_0) *
             b_h_dddt_tmp) +
        (((((((-z[12] * z[13] + g_z_tmp) * q2_idx_0 +
              ((((-(z[13] * z[13]) - h_dt_tmp) + j_z_tmp) - e_z_tmp) -
               f_z_tmp) *
                  q2_idx_1) -
             2.0 * q2__3 * z[16] * c_z_tmp) *
                q1_idx_0 +
            q1_idx_1 * ((((((Delta + h_dt_tmp) - j_z_tmp) + b_t) + f_z_tmp) *
                             q2_idx_0 +
                         (b_h_dt_tmp - g_z_tmp) * q2_idx_1) +
                        2.0 * q2__3 * z[15] * c_z_tmp)) *
               param[2] -
           2.0 * q1__3_tmp * q2_idx_0 * q2_idx_1 * z[19]) +
          ((2.0 * q1_idx_1 * q2__3_tmp * z[19] +
            z[19] * (k_z_tmp - 1.0) * q1_idx_1) +
           q2_idx_1 * q2__3 * h_dddt_tmp) *
              q1_idx_0) -
         q2_idx_0 * (z_tmp * q1_idx_1 - q2_idx_1 * z[19])) *
            q1__3) +
       o_z_tmp * ((q1__3_tmp * q2_idx_0 * q2_idx_1 -
                   q1_idx_1 * ((q2__3_tmp + k_z_tmp / 2.0) - 0.5) * q1_idx_0) -
                  q2_idx_0 * q2_idx_1 / 2.0)) /
      param[2] / q1__3);
  Ud[0] = (((k[0] * ((ref[0] * ref_1_tmp + ref[2]) - z[0]) +
             k[6] * (ref[0] * ref[1] * ref_1_dt_tmp - z[9])) +
            k[12] * (-ref[0] * (ref[1] * ref[1]) * ref_1_tmp - h_ddt[0])) +
           k[18] * (-ref[0] * rt_powd_snf(ref[1], 3.0) * ref_1_dt_tmp -
                    h_dddt[0])) +
          ref[0] * rt_powd_snf(ref[1], 4.0) * ref_1_tmp;
  Ud[1] = (((k[1] * ((ref[4] * ref_2_tmp + ref[6]) - z[1]) +
             k[7] * (ref[4] * ref[5] * ref_2_dt_tmp - z[10])) +
            k[13] * (-ref[4] * (ref[5] * ref[5]) * ref_2_tmp - h_ddt[1])) +
           k[19] * (-ref[4] * rt_powd_snf(ref[5], 3.0) * ref_2_dt_tmp -
                    h_dddt[1])) +
          ref[4] * rt_powd_snf(ref[5], 4.0) * ref_2_tmp;
  Ud[2] =
      (((k[2] * ((ref[8] * ref_3_tmp + ref[10]) - z[2]) +
         k[8] * (ref[8] * ref[9] * ref_3_dt_tmp - z[11])) +
        k[14] * (-ref[8] * (ref[9] * ref[9]) * ref_3_tmp - h_ddt[2])) +
       k[20] * (-ref[8] * rt_powd_snf(ref[9], 3.0) * ref_3_dt_tmp - z[20])) +
      ref[8] * rt_powd_snf(ref[9], 4.0) * ref_3_tmp;
  Ud[3] = k[3] * (x - h[3]) + k[9] * (0.0 - h_dt[3]);
  Ud[4] = k[4] * (0.0 - h[4]) + k[10] * (0.0 - h_dt[4]);
  Ud[5] = k[5] * (0.0 - h[5]) + k[11] * (0.0 - h_dt[5]);
  d = q2_idx_0 * q2_idx_0;
  a_tmp = q1_matriax[0] * q2_matriax[0];
  d1 = q1_matriax[1] * q2_matriax[1];
  d2 = a_tmp + d1;
  d3 = q1_idx_0 * q1_idx_0;
  d4 = d / 2.0;
  d5 = q2_matriax[0] * (d1 - 0.5);
  d6 = d1 / 2.0;
  d7 = (d - 0.5) * d3;
  d8 = d5 * q1_matriax[0];
  d9 = (((d7 + d8) - d4) - d6) + 0.5;
  d10 = z[18] * d;
  d11 = z[19] * (d2 - 1.0);
  d12 =
      q2__3 *
      ((q2_matriax[0] * q1_matriax[0] + q1_matriax[1] * q2_matriax[1]) - 1.0) *
      h_dddt_tmp;
  d13 = 2.0 * d3;
  d14 = 2.0 * d;
  d15 = 4.0 * r_z_tmp;
  d16 = (d15 * z[19] - c_h_dddt_tmp) + z[19] * (d14 - 3.0) * q2__3;
  d17 = 2.0 * (d2 - 0.5) * z[18] * q2__3;
  d18 = ((4.0 * z[19] * (d2 - 0.5) * k_z_tmp - d17) - d11) * b_h_dddt_tmp;
  d19 = z[19] * (d13 - 3.0) * r_z_tmp + z[18] * (-d3 + 2.0) * k_z_tmp;
  d20 = ((d19 + 4.0 * z[19] * d9 * q2__3) - 2.0 * z[18] * d9) * q1__3;
  d21 = ((d16 - d10) + z[18]) * d_z_tmp + d18;
  d22 = (d21 + d20) - d12;
  d23 = q2_matriax[0] * q1_matriax[1];
  d24 = q1_matriax[1] * d;
  d25 = d23 * q1_matriax[0];
  d26 = a_tmp * q2_matriax[1];
  d27 = d26 - d24;
  d28 = d3 * q2_matriax[1];
  d29 = d28 - d25;
  d30 = (d29 + q1_matriax[1]) - q2_matriax[1];
  d31 = 2.0 * b_h_dddt_tmp;
  d32 = (d31 * q2_matriax[1] * q2__3 +
         (((d27 - 2.0 * q1_matriax[1] * k_z_tmp) + q1_matriax[1]) -
          q2_matriax[1]) *
             q1__3) +
        q2__3 * d30;
  dv[1] = -param[2] * d32 * q1_matriax[0] * b_h_dddt_tmp * param[1] / d22;
  d33 = q1_matriax[0] * q2__3;
  d34 = q1__3 * q2_matriax[0];
  d35 = d33 - d34;
  q_z_tmp = (d2 + c_h_tmp) - 1.0;
  dv[7] =
      -b_h_dddt_tmp * d35 * q_z_tmp * param[2] * param[1] * q1_matriax[0] / d22;
  c_h_dddt_tmp = q1_matriax[0] * q2_matriax[1];
  b_h_dddt_tmp_tmp = q1_matriax[1] - q2_matriax[1];
  d_h_dddt_tmp = q2_matriax[0] * b_h_dddt_tmp_tmp;
  ref_3_dt_tmp = (-d24 + q1_matriax[1] / 2.0) - q2_matriax[1] / 2.0;
  d36 = q1_matriax[0] * q1_matriax[1];
  ref_2_dt_tmp = d3 * q2_matriax[0];
  ref_3_tmp = c_h_dddt_tmp - d23;
  x = d36 * k_z_tmp;
  ref_1_dt_tmp = ref_2_dt_tmp * q2_matriax[1];
  ref_2_tmp = b_h_dddt_tmp * q2_matriax[0] * q2_matriax[1];
  ref_1_tmp = (ref_2_tmp / 2.0 + q2__3 * ref_3_tmp * q1__3 / 2.0) - x / 2.0;
  dv[13] = 2.0 * param[2] * q1_matriax[0] *
           (((ref_1_tmp + ref_1_dt_tmp) + ref_3_dt_tmp * q1_matriax[0]) +
            d_h_dddt_tmp / 2.0) *
           b_h_dddt_tmp * param[1] / d22;
  h_tmp = rt_powd_snf(q1_idx_0, 3.0);
  d_h_tmp = h_tmp * q2_matriax[0] * q2_matriax[1];
  c_h_dt_tmp = q1_matriax[1] * k_z_tmp;
  f_h_dt_tmp = c_h_dt_tmp * z[18];
  q2__3_tmp = d27 + q1_matriax[1];
  b_h_tmp = z[18] * q2__3_tmp / 2.0;
  e_h_tmp = z[19] * (d - 1.5);
  g_h_dt_tmp = e_h_tmp * q2__3;
  q1__3_tmp = (r_z_tmp * z[19] - h_z_tmp / 2.0) + g_h_dt_tmp / 2.0;
  c_h_tmp = (d2 - 0.5) * z[18];
  d37 = c_h_tmp * q2__3 / 2.0;
  d38 = z[19] * (d2 - 0.5);
  d39 = d11 / 4.0;
  d40 = ((d38 * k_z_tmp - d37) - d39) * b_h_dddt_tmp;
  d41 = z[18] * (d3 - 2.0);
  d42 = z[19] * (d3 - 1.5) * r_z_tmp / 2.0 - d41 * k_z_tmp / 4.0;
  d9 = ((d42 + z[19] * d9 * q2__3) - z[18] * d9 / 2.0) * q1__3;
  d43 = d12 / 4.0;
  d44 = q1__3 + q2__3;
  d45 = rt_powd_snf(q1__3, 4.0);
  d46 = ((q1__3_tmp + z[18] / 4.0) - d10 / 4.0) * d_z_tmp + d40;
  d47 = (d46 + d9) - d43;
  d48 = q1_matriax[0] * b_h_dddt_tmp_tmp;
  d49 = d48 * q2_matriax[0];
  ref_3_dt_tmp *= d3;
  d50 = q1_matriax[1] * (-d + 0.5);
  d51 = d50 * d3;
  d52 = d49 / 2.0;
  d53 = -param[2] * param[1];
  dv[19] =
      d53 *
      ((((q1_matriax[0] * d45 * q2_matriax[0] * q2_matriax[1] * z[19] / 2.0 +
          d33 * z[19] * (c_h_dddt_tmp - d_h_dddt_tmp) * d_z_tmp / 2.0) +
         ((-(((b_h_dddt_tmp_tmp * d3 + d25) + 2.0 * q2_matriax[1]) * z[19] *
             k_z_tmp) /
               2.0 +
           q2_matriax[1] * q2__3 * z[18] / 2.0) +
          z[19] * (((d_h_tmp + ref_3_dt_tmp) + d52) + q2_matriax[1] / 2.0)) *
             b_h_dddt_tmp) +
        (((-(q1_matriax[1] * z[19] * (d3 - 2.0) * r_z_tmp) / 2.0 -
           f_h_dt_tmp / 2.0) +
          ((((d_h_tmp + d51) -
             1.5 * q1_matriax[0] * q2_matriax[0] * q2_matriax[1]) +
            d24) -
           q1_matriax[1]) *
              z[19] * q2__3) +
         b_h_tmp) *
            q1__3) -
       q2__3 * (d29 - q2_matriax[1]) * h_dddt_tmp / 2.0) /
      d47 / d44 / 2.0;
  c_h_dddt_tmp = q2__3 * z[18];
  d_h_dddt_tmp = c_h_dddt_tmp / 2.0;
  d54 = k_z_tmp * z[19] - d_h_dddt_tmp;
  d55 = z[18] * (-(q2_idx_0 * q2_idx_0) + 1.0);
  d9 = (((q1__3_tmp + d55 / 4.0) * d_z_tmp + d40) + d9) - d43;
  q1__3_tmp = d54 - z[19] / 4.0;
  f_h_dt_tmp = param[2] *
               (((2.0 * q2_matriax[1] * q1__3_tmp * d_z_tmp +
                  (((-(2.0 * q1_matriax[1] * r_z_tmp * z[19]) + f_h_dt_tmp) +
                    z[19] * (d27 + 1.5 * q1_matriax[1]) * q2__3) -
                   b_h_tmp) *
                      b_h_dddt_tmp) +
                 q2__3 *
                     (z[19] * (d29 - 1.5 * q2_matriax[1]) * q2__3 -
                      z[18] * (d29 - 2.0 * q2_matriax[1]) / 2.0) *
                     q1__3) +
                c_h_dt_tmp * h_dddt_tmp / 2.0) *
               param[1] / d9 / d44 / 2.0;
  dv[25] = f_h_dt_tmp;
  b_h_tmp = q1_matriax[0] * z[19];
  d40 = (q2_idx_0 * q2_idx_0 - 0.5) * h_tmp;
  d56 = d23 * q2_matriax[1];
  q1_idx_2 = (d + b_z_tmp) - 1.0;
  absxk = 2.0 * q1_matriax[1] * q2_matriax[1];
  b_t = q1_matriax[0] * k_z_tmp;
  Delta = d36 * q2_matriax[1];
  e_h_dt_tmp = q2__3 * ((ref_2_dt_tmp + Delta) - q2_matriax[0]);
  Mt = d5 * d3;
  h_dt_tmp = d3 * q1_matriax[1] * q2_matriax[0] * q2_matriax[1];
  dv[31] =
      param[2] *
      ((((b_h_tmp * q1_idx_2 * d45 / 2.0 +
          b_h_tmp * q2__3 * ((((q2__3 * q2__3 - 1.0) + a_tmp) + d4) + d1) *
              d_z_tmp) +
         (((((h_tmp + d13 * q2_matriax[0]) + q1_matriax[0] * (absxk - 1.0)) -
            2.0 * q2_matriax[0]) *
               z[19] * k_z_tmp / 2.0 +
           q2_matriax[0] * q2__3 * z[18] / 2.0) +
          (((d40 + Mt) +
            (-q1_matriax[1] * q2_matriax[1] - d) * q1_matriax[0] / 2.0) +
           q2_matriax[0] / 2.0) *
              z[19]) *
             b_h_dddt_tmp) +
        (((b_h_tmp * (q1_idx_0 * q1_idx_0 - 2.0) * r_z_tmp / 2.0 +
           b_t * z[18] / 2.0) +
          z[19] * (((d40 + h_dt_tmp) + (1.0 - 1.5 * d) * q1_matriax[0]) - d56) *
              q2__3) +
         z[18] * (q1_matriax[0] * (d - 1.0) + d56) / 2.0) *
            q1__3) -
       e_h_dt_tmp * h_dddt_tmp / 2.0) *
      param[1] / d47 / d44 / 2.0;
  b_h_tmp = d36 * q2_matriax[0];
  b_h_dt_tmp = b_h_tmp * q2_matriax[1];
  d_h_dt_tmp = (a_tmp + absxk) - 1.0;
  dv[2] =
      -param[2] * b_h_dddt_tmp *
      (((q1_idx_2 * d_z_tmp + q2__3 * d_h_dt_tmp * b_h_dddt_tmp) +
        (((((2.0 * (d3 - 1.0) * k_z_tmp + 1.0) +
            (q2_idx_0 * q2_idx_0 - 1.0) * d3) +
           b_h_dt_tmp) -
          d1) -
         d) *
            q1__3) +
       q2__3 * (q1_matriax[0] - 1.0) * (q1_matriax[0] + 1.0) *
           ((q2_matriax[0] * q1_matriax[0] + q1_matriax[1] * q2_matriax[1]) -
            1.0)) *
      param[1] / d22;
  c_z_tmp = q1_matriax[1] * q1__3;
  dv[8] = b_h_dddt_tmp * d35 *
          ((((d29 - c_z_tmp * q2__3) + b_h_dddt_tmp * q2_matriax[1]) +
            q1_matriax[1]) -
           q2_matriax[1]) *
          param[2] * param[1] / d22;
  d29 = q1_matriax[0] * d;
  e_z_tmp = d56 / 2.0;
  f_z_tmp = q2_matriax[0] * (d1 - 1.0);
  g_z_tmp = ((0.5 - d) - d6) * q1_matriax[0];
  h_tmp = (h_tmp - q1_matriax[0]) * k_z_tmp / 2.0;
  i_z_tmp = d_z_tmp * q2_matriax[0];
  dv[14] = 2.0 *
           (((((((i_z_tmp * q2__3 / 2.0 +
                  ((((d29 + b_t / 2.0) + e_z_tmp) - q1_matriax[0] / 2.0) -
                   q2_matriax[0] / 2.0) *
                      b_h_dddt_tmp) +
                 e_h_dt_tmp * q1__3 / 2.0) +
                h_tmp) +
               d40) +
              Mt) +
             g_z_tmp) -
            f_z_tmp / 2.0) *
           param[2] * b_h_dddt_tmp * param[1] / d22;
  d22 = rt_powd_snf(q1__3, 5.0);
  e_h_dt_tmp = q2_matriax[0] * z[18];
  j_z_tmp = z[18] * q1_idx_2;
  l_z_tmp = d3 / 2.0;
  m_z_tmp = (l_z_tmp - 1.0) * k_z_tmp + d7;
  n_z_tmp = (((m_z_tmp + d8) - d4) - d6) + 0.5;
  o_z_tmp = d * q2__3;
  d37 = (((((o_z_tmp / 2.0 + r_z_tmp) - 0.75 * q2__3) * z[19] - j_z_tmp / 4.0) *
              d_z_tmp +
          (((((d2 - 0.5) * k_z_tmp - a_tmp / 4.0) - d1 / 4.0) + 0.25) * z[19] -
           d37) *
              b_h_dddt_tmp) +
         (q2__3 *
              ((((((q1_idx_0 * q1_idx_0 - 1.5) * k_z_tmp / 2.0 + 0.5) + d7) +
                 d8) -
                d4) -
               d6) *
              z[19] -
          n_z_tmp * z[18] / 2.0) *
             q1__3) -
        d43;
  p_z_tmp = q1_matriax[0] + q2_matriax[0];
  dv[20] =
      -(((((d22 * q2_matriax[0] * q2__3 * z[19] / 2.0 +
            ((((p_z_tmp * k_z_tmp / 2.0 + d29) - q1_matriax[0] / 2.0) +
              e_z_tmp) -
             q2_matriax[0] / 2.0) *
                z[19] * d45) +
           q2__3 *
               (((b_t + ref_2_dt_tmp) + ((d1 + d14) - 1.0) * q1_matriax[0]) +
                f_z_tmp) *
               z[19] * d_z_tmp / 2.0) +
          ((((q1_matriax[0] * (((d3 + a_tmp) + d1) - 1.0) * k_z_tmp / 2.0 +
              d40) +
             Mt) +
            g_z_tmp) -
           e_z_tmp) *
              z[19] * b_h_dddt_tmp) +
         (q2__3 *
              ((((h_tmp + d40) + h_dt_tmp) + q1_matriax[0] / 2.0) + e_z_tmp) *
              z[19] -
          e_h_dt_tmp * d2 / 2.0) *
             q1__3) -
        d33 * d2 * h_dddt_tmp / 2.0) *
      param[2] * param[1] / d37 / d44 / 2.0;
  h_tmp = h_dddt_tmp_tmp - z[18] / 2.0;
  d40 = d35 * param[2] *
        (((d54 - z[19] / 2.0) * b_h_dddt_tmp + d2 * h_tmp * q1__3) -
         z_tmp / 2.0) *
        param[1] / d9 / d44 / 2.0;
  dv[26] = d40;
  d54 = d24 / 2.0;
  Mt = -q1_matriax[1] + q2_matriax[1];
  dv[32] =
      -param[2] *
      (((((d22 * q2_matriax[1] * q2__3 * z[19] +
           ((((Mt * k_z_tmp + d26) + q1_matriax[1] / 2.0) -
             q2_matriax[1] / 2.0) -
            d54) *
               z[19] * d45) +
          q2__3 *
              (((((-c_h_dt_tmp + d28) - d49) - d54) + q1_matriax[1]) -
               q2_matriax[1]) *
              z[19] * d_z_tmp) +
         ((((((((-q1_matriax[1] / 2.0 + q2_matriax[1]) * d3 - d25) +
               q1_matriax[1] / 2.0) -
              q2_matriax[1] / 2.0) *
                 k_z_tmp +
             d_h_tmp) +
            ref_3_dt_tmp) +
           q2_matriax[0] * (-2.0 * q2_matriax[1] + q1_matriax[1]) *
               q1_matriax[0] / 2.0) +
          d54) *
             z[19] * b_h_dddt_tmp) +
        ((((-(k_z_tmp * d3 * q1_matriax[1]) / 2.0 + d_h_tmp) + d51) - d54) *
             q2__3 * z[19] -
         e_h_dt_tmp * ref_3_tmp / 2.0) *
            q1__3) -
       d33 * ref_3_tmp * h_dddt_tmp / 2.0) *
      param[1] / d37 / d44 / 2.0;
  d16 = (((d16 + d55) * d_z_tmp + d18) + d20) - d12;
  dv[3] = q1__3 * q_z_tmp *
          (-q1__3 * q2_matriax[0] * z[19] + q1_matriax[0] * h_dddt_tmp) / d16;
  dv[9] = -((2.0 * q2_matriax[1] * h_tmp * b_h_dddt_tmp +
             ((-2.0 * q1_matriax[1] * k_z_tmp * z[19] +
               q1_matriax[1] * q2__3 * z[18]) +
              z[19] * (q2__3_tmp - q2_matriax[1])) *
                 q1__3) +
            h_dddt_tmp * d30) *
          q1__3 / d16;
  d16 = q1_idx_2 * z[19] - c_h_dddt_tmp;
  d18 = z[18] *
        ((q2_matriax[0] * q1_matriax[0] + q1_matriax[1] * q2_matriax[1]) - 1.0);
  dv[15] =
      (((d16 * d_z_tmp +
         (2.0 * q2__3 * (d2 - 0.5) * z[19] - d18) * b_h_dddt_tmp) +
        (((((((q1_idx_0 * q1_idx_0 - 2.0) * k_z_tmp + (d14 - 1.0) * d3) +
             q2_matriax[0] * (2.0 * q1_matriax[1] * q2_matriax[1] - 1.0) *
                 q1_matriax[0]) -
            d1) -
           d) +
          1.0) *
             z[19] +
         c_h_dddt_tmp) *
            q1__3) -
       h_dddt_tmp *
           ((q2_matriax[0] * q1_matriax[0] + q1_matriax[1] * q2_matriax[1]) -
            1.0)) /
      ((((((d14 * q2__3 + d15) - 3.0 * q2__3) * z[19] - j_z_tmp) * d_z_tmp +
         ((((((4.0 * q2_matriax[0] * q1_matriax[0] +
               4.0 * q1_matriax[1] * q2_matriax[1]) -
              2.0) *
                 k_z_tmp -
             a_tmp) -
            d1) +
           1.0) *
              z[19] -
          d17) *
             b_h_dddt_tmp) +
        (4.0 * q2__3 *
             ((((((l_z_tmp - 0.75) * k_z_tmp + d7) + d8) + 0.5) - d4) - d6) *
             z[19] -
         2.0 * n_z_tmp * z[18]) *
            q1__3) -
       d12);
  d15 = (q1_idx_0 * q1_idx_0 - 1.0) * k_z_tmp;
  d17 = h_dddt_tmp * h_dddt_tmp;
  dv[21] =
      ((((-(z[19] * d16 * d45) -
          2.0 * z[19] * (z[19] * (d2 - d4) * q2__3 - z[18] * (d2 - d) / 2.0) *
              d_z_tmp) -
         2.0 * ((((m_z_tmp + b_h_dt_tmp) + 0.5) - d4) * z[19] + d_h_dddt_tmp) *
             z[19] * b_h_dddt_tmp) +
        2.0 * z[19] *
            (((((d15 / 2.0 + 0.5) + d7) +
               q2_matriax[0] * (d1 + 0.5) * q1_matriax[0]) +
              d6) -
             d4) *
            h_dddt_tmp * q1__3) -
       d17 * d2) /
      q1__3 / d37 / d44 / 4.0;
  d16 = z[19] * z[19];
  dv[27] =
      ((((2.0 * z[19] * (((s_z_tmp - h_z_tmp) + g_h_dt_tmp) + d55 / 2.0) * d45 +
          4.0 * z[19] * q1__3_tmp * d2 * d_z_tmp) +
         (((d16 * (d13 - 5.0) * r_z_tmp -
            z[18] * z[19] * (d3 - 5.0) * k_z_tmp) +
           ((((2.0 * (2.0 * (q2_idx_0 * q2_idx_0) - 1.0) * d3 + 3.0) +
              4.0 * q1_matriax[0] * q1_matriax[1] * q2_matriax[0] *
                  q2_matriax[1]) -
             d14) *
                d16 -
            z[18] * z[18]) *
               q2__3) -
          2.0 * z[18] * z[19] * (((d7 + b_h_dt_tmp) - d4) + 1.0)) *
             b_h_dddt_tmp) -
        3.0 * d2 * (h_dddt_tmp_tmp - z[18] / 3.0) * h_dddt_tmp * q1__3) +
       q2__3 * d17) /
      q1__3 / d9 / d44 / 4.0;
  d4 = q2_matriax[0] * q2_matriax[1];
  dv[33] =
      -(((i_z_tmp * q2_matriax[1] * z[19] / 2.0 +
          h_dddt_tmp_tmp * ref_3_tmp * b_h_dddt_tmp / 2.0) +
         z[19] *
             (((-x / 2.0 + ref_1_dt_tmp) + d50 * q1_matriax[0]) - d4 / 2.0) *
             q1__3) -
        h_dddt_tmp * ref_3_tmp / 2.0) *
      ((q1__3 * z[19] - h_dddt_tmp_tmp) + z[18]) / q1__3 / d47 / d44 / 2.0;
  d7 = (d3 - 0.5) * d;
  d8 = (((d7 + d8) - l_z_tmp) - d6) + 0.5;
  d9 = 2.0 * z[18] * d8;
  d12 = (d21 + ((d19 + 4.0 * z[19] * d8 * q2__3) - d9) * q1__3) - d12;
  dv[4] = -q2__3 * param[2] * d32 * q1__3 * q2_matriax[0] * param[1] / d12;
  d13 = -q1__3 * q2__3 * d35;
  dv[10] = d13 * q_z_tmp * param[2] * param[1] * q2_matriax[0] / d12;
  d14 = (d28 + q1_matriax[1] / 2.0) - q2_matriax[1] / 2.0;
  d16 = d24 * q1_matriax[0];
  dv[16] = 2.0 * q2__3 * param[2] *
           (((ref_1_tmp - d16) + d14 * q2_matriax[0]) + d48 / 2.0) * q1__3 *
           q2_matriax[0] * param[1] / d12;
  d17 = Mt * d + d26;
  d19 = d23 + d48;
  d20 = rt_powd_snf(q2_idx_0, 3.0);
  d21 = d36 * d20;
  d22 = q2_matriax[1] * (q1_idx_0 * q1_idx_0 - 0.5);
  d23 = -d21 + d22 * d;
  d24 = q2_matriax[0] * z[19];
  d30 = z[19] * d8;
  d14 *= d;
  d32 = d28 / 2.0;
  dv[22] =
      param[2] *
      (((q2_matriax[1] * ((z[19] * (d - 2.0) * q2__3 - d10) + z[18]) * d_z_tmp /
             2.0 +
         (((d17 + 2.0 * q1_matriax[1]) * z[19] * k_z_tmp -
           (d17 + q1_matriax[1]) * z[18] * q2__3) -
          z[19] * q2__3_tmp) *
             b_h_dddt_tmp / 2.0) +
        (((-(d24 * d19 * r_z_tmp) / 2.0 + e_h_dt_tmp * d19 * k_z_tmp / 2.0) +
          z[19] *
              (((d23 + 1.5 * q2_matriax[0] * q1_matriax[1] * q1_matriax[0]) -
                d28) +
               q2_matriax[1]) *
              q2__3) -
         (((d23 + d25) - d32) + q2_matriax[1] / 2.0) * z[18]) *
            q1__3) +
       q2__3 *
           ((((-(b_h_tmp * k_z_tmp) / 2.0 - d21) + d14) + d52) -
            q1_matriax[1] / 2.0) *
           h_dddt_tmp) *
      param[1] /
      ((d46 + ((d42 + d30 * q2__3) - z[18] * d8 / 2.0) * q1__3) - d43) / d44 /
      2.0;
  dv[28] = f_h_dt_tmp;
  d8 = ((d3 + 2.0 * q2_matriax[0] * q1_matriax[0]) + absxk) - 2.0;
  d5 += d20 / 2.0 + d29;
  d10 = (q1_idx_0 * q1_idx_0 - 0.5) * d20;
  d17 = q1_matriax[0] * (q1_matriax[1] * q2_matriax[1] - 0.5) * d;
  d19 = d10 + d17;
  d21 = (d19 + (-(q1_idx_0 * q1_idx_0) - d1) * q2_matriax[0] / 2.0) +
        q1_matriax[0] / 2.0;
  d25 = d10 + d36 * d * q2_matriax[1];
  d33 = d3 + d31;
  d35 = Delta / 2.0;
  q_z_tmp = rt_powd_snf(q2__3, 4.0);
  c_h_dddt_tmp = -(i_z_tmp * z[18]);
  d20 -= q2_matriax[0];
  d_h_dddt_tmp = z[18] * (q2_idx_0 * q2_idx_0 - 1.0) * b_h_dddt_tmp;
  d9 =
      ((q1__3 * (d33 - 1.5) * z[19] * r_z_tmp / 2.0 +
        (((-(d_z_tmp * z[18]) / 2.0 + d38 * b_h_dddt_tmp) - d41 * q1__3 / 4.0) -
         d39) *
            k_z_tmp) +
       (((e_h_tmp * d_z_tmp / 2.0 - c_h_tmp * b_h_dddt_tmp / 2.0) +
         d30 * q1__3) +
        d18 / 4.0) *
           q2__3) -
      q1__3 * ((d_h_dddt_tmp + d11 * q1__3) + d9) / 4.0;
  dv[34] =
      -param[2] *
      ((((d24 * (d33 - 1.0) * q_z_tmp / 2.0 +
          q2_matriax[0] *
              (((2.0 * d_z_tmp * z[19] - d31 * z[18]) + z[19] * d8 * q1__3) +
               (-(q1_idx_0 * q1_idx_0) + 1.0) * z[18]) *
              r_z_tmp / 2.0) +
         (((c_h_dddt_tmp + z[19] * (d5 - q1_matriax[0]) * b_h_dddt_tmp) -
           e_h_dt_tmp * d8 * q1__3 / 2.0) +
          d21 * z[19]) *
             k_z_tmp) +
        (((d24 * (q2_idx_0 * q2_idx_0 - 2.0) * d_z_tmp / 2.0 -
           (d5 - q1_matriax[0] / 2.0) * z[18] * b_h_dddt_tmp) +
          ((d25 + (-1.5 * d3 + 1.0) * q2_matriax[0]) - Delta) * z[19] * q1__3) -
         d21 * z[18]) *
            q2__3) -
       q1__3 *
           ((z[18] * d20 * b_h_dddt_tmp +
             z[19] * ((d29 + d56) - q1_matriax[0]) * q1__3) +
            2.0 * z[18] * ((d25 + (0.5 - d3) * q2_matriax[0]) - d35)) /
           2.0) *
      param[1] / d9 / d44 / 2.0;
  dv[5] =
      q2__3 * param[2] * q1__3 *
      ((2.0 * ((o_z_tmp + r_z_tmp) - q2__3) * b_h_dddt_tmp +
        (d_h_dt_tmp * k_z_tmp +
         (q2_matriax[0] - 1.0) * (q2_matriax[0] + 1.0) *
             ((q2_matriax[0] * q1_matriax[0] + q1_matriax[1] * q2_matriax[1]) -
              1.0)) *
            q1__3) +
       q2__3 * (((((d15 + (q1_idx_0 * q1_idx_0 - 1.0) * d) + b_h_dt_tmp) - d3) -
                 d1) +
                1.0)) *
      param[1] / d12;
  dv[11] =
      d13 *
      ((((d27 - c_h_dt_tmp) + q1__3 * q2_matriax[1] * q2__3) + q1_matriax[1]) -
       q2_matriax[1]) *
      param[2] * param[1] / d12;
  d5 = q1_matriax[0] * (q1_matriax[1] * q2_matriax[1] - 1.0) / 2.0;
  d1 = ((-d1 / 2.0 - d3) + 0.5) * q2_matriax[0];
  dv[17] = -2.0 * q2__3 * param[2] * q1__3 *
           ((((((q2_matriax[0] * ((d + k_z_tmp) - 1.0) * b_h_dddt_tmp / 2.0 +
                 q2__3 * (((d29 + b_t) + d56) - q1_matriax[0]) * q1__3 / 2.0) +
                (((ref_2_dt_tmp + d35) - q1_matriax[0] / 2.0) -
                 q2_matriax[0] / 2.0) *
                    k_z_tmp) +
               d10) +
              d17) +
             d1) -
            d5) *
           param[1] / d12;
  d8 = q1_matriax[0] * q1__3;
  d10 = (q1_idx_0 * q1_idx_0 - 0.5) * q2_matriax[0] + d5;
  d3 = (d29 / 2.0 + ((d3 + d6) - 0.5) * q2_matriax[0]) + d5;
  d5 = (d2 + d) - 1.0;
  d1 = (d19 + d1) - d35;
  d6 = rt_powd_snf(q2__3, 5.0);
  dv[23] =
      -param[2] *
      (((((d8 * d6 * z[19] / 2.0 +
           ((z[19] * p_z_tmp * b_h_dddt_tmp / 2.0 - d8 * z[18] / 2.0) +
            d10 * z[19]) *
               q_z_tmp) +
          (((i_z_tmp * z[19] / 2.0 - z[18] * p_z_tmp * b_h_dddt_tmp / 2.0) +
            d3 * z[19] * q1__3) -
           z[18] * d10) *
              r_z_tmp) +
         (((c_h_dddt_tmp / 2.0 + d24 * d5 * b_h_dddt_tmp / 2.0) -
           d3 * z[18] * q1__3) +
          d1 * z[19]) *
             k_z_tmp) +
        (((z[19] * d20 * d_z_tmp / 2.0 - e_h_dt_tmp * d5 * b_h_dddt_tmp / 2.0) +
          ((d25 + q2_matriax[0] / 2.0) + d35) * z[19] * q1__3) -
         d1 * z[18]) *
            q2__3) -
       q1__3 *
           ((d_h_dddt_tmp + z[19] * d2 * q1__3) +
            2.0 * z[18] * (((d7 + b_h_dt_tmp) - l_z_tmp) + 0.5)) *
           q2_matriax[0] / 2.0) *
      param[1] / d9 / d44 / 2.0;
  dv[29] = d40;
  d1 = ((d28 - 2.0 * q2_matriax[0] * q1_matriax[1] * q1_matriax[0]) +
        q1_matriax[1]) -
       q2_matriax[1];
  d2 = d_z_tmp * q2_matriax[1];
  d3 = (((-2.0 * q1_matriax[1] * d -
          2.0 * q1_matriax[0] * b_h_dddt_tmp_tmp * q2_matriax[0]) +
         d28) +
        2.0 * q1_matriax[1]) -
       2.0 * q2_matriax[1];
  d5 = (((-q1_matriax[1] + q2_matriax[1] / 2.0) * d + d26) +
        q1_matriax[1] / 2.0) -
       q2_matriax[1] / 2.0;
  d7 = ((-(q1_matriax[0] * q1_matriax[1] * rt_powd_snf(q2_idx_0, 3.0)) + d14) +
        q1_matriax[0] * (q1_matriax[1] - q2_matriax[1] / 2.0) * q2_matriax[0]) -
       d32;
  dv[35] =
      d53 *
      (((((-(c_z_tmp * d6 * z[19]) +
           ((-(z[19] * b_h_dddt_tmp_tmp * b_h_dddt_tmp) + c_z_tmp * z[18]) +
            z[19] * d1 / 2.0) *
               q_z_tmp) +
          (((d2 * z[19] + z[18] * b_h_dddt_tmp_tmp * b_h_dddt_tmp) +
            d3 * z[19] * q1__3 / 2.0) -
           z[18] * d1 / 2.0) *
              r_z_tmp) +
         (((-(d2 * z[18]) + z[19] * d5 * b_h_dddt_tmp) -
           d3 * z[18] * q1__3 / 2.0) +
          d7 * z[19]) *
             k_z_tmp) +
        (((z[19] * d_z_tmp * d * q2_matriax[1] / 2.0 -
           z[18] * d5 * b_h_dddt_tmp) +
          z[19] * (d23 + d32) * q1__3) -
         d7 * z[18]) *
            q2__3) -
       d34 *
           ((ref_2_tmp * z[18] + z[19] * ref_3_tmp * q1__3) +
            2.0 * z[18] * ((-d16 + d22 * q2_matriax[0]) + d36 / 2.0)) /
           2.0) /
      d9 / d44 / 2.0;
  for (int i{0}; i < 6; i++) {
    dv[6 * i] = iv[i];
    b_z[i] += Ud[i];
  }
  for (int i{0}; i < 6; i++) {
    d = 0.0;
    for (int i1{0}; i1 < 6; i1++) {
      d += dv[i + 6 * i1] * b_z[i1];
    }
    Ud[i] = d;
  }
  //  Physical force input
  Mt = param[0] + param[1];
  a_tmp =
      (a_tmp + q1_matriax[1] * q2_matriax[1]) + q1_matriax[2] * q2_matriax[2];
  h_dt_tmp = param[1] * param[1];
  Delta = Mt * Mt - h_dt_tmp * (a_tmp * a_tmp);
  q1_idx_2 = 3.3121686421112381E-170;
  absxk = std::abs(z[12]);
  if (absxk > 3.3121686421112381E-170) {
    b_h_dt_tmp = 1.0;
    q1_idx_2 = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    b_h_dt_tmp = b_t * b_t;
  }
  absxk = std::abs(z[13]);
  if (absxk > q1_idx_2) {
    b_t = q1_idx_2 / absxk;
    b_h_dt_tmp = b_h_dt_tmp * b_t * b_t + 1.0;
    q1_idx_2 = absxk;
  } else {
    b_t = absxk / q1_idx_2;
    b_h_dt_tmp += b_t * b_t;
  }
  absxk = std::abs(z[14]);
  if (absxk > q1_idx_2) {
    b_t = q1_idx_2 / absxk;
    b_h_dt_tmp = b_h_dt_tmp * b_t * b_t + 1.0;
    q1_idx_2 = absxk;
  } else {
    b_t = absxk / q1_idx_2;
    b_h_dt_tmp += b_t * b_t;
  }
  b_h_dt_tmp = q1_idx_2 * std::sqrt(b_h_dt_tmp);
  q1_idx_2 = 3.3121686421112381E-170;
  absxk = std::abs(z[15]);
  if (absxk > 3.3121686421112381E-170) {
    e_h_dt_tmp = 1.0;
    q1_idx_2 = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    e_h_dt_tmp = b_t * b_t;
  }
  absxk = std::abs(z[16]);
  if (absxk > q1_idx_2) {
    b_t = q1_idx_2 / absxk;
    e_h_dt_tmp = e_h_dt_tmp * b_t * b_t + 1.0;
    q1_idx_2 = absxk;
  } else {
    b_t = absxk / q1_idx_2;
    e_h_dt_tmp += b_t * b_t;
  }
  absxk = std::abs(z[17]);
  if (absxk > q1_idx_2) {
    b_t = q1_idx_2 / absxk;
    e_h_dt_tmp = e_h_dt_tmp * b_t * b_t + 1.0;
    q1_idx_2 = absxk;
  } else {
    b_t = absxk / q1_idx_2;
    e_h_dt_tmp += b_t * b_t;
  }
  e_h_dt_tmp = q1_idx_2 * std::sqrt(e_h_dt_tmp);
  d_h_dt_tmp = -param[1] * param[2];
  b_t = e_h_dt_tmp * e_h_dt_tmp;
  absxk = h_dt_tmp * param[2];
  q1_idx_2 = b_h_dt_tmp * b_h_dt_tmp;
  h_dt_tmp = a_tmp * param[1];
  e_h_dt_tmp =
      (z[18] - h_dddt_tmp_tmp) / q1__3 -
      (d_h_dt_tmp * q1_idx_2 * Mt / Delta + absxk * b_t * a_tmp / Delta);
  q1_idx_2 = z[19] -
             (d_h_dt_tmp * b_t * Mt / Delta + absxk * q1_idx_2 * a_tmp / Delta);
  U[0] = q1_matriax[0];
  U[3] = -q1_matriax[1] * q1_matriax[0] / q1__3;
  U[6] = (q1_idx_0 * q1_idx_0 - 1.0) / q1__3;
  U[1] = q1_matriax[1];
  U[4] = (1.0 - q1_idx_1 * q1_idx_1) / q1__3;
  U[7] = d36 / q1__3;
  U[2] = q1__3;
  U[5] = -q1_matriax[1];
  U[8] = q1_matriax[0];
  d = Mt * e_h_dt_tmp + h_dt_tmp * q1_idx_2;
  d1 = Ud[1];
  d2 = Ud[2];
  for (int i{0}; i < 3; i++) {
    F1[i] = (U[i] * d + U[i + 3] * d1) + U[i + 6] * d2;
  }
  U[0] = q2_matriax[0];
  U[3] = -q2_matriax[1] * q2_matriax[0] / q2__3;
  U[6] = (q2_idx_0 * q2_idx_0 - 1.0) / q2__3;
  U[1] = q2_matriax[1];
  U[4] = (1.0 - q2_idx_1 * q2_idx_1) / q2__3;
  U[7] = d4 / q2__3;
  U[2] = q2__3;
  U[5] = -q2_matriax[1];
  U[8] = q2_matriax[0];
  d = h_dt_tmp * e_h_dt_tmp + Mt * q1_idx_2;
  d1 = Ud[4];
  d2 = Ud[5];
  for (int i{0}; i < 3; i++) {
    F2[i] = (U[i] * d + U[i + 3] * d1) + U[i + 6] * d2;
  }
  xi_dot[0] = z[20];
  xi_dot[1] = z[21];
  xi_dot[2] = Ud[0];
  xi_dot[3] = Ud[3];
  for (int i{0}; i < 6; i++) {
    sys_output[i] = h[i];
    sys_output[i + 6] = h_dt[i];
    sys_output[i + 12] = h_ddt[i];
    sys_output[i + 18] = h_dddt[i];
  }
}

//
// File trailer for DEAController.cpp
//
// [EOF]
//
