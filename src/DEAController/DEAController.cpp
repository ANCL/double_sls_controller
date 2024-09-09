//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DEAController.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 08-Sep-2024 12:31:23
//

// Include Files
#include "DEAController.h"
#include "rt_nonfinite.h"
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
  double c_z[9];
  double Ud[6];
  double b_z[6];
  double h[6];
  double h_dddt[6];
  double h_ddt[6];
  double h_dt[6];
  double Delta;
  double Mt;
  double ab_z_tmp;
  double absx;
  double absxk;
  double b_h_dddt_tmp;
  double b_h_dddt_tmp_tmp;
  double b_h_dt_tmp;
  double b_h_tmp;
  double b_t;
  double b_z_tmp;
  double b_z_tmp_tmp;
  double b_z_tmp_tmp_tmp;
  double bb_z_tmp;
  double c_h_dddt_tmp_tmp;
  double c_h_dt_tmp;
  double c_h_tmp;
  double c_z_tmp;
  double c_z_tmp_tmp;
  double cb_z_tmp;
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
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double d_h_dt_tmp;
  double d_h_tmp;
  double d_z_tmp;
  double d_z_tmp_tmp;
  double db_z_tmp;
  double e_h_dt_tmp;
  double e_h_tmp;
  double e_z_tmp;
  double e_z_tmp_tmp;
  double eb_z_tmp;
  double f_h_dt_tmp;
  double f_h_tmp;
  double f_z_tmp;
  double f_z_tmp_tmp;
  double fb_z_tmp;
  double g_h_tmp;
  double g_z_tmp;
  double g_z_tmp_tmp;
  double gb_z_tmp;
  double h_dddt_tmp;
  double h_dddt_tmp_tmp;
  double h_dt_tmp;
  double h_tmp;
  double h_z_tmp;
  double hb_z_tmp;
  double i_z_tmp;
  double ib_z_tmp;
  double j_z_tmp;
  double jb_z_tmp;
  double k_z_tmp;
  double kb_z_tmp;
  double l_z_tmp;
  double lb_z_tmp;
  double m_z_tmp;
  double mb_z_tmp;
  double n_z_tmp;
  double nb_z_tmp;
  double o_z_tmp;
  double ob_z_tmp;
  double p_z_tmp;
  double q_z_tmp;
  double r_z_tmp;
  double ref_1_dt_tmp;
  double ref_1_tmp;
  double ref_2_dt_tmp;
  double ref_2_tmp;
  double ref_3_dt_tmp;
  double ref_3_tmp;
  double s_z_tmp;
  double scale;
  double t_z_tmp;
  double u_z_tmp;
  double v_z_tmp;
  double w_z_tmp;
  double x;
  double x_z_tmp;
  double y_z_tmp;
  double z_tmp;
  double z_tmp_tmp;
  double z_tmp_tmp_tmp;
  //  Double SLS Controller
  //  output & derivatives
  h[0] = z[0];
  h[1] = z[1];
  h[2] = z[2];
  h_tmp = z[3] * z[6];
  b_h_tmp = z[4] * z[7];
  c_h_tmp = h_tmp + b_h_tmp;
  d_h_tmp = c_h_tmp + z[5] * z[8];
  h[3] = d_h_tmp;
  h[4] = z[5] - z[8];
  e_h_tmp = z[4] * z[6];
  f_h_tmp = z[3] * z[7];
  g_h_tmp = f_h_tmp - e_h_tmp;
  h[5] = g_h_tmp;
  h_dt[0] = z[9];
  h_dt[1] = z[10];
  h_dt[2] = z[11];
  scale = -z[12] + z[15];
  absx = -z[14] + z[17];
  absxk = z[8] * (-z[13] + z[16]);
  h_dt_tmp = z[13] - z[16];
  h_dt[3] = (((z[14] - z[17]) * z[7] + absxk) * z[3] +
             (absx * z[6] - scale * z[8]) * z[4]) +
            z[5] * (h_dt_tmp * z[6] + z[7] * scale);
  scale = z[6] * z[16];
  b_t = z[7] * z[15];
  b_h_dt_tmp = z[4] * z[12];
  h_dt[4] = ((-z[3] * z[13] + b_h_dt_tmp) + scale) - b_t;
  c_h_dt_tmp = z[5] * z[6];
  d_h_dt_tmp = z[5] * z[7];
  e_h_dt_tmp = z[3] * z[8];
  f_h_dt_tmp = z[4] * z[8];
  h_dt[5] = ((((h_tmp * absx - e_h_dt_tmp * z[15]) + b_h_tmp * absx) -
              f_h_dt_tmp * z[16]) +
             c_h_dt_tmp * z[12]) +
            d_h_dt_tmp * z[13];
  absx = -z[8] * z[19] + z[18];
  h_ddt[0] = (absx * z[3] + c_h_dt_tmp * z[19]) / z[5];
  h_ddt[1] = (absx * z[4] + d_h_dt_tmp * z[19]) / z[5];
  h_ddt[2] = param[3] + z[18];
  h_ddt[3] = 0.0;
  h_ddt[4] = 0.0;
  h_ddt[5] = 0.0;
  h_dddt_tmp_tmp = z[8] * z[19];
  h_dddt_tmp = h_dddt_tmp_tmp - z[18];
  b_h_dddt_tmp = z[5] * z[5];
  absx = z[8] * z[21];
  h_dddt[0] =
      (((((absxk - z[7] * z[17]) * z[19] + z[6] * z[21]) + z[13] * z[18]) *
            b_h_dddt_tmp +
        ((((scale - b_t) * z[19] - absx) + z[20]) * z[3] +
         z[4] * z[14] * h_dddt_tmp) *
            z[5]) -
       (z[3] * z[13] - b_h_dt_tmp) * z[3] * h_dddt_tmp) /
      b_h_dddt_tmp;
  scale = -z[6] * z[16] + b_t;
  b_h_dddt_tmp_tmp = z[3] * z[3];
  c_h_dddt_tmp_tmp = z[3] * z[4];
  absxk = (b_h_dddt_tmp_tmp * z[12] + c_h_dddt_tmp_tmp * z[13]) - z[12];
  h_dddt[1] =
      ((((z[6] * z[17] - z[8] * z[15]) * z[19] + z[7] * z[21]) * b_h_dddt_tmp +
        (((-z[3] * z[14] * z[8] - scale * z[4]) * z[19] +
          z[3] * z[14] * z[18]) -
         z[4] * (absx - z[20])) *
            z[5]) -
       h_dddt_tmp * absxk) /
      b_h_dddt_tmp;
  h_dddt[2] = z[20];
  h_dddt[3] = 0.0;
  h_dddt[4] = 0.0;
  h_dddt[5] = 0.0;
  //  reference & derivatives
  //  Ref 1
  absx = ref[1] * t + ref[3];
  ref_1_tmp = std::sin(absx);
  ref_1_dt_tmp = std::cos(absx);
  //  Ref 2
  absx = ref[5] * t + ref[7];
  ref_2_tmp = std::sin(absx);
  ref_2_dt_tmp = std::cos(absx);
  //  Ref 3
  absx = ref[9] * t + ref[11];
  ref_3_tmp = std::sin(absx);
  ref_3_dt_tmp = std::cos(absx);
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
    absx = std::abs(x);
    if (absx > 180.0) {
      if (x > 0.0) {
        x -= 360.0;
      } else {
        x += 360.0;
      }
      absx = std::abs(x);
    }
    if (absx <= 45.0) {
      x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
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
  z_tmp = z[12] * z[14];
  b_t = z[16] * z[16];
  b_z_tmp = z[12] * z[12];
  z_tmp_tmp = z[13] * z[13];
  c_z_tmp = 2.0 * z_tmp_tmp;
  Delta = z[14] * z[14];
  Mt = z[12] * z[13];
  d_z_tmp = 3.0 * z[12] * z[14];
  e_z_tmp = 2.0 * z[20] * z[13];
  f_z_tmp = 1.5 * z[13] * z[14];
  g_z_tmp = 2.0 * z[12] * z[14];
  h_z_tmp = rt_powd_snf(z[5], 3.0);
  b_h_dt_tmp = z[15] * z[15];
  absx = z[17] * z[17];
  i_z_tmp = z[15] * z[16];
  j_z_tmp = 2.0 * z[21] * z[17];
  k_z_tmp = 2.0 * z[14] * scale;
  l_z_tmp = z[6] * z[15] + z[7] * z[16];
  m_z_tmp = z[6] * z[6];
  n_z_tmp = z[8] * z[8];
  o_z_tmp = z[19] * h_dddt_tmp;
  p_z_tmp = b_z_tmp - z_tmp_tmp;
  q_z_tmp = 2.0 * z[12] * z[13];
  r_z_tmp = 2.0 * h_dddt_tmp;
  s_z_tmp = z[15] * z[17];
  t_z_tmp = z[17] * l_z_tmp;
  u_z_tmp = e_h_dt_tmp * z[19];
  v_z_tmp = z[6] * z[8];
  w_z_tmp = z[3] * n_z_tmp;
  x_z_tmp = e_h_tmp * z[7];
  y_z_tmp = z[3] * m_z_tmp;
  b_z[0] = -(
      ((((((((((((z_tmp + s_z_tmp) * z[8] +
                 (2.0 * z[6] * z[16] - 2.0 * z[7] * z[15]) * z[13]) +
                (-b_t - absx) * z[6]) +
               i_z_tmp * z[7]) *
                  z[19] -
              2.0 * z[21] * h_dt_tmp * z[8]) -
             j_z_tmp * z[7]) -
            z_tmp * z[18]) +
           e_z_tmp) *
              param[2] +
          v_z_tmp * z[19] * h_dddt_tmp) *
             h_z_tmp +
         (((((((((b_z_tmp - c_z_tmp) + Delta) + b_h_dt_tmp) + b_t) * z[8] -
              t_z_tmp) *
                 z[3] +
             z[4] * (Mt * z[8] + k_z_tmp)) *
                z[19] +
            ((-b_z_tmp * z[18] + (c_z_tmp - Delta) * z[18]) -
             2.0 * z[21] * scale) *
                z[3]) -
           2.0 * z[4] *
               ((-(z[14] * z[21] * z[8]) + Mt * z[18] / 2.0) + z[14] * z[20])) *
              param[2] +
          o_z_tmp * (((y_z_tmp - w_z_tmp) + x_z_tmp) - z[6])) *
             b_h_dddt_tmp) +
        (((((((d_z_tmp * z[8] - 2.0 * z[13] * scale) * b_h_dddt_tmp_tmp +
              2.0 * z[4] * (f_z_tmp * z[8] + z[12] * scale) * z[3]) -
             g_z_tmp * z[8]) *
                z[19] +
            ((-2.0 * z[8] * z[21] * z[13] - d_z_tmp * z[18]) + e_z_tmp) *
                b_h_dddt_tmp_tmp) -
           2.0 * z[4] *
               ((-(z[12] * z[21] * z[8]) + f_z_tmp * z[18]) + z[12] * z[20]) *
               z[3]) +
          g_z_tmp * z[18]) *
             param[2] -
         u_z_tmp * h_dddt_tmp * (c_h_tmp - 1.0)) *
            z[5]) +
       r_z_tmp *
           ((p_z_tmp * b_h_dddt_tmp_tmp + q_z_tmp * z[4] * z[3]) - b_z_tmp) *
           param[2] * z[3]) /
      h_z_tmp / param[2]);
  c_z_tmp = 2.0 * z[20] * z[12];
  d_z_tmp = z[16] * z[17];
  f_z_tmp = z[13] * z[14];
  b_z_tmp_tmp = z[4] * m_z_tmp;
  z_tmp_tmp_tmp = h_tmp * z[7];
  c_z_tmp_tmp = z_tmp_tmp_tmp - b_z_tmp_tmp;
  g_z_tmp = ((c_z_tmp_tmp - 2.0 * z[4] * n_z_tmp) + z[4]) - z[7];
  d_z_tmp_tmp = e_h_tmp * z[3];
  b_z_tmp_tmp_tmp = b_h_dddt_tmp_tmp * z[7];
  h_dt_tmp = b_z_tmp_tmp_tmp - d_z_tmp_tmp;
  ab_z_tmp = (h_dt_tmp + z[4]) - z[7];
  bb_z_tmp = rt_powd_snf(z[3], 3.0);
  b_z[1] = -(
      ((((((((((-2.0 * z[13] * z[14] + d_z_tmp) * z[8] +
               (-b_h_dt_tmp - absx) * z[7]) +
              i_z_tmp * z[6]) *
                 z[19] -
             2.0 * z[21] * z[15] * z[8]) +
            j_z_tmp * z[6]) +
           2.0 * z[13] * z[14] * z[18]) *
              param[2] +
          2.0 * z[7] * z[8] * z[19] * h_dddt_tmp) *
             h_z_tmp +
         (((((((-3.0 * z[12] * z[13] * z[3] -
                z[4] * (((z_tmp_tmp - Delta) - b_h_dt_tmp) - b_t)) *
                   z[8] -
               k_z_tmp * z[3]) -
              z[4] * z[17] * l_z_tmp) *
                 z[19] -
             2.0 * z[21] * z[14] * z[3] * z[8]) +
            (3.0 * z[12] * z[13] * z[18] + 2.0 * z[20] * z[14]) * z[3]) -
           2.0 * ((-z_tmp_tmp + Delta) * z[18] / 2.0 + z[21] * scale) * z[4]) *
              param[2] +
          o_z_tmp * g_z_tmp) *
             b_h_dddt_tmp) +
        (((((((3.0 * z[14] *
                   ((z[3] * z[12] * z[4] - z[13] * b_h_dddt_tmp_tmp) +
                    z[13] / 3.0) *
                   z[8] -
               2.0 * scale * absxk) *
                  z[19] -
              2.0 * z[21] * absxk * z[8]) +
             (3.0 * z[13] * z[14] * z[18] + c_z_tmp) * b_h_dddt_tmp_tmp) +
            z[4] * (-3.0 * z[12] * z[14] * z[18] + e_z_tmp) * z[3]) -
           f_z_tmp * z[18]) -
          c_z_tmp) *
             param[2] +
         h_dddt_tmp_tmp * h_dddt_tmp * ab_z_tmp) *
            z[5]) +
       r_z_tmp *
           (((-2.0 * z[12] * z[13] * bb_z_tmp +
              z[4] * p_z_tmp * b_h_dddt_tmp_tmp) +
             q_z_tmp * z[3]) -
            b_z_tmp * z[4]) *
           param[2]) /
      h_z_tmp / param[2]);
  b_z[2] = -0.0;
  c_z_tmp = z[14] - 2.0 * z[17];
  e_z_tmp = 2.0 * n_z_tmp;
  j_z_tmp = z[14] - z[17] / 2.0;
  k_z_tmp = 2.0 * z[13] * z[16];
  l_z_tmp = 2.0 * z[12] * z[15];
  o_z_tmp = 2.0 * z[14] * z[17];
  p_z_tmp = m_z_tmp / 2.0;
  q_z_tmp = (m_z_tmp - 0.5) * b_h_dddt_tmp_tmp;
  e_z_tmp_tmp = c_h_dddt_tmp_tmp * z[6];
  cb_z_tmp = e_z_tmp_tmp * z[7];
  db_z_tmp = rt_powd_snf(z[8], 3.0);
  scale = c_z_tmp * z[12];
  absxk = e_z_tmp * z[18];
  eb_z_tmp = z[18] * m_z_tmp;
  e_z_tmp = (m_z_tmp + e_z_tmp) - 1.0;
  fb_z_tmp = 2.0 * z[4] * z[7];
  gb_z_tmp = z[19] * e_z_tmp;
  hb_z_tmp = c_h_tmp - p_z_tmp;
  f_z_tmp_tmp = (b_h_dddt_tmp_tmp - 1.0) * n_z_tmp;
  ib_z_tmp = f_z_tmp_tmp / 2.0;
  jb_z_tmp = 2.0 * db_z_tmp * z[19];
  kb_z_tmp = (q_z_tmp + cb_z_tmp) - p_z_tmp;
  lb_z_tmp = 2.0 * z[6] * z[3];
  b_z[3] = -(
      (((-(gb_z_tmp * h_z_tmp) +
         ((((((((((((-(z[12] * z[12]) + l_z_tmp) - z_tmp_tmp) + k_z_tmp) -
                  b_h_dt_tmp) -
                 b_t) *
                    z[8] +
                (scale + s_z_tmp) * z[6]) +
               (c_z_tmp * z[13] + d_z_tmp) * z[7]) *
                  param[2] +
              jb_z_tmp) -
             absxk) -
            2.0 * (hb_z_tmp + 0.5) * z[19] * z[8]) -
           eb_z_tmp) +
          z[18]) *
             b_h_dddt_tmp) +
        (((((((z_tmp - 2.0 * z[15] * j_z_tmp) * z[3] +
              (f_z_tmp - 2.0 * z[16] * j_z_tmp) * z[4]) *
                 z[8] +
             ((((((-(z[13] * z[13]) + k_z_tmp) - Delta) + o_z_tmp) - b_t) -
               absx) *
                  z[6] +
              (Mt - 2.0 * z[15] * (z[13] - z[16] / 2.0)) * z[7]) *
                 z[3]) -
            (((-z[13] + 2.0 * z[16]) * z[12] - i_z_tmp) * z[6] +
             z[7] * (((((b_z_tmp - l_z_tmp) + Delta) - o_z_tmp) + b_h_dt_tmp) +
                     absx)) *
                z[4]) *
               param[2] -
           z[19] * (((b_h_dddt_tmp_tmp - lb_z_tmp) - fb_z_tmp) - 1.0) *
               n_z_tmp) -
          2.0 * z[18] * c_h_tmp * z[8]) -
         2.0 * kb_z_tmp * z[19]) *
            z[5]) +
       r_z_tmp * (((ib_z_tmp + q_z_tmp) + cb_z_tmp) - p_z_tmp)) /
      param[2] / z[5]);
  d_z_tmp = z[8] * h_dddt_tmp;
  k_z_tmp = n_z_tmp * z[18];
  l_z_tmp = z[19] * c_h_tmp;
  b_z[4] = -(
      (((-h_z_tmp * z[8] * z[19] +
         ((-(z[12] * z[12]) - z_tmp_tmp) * param[2] - l_z_tmp) * b_h_dddt_tmp) +
        ((((-db_z_tmp * z[19] + k_z_tmp) +
           ((b_h_dt_tmp + b_t) * param[2] + 2.0 * z[19]) * z[8]) +
          ((z_tmp * z[3] + f_z_tmp * z[4]) - t_z_tmp) * param[2]) -
         z[18]) *
            z[5]) -
       d_z_tmp * c_h_tmp) /
      param[2] / z[5]);
  z_tmp = 2.0 * b_h_dddt_tmp_tmp;
  z_tmp_tmp = b_h_dddt_tmp_tmp * z[6];
  f_z_tmp = z_tmp_tmp * z[7];
  s_z_tmp = z[7] * z[8];
  t_z_tmp = z_tmp * z[6];
  mb_z_tmp = h_z_tmp * z[6];
  nb_z_tmp = mb_z_tmp * z[7] * z[19];
  g_z_tmp_tmp = z[6] * z[7];
  ob_z_tmp = g_z_tmp_tmp / 2.0;
  b_z[5] = -(
      (((-nb_z_tmp + ((((-c_z_tmp * z[13] * z[6] + scale * z[7]) +
                        2.0 * z[8] * (z[12] * z[16] - z[13] * z[15])) *
                           param[2] -
                       f_h_tmp * z[8] * z[19]) +
                      (f_h_dt_tmp * z[19] + z[7] * h_dddt_tmp) * z[6]) *
                         b_h_dddt_tmp) +
        (((((((-z[12] * z[13] + i_z_tmp) * z[6] +
              ((((-(z[13] * z[13]) - Delta) + o_z_tmp) - b_h_dt_tmp) - absx) *
                  z[7]) -
             2.0 * z[8] * z[16] * j_z_tmp) *
                z[3] +
            z[4] * ((((((b_z_tmp + Delta) - o_z_tmp) + b_t) + absx) * z[6] +
                     (Mt - i_z_tmp) * z[7]) +
                    2.0 * z[8] * z[15] * j_z_tmp)) *
               param[2] -
           t_z_tmp * z[7] * z[19]) +
          ((2.0 * z[4] * m_z_tmp * z[19] + z[19] * (n_z_tmp - 1.0) * z[4]) +
           s_z_tmp * h_dddt_tmp) *
              z[3]) -
         z[6] * (d_z_tmp * z[4] - z[7] * z[19])) *
            z[5]) +
       r_z_tmp * ((f_z_tmp - z[4] * ((m_z_tmp + n_z_tmp / 2.0) - 0.5) * z[3]) -
                  ob_z_tmp)) /
      param[2] / z[5]);
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
  Ud[3] = k[3] * (x - d_h_tmp) + k[9] * (0.0 - h_dt[3]);
  Ud[4] = k[4] * (0.0 - h[4]) + k[10] * (0.0 - h_dt[4]);
  Ud[5] = k[5] * (0.0 - g_h_tmp) + k[11] * (0.0 - h_dt[5]);
  d = z[6] * (b_h_tmp - 0.5);
  d1 = b_h_tmp / 2.0;
  d2 = d * z[3];
  d3 = (((q_z_tmp + d2) - p_z_tmp) - d1) + 0.5;
  d4 = z[19] * ((z[3] * z[6] + z[4] * z[7]) - 1.0);
  d5 = z[8] * ((z[3] * z[6] + z[4] * z[7]) - 1.0) * h_dddt_tmp;
  d6 = 2.0 * m_z_tmp;
  d7 = 4.0 * db_z_tmp;
  d8 = (d7 * z[19] - absxk) + z[19] * (d6 - 3.0) * z[8];
  d9 = (z[3] * z[6] + z[4] * z[7]) - 0.5;
  d10 = 2.0 * d9 * z[18] * z[8];
  d11 = ((4.0 * z[19] * (c_h_tmp - 0.5) * n_z_tmp - d10) - d4) * b_h_dddt_tmp;
  d12 = z[19] * (z_tmp - 3.0) * db_z_tmp +
        z[18] * (-b_h_dddt_tmp_tmp + 2.0) * n_z_tmp;
  d13 = ((d12 + 4.0 * z[19] * d3 * z[8]) - 2.0 * z[18] * d3) * z[5];
  d14 = ((d8 - eb_z_tmp) + z[18]) * h_z_tmp + d11;
  d15 = (d14 + d13) - d5;
  d16 = 2.0 * b_h_dddt_tmp;
  d17 = (d16 * z[7] * z[8] + g_z_tmp * z[5]) + z[8] * ab_z_tmp;
  dv[1] = -param[2] * d17 * z[3] * b_h_dddt_tmp * param[1] / d15;
  d18 = e_h_dt_tmp - c_h_dt_tmp;
  dv[7] =
      -b_h_dddt_tmp * d18 * (d_h_tmp - 1.0) * param[2] * param[1] * z[3] / d15;
  ref_1_dt_tmp = z[4] - z[7];
  ref_2_tmp = z[6] * ref_1_dt_tmp;
  ref_1_tmp = (-b_z_tmp_tmp + z[4] / 2.0) - z[7] / 2.0;
  g_z_tmp = c_h_dddt_tmp_tmp * n_z_tmp;
  d_h_tmp = b_h_dddt_tmp * z[6] * z[7];
  d19 = (d_h_tmp / 2.0 + z[8] * g_h_tmp * z[5] / 2.0) - g_z_tmp / 2.0;
  dv[13] = 2.0 * param[2] * z[3] *
           (((d19 + f_z_tmp) + ref_1_tmp * z[3]) + ref_2_tmp / 2.0) *
           b_h_dddt_tmp * param[1] / d15;
  d20 = bb_z_tmp * z[6] * z[7];
  d21 = z[4] * n_z_tmp;
  d22 = d21 * z[18];
  d23 = c_z_tmp_tmp + z[4];
  d24 = z[18] * d23 / 2.0;
  d25 = z[19] * (m_z_tmp - 1.5);
  d26 = d25 * z[8];
  d27 = (db_z_tmp * z[19] - k_z_tmp / 2.0) + d26 / 2.0;
  d28 = d9 * z[18];
  d29 = d28 * z[8] / 2.0;
  d30 = z[19] * d9;
  d31 = d4 / 4.0;
  d32 = ((d30 * n_z_tmp - d29) - d31) * b_h_dddt_tmp;
  d33 = z[18] * (b_h_dddt_tmp_tmp - 2.0);
  d34 = z[19] * (b_h_dddt_tmp_tmp - 1.5) * db_z_tmp / 2.0 - d33 * n_z_tmp / 4.0;
  d3 = ((d34 + z[19] * d3 * z[8]) - z[18] * d3 / 2.0) * z[5];
  d35 = d5 / 4.0;
  d36 = z[5] + z[8];
  d37 = rt_powd_snf(z[5], 4.0);
  d38 = ((d27 + z[18] / 4.0) - eb_z_tmp / 4.0) * h_z_tmp + d32;
  d39 = (d38 + d3) - d35;
  d40 = z[3] * ref_1_dt_tmp;
  d41 = d40 * z[6];
  ref_1_tmp *= b_h_dddt_tmp_tmp;
  d42 = z[4] * (-m_z_tmp + 0.5);
  d43 = d42 * b_h_dddt_tmp_tmp;
  d44 = d41 / 2.0;
  d45 = -param[2] * param[1];
  dv[19] =
      d45 *
      ((((z[3] * d37 * z[6] * z[7] * z[19] / 2.0 +
          u_z_tmp * (f_h_tmp - ref_2_tmp) * h_z_tmp / 2.0) +
         ((-(((ref_1_dt_tmp * b_h_dddt_tmp_tmp + d_z_tmp_tmp) + 2.0 * z[7]) *
             z[19] * n_z_tmp) /
               2.0 +
           s_z_tmp * z[18] / 2.0) +
          z[19] * (((d20 + ref_1_tmp) + d44) + z[7] / 2.0)) *
             b_h_dddt_tmp) +
        (((-(z[4] * z[19] * (b_h_dddt_tmp_tmp - 2.0) * db_z_tmp) / 2.0 -
           d22 / 2.0) +
          ((((d20 + d43) - 1.5 * z[3] * z[6] * z[7]) + b_z_tmp_tmp) - z[4]) *
              z[19] * z[8]) +
         d24) *
            z[5]) -
       z[8] * (h_dt_tmp - z[7]) * h_dddt_tmp / 2.0) /
      d39 / d36 / 2.0;
  ref_2_tmp = z[8] * z[18];
  j_z_tmp = ref_2_tmp / 2.0;
  o_z_tmp = n_z_tmp * z[19] - j_z_tmp;
  r_z_tmp = z[18] * (-(z[6] * z[6]) + 1.0);
  d3 = (((d27 + r_z_tmp / 4.0) * h_z_tmp + d32) + d3) - d35;
  d27 = o_z_tmp - z[19] / 4.0;
  d22 = param[2] *
        (((2.0 * z[7] * d27 * h_z_tmp +
           (((-(2.0 * z[4] * db_z_tmp * z[19]) + d22) +
             z[19] * (c_z_tmp_tmp + 1.5 * z[4]) * z[8]) -
            d24) *
               b_h_dddt_tmp) +
          z[8] *
              (z[19] * (h_dt_tmp - 1.5 * z[7]) * z[8] -
               z[18] * (h_dt_tmp - 2.0 * z[7]) / 2.0) *
              z[5]) +
         d21 * h_dddt_tmp / 2.0) *
        param[1] / d3 / d36 / 2.0;
  dv[25] = d22;
  d24 = z[3] * z[19];
  d32 = (z[6] * z[6] - 0.5) * bb_z_tmp;
  ref_3_dt_tmp = c_h_dddt_tmp_tmp * z[7];
  x = z[8] * ((z_tmp_tmp + ref_3_dt_tmp) - z[6]);
  ref_2_dt_tmp = d * b_h_dddt_tmp_tmp;
  ref_3_tmp = b_h_dddt_tmp_tmp * z[4] * z[6] * z[7];
  dv[31] =
      param[2] *
      ((((d24 * e_z_tmp * d37 / 2.0 +
          d24 * z[8] * ((((z[8] * z[8] - 1.0) + h_tmp) + p_z_tmp) + b_h_tmp) *
              h_z_tmp) +
         (((((bb_z_tmp + t_z_tmp) + z[3] * (fb_z_tmp - 1.0)) - 2.0 * z[6]) *
               z[19] * n_z_tmp / 2.0 +
           v_z_tmp * z[18] / 2.0) +
          (((d32 + ref_2_dt_tmp) + (-z[4] * z[7] - m_z_tmp) * z[3] / 2.0) +
           z[6] / 2.0) *
              z[19]) *
             b_h_dddt_tmp) +
        (((d24 * (z[3] * z[3] - 2.0) * db_z_tmp / 2.0 + w_z_tmp * z[18] / 2.0) +
          z[19] *
              (((d32 + ref_3_tmp) + (1.0 - 1.5 * m_z_tmp) * z[3]) - x_z_tmp) *
              z[8]) +
         z[18] * (z[3] * (m_z_tmp - 1.0) + x_z_tmp) / 2.0) *
            z[5]) -
       x * h_dddt_tmp / 2.0) *
      param[1] / d39 / d36 / 2.0;
  d24 = (h_tmp + fb_z_tmp) - 1.0;
  dv[2] = -param[2] * b_h_dddt_tmp *
          (((e_z_tmp * h_z_tmp + z[8] * d24 * b_h_dddt_tmp) +
            (((((2.0 * (z[3] * z[3] - 1.0) * n_z_tmp + 1.0) +
                (z[6] * z[6] - 1.0) * b_h_dddt_tmp_tmp) +
               cb_z_tmp) -
              b_h_tmp) -
             m_z_tmp) *
                z[5]) +
           z[8] * (z[3] - 1.0) * (z[3] + 1.0) *
               ((z[3] * z[6] + z[4] * z[7]) - 1.0)) *
          param[1] / d15;
  i_z_tmp = z[4] * z[5];
  dv[8] =
      b_h_dddt_tmp * d18 *
      ((((h_dt_tmp - i_z_tmp * z[8]) + b_h_dddt_tmp * z[7]) + z[4]) - z[7]) *
      param[2] * param[1] / d15;
  h_dt_tmp = x_z_tmp / 2.0;
  b_z_tmp = z[6] * (b_h_tmp - 1.0);
  c_z_tmp = ((0.5 - m_z_tmp) - d1) * z[3];
  absx = (bb_z_tmp - z[3]) * n_z_tmp / 2.0;
  dv[14] = 2.0 *
           (((((((mb_z_tmp * z[8] / 2.0 +
                  ((((y_z_tmp + w_z_tmp / 2.0) + h_dt_tmp) - z[3] / 2.0) -
                   z[6] / 2.0) *
                      b_h_dddt_tmp) +
                 x * z[5] / 2.0) +
                absx) +
               d32) +
              ref_2_dt_tmp) +
             c_z_tmp) -
            b_z_tmp / 2.0) *
           param[2] * b_h_dddt_tmp * param[1] / d15;
  d15 = rt_powd_snf(z[5], 5.0);
  x = z[6] * z[18];
  scale = z[18] * e_z_tmp;
  absxk = b_h_dddt_tmp_tmp / 2.0;
  b_t = (absxk - 1.0) * n_z_tmp + q_z_tmp;
  Delta = (((b_t + d2) - p_z_tmp) - d1) + 0.5;
  Mt = m_z_tmp * z[8];
  d29 = (((((Mt / 2.0 + db_z_tmp) - 0.75 * z[8]) * z[19] - scale / 4.0) *
              h_z_tmp +
          ((((d9 * n_z_tmp - h_tmp / 4.0) - b_h_tmp / 4.0) + 0.25) * z[19] -
           d29) *
              b_h_dddt_tmp) +
         (z[8] *
              ((((((z[3] * z[3] - 1.5) * n_z_tmp / 2.0 + 0.5) + q_z_tmp) + d2) -
                p_z_tmp) -
               d1) *
              z[19] -
          Delta * z[18] / 2.0) *
             z[5]) -
        d35;
  b_h_dt_tmp = z[3] + z[6];
  dv[20] = -(((((d15 * z[6] * z[8] * z[19] / 2.0 +
                 ((((b_h_dt_tmp * n_z_tmp / 2.0 + y_z_tmp) - z[3] / 2.0) +
                   h_dt_tmp) -
                  z[6] / 2.0) *
                     z[19] * d37) +
                z[8] *
                    (((w_z_tmp + z_tmp_tmp) + ((b_h_tmp + d6) - 1.0) * z[3]) +
                     b_z_tmp) *
                    z[19] * h_z_tmp / 2.0) +
               ((((z[3] * (((b_h_dddt_tmp_tmp + h_tmp) + b_h_tmp) - 1.0) *
                       n_z_tmp / 2.0 +
                   d32) +
                  ref_2_dt_tmp) +
                 c_z_tmp) -
                h_dt_tmp) *
                   z[19] * b_h_dddt_tmp) +
              (z[8] * ((((absx + d32) + ref_3_tmp) + z[3] / 2.0) + h_dt_tmp) *
                   z[19] -
               x * c_h_tmp / 2.0) *
                  z[5]) -
             e_h_dt_tmp * c_h_tmp * h_dddt_tmp / 2.0) *
           param[2] * param[1] / d29 / d36 / 2.0;
  d32 = h_dddt_tmp_tmp - z[18] / 2.0;
  o_z_tmp = d18 * param[2] *
            (((o_z_tmp - z[19] / 2.0) * b_h_dddt_tmp + c_h_tmp * d32 * z[5]) -
             d_z_tmp / 2.0) *
            param[1] / d3 / d36 / 2.0;
  dv[26] = o_z_tmp;
  ref_2_dt_tmp = b_z_tmp_tmp / 2.0;
  ref_3_tmp = -z[4] + z[7];
  dv[32] = -param[2] *
           (((((d15 * z[7] * z[8] * z[19] +
                ((((ref_3_tmp * n_z_tmp + z_tmp_tmp_tmp) + z[4] / 2.0) -
                  z[7] / 2.0) -
                 ref_2_dt_tmp) *
                    z[19] * d37) +
               z[8] *
                   (((((-d21 + b_z_tmp_tmp_tmp) - d41) - ref_2_dt_tmp) + z[4]) -
                    z[7]) *
                   z[19] * h_z_tmp) +
              ((((((((-z[4] / 2.0 + z[7]) * b_h_dddt_tmp_tmp - d_z_tmp_tmp) +
                    z[4] / 2.0) -
                   z[7] / 2.0) *
                      n_z_tmp +
                  d20) +
                 ref_1_tmp) +
                z[6] * (-2.0 * z[7] + z[4]) * z[3] / 2.0) +
               ref_2_dt_tmp) *
                  z[19] * b_h_dddt_tmp) +
             ((((-(n_z_tmp * b_h_dddt_tmp_tmp * z[4]) / 2.0 + d20) + d43) -
               ref_2_dt_tmp) *
                  z[8] * z[19] -
              x * g_h_tmp / 2.0) *
                 z[5]) -
            e_h_dt_tmp * g_h_tmp * h_dddt_tmp / 2.0) *
           param[1] / d29 / d36 / 2.0;
  d8 = (((d8 + r_z_tmp) * h_z_tmp + d11) + d13) - d5;
  dv[3] = z[5] * (((z[3] * z[6] + z[4] * z[7]) + z[5] * z[8]) - 1.0) *
          (-z[5] * z[6] * z[19] + z[3] * h_dddt_tmp) / d8;
  dv[9] = -((2.0 * z[7] * d32 * b_h_dddt_tmp +
             ((-2.0 * z[4] * n_z_tmp * z[19] + f_h_dt_tmp * z[18]) +
              z[19] * (d23 - z[7])) *
                 z[5]) +
            h_dddt_tmp * ab_z_tmp) *
          z[5] / d8;
  d8 = gb_z_tmp - ref_2_tmp;
  d11 = z[18] * ((z[3] * z[6] + z[4] * z[7]) - 1.0);
  dv[15] =
      (((d8 * h_z_tmp + (2.0 * z[8] * d9 * z[19] - d11) * b_h_dddt_tmp) +
        (((((((z[3] * z[3] - 2.0) * n_z_tmp + (d6 - 1.0) * b_h_dddt_tmp_tmp) +
             z[6] * (2.0 * z[4] * z[7] - 1.0) * z[3]) -
            b_h_tmp) -
           m_z_tmp) +
          1.0) *
             z[19] +
         ref_2_tmp) *
            z[5]) -
       h_dddt_tmp * ((z[3] * z[6] + z[4] * z[7]) - 1.0)) /
      ((((((d6 * z[8] + d7) - 3.0 * z[8]) * z[19] - scale) * h_z_tmp +
         ((((((4.0 * z[6] * z[3] + 4.0 * z[4] * z[7]) - 2.0) * n_z_tmp -
             h_tmp) -
            b_h_tmp) +
           1.0) *
              z[19] -
          d10) *
             b_h_dddt_tmp) +
        (4.0 * z[8] *
             ((((((absxk - 0.75) * n_z_tmp + q_z_tmp) + d2) + 0.5) - p_z_tmp) -
              d1) *
             z[19] -
         2.0 * Delta * z[18]) *
            z[5]) -
       d5);
  d7 = h_dddt_tmp * h_dddt_tmp;
  dv[21] =
      ((((-(z[19] * d8 * d37) -
          2.0 * z[19] *
              (z[19] * hb_z_tmp * z[8] - z[18] * (c_h_tmp - m_z_tmp) / 2.0) *
              h_z_tmp) -
         2.0 * ((((b_t + cb_z_tmp) + 0.5) - p_z_tmp) * z[19] + j_z_tmp) *
             z[19] * b_h_dddt_tmp) +
        2.0 * z[19] *
            (((((ib_z_tmp + 0.5) + q_z_tmp) + z[6] * (b_h_tmp + 0.5) * z[3]) +
              d1) -
             p_z_tmp) *
            h_dddt_tmp * z[5]) -
       d7 * c_h_tmp) /
      z[5] / d29 / d36 / 4.0;
  d8 = z[19] * z[19];
  dv[27] =
      ((((2.0 * z[19] * (((jb_z_tmp - k_z_tmp) + d26) + r_z_tmp / 2.0) * d37 +
          4.0 * z[19] * d27 * c_h_tmp * h_z_tmp) +
         (((d8 * (z_tmp - 5.0) * db_z_tmp -
            z[18] * z[19] * (b_h_dddt_tmp_tmp - 5.0) * n_z_tmp) +
           ((((2.0 * (2.0 * (z[6] * z[6]) - 1.0) * b_h_dddt_tmp_tmp + 3.0) +
              4.0 * z[3] * z[4] * z[6] * z[7]) -
             d6) *
                d8 -
            z[18] * z[18]) *
               z[8]) -
          2.0 * z[18] * z[19] * (kb_z_tmp + 1.0)) *
             b_h_dddt_tmp) -
        3.0 * c_h_tmp * (h_dddt_tmp_tmp - z[18] / 3.0) * h_dddt_tmp * z[5]) +
       z[8] * d7) /
      z[5] / d3 / d36 / 4.0;
  dv[33] = -(((nb_z_tmp / 2.0 + h_dddt_tmp_tmp * g_h_tmp * b_h_dddt_tmp / 2.0) +
              z[19] * (((-g_z_tmp / 2.0 + f_z_tmp) + d42 * z[3]) - ob_z_tmp) *
                  z[5]) -
             h_dddt_tmp * g_h_tmp / 2.0) *
           ((z[5] * z[19] - h_dddt_tmp_tmp) + z[18]) / z[5] / d39 / d36 / 2.0;
  d3 = (b_h_dddt_tmp_tmp - 0.5) * m_z_tmp;
  d2 = (((d3 + d2) - absxk) - d1) + 0.5;
  d6 = 2.0 * z[18] * d2;
  d5 = (d14 + ((d12 + 4.0 * z[19] * d2 * z[8]) - d6) * z[5]) - d5;
  dv[4] = -z[8] * param[2] * d17 * z[5] * z[6] * param[1] / d5;
  d7 = -z[5] * z[8] * d18;
  dv[10] = d7 * (((z[3] * z[6] + z[4] * z[7]) + z[5] * z[8]) - 1.0) * param[2] *
           param[1] * z[6] / d5;
  d8 = (b_z_tmp_tmp_tmp + z[4] / 2.0) - z[7] / 2.0;
  d9 = b_z_tmp_tmp * z[3];
  dv[16] = 2.0 * z[8] * param[2] * (((d19 - d9) + d8 * z[6]) + d40 / 2.0) *
           z[5] * z[6] * param[1] / d5;
  d10 = ref_3_tmp * m_z_tmp + z_tmp_tmp_tmp;
  d12 = e_h_tmp + d40;
  d13 = rt_powd_snf(z[6], 3.0);
  d14 = c_h_dddt_tmp_tmp * d13;
  d15 = z[7] * (z[3] * z[3] - 0.5);
  d17 = -d14 + d15 * m_z_tmp;
  d18 = z[6] * z[19];
  ref_2_tmp = z[19] * d2;
  d8 *= m_z_tmp;
  ref_1_tmp = b_z_tmp_tmp_tmp / 2.0;
  dv[22] =
      param[2] *
      (((z[7] * ((z[19] * (m_z_tmp - 2.0) * z[8] - eb_z_tmp) + z[18]) *
             h_z_tmp / 2.0 +
         (((d10 + 2.0 * z[4]) * z[19] * n_z_tmp - (d10 + z[4]) * z[18] * z[8]) -
          z[19] * d23) *
             b_h_dddt_tmp / 2.0) +
        (((-(d18 * d12 * db_z_tmp) / 2.0 + x * d12 * n_z_tmp / 2.0) +
          z[19] *
              (((d17 + 1.5 * z[6] * z[4] * z[3]) - b_z_tmp_tmp_tmp) + z[7]) *
              z[8]) -
         (((d17 + d_z_tmp_tmp) - ref_1_tmp) + z[7] / 2.0) * z[18]) *
            z[5]) +
       z[8] *
           ((((-(e_z_tmp_tmp * n_z_tmp) / 2.0 - d14) + d8) + d44) -
            z[4] / 2.0) *
           h_dddt_tmp) *
      param[1] /
      ((d38 + ((d34 + ref_2_tmp * z[8]) - z[18] * d2 / 2.0) * z[5]) - d35) /
      d36 / 2.0;
  dv[28] = d22;
  d2 = ((b_h_dddt_tmp_tmp + lb_z_tmp) + fb_z_tmp) - 2.0;
  d += d13 / 2.0 + y_z_tmp;
  d10 = (z[3] * z[3] - 0.5) * d13;
  d12 = z[3] * (z[4] * z[7] - 0.5) * m_z_tmp;
  d14 = d10 + d12;
  g_z_tmp = (d14 + (-(z[3] * z[3]) - b_h_tmp) * z[6] / 2.0) + z[3] / 2.0;
  d19 = d10 + c_h_dddt_tmp_tmp * m_z_tmp * z[7];
  d20 = b_h_dddt_tmp_tmp + d16;
  d22 = ref_3_dt_tmp / 2.0;
  d23 = rt_powd_snf(z[8], 4.0);
  d26 = -(mb_z_tmp * z[18]);
  d13 -= z[6];
  d27 = z[18] * (z[6] * z[6] - 1.0) * b_h_dddt_tmp;
  d4 = ((z[5] * (d20 - 1.5) * z[19] * db_z_tmp / 2.0 +
         (((-(h_z_tmp * z[18]) / 2.0 + d30 * b_h_dddt_tmp) - d33 * z[5] / 4.0) -
          d31) *
             n_z_tmp) +
        (((d25 * h_z_tmp / 2.0 - d28 * b_h_dddt_tmp / 2.0) + ref_2_tmp * z[5]) +
         d11 / 4.0) *
            z[8]) -
       z[5] * ((d27 + d4 * z[5]) + d6) / 4.0;
  dv[34] =
      -param[2] *
      ((((d18 * (d20 - 1.0) * d23 / 2.0 +
          z[6] *
              (((2.0 * h_z_tmp * z[19] - d16 * z[18]) + z[19] * d2 * z[5]) +
               (-(z[3] * z[3]) + 1.0) * z[18]) *
              db_z_tmp / 2.0) +
         (((d26 + z[19] * (d - z[3]) * b_h_dddt_tmp) - x * d2 * z[5] / 2.0) +
          g_z_tmp * z[19]) *
             n_z_tmp) +
        (((d18 * (z[6] * z[6] - 2.0) * h_z_tmp / 2.0 -
           (d - z[3] / 2.0) * z[18] * b_h_dddt_tmp) +
          ((d19 + (-1.5 * b_h_dddt_tmp_tmp + 1.0) * z[6]) - ref_3_dt_tmp) *
              z[19] * z[5]) -
         g_z_tmp * z[18]) *
            z[8]) -
       z[5] *
           ((z[18] * d13 * b_h_dddt_tmp +
             z[19] * ((y_z_tmp + x_z_tmp) - z[3]) * z[5]) +
            2.0 * z[18] * ((d19 + (0.5 - b_h_dddt_tmp_tmp) * z[6]) - d22)) /
           2.0) *
      param[1] / d4 / d36 / 2.0;
  dv[5] =
      param[2] * z[8] * z[5] *
      ((2.0 * ((Mt + db_z_tmp) - z[8]) * b_h_dddt_tmp +
        (d24 * n_z_tmp +
         (z[6] - 1.0) * (z[6] + 1.0) * ((z[3] * z[6] + z[4] * z[7]) - 1.0)) *
            z[5]) +
       z[8] * (((((f_z_tmp_tmp + (z[3] * z[3] - 1.0) * m_z_tmp) + cb_z_tmp) -
                 b_h_dddt_tmp_tmp) -
                b_h_tmp) +
               1.0)) *
      param[1] / d5;
  dv[11] = d7 * ((((c_z_tmp_tmp - d21) + d_h_dt_tmp * z[8]) + z[4]) - z[7]) *
           param[2] * param[1] / d5;
  d = z[3] * (z[4] * z[7] - 1.0) / 2.0;
  d2 = ((-b_h_tmp / 2.0 - b_h_dddt_tmp_tmp) + 0.5) * z[6];
  dv[17] = -2.0 * z[8] * param[2] * z[5] *
           ((((((z[6] * ((m_z_tmp + n_z_tmp) - 1.0) * b_h_dddt_tmp / 2.0 +
                 z[8] * (((y_z_tmp + w_z_tmp) + x_z_tmp) - z[3]) * z[5] / 2.0) +
                (((z_tmp_tmp + d22) - z[3] / 2.0) - z[6] / 2.0) * n_z_tmp) +
               d10) +
              d12) +
             d2) -
            d) *
           param[1] / d5;
  d5 = z[3] * z[5];
  d6 = (z[3] * z[3] - 0.5) * z[6] + d;
  d += y_z_tmp / 2.0 + ((b_h_dddt_tmp_tmp + d1) - 0.5) * z[6];
  d1 = (c_h_tmp + m_z_tmp) - 1.0;
  d2 = (d14 + d2) - d22;
  d7 = rt_powd_snf(z[8], 5.0);
  dv[23] =
      -param[2] *
      (((((d5 * d7 * z[19] / 2.0 +
           ((z[19] * b_h_dt_tmp * b_h_dddt_tmp / 2.0 - d5 * z[18] / 2.0) +
            d6 * z[19]) *
               d23) +
          (((mb_z_tmp * z[19] / 2.0 - z[18] * b_h_dt_tmp * b_h_dddt_tmp / 2.0) +
            d * z[19] * z[5]) -
           z[18] * d6) *
              db_z_tmp) +
         (((d26 / 2.0 + d18 * d1 * b_h_dddt_tmp / 2.0) - d * z[18] * z[5]) +
          d2 * z[19]) *
             n_z_tmp) +
        (((z[19] * d13 * h_z_tmp / 2.0 - x * d1 * b_h_dddt_tmp / 2.0) +
          ((d19 + z[6] / 2.0) + d22) * z[19] * z[5]) -
         d2 * z[18]) *
            z[8]) -
       z[5] *
           ((d27 + l_z_tmp * z[5]) +
            2.0 * z[18] * (((d3 + cb_z_tmp) - absxk) + 0.5)) *
           z[6] / 2.0) *
      param[1] / d4 / d36 / 2.0;
  dv[29] = o_z_tmp;
  d = ((b_z_tmp_tmp_tmp - 2.0 * z[6] * z[4] * z[3]) + z[4]) - z[7];
  d1 = h_z_tmp * z[7];
  d2 = (((-2.0 * z[4] * m_z_tmp - 2.0 * z[3] * ref_1_dt_tmp * z[6]) +
         b_z_tmp_tmp_tmp) +
        2.0 * z[4]) -
       2.0 * z[7];
  d3 = (((-z[4] + z[7] / 2.0) * m_z_tmp + z_tmp_tmp_tmp) + z[4] / 2.0) -
       z[7] / 2.0;
  d5 = ((-(z[3] * z[4] * rt_powd_snf(z[6], 3.0)) + d8) +
        z[3] * (z[4] - z[7] / 2.0) * z[6]) -
       ref_1_tmp;
  dv[35] =
      d45 *
      (((((-(i_z_tmp * d7 * z[19]) +
           ((-(z[19] * ref_1_dt_tmp * b_h_dddt_tmp) + i_z_tmp * z[18]) +
            z[19] * d / 2.0) *
               d23) +
          (((d1 * z[19] + z[18] * ref_1_dt_tmp * b_h_dddt_tmp) +
            d2 * z[19] * z[5] / 2.0) -
           z[18] * d / 2.0) *
              db_z_tmp) +
         (((-(d1 * z[18]) + z[19] * d3 * b_h_dddt_tmp) -
           d2 * z[18] * z[5] / 2.0) +
          d5 * z[19]) *
             n_z_tmp) +
        (((z[19] * h_z_tmp * m_z_tmp * z[7] / 2.0 - z[18] * d3 * b_h_dddt_tmp) +
          z[19] * (d17 + ref_1_tmp) * z[5]) -
         d5 * z[18]) *
            z[8]) -
       c_h_dt_tmp *
           ((d_h_tmp * z[18] + z[19] * g_h_tmp * z[5]) +
            2.0 * z[18] * ((-d9 + d15 * z[6]) + c_h_dddt_tmp_tmp / 2.0)) /
           2.0) /
      d4 / d36 / 2.0;
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
  c_z_tmp = (z[3] * z[6] + z[4] * z[7]) + z[5] * z[8];
  b_h_dt_tmp = param[1] * param[1];
  Delta = Mt * Mt - b_h_dt_tmp * (c_z_tmp * c_z_tmp);
  scale = 3.3121686421112381E-170;
  absxk = std::abs(z[12]);
  if (absxk > 3.3121686421112381E-170) {
    h_dt_tmp = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    h_dt_tmp = b_t * b_t;
  }
  absxk = std::abs(z[13]);
  if (absxk > scale) {
    b_t = scale / absxk;
    h_dt_tmp = h_dt_tmp * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    h_dt_tmp += b_t * b_t;
  }
  absxk = std::abs(z[14]);
  if (absxk > scale) {
    b_t = scale / absxk;
    h_dt_tmp = h_dt_tmp * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    h_dt_tmp += b_t * b_t;
  }
  h_dt_tmp = scale * std::sqrt(h_dt_tmp);
  scale = 3.3121686421112381E-170;
  absxk = std::abs(z[15]);
  if (absxk > 3.3121686421112381E-170) {
    absx = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    absx = b_t * b_t;
  }
  absxk = std::abs(z[16]);
  if (absxk > scale) {
    b_t = scale / absxk;
    absx = absx * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    absx += b_t * b_t;
  }
  absxk = std::abs(z[17]);
  if (absxk > scale) {
    b_t = scale / absxk;
    absx = absx * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    absx += b_t * b_t;
  }
  absx = scale * std::sqrt(absx);
  b_z_tmp = -param[1] * param[2];
  b_t = absx * absx;
  scale = b_h_dt_tmp * param[2];
  absx = h_dt_tmp * h_dt_tmp;
  b_h_dt_tmp = c_z_tmp * param[1];
  absxk = (z[18] - h_dddt_tmp_tmp) / z[5] -
          (b_z_tmp * absx * Mt / Delta + scale * b_t * c_z_tmp / Delta);
  b_t = z[19] - (b_z_tmp * b_t * Mt / Delta + scale * absx * c_z_tmp / Delta);
  absx = Mt * absxk + b_h_dt_tmp * b_t;
  absxk = b_h_dt_tmp * absxk + Mt * b_t;
  c_z[0] = z[3];
  c_z[3] = -z[4] * z[3] / z[5];
  c_z[6] = (z[3] * z[3] - 1.0) / z[5];
  c_z[1] = z[4];
  c_z[4] = (1.0 - z[4] * z[4]) / z[5];
  c_z[7] = c_h_dddt_tmp_tmp / z[5];
  c_z[2] = z[5];
  c_z[5] = -z[4];
  c_z[8] = z[3];
  b_t = Ud[1];
  scale = Ud[2];
  for (int i{0}; i < 3; i++) {
    F1[i] = (c_z[i] * absx + c_z[i + 3] * b_t) + c_z[i + 6] * scale;
  }
  c_z[0] = z[6];
  c_z[3] = -z[7] * z[6] / z[8];
  c_z[6] = (z[6] * z[6] - 1.0) / z[8];
  c_z[1] = z[7];
  c_z[4] = (1.0 - z[7] * z[7]) / z[8];
  c_z[7] = g_z_tmp_tmp / z[8];
  c_z[2] = z[8];
  c_z[5] = -z[7];
  c_z[8] = z[6];
  b_t = Ud[4];
  scale = Ud[5];
  for (int i{0}; i < 3; i++) {
    F2[i] = (c_z[i] * absxk + c_z[i + 3] * b_t) + c_z[i + 6] * scale;
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
