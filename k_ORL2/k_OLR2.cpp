/*
 * File: k_OLR2.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-Apr-2024 20:37:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "k_OLR2.h"
#include "norm.h"
#include "CoordinateTrans.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : const double theta[7]
 *                double weizi[7]
 * Return Type  : void
 */
void k_OLR2(const double theta[7], double weizi[7])
{
  double T01[16];
  double T12[16];
  double T23[16];
  double T34[16];
  double dv0[16];
  double dv1[16];
  double dv2[16];
  double b_T01[16];
  double c_T01[16];
  double d_T01[16];
  double e_T01[16];
  double f_T01[16];
  double g_T01[16];
  double h_T01[16];
  double i_T01[16];
  int k;
  double BE[3];
  int i0;
  double n35[3];
  double T07[16];
  int i1;
  double nx5[3];
  double dir_n[3];
  double c;
  double bet;
  double y_beta;
  double x_gamma;
  double z_alpha;

  /*  z_alpha,y_beta,x_gamma,p_x,p_y,p_z,bet */
  CoordinateTrans(0.0, 0.0, theta[0], 0.0, T01);
  CoordinateTrans(-1.5707963267948966, 0.0, theta[1], 0.0, T12);
  CoordinateTrans(1.5707963267948966, 0.0, theta[2], 300.0, T23);
  CoordinateTrans(-1.5707963267948966, 24.0, theta[3], 0.0, T34);

  /*  T56 = CoordinateTrans(-pi/2,0,theta(6),0); */
  CoordinateTrans(1.5707963267948966, -24.0, theta[4], 300.0, dv0);
  CoordinateTrans(-1.5707963267948966, 0.0, theta[5] + 1.5707963267948966, 0.0,
                  dv1);
  CoordinateTrans(1.5707963267948966, 0.0, theta[6], 0.0, dv2);
  for (k = 0; k < 4; k++) {
    for (i0 = 0; i0 < 4; i0++) {
      b_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        b_T01[k + (i0 << 2)] += T01[k + (i1 << 2)] * T12[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      c_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        c_T01[k + (i0 << 2)] += b_T01[k + (i1 << 2)] * T23[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      d_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        d_T01[k + (i0 << 2)] += c_T01[k + (i1 << 2)] * T34[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      e_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        e_T01[k + (i0 << 2)] += d_T01[k + (i1 << 2)] * dv0[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      f_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        f_T01[k + (i0 << 2)] += e_T01[k + (i1 << 2)] * dv1[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      T07[k + (i0 << 2)] = 0.0;
      g_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        T07[k + (i0 << 2)] += f_T01[k + (i1 << 2)] * dv2[i1 + (i0 << 2)];
        g_T01[k + (i0 << 2)] += T01[k + (i1 << 2)] * T12[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      h_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        h_T01[k + (i0 << 2)] += g_T01[k + (i1 << 2)] * T23[i1 + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      i_T01[k + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        i_T01[k + (i0 << 2)] += h_T01[k + (i1 << 2)] * T34[i1 + (i0 << 2)];
      }
    }
  }

  for (k = 0; k < 3; k++) {
    BE[k] = i_T01[12 + k];
  }

  n35[0] = T07[13] * BE[2] - T07[14] * BE[1];
  n35[1] = T07[14] * BE[0] - T07[12] * BE[2];
  n35[2] = T07[12] * BE[1] - T07[13] * BE[0];
  nx5[0] = T07[13] * -0.0 - T07[14] * -0.0;
  nx5[1] = -T07[14] - T07[12] * -0.0;
  nx5[2] = T07[12] * -0.0 - (-T07[13]);
  dir_n[0] = n35[1] * nx5[2] - n35[2] * nx5[1];
  dir_n[1] = n35[2] * nx5[0] - n35[0] * nx5[2];
  dir_n[2] = n35[0] * nx5[1] - n35[1] * nx5[0];
  c = 0.0;
  for (k = 0; k < 3; k++) {
    c += dir_n[k] * T07[12 + k];
  }

  if (c >= 0.0) {
    c = 0.0;
    for (k = 0; k < 3; k++) {
      c += n35[k] * nx5[k];
    }

    bet = -acos(c / (norm(n35) * b_norm(nx5)));

    /*  beta�� */
  } else {
    c = 0.0;
    for (k = 0; k < 3; k++) {
      c += n35[k] * nx5[k];
    }

    bet = acos(c / (norm(n35) * b_norm(nx5)));
  }

  /*  q = dcm2quat(T07(1:3,1:3)); */
  y_beta = rt_atan2d_snf(T07[8], sqrt(T07[0] * T07[0] + T07[4] * T07[4]));
  if (fabs( y_beta - 1.5707963267948966 ) < 1.0E-6) {
    x_gamma = 0.0;
    z_alpha = rt_atan2d_snf(T07[1], T07[5]);
  } else if (fabs( y_beta - -1.5707963267948966 ) < 1.0E-6) {
    x_gamma = 0.0;
    z_alpha = rt_atan2d_snf(T07[1], T07[5]);
  } else {
    z_alpha = rt_atan2d_snf(-T07[4] / cos(y_beta), T07[0] / cos(y_beta));
    x_gamma = rt_atan2d_snf(-T07[9] / cos(y_beta), T07[10] / cos(y_beta));
  }

  weizi[0] = z_alpha;
  weizi[1] = y_beta;
  weizi[2] = x_gamma;
  for (k = 0; k < 3; k++) {
    weizi[3 + k] = T07[12 + k];
  }

  weizi[6] = bet;
}

/*
 * File trailer for k_OLR2.c
 *
 * [EOF]
 */
