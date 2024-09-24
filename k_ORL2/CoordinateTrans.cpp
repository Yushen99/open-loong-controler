/*
 * File: CoordinateTrans.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-Apr-2024 20:37:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "k_OLR2.h"
#include "CoordinateTrans.h"

/* Function Definitions */

/*
 * 功能----------依据MDH四参数，计算相邻坐标系转移矩阵
 * 输入----------MDH四参数
 * 输出----------相邻坐标系转移矩阵4x4
 * Arguments    : double alpha0
 *                double a0
 *                double theta1
 *                double d1
 *                double coordinateTrans[16]
 * Return Type  : void
 */
void CoordinateTrans(double alpha0, double a0, double theta1, double d1, double
                     coordinateTrans[16])
{
  double ca;
  double sa;
  int i2;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  if (fabs(alpha0 - 1.5707963267948966) < 1.0E-6) {
    ca = 0.0;
    sa = 1.0;
  } else if (fabs(alpha0 + 1.5707963267948966) < 1.0E-6) {
    ca = 0.0;
    sa = -1.0;
  } else if (fabs(alpha0 - 3.1415926535897931) < 1.0E-6) {
    ca = -1.0;
    sa = 0.0;
  } else {
    ca = cos(alpha0);
    sa = sin(alpha0);
  }

  coordinateTrans[0] = cos(theta1);
  coordinateTrans[4] = -sin(theta1);
  coordinateTrans[8] = 0.0;
  coordinateTrans[12] = a0;
  coordinateTrans[1] = sin(theta1) * ca;
  coordinateTrans[5] = cos(theta1) * ca;
  coordinateTrans[9] = -sa;
  coordinateTrans[13] = -sa * d1;
  coordinateTrans[2] = sin(theta1) * sa;
  coordinateTrans[6] = cos(theta1) * sa;
  coordinateTrans[10] = ca;
  coordinateTrans[14] = ca * d1;
  for (i2 = 0; i2 < 4; i2++) {
    coordinateTrans[3 + (i2 << 2)] = iv0[i2];
  }
}

/*
 * File trailer for CoordinateTrans.c
 *
 * [EOF]
 */
