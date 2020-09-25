/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rotm2eul.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "rotm2eul.h"
#include "atan2.h"
#include "rotm2axang.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const double R[9]
 *                double eul[3]
 * Return Type  : void
 */
void rotm2eul(const double R[9], double eul[3])
{
  double sy;
  int R_size[3];
  int i2;
  int b_R_size[3];
  double R_data[1];
  double b_R_data[1];
  double varargin_1_data[1];
  int varargin_1_size[3];
  double varargin_2_data[1];
  int varargin_2_size[3];
  int loop_ub;
  sy = sqrt(R[8] * R[8] + R[7] * R[7]);
  eul[0] = rt_atan2d_snf(R[3], R[0]);
  eul[1] = rt_atan2d_snf(-R[6], sy);
  eul[2] = rt_atan2d_snf(R[7], R[8]);
  if (sy < 2.2204460492503131E-15) {
    R_size[0] = 1;
    R_size[1] = 1;
    R_size[2] = 1;
    for (i2 = 0; i2 < 1; i2++) {
      R_data[0] = -R[1];
    }

    b_R_size[0] = 1;
    b_R_size[1] = 1;
    b_R_size[2] = 1;
    for (i2 = 0; i2 < 1; i2++) {
      b_R_data[0] = R[4];
    }

    b_atan2(R_data, R_size, b_R_data, b_R_size, varargin_1_data, varargin_1_size);
    R_size[0] = 1;
    R_size[1] = 1;
    R_size[2] = 1;
    for (i2 = 0; i2 < 1; i2++) {
      R_data[0] = -R[6];
    }

    b_R_size[0] = 1;
    b_R_size[1] = 1;
    b_R_size[2] = 1;
    for (i2 = 0; i2 < 1; i2++) {
      b_R_data[0] = sy;
    }

    b_atan2(R_data, R_size, b_R_data, b_R_size, varargin_2_data, varargin_2_size);
    loop_ub = varargin_1_size[2];
    for (i2 = 0; i2 < loop_ub; i2++) {
      eul[0] = varargin_1_data[i2];
    }

    loop_ub = varargin_2_size[2];
    for (i2 = 0; i2 < loop_ub; i2++) {
      eul[1] = varargin_2_data[i2];
    }

    eul[2] = 0.0;
  }

  eul[0] = -eul[0];
  eul[1] = -eul[1];
  eul[2] = -eul[2];
  sy = eul[0];
  eul[0] = eul[2];
  eul[2] = sy;
}

/*
 * File trailer for rotm2eul.c
 *
 * [EOF]
 */
