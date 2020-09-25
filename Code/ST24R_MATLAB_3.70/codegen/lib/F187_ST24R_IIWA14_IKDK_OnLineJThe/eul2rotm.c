/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eul2rotm.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "eul2rotm.h"

/* Function Definitions */

/*
 * Arguments    : const double eul[3]
 *                double R[9]
 * Return Type  : void
 */
void eul2rotm(const double eul[3], double R[9])
{
  double ct_idx_0;
  double st_idx_0;
  double ct_idx_1;
  double st_idx_1;
  double ct_idx_2;
  double st_idx_2;
  double R_tmp;
  ct_idx_0 = cos(eul[0]);
  st_idx_0 = sin(eul[0]);
  ct_idx_1 = cos(eul[1]);
  st_idx_1 = sin(eul[1]);
  ct_idx_2 = cos(eul[2]);
  st_idx_2 = sin(eul[2]);
  R[0] = ct_idx_1 * ct_idx_2;
  R[3] = -ct_idx_1 * st_idx_2;
  R[6] = st_idx_1;
  R_tmp = ct_idx_2 * st_idx_0;
  R[1] = ct_idx_0 * st_idx_2 + R_tmp * st_idx_1;
  ct_idx_2 *= ct_idx_0;
  R[4] = ct_idx_2 - st_idx_0 * st_idx_1 * st_idx_2;
  R[7] = -ct_idx_1 * st_idx_0;
  R[2] = st_idx_0 * st_idx_2 - ct_idx_2 * st_idx_1;
  R[5] = R_tmp + ct_idx_0 * st_idx_1 * st_idx_2;
  R[8] = ct_idx_0 * ct_idx_1;
}

/*
 * File trailer for eul2rotm.c
 *
 * [EOF]
 */
