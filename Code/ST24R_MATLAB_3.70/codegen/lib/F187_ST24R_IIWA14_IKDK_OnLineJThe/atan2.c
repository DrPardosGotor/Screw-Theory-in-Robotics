/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: atan2.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "atan2.h"
#include "rotm2axang.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const double y_data[]
 *                const int y_size[3]
 *                const double x_data[]
 *                const int x_size[3]
 *                double r_data[]
 *                int r_size[3]
 * Return Type  : void
 */
void b_atan2(const double y_data[], const int y_size[3], const double x_data[],
             const int x_size[3], double r_data[], int r_size[3])
{
  signed char csz_idx_2;
  if (y_size[2] <= x_size[2]) {
    csz_idx_2 = (signed char)y_size[2];
  } else {
    csz_idx_2 = 0;
  }

  r_size[0] = 1;
  r_size[1] = 1;
  r_size[2] = csz_idx_2;
  if (0 <= csz_idx_2 - 1) {
    r_data[0] = rt_atan2d_snf(y_data[0], x_data[0]);
  }
}

/*
 * File trailer for atan2.c
 *
 * [EOF]
 */
