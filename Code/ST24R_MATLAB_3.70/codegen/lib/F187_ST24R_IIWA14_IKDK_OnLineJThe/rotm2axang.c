/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rotm2axang.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "rotm2axang.h"
#include "svd1.h"
#include "complexTimes.h"
#include "sqrt1.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const double R[9]
 *                double axang[4]
 * Return Type  : void
 */
void rotm2axang(const double R[9], double axang[4])
{
  creal_T u;
  creal_T v;
  double ci;
  double b_v[3];
  double q;
  boolean_T rEQ0;
  boolean_T b0;
  int k;
  boolean_T exitg1;
  int loop_ub_tmp;
  double vspecial_data[3];
  int i;
  double b_I[9];
  int i3;
  boolean_T p;
  double U[9];
  double V[9];
  u.re = 0.5 * (((R[0] + R[4]) + R[8]) - 1.0);
  if (!(fabs(u.re) > 1.0)) {
    ci = u.re;
    u.re = acos(ci);
  } else {
    v.re = 1.0 + u.re;
    v.im = 0.0;
    c_sqrt(&v);
    ci = u.re;
    u.re = 1.0 - ci;
    u.im = 0.0;
    c_sqrt(&u);
    if ((-v.im == 0.0) && (u.im == 0.0)) {
    } else {
      ci = v.re * u.im + -v.im * u.re;
      if ((rtIsInf(ci) || rtIsNaN(ci)) && (!rtIsNaN(v.re)) && (!rtIsNaN(-v.im)) &&
          (!rtIsNaN(u.re)) && (!rtIsNaN(u.im))) {
        ci = v.re;
        q = -v.im;
        rescale(&ci, &q);
        ci = u.re;
        q = u.im;
        rescale(&ci, &q);
      }
    }

    ci = u.re;
    u.re = 2.0 * rt_atan2d_snf(ci, v.re);
  }

  ci = 2.0 * sin(u.re);
  b_v[0] = R[5] - R[7];
  b_v[1] = R[6] - R[2];
  b_v[2] = R[1] - R[3];
  b_v[0] /= ci;
  b_v[1] /= ci;
  b_v[2] /= ci;
  if (rtIsNaN(u.re) || rtIsInf(u.re)) {
    ci = rtNaN;
  } else if (u.re == 0.0) {
    ci = 0.0;
  } else {
    ci = fmod(u.re, 3.1415926535897931);
    rEQ0 = (ci == 0.0);
    if (!rEQ0) {
      q = fabs(u.re / 3.1415926535897931);
      rEQ0 = (fabs(q - floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      ci = 0.0;
    } else {
      if (u.re < 0.0) {
        ci += 3.1415926535897931;
      }
    }
  }

  rEQ0 = (ci == 0.0);
  b0 = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!(b_v[k] == 0.0)) {
      b0 = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (rEQ0 || b0) {
    loop_ub_tmp = (rEQ0 || b0);
    k = 3 * loop_ub_tmp;
    if (0 <= k - 1) {
      memset(&vspecial_data[0], 0, (unsigned int)(k * (int)sizeof(double)));
    }

    for (i = 0; i < loop_ub_tmp; i++) {
      memset(&b_I[0], 0, 9U * sizeof(double));
      b_I[0] = 1.0;
      b_I[4] = 1.0;
      b_I[8] = 1.0;
      p = true;
      for (k = 0; k < 9; k++) {
        ci = b_I[k] - R[k];
        b_I[k] = ci;
        if (p && ((!rtIsInf(ci)) && (!rtIsNaN(ci)))) {
          p = true;
        } else {
          p = false;
        }
      }

      if (p) {
        svd(b_I, U, vspecial_data, V);
      } else {
        for (i3 = 0; i3 < 9; i3++) {
          V[i3] = rtNaN;
        }
      }

      vspecial_data[0] = V[6];
      vspecial_data[1] = V[7];
      vspecial_data[2] = V[8];
    }

    k = 0;
    if (rEQ0 || b0) {
      k = 1;
    }

    for (i3 = 0; i3 < k; i3++) {
      b_v[0] = vspecial_data[0];
      b_v[1] = vspecial_data[1];
      b_v[2] = vspecial_data[2];
    }
  }

  ci = 1.0 / sqrt((b_v[0] * b_v[0] + b_v[1] * b_v[1]) + b_v[2] * b_v[2]);
  axang[0] = b_v[0] * ci;
  axang[1] = b_v[1] * ci;
  axang[2] = b_v[2] * ci;
  axang[3] = u.re;
}

/*
 * File trailer for rotm2axang.c
 *
 * [EOF]
 */
