/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: PadenKahanTwo.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 17-May-2020 16:49:25
 */

/* Include Files */
#include "PadenKahanTwo.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */

/*
 * Arguments    : const double x1[6]
 *                const double x2[6]
 *                const double pp[3]
 *                const double pk[3]
 *                double Theta1Theta2[4]
 * Return Type  : void
 */
void PadenKahanTwo(const double x1[6], const double x2[6], const double pp[3],
                   const double pk[3], double Theta1Theta2[4])
{
  double scale;
  double b_scale;
  double absxk;
  double t;
  double a;
  double t12;
  double t11;
  double b;
  double Points_idx_0;
  double Points_idx_1;
  double Points_idx_2;
  double g[3];
  double Cfg[3];
  double b_a;
  double g_tmp;
  double b_g_tmp;
  double c_g_tmp;
  boolean_T y;
  int k;
  boolean_T exitg1;
  double b_y;
  double u[3];
  double v[3];
  double n[3];
  double m[3];
  double m1p_tmp[9];
  double up[3];
  double m2p[3];
  double up_tmp[9];
  double m1p[3];

  /*  "PADENKAHANTWO" Find the Inverse Kinematics of two consecutive */
  /*  ROTATION SCREWS which which applied to a point move it in SE(3). */
  /*  */
  /*  	Theta1Theta2 = PadenKahanTwo(x1, x2, pp, pk) */
  /*    by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB. */
  /*  */
  /*  In the case of TWO CONSECUTIVE ROTATIONS: */
  /*  Compute angles "Th2" and "Th1" of two subsequently applies SCREWS with */
  /*  corresponding twists x2 and x1, to move the point "p" to the point "k". */
  /*  Beware of the order for the movement, first applied x2 & subsequently x1. */
  /*  Point p is first moved to the point "c" or "d" by the Screw with Theta2 */
  /*  and then moved from "c" or "d" to the point "k" by the Screw with Theta1. */
  /*  */
  /*  exp(E1^Theta1) * exp(E2^Theta2) * p = exp(E1^Theta1) * (c or d) = k */
  /*           |v| */
  /*  Ei = xi =| | is 6x1 ; and p and k are points (3x1). */
  /*           |w| */
  /*  Based on the work of Paden & Kahan subproblem two for INVERSE KINEMATICS. */
  /*  The problem could have up to TWO DOUBLE solutions for Theta1-Theta2: */
  /*  Theta1Theta2 = [t11 t21; t12 t22] */
  /*  as a consequence of the possible paths from p-c-k or p-d-k. */
  /*  The problem could have two solutions for each Theta, which is Theta-2pi,  */
  /*  but the trivial second one is not considered, even though it might be a */
  /*  valid solutions in robotics. */
  /*                */
  /*  See also: PadenKahanOne, PadenKahanThree */
  /*  See also: PardosGotorOne, PardosGotorTwo, PardosGotorThree */
  /*  See also: PardosGotorFour */
  /*   */
  /*  Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor. */
  /*  */
  /*  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB */
  /*   */
  /*  ST24R is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published */
  /*  by the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  ST24R is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with ST24R.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www. */
  /*  */
  /*  CHANGES: */
  /*  Revision 1.1  2019/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  Theta1Theta2  = PadenKahanTwo(x1,x2,pp,pk) */
  /*  */
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = fabs(x1[3]);
  if (absxk > 3.3121686421112381E-170) {
    a = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    a = t * t;
  }

  absxk = fabs(x2[3]);
  if (absxk > 3.3121686421112381E-170) {
    t12 = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    t12 = t * t;
  }

  absxk = fabs(x1[4]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  absxk = fabs(x2[4]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    t12 = t12 * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    t12 += t * t;
  }

  absxk = fabs(x1[5]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  absxk = fabs(x2[5]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    t12 = t12 * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    t12 += t * t;
  }

  a = scale * sqrt(a);
  t11 = a * a;
  t12 = b_scale * sqrt(t12);
  b = t12 * t12;
  Points_idx_0 = (x1[4] * x1[2] - x1[5] * x1[1]) / t11;
  Points_idx_1 = (x1[5] * x1[0] - x1[3] * x1[2]) / t11;
  Points_idx_2 = (x1[3] * x1[1] - x1[4] * x1[0]) / t11;

  /*  */
  /*  "INTERSECTLINES3D" Find intersection point of two axes in SE(3). */
  /*  */
  /*  	m = IntersectLines3D(Axes,Points) */
  /*    by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB. */
  /*  */
  /*  Compute a point "m" (3x1) in intersectin of two lines in 3D. */
  /*  Axis is the matrix (3x2) with the Axis x1 & x2 of the two lines. */
  /*  Point is the matrix (3x2) with the Points p1 & p2 on the two lines. */
  /*  */
  /*  c = p1 ; a point on the Axis of line1. */
  /*  e = x1 ; Axis or DIRECTION VECTOR (normalized) for line1. */
  /*  d = p2 ; a point on the Axis of line2. */
  /*  f = x2 ; Axis or DIRECTION VECTOR (normalized) for line2. */
  /*          || f x g ||  */
  /*  m = c + ----------- e ; for   (f x g)'(f x e)>=0 in same direction */
  /*          || f x e ||  */
  /*  */
  /*          || f x g ||  */
  /*  m = c - ----------- e ; for  (f x g)'(f x e)<0 in different direction */
  /*          || f x e ||  */
  /*  */
  /*  See also:  */
  /*  */
  /*  Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor. */
  /*  */
  /*  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB */
  /*   */
  /*  ST24R is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published */
  /*  by the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  ST24R is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with ST24R.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www. */
  /*  */
  /*  CHANGES: */
  /*  Revision 1.1  2019/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  m = IntersectLines3D(Axes,Points) */
  /*  */
  g[0] = (x2[4] * x2[2] - x2[5] * x2[1]) / b - Points_idx_0;
  g[1] = (x2[5] * x2[0] - x2[3] * x2[2]) / b - Points_idx_1;
  g[2] = (x2[3] * x2[1] - x2[4] * x2[0]) / b - Points_idx_2;
  Cfg[0] = x2[4] * g[2] - x2[5] * g[1];
  Cfg[1] = x2[5] * g[0] - x2[3] * g[2];
  Cfg[2] = x2[3] * g[1] - x2[4] * g[0];
  t12 = x2[5] * x1[4];
  b = x2[4] * x1[5];
  g[0] = b - t12;
  b_a = x2[3] * x1[5];
  g_tmp = x2[5] * x1[3];
  g[1] = g_tmp - b_a;
  b_g_tmp = x2[4] * x1[3];
  c_g_tmp = x2[3] * x1[4];
  g[2] = c_g_tmp - b_g_tmp;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!(g[k] == 0.0)) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (y) {
    g[0] = rtInf;
    g[1] = rtInf;
    g[2] = rtInf;
  } else if ((Cfg[0] * g[0] + Cfg[1] * g[1]) + Cfg[2] * g[2] >= 0.0) {
    scale = 3.3121686421112381E-170;
    b_scale = 3.3121686421112381E-170;
    absxk = fabs(Cfg[0]);
    if (absxk > 3.3121686421112381E-170) {
      b_y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      b_y = t * t;
    }

    absxk = fabs(g[0]);
    if (absxk > 3.3121686421112381E-170) {
      t11 = 1.0;
      b_scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      t11 = t * t;
    }

    absxk = fabs(Cfg[1]);
    if (absxk > scale) {
      t = scale / absxk;
      b_y = b_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_y += t * t;
    }

    absxk = fabs(g[1]);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      t11 = t11 * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      t11 += t * t;
    }

    absxk = fabs(Cfg[2]);
    if (absxk > scale) {
      t = scale / absxk;
      b_y = b_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_y += t * t;
    }

    absxk = fabs(g[2]);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      t11 = t11 * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      t11 += t * t;
    }

    b_y = scale * sqrt(b_y);
    t11 = b_scale * sqrt(t11);
    a = b_y / t11;
    g[0] = Points_idx_0 + a * x1[3];
    g[1] = Points_idx_1 + a * x1[4];
    g[2] = Points_idx_2 + a * x1[5];
  } else {
    scale = 3.3121686421112381E-170;
    b_scale = 3.3121686421112381E-170;
    absxk = fabs(Cfg[0]);
    if (absxk > 3.3121686421112381E-170) {
      b_y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      b_y = t * t;
    }

    absxk = fabs(g[0]);
    if (absxk > 3.3121686421112381E-170) {
      t11 = 1.0;
      b_scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      t11 = t * t;
    }

    absxk = fabs(Cfg[1]);
    if (absxk > scale) {
      t = scale / absxk;
      b_y = b_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_y += t * t;
    }

    absxk = fabs(g[1]);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      t11 = t11 * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      t11 += t * t;
    }

    absxk = fabs(Cfg[2]);
    if (absxk > scale) {
      t = scale / absxk;
      b_y = b_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      b_y += t * t;
    }

    absxk = fabs(g[2]);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      t11 = t11 * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      t11 += t * t;
    }

    b_y = scale * sqrt(b_y);
    t11 = b_scale * sqrt(t11);
    a = b_y / t11;
    g[0] = Points_idx_0 - a * x1[3];
    g[1] = Points_idx_1 - a * x1[4];
    g[2] = Points_idx_2 - a * x1[5];
  }

  /*     */
  scale = 3.3121686421112381E-170;
  absxk = fabs(g[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = fabs(g[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  absxk = fabs(g[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  b_y = scale * sqrt(b_y);
  if (b_y == rtInf) {
    t11 = 0.0;
    b = 0.0;
    t12 = 0.0;
    b_a = 0.0;

    /*  NO SOLUTION */
  } else {
    /*  Calculate the a, b, g parameters for the PadenKahan2 solution */
    Cfg[0] = t12 - b;
    Cfg[1] = b_a - g_tmp;
    Cfg[2] = b_g_tmp - c_g_tmp;
    u[0] = pp[0] - g[0];
    v[0] = pk[0] - g[0];
    u[1] = pp[1] - g[1];
    v[1] = pk[1] - g[1];
    u[2] = pp[2] - g[2];
    v[2] = pk[2] - g[2];
    t11 = (x1[3] * x2[3] + x1[4] * x2[4]) + x1[5] * x2[5];
    scale = 3.3121686421112381E-170;
    b_scale = 3.3121686421112381E-170;
    absxk = fabs(u[0]);
    if (absxk > 3.3121686421112381E-170) {
      a = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      a = t * t;
    }

    absxk = fabs(Cfg[0]);
    if (absxk > 3.3121686421112381E-170) {
      t12 = 1.0;
      b_scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      t12 = t * t;
    }

    absxk = fabs(u[1]);
    if (absxk > scale) {
      t = scale / absxk;
      a = a * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      a += t * t;
    }

    absxk = fabs(Cfg[1]);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      t12 = t12 * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      t12 += t * t;
    }

    absxk = fabs(u[2]);
    if (absxk > scale) {
      t = scale / absxk;
      a = a * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      a += t * t;
    }

    absxk = fabs(Cfg[2]);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      t12 = t12 * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      t12 += t * t;
    }

    b_a = (((t11 * x2[3] * u[0] + t11 * x2[4] * u[1]) + t11 * x2[5] * u[2]) -
           ((x1[3] * v[0] + x1[4] * v[1]) + x1[5] * v[2])) / (t11 * t11 - 1.0);
    b = (((t11 * x1[3] * v[0] + t11 * x1[4] * v[1]) + t11 * x1[5] * v[2]) -
         ((x2[3] * u[0] + x2[4] * u[1]) + x2[5] * u[2])) / (t11 * t11 - 1.0);
    a = scale * sqrt(a);
    b_y = 2.0 * b_a * b;
    t12 = b_scale * sqrt(t12);
    n[0] = (g[0] + b_a * x1[3]) + b * x2[3];
    n[1] = (g[1] + b_a * x1[4]) + b * x2[4];
    n[2] = (g[2] + b_a * x1[5]) + b * x2[5];
    a = sqrt(fabs((((a * a - b_a * b_a) - b * b) - ((b_y * x1[3] * x2[3] + b_y *
      x1[4] * x2[4]) + b_y * x1[5] * x2[5])) / (t12 * t12)));

    /*  Solve the TWO DOUBLE SOLUTIONS using the PadenKahanOne */
    for (k = 0; k < 3; k++) {
      t12 = a * Cfg[k];
      Cfg[k] = t12;
      m[k] = (n[k] + t12) - g[k];
      n[k] = (n[k] - t12) - g[k];
      b = x2[k + 3];
      up_tmp[3 * k] = x2[3] * b;
      up_tmp[3 * k + 1] = x2[4] * b;
      up_tmp[3 * k + 2] = x2[5] * b;
    }

    for (k = 0; k < 3; k++) {
      b = x1[k + 3];
      m1p_tmp[3 * k] = x1[3] * b;
      t12 = up_tmp[k + 3];
      t11 = up_tmp[k] * u[0] + t12 * u[1];
      b_a = up_tmp[k] * m[0] + t12 * m[1];
      m1p_tmp[3 * k + 1] = x1[4] * b;
      t12 = up_tmp[k + 6];
      t11 += t12 * u[2];
      b_a += t12 * m[2];
      m1p_tmp[3 * k + 2] = x1[5] * b;
      up[k] = u[k] - t11;
      m2p[k] = m[k] - b_a;
    }

    for (k = 0; k < 3; k++) {
      t12 = m1p_tmp[k + 3];
      t11 = m1p_tmp[k] * m[0] + t12 * m[1];
      b_a = m1p_tmp[k] * n[0] + t12 * n[1];
      b = m1p_tmp[k] * v[0] + t12 * v[1];
      t12 = m1p_tmp[k + 6];
      t11 += t12 * m[2];
      b_a += t12 * n[2];
      b += t12 * v[2];
      m1p[k] = m[k] - t11;
      Cfg[k] = n[k] - ((up_tmp[k] * n[0] + up_tmp[k + 3] * n[1]) + up_tmp[k + 6]
                       * n[2]);
      u[k] = n[k] - b_a;
      g[k] = v[k] - b;
    }

    b = rt_atan2d_snf((x2[3] * (up[1] * m2p[2] - up[2] * m2p[1]) + x2[4] * (up[2]
      * m2p[0] - up[0] * m2p[2])) + x2[5] * (up[0] * m2p[1] - up[1] * m2p[0]),
                      (up[0] * m2p[0] + up[1] * m2p[1]) + up[2] * m2p[2]);
    t11 = rt_atan2d_snf((x1[3] * (m1p[1] * g[2] - m1p[2] * g[1]) + x1[4] * (m1p
      [2] * g[0] - m1p[0] * g[2])) + x1[5] * (m1p[0] * g[1] - m1p[1] * g[0]),
                        (m1p[0] * g[0] + m1p[1] * g[1]) + m1p[2] * g[2]);
    b_a = rt_atan2d_snf((x2[3] * (up[1] * Cfg[2] - up[2] * Cfg[1]) + x2[4] *
                         (up[2] * Cfg[0] - up[0] * Cfg[2])) + x2[5] * (up[0] *
      Cfg[1] - up[1] * Cfg[0]), (up[0] * Cfg[0] + up[1] * Cfg[1]) + up[2] * Cfg
                        [2]);
    t12 = rt_atan2d_snf((x1[3] * (u[1] * g[2] - u[2] * g[1]) + x1[4] * (u[2] *
      g[0] - u[0] * g[2])) + x1[5] * (u[0] * g[1] - u[1] * g[0]), (u[0] * g[0] +
      u[1] * g[1]) + u[2] * g[2]);
  }

  Theta1Theta2[0] = t11;
  Theta1Theta2[2] = b;
  Theta1Theta2[1] = t12;
  Theta1Theta2[3] = b_a;

  /*     */
}

/*
 * File trailer for PadenKahanTwo.c
 *
 * [EOF]
 */
