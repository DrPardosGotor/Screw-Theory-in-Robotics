/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: PadenKahanTwo.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "PadenKahanTwo.h"
#include "rotm2axang.h"
#include "norm.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

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
  double a;
  double t11;
  double b;
  double Points[6];
  double g[3];
  double Cfg[3];
  boolean_T y;
  int k;
  boolean_T exitg1;
  double t12;
  double t22;
  double u[3];
  double v[3];
  double b_x1;
  double n[3];
  double up[3];
  double m2p[3];
  double m[3];
  double m1p[3];
  double m1p_tmp[9];
  double up_tmp[9];

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
  a = b_norm(*(double (*)[3])&x1[3]);
  t11 = a * a;
  a = b_norm(*(double (*)[3])&x2[3]);
  b = a * a;
  Points[0] = (x1[4] * x1[2] - x1[5] * x1[1]) / t11;
  Points[1] = (x1[5] * x1[0] - x1[3] * x1[2]) / t11;
  Points[2] = (x1[3] * x1[1] - x1[4] * x1[0]) / t11;

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
  g[0] = (x2[4] * x2[2] - x2[5] * x2[1]) / b - Points[0];
  g[1] = (x2[5] * x2[0] - x2[3] * x2[2]) / b - Points[1];
  g[2] = (x2[3] * x2[1] - x2[4] * x2[0]) / b - Points[2];
  Cfg[0] = x2[4] * g[2] - x2[5] * g[1];
  Cfg[1] = x2[5] * g[0] - x2[3] * g[2];
  Cfg[2] = x2[3] * g[1] - x2[4] * g[0];
  g[0] = x2[4] * x1[5] - x2[5] * x1[4];
  g[1] = x2[5] * x1[3] - x2[3] * x1[5];
  g[2] = x2[3] * x1[4] - x2[4] * x1[3];
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
    a = b_norm(Cfg) / b_norm(g);
    g[0] = Points[0] + a * x1[3];
    g[1] = Points[1] + a * x1[4];
    g[2] = Points[2] + a * x1[5];
  } else {
    b = b_norm(Cfg) / b_norm(g);
    g[0] = Points[0] - b * x1[3];
    g[1] = Points[1] - b * x1[4];
    g[2] = Points[2] - b * x1[5];
  }

  /*     */
  if (b_norm(g) == rtInf) {
    t11 = 0.0;
    b = 0.0;
    t12 = 0.0;
    t22 = 0.0;

    /*  NO SOLUTION */
  } else {
    /*  Calculate the a, b, g parameters for the PadenKahan2 solution */
    Cfg[0] = x1[4] * x2[5] - x1[5] * x2[4];
    Cfg[1] = x1[5] * x2[3] - x1[3] * x2[5];
    Cfg[2] = x1[3] * x2[4] - x1[4] * x2[3];
    u[0] = pp[0] - g[0];
    v[0] = pk[0] - g[0];
    t11 = x1[3] * x2[3];
    u[1] = pp[1] - g[1];
    v[1] = pk[1] - g[1];
    t22 = x1[4] * x2[4];
    u[2] = pp[2] - g[2];
    v[2] = pk[2] - g[2];
    t12 = x1[5] * x2[5];
    b = (t11 + t22) + t12;
    b_x1 = (t11 + t22) + t12;
    a = (((b * x2[3] * u[0] + b * x2[4] * u[1]) + b * x2[5] * u[2]) - ((x1[3] *
           v[0] + x1[4] * v[1]) + x1[5] * v[2])) / (b_x1 * b_x1 - 1.0);
    b = (t11 + t22) + t12;
    b_x1 = (t11 + t22) + t12;
    b = (((b * x1[3] * v[0] + b * x1[4] * v[1]) + b * x1[5] * v[2]) - ((x2[3] *
           u[0] + x2[4] * u[1]) + x2[5] * u[2])) / (b_x1 * b_x1 - 1.0);
    t11 = b_norm(u);
    t22 = 2.0 * a * b;
    t12 = b_norm(Cfg);
    n[0] = (g[0] + a * x1[3]) + b * x2[3];
    n[1] = (g[1] + a * x1[4]) + b * x2[4];
    n[2] = (g[2] + a * x1[5]) + b * x2[5];
    a = sqrt(fabs((((t11 * t11 - a * a) - b * b) - ((t22 * x1[3] * x2[3] + t22 *
      x1[4] * x2[4]) + t22 * x1[5] * x2[5])) / (t12 * t12)));

    /*  Solve the TWO DOUBLE SOLUTIONS using the PadenKahanOne */
    for (k = 0; k < 3; k++) {
      t22 = a * Cfg[k];
      Cfg[k] = t22;
      m[k] = (n[k] + t22) - g[k];
      n[k] = (n[k] - t22) - g[k];
      t22 = x2[3 + k];
      b = t22 * x2[3];
      up_tmp[k] = b;
      t11 = b * u[0];
      b = t22 * x2[4];
      up_tmp[k + 3] = b;
      t11 += b * u[1];
      b = t22 * x2[5];
      up_tmp[k + 6] = b;
      t11 += b * u[2];
      up[k] = u[k] - t11;
    }

    for (k = 0; k < 3; k++) {
      b = x1[3 + k];
      m1p_tmp[k] = b * x1[3];
      m1p_tmp[k + 3] = b * x1[4];
      t22 = up_tmp[k + 6];
      m1p_tmp[k + 6] = b * x1[5];
      m2p[k] = m[k] - ((up_tmp[k] * m[0] + up_tmp[k + 3] * m[1]) + t22 * m[2]);
      b = m1p_tmp[k + 3];
      t11 = m1p_tmp[k + 6];
      m1p[k] = m[k] - ((m1p_tmp[k] * m[0] + b * m[1]) + t11 * m[2]);
      Cfg[k] = n[k] - ((up_tmp[k] * n[0] + up_tmp[k + 3] * n[1]) + t22 * n[2]);
      u[k] = n[k] - ((m1p_tmp[k] * n[0] + b * n[1]) + t11 * n[2]);
      g[k] = v[k] - ((m1p_tmp[k] * v[0] + b * v[1]) + t11 * v[2]);
    }

    b = rt_atan2d_snf((x2[3] * (up[1] * m2p[2] - up[2] * m2p[1]) + x2[4] * (up[2]
      * m2p[0] - up[0] * m2p[2])) + x2[5] * (up[0] * m2p[1] - up[1] * m2p[0]),
                      (up[0] * m2p[0] + up[1] * m2p[1]) + up[2] * m2p[2]);
    t11 = rt_atan2d_snf((x1[3] * (m1p[1] * g[2] - m1p[2] * g[1]) + x1[4] * (m1p
      [2] * g[0] - m1p[0] * g[2])) + x1[5] * (m1p[0] * g[1] - m1p[1] * g[0]),
                        (m1p[0] * g[0] + m1p[1] * g[1]) + m1p[2] * g[2]);
    t22 = rt_atan2d_snf((x2[3] * (up[1] * Cfg[2] - up[2] * Cfg[1]) + x2[4] *
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
  Theta1Theta2[3] = t22;

  /*     */
}

/*
 * File trailer for PadenKahanTwo.c
 *
 * [EOF]
 */
