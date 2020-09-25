/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: PadenKahanOne.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "PadenKahanOne.h"
#include "rotm2axang.h"
#include "norm.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const double x1[6]
 *                const double pp[3]
 *                const double pk[3]
 * Return Type  : double
 */
double PadenKahanOne(const double x1[6], const double pp[3], const double pk[3])
{
  double Theta1;
  double a;
  double r1[3];
  int i;
  double u[3];
  double up[3];
  double up_tmp[9];

  /*  "PADENKAHANONE" Find the Inverse Kinematics of a single ROTATION SCREW */
  /*  when applied to a point in SE(3). */
  /*  */
  /*  	Theta1 = PadenKahanOne(x1, pp, pk) */
  /*    by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB. */
  /*  */
  /*  Compute the magnitude "Theta" (angle) of the Screw, to */
  /*  move the point "p" from its original position to the point "k" by */
  /*  the ROTATION of the Twist x1. */
  /*  */
  /*          |v| */
  /*  E = x1 =| | (6x1); and the points p and k are (3x1) coordinates.  */
  /*          |w| */
  /*  exp(E^Theta) * p = k */
  /*  */
  /*  For a SCREW of pure ROTATION: */
  /*  Based on the work of Paden & Kahan subproblem ONE for INVERSE KINEMATICS. */
  /*  For a SCREW of pure ROTATION the problem has two solutions t1 and t1-2pi,  */
  /*  but the trivial second one is not considered, even though it might be a */
  /*  valid solutions in robotics and calculated as follows: */
  /*  switch sign(t11) */
  /*     case -1 */
  /*         t12 = 2*pi+t11; */
  /*     case 0 */
  /*         t12 = 2*pi; */
  /*     case 1 */
  /*         t12 = t11-2*pi; */
  /*     otherwise */
  /*         t12 = NaN; */
  /*     end */
  /*  IDEA: In fact it computes "Theta" to rotate the vector u' around x1 */
  /*  to make it parallel to the vector v', but "p" and "k" can be in */
  /*  different planes and even u' and v' be of different module, but o course */
  /*  both planes are parallel & perpendicular to the axis of x1, then �p� and */
  /*  �k� can not coincide after the movement, but the algorithm works. */
  /*  */
  /*  See also: PadenKahanTwo, PadenKahanThree */
  /*  See also: PardosOne, PardosTwo, PardosThree, PardosFour */
  /*  See also: PadenKahanPardosOne, PadenKahanPardosTwo, PadenKahanPardosThree */
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
  /*  Theta1 = PadenKahanOne(x1, pp, pk) */
  /*  */
  a = b_norm(*(double (*)[3])&x1[3]);
  a *= a;
  r1[0] = (x1[4] * x1[2] - x1[5] * x1[1]) / a;
  r1[1] = (x1[5] * x1[0] - x1[3] * x1[2]) / a;
  r1[2] = (x1[3] * x1[1] - x1[4] * x1[0]) / a;
  for (i = 0; i < 3; i++) {
    u[i] = pp[i] - r1[i];
    a = x1[3 + i];
    up_tmp[i] = a * x1[3];
    up_tmp[i + 3] = a * x1[4];
    up_tmp[i + 6] = a * x1[5];
  }

  for (i = 0; i < 3; i++) {
    up[i] = u[i] - ((up_tmp[i] * u[0] + up_tmp[i + 3] * u[1]) + up_tmp[i + 6] *
                    u[2]);
    r1[i] = pk[i] - r1[i];
  }

  for (i = 0; i < 3; i++) {
    u[i] = r1[i] - ((up_tmp[i] * r1[0] + up_tmp[i + 3] * r1[1]) + up_tmp[i + 6] *
                    r1[2]);
  }

  Theta1 = rt_atan2d_snf((x1[3] * (up[1] * u[2] - up[2] * u[1]) + x1[4] * (up[2]
    * u[0] - up[0] * u[2])) + x1[5] * (up[0] * u[1] - up[1] * u[0]), (up[0] * u
    [0] + up[1] * u[1]) + up[2] * u[2]);

  /*     */
  return Theta1;
}

/*
 * File trailer for PadenKahanOne.c
 *
 * [EOF]
 */
