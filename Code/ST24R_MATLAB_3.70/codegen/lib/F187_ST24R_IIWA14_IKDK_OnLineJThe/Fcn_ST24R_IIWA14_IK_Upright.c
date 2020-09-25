/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Fcn_ST24R_IIWA14_IK_Upright.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "Fcn_ST24R_IIWA14_IK_Upright.h"
#include "jointmag2limits.h"
#include "PadenKahanOne.h"
#include "mldivide.h"
#include "expScrew.h"
#include "PadenKahanTwo.h"
#include "rotm2axang.h"
#include "norm.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

/* Function Definitions */

/*
 * #codegen
 *
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *  Mechanical characteristics of the Robot (AT REF HOME POSITION):
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *  Joints TWISTS definition and TcP at home.
 * Arguments    : const double u[6]
 *                double ThetaSet[112]
 * Return Type  : void
 */
void Fcn_ST24R_IIWA14_IK_Upright(const double u[6], double ThetaSet[112])
{
  double ca;
  double sa;
  double cb;
  double sb;
  double cg;
  double sg;
  double dv6[16];
  double b_cb[16];
  int i4;
  int i5;
  int i6;
  double dv7[16];
  double noap[16];
  static const double dv8[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const double dv9[3] = { 1.0, 0.0, 0.0 };

  double b_u[3];
  double noapHst0if[4];
  static const double w1[3] = { 0.0, -1.0, 0.0 };

  double r1[3];
  double up[3];
  static const signed char up_tmp[9] = { 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const double pk[3] = { 0.0, 0.0, 0.36 };

  int i;
  int b_i;
  double x1[7];
  static const double b_x1[6] = { 0.78, 0.0, 0.0, 0.0, -1.0, 0.0 };

  static const double dv10[6] = { -0.36, 0.0, 0.0, 0.0, 1.0, 0.0 };

  double dv11[4];
  double t1t2[4];
  static const double b[4] = { 0.0, 0.0, 1.18, 1.0 };

  double dv12[16];
  double dv13[4];
  static const double dv14[6] = { -1.18, 0.0, 0.0, 0.0, 1.0, 0.0 };

  double dv15[16];
  static const double dv16[3] = { 0.0, 0.0, 1.38 };

  double dv17[16];
  double dv18[16];

  /*  Function "Fcn_ST24R_IIWA14_IK_Upright" IIWA14 INVERSE KINEMATICS */
  /*  KUKA IIWA 14 R820 - Robot HOME pose is Upright. */
  /*  */
  /*  It solves the INVERSE KINEMATICS for any desired Tool POSE  */
  /*  (i.e. position & orientation of the TCP system (noap goal) of the Robot. */
  /*  by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB. */
  /*  */
  /*  function ThetaSet = Fcn_ST24R_IIWA14_IK_Upright(u) */
  /*  */
  /*  The inputs "u" (7x1) are composed by the following vectors. */
  /*  "traXYZ" (3x1) desired translation for the TcP (noap - "p" goal). */
  /*  "rotXYZ" (3x1) desired rotation TcP system (noap - "noa" goal X+Y+Z) */
  /*  */
  /*  "ThetaSet" (16x7), with rows magnitudes solution for Robot Joints 1..7. */
  /*  it respects maximum Magnitude for the robot joints POSITION rad. */
  /*  Thmax = pi/180*[170 120 170 120 170 120 175]; */
  /*  */
  /*  Mechanical characteristics of the Robot (AT REF HOME POSITION): */
  /*  The S Spatial system has the "Z" axis up (i.e., -g direction). */
  /*  po = Origen for he STATIONARY system of reference. */
  /*  pk = point in the crossing of the DOF Th1(rot) & Th2(rot) & Th3(rot). */
  /*  pr = point in the axis of Th4(rot) Th5(rot). */
  /*  pf = point in the crossing of the DOF Th5(rot), Th6(rot), Th7(rot). */
  /*  pp = TcP Tool Center Point */
  /*  hst0 = Tool (TcP) POSE configuration (rot+tra) at reference position.  */
  /*  */
  /*  */
  /*  Copyright (C) 2003-2019, by Dr. Jose M. Pardos-Gotor. */
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
  /*  along with ST24R. If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www. */
  /*  */
  /*  CHANGES: */
  /*  Revision 1.1  2019/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  F183_ST24R_IIWA14_IK_OnLineJTra */
  /*  */
  /*  */
  /*  Motion RANGE for the robot joints POSITION rad, (by catalog). */
  /*  */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  INVERSE KINEMATICS */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Calculate Homogeneous transformation for the GOAL "noap" */
  /*  ROTX2TFORM Convert to Homogeneous matrix a rotation around the X axis  */
  /*    H = ROTX2TFORM(A) converts an angle rotation A around X axis into the */
  /*    corresponding homogeneous matrix, H. A is a rotation angles in radians. */
  /*  */
  /*    Example: */
  /*        %Calculate the homogeneous matrix for a rotation angle a = pi/2 */
  /*        around X axis. */
  /*        Hxa = rotX2tform(a) */
  /*        % Hxa = [1 0 0 0; 0 cos(a) -sin(a) 0; 0 sin(a) cos(a) 0; 0 0 0 1]  */
  /*        ans = */
  /*        1.0000         0         0         0 */
  /*             0    0.0000   -1.0000         0 */
  /*             0    1.0000    0.0000         0 */
  /*             0         0         0    1.0000 */
  /*  */
  /*  See also rotY2tform(b), rotZ2tform(g), */
  /*  */
  /*  Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor. */
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
  /*  Revision 1.1  2018/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  Hxa = rotX2tform(a) */
  ca = cos(u[3]);
  sa = sin(u[3]);

  /*  ROTY2TFORM Convert to Homogeneous matrix b rotation around the Y axis  */
  /*    H = ROTY2TFORM(B) converts an angle rotation B around Y axis into the */
  /*    corresponding homogeneous matrix, H. B is a rotation angle in radians. */
  /*  */
  /*    Example: */
  /*        %Calculate the homogeneous matrix for a rotation angle b = pi/2 */
  /*        around Y axis. */
  /*        Hyb = rotY2tform(b) */
  /*        % Hyb = [cos(b) 0 sin(b) 0; 0 1 0 0; -sin(b) 0 cos(b) 0; 0 0 0 1]  */
  /*        ans = */
  /*            0.0000         0    1.0000         0 */
  /*                 0    1.0000         0         0 */
  /*           -1.0000         0    0.0000         0 */
  /*                 0         0         0    1.0000 */
  /*  */
  /*  See also rotX2tform(b), rotZ2tform(g), */
  /*  */
  /*  Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor. */
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
  /*  Revision 1.1  2018/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  Hyb = rotY2tform(b) */
  cb = cos(u[4]);
  sb = sin(u[4]);

  /*  ROTZ2TFORM Convert to Homogeneous matrix a rotation around the Z axis  */
  /*    H = ROTZ2TFORM(G) converts an angle rotation G around Z axis into the */
  /*    corresponding homogeneous matrix, H. G is a rotation angle in radians. */
  /*  */
  /*    Example: */
  /*        %Calculate the homogeneous matrix for a rotation angle g = pi/2 */
  /*        around Z axis. */
  /*        Hzg = rotZ2tform(g) */
  /*        % Hzg = [cos(g) -sin(a) 0 0; sin(a) cos(a) 0 0; 0 0 1 0; 0 0 0 1]  */
  /*        ans = */
  /*        0.0000   -1.0000         0         0 */
  /*        1.0000    0.0000         0         0 */
  /*             0         0    1.0000         0 */
  /*             0         0         0    1.0000 */
  /*  */
  /*  See also rotX2tform(a), rotY2tform(b), */
  /*  */
  /*  Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor. */
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
  /*  Revision 1.1  2018/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  Hzg = rotZ2tform(g) */
  cg = cos(u[5]);
  sg = sin(u[5]);
  dv6[1] = 0.0;
  dv6[5] = ca;
  dv6[9] = -sa;
  dv6[13] = 0.0;
  dv6[2] = 0.0;
  dv6[6] = sa;
  dv6[10] = ca;
  dv6[14] = 0.0;
  b_cb[0] = cb;
  b_cb[4] = 0.0;
  b_cb[8] = sb;
  b_cb[12] = 0.0;
  b_cb[2] = -sb;
  b_cb[6] = 0.0;
  b_cb[10] = cb;
  b_cb[14] = 0.0;
  dv6[0] = 1.0;
  dv6[3] = 0.0;
  b_cb[1] = 0.0;
  b_cb[3] = 0.0;
  dv6[4] = 0.0;
  dv6[7] = 0.0;
  b_cb[5] = 1.0;
  b_cb[7] = 0.0;
  dv6[8] = 0.0;
  dv6[11] = 0.0;
  b_cb[9] = 0.0;
  b_cb[11] = 0.0;
  dv6[12] = 0.0;
  dv6[15] = 1.0;
  b_cb[13] = 0.0;
  b_cb[15] = 1.0;
  for (i4 = 0; i4 < 4; i4++) {
    for (i5 = 0; i5 < 4; i5++) {
      i6 = i5 << 2;
      dv7[i4 + i6] = ((dv6[i4] * b_cb[i6] + dv6[i4 + 4] * b_cb[1 + i6]) + dv6[i4
                      + 8] * b_cb[2 + i6]) + dv6[i4 + 12] * b_cb[3 + i6];
    }
  }

  b_cb[0] = cg;
  b_cb[4] = -sg;
  b_cb[8] = 0.0;
  b_cb[12] = 0.0;
  b_cb[1] = sg;
  b_cb[5] = cg;
  b_cb[9] = 0.0;
  b_cb[13] = 0.0;
  b_cb[2] = 0.0;
  b_cb[3] = 0.0;
  b_cb[6] = 0.0;
  b_cb[7] = 0.0;
  b_cb[10] = 1.0;
  b_cb[11] = 0.0;
  b_cb[14] = 0.0;
  b_cb[15] = 1.0;
  for (i4 = 0; i4 < 4; i4++) {
    for (i5 = 0; i5 < 4; i5++) {
      i6 = i5 << 2;
      noap[i4 + i6] = ((dv7[i4] * b_cb[i6] + dv7[i4 + 4] * b_cb[1 + i6]) +
                       dv7[i4 + 8] * b_cb[2 + i6]) + dv7[i4 + 12] * b_cb[3 + i6];
    }
  }

  noap[12] = u[0];
  noap[13] = u[1];
  noap[14] = u[2];

  /*  */
  /*  Calculate the IK joints solutions (Theta1 - Theta7) using SCREW THEORY */
  /*  and basically the PADEN-KAHAN Canonic Subproblems. The algorithm is  */
  /*  proposed by Dr. Pardos-Gotor and gives a set of 16 possible solutions. */
  memset(&ThetaSet[0], 0, 112U * sizeof(double));

  /*  */
  /*  STEP1: Calculate two "feasible and logical" inputs for t1 and t3. */
  /*  First, as this is REDUNDANT robot with 7DoF and theoretical infinite */
  /*  solutions, we reduce the number of possibilies to 16, and for doing so */
  /*  we give some solution INPUTS for t1 and t3 based on some other criteria, */
  /*  and then well get 8 solutions for each of these INPUTS for the rest 6DoF. */
  /*  For this function we choose t1 and t3 to be the magnitudes for orientate */
  /*  these joint towards the TcP, and for doing so we use PADEN-KAHAN-ONE. */
  ca = PadenKahanOne(dv8, dv9, *(double (*)[3])&u[0]);
  ca = jointmag2limits(ca, 2.9670597283903604, -2.9670597283903604);

  /*  */
  sa = PadenKahanOne(dv8, dv9, *(double (*)[3])&u[0]);
  sa = jointmag2limits(sa, 2.9670597283903604, -2.9670597283903604);
  for (i4 = 0; i4 < 8; i4++) {
    ThetaSet[32 + i4] = sa;
    ThetaSet[8 + i4] = ca;
  }

  /*  */
  /*  STEP2-1: Calculate Theta4. */
  /*  With "pf" on the axis of E5, E6, E7 and "pk" on the axis of E1, E2, E3. */
  /*  We apply (noap*hst0^-1) to "pf" and take the norm of the diffence of that */
  /*  resulting point and "pk". Doing so we can calculate Theta4 applying the */
  /*  Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not  */
  /*  affect "pf" and the E1,E2,E3 do not affect the norm of a vector with an */
  /*  end on "pk" resulting the problem  */
  /*  ||exp(E4^t4)*pf-pk|| = ||noap*hst0^-1*pf-pk|| = ||pk1p-pk|| = de */
  /*  which by PADEN-KAHAN-THREE has none, one or two solutions */
  /*  t401 & t402. */
  for (i4 = 0; i4 < 4; i4++) {
    noapHst0if[i4] = ((noap[i4] * 0.0 + noap[i4 + 4] * 0.0) + noap[i4 + 8] *
                      -0.19999999999999996) + noap[i4 + 12];
  }

  b_u[0] = noapHst0if[0];
  b_u[1] = noapHst0if[1];
  b_u[2] = noapHst0if[2] - 0.36;
  cb = b_norm(b_u);

  /*  */
  /*  "PADENKAHANTHREE" Find the Inverse Kinematics of a ROTATION SCREW */
  /*  which applied to a point move it to a certain */
  /*  distance from another point in space in SE(3). */
  /*  */
  /*  	Theta1 = PadenKahanThree(x1, pp, pk, de) */
  /*    by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB. */
  /*  */
  /*  Compute the magnitude Theta1 of the Screw with twist x1, to move the */
  /*  point "pp" from its position in 3D to another point "c" or "e" complying */
  /*  that the distance between "c" or "e" to the point "pk" is "de".  */
  /*   */
  /*          |v| */
  /*  E = x1 =| | (6x1); the points pp, pk, c and e are (3x1).  */
  /*          |w| */
  /*  exp(E^Theta) * pp = c; and de = norm(c-pk) and de = norm(e-pk). */
  /*  */
  /*  For a SCREW of pure ROTATION: */
  /*  Based on the Paden & Kahan subproblem Three for INVERSE KINEMATICS. */
  /*  The problem could has zero, one or two solutions, Theta1 = [t11; t12] */
  /*  Besides the problem could have another two solutions t11-2pi and t12-2pi,  */
  /*  but these trivial second ones are not considered, even though it might be */
  /*  valid and useful solutions for robotics. */
  /*  */
  /*  See also: PadenKahanOne, PadenKahanTwo */
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
  /*  Theta1 = PadenKahanThree(x1, pp, pk, de) */
  /*  */
  ca = b_norm(w1);
  ca *= ca;
  sa = -0.0 / ca;
  r1[0] = sa;
  b_u[0] = 0.0 - sa;
  sa = 0.0 / ca;
  r1[1] = sa;
  b_u[1] = 0.0 - sa;
  sa = 0.78 / ca;
  r1[2] = sa;
  b_u[2] = 1.18 - sa;
  for (i4 = 0; i4 < 3; i4++) {
    up[i4] = b_u[i4] - ((0.0 * b_u[0] + (double)up_tmp[i4 + 3] * b_u[1]) + 0.0 *
                        (1.18 - sa));
    r1[i4] = pk[i4] - r1[i4];
  }

  ca = b_norm(up);
  for (i4 = 0; i4 < 3; i4++) {
    b_u[i4] = r1[i4] - ((0.0 * r1[0] + (double)up_tmp[i4 + 3] * r1[1]) + 0.0 *
                        r1[2]);
  }

  sa = b_norm(b_u);
  sb = rt_atan2d_snf((0.0 * (up[1] * b_u[2] - up[2] * b_u[1]) + -(up[2] * b_u[0]
    - up[0] * b_u[2])) + 0.0 * (up[0] * b_u[1] - up[1] * b_u[0]), (up[0] * b_u[0]
    + up[1] * b_u[1]) + up[2] * b_u[2]);
  ca = ((ca * ca + sa * sa) - cb * cb) / (2.0 * ca * sa);

  /*  beta can be >1 <-1 if an error because there is NO SOLUTION. */
  if (ca > 1.0) {
    ca = 1.0;
  }

  if (ca < -1.0) {
    ca = -1.0;
  }

  ca = acos(ca);

  /*  */
  sa = jointmag2limits(sb - ca, 2.0943951023931953, -2.0943951023931953);
  ca = jointmag2limits(sb + ca, 2.0943951023931953, -2.0943951023931953);
  ThetaSet[48] = sa;
  ThetaSet[56] = sa;
  ThetaSet[52] = ca;
  ThetaSet[60] = ca;
  ThetaSet[49] = sa;
  ThetaSet[57] = sa;
  ThetaSet[53] = ca;
  ThetaSet[61] = ca;
  ThetaSet[50] = sa;
  ThetaSet[58] = sa;
  ThetaSet[54] = ca;
  ThetaSet[62] = ca;
  ThetaSet[51] = sa;
  ThetaSet[59] = sa;
  ThetaSet[55] = ca;
  ThetaSet[63] = ca;

  /*  */
  /*  STEP2-2: Calculate Theta1 & Theta2, knowing Theta3 (t3in). */
  /*  With "pf" on the axis of E5, E6, E7 we apply (noap*hsts0^-1) to "pf" and */
  /*  the POE E1..E7 also to "pf" having already known the value for t4 & t3 */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws */
  /*  E5,E6,E7 do not affect "pf" and the E4 & E3 are known,resulting */
  /*  the problem exp(E1^theta1)*exp(E2^theta2)*pf' = noap*hst0^-1*pf = pk1p */
  /*  which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions */
  /*  t401 & t3in => t201-t101 & t202-t102. */
  /*  t402 & t3in => t203-t103 & t204-t104.  */
  for (i = 0; i < 2; i++) {
    b_i = i << 2;
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[32 + b_i];
    expScrew(x1, dv6);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = b_x1[i4];
    }

    x1[6] = ThetaSet[48 + b_i];
    expScrew(x1, dv7);
    for (i4 = 0; i4 < 4; i4++) {
      dv11[i4] = 0.0;
      for (i5 = 0; i5 < 4; i5++) {
        i6 = i5 << 2;
        dv11[i4] += (((dv6[i4] * dv7[i6] + dv6[i4 + 4] * dv7[1 + i6]) + dv6[i4 +
                      8] * dv7[2 + i6]) + dv6[i4 + 12] * dv7[3 + i6]) * b[i5];
      }
    }

    PadenKahanTwo(dv8, dv10, *(double (*)[3])&dv11[0], *(double (*)[3])&
                  noapHst0if[0], t1t2);
    t1t2[0] = jointmag2limits(t1t2[0], 2.9670597283903604, -2.9670597283903604);
    t1t2[1] = jointmag2limits(t1t2[1], 2.9670597283903604, -2.9670597283903604);
    t1t2[2] = jointmag2limits(t1t2[2], 2.0943951023931953, -2.0943951023931953);
    t1t2[3] = jointmag2limits(t1t2[3], 2.0943951023931953, -2.0943951023931953);
    ThetaSet[b_i] = t1t2[0];
    ThetaSet[b_i + 16] = t1t2[2];
    ThetaSet[b_i + 1] = t1t2[0];
    ThetaSet[b_i + 17] = t1t2[2];
    ThetaSet[b_i + 2] = t1t2[1];
    ThetaSet[b_i + 18] = t1t2[3];
    ThetaSet[b_i + 3] = t1t2[1];
    ThetaSet[b_i + 19] = t1t2[3];
  }

  /*  */
  /*  STEP2-3: Calculate Theta2 & Theta3, knowing Theta1 (t1in). */
  /*  With "pf" on the axis of E5,E6,E7 we apply E1^-1*(noap*hsts0^-1) to "pf" */
  /*  and POE E2..E7 also to "pf" having already known the value for t4 & t1 */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws */
  /*  E5,E6,E7 do not affect "pf" and the E4 is known,resulting */
  /*  the problem exp(E2^t2)*exp(E3^t3)*pf'' = E1^-1*noap*hst0^-1*pf = pk2p */
  /*  which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions */
  /*  t401 & t1in => t301-t205 & t302-t206. */
  /*  t402 & t1in => t303-t207 & t304-t208. */
  /*  */
  for (i = 0; i < 2; i++) {
    b_i = 8 + (i << 2);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = b_x1[i4];
    }

    x1[6] = ThetaSet[48 + b_i];
    expScrew(x1, dv6);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[b_i];
    dv11[0] = noapHst0if[0];
    dv11[1] = noapHst0if[1];
    dv11[2] = noapHst0if[2];
    dv11[3] = noapHst0if[3];
    expScrew(x1, dv12);
    mldivide(dv12, dv11);
    for (i4 = 0; i4 < 4; i4++) {
      dv13[i4] = ((dv6[i4] * 0.0 + dv6[i4 + 4] * 0.0) + dv6[i4 + 8] * 1.18) +
        dv6[i4 + 12];
    }

    PadenKahanTwo(dv10, dv8, *(double (*)[3])&dv13[0], *(double (*)[3])&dv11[0],
                  t1t2);
    t1t2[0] = jointmag2limits(t1t2[0], 2.0943951023931953, -2.0943951023931953);
    t1t2[1] = jointmag2limits(t1t2[1], 2.0943951023931953, -2.0943951023931953);
    t1t2[2] = jointmag2limits(t1t2[2], 2.9670597283903604, -2.9670597283903604);
    t1t2[3] = jointmag2limits(t1t2[3], 2.9670597283903604, -2.9670597283903604);
    ThetaSet[b_i + 16] = t1t2[0];
    ThetaSet[b_i + 32] = t1t2[2];
    ThetaSet[b_i + 17] = t1t2[0];
    ThetaSet[b_i + 33] = t1t2[2];
    ThetaSet[b_i + 18] = t1t2[1];
    ThetaSet[b_i + 34] = t1t2[3];
    ThetaSet[b_i + 19] = t1t2[1];
    ThetaSet[b_i + 35] = t1t2[3];
  }

  /*  */
  /*  STEP2-4: Calculate Theta5 & Theta6. */
  /*  With "pp" on the axis of E7 apply E4^-1*E3^-1**E2^-1*E1^-1*noap*hst0^-1 */
  /*  to "pp" and also to the POE E5*E6*E7 knowing already t4,t3,t2,t1. */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws */
  /*  E7 does not affect "pp" and the result is the problem */
  /*  exp(E5^theta5)*exp(E6^theta6)*pp = pk3p ; with */
  /*  pk3p = exp(E4^t4)^-1*exp(E3^t3)^-1*exp(E2^t2)^-1*exp(E1^t1)^-1*noap*hst0^-1*pp  */
  /*  which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions: */
  /*  t101-t201-t3in-t401 => t501-t601 & t502-t602 */
  /*  t102-t202-t3in-t401 => t503-t603 & t504-t604 */
  /*  t103-t203-t3in-t402 => t505-t605 & t506-t606 */
  /*  t104-t204-t3in-t402 => t507-t607 & t508-t608 */
  /*  t1in-t205-t301-t401 => t509-t609 & t510-t610 */
  /*  t1in-t206-t302-t401 => t511-t611 & t512-t612 */
  /*  t1in-t207-t303-t402 => t513-t613 & t514-t614 */
  /*  t1in-t208-t304-t402 => t515-t615 & t516-t616 */
  /*  */
  for (i4 = 0; i4 < 4; i4++) {
    t1t2[i4] = ((noap[i4] * 0.0 + noap[i4 + 4] * 0.0) + noap[i4 + 8] * 0.0) +
      noap[i4 + 12];
  }

  for (i = 0; i < 8; i++) {
    b_i = i << 1;
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = b_x1[i4];
    }

    x1[6] = ThetaSet[48 + b_i];
    expScrew(x1, dv6);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[32 + b_i];
    expScrew(x1, dv7);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv10[i4];
    }

    x1[6] = ThetaSet[16 + b_i];
    expScrew(x1, b_cb);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[b_i];
    dv11[0] = t1t2[0];
    dv11[1] = t1t2[1];
    dv11[2] = t1t2[2];
    dv11[3] = t1t2[3];
    expScrew(x1, dv15);
    mldivide(dv15, dv11);
    mldivide(b_cb, dv11);
    mldivide(dv7, dv11);
    mldivide(dv6, dv11);
    PadenKahanTwo(dv8, dv14, dv16, *(double (*)[3])&dv11[0], dv13);
    ThetaSet[b_i + 64] = jointmag2limits(dv13[0], 2.9670597283903604,
      -2.9670597283903604);
    ThetaSet[b_i + 65] = jointmag2limits(dv13[1], 2.9670597283903604,
      -2.9670597283903604);
    ThetaSet[b_i + 80] = jointmag2limits(dv13[2], 2.0943951023931953,
      -2.0943951023931953);
    ThetaSet[b_i + 81] = jointmag2limits(dv13[3], 2.0943951023931953,
      -2.0943951023931953);
  }

  /*  */
  /*  STEP2-5: Calculate Theta7. */
  /*  With "po" not in the axis of E7 apply E6^-1...*E1^-1*noap*gs0^-1 to "po" */
  /*  and applying E7 to "po" knowing already Theta6...Theta1, */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem: */
  /*  exp(E7^theta7)*po = pk4p ; with */
  /*  pk4p = exp(E6^Th6)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po  */
  /*  which by PADEN-KAHAN-ONE has none or one solution. Then for all */
  /*  Th6-Th5-Th4-Th3-Th2-Th1 we get a Th7 = t701...t716: */
  /*  */
  for (i4 = 0; i4 < 4; i4++) {
    t1t2[i4] = ((noap[i4] + noap[i4 + 4] * 0.0) + noap[i4 + 8] * -1.38) +
      noap[i4 + 12];
  }

  for (i = 0; i < 16; i++) {
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv14[i4];
    }

    x1[6] = ThetaSet[80 + i];
    expScrew(x1, dv6);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[64 + i];
    expScrew(x1, dv7);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = b_x1[i4];
    }

    x1[6] = ThetaSet[48 + i];
    expScrew(x1, b_cb);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[32 + i];
    expScrew(x1, dv17);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv10[i4];
    }

    x1[6] = ThetaSet[16 + i];
    expScrew(x1, noap);
    for (i4 = 0; i4 < 6; i4++) {
      x1[i4] = dv8[i4];
    }

    x1[6] = ThetaSet[i];
    noapHst0if[0] = t1t2[0];
    noapHst0if[1] = t1t2[1];
    noapHst0if[2] = t1t2[2];
    noapHst0if[3] = t1t2[3];
    expScrew(x1, dv18);
    mldivide(dv18, noapHst0if);
    mldivide(noap, noapHst0if);
    mldivide(dv17, noapHst0if);
    mldivide(b_cb, noapHst0if);
    mldivide(dv7, noapHst0if);
    mldivide(dv6, noapHst0if);
    ca = PadenKahanOne(dv8, dv9, *(double (*)[3])&noapHst0if[0]);
    ThetaSet[96 + i] = jointmag2limits(ca, 3.0543261909900767,
      -3.0543261909900767);
  }

  /*  */
  /*  */
}

/*
 * File trailer for Fcn_ST24R_IIWA14_IK_Upright.c
 *
 * [EOF]
 */
