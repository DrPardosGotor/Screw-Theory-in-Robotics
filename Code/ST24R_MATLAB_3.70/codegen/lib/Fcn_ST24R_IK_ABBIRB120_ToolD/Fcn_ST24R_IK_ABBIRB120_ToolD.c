/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Fcn_ST24R_IK_ABBIRB120_ToolD.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 17-May-2020 16:49:25
 */

/* Include Files */
#include "Fcn_ST24R_IK_ABBIRB120_ToolD.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD_data.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD_initialize.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD_rtwutil.h"
#include "PadenKahanTwo.h"
#include "expScrew.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */

/*
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *  Mechanical characteristics of the Robot (AT REF HOME POSITION):
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *  Joints TWISTS definition and TcP at home.
 * Arguments    : const double u[6]
 *                double ThetaSET[48]
 * Return Type  : void
 */
void Fcn_ST24R_IK_ABBIRB120_ToolD(const double u[6], double ThetaSET[48])
{
  double ca;
  double sa;
  double cb;
  double sb;
  double cg;
  double sg;
  double dv[16];
  double b_cb[16];
  int i;
  double d;
  int k;
  int b_i;
  double dv1[16];
  double noap[16];
  double noapHst0if[4];
  double r1[3];
  double b_u[3];
  double up[3];
  static const signed char up_tmp[9] = { 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const double pk[3] = { 0.0, 0.0, 0.29 };

  double x1[7];
  static const double b_x1[6] = { -0.56, 0.0, 0.0, 0.0, 1.0, 0.0 };

  static const double dv2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const double dv3[6] = { -0.29, 0.0, 0.0, 0.0, 1.0, 0.0 };

  double dv4[4];
  double t1t2[4];
  double dv5[16];
  static const double dv6[6] = { 0.0, 0.63, 0.0, 1.0, 0.0, 0.0 };

  static const double dv7[6] = { -0.63, 0.0, 0.302, 0.0, 1.0, 0.0 };

  static const double dv8[3] = { 0.302, 0.0, 0.47 };

  static const signed char b_up_tmp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  if (!isInitialized_Fcn_ST24R_IK_ABBIRB120_ToolD) {
    Fcn_ST24R_IK_ABBIRB120_ToolD_initialize();
  }

  /*  "Fcn_ST24R_IK_ABBIRB120_ToolD" Inverse Kinematics IRB120 ToolDown. */
  /*  */
  /*  Solves rhe INVERSE KINEMATICS for any desired position & orientation */
  /*  of the TCP (noap goal) of the ABB IRB120 Robot. */
  /*  */
  /*  function ThetaSet = Fcn_ST24R_IK_ABBIRB120_ToolD(u) */
  /*  */
  /*  The inputs "u" (7x1) are composed by the following vectors. */
  /*  "traXYZ" (3x1) desired translation for the TcP (noap - "p" goal). */
  /*  "rotXYZ" (3x1) desired rotation TcP system (noap - "noa" goal X+Y+Z) */
  /*  */
  /*  "ThetaSet" (8x6), with rows magnitudes solution for the Robot Joints1..6. */
  /*  it respects maximum Magnitude for the robot joints POSITION rad. */
  /*  Thmax = pi/180*[170 120 170 120 170 120 175]; */
  /*  */
  /*  Mechanical characteristics of the Robot (AT REF POSITION), which is the */
  /*  elbow pose with the TOOL facing DOWN: */
  /*  po = Origen for he STATIONARY system of reference. */
  /*  pk = point in the crossing of the DOF Th1(rot) & Th2(rot). */
  /*  pr = point in the axis of Th3(rot). */
  /*  pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot). */
  /*  pp = TcP Tool Center Point */
  /*  hst0 = Tool (TcP) configuration (rot+tra) at robot reference position. */
  /*  */
  /*  */
  /*  Copyright (C) 2003-2020, by Dr. Jose M. Pardos-Gotor. */
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
  /*  Fcn_ST24R_IK_ABBIRB120_ToolD */
  /*  */
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
  dv[1] = 0.0;
  dv[5] = ca;
  dv[9] = -sa;
  dv[13] = 0.0;
  dv[2] = 0.0;
  dv[6] = sa;
  dv[10] = ca;
  dv[14] = 0.0;
  b_cb[0] = cb;
  b_cb[4] = 0.0;
  b_cb[8] = sb;
  b_cb[12] = 0.0;
  b_cb[2] = -sb;
  b_cb[6] = 0.0;
  b_cb[10] = cb;
  b_cb[14] = 0.0;
  dv[0] = 1.0;
  dv[3] = 0.0;
  b_cb[1] = 0.0;
  b_cb[3] = 0.0;
  dv[4] = 0.0;
  dv[7] = 0.0;
  b_cb[5] = 1.0;
  b_cb[7] = 0.0;
  dv[8] = 0.0;
  dv[11] = 0.0;
  b_cb[9] = 0.0;
  b_cb[11] = 0.0;
  dv[12] = 0.0;
  dv[15] = 1.0;
  b_cb[13] = 0.0;
  b_cb[15] = 1.0;
  for (i = 0; i < 4; i++) {
    d = dv[i + 4];
    ca = dv[i + 8];
    sa = dv[i + 12];
    for (k = 0; k < 4; k++) {
      b_i = k << 2;
      dv1[i + b_i] = ((dv[i] * b_cb[b_i] + d * b_cb[b_i + 1]) + ca * b_cb[b_i +
                      2]) + sa * b_cb[b_i + 3];
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
  for (i = 0; i < 4; i++) {
    d = dv1[i + 4];
    ca = dv1[i + 8];
    sa = dv1[i + 12];
    for (k = 0; k < 4; k++) {
      b_i = k << 2;
      noap[i + b_i] = ((dv1[i] * b_cb[b_i] + d * b_cb[b_i + 1]) + ca * b_cb[b_i
                       + 2]) + sa * b_cb[b_i + 3];
    }
  }

  noap[12] = u[0];
  noap[13] = u[1];
  noap[14] = u[2];

  /*  */
  /*  Calculate the IK joints solutions (Theta1 - Theta6) using SCREW THEORY */
  /*  and basically the PADEN-KAHAN Canonic Subproblems. */
  memset(&ThetaSET[0], 0, 48U * sizeof(double));

  /*  */
  /*  STEP1: Calculate Theta3. */
  /*  With "pf" on the axis of E4, E5, E6 and "pk" on the axis of E1, E2. */
  /*  We apply (noap*gs0^-1) to "pf" and take the norm of the diffence of that */
  /*  resulting point and "pk". Doing so we can calculate Theta3 applying the */
  /*  Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not affect */
  /*  "pf" and the E1,E2 do not affect the norm of a vector with an end on "pk" */
  /*  resulting the problem ||exp(E3^theta3)*pf-pk||=||noap*gs0^-1*pf-pk|| */
  /*  which by PADEN-KAHAN-THREE has none, one or two solutions for t31 t32. */
  for (i = 0; i < 4; i++) {
    noapHst0if[i] = ((noap[i] * 0.0 + noap[i + 4] * 0.0) + noap[i + 8] *
                     -0.16000000000000003) + noap[i + 12];
  }

  cb = 3.3121686421112381E-170;

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
  ca = fabs(noapHst0if[0]);
  if (ca > 3.3121686421112381E-170) {
    sb = 1.0;
    cb = ca;
  } else {
    sa = ca / 3.3121686421112381E-170;
    sb = sa * sa;
  }

  ca = fabs(noapHst0if[1]);
  if (ca > cb) {
    sa = cb / ca;
    sb = sb * sa * sa + 1.0;
    cb = ca;
  } else {
    sa = ca / cb;
    sb += sa * sa;
  }

  ca = fabs(noapHst0if[2] - 0.29);
  if (ca > cb) {
    sa = cb / ca;
    sb = sb * sa * sa + 1.0;
    cb = ca;
  } else {
    sa = ca / cb;
    sb += sa * sa;
  }

  sb = cb * sqrt(sb);
  r1[0] = 0.0;
  b_u[0] = 0.302;
  r1[1] = -0.0;
  b_u[1] = 0.0;
  r1[2] = 0.56;
  b_u[2] = 0.069999999999999951;
  cg = 0.0;
  cb = 3.3121686421112381E-170;
  for (k = 0; k < 3; k++) {
    up[k] = b_u[k];
    if (b_u[k] > cb) {
      sa = cb / b_u[k];
      cg = cg * sa * sa + 1.0;
      cb = b_u[k];
    } else {
      sa = b_u[k] / cb;
      cg += sa * sa;
    }

    r1[k] = pk[k] - r1[k];
  }

  cg = cb * sqrt(cg);
  sg = 0.0;
  cb = 3.3121686421112381E-170;
  for (k = 0; k < 3; k++) {
    d = r1[k] - ((0.0 * r1[0] + (double)up_tmp[k + 3] * r1[1]) + 0.0 * r1[2]);
    b_u[k] = d;
    ca = fabs(d);
    if (ca > cb) {
      sa = cb / ca;
      sg = sg * sa * sa + 1.0;
      cb = ca;
    } else {
      sa = ca / cb;
      sg += sa * sa;
    }
  }

  sg = cb * sqrt(sg);
  cb = rt_atan2d_snf((0.0 * (up[1] * b_u[2] - up[2] * b_u[1]) + (up[2] * b_u[0]
    - up[0] * b_u[2])) + 0.0 * (up[0] * b_u[1] - up[1] * b_u[0]), (up[0] * b_u[0]
    + up[1] * b_u[1]) + up[2] * b_u[2]);
  ca = ((cg * cg + sg * sg) - sb * sb) / (2.0 * cg * sg);

  /*  beta can be >1 <-1 if an error because there is NO SOLUTION. */
  if (ca > 1.0) {
    ca = 1.0;
  }

  if (ca < -1.0) {
    ca = -1.0;
  }

  ca = acos(ca);
  sa = cb - ca;
  ca += cb;

  /*  */
  /* t3(1) = jointmag2limits(t3(1), Thmax(3), Thmin(3), "rot"); */
  /* t3(2) = jointmag2limits(t3(2), Thmax(3), Thmin(3), "rot"); */
  ThetaSET[16] = sa;
  ThetaSET[20] = ca;
  ThetaSET[17] = sa;
  ThetaSET[21] = ca;
  ThetaSET[18] = sa;
  ThetaSET[22] = ca;
  ThetaSET[19] = sa;
  ThetaSET[23] = ca;

  /*  put results into Theta */
  /*  */
  /*  STEP2: Calculate Theta1 & Theta2. */
  /*  With "pf" on the axis of E4, E5, E6 we apply (noap*gs0^-1) to "pf" and */
  /*  the POE E1..E6 also to "pf" having already known the value for Theta3 */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws */
  /*  E4,E5,E6 do not affect "pf" and the E3 is known (two values),resulting */
  /*  the problem exp(E1^theta1)*exp(E2^theta2)*pf' = noap*gs0^-1*pf */
  /*  which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions */
  /*  t11-t21 & t12-t22 for each value of t3, but we have two, then consider */
  /*  for t31 we get t11-t21 & t12-t22 & for t32 we get t13-t23 & t14-t24. */
  for (k = 0; k < 2; k++) {
    b_i = k << 2;
    for (i = 0; i < 6; i++) {
      x1[i] = b_x1[i];
    }

    x1[6] = ThetaSET[b_i + 16];
    expScrew(x1, dv);
    for (i = 0; i < 4; i++) {
      dv4[i] = ((dv[i] * 0.302 + dv[i + 4] * 0.0) + dv[i + 8] * 0.63) + dv[i +
        12];
    }

    PadenKahanTwo(dv2, dv3, *(double (*)[3])&dv4[0], *(double (*)[3])&
                  noapHst0if[0], t1t2);

    /*     t1t2(1,1) = jointmag2limits(t1t2(1,1), Thmax(1), Thmin(1), "rot"); */
    /*     t1t2(2,1) = jointmag2limits(t1t2(2,1), Thmax(1), Thmin(1), "rot"); */
    /*     t1t2(1,2) = jointmag2limits(t1t2(1,2), Thmax(2), Thmin(2), "rot"); */
    /*     t1t2(2,2) = jointmag2limits(t1t2(2,2), Thmax(2), Thmin(2), "rot"); */
    ThetaSET[b_i] = t1t2[0];
    ThetaSET[b_i + 1] = t1t2[0];
    ThetaSET[b_i + 2] = t1t2[1];
    ThetaSET[b_i + 3] = t1t2[1];
    ThetaSET[b_i + 8] = t1t2[2];
    ThetaSET[b_i + 9] = t1t2[2];
    ThetaSET[b_i + 10] = t1t2[3];
    ThetaSET[b_i + 11] = t1t2[3];
  }

  /*  */
  /*  STEP3: Calculate Theta4 & Theta5. */
  /*  With "pp" on the axis of E6 apply E3^-1*E2^-1*E1^-1*noap*gs0^-1 to "pp" */
  /*  and also the POE E4*E5*E6 to "pp" knowing already Theta3-Theta2-Theta1, */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws */
  /*  E6 does not affect "pp" & Th3-Th2-Th1 known (four solutions), the problem */
  /*  exp(E4^theta4)*exp(E5^theta5)*pp = pk2p ; with */
  /*  pk2p = exp(E3^Th3)^-1*exp(E2^Th2)^-1*exp(E1^Th1)^-1*noap*gs0^-1*pp  */
  /*  which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions: */
  /*  t31,t21,t11 to t41-t51 & t42-t52 ; t31,t22,t12 to t43-t53 & t44-t54 */
  /*  t32,t23,t13 to t45-t55 & t46-t56 ; t32,t24,t14 to t47-t57 & t48-t58 */
  /*  */
  for (i = 0; i < 4; i++) {
    noapHst0if[i] = ((noap[i] * 0.0 + noap[i + 4] * 0.0) + noap[i + 8] * 0.0) +
      noap[i + 12];
  }

  for (k = 0; k < 4; k++) {
    b_i = k << 1;

    /*  for the 4 values of t3-t2-t1. */
    for (i = 0; i < 6; i++) {
      x1[i] = b_x1[i];
    }

    x1[6] = ThetaSET[b_i + 16];
    expScrew(x1, dv);
    for (i = 0; i < 6; i++) {
      x1[i] = dv3[i];
    }

    x1[6] = ThetaSET[b_i + 8];
    expScrew(x1, dv1);
    for (i = 0; i < 6; i++) {
      x1[i] = dv2[i];
    }

    x1[6] = ThetaSET[b_i];
    t1t2[0] = noapHst0if[0];
    t1t2[1] = noapHst0if[1];
    t1t2[2] = noapHst0if[2];
    t1t2[3] = noapHst0if[3];
    expScrew(x1, dv5);
    mldivide(dv5, t1t2);
    mldivide(dv1, t1t2);
    mldivide(dv, t1t2);
    PadenKahanTwo(dv6, dv7, dv8, *(double (*)[3])&t1t2[0], dv4);
    ThetaSET[b_i + 24] = dv4[0];
    ThetaSET[b_i + 25] = dv4[1];
    ThetaSET[b_i + 32] = dv4[2];
    ThetaSET[b_i + 33] = dv4[3];

    /*     t4t5(1,1) = jointmag2limits(t4t5(1,1), Thmax(4), Thmin(4), "rot"); */
    /*     t4t5(2,1) = jointmag2limits(t4t5(2,1), Thmax(4), Thmin(4), "rot"); */
    /*     t4t5(1,2) = jointmag2limits(t4t5(1,2), Thmax(5), Thmin(5), "rot"); */
    /*     t4t5(2,2) = jointmag2limits(t4t5(2,2), Thmax(5), Thmin(5), "rot"); */
  }

  /*  */
  /*  STEP4: Calculate Theta6. */
  /*  With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po" */
  /*  and applying E6 to "po" knowing already Theta5...Theta1 (8 solutions), */
  /*  resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem: */
  /*  exp(E6^theta6)*po = pk3p ; with */
  /*  pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po  */
  /*  which by PADEN-KAHAN-ONE has none or one solution. Then for all */
  /*  Th5-Th4-Th3-Th2-Th1 known (eight solutions) we get t61...t68: */
  for (i = 0; i < 4; i++) {
    noapHst0if[i] = ((noap[i] * 0.302 + noap[i + 4] * 0.0) + noap[i + 8] * 0.47)
      + noap[i + 12];
  }

  for (k = 0; k < 8; k++) {
    /*  */
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
    b_u[0] = -0.302;
    b_u[1] = 0.0;
    b_u[2] = 0.0;
    for (i = 0; i < 3; i++) {
      up[i] = b_u[i];
    }

    for (i = 0; i < 6; i++) {
      x1[i] = dv7[i];
    }

    x1[6] = ThetaSET[k + 32];
    expScrew(x1, dv);
    for (i = 0; i < 6; i++) {
      x1[i] = dv6[i];
    }

    x1[6] = ThetaSET[k + 24];
    expScrew(x1, dv1);
    for (i = 0; i < 6; i++) {
      x1[i] = b_x1[i];
    }

    x1[6] = ThetaSET[k + 16];
    expScrew(x1, b_cb);
    for (i = 0; i < 6; i++) {
      x1[i] = dv3[i];
    }

    x1[6] = ThetaSET[k + 8];
    expScrew(x1, noap);
    for (i = 0; i < 6; i++) {
      x1[i] = dv2[i];
    }

    x1[6] = ThetaSET[k];
    t1t2[0] = noapHst0if[0];
    t1t2[1] = noapHst0if[1];
    t1t2[2] = noapHst0if[2];
    t1t2[3] = noapHst0if[3];
    expScrew(x1, dv5);
    mldivide(dv5, t1t2);
    mldivide(noap, t1t2);
    mldivide(b_cb, t1t2);
    mldivide(dv1, t1t2);
    mldivide(dv, t1t2);
    r1[0] = t1t2[0] - 0.302;
    r1[1] = t1t2[1];
    r1[2] = t1t2[2];
    for (i = 0; i < 3; i++) {
      b_u[i] = r1[i] - ((0.0 * (t1t2[0] - 0.302) + 0.0 * t1t2[1]) + (double)
                        b_up_tmp[i + 6] * r1[2]);
    }

    ThetaSET[k + 40] = rt_atan2d_snf((0.0 * (up[1] * b_u[2] - up[2] * b_u[1]) +
      0.0 * (up[2] * b_u[0] - up[0] * b_u[2])) + -(up[0] * b_u[1] - up[1] * b_u
      [0]), (up[0] * b_u[0] + up[1] * b_u[1]) + up[2] * b_u[2]);

    /*     */
    /*     t6 = jointmag2limits(t6, Thmax(6), Thmin(6), "rot"); */
  }

  /*  */
  /*  */
}

/*
 * File trailer for Fcn_ST24R_IK_ABBIRB120_ToolD.c
 *
 * [EOF]
 */
