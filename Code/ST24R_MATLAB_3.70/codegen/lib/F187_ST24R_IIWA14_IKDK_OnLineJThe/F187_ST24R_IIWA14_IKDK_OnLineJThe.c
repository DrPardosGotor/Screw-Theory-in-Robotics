/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: F187_ST24R_IIWA14_IKDK_OnLineJThe.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "norm.h"
#include "rotm2axang.h"
#include "eul2rotm.h"
#include "rotm2eul.h"
#include "expScrew.h"
#include "jointmag2limits.h"
#include "Fcn_ST24R_IIWA14_IK_Upright.h"
#include "xgeqp3.h"

/* Variable Definitions */
static double ThetaVAL[7];
static double ThetapVAL[7];

/* Function Definitions */

/*
 * #codegen
 * Arguments    : const double u[7]
 *                double ThetaOut[21]
 * Return Type  : void
 */
void F187_ST24R_IIWA14_IKDK_OnLineJThe(const double u[7], double ThetaOut[21])
{
  int i0;
  static const double dv0[7] = { -1.05746, 0.138873, -2.06204, 1.28518,
    -0.610222, -0.894605, 0.886743 };

  int PoE_tmp;
  double TwMag[49];
  static const double dv1[42] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.36, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.78, 0.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.18, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0 };

  double PoE[16];
  double HstR[16];
  int i;
  double HstThe[16];
  double b_HstR[16];
  double b_HstThe[9];
  double dv2[3];
  int HstThe_tmp;
  static const double b[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.38, 1.0 };

  double TargetVAL[6];
  double vtcpS_tmp[3];
  double a_tmp[9];
  double b_tmp[9];
  double b_a_tmp[9];
  double wstSaxang[4];
  double b_vtcpS_tmp[6];
  double VtS[6];
  double JstS[42];
  double VstS[6];
  int jpvt[7];
  int rankR;
  double tol;
  int j;
  double Y[7];
  double Thetapp[7];
  double ThetaSet[112];
  double ThetaVALnew[7];
  static const double dv3[7] = { 2.9670597283903604, 2.0943951023931953,
    2.9670597283903604, 2.0943951023931953, 2.9670597283903604,
    2.0943951023931953, 3.0543261909900767 };

  static const double dv4[7] = { -2.9670597283903604, -2.0943951023931953,
    -2.9670597283903604, -2.0943951023931953, -2.9670597283903604,
    -2.0943951023931953, -3.0543261909900767 };

  double b_PoE[36];
  double dv5[9];
  double b_wstSaxang[4];
  static const signed char iv0[6] = { 0, 1, 2, 3, 4, 5 };

  static const signed char iv1[3] = { 0, 1, 2 };

  /*  Function "F187_ST24R_IIWA14_IKDK_OnLineJThe" */
  /*  KUKA IIWA 14 R820 - Robot HOME Straight Up with complete dynamics. */
  /*  Function solves DIFFERENTIAL KINEMATICS in REAL TIME for robot joints. */
  /*  It applies the EULER EXPLICIT method for integration. */
  /*  */
  /*  It solves the DIFFERENTIAL KINEMATICS for desired position & orientation */
  /*  velocities of the TCP (noap goal) of the Robot. */
  /*  Actually, the aim is to calculate de Joints position without solving */
  /*  the Inverse Kinematics, but using the DIFFERENTIAL INVERSE KINEMATICS */
  /*  from the GEOMETRIC JACOBIAN (inverse) and the Tool Velocities. */
  /*  by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB. */
  /*  */
  /*  function ThetaOut = F187_ST24R_IIWA14_IKDK_OnLineJThe(u) */
  /*  */
  /*  The INPUTS "u" (7x1) are composed by the following vectors. */
  /*  Target Value composed by "traXYZ" and "rotXYZ" as */
  /*  "traXYZ" (3x1) translations for the TcP (noap - "p" goal) in S frame. */
  /*  "rotXYZ" (3x1) rotations for Tool (noap - "noa" (X+Y+Z)) in S frame. */
  /*  "Solutions" (1x1) is the value "Theta index" for choosing one out of 16 */
  /*  Solutions = 1 to 16, for Robot Joint possible different solutions */
  /*  Solutions = 17 is for sending the robot to HOME POSITION (ThetaOut=17) */
  /*  Solutions = 18 is for sending the robot to the FREEZE POSITION. */
  /*  Solutions = 19 is for using Differential Kinematics for Theta AUTO, which */
  /*  means to calculate the INVERSE DIFFERENTIAL KINEMATICS. */
  /*  */
  /*  OUTPUTS (1,21): */
  /*  "ThetaOut" (1,7) POSITION magnitudes solution for Joints1..7. */
  /*  ThetaOut respects maximum Magnitude for the robot joints POSITION rad. */
  /*  Thmax = pi/180*[170 120 170 120 170 120 175]; */
  /*  "ThetaOut" (8,14) VELOCITIES magnitudes solution for Joints1..7. */
  /*  "ThetaOut" (15,21) ACCELERATION magnitudes solution for Joints1..7. */
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
  /*  F187_ST24R_IIWA14_IKDK_OnLineJThe */
  /*  */
  /*  */
  if (u[6] == 17.0) {
    memset(&ThetaOut[0], 0, 21U * sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      ThetaVAL[i0] = 0.0;
    }
  } else {
    /*  */
    if (u[6] == 18.0) {
      for (i0 = 0; i0 < 7; i0++) {
        ThetaOut[i0] = ThetaVAL[i0];
      }

      memset(&ThetaOut[7], 0, 14U * sizeof(double));
    } else if (u[6] == 19.0) {
      for (i0 = 0; i0 < 7; i0++) {
        ThetaVAL[i0] = dv0[i0];
        ThetaOut[i0] = dv0[i0];
      }

      memset(&ThetaOut[7], 0, 14U * sizeof(double));
    } else {
      /*  */
      /*  */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /*  Mechanical characteristics of the Robot (AT REF HOME POSITION): */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /*  Joints TWISTS definition and TcP at home. */
      /*  Motion RANGE for the robot joints POSITION rad, (by catalog). */
      /*  Maximum SPEED for the robot joints rad/sec, (by catalog). */
      /* Thpmax = pi/180*[85 85 100 75 130 135 135]; */
      /*  */
      for (i0 = 0; i0 < 7; i0++) {
        for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
          TwMag[PoE_tmp + 7 * i0] = dv1[PoE_tmp + 6 * i0];
        }

        TwMag[6 + 7 * i0] = ThetaVAL[i0];
      }

      /*  */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /*  INVERSE KINEMATICS & DIFFERENTIAL KINEMATICS */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /*  Target VALUE is the current cartesian trajectory point. */
      /*  it is expressed as a Tool pose [trvX trvY trvZ rotX rotY rotZ] in Euler */
      /*  coordinates with translation X-Y-Z and orientation with scheme X-Y-Z: */
      /*  from the Forward Kinematics with the current joint position values. */
      /*  */
      /*  FORWARDKINEMATICSPOE Forwark Kinematics for a Rigid Body Tree by POE */
      /*  Product Of Exponentials from a Screw Theory movement. */
      /*  Use in SE(3). */
      /*  */
      /*  HstR = ForwardKinematicsPOE(TwMag) */
      /*  */
      /*  Forward kinematics compute the RELATIVE TO THE STATIONARY SYSTEM */
      /*  end-effector motion (pos & rot) into a homogeneous matrix tform(4x4) */
      /*  as a funtion of the given joints motion defined by the "Twist-Mangitude" */
      /*  of n links. */
      /*  INPUTS: */
      /*  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn) */
      /*  for each rigid body joint (link 1..n). */
      /*  Twn1..Twn6: The TWIST components for the joint SCREW movement. */
      /*  Magn: The MAGNITUDE component for the joint SCREW movement. */
      /*  */
      /*    Example: */
      /*        %Calculate the Homogeneous Matrix RELATIVE transformation for the */
      /*        %end-effector of a robot with four links whose "Twist-Mangitude" */
      /*        %parameters are: */
      /*        TwMag1 = [xi1 t1]' = [[0 0 0 0 0 1] pi]'; */
      /*        TwMag2 = [xi2 t2]' = [[0 0 1 0 0 0] 2]'; */
      /*        TwMag3 = [xi3 t3]' = [[1 0 0 0 0 0] 3]'; */
      /*        TwMag4 = [xi4 t4]' = [[0 1 0 1 0 0] pi/4]'; */
      /*        TwMag = [TwMag1 TwMag2 TwMag3 TwMag4]; */
      /*        HstR = ForwardKinematicsPOE(TwMag) */
      /*        ans = */
      /*        -1.0000   -0.0000    0.0000   -3.0000 */
      /*         0.0000   -0.7071    0.7071   -0.7071 */
      /*              0    0.7071    0.7071    2.2929 */
      /*              0         0         0    1.0000 */
      /*  */
      /*  See also ForwardKinematicsDH(dhparams) */
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
      /*  HstR = ForwardKinematicsPOE(TwMag) */
      /*  */
      expScrew(*(double (*)[7])&TwMag[0], PoE);
      memcpy(&HstR[0], &PoE[0], sizeof(double) << 4);
      for (i = 0; i < 6; i++) {
        expScrew(*(double (*)[7])&TwMag[7 * (i + 1)], HstThe);
        for (i0 = 0; i0 < 4; i0++) {
          for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
            HstThe_tmp = PoE_tmp << 2;
            b_HstR[i0 + HstThe_tmp] = ((HstR[i0] * HstThe[HstThe_tmp] + HstR[i0
              + 4] * HstThe[1 + HstThe_tmp]) + HstR[i0 + 8] * HstThe[2 +
              HstThe_tmp]) + HstR[i0 + 12] * HstThe[3 + HstThe_tmp];
          }
        }

        memcpy(&HstR[0], &b_HstR[0], sizeof(double) << 4);
      }

      /*  */
      for (i0 = 0; i0 < 4; i0++) {
        for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
          HstThe_tmp = PoE_tmp << 2;
          HstThe[i0 + HstThe_tmp] = ((HstR[i0] * b[HstThe_tmp] + HstR[i0 + 4] *
            b[1 + HstThe_tmp]) + HstR[i0 + 8] * b[2 + HstThe_tmp]) + HstR[i0 +
            12] * b[3 + HstThe_tmp];
        }
      }

      for (i0 = 0; i0 < 3; i0++) {
        HstThe_tmp = i0 << 2;
        b_HstThe[3 * i0] = HstThe[HstThe_tmp];
        b_HstThe[1 + 3 * i0] = HstThe[1 + HstThe_tmp];
        b_HstThe[2 + 3 * i0] = HstThe[2 + HstThe_tmp];
      }

      rotm2eul(b_HstThe, dv2);

      /*  */
      /*  stamp is the integration step size (seconds). */
      /*  */
      /*  Target REFERENCE is the desired next cartesian trajectory point INPUT */
      /*  it is expressed as a Tool pose [trvX trvY trvZ rotX rotY rotZ] in Euler */
      /*  coordinates with translation X-Y-Z and orientation with scheme X-Y-Z: */
      /*  */
      /*  We now solve for the THETAP VELOCITIES values. */
      /*   */
      /*  VtS is the velocity for the Tool Pose in spatial frame (S) rad/s. */
      /*  consider the differentiartion step size. */
      /*    */
      /*  MINUSPOSEEUL - Pose difference between two trajectoria points */
      /*  the poses are expresed as Euler coordinates with translation X-Y-Z and  */
      /*  orientation with scheme X-Y-Z: Pose = [trvX trvY trvZ rotX rotY rotZ] */
      /*  */
      /*  	posediff = minusposeEul(pose1, pose2) */
      /*  */
      /*  Returns a pose difference expresed as Euler coordinates with the same */
      /*  structure as the inputs: pose difference = pose2 - pose1 */
      /*  expressed in the same reference frame as both input poses. */
      /*  The formulation is as follows: */
      /*  Position difference = [trvX1-trvX2 trvY1-trvY2 trvZ1-trvZ2] */
      /*  Rotation difference = R12 = R01' * R02 */
      /*  Pass R12 to axis/angle form, plus multiply the axis vector by the angle, */
      /*  getting the difference as an Euler X-Y-Z vector in the pose1 frame. */
      /*  The rotation difference is then expressed on the spatial common frame */
      /*  by multiplying the euler vector by the pose1 (R01) transformation. */
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
      /*  	posediff = minusposeEul(pose1, pose2) */
      /*  */
      TargetVAL[0] = HstThe[12];
      TargetVAL[3] = dv2[0];
      vtcpS_tmp[0] = u[0] - HstThe[12];
      TargetVAL[1] = HstThe[13];
      TargetVAL[4] = dv2[1];
      vtcpS_tmp[1] = u[1] - HstThe[13];
      TargetVAL[2] = HstThe[14];
      TargetVAL[5] = dv2[2];
      vtcpS_tmp[2] = u[2] - HstThe[14];
      eul2rotm(*(double (*)[3])&TargetVAL[3], a_tmp);
      for (i0 = 0; i0 < 3; i0++) {
        b_a_tmp[3 * i0] = a_tmp[i0];
        b_a_tmp[1 + 3 * i0] = a_tmp[i0 + 3];
        b_a_tmp[2 + 3 * i0] = a_tmp[i0 + 6];
      }

      eul2rotm(*(double (*)[3])&(*(double (*)[6])&u[0])[3], b_tmp);
      for (i0 = 0; i0 < 3; i0++) {
        for (PoE_tmp = 0; PoE_tmp < 3; PoE_tmp++) {
          b_HstThe[i0 + 3 * PoE_tmp] = (b_a_tmp[i0] * b_tmp[3 * PoE_tmp] +
            b_a_tmp[i0 + 3] * b_tmp[1 + 3 * PoE_tmp]) + b_a_tmp[i0 + 6] * b_tmp
            [2 + 3 * PoE_tmp];
        }
      }

      rotm2axang(b_HstThe, wstSaxang);

      /*  */
      /*  */
      for (i0 = 0; i0 < 3; i0++) {
        b_vtcpS_tmp[i0] = vtcpS_tmp[i0];
        b_vtcpS_tmp[i0 + 3] = (a_tmp[i0] * (wstSaxang[0] * wstSaxang[3]) +
          a_tmp[i0 + 3] * (wstSaxang[1] * wstSaxang[3])) + a_tmp[i0 + 6] *
          (wstSaxang[2] * wstSaxang[3]);
      }

      for (i0 = 0; i0 < 6; i0++) {
        VtS[i0] = b_vtcpS_tmp[i0] / 0.0005;
      }

      /*  */
      /*  GEOMETRIC JACOBIAN JstS and SPATIAL TWIST VELOCITY "VstS" at current pose */
      /*  */
      /*  "GeoJacobianS" computes Geometric Jacobian refered to the Spatial frame */
      /*  of a robot of any number of links. */
      /*  Use in SE(3). */
      /*  */
      /*  	JstS = GeoJacobianS(TwMag) */
      /*  */
      /*  GEOMETRIC JACOBIAN in SPATIAL frame: At each configuration of theta,  */
      /*  maps the joint velocity vector into the velocity of end effector. */
      /*  The contribution of the "ith" joint velocity to the end effector velicity */
      /*  is independent of the configuration of later joints in the chain. */
      /*  Thus, the "ith" column of jst is the "ith" joint twist, transformed to */
      /*  the current manipulator configuration. */
      /*  */
      /*  INPUTS: */
      /*  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn) */
      /*  for each rigid body joint (link 1..n). */
      /*  Twn1..Twn6: The TWIST components for the joint SCREW movement. */
      /*  Magn: The MAGNITUDE component for the joint SCREW movement. */
      /*  */
      /*                |v1 v2' ... vn'|  */
      /*  JstS(theta) = |              |    */
      /*                |w1 w2' ... wn'|   */
      /*            |vi'| */
      /*  With: Ei'=|   |=Ad                                       *Ei */
      /*            |wi'|   (exp(E1^theta1)*...*exp(Ei-1^thetai-1)) */
      /*  */
      /*  JstS(t)=Adg(t)*JstB(t) ; and Adg(t) is the Adjoint of gst(theta). */
      /*  */
      /*  See also: GeoJacobianT, expScrew,Tform2adjoint. */
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
      /*  JstS = GeoJacobianS(TwMag) */
      /*  */
      for (i0 = 0; i0 < 7; i0++) {
        for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
          JstS[PoE_tmp + 6 * i0] = TwMag[PoE_tmp + 7 * i0];
        }
      }

      for (i = 0; i < 6; i++) {
        /*  */
        /*  "TFORM2ADJOINT" Find the adjoint matrix associated with a tform. */
        /*  Use in SE(3). */
        /*  */
        /*  	Ad = tform2adjoint(tform) */
        /*  */
        /*  ADJOINT TRANSFORMATION: */
        /*  it is used to transforms twist from one coordinate frame to another. */
        /*  Vs =Adg*Vb ; Vac = Adgab*Vbc ; E'=Adg*E */
        /*  The adjoint transformation maps twist vectors to twist vectors. */
        /*  Compute the Adg in R^6 (6x6 matrix9 from the homogeneous matrix g 4x4. */
        /*       |R p^R|            |R p| */
        /*  Adg =|     | <= tform = |   | */
        /*       |0   R|            |0 1| */
        /*  With p^=axis2skew(p) */
        /*  */
        /*  See also: axis2skew, */
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
        /*  	Ad = tform2adjoint(tform) */
        /*  */
        /*  "axis2skew" Generate a skew symmetric matrix from a vector (axis) . */
        /*  Use in SO(3). */
        /*  */
        /*  	r = axis2skew(w) */
        /*  */
        /*  Returns a skew symmetric matrix r 3x3 from the vector 3x1 w[a1;a2;a3;]. */
        /*     |0  -a3  a2|  */
        /*  r =|a3   0 -a1| */
        /*     |-a2 a1   0| */
        /*  */
        /*  See also: skew2axis. */
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
        /*  r = axis2skew(w) */
        /*  */
        /*  */
        b_HstThe[0] = 0.0;
        b_HstThe[3] = -PoE[14];
        b_HstThe[6] = PoE[13];
        b_HstThe[1] = PoE[14];
        b_HstThe[4] = 0.0;
        b_HstThe[7] = -PoE[12];
        b_HstThe[2] = -PoE[13];
        b_HstThe[5] = PoE[12];
        b_HstThe[8] = 0.0;
        for (i0 = 0; i0 < 3; i0++) {
          for (PoE_tmp = 0; PoE_tmp < 3; PoE_tmp++) {
            HstThe_tmp = PoE_tmp << 2;
            dv5[i0 + 3 * PoE_tmp] = (b_HstThe[i0] * PoE[HstThe_tmp] +
              b_HstThe[i0 + 3] * PoE[1 + HstThe_tmp]) + b_HstThe[i0 + 6] * PoE[2
              + HstThe_tmp];
            b_PoE[PoE_tmp + 6 * i0] = PoE[PoE_tmp + (i0 << 2)];
          }
        }

        for (i0 = 0; i0 < 3; i0++) {
          HstThe_tmp = 6 * (i0 + 3);
          b_PoE[HstThe_tmp] = dv5[3 * i0];
          b_PoE[6 * i0 + 3] = 0.0;
          PoE_tmp = i0 << 2;
          b_PoE[HstThe_tmp + 3] = PoE[PoE_tmp];
          b_PoE[1 + HstThe_tmp] = dv5[1 + 3 * i0];
          b_PoE[6 * i0 + 4] = 0.0;
          b_PoE[HstThe_tmp + 4] = PoE[1 + PoE_tmp];
          b_PoE[2 + HstThe_tmp] = dv5[2 + 3 * i0];
          b_PoE[6 * i0 + 5] = 0.0;
          b_PoE[HstThe_tmp + 5] = PoE[2 + PoE_tmp];
        }

        for (i0 = 0; i0 < 6; i0++) {
          HstThe_tmp = i0 + 6 * (i + 1);
          JstS[HstThe_tmp] = 0.0;
          for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
            JstS[HstThe_tmp] += b_PoE[i0 + 6 * PoE_tmp] * TwMag[PoE_tmp + 7 * (i
              + 1)];
          }
        }

        expScrew(*(double (*)[7])&TwMag[7 * (i + 1)], HstThe);
        for (i0 = 0; i0 < 4; i0++) {
          for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
            HstThe_tmp = PoE_tmp << 2;
            HstR[i0 + HstThe_tmp] = ((PoE[i0] * HstThe[HstThe_tmp] + PoE[i0 + 4]
              * HstThe[1 + HstThe_tmp]) + PoE[i0 + 8] * HstThe[2 + HstThe_tmp])
              + PoE[i0 + 12] * HstThe[3 + HstThe_tmp];
          }
        }

        memcpy(&PoE[0], &HstR[0], sizeof(double) << 4);
      }

      /*  */
      /*  */
      /*  "axis2skew" Generate a skew symmetric matrix from a vector (axis) . */
      /*  Use in SO(3). */
      /*  */
      /*  	r = axis2skew(w) */
      /*  */
      /*  Returns a skew symmetric matrix r 3x3 from the vector 3x1 w[a1;a2;a3;]. */
      /*     |0  -a3  a2|  */
      /*  r =|a3   0 -a1| */
      /*     |-a2 a1   0| */
      /*  */
      /*  See also: skew2axis. */
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
      /*  r = axis2skew(w) */
      /*  */
      b_HstThe[0] = 0.0;
      b_HstThe[3] = -VtS[5];
      b_HstThe[6] = VtS[4];
      b_HstThe[1] = VtS[5];
      b_HstThe[4] = 0.0;
      b_HstThe[7] = -VtS[3];
      b_HstThe[2] = -VtS[4];
      b_HstThe[5] = VtS[3];
      b_HstThe[8] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        VstS[i0] = VtS[i0] - ((b_HstThe[i0] * TargetVAL[0] + b_HstThe[i0 + 3] *
          TargetVAL[1]) + b_HstThe[i0 + 6] * TargetVAL[2]);
        VstS[i0 + 3] = VtS[3 + i0];
      }

      /*  */
      /*  Using Moore-Penrose generalized inverse for getting Theta velocities. */
      /*  Thetap = JstS'*((JstS*JstS')\VstS; */
      /*  Thetap = pinv(JstS)*VstS; it is giving worse results. */
      xgeqp3(JstS, TargetVAL, jpvt);
      rankR = 0;
      tol = 1.5543122344752192E-14 * fabs(JstS[0]);
      while ((rankR < 6) && (!(fabs(JstS[rankR + 6 * rankR]) <= tol))) {
        rankR++;
      }

      for (i = 0; i < 7; i++) {
        Y[i] = 0.0;
      }

      for (j = 0; j < 6; j++) {
        if (TargetVAL[j] != 0.0) {
          tol = VstS[j];
          i0 = j + 2;
          for (i = i0; i < 7; i++) {
            tol += JstS[(i + 6 * j) - 1] * VstS[i - 1];
          }

          tol *= TargetVAL[j];
          if (tol != 0.0) {
            VstS[j] -= tol;
            i0 = j + 2;
            for (i = i0; i < 7; i++) {
              VstS[i - 1] -= JstS[(i + 6 * j) - 1] * tol;
            }
          }
        }
      }

      for (i = 0; i < rankR; i++) {
        Y[jpvt[i] - 1] = VstS[i];
      }

      for (j = rankR; j >= 1; j--) {
        HstThe_tmp = jpvt[j - 1] - 1;
        PoE_tmp = 6 * (j - 1);
        Y[HstThe_tmp] /= JstS[(j + PoE_tmp) - 1];
        for (i = 0; i <= j - 2; i++) {
          Y[jpvt[i] - 1] -= Y[HstThe_tmp] * JstS[i + PoE_tmp];
        }
      }

      /*  */
      /*  The Theta VELOCITIES values are limited by the joints spped limits. */
      /* Thetap = min(abs(Thetap),Thpmax)*diag(sign(Thetap)); */
      /*  */
      /*  The Theta ACCELERATION is calculated with reference to Theta VELOCITIES. */
      /* Thetapp = (Thetap - ThetapVAL) / stamp; */
      for (i0 = 0; i0 < 7; i0++) {
        Thetapp[i0] = (Y[i0] - ThetapVAL[i0]) / 2.0;
      }

      /*  */
      /*  Now we solve for the NEW THETA POSITION values. */
      /*  */
      if (u[6] <= 16.0) {
        /*  Solve Inverse Kinematics to get a set of possible solutions for joint */
        /*  positions (exact or approximate). Theta Set has 16 solutions (16x7) */
        /*  From the set of solutions we choose only one to proceed. */
        /*  Theta VALUE is the new joint positions vector (1x7) OUTPUT */
        Fcn_ST24R_IIWA14_IK_Upright(*(double (*)[6])&u[0], ThetaSet);
        for (i0 = 0; i0 < 7; i0++) {
          ThetaVALnew[i0] = ThetaSet[((int)u[6] + (i0 << 4)) - 1];
        }
      } else {
        /*  */
        /*  When selector is AUTO DIFFERENTIAL KINEMATICS is applied. */
        /*  from Inverse DK we get the incremental joint coordinates */
        /*  and then integrating with EULER Explicit Method the Theta VALUE  */
        /*  with the new joint positions vector (1x7) OUTPUT. */
        /*  consider the integration step size. */
        for (i0 = 0; i0 < 7; i0++) {
          ThetaVALnew[i0] = ThetaVAL[i0] + Y[i0] * 0.0005;
        }

        /*  */
      }

      /*  */
      /*  The Theta POSITION values are limited by the joints position limits. */
      /*  */
      /*  For some Target REFERENCE out of the workspace, the Theta VALUE can get */
      /*  some huge incorrect values. To avoid these kind of problems, we implement  */
      /*  a check loop to select the solution with the better approximation to the */
      /*  Target REFERENCE POSITION. The new or the previous Theta VALUE. */
      for (i = 0; i < 7; i++) {
        tol = jointmag2limits(ThetaVALnew[i], dv3[i], dv4[i]);
        ThetaVALnew[i] = tol;
        for (i0 = 0; i0 < 6; i0++) {
          TwMag[i0 + 7 * i] = dv1[i0 + 6 * i];
        }

        TwMag[6 + 7 * i] = tol;
      }

      /*  */
      /*  FORWARDKINEMATICSPOE Forwark Kinematics for a Rigid Body Tree by POE */
      /*  Product Of Exponentials from a Screw Theory movement. */
      /*  Use in SE(3). */
      /*  */
      /*  HstR = ForwardKinematicsPOE(TwMag) */
      /*  */
      /*  Forward kinematics compute the RELATIVE TO THE STATIONARY SYSTEM */
      /*  end-effector motion (pos & rot) into a homogeneous matrix tform(4x4) */
      /*  as a funtion of the given joints motion defined by the "Twist-Mangitude" */
      /*  of n links. */
      /*  INPUTS: */
      /*  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn) */
      /*  for each rigid body joint (link 1..n). */
      /*  Twn1..Twn6: The TWIST components for the joint SCREW movement. */
      /*  Magn: The MAGNITUDE component for the joint SCREW movement. */
      /*  */
      /*    Example: */
      /*        %Calculate the Homogeneous Matrix RELATIVE transformation for the */
      /*        %end-effector of a robot with four links whose "Twist-Mangitude" */
      /*        %parameters are: */
      /*        TwMag1 = [xi1 t1]' = [[0 0 0 0 0 1] pi]'; */
      /*        TwMag2 = [xi2 t2]' = [[0 0 1 0 0 0] 2]'; */
      /*        TwMag3 = [xi3 t3]' = [[1 0 0 0 0 0] 3]'; */
      /*        TwMag4 = [xi4 t4]' = [[0 1 0 1 0 0] pi/4]'; */
      /*        TwMag = [TwMag1 TwMag2 TwMag3 TwMag4]; */
      /*        HstR = ForwardKinematicsPOE(TwMag) */
      /*        ans = */
      /*        -1.0000   -0.0000    0.0000   -3.0000 */
      /*         0.0000   -0.7071    0.7071   -0.7071 */
      /*              0    0.7071    0.7071    2.2929 */
      /*              0         0         0    1.0000 */
      /*  */
      /*  See also ForwardKinematicsDH(dhparams) */
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
      /*  HstR = ForwardKinematicsPOE(TwMag) */
      /*  */
      expScrew(*(double (*)[7])&TwMag[0], HstR);
      for (i = 0; i < 6; i++) {
        expScrew(*(double (*)[7])&TwMag[7 * (i + 1)], HstThe);
        for (i0 = 0; i0 < 4; i0++) {
          for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
            HstThe_tmp = PoE_tmp << 2;
            b_HstR[i0 + HstThe_tmp] = ((HstR[i0] * HstThe[HstThe_tmp] + HstR[i0
              + 4] * HstThe[1 + HstThe_tmp]) + HstR[i0 + 8] * HstThe[2 +
              HstThe_tmp]) + HstR[i0 + 12] * HstThe[3 + HstThe_tmp];
          }
        }

        memcpy(&HstR[0], &b_HstR[0], sizeof(double) << 4);
      }

      /*  */
      for (i0 = 0; i0 < 4; i0++) {
        for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
          HstThe_tmp = PoE_tmp << 2;
          HstThe[i0 + HstThe_tmp] = ((HstR[i0] * b[HstThe_tmp] + HstR[i0 + 4] *
            b[1 + HstThe_tmp]) + HstR[i0 + 8] * b[2 + HstThe_tmp]) + HstR[i0 +
            12] * b[3 + HstThe_tmp];
        }
      }

      for (i0 = 0; i0 < 3; i0++) {
        HstThe_tmp = i0 << 2;
        b_HstThe[3 * i0] = HstThe[HstThe_tmp];
        b_HstThe[1 + 3 * i0] = HstThe[1 + HstThe_tmp];
        b_HstThe[2 + 3 * i0] = HstThe[2 + HstThe_tmp];
      }

      rotm2eul(b_HstThe, dv2);

      /*    */
      /*  MINUSPOSEEUL - Pose difference between two trajectoria points */
      /*  the poses are expresed as Euler coordinates with translation X-Y-Z and  */
      /*  orientation with scheme X-Y-Z: Pose = [trvX trvY trvZ rotX rotY rotZ] */
      /*  */
      /*  	posediff = minusposeEul(pose1, pose2) */
      /*  */
      /*  Returns a pose difference expresed as Euler coordinates with the same */
      /*  structure as the inputs: pose difference = pose2 - pose1 */
      /*  expressed in the same reference frame as both input poses. */
      /*  The formulation is as follows: */
      /*  Position difference = [trvX1-trvX2 trvY1-trvY2 trvZ1-trvZ2] */
      /*  Rotation difference = R12 = R01' * R02 */
      /*  Pass R12 to axis/angle form, plus multiply the axis vector by the angle, */
      /*  getting the difference as an Euler X-Y-Z vector in the pose1 frame. */
      /*  The rotation difference is then expressed on the spatial common frame */
      /*  by multiplying the euler vector by the pose1 (R01) transformation. */
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
      /*  	posediff = minusposeEul(pose1, pose2) */
      /*  */
      for (i0 = 0; i0 < 3; i0++) {
        TargetVAL[i0] = HstThe[12 + i0];
        TargetVAL[i0 + 3] = dv2[i0];
        for (PoE_tmp = 0; PoE_tmp < 3; PoE_tmp++) {
          b_HstThe[i0 + 3 * PoE_tmp] = (b_a_tmp[i0] * b_tmp[3 * PoE_tmp] +
            b_a_tmp[i0 + 3] * b_tmp[1 + 3 * PoE_tmp]) + b_a_tmp[i0 + 6] * b_tmp
            [2 + 3 * PoE_tmp];
        }
      }

      rotm2axang(b_HstThe, wstSaxang);

      /*  */
      /*  */
      /*    */
      /*  MINUSPOSEEUL - Pose difference between two trajectoria points */
      /*  the poses are expresed as Euler coordinates with translation X-Y-Z and  */
      /*  orientation with scheme X-Y-Z: Pose = [trvX trvY trvZ rotX rotY rotZ] */
      /*  */
      /*  	posediff = minusposeEul(pose1, pose2) */
      /*  */
      /*  Returns a pose difference expresed as Euler coordinates with the same */
      /*  structure as the inputs: pose difference = pose2 - pose1 */
      /*  expressed in the same reference frame as both input poses. */
      /*  The formulation is as follows: */
      /*  Position difference = [trvX1-trvX2 trvY1-trvY2 trvZ1-trvZ2] */
      /*  Rotation difference = R12 = R01' * R02 */
      /*  Pass R12 to axis/angle form, plus multiply the axis vector by the angle, */
      /*  getting the difference as an Euler X-Y-Z vector in the pose1 frame. */
      /*  The rotation difference is then expressed on the spatial common frame */
      /*  by multiplying the euler vector by the pose1 (R01) transformation. */
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
      /*  	posediff = minusposeEul(pose1, pose2) */
      /*  */
      eul2rotm(*(double (*)[3])&TargetVAL[3], b_a_tmp);
      for (i0 = 0; i0 < 3; i0++) {
        for (PoE_tmp = 0; PoE_tmp < 3; PoE_tmp++) {
          b_HstThe[i0 + 3 * PoE_tmp] = (b_a_tmp[3 * i0] * b_tmp[3 * PoE_tmp] +
            b_a_tmp[1 + 3 * i0] * b_tmp[1 + 3 * PoE_tmp]) + b_a_tmp[2 + 3 * i0] *
            b_tmp[2 + 3 * PoE_tmp];
        }
      }

      rotm2axang(b_HstThe, b_wstSaxang);

      /*  */
      /*  */
      for (i0 = 0; i0 < 3; i0++) {
        b_vtcpS_tmp[i0] = vtcpS_tmp[i0];
        b_vtcpS_tmp[i0 + 3] = (a_tmp[i0] * (wstSaxang[0] * wstSaxang[3]) +
          a_tmp[i0 + 3] * (wstSaxang[1] * wstSaxang[3])) + a_tmp[i0 + 6] *
          (wstSaxang[2] * wstSaxang[3]);
        VtS[i0] = u[iv0[iv1[i0]]] - TargetVAL[iv1[i0]];
        VtS[i0 + 3] = (b_a_tmp[i0] * (b_wstSaxang[0] * b_wstSaxang[3]) +
                       b_a_tmp[i0 + 3] * (b_wstSaxang[1] * b_wstSaxang[3])) +
          b_a_tmp[i0 + 6] * (b_wstSaxang[2] * b_wstSaxang[3]);
      }

      if (b_norm(*(double (*)[3])&b_vtcpS_tmp[0]) < b_norm(*(double (*)[3])&VtS
           [0])) {
        for (i0 = 0; i0 < 7; i0++) {
          ThetaVALnew[i0] = ThetaVAL[i0];
        }
      }

      /*  */
      /*  */
      /*  Theta VALUE is the new joint positions vector (1x7) OUTPUT */
      for (i0 = 0; i0 < 7; i0++) {
        ThetaVAL[i0] = ThetaVALnew[i0];
        ThetapVAL[i0] = Y[i0];
        ThetaOut[i0] = ThetaVALnew[i0];
        ThetaOut[i0 + 7] = Y[i0];
        ThetaOut[i0 + 14] = Thetapp[i0];
      }

      /*  */
    }
  }

  /*  */
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void c_F187_ST24R_IIWA14_IKDK_OnLine(void)
{
  int i7;
  for (i7 = 0; i7 < 7; i7++) {
    ThetaVAL[i7] = 0.0;
    ThetapVAL[i7] = 0.0;
  }
}

/*
 * File trailer for F187_ST24R_IIWA14_IKDK_OnLineJThe.c
 *
 * [EOF]
 */
