//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: NPotentialWre6J.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 11-Nov-2020 15:21:44
//

// Include Files
#include "NPotentialWre6J.h"
#include "expScrew.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double TwMag[42]
//                const double LiMas[42]
//                const double PoAcc[3]
//                double Nt[6]
// Return Type  : void
//
void NPotentialWre6J(const double TwMag[42], const double LiMas[42], const
                     double PoAcc[3], double Nt[6])
{
  double JstS[36];
  double Wrench[36];
  double c_PoE[36];
  double tmp_data[36];
  double PoE[16];
  double b_PoE[16];
  double dv1[16];
  double dv[9];
  double dv2[9];
  double b_TwMag[7];
  double MagnG;
  double absxk;
  double d;
  double d1;
  double d2;
  double d3;
  double scale;
  double t;
  int PoE_tmp;
  int b_PoE_tmp;
  int b_i;
  int c_i;
  int i;

  //  "NPotentialWre" Potential with the NEW GRAVITY WRENCH Matrix N(t)
  //  for an open chain manipulator.
  //  computation based on the use of the gravity WRENCH on the robot links.
  //  Use in SE(3).
  //
  //  A N(t) new formulation in ST24R by Dr. Pardos-Gotor
  //  It allows to get the Potential matrix in Dynamics, avoiding
  //  the differentiation of the Potential Energy V(t).
  //
  //  	Nt = NPotentialWre(TwMag,LiMas,PoAcc)
  //
  //  Gives the POTENTIAL MATRIX N(t) for the Lagangian's equations:
  //  M(t)*ddt + C(t,dt)*dt + N(t) = T
  //  of the dynamics of the robot formed by links on an open chain.
  //  It does not consider friction N(t, dt) and only gravitational N(t).
  //
  //  INPUTS:
  //  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
  //  for each rigid body joint (1..n).
  //  Tw1..Twn: The TWIST components for the joint movement.
  //  Mag1..Magn: The MAGNITUDE component for the joint SCREW movement.
  //  LiMas = [CM1; IT1; Mass1, ..., CMn; ITn; Massn] (7xn)
  //  for each rigid body link (1..n).
  //  CM1..CMn: Center of Mass x-y-z Position components to S for each Link.
  //  IT1..ITn: Inertia x-y-z components for each Link refered to its CM.
  //  Mass1..Massn: The Mass for each Link.
  //  PoAcc = Vector (3x1) with the accelerations for potential energies.
  //  e.g PoAcc = [0 0 -9.81] if the only acceleration is gravity on -Z axis.
  //
  //  OUTPUTS:
  //  N(t) (nx1) potential matrix for the dynamics expression.
  //  Even though it is normally used only with the gravity on Z, the
  //  expression is genral for whatever selection of axes and even for other
  //  types of acceleration. Imagine a mobil robot in 3D (e.g. drone) or a
  //  space robot with different values and directions for gravity.
  //
  //       |Nt1|          n
  //  Nt = |   |; Nti = - Sum  JstSi*Fgi
  //       |Ntn|          1
  //
  //  with JslSi the Spatial Jacobian affecting the link i.
  //
  //  with Fgi the GRAVITY WRENCH (pure translational) in the SPATIAL frame.
  //            |  w |      |   wg  |  |        wg          |
  //  Fgi = mi*g|    |= mi*g|       |= |                    |
  //            |-wxq|      |-wgxCMi|  |-wg x POE(1:i)*Hsli0|
  //  wg is the gravity axis application.
  //  CMi center of mass for the link i, calculated for the current pose.
  //
  //  See also: MInertiaAij, CCoriolisAij, NPotentialAij.
  //  See also: MInertiaJsl, NPotentialDifSym.
  //
  //  Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor.
  //
  //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB
  //
  //  ST24R is free software: you can redistribute it and/or modify
  //  it under the terms of the GNU Lesser General Public License as published
  //  by the Free Software Foundation, either version 3 of the License, or
  //  (at your option) any later version.
  //
  //  ST24R is distributed in the hope that it will be useful,
  //  but WITHOUT ANY WARRANTY; without even the implied warranty of
  //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  //  GNU Lesser General Public License for more details.
  //
  //  You should have received a copy of the GNU Leser General Public License
  //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
  //
  //  http://www.
  //
  //  CHANGES:
  //  Revision 1.1  2020/02/11 00:00:01
  //  General cleanup of code: help comments, see also, copyright
  //  references, clarification of functions.
  //
  //  Nt = NPotentialWre(TwMag,LiMas,PoAcc)
  //
  scale = 3.3121686421112381E-170;
  absxk = std::abs(PoAcc[0]);
  if (absxk > 3.3121686421112381E-170) {
    MagnG = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    MagnG = t * t;
  }

  absxk = std::abs(PoAcc[1]);
  if (absxk > scale) {
    t = scale / absxk;
    MagnG = MagnG * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    MagnG += t * t;
  }

  absxk = std::abs(PoAcc[2]);
  if (absxk > scale) {
    t = scale / absxk;
    MagnG = MagnG * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    MagnG += t * t;
  }

  MagnG = scale * std::sqrt(MagnG);
  scale = PoAcc[0] / MagnG;
  absxk = PoAcc[1] / MagnG;
  t = PoAcc[2] / MagnG;
  for (i = 0; i < 6; i++) {
    double a;

    //
    //  TRVP2TFORM Convert to Homogeneous matrix a translation P vector
    //   Hp = TRVP2TFORM(P) converts a translation P axis into the
    //   corresponding homogeneous matrix H. P is a position in longitude units. 
    //
    //    Example:
    //        %Calculate the homogeneous matrix for a translation p = [px;py;pz] 
    //        on X axis.
    //        Hxp = trvP2tform(p)
    //        % Hp = [1 0 0 px; 0 1 0 py; 0 0 1 pz; 0 0 0 1]
    //        ans =
    //                 1         0         0         px
    //                 0         1         0         py
    //                 0         0         1         pz
    //                 0         0         0         1
    //
    //  See also trvY2tform(p), trvY2tform(p), trvZ2tform(p).
    //
    //  Copyright (C) 2003-2019, by Dr. Jose M. Pardos-Gotor.
    //
    //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB 
    //
    //  ST24R is free software: you can redistribute it and/or modify
    //  it under the terms of the GNU Lesser General Public License as published 
    //  by the Free Software Foundation, either version 3 of the License, or
    //  (at your option) any later version.
    //
    //  ST24R is distributed in the hope that it will be useful,
    //  but WITHOUT ANY WARRANTY; without even the implied warranty of
    //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    //  GNU Lesser General Public License for more details.
    //
    //  You should have received a copy of the GNU Leser General Public License
    //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
    //
    //  http://www.
    //
    //  CHANGES:
    //  Revision 1.1  2019/02/11 00:00:01
    //  General cleanup of code: help comments, see also, copyright
    //  references, clarification of functions.
    //
    //  Hxp = trvX2tform(pg)
    //
    //
    //
    //  FORWARDKINEMATICSPOE Forwark Kinematics for a Rigid Body Tree by POE
    //  Product Of Exponentials from a Screw Theory movement.
    //  Use in SE(3).
    //
    //  HstR = ForwardKinematicsPOE(TwMag)
    //
    //  Forward kinematics compute the RELATIVE TO THE STATIONARY SYSTEM
    //  end-effector motion (pos & rot) into a homogeneous matrix tform(4x4)
    //  as a funtion of the given joints motion defined by the "Twist-Mangitude" 
    //  of n links.
    //  INPUTS:
    //  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
    //  for each rigid body joint (link 1..n).
    //  Twn1..Twn6: The TWIST components for the joint SCREW movement.
    //  Magn: The MAGNITUDE component for the joint SCREW movement.
    //
    //    Example:
    //        %Calculate the Homogeneous Matrix RELATIVE transformation for the
    //        %end-effector of a robot with four links whose "Twist-Mangitude"
    //        %parameters are:
    //        TwMag1 = [xi1 t1]' = [[0 0 0 0 0 1] pi]';
    //        TwMag2 = [xi2 t2]' = [[0 0 1 0 0 0] 2]';
    //        TwMag3 = [xi3 t3]' = [[1 0 0 0 0 0] 3]';
    //        TwMag4 = [xi4 t4]' = [[0 1 0 1 0 0] pi/4]';
    //        TwMag = [TwMag1 TwMag2 TwMag3 TwMag4];
    //        HstR = ForwardKinematicsPOE(TwMag)
    //        ans =
    //        -1.0000   -0.0000    0.0000   -3.0000
    //         0.0000   -0.7071    0.7071   -0.7071
    //              0    0.7071    0.7071    2.2929
    //              0         0         0    1.0000
    //
    //  See also ForwardKinematicsDH(dhparams)
    //
    //  Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor.
    //
    //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB 
    //
    //  ST24R is free software: you can redistribute it and/or modify
    //  it under the terms of the GNU Lesser General Public License as published 
    //  by the Free Software Foundation, either version 3 of the License, or
    //  (at your option) any later version.
    //
    //  ST24R is distributed in the hope that it will be useful,
    //  but WITHOUT ANY WARRANTY; without even the implied warranty of
    //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    //  GNU Lesser General Public License for more details.
    //
    //  You should have received a copy of the GNU Leser General Public License
    //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
    //
    //  http://www.
    //
    //  CHANGES:
    //  Revision 1.1  2020/02/11 00:00:01
    //  General cleanup of code: help comments, see also, copyright
    //  references, clarification of functions.
    //
    //  HstR = ForwardKinematicsPOE(TwMag)
    //
    for (b_i = 0; b_i < 7; b_i++) {
      b_TwMag[b_i] = TwMag[b_i];
    }

    expScrew(b_TwMag, PoE);
    for (c_i = 0; c_i < i; c_i++) {
      for (b_i = 0; b_i < 7; b_i++) {
        b_TwMag[b_i] = TwMag[b_i + 7 * (c_i + 1)];
      }

      expScrew(b_TwMag, dv1);
      for (b_i = 0; b_i < 4; b_i++) {
        d = PoE[b_i];
        d1 = PoE[b_i + 4];
        d2 = PoE[b_i + 8];
        d3 = PoE[b_i + 12];
        for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
          b_PoE_tmp = PoE_tmp << 2;
          b_PoE[b_i + b_PoE_tmp] = ((d * dv1[b_PoE_tmp] + d1 * dv1[b_PoE_tmp + 1])
            + d2 * dv1[b_PoE_tmp + 2]) + d3 * dv1[b_PoE_tmp + 3];
        }
      }

      std::memcpy(&PoE[0], &b_PoE[0], 16U * sizeof(double));
    }

    //
    a = LiMas[7 * i + 6] * MagnG;
    dv1[0] = 1.0;
    dv1[4] = 0.0;
    dv1[8] = 0.0;
    dv1[12] = LiMas[7 * i];
    dv1[1] = 0.0;
    dv1[5] = 1.0;
    dv1[9] = 0.0;
    dv1[13] = LiMas[7 * i + 1];
    dv1[2] = 0.0;
    dv1[6] = 0.0;
    dv1[10] = 1.0;
    dv1[14] = LiMas[7 * i + 2];
    dv1[3] = 0.0;
    dv1[7] = 0.0;
    dv1[11] = 0.0;
    dv1[15] = 1.0;
    for (b_i = 0; b_i < 4; b_i++) {
      d = PoE[b_i];
      d1 = PoE[b_i + 4];
      d2 = PoE[b_i + 8];
      d3 = PoE[b_i + 12];
      for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
        b_PoE_tmp = PoE_tmp << 2;
        b_PoE[b_i + b_PoE_tmp] = ((d * dv1[b_PoE_tmp] + d1 * dv1[b_PoE_tmp + 1])
          + d2 * dv1[b_PoE_tmp + 2]) + d3 * dv1[b_PoE_tmp + 3];
      }
    }

    //
    //  LINK2WRENCH gets de WRENCH from the LINK AXIS and a POINT on that axis
    //  Use in SE(3).
    //
    //  where the WRENCH "fo" (6x1) has the two components f (3x1) and T (3x1)
    //  From AXIS (3x1) & a POINT q (3X1) on that axis AT THE REFERENCE POSITION. 
    //  It is also necessary to indicate the type of FORCETYPE ('rot' or 'tra')
    //  for the function to work with both ROTATION & TRANSLATION movements.
    //  Use in SE(3).
    //
    //    fo = link2wrench(ForceAxis, ForcePoint, forceType)
    //
    //      |f|   |       0        |
    //  fo =| | = |                |: for only ROTATION torque.
    //      |T|   |    ForceAxis   |
    //
    //      |f|   |    ForceAxis    |
    //  fo =| | = |                 |: for only TRANSLATION force.
    //      |T|   | -ForceAxis X q  |
    //
    //  See also: .
    //
    //  Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor.
    //
    //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB 
    //
    //  ST24R is free software: you can redistribute it and/or modify
    //  it under the terms of the GNU Lesser General Public License as published 
    //  by the Free Software Foundation, either version 3 of the License, or
    //  (at your option) any later version.
    //
    //  ST24R is distributed in the hope that it will be useful,
    //  but WITHOUT ANY WARRANTY; without even the implied warranty of
    //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    //  GNU Lesser General Public License for more details.
    //
    //  You should have received a copy of the GNU Leser General Public License
    //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
    //
    //  http://www.
    //
    //  CHANGES:
    //  Revision 1.1  2019/02/11 00:00:01
    //  General cleanup of code: help comments, see also, copyright
    //  references, clarification of functions.
    //
    //  	fo = link2wrench(ForceAxis, ForcePoint, forceType)
    //
    //
    Wrench[6 * i] = a * scale;
    Wrench[6 * i + 1] = a * absxk;
    Wrench[6 * i + 2] = a * t;
    Wrench[6 * i + 3] = a * -(absxk * b_PoE[14] - t * b_PoE[13]);
    Wrench[6 * i + 4] = a * -(t * b_PoE[12] - scale * b_PoE[14]);
    Wrench[6 * i + 5] = a * -(scale * b_PoE[13] - absxk * b_PoE[12]);
    Nt[i] = 0.0;
  }

  for (i = 0; i < 6; i++) {
    int tmp_size_idx_1;
    std::memset(&JstS[0], 0, 36U * sizeof(double));

    //
    //  "GeoJacobianS" computes Geometric Jacobian refered to the Spatial frame
    //  of a robot of any number of links.
    //  Use in SE(3).
    //
    //  	JstS = GeoJacobianS(TwMag)
    //
    //  GEOMETRIC JACOBIAN in SPATIAL frame: At each configuration of theta,
    //  maps the joint velocity vector into the velocity of end effector.
    //  The contribution of the "ith" joint velocity to the end effector velicity 
    //  is independent of the configuration of later joints in the chain.
    //  Thus, the "ith" column of jst is the "ith" joint twist, transformed to
    //  the current manipulator configuration.
    //
    //  INPUTS:
    //  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
    //  for each rigid body joint (link 1..n).
    //  Twn1..Twn6: The TWIST components for the joint SCREW movement.
    //  Magn: The MAGNITUDE component for the joint SCREW movement.
    //
    //                |v1 v2' ... vn'|
    //  JstS(theta) = |              |
    //                |w1 w2' ... wn'|
    //            |vi'|
    //  With: Ei'=|   |=Ad                                       *Ei
    //            |wi'|   (exp(E1^theta1)*...*exp(Ei-1^thetai-1))
    //
    //  JstS(t)=Adg(t)*JstB(t) ; and Adg(t) is the Adjoint of gst(theta).
    //
    //  See also: GeoJacobianT, expScrew,Tform2adjoint.
    //
    //  Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor.
    //
    //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB 
    //
    //  ST24R is free software: you can redistribute it and/or modify
    //  it under the terms of the GNU Lesser General Public License as published 
    //  by the Free Software Foundation, either version 3 of the License, or
    //  (at your option) any later version.
    //
    //  ST24R is distributed in the hope that it will be useful,
    //  but WITHOUT ANY WARRANTY; without even the implied warranty of
    //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    //  GNU Lesser General Public License for more details.
    //
    //  You should have received a copy of the GNU Leser General Public License
    //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
    //
    //  http://www.
    //
    //  CHANGES:
    //  Revision 1.1  2019/02/11 00:00:01
    //  General cleanup of code: help comments, see also, copyright
    //  references, clarification of functions.
    //
    //  JstS = GeoJacobianS(TwMag)
    //
    tmp_size_idx_1 = i + 1;
    for (b_i = 0; b_i <= i; b_i++) {
      for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
        tmp_data[PoE_tmp + 6 * b_i] = TwMag[PoE_tmp + 7 * b_i];
      }
    }

    for (b_i = 0; b_i < 7; b_i++) {
      b_TwMag[b_i] = TwMag[b_i];
    }

    expScrew(b_TwMag, PoE);
    if (0 <= i - 1) {
      dv[0] = 0.0;
      dv[4] = 0.0;
      dv[8] = 0.0;
    }

    for (c_i = 0; c_i < i; c_i++) {
      //
      //  "TFORM2ADJOINT" Find the adjoint matrix associated with a tform.
      //  Use in SE(3).
      //
      //  	Ad = tform2adjoint(tform)
      //
      //  ADJOINT TRANSFORMATION:
      //  it is used to transforms twist from one coordinate frame to another.
      //  Vs =Adg*Vb ; Vac = Adgab*Vbc ; E'=Adg*E
      //  The adjoint transformation maps twist vectors to twist vectors.
      //  Compute the Adg in R^6 (6x6 matrix9 from the homogeneous matrix g 4x4. 
      //       |R p^R|            |R p|
      //  Adg =|     | <= tform = |   |
      //       |0   R|            |0 1|
      //  With p^=axis2skew(p)
      //
      //  See also: axis2skew,
      //
      //  Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor.
      //
      //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB 
      //
      //  ST24R is free software: you can redistribute it and/or modify
      //  it under the terms of the GNU Lesser General Public License as published 
      //  by the Free Software Foundation, either version 3 of the License, or
      //  (at your option) any later version.
      //
      //  ST24R is distributed in the hope that it will be useful,
      //  but WITHOUT ANY WARRANTY; without even the implied warranty of
      //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      //  GNU Lesser General Public License for more details.
      //
      //  You should have received a copy of the GNU Leser General Public License 
      //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
      //
      //  http://www.
      //
      //  CHANGES:
      //  Revision 1.1  2019/02/11 00:00:01
      //  General cleanup of code: help comments, see also, copyright
      //  references, clarification of functions.
      //
      //  	Ad = tform2adjoint(tform)
      //
      //  "axis2skew" Generate a skew symmetric matrix from a vector (axis) .
      //  Use in SO(3).
      //
      //  	r = axis2skew(w)
      //
      //  Returns a skew symmetric matrix r 3x3 from the vector 3x1 w[a1;a2;a3;]. 
      //     |0  -a3  a2|
      //  r =|a3   0 -a1|
      //     |-a2 a1   0|
      //
      //  See also: skew2axis.
      //
      //  Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor.
      //
      //  This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB 
      //
      //  ST24R is free software: you can redistribute it and/or modify
      //  it under the terms of the GNU Lesser General Public License as published 
      //  by the Free Software Foundation, either version 3 of the License, or
      //  (at your option) any later version.
      //
      //  ST24R is distributed in the hope that it will be useful,
      //  but WITHOUT ANY WARRANTY; without even the implied warranty of
      //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      //  GNU Lesser General Public License for more details.
      //
      //  You should have received a copy of the GNU Leser General Public License 
      //  along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
      //
      //  http://www.
      //
      //  CHANGES:
      //  Revision 1.1  2020/02/11 00:00:01
      //  General cleanup of code: help comments, see also, copyright
      //  references, clarification of functions.
      //
      //  r = axis2skew(w)
      //
      //
      dv[3] = -PoE[14];
      dv[6] = PoE[13];
      dv[1] = PoE[14];
      dv[7] = -PoE[12];
      dv[2] = -PoE[13];
      dv[5] = PoE[12];
      for (b_i = 0; b_i < 3; b_i++) {
        d = dv[b_i];
        d1 = dv[b_i + 3];
        d2 = dv[b_i + 6];
        for (PoE_tmp = 0; PoE_tmp < 3; PoE_tmp++) {
          b_PoE_tmp = PoE_tmp << 2;
          dv2[b_i + 3 * PoE_tmp] = (d * PoE[b_PoE_tmp] + d1 * PoE[b_PoE_tmp + 1])
            + d2 * PoE[b_PoE_tmp + 2];
          c_PoE[PoE_tmp + 6 * b_i] = PoE[PoE_tmp + (b_i << 2)];
        }
      }

      for (b_i = 0; b_i < 3; b_i++) {
        b_PoE_tmp = 6 * (b_i + 3);
        c_PoE[b_PoE_tmp] = dv2[3 * b_i];
        c_PoE[6 * b_i + 3] = 0.0;
        PoE_tmp = b_i << 2;
        c_PoE[b_PoE_tmp + 3] = PoE[PoE_tmp];
        c_PoE[b_PoE_tmp + 1] = dv2[3 * b_i + 1];
        c_PoE[6 * b_i + 4] = 0.0;
        c_PoE[b_PoE_tmp + 4] = PoE[PoE_tmp + 1];
        c_PoE[b_PoE_tmp + 2] = dv2[3 * b_i + 2];
        c_PoE[6 * b_i + 5] = 0.0;
        c_PoE[b_PoE_tmp + 5] = PoE[PoE_tmp + 2];
      }

      for (b_i = 0; b_i < 6; b_i++) {
        d = 0.0;
        for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
          d += c_PoE[b_i + 6 * PoE_tmp] * TwMag[PoE_tmp + 7 * (c_i + 1)];
        }

        tmp_data[b_i + 6 * (c_i + 1)] = d;
      }

      for (b_i = 0; b_i < 7; b_i++) {
        b_TwMag[b_i] = TwMag[b_i + 7 * (c_i + 1)];
      }

      expScrew(b_TwMag, dv1);
      for (b_i = 0; b_i < 4; b_i++) {
        d = PoE[b_i];
        d1 = PoE[b_i + 4];
        d2 = PoE[b_i + 8];
        d3 = PoE[b_i + 12];
        for (PoE_tmp = 0; PoE_tmp < 4; PoE_tmp++) {
          b_PoE_tmp = PoE_tmp << 2;
          b_PoE[b_i + b_PoE_tmp] = ((d * dv1[b_PoE_tmp] + d1 * dv1[b_PoE_tmp + 1])
            + d2 * dv1[b_PoE_tmp + 2]) + d3 * dv1[b_PoE_tmp + 3];
        }
      }

      std::memcpy(&PoE[0], &b_PoE[0], 16U * sizeof(double));
    }

    //
    for (b_i = 0; b_i < tmp_size_idx_1; b_i++) {
      for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
        b_PoE_tmp = PoE_tmp + 6 * b_i;
        JstS[b_PoE_tmp] = tmp_data[b_PoE_tmp];
      }
    }

    for (b_i = 0; b_i < 6; b_i++) {
      d = 0.0;
      for (PoE_tmp = 0; PoE_tmp < 6; PoE_tmp++) {
        d += JstS[PoE_tmp + 6 * b_i] * Wrench[PoE_tmp + 6 * i];
      }

      Nt[b_i] -= d;
    }
  }

  //
}

//
// File trailer for NPotentialWre6J.cpp
//
// [EOF]
//
