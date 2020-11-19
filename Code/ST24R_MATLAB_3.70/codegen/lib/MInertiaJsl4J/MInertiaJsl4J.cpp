//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MInertiaJsl4J.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 14-Nov-2020 15:58:41
//

// Include Files
#include "MInertiaJsl4J.h"
#include "expScrew.h"
#include "xgetrf.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const double TwMag[28]
//                const double LiMas[28]
//                double Mt[16]
// Return Type  : void
//
void MInertiaJsl4J(const double TwMag[28], const double LiMas[28], double Mt[16])
{
  static const signed char AI[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double A[36];
  double AZ[36];
  double AdHsli0[36];
  double JslT[24];
  double b_JslT[24];
  double Hsli0[16];
  double b_Hsli0[16];
  double dv2[16];
  double dv[9];
  double dv1[9];
  double X[6];
  double temp;
  int ipiv[6];
  int jBcol;

  //  "MInertiaL" INERTIA MATRIX M(t) for an open chain manipulator.
  //  computation based on the use of the JslT LINK TOOL Jacobian (mobil).
  //  Use in SE(3).
  //
  //  	Mt = MInertiaJsl(TwMag,LiMas)
  //
  //  Gives the MANIPULATOR INERTIA MATRIX M corresponding to the Lagangian's
  //  dynamics equations: M(t)
  //  M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
  //  of the robot formed by links on an open chain.
  //
  //  INPUTS:
  //  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
  //  for each rigid body joint (1..n).
  //  Tw1..Twn: The TWIST components for the joint movement.
  //  Mag1..Magn: The MAGNITUDE component for the joint SCREW movement.
  //  LiMas = [CM1; IT1; Mas1, ..., CMn; ITn; Masn] (7xn)
  //  for each rigid body link (1..n).
  //  CM1..CMn: Center of Mass x-y-z Position components to S for each Link.
  //  IT1..ITn: Inertia x-y-z components for each Link refered to its CM.
  //  Mas1..Masn: The Mass for each Link.
  //
  //  OUTPUT:
  //       |M11...M1n|                          i=n
  //  Mt = |         | = MJslT(t) = Mij(t) = Sum   JslT' * Mi * JslT
  //       |Mn1...Mnn|                          i=1
  //  Where JslT is the LINK TOOL Jacobian for each link.
  //  Where Mi is the LINK JACOBIAN Inertia Matrix of the Robot. It is diagonal
  //  Tensor (6x6) with the mass of the link mi in a tensor (3x3) and another
  //  (3x3)for the inertias Iix, Iiy and Iiz, which are the moments of
  //  inertia about the x, y, and z-axes of the ith link frame on the CMi.
  //
  //  See also: GeoJacobianL.
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
  //  Mt = MInertiaJsl(TwMag,LiMas)
  //
  std::memset(&Mt[0], 0, 16U * sizeof(double));
  for (int i = 0; i < 4; i++) {
    double d;
    double d1;
    int b_i;
    int b_temp;
    int i1;

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
    Hsli0[0] = 1.0;
    Hsli0[4] = 0.0;
    Hsli0[8] = 0.0;
    Hsli0[12] = LiMas[7 * i];
    Hsli0[1] = 0.0;
    Hsli0[5] = 1.0;
    Hsli0[9] = 0.0;
    Hsli0[13] = LiMas[7 * i + 1];
    Hsli0[2] = 0.0;
    Hsli0[6] = 0.0;
    Hsli0[10] = 1.0;
    Hsli0[14] = LiMas[7 * i + 2];
    Hsli0[3] = 0.0;
    Hsli0[7] = 0.0;
    Hsli0[11] = 0.0;
    Hsli0[15] = 1.0;

    //
    //
    //  "GeoJacobianT" computes Geometric LINK TOOL Jacobian
    //  the coordinate system is refered to the Link frame.
    //  Use in SE(3).
    //
    //  	JslT = GeoJacobianL(TwMag,Hsl0,Li)
    //
    //  GEOMETRIC JACOBIAN in a LINK frame: At each configuration of theta,
    //  maps the joint velocity vector, into the corresponding velocity of some
    //  robot LINK.
    //  The "ith" column of jst is the "ith" joint twist, written with respect
    //  to the LINK frame at to the current manipulator configuration.
    //
    //  INPUTS:
    //  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
    //  for each rigid body joint (link 1..n).
    //  Tw1..Twn: The TWIST components for the joint SCREW movement.
    //  Mag1..Magn: The MAGNITUDE component for the joint SCREW movement.
    //  Hsl0 is the pose of the link frame "L", associated to the Center o Mass
    //  of that link at the reference (home) configuration or the manipulator.
    //  Li: link number.
    //
    //  JslL = (Ad(Hsl0)^-1)*[Ai1*E1 ... Aii*Ei 0 ... 0] (6xn)
    //  where Aij = Aij2Adjoint and En are the Twists.
    //
    //  See also: GeoJacobianS, GeoJacobianT, Aij2adjoint, tform2adjoint.
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
    //  JslT = GeoJacobianL(TwMag,Hsli0,Li)
    //
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
    dv[0] = 0.0;
    dv[3] = -Hsli0[14];
    dv[6] = Hsli0[13];
    dv[1] = Hsli0[14];
    dv[4] = 0.0;
    dv[7] = -Hsli0[12];
    dv[2] = -Hsli0[13];
    dv[5] = Hsli0[12];
    dv[8] = 0.0;
    for (b_i = 0; b_i < 3; b_i++) {
      temp = dv[b_i];
      d = dv[b_i + 3];
      d1 = dv[b_i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        jBcol = i1 << 2;
        dv1[b_i + 3 * i1] = (temp * Hsli0[jBcol] + d * Hsli0[jBcol + 1]) + d1 *
          Hsli0[jBcol + 2];
        AdHsli0[i1 + 6 * b_i] = Hsli0[i1 + (b_i << 2)];
      }
    }

    for (b_i = 0; b_i < 3; b_i++) {
      jBcol = 6 * (b_i + 3);
      AdHsli0[jBcol] = dv1[3 * b_i];
      AdHsli0[6 * b_i + 3] = 0.0;
      b_temp = b_i << 2;
      AdHsli0[jBcol + 3] = Hsli0[b_temp];
      AdHsli0[jBcol + 1] = dv1[3 * b_i + 1];
      AdHsli0[6 * b_i + 4] = 0.0;
      AdHsli0[jBcol + 4] = Hsli0[b_temp + 1];
      AdHsli0[jBcol + 2] = dv1[3 * b_i + 2];
      AdHsli0[6 * b_i + 5] = 0.0;
      AdHsli0[jBcol + 5] = Hsli0[b_temp + 2];
    }

    //
    for (int j = 0; j < 4; j++) {
      int c_i;
      int k;

      //
      //  "AIJ2ADJOINT" Computes ADJOINT TRANSFORMATION for a list of twists-mag. 
      //  Use in SE(3).
      //  Notation useful for Link Jacobian (mobile).
      //  Notation useful for Christofell Symbols.
      //  Use in SE(3).
      //
      //  	Ad = Aij2adjoint(i,j,TwMag)
      //
      //  INPUTS:
      //  TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
      //  for each rigid body joint (link 1..n).
      //  Twn1..Twn6: The TWIST components for the joint SCREW movement.
      //  Magn: The MAGNITUDE component for the joint SCREW movement.
      //
      //  ADJOINT TRANSFORMATION: This is a special notation which gives us a most  
      //  form of the Adjoint of an open chain manipulator
      //  We use this notation for an easy calculation of the Manipulator Inertia  
      //  Matrix and the Manipulator Coriolis Matrix.
      //  Computes the Adg in R^6 (6x6 matrix) from any robot link.
      //       I                                    if i=j
      //  Aij= Ad^-1[(exp(Ej+1,Tj+1)...(exp(Ei,Ti)] if i>j
      //       0                                    if i<j
      //
      //  See also: tform2adjoint, expScrew.
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
      //  	Ad = Aij2adjoint(i,j,TwMag)
      //
      std::memset(&AZ[0], 0, 36U * sizeof(double));
      if (i >= j) {
        if (i == j) {
          for (b_i = 0; b_i < 36; b_i++) {
            AZ[b_i] = AI[b_i];
          }
        } else {
          int AZ_tmp;
          int b_j;
          expScrew(*(double (*)[7])&TwMag[7 * (j + 1)], Hsli0);
          b_i = i - j;
          for (k = 0; k <= b_i - 2; k++) {
            expScrew(*(double (*)[7])&TwMag[7 * ((j + k) + 2)], dv2);
            for (i1 = 0; i1 < 4; i1++) {
              double d2;
              temp = Hsli0[i1];
              d = Hsli0[i1 + 4];
              d1 = Hsli0[i1 + 8];
              d2 = Hsli0[i1 + 12];
              for (jBcol = 0; jBcol < 4; jBcol++) {
                b_temp = jBcol << 2;
                b_Hsli0[i1 + b_temp] = ((temp * dv2[b_temp] + d * dv2[b_temp + 1])
                  + d1 * dv2[b_temp + 2]) + d2 * dv2[b_temp + 3];
              }
            }

            std::memcpy(&Hsli0[0], &b_Hsli0[0], 16U * sizeof(double));
          }

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
          dv[0] = 0.0;
          dv[3] = -Hsli0[14];
          dv[6] = Hsli0[13];
          dv[1] = Hsli0[14];
          dv[4] = 0.0;
          dv[7] = -Hsli0[12];
          dv[2] = -Hsli0[13];
          dv[5] = Hsli0[12];
          dv[8] = 0.0;
          for (b_i = 0; b_i < 3; b_i++) {
            temp = dv[b_i];
            d = dv[b_i + 3];
            d1 = dv[b_i + 6];
            for (i1 = 0; i1 < 3; i1++) {
              jBcol = i1 << 2;
              dv1[b_i + 3 * i1] = (temp * Hsli0[jBcol] + d * Hsli0[jBcol + 1]) +
                d1 * Hsli0[jBcol + 2];
              A[i1 + 6 * b_i] = Hsli0[i1 + (b_i << 2)];
            }
          }

          for (b_i = 0; b_i < 3; b_i++) {
            jBcol = 6 * (b_i + 3);
            A[jBcol] = dv1[3 * b_i];
            A[6 * b_i + 3] = 0.0;
            b_temp = b_i << 2;
            A[jBcol + 3] = Hsli0[b_temp];
            A[jBcol + 1] = dv1[3 * b_i + 1];
            A[6 * b_i + 4] = 0.0;
            A[jBcol + 4] = Hsli0[b_temp + 1];
            A[jBcol + 2] = dv1[3 * b_i + 2];
            A[6 * b_i + 5] = 0.0;
            A[jBcol + 5] = Hsli0[b_temp + 2];
          }

          coder::internal::lapack::xgetrf(A, ipiv, &jBcol);
          for (b_i = 0; b_i < 36; b_i++) {
            AZ[b_i] = AI[b_i];
          }

          for (c_i = 0; c_i < 5; c_i++) {
            b_i = ipiv[c_i];
            if (b_i != c_i + 1) {
              for (b_j = 0; b_j < 6; b_j++) {
                jBcol = c_i + 6 * b_j;
                b_temp = static_cast<int>(AZ[jBcol]);
                AZ_tmp = (b_i + 6 * b_j) - 1;
                AZ[jBcol] = AZ[AZ_tmp];
                AZ[AZ_tmp] = b_temp;
              }
            }
          }

          for (b_j = 0; b_j < 6; b_j++) {
            jBcol = 6 * b_j;
            for (k = 0; k < 6; k++) {
              b_temp = 6 * k;
              b_i = k + jBcol;
              if (AZ[b_i] != 0.0) {
                i1 = k + 2;
                for (c_i = i1; c_i < 7; c_i++) {
                  AZ_tmp = (c_i + jBcol) - 1;
                  AZ[AZ_tmp] -= AZ[b_i] * A[(c_i + b_temp) - 1];
                }
              }
            }
          }

          for (b_j = 0; b_j < 6; b_j++) {
            jBcol = 6 * b_j;
            for (k = 5; k >= 0; k--) {
              b_temp = 6 * k;
              b_i = k + jBcol;
              temp = AZ[b_i];
              if (temp != 0.0) {
                AZ[b_i] = temp / A[k + b_temp];
                for (c_i = 0; c_i < k; c_i++) {
                  AZ_tmp = c_i + jBcol;
                  AZ[AZ_tmp] -= AZ[b_i] * A[c_i + b_temp];
                }
              }
            }
          }
        }
      }

      //
      //
      for (b_i = 0; b_i < 6; b_i++) {
        temp = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          temp += AZ[b_i + 6 * i1] * TwMag[i1 + 7 * j];
        }

        b_JslT[b_i + 6 * j] = temp;
        X[b_i] = temp;
      }

      std::memcpy(&A[0], &AdHsli0[0], 36U * sizeof(double));
      coder::internal::lapack::xgetrf(A, ipiv, &jBcol);
      for (c_i = 0; c_i < 5; c_i++) {
        if (ipiv[c_i] != c_i + 1) {
          temp = X[c_i];
          X[c_i] = X[ipiv[c_i] - 1];
          X[ipiv[c_i] - 1] = temp;
        }
      }

      for (k = 0; k < 6; k++) {
        b_temp = 6 * k;
        if (X[k] != 0.0) {
          b_i = k + 2;
          for (c_i = b_i; c_i < 7; c_i++) {
            X[c_i - 1] -= X[k] * A[(c_i + b_temp) - 1];
          }
        }
      }

      for (k = 5; k >= 0; k--) {
        b_temp = 6 * k;
        temp = X[k];
        if (temp != 0.0) {
          temp /= A[k + b_temp];
          X[k] = temp;
          for (c_i = 0; c_i < k; c_i++) {
            X[c_i] -= X[k] * A[c_i + b_temp];
          }
        }
      }

      for (b_i = 0; b_i < 6; b_i++) {
        b_JslT[b_i + 6 * j] = X[b_i];
      }
    }

    //
    temp = LiMas[7 * i + 6];
    for (b_i = 0; b_i < 3; b_i++) {
      AdHsli0[6 * b_i] = temp * static_cast<double>(b[3 * b_i]);
      jBcol = 6 * (b_i + 3);
      AdHsli0[jBcol] = 0.0;
      AdHsli0[6 * b_i + 3] = 0.0;
      AdHsli0[6 * b_i + 1] = temp * static_cast<double>(b[3 * b_i + 1]);
      AdHsli0[jBcol + 1] = 0.0;
      AdHsli0[6 * b_i + 4] = 0.0;
      AdHsli0[6 * b_i + 2] = temp * static_cast<double>(b[3 * b_i + 2]);
      AdHsli0[jBcol + 2] = 0.0;
      AdHsli0[6 * b_i + 5] = 0.0;
    }

    AdHsli0[21] = LiMas[7 * i + 3];
    AdHsli0[27] = 0.0;
    AdHsli0[33] = 0.0;
    AdHsli0[22] = 0.0;
    AdHsli0[28] = LiMas[7 * i + 4];
    AdHsli0[34] = 0.0;
    AdHsli0[23] = 0.0;
    AdHsli0[29] = 0.0;
    AdHsli0[35] = LiMas[7 * i + 5];
    for (b_i = 0; b_i < 4; b_i++) {
      for (i1 = 0; i1 < 6; i1++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += b_JslT[jBcol + 6 * b_i] * AdHsli0[jBcol + 6 * i1];
        }

        JslT[b_i + (i1 << 2)] = temp;
      }

      for (i1 = 0; i1 < 4; i1++) {
        temp = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          temp += JslT[b_i + (jBcol << 2)] * b_JslT[jBcol + 6 * i1];
        }

        jBcol = b_i + (i1 << 2);
        Mt[jBcol] += temp;
      }
    }
  }

  //
}

//
// File trailer for MInertiaJsl4J.cpp
//
// [EOF]
//
