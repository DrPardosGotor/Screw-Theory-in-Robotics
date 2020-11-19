//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Christoffel.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 15-Nov-2020 13:46:10
//

// Include Files
#include "Christoffel.h"
#include "expScrew.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const double TwMag[49]
//                const double LiMas[49]
//                double i
//                double j
//                double k
// Return Type  : double
//
double Christoffel(const double TwMag[49], const double LiMas[49], double i,
                   double j, double k)
{
  static const signed char AI[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char iv[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char c_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double AdHsli0[36];
  double a[36];
  double b[36];
  double b_AdHsli0[36];
  double b_PoE[36];
  double b_b[36];
  double dv1[36];
  double dv2[36];
  double E1[16];
  double E2[16];
  double Hsli0[16];
  double PoE[16];
  double b_E1[16];
  double b_E2[16];
  double dv[9];
  double wr[9];
  double C[6];
  double c_C[6];
  double y[6];
  double b_C;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double dMt;
  double m;
  double s;
  int b_i;
  int b_j;
  int b_k;
  int c_i;
  int c_j;
  int c_k;
  int d_i;
  int d_k;
  int e_k;
  int f_k;
  int g_k;
  int h_k;
  int i_k;
  int j_k;
  int k_k;
  int l_k;
  int m_k;

  //  "Christoffel" Christoffel Symbols for Coriolis matrix C(t,dt)
  //  for an open chain manipulator.
  //  computation based on the Adjoint Transformation Aij.
  //  Use in SE(3).
  //
  //  	dMt = Christoffel(TwMag,LiMas,I,J,K)
  //
  //  Gives the CHRISTOFFEL SYMBOLS (t,dt) for the Lagangian's equations:
  //  M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
  //  of the dynamics of the robot formed by links on an open chain.
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
  //  I, J, K: index for Cristoffel formulation.
  //
  //  dMt = dMij/dtk = Sum(l=max(i,j),n)[[Aki*Ei,Ek]'*Alk'*Ml*Alj*Ej + Ei'*Ali'*Ml*Alk*[Akj*Ej,Ek]] 
  //
  //  With Ml being the link inertia nxn LinkInertiaS.
  //  With Ei being the twist xi 6x1.
  //  With Aij being an element 6x6 of the adjoint transformation Aij2adjoint
  //
  //  See also: LinkInertiaS, Aij2adjoint,. CCoriolisAij.
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
  //  dMt = Christoffel(TwMag,LiMas,I,J,K)
  //
  if ((i > j) || rtIsNaN(j)) {
    m = i;
  } else {
    m = j;
  }

  dMt = 0.0;
  b_i = static_cast<int>((1.0 - m) + 7.0);
  if (0 <= static_cast<int>((1.0 - m) + 7.0) - 1) {
    c_i = static_cast<int>(i);
    b_k = static_cast<int>(k);
    c_k = static_cast<int>(k);
    e_k = static_cast<int>(k);
    f_k = static_cast<int>(k);
    g_k = static_cast<int>(k);
    h_k = static_cast<int>(k);
    b_j = static_cast<int>(j);
    i_k = static_cast<int>(k);
    j_k = static_cast<int>(k);
    d_k = static_cast<int>(k);
    k_k = static_cast<int>(k);
    l_k = static_cast<int>(k);
    m_k = static_cast<int>(k);
    d_i = static_cast<int>(i);
    c_j = static_cast<int>(j);
  }

  if (0 <= b_i - 1) {
    Hsli0[0] = 1.0;
    Hsli0[4] = 0.0;
    Hsli0[8] = 0.0;
    Hsli0[1] = 0.0;
    Hsli0[5] = 1.0;
    Hsli0[9] = 0.0;
    Hsli0[2] = 0.0;
    Hsli0[6] = 0.0;
    Hsli0[10] = 1.0;
    Hsli0[3] = 0.0;
    Hsli0[7] = 0.0;
    Hsli0[11] = 0.0;
    Hsli0[15] = 1.0;
    d = TwMag[7 * (b_k - 1) + 3];
    d1 = TwMag[7 * (c_k - 1) + 4];
    d2 = TwMag[7 * (e_k - 1) + 5];
    d3 = TwMag[7 * (i_k - 1) + 3];
    d4 = TwMag[7 * (j_k - 1) + 4];
    d5 = TwMag[7 * (d_k - 1) + 5];
  }

  for (d_k = 0; d_k < b_i; d_k++) {
    double a_tmp;
    double b_a_tmp;
    double c_a_tmp;
    int Hsli0_tmp;
    s = m + static_cast<double>(d_k);

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
    Hsli0_tmp = 7 * (static_cast<int>(s) - 1);
    Hsli0[12] = LiMas[Hsli0_tmp];
    Hsli0[13] = LiMas[Hsli0_tmp + 1];
    Hsli0[14] = LiMas[Hsli0_tmp + 2];

    //
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
    std::memset(&a[0], 0, 36U * sizeof(double));
    if (!(k < i)) {
      if (k == i) {
        for (j_k = 0; j_k < 36; j_k++) {
          a[j_k] = AI[j_k];
        }
      } else {
        expScrew(*(double (*)[7])&TwMag[7 * (static_cast<int>(i + 1.0) - 1)],
                 PoE);
        j_k = static_cast<int>(k + (1.0 - (i + 2.0)));
        for (b_k = 0; b_k < j_k; b_k++) {
          expScrew(*(double (*)[7])&TwMag[7 * (static_cast<int>((i + 2.0) +
                     static_cast<double>(b_k)) - 1)], E1);
          for (e_k = 0; e_k < 4; e_k++) {
            d6 = PoE[e_k];
            a_tmp = PoE[e_k + 4];
            b_a_tmp = PoE[e_k + 8];
            c_a_tmp = PoE[e_k + 12];
            for (i_k = 0; i_k < 4; i_k++) {
              c_k = i_k << 2;
              E2[e_k + c_k] = ((d6 * E1[c_k] + a_tmp * E1[c_k + 1]) + b_a_tmp *
                               E1[c_k + 2]) + c_a_tmp * E1[c_k + 3];
            }
          }

          std::memcpy(&PoE[0], &E2[0], 16U * sizeof(double));
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
        for (j_k = 0; j_k < 36; j_k++) {
          a[j_k] = AI[j_k];
        }

        wr[0] = 0.0;
        wr[3] = -PoE[14];
        wr[6] = PoE[13];
        wr[1] = PoE[14];
        wr[4] = 0.0;
        wr[7] = -PoE[12];
        wr[2] = -PoE[13];
        wr[5] = PoE[12];
        wr[8] = 0.0;
        for (j_k = 0; j_k < 3; j_k++) {
          d6 = wr[j_k];
          a_tmp = wr[j_k + 3];
          b_a_tmp = wr[j_k + 6];
          for (e_k = 0; e_k < 3; e_k++) {
            i_k = e_k << 2;
            dv[j_k + 3 * e_k] = (d6 * PoE[i_k] + a_tmp * PoE[i_k + 1]) + b_a_tmp
              * PoE[i_k + 2];
            b_PoE[e_k + 6 * j_k] = PoE[e_k + (j_k << 2)];
          }
        }

        for (j_k = 0; j_k < 3; j_k++) {
          c_k = 6 * (j_k + 3);
          b_PoE[c_k] = dv[3 * j_k];
          b_PoE[6 * j_k + 3] = 0.0;
          e_k = j_k << 2;
          b_PoE[c_k + 3] = PoE[e_k];
          b_PoE[c_k + 1] = dv[3 * j_k + 1];
          b_PoE[6 * j_k + 4] = 0.0;
          b_PoE[c_k + 4] = PoE[e_k + 1];
          b_PoE[c_k + 2] = dv[3 * j_k + 2];
          b_PoE[6 * j_k + 5] = 0.0;
          b_PoE[c_k + 5] = PoE[e_k + 2];
        }

        coder::mldivide(b_PoE, a);
      }
    }

    //
    //
    for (j_k = 0; j_k < 6; j_k++) {
      d6 = 0.0;
      for (e_k = 0; e_k < 6; e_k++) {
        d6 += a[j_k + 6 * e_k] * TwMag[e_k + 7 * (c_i - 1)];
      }

      y[j_k] = d6;
    }

    //
    //  "twistbracket" Computes the bracket operation to two twists.
    //  Use in SE(3).
    //
    //  	xi = twistbracket(x1,x2)
    //
    //  Returns a twist "xi" from a the application of the bracket operation to
    //  the twists x1 and x2. This operation is a generalization of the cross
    //  product on R3 to vectors in R6.
    //     |v|                              v
    //  xi=| |  = [E1,E2] = [E1^*E2^-E2^*E1^]
    //     |W|
    //  Con ^ wedge or twist2tform operation.
    //  Con v vee or tform2twist operation.
    //
    //  See also: tform2twist, twist2tform.
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
    //  x = twistbracket(x1,x2)
    //
    //  "TWIST2TFORM" Convert a twist "xi" (6x1) to homogeneous matrix "E^" 3x4. 
    //  Use in SE(3).
    //
    //  	tform = twist2tform(xi)
    //
    //  Returns the homogeneous "h" matrix 4x4 from a twixt "xi".
    //     |v|         |W^ v|
    //  xi=| |  =>  E^=|    |  andn W^ = skew(W)
    //     |W|         |0  0|
    //
    //
    //  See also: tform2twist, axis2skew.
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
    //  	tform = twist2tform(xi)
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
    E1[0] = 0.0;
    E1[4] = -y[5];
    E1[8] = y[4];
    E1[12] = y[0];
    E1[1] = y[5];
    E1[5] = 0.0;
    E1[9] = -y[3];
    E1[13] = y[1];
    E1[2] = -y[4];
    E1[6] = y[3];
    E1[10] = 0.0;
    E1[14] = y[2];

    //
    //
    //  "TWIST2TFORM" Convert a twist "xi" (6x1) to homogeneous matrix "E^" 3x4. 
    //  Use in SE(3).
    //
    //  	tform = twist2tform(xi)
    //
    //  Returns the homogeneous "h" matrix 4x4 from a twixt "xi".
    //     |v|         |W^ v|
    //  xi=| |  =>  E^=|    |  andn W^ = skew(W)
    //     |W|         |0  0|
    //
    //
    //  See also: tform2twist, axis2skew.
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
    //  	tform = twist2tform(xi)
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
    E2[0] = 0.0;
    E2[4] = -d2;
    E2[8] = d1;
    E2[12] = TwMag[7 * (f_k - 1)];
    E2[1] = d2;
    E2[5] = 0.0;
    E2[9] = -d;
    E2[13] = TwMag[7 * (g_k - 1) + 1];
    E2[2] = -d1;
    E2[6] = d;
    E2[10] = 0.0;
    E2[14] = TwMag[7 * (h_k - 1) + 2];
    E1[3] = 0.0;
    E2[3] = 0.0;
    E1[7] = 0.0;
    E2[7] = 0.0;
    E1[11] = 0.0;
    E2[11] = 0.0;
    E1[15] = 0.0;
    E2[15] = 0.0;

    //
    for (j_k = 0; j_k < 4; j_k++) {
      for (e_k = 0; e_k < 4; e_k++) {
        i_k = e_k << 2;
        c_k = j_k + i_k;
        b_E2[c_k] = ((E2[j_k] * E1[i_k] + E2[j_k + 4] * E1[i_k + 1]) + E2[j_k +
                     8] * E1[i_k + 2]) + E2[j_k + 12] * E1[i_k + 3];
        b_E1[c_k] = ((E1[j_k] * E2[i_k] + E1[j_k + 4] * E2[i_k + 1]) + E1[j_k +
                     8] * E2[i_k + 2]) + E1[j_k + 12] * E2[i_k + 3];
      }
    }

    for (j_k = 0; j_k < 16; j_k++) {
      b_E1[j_k] -= b_E2[j_k];
    }

    //
    //  "TFORM2TWIST" Convert a matrix "E^" 4x4 into a twist "xi" 6x1.
    //  Use in SE(3).
    //
    //  	xi = tform2twist(tform)
    //
    //  Returns a twixt "xi" from a homogeneous "tform" matrix 4x4.
    //     |v|         |W^ v|
    //  xi=| |  <=  E^=|    |
    //     |W|         |0  0|
    //
    //
    //  See also: twist2tform, skew2axis.
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
    //  	xi = tform2twist(tform)
    //
    //  "skew2axis" Generate an axis from an skew symmetric matrix.
    //  Use in SO(3).
    //
    //  	w = skew2axis(r)
    //
    //  Returns a vector 3x1 w[a1;a2;a3;] from a skew symmetric matrix r 3x3 .
    //     |0  -a3  a2|
    //  r =|a3   0 -a1|
    //     |-a2 a1   0|
    //
    //  See also: axis2skew.
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
    //  w = skew2axis(r)
    //
    //
    //
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
    std::memset(&dv1[0], 0, 36U * sizeof(double));
    if (!(s < k)) {
      if (s == k) {
        for (j_k = 0; j_k < 36; j_k++) {
          dv1[j_k] = AI[j_k];
        }
      } else {
        expScrew(*(double (*)[7])&TwMag[7 * static_cast<int>(k)], PoE);
        j_k = static_cast<int>(static_cast<float>(s) + (1.0F - (static_cast<
          float>(k) + 2.0F)));
        for (b_k = 0; b_k < j_k; b_k++) {
          expScrew(*(double (*)[7])&TwMag[7 * ((static_cast<int>(k) + b_k) + 1)],
                   E1);
          for (e_k = 0; e_k < 4; e_k++) {
            d6 = PoE[e_k];
            a_tmp = PoE[e_k + 4];
            b_a_tmp = PoE[e_k + 8];
            c_a_tmp = PoE[e_k + 12];
            for (i_k = 0; i_k < 4; i_k++) {
              c_k = i_k << 2;
              E2[e_k + c_k] = ((d6 * E1[c_k] + a_tmp * E1[c_k + 1]) + b_a_tmp *
                               E1[c_k + 2]) + c_a_tmp * E1[c_k + 3];
            }
          }

          std::memcpy(&PoE[0], &E2[0], 16U * sizeof(double));
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
        for (j_k = 0; j_k < 36; j_k++) {
          dv1[j_k] = AI[j_k];
        }

        wr[0] = 0.0;
        wr[3] = -PoE[14];
        wr[6] = PoE[13];
        wr[1] = PoE[14];
        wr[4] = 0.0;
        wr[7] = -PoE[12];
        wr[2] = -PoE[13];
        wr[5] = PoE[12];
        wr[8] = 0.0;
        for (j_k = 0; j_k < 3; j_k++) {
          d6 = wr[j_k];
          a_tmp = wr[j_k + 3];
          b_a_tmp = wr[j_k + 6];
          for (e_k = 0; e_k < 3; e_k++) {
            i_k = e_k << 2;
            dv[j_k + 3 * e_k] = (d6 * PoE[i_k] + a_tmp * PoE[i_k + 1]) + b_a_tmp
              * PoE[i_k + 2];
            b_PoE[e_k + 6 * j_k] = PoE[e_k + (j_k << 2)];
          }
        }

        for (j_k = 0; j_k < 3; j_k++) {
          c_k = 6 * (j_k + 3);
          b_PoE[c_k] = dv[3 * j_k];
          b_PoE[6 * j_k + 3] = 0.0;
          e_k = j_k << 2;
          b_PoE[c_k + 3] = PoE[e_k];
          b_PoE[c_k + 1] = dv[3 * j_k + 1];
          b_PoE[6 * j_k + 4] = 0.0;
          b_PoE[c_k + 4] = PoE[e_k + 1];
          b_PoE[c_k + 2] = dv[3 * j_k + 2];
          b_PoE[6 * j_k + 5] = 0.0;
          b_PoE[c_k + 5] = PoE[e_k + 2];
        }

        coder::mldivide(b_PoE, dv1);
      }
    }

    //
    //
    //
    //  "linkInertiaS" the LINK TRANSFORMED INERTIA MATRIX for robot's link.
    //  Use in SE(3).
    //
    //  	ImS = LinkInertiaS(Hsli0,Ii,mi)
    //
    //  It gives the LINK TRANSFORMED INERTIA MATRIX corresponding to the inertia 
    //  of the link into the base frame of the manipulator.
    //
    //  INPUTS:
    //  Hsli0 is the homogeneous matrix 4x4 for the pose of the i link frame "L",  
    //  associated to the Center o Mass, at the reference (home) configuration
    //  of the manipulator.
    //  Ii: Inertia Tensor (3x1) = [Ixi; Iyi; Izi] are the moments of
    //  inertia about the x, y, and z-axes of the ith link frame on the CM.
    //  mi: link mass.
    //
    //  the Inertia matrix "ImS" is nxn with n the number of links (joints).
    //  (Ad(Hsl0)^-1)*
    //  AdHsl0 = Ad(Hsl0^-1)= Adjoint transformation of the inverse of the
    //  center of mass ref config.
    //                       |mI  0|
    //  ImS = (Ad(Hsl0)^-1)'*|     |*(Ad(Hsl0)^-1)=(Ad(Hsl0)^-1)'*M*(Ad(Hsl0)^-1) 
    //                       |0  Ii|
    //  ImS = Link Transformed Inertia Matrix.
    //  with M Generalized Inertia Matrix, defined through the diagonal mass and 
    //  the inertia tensor.
    //
    //  See also: tform2adjoint.
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
    //  ImS = LinkInertiaS(Hsli0,Ii,mi)
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
    //
    for (j_k = 0; j_k < 36; j_k++) {
      AdHsli0[j_k] = iv[j_k];
    }

    wr[0] = 0.0;
    wr[3] = -Hsli0[14];
    wr[6] = Hsli0[13];
    wr[1] = Hsli0[14];
    wr[4] = 0.0;
    wr[7] = -Hsli0[12];
    wr[2] = -Hsli0[13];
    wr[5] = Hsli0[12];
    wr[8] = 0.0;
    for (j_k = 0; j_k < 3; j_k++) {
      d6 = wr[j_k];
      a_tmp = wr[j_k + 3];
      b_a_tmp = wr[j_k + 6];
      for (e_k = 0; e_k < 3; e_k++) {
        i_k = e_k << 2;
        dv[j_k + 3 * e_k] = (d6 * Hsli0[i_k] + a_tmp * Hsli0[i_k + 1]) + b_a_tmp
          * Hsli0[i_k + 2];
        a[e_k + 6 * j_k] = Hsli0[e_k + (j_k << 2)];
      }
    }

    for (j_k = 0; j_k < 3; j_k++) {
      c_k = 6 * (j_k + 3);
      a[c_k] = dv[3 * j_k];
      a[6 * j_k + 3] = 0.0;
      e_k = j_k << 2;
      a[c_k + 3] = Hsli0[e_k];
      a[c_k + 1] = dv[3 * j_k + 1];
      a[6 * j_k + 4] = 0.0;
      a[c_k + 4] = Hsli0[e_k + 1];
      a[c_k + 2] = dv[3 * j_k + 2];
      a[6 * j_k + 5] = 0.0;
      a[c_k + 5] = Hsli0[e_k + 2];
    }

    coder::mldivide(a, AdHsli0);

    //  Ad(Hsli0)^-1
    //
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
    std::memset(&b[0], 0, 36U * sizeof(double));
    if (!(s < j)) {
      if (s == j) {
        for (j_k = 0; j_k < 36; j_k++) {
          b[j_k] = AI[j_k];
        }
      } else {
        expScrew(*(double (*)[7])&TwMag[7 * (static_cast<int>(j + 1.0) - 1)],
                 PoE);
        j_k = static_cast<int>(s + (1.0 - (j + 2.0)));
        for (b_k = 0; b_k < j_k; b_k++) {
          expScrew(*(double (*)[7])&TwMag[7 * (static_cast<int>((j + 2.0) +
                     static_cast<double>(b_k)) - 1)], E1);
          for (e_k = 0; e_k < 4; e_k++) {
            d6 = PoE[e_k];
            a_tmp = PoE[e_k + 4];
            b_a_tmp = PoE[e_k + 8];
            c_a_tmp = PoE[e_k + 12];
            for (i_k = 0; i_k < 4; i_k++) {
              c_k = i_k << 2;
              E2[e_k + c_k] = ((d6 * E1[c_k] + a_tmp * E1[c_k + 1]) + b_a_tmp *
                               E1[c_k + 2]) + c_a_tmp * E1[c_k + 3];
            }
          }

          std::memcpy(&PoE[0], &E2[0], 16U * sizeof(double));
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
        for (j_k = 0; j_k < 36; j_k++) {
          b[j_k] = AI[j_k];
        }

        wr[0] = 0.0;
        wr[3] = -PoE[14];
        wr[6] = PoE[13];
        wr[1] = PoE[14];
        wr[4] = 0.0;
        wr[7] = -PoE[12];
        wr[2] = -PoE[13];
        wr[5] = PoE[12];
        wr[8] = 0.0;
        for (j_k = 0; j_k < 3; j_k++) {
          d6 = wr[j_k];
          a_tmp = wr[j_k + 3];
          b_a_tmp = wr[j_k + 6];
          for (e_k = 0; e_k < 3; e_k++) {
            i_k = e_k << 2;
            dv[j_k + 3 * e_k] = (d6 * PoE[i_k] + a_tmp * PoE[i_k + 1]) + b_a_tmp
              * PoE[i_k + 2];
            b_PoE[e_k + 6 * j_k] = PoE[e_k + (j_k << 2)];
          }
        }

        for (j_k = 0; j_k < 3; j_k++) {
          c_k = 6 * (j_k + 3);
          b_PoE[c_k] = dv[3 * j_k];
          b_PoE[6 * j_k + 3] = 0.0;
          e_k = j_k << 2;
          b_PoE[c_k + 3] = PoE[e_k];
          b_PoE[c_k + 1] = dv[3 * j_k + 1];
          b_PoE[6 * j_k + 4] = 0.0;
          b_PoE[c_k + 4] = PoE[e_k + 1];
          b_PoE[c_k + 2] = dv[3 * j_k + 2];
          b_PoE[6 * j_k + 5] = 0.0;
          b_PoE[c_k + 5] = PoE[e_k + 2];
        }

        coder::mldivide(b_PoE, b);
      }
    }

    //
    //
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
    std::memset(&dv2[0], 0, 36U * sizeof(double));
    if (!(s < i)) {
      if (s == i) {
        for (j_k = 0; j_k < 36; j_k++) {
          dv2[j_k] = AI[j_k];
        }
      } else {
        expScrew(*(double (*)[7])&TwMag[7 * static_cast<int>(i)], PoE);
        j_k = static_cast<int>(static_cast<float>(s) + (1.0F - (static_cast<
          float>(i) + 2.0F)));
        for (b_k = 0; b_k < j_k; b_k++) {
          expScrew(*(double (*)[7])&TwMag[7 * ((static_cast<int>(i) + b_k) + 1)],
                   E1);
          for (e_k = 0; e_k < 4; e_k++) {
            d6 = PoE[e_k];
            a_tmp = PoE[e_k + 4];
            b_a_tmp = PoE[e_k + 8];
            c_a_tmp = PoE[e_k + 12];
            for (i_k = 0; i_k < 4; i_k++) {
              c_k = i_k << 2;
              E2[e_k + c_k] = ((d6 * E1[c_k] + a_tmp * E1[c_k + 1]) + b_a_tmp *
                               E1[c_k + 2]) + c_a_tmp * E1[c_k + 3];
            }
          }

          std::memcpy(&PoE[0], &E2[0], 16U * sizeof(double));
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
        for (j_k = 0; j_k < 36; j_k++) {
          dv2[j_k] = AI[j_k];
        }

        wr[0] = 0.0;
        wr[3] = -PoE[14];
        wr[6] = PoE[13];
        wr[1] = PoE[14];
        wr[4] = 0.0;
        wr[7] = -PoE[12];
        wr[2] = -PoE[13];
        wr[5] = PoE[12];
        wr[8] = 0.0;
        for (j_k = 0; j_k < 3; j_k++) {
          d6 = wr[j_k];
          a_tmp = wr[j_k + 3];
          b_a_tmp = wr[j_k + 6];
          for (e_k = 0; e_k < 3; e_k++) {
            i_k = e_k << 2;
            dv[j_k + 3 * e_k] = (d6 * PoE[i_k] + a_tmp * PoE[i_k + 1]) + b_a_tmp
              * PoE[i_k + 2];
            b_PoE[e_k + 6 * j_k] = PoE[e_k + (j_k << 2)];
          }
        }

        for (j_k = 0; j_k < 3; j_k++) {
          c_k = 6 * (j_k + 3);
          b_PoE[c_k] = dv[3 * j_k];
          b_PoE[6 * j_k + 3] = 0.0;
          e_k = j_k << 2;
          b_PoE[c_k + 3] = PoE[e_k];
          b_PoE[c_k + 1] = dv[3 * j_k + 1];
          b_PoE[6 * j_k + 4] = 0.0;
          b_PoE[c_k + 4] = PoE[e_k + 1];
          b_PoE[c_k + 2] = dv[3 * j_k + 2];
          b_PoE[6 * j_k + 5] = 0.0;
          b_PoE[c_k + 5] = PoE[e_k + 2];
        }

        coder::mldivide(b_PoE, dv2);
      }
    }

    //
    //
    //
    //  "linkInertiaS" the LINK TRANSFORMED INERTIA MATRIX for robot's link.
    //  Use in SE(3).
    //
    //  	ImS = LinkInertiaS(Hsli0,Ii,mi)
    //
    //  It gives the LINK TRANSFORMED INERTIA MATRIX corresponding to the inertia 
    //  of the link into the base frame of the manipulator.
    //
    //  INPUTS:
    //  Hsli0 is the homogeneous matrix 4x4 for the pose of the i link frame "L",  
    //  associated to the Center o Mass, at the reference (home) configuration
    //  of the manipulator.
    //  Ii: Inertia Tensor (3x1) = [Ixi; Iyi; Izi] are the moments of
    //  inertia about the x, y, and z-axes of the ith link frame on the CM.
    //  mi: link mass.
    //
    //  the Inertia matrix "ImS" is nxn with n the number of links (joints).
    //  (Ad(Hsl0)^-1)*
    //  AdHsl0 = Ad(Hsl0^-1)= Adjoint transformation of the inverse of the
    //  center of mass ref config.
    //                       |mI  0|
    //  ImS = (Ad(Hsl0)^-1)'*|     |*(Ad(Hsl0)^-1)=(Ad(Hsl0)^-1)'*M*(Ad(Hsl0)^-1) 
    //                       |0  Ii|
    //  ImS = Link Transformed Inertia Matrix.
    //  with M Generalized Inertia Matrix, defined through the diagonal mass and 
    //  the inertia tensor.
    //
    //  See also: tform2adjoint.
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
    //  ImS = LinkInertiaS(Hsli0,Ii,mi)
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
    //
    for (j_k = 0; j_k < 36; j_k++) {
      b_AdHsli0[j_k] = iv[j_k];
    }

    wr[0] = 0.0;
    wr[3] = -Hsli0[14];
    wr[6] = Hsli0[13];
    wr[1] = Hsli0[14];
    wr[4] = 0.0;
    wr[7] = -Hsli0[12];
    wr[2] = -Hsli0[13];
    wr[5] = Hsli0[12];
    wr[8] = 0.0;
    for (j_k = 0; j_k < 3; j_k++) {
      d6 = wr[j_k];
      a_tmp = wr[j_k + 3];
      b_a_tmp = wr[j_k + 6];
      for (e_k = 0; e_k < 3; e_k++) {
        i_k = e_k << 2;
        dv[j_k + 3 * e_k] = (d6 * Hsli0[i_k] + a_tmp * Hsli0[i_k + 1]) + b_a_tmp
          * Hsli0[i_k + 2];
        a[e_k + 6 * j_k] = Hsli0[e_k + (j_k << 2)];
      }
    }

    for (j_k = 0; j_k < 3; j_k++) {
      c_k = 6 * (j_k + 3);
      a[c_k] = dv[3 * j_k];
      a[6 * j_k + 3] = 0.0;
      e_k = j_k << 2;
      a[c_k + 3] = Hsli0[e_k];
      a[c_k + 1] = dv[3 * j_k + 1];
      a[6 * j_k + 4] = 0.0;
      a[c_k + 4] = Hsli0[e_k + 1];
      a[c_k + 2] = dv[3 * j_k + 2];
      a[6 * j_k + 5] = 0.0;
      a[c_k + 5] = Hsli0[e_k + 2];
    }

    coder::mldivide(a, b_AdHsli0);

    //  Ad(Hsli0)^-1
    //
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
    std::memset(&b_b[0], 0, 36U * sizeof(double));
    if (!(s < k)) {
      if (s == k) {
        for (j_k = 0; j_k < 36; j_k++) {
          b_b[j_k] = AI[j_k];
        }
      } else {
        expScrew(*(double (*)[7])&TwMag[7 * static_cast<int>(k)], PoE);
        j_k = static_cast<int>(static_cast<float>(s) + (1.0F - (static_cast<
          float>(k) + 2.0F)));
        for (b_k = 0; b_k < j_k; b_k++) {
          expScrew(*(double (*)[7])&TwMag[7 * ((static_cast<int>(k) + b_k) + 1)],
                   E1);
          for (e_k = 0; e_k < 4; e_k++) {
            d6 = PoE[e_k];
            a_tmp = PoE[e_k + 4];
            b_a_tmp = PoE[e_k + 8];
            c_a_tmp = PoE[e_k + 12];
            for (i_k = 0; i_k < 4; i_k++) {
              c_k = i_k << 2;
              E2[e_k + c_k] = ((d6 * E1[c_k] + a_tmp * E1[c_k + 1]) + b_a_tmp *
                               E1[c_k + 2]) + c_a_tmp * E1[c_k + 3];
            }
          }

          std::memcpy(&PoE[0], &E2[0], 16U * sizeof(double));
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
        for (j_k = 0; j_k < 36; j_k++) {
          b_b[j_k] = AI[j_k];
        }

        wr[0] = 0.0;
        wr[3] = -PoE[14];
        wr[6] = PoE[13];
        wr[1] = PoE[14];
        wr[4] = 0.0;
        wr[7] = -PoE[12];
        wr[2] = -PoE[13];
        wr[5] = PoE[12];
        wr[8] = 0.0;
        for (j_k = 0; j_k < 3; j_k++) {
          d6 = wr[j_k];
          a_tmp = wr[j_k + 3];
          b_a_tmp = wr[j_k + 6];
          for (e_k = 0; e_k < 3; e_k++) {
            i_k = e_k << 2;
            dv[j_k + 3 * e_k] = (d6 * PoE[i_k] + a_tmp * PoE[i_k + 1]) + b_a_tmp
              * PoE[i_k + 2];
            b_PoE[e_k + 6 * j_k] = PoE[e_k + (j_k << 2)];
          }
        }

        for (j_k = 0; j_k < 3; j_k++) {
          c_k = 6 * (j_k + 3);
          b_PoE[c_k] = dv[3 * j_k];
          b_PoE[6 * j_k + 3] = 0.0;
          e_k = j_k << 2;
          b_PoE[c_k + 3] = PoE[e_k];
          b_PoE[c_k + 1] = dv[3 * j_k + 1];
          b_PoE[6 * j_k + 4] = 0.0;
          b_PoE[c_k + 4] = PoE[e_k + 1];
          b_PoE[c_k + 2] = dv[3 * j_k + 2];
          b_PoE[6 * j_k + 5] = 0.0;
          b_PoE[c_k + 5] = PoE[e_k + 2];
        }

        coder::mldivide(b_PoE, b_b);
      }
    }

    //
    //
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
    std::memset(&a[0], 0, 36U * sizeof(double));
    if (!(k < j)) {
      if (k == j) {
        for (j_k = 0; j_k < 36; j_k++) {
          a[j_k] = AI[j_k];
        }
      } else {
        expScrew(*(double (*)[7])&TwMag[7 * static_cast<int>(j)], PoE);
        j_k = static_cast<int>(static_cast<float>(k) + (1.0F - (static_cast<
          float>(j) + 2.0F)));
        for (b_k = 0; b_k < j_k; b_k++) {
          expScrew(*(double (*)[7])&TwMag[7 * ((static_cast<int>(j) + b_k) + 1)],
                   E1);
          for (e_k = 0; e_k < 4; e_k++) {
            d6 = PoE[e_k];
            a_tmp = PoE[e_k + 4];
            b_a_tmp = PoE[e_k + 8];
            c_a_tmp = PoE[e_k + 12];
            for (i_k = 0; i_k < 4; i_k++) {
              c_k = i_k << 2;
              E2[e_k + c_k] = ((d6 * E1[c_k] + a_tmp * E1[c_k + 1]) + b_a_tmp *
                               E1[c_k + 2]) + c_a_tmp * E1[c_k + 3];
            }
          }

          std::memcpy(&PoE[0], &E2[0], 16U * sizeof(double));
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
        for (j_k = 0; j_k < 36; j_k++) {
          a[j_k] = AI[j_k];
        }

        wr[0] = 0.0;
        wr[3] = -PoE[14];
        wr[6] = PoE[13];
        wr[1] = PoE[14];
        wr[4] = 0.0;
        wr[7] = -PoE[12];
        wr[2] = -PoE[13];
        wr[5] = PoE[12];
        wr[8] = 0.0;
        for (j_k = 0; j_k < 3; j_k++) {
          d6 = wr[j_k];
          a_tmp = wr[j_k + 3];
          b_a_tmp = wr[j_k + 6];
          for (e_k = 0; e_k < 3; e_k++) {
            i_k = e_k << 2;
            dv[j_k + 3 * e_k] = (d6 * PoE[i_k] + a_tmp * PoE[i_k + 1]) + b_a_tmp
              * PoE[i_k + 2];
            b_PoE[e_k + 6 * j_k] = PoE[e_k + (j_k << 2)];
          }
        }

        for (j_k = 0; j_k < 3; j_k++) {
          c_k = 6 * (j_k + 3);
          b_PoE[c_k] = dv[3 * j_k];
          b_PoE[6 * j_k + 3] = 0.0;
          e_k = j_k << 2;
          b_PoE[c_k + 3] = PoE[e_k];
          b_PoE[c_k + 1] = dv[3 * j_k + 1];
          b_PoE[6 * j_k + 4] = 0.0;
          b_PoE[c_k + 4] = PoE[e_k + 1];
          b_PoE[c_k + 2] = dv[3 * j_k + 2];
          b_PoE[6 * j_k + 5] = 0.0;
          b_PoE[c_k + 5] = PoE[e_k + 2];
        }

        coder::mldivide(b_PoE, a);
      }
    }

    //
    //
    for (j_k = 0; j_k < 6; j_k++) {
      d6 = 0.0;
      for (e_k = 0; e_k < 6; e_k++) {
        d6 += a[j_k + 6 * e_k] * TwMag[e_k + 7 * (b_j - 1)];
      }

      y[j_k] = d6;
    }

    //
    //  "twistbracket" Computes the bracket operation to two twists.
    //  Use in SE(3).
    //
    //  	xi = twistbracket(x1,x2)
    //
    //  Returns a twist "xi" from a the application of the bracket operation to
    //  the twists x1 and x2. This operation is a generalization of the cross
    //  product on R3 to vectors in R6.
    //     |v|                              v
    //  xi=| |  = [E1,E2] = [E1^*E2^-E2^*E1^]
    //     |W|
    //  Con ^ wedge or twist2tform operation.
    //  Con v vee or tform2twist operation.
    //
    //  See also: tform2twist, twist2tform.
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
    //  x = twistbracket(x1,x2)
    //
    //  "TWIST2TFORM" Convert a twist "xi" (6x1) to homogeneous matrix "E^" 3x4. 
    //  Use in SE(3).
    //
    //  	tform = twist2tform(xi)
    //
    //  Returns the homogeneous "h" matrix 4x4 from a twixt "xi".
    //     |v|         |W^ v|
    //  xi=| |  =>  E^=|    |  andn W^ = skew(W)
    //     |W|         |0  0|
    //
    //
    //  See also: tform2twist, axis2skew.
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
    //  	tform = twist2tform(xi)
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
    E1[0] = 0.0;
    E1[4] = -y[5];
    E1[8] = y[4];
    E1[12] = y[0];
    E1[1] = y[5];
    E1[5] = 0.0;
    E1[9] = -y[3];
    E1[13] = y[1];
    E1[2] = -y[4];
    E1[6] = y[3];
    E1[10] = 0.0;
    E1[14] = y[2];

    //
    //
    //  "TWIST2TFORM" Convert a twist "xi" (6x1) to homogeneous matrix "E^" 3x4. 
    //  Use in SE(3).
    //
    //  	tform = twist2tform(xi)
    //
    //  Returns the homogeneous "h" matrix 4x4 from a twixt "xi".
    //     |v|         |W^ v|
    //  xi=| |  =>  E^=|    |  andn W^ = skew(W)
    //     |W|         |0  0|
    //
    //
    //  See also: tform2twist, axis2skew.
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
    //  	tform = twist2tform(xi)
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
    E2[0] = 0.0;
    E2[4] = -d5;
    E2[8] = d4;
    E2[12] = TwMag[7 * (k_k - 1)];
    E2[1] = d5;
    E2[5] = 0.0;
    E2[9] = -d3;
    E2[13] = TwMag[7 * (l_k - 1) + 1];
    E2[2] = -d4;
    E2[6] = d3;
    E2[10] = 0.0;
    E2[14] = TwMag[7 * (m_k - 1) + 2];
    E1[3] = 0.0;
    E2[3] = 0.0;
    E1[7] = 0.0;
    E2[7] = 0.0;
    E1[11] = 0.0;
    E2[11] = 0.0;
    E1[15] = 0.0;
    E2[15] = 0.0;

    //
    for (j_k = 0; j_k < 4; j_k++) {
      for (e_k = 0; e_k < 4; e_k++) {
        i_k = e_k << 2;
        c_k = j_k + i_k;
        b_E2[c_k] = ((E2[j_k] * E1[i_k] + E2[j_k + 4] * E1[i_k + 1]) + E2[j_k +
                     8] * E1[i_k + 2]) + E2[j_k + 12] * E1[i_k + 3];
        PoE[c_k] = ((E1[j_k] * E2[i_k] + E1[j_k + 4] * E2[i_k + 1]) + E1[j_k + 8]
                    * E2[i_k + 2]) + E1[j_k + 12] * E2[i_k + 3];
      }
    }

    for (j_k = 0; j_k < 16; j_k++) {
      PoE[j_k] -= b_E2[j_k];
    }

    //
    //  "TFORM2TWIST" Convert a matrix "E^" 4x4 into a twist "xi" 6x1.
    //  Use in SE(3).
    //
    //  	xi = tform2twist(tform)
    //
    //  Returns a twixt "xi" from a homogeneous "tform" matrix 4x4.
    //     |v|         |W^ v|
    //  xi=| |  <=  E^=|    |
    //     |W|         |0  0|
    //
    //
    //  See also: twist2tform, skew2axis.
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
    //  	xi = tform2twist(tform)
    //
    //  "skew2axis" Generate an axis from an skew symmetric matrix.
    //  Use in SO(3).
    //
    //  	w = skew2axis(r)
    //
    //  Returns a vector 3x1 w[a1;a2;a3;] from a skew symmetric matrix r 3x3 .
    //     |0  -a3  a2|
    //  r =|a3   0 -a1|
    //     |-a2 a1   0|
    //
    //  See also: axis2skew.
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
    //  w = skew2axis(r)
    //
    //
    //
    y[0] = b_E1[12];
    y[1] = b_E1[13];
    y[2] = b_E1[14];
    y[3] = b_E1[6];
    y[4] = b_E1[8];
    y[5] = b_E1[1];
    for (c_k = 0; c_k < 6; c_k++) {
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        s += y[b_k] * dv1[b_k * 6 + c_k];
      }

      C[c_k] = s;
    }

    for (c_k = 0; c_k < 6; c_k++) {
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        s += TwMag[b_k + 7 * (d_i - 1)] * dv2[b_k * 6 + c_k];
      }

      y[c_k] = s;
    }

    s = LiMas[Hsli0_tmp + 6];
    for (j_k = 0; j_k < 3; j_k++) {
      a[6 * j_k] = s * static_cast<double>(c_b[3 * j_k]);
      c_k = 6 * (j_k + 3);
      a[c_k] = 0.0;
      a[6 * j_k + 3] = 0.0;
      a[6 * j_k + 1] = s * static_cast<double>(c_b[3 * j_k + 1]);
      a[c_k + 1] = 0.0;
      a[6 * j_k + 4] = 0.0;
      a[6 * j_k + 2] = s * static_cast<double>(c_b[3 * j_k + 2]);
      a[c_k + 2] = 0.0;
      a[6 * j_k + 5] = 0.0;
    }

    a_tmp = LiMas[Hsli0_tmp + 3];
    a[21] = a_tmp;
    a[27] = 0.0;
    a[33] = 0.0;
    a[22] = 0.0;
    b_a_tmp = LiMas[Hsli0_tmp + 4];
    a[28] = b_a_tmp;
    a[34] = 0.0;
    a[23] = 0.0;
    a[29] = 0.0;
    c_a_tmp = LiMas[Hsli0_tmp + 5];
    a[35] = c_a_tmp;
    for (j_k = 0; j_k < 6; j_k++) {
      for (e_k = 0; e_k < 6; e_k++) {
        d6 = 0.0;
        for (i_k = 0; i_k < 6; i_k++) {
          d6 += AdHsli0[i_k + 6 * j_k] * a[i_k + 6 * e_k];
        }

        b_PoE[j_k + 6 * e_k] = d6;
      }
    }

    for (j_k = 0; j_k < 6; j_k++) {
      for (e_k = 0; e_k < 6; e_k++) {
        d6 = 0.0;
        for (i_k = 0; i_k < 6; i_k++) {
          d6 += b_PoE[j_k + 6 * i_k] * AdHsli0[i_k + 6 * e_k];
        }

        a[j_k + 6 * e_k] = d6;
      }
    }

    for (j_k = 0; j_k < 6; j_k++) {
      d6 = 0.0;
      for (e_k = 0; e_k < 6; e_k++) {
        d6 += C[e_k] * a[e_k + 6 * j_k];
      }

      c_C[j_k] = d6;
    }

    b_C = 0.0;
    for (j_k = 0; j_k < 6; j_k++) {
      d6 = 0.0;
      for (e_k = 0; e_k < 6; e_k++) {
        d6 += c_C[e_k] * b[e_k + 6 * j_k];
      }

      b_C += d6 * TwMag[j_k + 7 * (c_j - 1)];
    }

    for (j_k = 0; j_k < 3; j_k++) {
      a[6 * j_k] = s * static_cast<double>(c_b[3 * j_k]);
      c_k = 6 * (j_k + 3);
      a[c_k] = 0.0;
      a[6 * j_k + 3] = 0.0;
      a[6 * j_k + 1] = s * static_cast<double>(c_b[3 * j_k + 1]);
      a[c_k + 1] = 0.0;
      a[6 * j_k + 4] = 0.0;
      a[6 * j_k + 2] = s * static_cast<double>(c_b[3 * j_k + 2]);
      a[c_k + 2] = 0.0;
      a[6 * j_k + 5] = 0.0;
    }

    a[21] = a_tmp;
    a[27] = 0.0;
    a[33] = 0.0;
    a[22] = 0.0;
    a[28] = b_a_tmp;
    a[34] = 0.0;
    a[23] = 0.0;
    a[29] = 0.0;
    a[35] = c_a_tmp;
    for (j_k = 0; j_k < 6; j_k++) {
      for (e_k = 0; e_k < 6; e_k++) {
        d6 = 0.0;
        for (i_k = 0; i_k < 6; i_k++) {
          d6 += b_AdHsli0[i_k + 6 * j_k] * a[i_k + 6 * e_k];
        }

        AdHsli0[j_k + 6 * e_k] = d6;
      }

      for (e_k = 0; e_k < 6; e_k++) {
        d6 = 0.0;
        for (i_k = 0; i_k < 6; i_k++) {
          d6 += AdHsli0[j_k + 6 * i_k] * b_AdHsli0[i_k + 6 * e_k];
        }

        b_PoE[j_k + 6 * e_k] = d6;
      }
    }

    for (j_k = 0; j_k < 6; j_k++) {
      d6 = 0.0;
      for (e_k = 0; e_k < 6; e_k++) {
        d6 += y[e_k] * b_PoE[e_k + 6 * j_k];
      }

      C[j_k] = d6;
    }

    y[0] = PoE[12];
    y[1] = PoE[13];
    y[2] = PoE[14];
    y[3] = PoE[6];
    y[4] = PoE[8];
    y[5] = PoE[1];
    s = 0.0;
    for (j_k = 0; j_k < 6; j_k++) {
      d6 = 0.0;
      for (e_k = 0; e_k < 6; e_k++) {
        d6 += C[e_k] * b_b[e_k + 6 * j_k];
      }

      s += d6 * y[j_k];
    }

    dMt = (dMt + b_C) + s;
  }

  //
  return dMt;
}

//
// File trailer for Christoffel.cpp
//
// [EOF]
//
