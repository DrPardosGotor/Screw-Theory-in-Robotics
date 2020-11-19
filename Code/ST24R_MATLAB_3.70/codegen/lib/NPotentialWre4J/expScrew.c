/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: expScrew.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Nov-2020 16:04:46
 */

/* Include Files */
#include "expScrew.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double TwMag[7]
 *                double H[16]
 * Return Type  : void
 */
void expScrew(const double TwMag[7], double H[16])
{
  double b_ws[9];
  double ws[9];
  double p[3];
  double absxk;
  double scale;
  double t;
  double y;
  int i;
  int k;
  signed char b_I[9];

  /*  EXPSCREW Matrix Exponential of a  Rigid Body Motion by SCREW movement */
  /*  */
  /*  defined by the "Twist-Mangitude" (TwMag) [xi; theta] (7x1) */
  /*  where the TWIST "xi" (6x1) and its MAGNITUDE "theta" */
  /*  into a matrix "H = exp(xi^theta)" (4x4). */
  /*  Use in SE(3). */
  /*  */
  /*    H = expScrew(TwMag) */
  /*  */
  /*  Returns a homogeneous "H" matrix (4x4) */
  /*     |v|  */
  /*  xi=| |    */
  /*     |W|   */
  /*               |exp(W^Th)  (I-exp(W^Th))*(W x v)+W*W'*v*Theta| */
  /*  exp(E^Theta)=|                                             |:if W not 0. */
  /*               |0                          1                 | */
  /*               |exp(W^Th)  (I-exp(W^Th))*(W x v)| */
  /*  exp(E^Theta)=|                                |:if W not 0 and only rot. */
  /*               |0                          1    | */
  /*               |I v*Theta| */
  /*  exp(E^Theta)=|         |: if W is Zero (i.e., translation) */
  /*               |0     1  | */
  /*  exp(E^Theta)=gst(theta)*gst(0)^-1                                             */
  /*  Use Rodrigues's formula:  */
  /*  exp(W^Th)=I + W^ * sin(Th)+ W^ * W^ * (1-cos(Th))  */
  /*  With W^=skew(W), which means W x v == cross product W^ * v. */
  /*  */
  /*  BE AWARE that a value theta = 0 produces a result H=I. */
  /*  */
  /*  See also: expAxAng. */
  /*  */
  /*  Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor. */
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
  /*  Revision 1.1  2020/02/11 00:00:01 */
  /*  General cleanup of code: help comments, see also, copyright */
  /*  references, clarification of functions. */
  /*  */
  /*  	H = expScrew(TwMag) */
  /*  "vee" component of the TWIST. */
  /*  "omega" component of the TWIST. */
  /*  "theta" magnitude component of the SCREW. */
  scale = 3.3121686421112381E-170;
  absxk = fabs(TwMag[3]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(TwMag[4]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(TwMag[5]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  if (y == 0.0) {
    /*  only translation */
    memset(&ws[0], 0, 9U * sizeof(double));
    ws[0] = 1.0;
    p[0] = TwMag[0] * TwMag[6];
    ws[4] = 1.0;
    p[1] = TwMag[1] * TwMag[6];
    ws[8] = 1.0;
    p[2] = TwMag[2] * TwMag[6];
  } else {
    /*  */
    /*  EXPAXANG Matrix Exponential of a Rigid Body ORIENTATION */
    /*  defined by the "Axis-Angle" (AxAng) [x y z theta] == [W theta] (1x4) */
    /*  into a matrix "RotM = exp(W^theta)" (3x3) */
    /*  Use in SO(3). */
    /*  */
    /*  	rotm = expAxAng(AxAng) */
    /*  */
    /*  Returns a rotation matrix from the rotation of magnitude "theta" */
    /*  around the "vector" W = [x y Z], using Rodrigues's formula:  */
    /*  rotm(W,Th)=exp(W^ * Th)=I + W^ * sin(Th)+ W^ * W^ * (1-cos(Th))  */
    /*  Con W^=axis2skew(W) */
    /*  */
    /*  See also: expScrew, axis2skew. */
    /*  */
    /*  Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor. */
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
    /*  Revision 1.1  2020/02/11 00:00:01 */
    /*  General cleanup of code: help comments, see also, copyright */
    /*  references, clarification of functions. */
    /*  */
    /*  	expAxAng(AxAng) */
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
    /*  Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor. */
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
    /*  Revision 1.1  2020/02/11 00:00:01 */
    /*  General cleanup of code: help comments, see also, copyright */
    /*  references, clarification of functions. */
    /*  */
    /*  r = axis2skew(w) */
    ws[0] = 0.0;
    ws[3] = -TwMag[5];
    ws[6] = TwMag[4];
    ws[1] = TwMag[5];
    ws[4] = 0.0;
    ws[7] = -TwMag[3];
    ws[2] = -TwMag[4];
    ws[5] = TwMag[3];
    ws[8] = 0.0;

    /*  */
    scale = sin(TwMag[6]);
    absxk = 1.0 - cos(TwMag[6]);
    for (i = 0; i < 9; i++) {
      b_I[i] = 0;
    }

    for (k = 0; k < 3; k++) {
      b_I[k + 3 * k] = 1;
      for (i = 0; i < 3; i++) {
        b_ws[k + 3 * i] = (ws[k] * ws[3 * i] + ws[k + 3] * ws[3 * i + 1]) + ws[k
          + 6] * ws[3 * i + 2];
      }
    }

    /*  */
    for (i = 0; i < 9; i++) {
      ws[i] = ((double)b_I[i] + ws[i] * scale) + b_ws[i] * absxk;
      b_I[i] = 0;
    }

    b_I[0] = 1;
    b_I[4] = 1;
    b_I[8] = 1;
    for (i = 0; i < 9; i++) {
      b_ws[i] = (double)b_I[i] - ws[i];
    }

    scale = TwMag[4] * TwMag[2] - TwMag[5] * TwMag[1];
    absxk = TwMag[5] * TwMag[0] - TwMag[3] * TwMag[2];
    t = TwMag[3] * TwMag[1] - TwMag[4] * TwMag[0];
    for (i = 0; i < 3; i++) {
      p[i] = (b_ws[i] * scale + b_ws[i + 3] * absxk) + b_ws[i + 6] * t;
    }

    /*  for only rotation joint. */
    /*       p = (eye(3)-r)*(cross(w,v))+w*w'*v*t; % for general (rot+tra) screw.      */
  }

  for (i = 0; i < 3; i++) {
    k = i << 2;
    H[k] = ws[3 * i];
    H[k + 1] = ws[3 * i + 1];
    H[k + 2] = ws[3 * i + 2];
    H[i + 12] = p[i];
  }

  H[3] = 0.0;
  H[7] = 0.0;
  H[11] = 0.0;
  H[15] = 1.0;

  /*  */
}

/*
 * File trailer for expScrew.c
 *
 * [EOF]
 */
