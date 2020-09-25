/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: jointmag2limits.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "jointmag2limits.h"

/* Function Definitions */

/*
 * Arguments    : double ThTheory
 *                double LimitPos
 *                double LimitNeg
 * Return Type  : double
 */
double jointmag2limits(double ThTheory, double LimitPos, double LimitNeg)
{
  double Th;
  double x;

  /*  JOINTMAG2LIMITS gets the MAGNITUDE for the joint inside mechanical LIMITS */
  /*  Use in SE(3). */
  /*  */
  /*  ThTheory = Ideal joint magnitude. */
  /*  LimitPos = maximun positive magnitude. */
  /*  LimitNeg = maximum negative magnitude. */
  /*  It is also necessary to indicate the type of JOINTTYPE ('rot' or 'tra') */
  /*  for the function to work with both ROTATION & TRANSLATION movements. */
  /*  Use in SE(3). */
  /*  */
  /*    xi = Th= jointmag2limits(ThTheory, LimitPos, LimitNeg, JointType) */
  /*  */
  /*  See also: . */
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
  /*  	Th= jointmag2limits(ThTheory, LimitPos, LimitNeg, JointType) */
  /*  */
  if (fabs(ThTheory) > 3.1415926535897931) {
    x = ThTheory;
    if (ThTheory < 0.0) {
      x = -1.0;
    } else if (ThTheory > 0.0) {
      x = 1.0;
    } else {
      if (ThTheory == 0.0) {
        x = 0.0;
      }
    }

    ThTheory -= x * 2.0 * 3.1415926535897931;
  }

  if (ThTheory >= 0.0) {
    if (LimitPos >= ThTheory) {
      Th = ThTheory;
    } else if (LimitNeg <= ThTheory - 6.2831853071795862) {
      Th = ThTheory - 6.2831853071795862;
    } else if (ThTheory - LimitPos <= LimitNeg - (ThTheory - 6.2831853071795862))
    {
      Th = LimitPos;
    } else {
      Th = LimitNeg;
    }
  } else if (LimitNeg <= ThTheory) {
    Th = ThTheory;
  } else if (LimitPos >= ThTheory + 6.2831853071795862) {
    Th = ThTheory + 6.2831853071795862;
  } else if (LimitNeg - ThTheory <= (ThTheory + 6.2831853071795862) - LimitPos)
  {
    Th = LimitNeg;
  } else {
    Th = LimitPos;
  }

  /*  */
  return Th;
}

/*
 * File trailer for jointmag2limits.c
 *
 * [EOF]
 */
