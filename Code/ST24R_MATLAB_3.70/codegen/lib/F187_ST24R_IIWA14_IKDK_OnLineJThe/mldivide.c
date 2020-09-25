/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mldivide.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * Arguments    : const double A[16]
 *                double B[4]
 * Return Type  : void
 */
void mldivide(const double A[16], double B[4])
{
  double b_A[16];
  signed char ipiv[4];
  int j;
  int b;
  int iy;
  int jj;
  int jp1j;
  int n;
  int ix;
  double smax;
  int jA;
  double s;
  int i13;
  int ijA;
  memcpy(&b_A[0], &A[0], sizeof(double) << 4);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    b = j * 5;
    jj = j * 5;
    jp1j = b + 2;
    n = 4 - j;
    iy = 0;
    ix = b;
    smax = fabs(b_A[b]);
    for (jA = 2; jA <= n; jA++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        iy = jA - 1;
        smax = s;
      }
    }

    if (b_A[jj + iy] != 0.0) {
      if (iy != 0) {
        iy += j;
        ipiv[j] = (signed char)(iy + 1);
        smax = b_A[j];
        b_A[j] = b_A[iy];
        b_A[iy] = smax;
        ix = j + 4;
        iy += 4;
        smax = b_A[ix];
        b_A[ix] = b_A[iy];
        b_A[iy] = smax;
        ix += 4;
        iy += 4;
        smax = b_A[ix];
        b_A[ix] = b_A[iy];
        b_A[iy] = smax;
        ix += 4;
        iy += 4;
        smax = b_A[ix];
        b_A[ix] = b_A[iy];
        b_A[iy] = smax;
      }

      i13 = jj - j;
      for (iy = jp1j; iy <= i13 + 4; iy++) {
        b_A[iy - 1] /= b_A[jj];
      }
    }

    n = 2 - j;
    iy = b + 4;
    jA = jj + 5;
    for (b = 0; b <= n; b++) {
      smax = b_A[iy];
      if (b_A[iy] != 0.0) {
        ix = jj + 1;
        i13 = jA + 1;
        jp1j = (jA - j) + 3;
        for (ijA = i13; ijA <= jp1j; ijA++) {
          b_A[ijA - 1] += b_A[ix] * -smax;
          ix++;
        }
      }

      iy += 4;
      jA += 4;
    }

    if (ipiv[j] != j + 1) {
      smax = B[j];
      i13 = ipiv[j] - 1;
      B[j] = B[i13];
      B[i13] = smax;
    }
  }

  if (B[0] != 0.0) {
    for (iy = 2; iy < 5; iy++) {
      B[iy - 1] -= B[0] * b_A[iy - 1];
    }
  }

  if (B[1] != 0.0) {
    for (iy = 3; iy < 5; iy++) {
      B[iy - 1] -= B[1] * b_A[iy + 3];
    }
  }

  if (B[2] != 0.0) {
    for (iy = 4; iy < 5; iy++) {
      B[3] -= B[2] * b_A[11];
    }
  }

  if (B[3] != 0.0) {
    B[3] /= b_A[15];
    for (iy = 0; iy < 3; iy++) {
      B[iy] -= B[3] * b_A[iy + 12];
    }
  }

  if (B[2] != 0.0) {
    B[2] /= b_A[10];
    for (iy = 0; iy < 2; iy++) {
      B[iy] -= B[2] * b_A[iy + 8];
    }
  }

  if (B[1] != 0.0) {
    B[1] /= b_A[5];
    for (iy = 0; iy < 1; iy++) {
      B[0] -= B[1] * b_A[4];
    }
  }

  if (B[0] != 0.0) {
    B[0] /= b_A[0];
  }
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
