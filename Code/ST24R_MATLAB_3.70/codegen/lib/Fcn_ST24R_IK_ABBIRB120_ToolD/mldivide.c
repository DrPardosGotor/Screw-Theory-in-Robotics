/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mldivide.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 17-May-2020 16:49:25
 */

/* Include Files */
#include "mldivide.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

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
  int mmj_tmp;
  int iy;
  int b;
  int jj;
  int jp1j;
  int jA;
  int ix;
  double smax;
  int k;
  double s;
  int i;
  memcpy(&b_A[0], &A[0], 16U * sizeof(double));
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    mmj_tmp = 2 - j;
    b = j * 5;
    jj = j * 5;
    jp1j = b + 2;
    iy = 4 - j;
    jA = 0;
    ix = b;
    smax = fabs(b_A[jj]);
    for (k = 2; k <= iy; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        jA = k - 1;
        smax = s;
      }
    }

    if (b_A[jj + jA] != 0.0) {
      if (jA != 0) {
        iy = j + jA;
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

      i = (jj - j) + 4;
      for (iy = jp1j; iy <= i; iy++) {
        b_A[iy - 1] /= b_A[jj];
      }
    }

    iy = b + 4;
    jA = jj;
    for (b = 0; b <= mmj_tmp; b++) {
      smax = b_A[iy];
      if (b_A[iy] != 0.0) {
        ix = jj + 1;
        i = jA + 6;
        k = (jA - j) + 8;
        for (jp1j = i; jp1j <= k; jp1j++) {
          b_A[jp1j - 1] += b_A[ix] * -smax;
          ix++;
        }
      }

      iy += 4;
      jA += 4;
    }

    if (ipiv[j] != j + 1) {
      smax = B[j];
      i = ipiv[j] - 1;
      B[j] = B[i];
      B[i] = smax;
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
