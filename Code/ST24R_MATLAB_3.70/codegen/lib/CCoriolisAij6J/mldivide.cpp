//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mldivide.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 11-Nov-2020 15:20:33
//

// Include Files
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A[36]
//                double B[36]
// Return Type  : void
//
namespace coder
{
  void mldivide(const double A[36], double B[36])
  {
    double b_A[36];
    double smax;
    int b_i;
    int i;
    int i1;
    int ix;
    int iy;
    int j;
    int jA;
    int jp1j;
    int k;
    signed char ipiv[6];
    std::memcpy(&b_A[0], &A[0], 36U * sizeof(double));
    for (i = 0; i < 6; i++) {
      ipiv[i] = static_cast<signed char>(i + 1);
    }

    for (j = 0; j < 5; j++) {
      int b_tmp;
      int mmj_tmp;
      signed char i2;
      mmj_tmp = 4 - j;
      b_tmp = j * 7;
      jp1j = b_tmp + 2;
      iy = 6 - j;
      jA = 0;
      ix = b_tmp;
      smax = std::abs(b_A[b_tmp]);
      for (k = 2; k <= iy; k++) {
        double s;
        ix++;
        s = std::abs(b_A[ix]);
        if (s > smax) {
          jA = k - 1;
          smax = s;
        }
      }

      if (b_A[b_tmp + jA] != 0.0) {
        if (jA != 0) {
          iy = j + jA;
          ipiv[j] = static_cast<signed char>(iy + 1);
          ix = j;
          for (k = 0; k < 6; k++) {
            smax = b_A[ix];
            b_A[ix] = b_A[iy];
            b_A[iy] = smax;
            ix += 6;
            iy += 6;
          }
        }

        i = (b_tmp - j) + 6;
        for (b_i = jp1j; b_i <= i; b_i++) {
          b_A[b_i - 1] /= b_A[b_tmp];
        }
      }

      iy = b_tmp + 6;
      jA = b_tmp;
      for (b_i = 0; b_i <= mmj_tmp; b_i++) {
        smax = b_A[iy];
        if (b_A[iy] != 0.0) {
          ix = b_tmp + 1;
          i = jA + 8;
          i1 = (jA - j) + 12;
          for (jp1j = i; jp1j <= i1; jp1j++) {
            b_A[jp1j - 1] += b_A[ix] * -smax;
            ix++;
          }
        }

        iy += 6;
        jA += 6;
      }

      i2 = ipiv[j];
      if (i2 != j + 1) {
        for (b_i = 0; b_i < 6; b_i++) {
          iy = j + 6 * b_i;
          smax = B[iy];
          i = (i2 + 6 * b_i) - 1;
          B[iy] = B[i];
          B[i] = smax;
        }
      }
    }

    for (j = 0; j < 6; j++) {
      iy = 6 * j;
      for (k = 0; k < 6; k++) {
        jA = 6 * k;
        i = k + iy;
        if (B[i] != 0.0) {
          i1 = k + 2;
          for (b_i = i1; b_i < 7; b_i++) {
            jp1j = (b_i + iy) - 1;
            B[jp1j] -= B[i] * b_A[(b_i + jA) - 1];
          }
        }
      }
    }

    for (j = 0; j < 6; j++) {
      iy = 6 * j;
      for (k = 5; k >= 0; k--) {
        jA = 6 * k;
        i = k + iy;
        smax = B[i];
        if (smax != 0.0) {
          B[i] = smax / b_A[k + jA];
          for (b_i = 0; b_i < k; b_i++) {
            i1 = b_i + iy;
            B[i1] -= B[i] * b_A[b_i + jA];
          }
        }
      }
    }
  }
}

//
// File trailer for mldivide.cpp
//
// [EOF]
//
