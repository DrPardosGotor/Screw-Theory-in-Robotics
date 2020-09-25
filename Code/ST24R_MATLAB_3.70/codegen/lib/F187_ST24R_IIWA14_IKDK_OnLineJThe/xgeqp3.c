/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgeqp3.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe.h"
#include "xgeqp3.h"
#include "xnrm2.h"
#include "sqrt1.h"
#include "F187_ST24R_IIWA14_IKDK_OnLineJThe_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : double A[42]
 *                double tau[6]
 *                int jpvt[7]
 * Return Type  : void
 */
void xgeqp3(double A[42], double tau[6], int jpvt[7])
{
  int k;
  int j;
  int i;
  double work[7];
  int ip1;
  double smax;
  int i_i;
  double scale;
  int kend;
  int pvt;
  int ix;
  double vn1[7];
  double absxk;
  double vn2[7];
  double t;
  int iy;
  int lastv;
  int lastc;
  int i11;
  boolean_T exitg2;
  int exitg1;
  int i12;
  k = 1;
  for (j = 0; j < 7; j++) {
    jpvt[j] = 1 + j;
    work[j] = 0.0;
    smax = 0.0;
    scale = 3.3121686421112381E-170;
    kend = k + 5;
    for (pvt = k; pvt <= kend; pvt++) {
      absxk = fabs(A[pvt - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        smax = 1.0 + smax * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        smax += t * t;
      }
    }

    smax = scale * sqrt(smax);
    vn1[j] = smax;
    vn2[j] = smax;
    k += 6;
  }

  for (i = 0; i < 6; i++) {
    ip1 = i + 2;
    i_i = i + i * 6;
    kend = 7 - i;
    pvt = 1;
    ix = i;
    smax = fabs(vn1[i]);
    for (k = 2; k <= kend; k++) {
      ix++;
      scale = fabs(vn1[ix]);
      if (scale > smax) {
        pvt = k;
        smax = scale;
      }
    }

    pvt = (i + pvt) - 1;
    if (pvt != i) {
      ix = 6 * pvt;
      iy = 6 * i;
      for (k = 0; k < 6; k++) {
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix++;
        iy++;
      }

      kend = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = kend;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    if (i + 1 < 6) {
      absxk = A[i_i];
      kend = i_i + 2;
      tau[i] = 0.0;
      smax = c_xnrm2(5 - i, A, i_i + 2);
      if (smax != 0.0) {
        scale = rt_hypotd_snf(A[i_i], smax);
        if (A[i_i] >= 0.0) {
          scale = -scale;
        }

        if (fabs(scale) < 1.0020841800044864E-292) {
          pvt = -1;
          i11 = (i_i - i) + 6;
          do {
            pvt++;
            for (k = kend; k <= i11; k++) {
              A[k - 1] *= 9.9792015476736E+291;
            }

            scale *= 9.9792015476736E+291;
            absxk *= 9.9792015476736E+291;
          } while (!(fabs(scale) >= 1.0020841800044864E-292));

          scale = rt_hypotd_snf(absxk, c_xnrm2(5 - i, A, i_i + 2));
          if (absxk >= 0.0) {
            scale = -scale;
          }

          tau[i] = (scale - absxk) / scale;
          smax = 1.0 / (absxk - scale);
          for (k = kend; k <= i11; k++) {
            A[k - 1] *= smax;
          }

          for (k = 0; k <= pvt; k++) {
            scale *= 1.0020841800044864E-292;
          }

          absxk = scale;
        } else {
          tau[i] = (scale - A[i_i]) / scale;
          smax = 1.0 / (A[i_i] - scale);
          i11 = (i_i - i) + 6;
          for (k = kend; k <= i11; k++) {
            A[k - 1] *= smax;
          }

          absxk = scale;
        }
      }

      A[i_i] = absxk;
    } else {
      tau[5] = 0.0;
    }

    absxk = A[i_i];
    A[i_i] = 1.0;
    k = (i + (i + 1) * 6) + 1;
    if (tau[i] != 0.0) {
      lastv = 6 - i;
      kend = (i_i - i) + 5;
      while ((lastv > 0) && (A[kend] == 0.0)) {
        lastv--;
        kend--;
      }

      lastc = 5 - i;
      exitg2 = false;
      while ((!exitg2) && (lastc + 1 > 0)) {
        kend = k + lastc * 6;
        pvt = kend;
        do {
          exitg1 = 0;
          if (pvt <= (kend + lastv) - 1) {
            if (A[pvt - 1] != 0.0) {
              exitg1 = 1;
            } else {
              pvt++;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = -1;
    }

    if (lastv > 0) {
      if (lastc + 1 != 0) {
        if (0 <= lastc) {
          memset(&work[0], 0, (unsigned int)((lastc + 1) * (int)sizeof(double)));
        }

        iy = 0;
        i11 = k + 6 * lastc;
        for (kend = k; kend <= i11; kend += 6) {
          ix = i_i;
          smax = 0.0;
          i12 = (kend + lastv) - 1;
          for (pvt = kend; pvt <= i12; pvt++) {
            smax += A[pvt - 1] * A[ix];
            ix++;
          }

          work[iy] += smax;
          iy++;
        }
      }

      if (!(-tau[i] == 0.0)) {
        kend = k - 1;
        pvt = 0;
        for (j = 0; j <= lastc; j++) {
          if (work[pvt] != 0.0) {
            smax = work[pvt] * -tau[i];
            ix = i_i;
            i11 = kend + 1;
            i12 = lastv + kend;
            for (iy = i11; iy <= i12; iy++) {
              A[iy - 1] += A[ix] * smax;
              ix++;
            }
          }

          pvt++;
          kend += 6;
        }
      }
    }

    A[i_i] = absxk;
    for (j = ip1; j < 8; j++) {
      smax = vn1[j - 1];
      if (smax != 0.0) {
        kend = i + 6 * (j - 1);
        scale = fabs(A[kend]) / smax;
        scale = 1.0 - scale * scale;
        if (scale < 0.0) {
          scale = 0.0;
        }

        absxk = smax / vn2[j - 1];
        absxk = scale * (absxk * absxk);
        if (absxk <= 1.4901161193847656E-8) {
          if (i + 1 < 6) {
            smax = c_xnrm2(5 - i, A, kend + 2);
            vn1[j - 1] = smax;
            vn2[j - 1] = smax;
          } else {
            vn1[j - 1] = 0.0;
            vn2[j - 1] = 0.0;
          }
        } else {
          vn1[j - 1] = smax * sqrt(scale);
        }
      }
    }
  }
}

/*
 * File trailer for xgeqp3.c
 *
 * [EOF]
 */
