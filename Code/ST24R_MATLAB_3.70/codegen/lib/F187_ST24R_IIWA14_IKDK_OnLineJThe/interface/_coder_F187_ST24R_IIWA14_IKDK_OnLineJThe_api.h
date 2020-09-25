/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_api.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

#ifndef _CODER_F187_ST24R_IIWA14_IKDK_ONLINEJTHE_API_H
#define _CODER_F187_ST24R_IIWA14_IKDK_ONLINEJTHE_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe(real_T u[7], real_T ThetaOut[21]);
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe_api(const mxArray * const prhs[1],
  int32_T nlhs, const mxArray *plhs[1]);
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe_atexit(void);
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe_initialize(void);
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe_terminate(void);
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe_xil_shutdown(void);
extern void F187_ST24R_IIWA14_IKDK_OnLineJThe_xil_terminate(void);

#endif

/*
 * File trailer for _coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_api.h
 *
 * [EOF]
 */
