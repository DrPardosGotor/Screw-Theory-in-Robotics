/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_Fcn_ST24R_IK_ABBIRB120_ToolD_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 17-May-2020 16:49:25
 */

#ifndef _CODER_FCN_ST24R_IK_ABBIRB120_TOOLD_API_H
#define _CODER_FCN_ST24R_IK_ABBIRB120_TOOLD_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void Fcn_ST24R_IK_ABBIRB120_ToolD(real_T u[6], real_T ThetaSET[48]);
extern void Fcn_ST24R_IK_ABBIRB120_ToolD_api(const mxArray * const prhs[1],
  int32_T nlhs, const mxArray *plhs[1]);
extern void Fcn_ST24R_IK_ABBIRB120_ToolD_atexit(void);
extern void Fcn_ST24R_IK_ABBIRB120_ToolD_initialize(void);
extern void Fcn_ST24R_IK_ABBIRB120_ToolD_terminate(void);
extern void Fcn_ST24R_IK_ABBIRB120_ToolD_xil_shutdown(void);
extern void Fcn_ST24R_IK_ABBIRB120_ToolD_xil_terminate(void);

#endif

/*
 * File trailer for _coder_Fcn_ST24R_IK_ABBIRB120_ToolD_api.h
 *
 * [EOF]
 */
