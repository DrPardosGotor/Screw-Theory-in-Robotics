/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_MInertiaJsl6J_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 11-Nov-2020 15:19:21
 */

#ifndef _CODER_MINERTIAJSL6J_API_H
#define _CODER_MINERTIAJSL6J_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void MInertiaJsl6J(real_T TwMag[42], real_T LiMas[42], real_T Mt[36]);
  void MInertiaJsl6J_api(const mxArray * const prhs[2], const mxArray *plhs[1]);
  void MInertiaJsl6J_atexit(void);
  void MInertiaJsl6J_initialize(void);
  void MInertiaJsl6J_terminate(void);
  void MInertiaJsl6J_xil_shutdown(void);
  void MInertiaJsl6J_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_MInertiaJsl6J_api.h
 *
 * [EOF]
 */
