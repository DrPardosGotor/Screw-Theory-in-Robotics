/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_CCoriolisAij7J_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 15-Nov-2020 13:46:10
 */

#ifndef _CODER_CCORIOLISAIJ7J_API_H
#define _CODER_CCORIOLISAIJ7J_API_H

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
  void CCoriolisAij7J(real_T TwMag[49], real_T LiMas[49], real_T Thetap[7],
                      real_T Ctdt[49]);
  void CCoriolisAij7J_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void CCoriolisAij7J_atexit(void);
  void CCoriolisAij7J_initialize(void);
  void CCoriolisAij7J_terminate(void);
  void CCoriolisAij7J_xil_shutdown(void);
  void CCoriolisAij7J_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_CCoriolisAij7J_api.h
 *
 * [EOF]
 */
