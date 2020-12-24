/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_CCoriolisAij4J_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Nov-2020 16:02:13
 */

#ifndef _CODER_CCORIOLISAIJ4J_API_H
#define _CODER_CCORIOLISAIJ4J_API_H

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
  void CCoriolisAij4J(real_T TwMag[28], real_T LiMas[28], real_T Thetap[4],
                      real_T Ctdt[16]);
  void CCoriolisAij4J_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void CCoriolisAij4J_atexit(void);
  void CCoriolisAij4J_initialize(void);
  void CCoriolisAij4J_terminate(void);
  void CCoriolisAij4J_xil_shutdown(void);
  void CCoriolisAij4J_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_CCoriolisAij4J_api.h
 *
 * [EOF]
 */
