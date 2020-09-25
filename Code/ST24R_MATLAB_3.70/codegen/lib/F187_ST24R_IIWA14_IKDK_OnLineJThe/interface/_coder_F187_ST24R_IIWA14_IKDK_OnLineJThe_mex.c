/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_mex.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 25-Aug-2019 20:35:55
 */

/* Include Files */
#include "_coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_api.h"
#include "_coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_mex.h"

/* Function Declarations */
static void c_F187_ST24R_IIWA14_IKDK_OnLine(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[1]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[1]
 * Return Type  : void
 */
static void c_F187_ST24R_IIWA14_IKDK_OnLine(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[1])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        33, "F187_ST24R_IIWA14_IKDK_OnLineJThe");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 33,
                        "F187_ST24R_IIWA14_IKDK_OnLineJThe");
  }

  /* Call the function. */
  F187_ST24R_IIWA14_IKDK_OnLineJThe_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray * const plhs[]
 *                int32_T nrhs
 *                const mxArray * const prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(F187_ST24R_IIWA14_IKDK_OnLineJThe_atexit);

  /* Module initialization. */
  F187_ST24R_IIWA14_IKDK_OnLineJThe_initialize();

  /* Dispatch the entry-point. */
  c_F187_ST24R_IIWA14_IKDK_OnLine(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  F187_ST24R_IIWA14_IKDK_OnLineJThe_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_F187_ST24R_IIWA14_IKDK_OnLineJThe_mex.c
 *
 * [EOF]
 */
