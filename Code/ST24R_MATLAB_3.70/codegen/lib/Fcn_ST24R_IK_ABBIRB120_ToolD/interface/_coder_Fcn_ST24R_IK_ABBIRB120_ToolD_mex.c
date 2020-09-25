/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_Fcn_ST24R_IK_ABBIRB120_ToolD_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 17-May-2020 16:49:25
 */

/* Include Files */
#include "_coder_Fcn_ST24R_IK_ABBIRB120_ToolD_mex.h"
#include "_coder_Fcn_ST24R_IK_ABBIRB120_ToolD_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void c_Fcn_ST24R_IK_ABBIRB120_ToolD_(int32_T nlhs, mxArray
  *plhs[1], int32_T nrhs, const mxArray *prhs[1]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[1]
 * Return Type  : void
 */
void c_Fcn_ST24R_IK_ABBIRB120_ToolD_(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[1])
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
                        28, "Fcn_ST24R_IK_ABBIRB120_ToolD");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 28,
                        "Fcn_ST24R_IK_ABBIRB120_ToolD");
  }

  /* Call the function. */
  Fcn_ST24R_IK_ABBIRB120_ToolD_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(&Fcn_ST24R_IK_ABBIRB120_ToolD_atexit);

  /* Module initialization. */
  Fcn_ST24R_IK_ABBIRB120_ToolD_initialize();

  /* Dispatch the entry-point. */
  c_Fcn_ST24R_IK_ABBIRB120_ToolD_(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  Fcn_ST24R_IK_ABBIRB120_ToolD_terminate();
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
 * File trailer for _coder_Fcn_ST24R_IK_ABBIRB120_ToolD_mex.c
 *
 * [EOF]
 */
