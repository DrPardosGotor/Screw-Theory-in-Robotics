/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_MInertiaJsl4J_mex.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Nov-2020 15:58:41
 */

/* Include Files */
#include "_coder_MInertiaJsl4J_mex.h"
#include "_coder_MInertiaJsl4J_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[2]
 * Return Type  : void
 */
void MInertiaJsl4J_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[2])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  const mxArray *outputs[1];
  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        13, "MInertiaJsl4J");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "MInertiaJsl4J");
  }

  /* Call the function. */
  MInertiaJsl4J_api(prhs, outputs);

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
  mexAtExit(&MInertiaJsl4J_atexit);

  /* Module initialization. */
  MInertiaJsl4J_initialize();
  try {
    emlrtShouldCleanupOnError(emlrtRootTLSGlobal, false);

    /* Dispatch the entry-point. */
    MInertiaJsl4J_mexFunction(nlhs, plhs, nrhs, prhs);

    /* Module termination. */
    MInertiaJsl4J_terminate();
  } catch (...) {
    emlrtCleanupOnException(emlrtRootTLSGlobal);
    throw;
  }
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
 * File trailer for _coder_MInertiaJsl4J_mex.c
 *
 * [EOF]
 */
