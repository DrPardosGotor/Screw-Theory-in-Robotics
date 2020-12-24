/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_NPotentialWre6J_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 11-Nov-2020 15:21:44
 */

/* Include Files */
#include "_coder_NPotentialWre6J_api.h"
#include "_coder_NPotentialWre6J_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "NPotentialWre6J",                   /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[42];
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *PoAcc,
  const char_T *identifier))[3];
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[42];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *TwMag,
  const char_T *identifier))[42];
static const mxArray *emlrt_marshallOut(const real_T u[6]);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[42]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[42]
{
  real_T (*y)[42];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *PoAcc
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
  static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *PoAcc,
  const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(PoAcc), &thisId);
  emlrtDestroyArray(&PoAcc);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[42]
 */
  static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[42]
{
  static const int32_T dims[2] = { 7, 6 };

  real_T (*ret)[42];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[42])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *TwMag
 *                const char_T *identifier
 * Return Type  : real_T (*)[42]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *TwMag,
  const char_T *identifier))[42]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[42];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(TwMag), &thisId);
  emlrtDestroyArray(&TwMag);
  return y;
}
/*
 * Arguments    : const real_T u[6]
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real_T u[6])
{
  static const int32_T iv[1] = { 0 };

  static const int32_T iv1[1] = { 6 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[1] = { 3 };

  real_T (*ret)[3];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
  void NPotentialWre6J_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*LiMas)[42];
  real_T (*TwMag)[42];
  real_T (*Nt)[6];
  real_T (*PoAcc)[3];
  st.tls = emlrtRootTLSGlobal;
  Nt = (real_T (*)[6])mxMalloc(sizeof(real_T [6]));

  /* Marshall function inputs */
  TwMag = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "TwMag");
  LiMas = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "LiMas");
  PoAcc = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "PoAcc");

  /* Invoke the target function */
  NPotentialWre6J(*TwMag, *LiMas, *PoAcc, *Nt);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Nt);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void NPotentialWre6J_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  NPotentialWre6J_xil_terminate();
  NPotentialWre6J_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void NPotentialWre6J_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void NPotentialWre6J_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_NPotentialWre6J_api.c
 *
 * [EOF]
 */
