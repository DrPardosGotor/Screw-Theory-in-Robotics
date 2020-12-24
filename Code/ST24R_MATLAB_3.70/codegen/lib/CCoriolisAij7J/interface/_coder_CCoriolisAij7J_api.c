/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_CCoriolisAij7J_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 15-Nov-2020 13:46:10
 */

/* Include Files */
#include "_coder_CCoriolisAij7J_api.h"
#include "_coder_CCoriolisAij7J_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "CCoriolisAij7J",                    /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[49];
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Thetap,
  const char_T *identifier))[7];
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[7];
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[49];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *TwMag,
  const char_T *identifier))[49];
static const mxArray *emlrt_marshallOut(const real_T u[49]);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[7];

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[49]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[49]
{
  real_T (*y)[49];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *Thetap
 *                const char_T *identifier
 * Return Type  : real_T (*)[7]
 */
  static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Thetap,
  const char_T *identifier))[7]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[7];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(Thetap), &thisId);
  emlrtDestroyArray(&Thetap);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[7]
 */
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[7]
{
  real_T (*y)[7];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[49]
 */
  static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[49]
{
  static const int32_T dims[2] = { 7, 7 };

  real_T (*ret)[49];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[49])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *TwMag
 *                const char_T *identifier
 * Return Type  : real_T (*)[49]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *TwMag,
  const char_T *identifier))[49]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[49];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(TwMag), &thisId);
  emlrtDestroyArray(&TwMag);
  return y;
}
/*
 * Arguments    : const real_T u[49]
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real_T u[49])
{
  static const int32_T iv[2] = { 0, 0 };

  static const int32_T iv1[2] = { 7, 7 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[7]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[7]
{
  static const int32_T dims[2] = { 1, 7 };

  real_T (*ret)[7];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[7])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
  void CCoriolisAij7J_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*Ctdt)[49];
  real_T (*LiMas)[49];
  real_T (*TwMag)[49];
  real_T (*Thetap)[7];
  st.tls = emlrtRootTLSGlobal;
  Ctdt = (real_T (*)[49])mxMalloc(sizeof(real_T [49]));

  /* Marshall function inputs */
  TwMag = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "TwMag");
  LiMas = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "LiMas");
  Thetap = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "Thetap");

  /* Invoke the target function */
  CCoriolisAij7J(*TwMag, *LiMas, *Thetap, *Ctdt);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Ctdt);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void CCoriolisAij7J_atexit(void)
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
  CCoriolisAij7J_xil_terminate();
  CCoriolisAij7J_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void CCoriolisAij7J_initialize(void)
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
void CCoriolisAij7J_terminate(void)
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
 * File trailer for _coder_CCoriolisAij7J_api.c
 *
 * [EOF]
 */
