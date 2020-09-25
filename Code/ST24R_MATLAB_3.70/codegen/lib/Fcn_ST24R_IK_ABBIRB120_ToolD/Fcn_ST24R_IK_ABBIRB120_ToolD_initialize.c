/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Fcn_ST24R_IK_ABBIRB120_ToolD_initialize.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 17-May-2020 16:49:25
 */

/* Include Files */
#include "Fcn_ST24R_IK_ABBIRB120_ToolD_initialize.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD.h"
#include "Fcn_ST24R_IK_ABBIRB120_ToolD_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void Fcn_ST24R_IK_ABBIRB120_ToolD_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_Fcn_ST24R_IK_ABBIRB120_ToolD = true;
}

/*
 * File trailer for Fcn_ST24R_IK_ABBIRB120_ToolD_initialize.c
 *
 * [EOF]
 */
