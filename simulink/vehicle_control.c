/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: vehicle_control.c
 *
 * Code generated for Simulink model 'vehicle_control'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Thu Jul 25 16:06:43 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "vehicle_control.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Continuous states */
X rtX;

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void BINARYSEARCH_real_T(uint32_T *piLeft, uint32_T *piRght, real_T u,
  const real_T *pData, uint32_T iHi);
static real_T look2_binlc(real_T u0, real_T u1, const real_T bp0[], const real_T
  bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride);

/* private model entry point functions */
extern void vehicle_control_derivatives(void);

/* Lookup Binary Search Utility BINARYSEARCH_real_T */
static void BINARYSEARCH_real_T(uint32_T *piLeft, uint32_T *piRght, real_T u,
  const real_T *pData, uint32_T iHi)
{
  /* Find the location of current input value in the data table. */
  *piLeft = 0U;
  *piRght = iHi;
  if (u <= pData[0] ) {
    /* Less than or equal to the smallest point in the table. */
    *piRght = 0U;
  } else if (u >= pData[iHi] ) {
    /* Greater than or equal to the largest point in the table. */
    *piLeft = iHi;
  } else {
    uint32_T i;

    /* Do a binary search. */
    while (( *piRght - *piLeft ) > 1U ) {
      /* Get the average of the left and right indices using to Floor rounding. */
      i = (*piLeft + *piRght) >> 1;

      /* Move either the right index or the left index so that */
      /*  LeftDataPoint <= CurrentValue < RightDataPoint */
      if (u < pData[i] ) {
        *piRght = i;
      } else {
        *piLeft = i;
      }
    }
  }
}

static real_T look2_binlc(real_T u0, real_T u1, const real_T bp0[], const real_T
  bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride)
{
  real_T fractions[2];
  real_T frac;
  real_T yL_0d0;
  real_T yL_0d1;
  uint32_T bpIndices[2];
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 2-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex[0U]]) {
    /* Binary Search */
    bpIdx = maxIndex[0U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[0U];
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex[0U] - 1U;
    frac = 1.0;
  }

  fractions[0U] = frac;
  bpIndices[0U] = iLeft;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u1 <= bp1[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u1 < bp1[maxIndex[1U]]) {
    /* Binary Search */
    bpIdx = maxIndex[1U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[1U];
    while (iRght - iLeft > 1U) {
      if (u1 < bp1[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u1 - bp1[iLeft]) / (bp1[iLeft + 1U] - bp1[iLeft]);
  } else {
    iLeft = maxIndex[1U] - 1U;
    frac = 1.0;
  }

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  bpIdx = iLeft * stride + bpIndices[0U];
  yL_0d0 = table[bpIdx];
  yL_0d0 += (table[bpIdx + 1U] - yL_0d0) * fractions[0U];
  bpIdx += stride;
  yL_0d1 = table[bpIdx];
  return (((table[bpIdx + 1U] - yL_0d1) * fractions[0U] + yL_0d1) - yL_0d0) *
    frac + yL_0d0;
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  vehicle_control_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  vehicle_control_step();
  vehicle_control_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  vehicle_control_step();
  vehicle_control_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  vehicle_control_step();
  vehicle_control_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void vehicle_control_step(void)
{
  real_T rtb_DProdOut;
  real_T rtb_Gain;
  real_T rtb_I_gain_1;
  real_T rtb_Subtract1;
  real_T rtb_Switch2;
  real_T rtb_Switch_p;
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   *  Inport: '<Root>/whl_spd_rl'
   */
  if (rtU.whl_spd_rl > 0.0) {
    /* Product: '<S8>/Divide' incorporates:
     *  Constant: '<S8>/Constant11'
     *  Gain: '<S8>/Gain'
     */
    rtb_Switch2 = 30000.0 / (13.15 * rtU.whl_spd_rl);

    /* Saturate: '<S8>/Saturation' */
    if (rtb_Switch2 > 29.0) {
      rtb_Switch2 = 29.0;
    } else if (rtb_Switch2 < -0.5) {
      rtb_Switch2 = -0.5;
    }

    /* End of Saturate: '<S8>/Saturation' */
  } else {
    rtb_Switch2 = 29.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* Gain: '<S8>/Gain2' */
  rtb_Switch2 *= 13.15;

  /* TransferFcn: '<Root>/Transfer Fcn' */
  rtb_Subtract1 = 66.666666666666671 * rtX.TransferFcn_CSTATE;

  /* Gain: '<Root>/Gain' */
  rtb_Gain = 13.15 * rtb_Subtract1;

  /* Sum: '<S12>/Subtract1' incorporates:
   *  Gain: '<S12>/Gain6'
   *  Inport: '<Root>/steer_rl'
   *  Inport: '<Root>/steer_rr'
   *  Inport: '<Root>/v_x'
   *  Inport: '<Root>/yaw_rate'
   *  MATLAB Function: '<S12>/reference yaw rate 1'
   *  Sum: '<S12>/Sum1'
   */
  rtb_Subtract1 = (rtU.steer_rl + rtU.steer_rr) * 0.5 * rtU.v_x * 1.528 /
    (rtU.v_x * rtU.v_x * 0.002 + 1.0) - rtU.yaw_rate;

  /* Gain: '<S3>/Multiply3' incorporates:
   *  Inport: '<Root>/v_x'
   */
  rtb_I_gain_1 = 3.6 * rtU.v_x;

  /* Lookup_n-D: '<S3>/D_gain_1' incorporates:
   *  Inport: '<Root>/a_y'
   *  Lookup_n-D: '<S3>/I_gain_1'
   */
  rtb_Switch_p = look2_binlc(rtb_I_gain_1, rtU.a_y, rtConstP.pooled9,
    rtConstP.pooled10, rtConstP.D_gain_1_tableData, rtConstP.pooled15, 11U);

  /* Product: '<S49>/DProd Out' */
  rtb_DProdOut = rtb_Subtract1 * rtb_Switch_p;

  /* Integrator: '<S50>/Filter' */
  rtb_Switch_p = rtX.Filter_CSTATE;

  /* Product: '<S58>/NProd Out' incorporates:
   *  Sum: '<S50>/SumD'
   */
  rtDW.NProdOut = rtb_DProdOut - rtb_Switch_p;

  /* Sum: '<S64>/Sum' incorporates:
   *  Inport: '<Root>/a_y'
   *  Integrator: '<S55>/Integrator'
   *  Lookup_n-D: '<S3>/I_gain_1'
   *  Lookup_n-D: '<S3>/P_gain_1'
   *  Product: '<S60>/PProd Out'
   */
  rtb_DProdOut = (rtb_Subtract1 * look2_binlc(rtb_I_gain_1, rtU.a_y,
    rtConstP.pooled9, rtConstP.pooled10, rtConstP.P_gain_1_tableData,
    rtConstP.pooled15, 11U) + rtX.Integrator_CSTATE) + rtDW.NProdOut;

  /* Sum: '<Root>/Sum1' */
  rtb_Gain -= rtb_DProdOut;

  /* Switch: '<S6>/Switch2' incorporates:
   *  RelationalOperator: '<S6>/LowerRelop1'
   */
  if (!(rtb_Gain > rtb_Switch2)) {
    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S11>/Constant'
     *  Inport: '<Root>/whl_spd_rl'
     */
    if (rtU.whl_spd_rl > 0.0) {
      /* Product: '<S11>/Divide' incorporates:
       *  Constant: '<S11>/Constant11'
       *  Gain: '<S11>/Gain'
       */
      rtb_Switch2 = 30000.0 / (13.15 * rtU.whl_spd_rl);

      /* Saturate: '<S11>/Saturation' */
      if (rtb_Switch2 > 29.0) {
        rtb_Switch2 = 29.0;
      } else if (rtb_Switch2 < -0.5) {
        rtb_Switch2 = -0.5;
      }

      /* End of Saturate: '<S11>/Saturation' */
    } else {
      rtb_Switch2 = 29.0;
    }

    /* End of Switch: '<S11>/Switch' */

    /* Gain: '<S11>/Gain1' incorporates:
     *  Gain: '<S11>/Gain2'
     */
    rtb_Switch2 = -(13.15 * rtb_Switch2);

    /* Switch: '<S6>/Switch' incorporates:
     *  RelationalOperator: '<S6>/UpperRelop'
     */
    if (!(rtb_Gain < rtb_Switch2)) {
      rtb_Switch2 = rtb_Gain;
    }

    /* End of Switch: '<S6>/Switch' */
  }

  /* End of Switch: '<S6>/Switch2' */

  /* Outport: '<Root>/trq_rl' */
  rtY.trq_rl = rtb_Switch2;

  /* TransferFcn: '<Root>/Transfer Fcn1' */
  rtb_Switch_p = 66.666666666666671 * rtX.TransferFcn1_CSTATE;

  /* Sum: '<Root>/Sum4' incorporates:
   *  Gain: '<Root>/Gain1'
   */
  rtb_Switch2 = 13.15 * rtb_Switch_p + rtb_DProdOut;

  /* Switch: '<S9>/Switch' incorporates:
   *  Constant: '<S9>/Constant'
   *  Inport: '<Root>/whl_spd_rr'
   */
  if (rtU.whl_spd_rr > 0.0) {
    /* Product: '<S9>/Divide' incorporates:
     *  Constant: '<S9>/Constant11'
     *  Gain: '<S9>/Gain'
     */
    rtb_Switch_p = 30000.0 / (13.15 * rtU.whl_spd_rr);

    /* Saturate: '<S9>/Saturation' */
    if (rtb_Switch_p > 29.0) {
      rtb_Switch_p = 29.0;
    } else if (rtb_Switch_p < -0.5) {
      rtb_Switch_p = -0.5;
    }

    /* End of Saturate: '<S9>/Saturation' */
  } else {
    rtb_Switch_p = 29.0;
  }

  /* End of Switch: '<S9>/Switch' */

  /* Gain: '<S9>/Gain2' */
  rtb_Switch_p *= 13.15;

  /* Switch: '<S7>/Switch2' incorporates:
   *  RelationalOperator: '<S7>/LowerRelop1'
   */
  if (!(rtb_Switch2 > rtb_Switch_p)) {
    /* Switch: '<S10>/Switch' incorporates:
     *  Constant: '<S10>/Constant'
     *  Inport: '<Root>/whl_spd_rr'
     */
    if (rtU.whl_spd_rr > 0.0) {
      /* Product: '<S10>/Divide' incorporates:
       *  Constant: '<S10>/Constant11'
       *  Gain: '<S10>/Gain'
       */
      rtb_Switch_p = 30000.0 / (13.15 * rtU.whl_spd_rr);

      /* Saturate: '<S10>/Saturation' */
      if (rtb_Switch_p > 29.0) {
        rtb_Switch_p = 29.0;
      } else if (rtb_Switch_p < -0.5) {
        rtb_Switch_p = -0.5;
      }

      /* End of Saturate: '<S10>/Saturation' */
    } else {
      rtb_Switch_p = 29.0;
    }

    /* End of Switch: '<S10>/Switch' */

    /* Gain: '<S10>/Gain1' incorporates:
     *  Gain: '<S10>/Gain2'
     */
    rtb_Switch_p = -(13.15 * rtb_Switch_p);

    /* Switch: '<S7>/Switch' incorporates:
     *  RelationalOperator: '<S7>/UpperRelop'
     */
    if (!(rtb_Switch2 < rtb_Switch_p)) {
      rtb_Switch_p = rtb_Switch2;
    }

    /* End of Switch: '<S7>/Switch' */
  }

  /* End of Switch: '<S7>/Switch2' */

  /* Outport: '<Root>/trq_rr' */
  rtY.trq_rr = rtb_Switch_p;

  /* Product: '<S52>/IProd Out' incorporates:
   *  Inport: '<Root>/a_y'
   *  Lookup_n-D: '<S3>/I_gain_1'
   */
  rtDW.IProdOut = rtb_Subtract1 * look2_binlc(rtb_I_gain_1, rtU.a_y,
    rtConstP.pooled9, rtConstP.pooled10, rtConstP.I_gain_1_tableData,
    rtConstP.pooled15, 11U);

  /* Gain: '<S4>/Gain1' incorporates:
   *  Gain: '<S4>/Gain'
   *  Inport: '<Root>/whl_spd_rl'
   */
  rtb_Subtract1 = 13.15 * rtU.whl_spd_rl * 9.5492965855137211;

  /* Switch: '<S1>/Switch' incorporates:
   *  Inport: '<Root>/gas'
   */
  rtb_Switch_p = rtU.gas;

  /* Lookup2D: '<Root>/E-Motor Mapping RL' */
  /*
   * About '<Root>/E-Motor Mapping RL':
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   * Row Data    parameter uses the same data type and scaling as Input0
   * Column Data parameter uses the same data type and scaling as Input1
   * Table Data  parameter uses the same data type and scaling as Output0
   */
  {
    uint32_T iLeftU0, iRghtU0, iLeftU1, iRghtU1;

    /* Find the location of current input value in the data table.
     */
    if (rtb_Subtract1 <= 0.0 ) {
      /* Less than or equal to the smallest point in the table.
       */
      iLeftU0 = 0U;
      iRghtU0 = 0U;
    } else if (rtb_Subtract1 >= 14000.0 ) {
      /* Greater than or equal to the largest point in the table. */
      iLeftU0 = 14U;
      iRghtU0 = 14U;
    } else {
      /* The table is inlined and evenly spaced.
       * The index is found directly by use of division.
       */
      iLeftU0 = (uint32_T)(( rtb_Subtract1 ) / 1000.0);
      iRghtU0 = iLeftU0 + 1;
    }

    BINARYSEARCH_real_T( &(iLeftU1), &(iRghtU1), rtb_Switch_p, rtConstP.pooled13,
                        5U);

    {
      real_T yTemp;
      real_T u1Lambda;
      real_T u0Lambda;
      if ((rtConstP.pooled13[iRghtU1]) > (rtConstP.pooled13[iLeftU1]) ) {
        real_T num;
        real_T den;
        den = (rtConstP.pooled13[iRghtU1]);
        den -= (rtConstP.pooled13[iLeftU1]);
        num = rtb_Switch_p;
        num -= (rtConstP.pooled13[iLeftU1]);
        u1Lambda = num / den;
      } else {
        u1Lambda = 0.0;
      }

      if (iLeftU0 != iRghtU0 ) {
        {
          real_T num = (real_T)( rtb_Subtract1 ) - ( iLeftU0 * 1000.0 );
          u0Lambda = num / 1000.0;
        }
      } else {
        u0Lambda = 0.0;
      }

      /* Interpolate along u1 variable
       *    with the u0 variable locked on the left u0
       */
      {
        real_T yLeftCast;
        real_T yRghtCast;
        yLeftCast = (rtConstP.pooled14[(iLeftU0+15U*iLeftU1)]);
        yRghtCast = (rtConstP.pooled14[(iLeftU0+15U*iRghtU1)]);
        yLeftCast += u1Lambda * ( yRghtCast - yLeftCast );
        rtDW.EMotorMappingRL = yLeftCast;
      }

      /* Interpolate along u1 variable
       *    with the u0 variable locked on the right u0
       */
      {
        real_T yLeftCast;
        real_T yRghtCast;
        yLeftCast = (rtConstP.pooled14[(iRghtU0+15U*iLeftU1)]);
        yRghtCast = (rtConstP.pooled14[(iRghtU0+15U*iRghtU1)]);
        yLeftCast += u1Lambda * ( yRghtCast - yLeftCast );
        yTemp = yLeftCast;
      }

      /*
       * Interpolate along u0 variable
       *    with the u1 variable locked on its interpolated value
       */
      {
        real_T yLeftCast;
        real_T yRghtCast;
        yLeftCast = rtDW.EMotorMappingRL;
        yRghtCast = yTemp;
        yLeftCast += u0Lambda * ( yRghtCast - yLeftCast );
        rtDW.EMotorMappingRL = yLeftCast;
      }
    }
  }

  /* Gain: '<S5>/Gain1' incorporates:
   *  Gain: '<S5>/Gain'
   *  Inport: '<Root>/whl_spd_rr'
   */
  rtb_Subtract1 = 13.15 * rtU.whl_spd_rr * 9.5492965855137211;

  /* Switch: '<S2>/Switch' incorporates:
   *  Inport: '<Root>/gas'
   */
  rtb_Switch_p = rtU.gas;

  /* Lookup2D: '<Root>/E-Motor Mapping RR' */
  /*
   * About '<Root>/E-Motor Mapping RR':
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   * Row Data    parameter uses the same data type and scaling as Input0
   * Column Data parameter uses the same data type and scaling as Input1
   * Table Data  parameter uses the same data type and scaling as Output0
   */
  {
    uint32_T iLeftU0, iRghtU0, iLeftU1, iRghtU1;

    /* Find the location of current input value in the data table.
     */
    if (rtb_Subtract1 <= 0.0 ) {
      /* Less than or equal to the smallest point in the table.
       */
      iLeftU0 = 0U;
      iRghtU0 = 0U;
    } else if (rtb_Subtract1 >= 14000.0 ) {
      /* Greater than or equal to the largest point in the table. */
      iLeftU0 = 14U;
      iRghtU0 = 14U;
    } else {
      /* The table is inlined and evenly spaced.
       * The index is found directly by use of division.
       */
      iLeftU0 = (uint32_T)(( rtb_Subtract1 ) / 1000.0);
      iRghtU0 = iLeftU0 + 1;
    }

    BINARYSEARCH_real_T( &(iLeftU1), &(iRghtU1), rtb_Switch_p, rtConstP.pooled13,
                        5U);

    {
      real_T yTemp;
      real_T u1Lambda;
      real_T u0Lambda;
      if ((rtConstP.pooled13[iRghtU1]) > (rtConstP.pooled13[iLeftU1]) ) {
        real_T num;
        real_T den;
        den = (rtConstP.pooled13[iRghtU1]);
        den -= (rtConstP.pooled13[iLeftU1]);
        num = rtb_Switch_p;
        num -= (rtConstP.pooled13[iLeftU1]);
        u1Lambda = num / den;
      } else {
        u1Lambda = 0.0;
      }

      if (iLeftU0 != iRghtU0 ) {
        {
          real_T num = (real_T)( rtb_Subtract1 ) - ( iLeftU0 * 1000.0 );
          u0Lambda = num / 1000.0;
        }
      } else {
        u0Lambda = 0.0;
      }

      /* Interpolate along u1 variable
       *    with the u0 variable locked on the left u0
       */
      {
        real_T yLeftCast;
        real_T yRghtCast;
        yLeftCast = (rtConstP.pooled14[(iLeftU0+15U*iLeftU1)]);
        yRghtCast = (rtConstP.pooled14[(iLeftU0+15U*iRghtU1)]);
        yLeftCast += u1Lambda * ( yRghtCast - yLeftCast );
        rtDW.EMotorMappingRR = yLeftCast;
      }

      /* Interpolate along u1 variable
       *    with the u0 variable locked on the right u0
       */
      {
        real_T yLeftCast;
        real_T yRghtCast;
        yLeftCast = (rtConstP.pooled14[(iRghtU0+15U*iLeftU1)]);
        yRghtCast = (rtConstP.pooled14[(iRghtU0+15U*iRghtU1)]);
        yLeftCast += u1Lambda * ( yRghtCast - yLeftCast );
        yTemp = yLeftCast;
      }

      /*
       * Interpolate along u0 variable
       *    with the u1 variable locked on its interpolated value
       */
      {
        real_T yLeftCast;
        real_T yRghtCast;
        yLeftCast = rtDW.EMotorMappingRR;
        yRghtCast = yTemp;
        yLeftCast += u0Lambda * ( yRghtCast - yLeftCast );
        rtDW.EMotorMappingRR = yLeftCast;
      }
    }
  }

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void vehicle_control_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for TransferFcn: '<Root>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += -66.666666666666671 * rtX.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += rtDW.EMotorMappingRL;

  /* Derivatives for Integrator: '<S55>/Integrator' */
  _rtXdot->Integrator_CSTATE = rtDW.IProdOut;

  /* Derivatives for Integrator: '<S50>/Filter' */
  _rtXdot->Filter_CSTATE = rtDW.NProdOut;

  /* Derivatives for TransferFcn: '<Root>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += -66.666666666666671 * rtX.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += rtDW.EMotorMappingRR;
}

/* Model initialize function */
void vehicle_control_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->intgData.f[3] = rtM->odeF[3];
  rtM->contStates = ((X *) &rtX);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode4");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.01;

  /* InitializeConditions for TransferFcn: '<Root>/Transfer Fcn' */
  rtX.TransferFcn_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S55>/Integrator' */
  rtX.Integrator_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S50>/Filter' */
  rtX.Filter_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<Root>/Transfer Fcn1' */
  rtX.TransferFcn1_CSTATE = 0.0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
