/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: vehicle_control.h
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

#ifndef RTW_HEADER_vehicle_control_h_
#define RTW_HEADER_vehicle_control_h_
#include "rtwtypes.h"
#include <string.h>
#ifndef vehicle_control_COMMON_INCLUDES_
#define vehicle_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* vehicle_control_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef DEFINED_TYPEDEF_FOR_D_gain_type_
#define DEFINED_TYPEDEF_FOR_D_gain_type_

typedef struct {
  real_T BP1_Vx[11];
  real_T BP2_a[11];
  real_T Table_D[121];
} D_gain_type;

#endif

#ifndef DEFINED_TYPEDEF_FOR_I_gain_type_
#define DEFINED_TYPEDEF_FOR_I_gain_type_

typedef struct {
  real_T BP1_Vx[11];
  real_T BP2_a[11];
  real_T Table_I[121];
} I_gain_type;

#endif

#ifndef DEFINED_TYPEDEF_FOR_P_gain_type_
#define DEFINED_TYPEDEF_FOR_P_gain_type_

typedef struct {
  real_T BP1_Vx[11];
  real_T BP2_a[11];
  real_T Table_P[121];
} P_gain_type;

#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T NProdOut;                     /* '<S58>/NProd Out' */
  real_T IProdOut;                     /* '<S52>/IProd Out' */
  real_T EMotorMappingRL;              /* '<Root>/E-Motor Mapping RL' */
  real_T EMotorMappingRR;              /* '<Root>/E-Motor Mapping RR' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<Root>/Transfer Fcn' */
  real_T Integrator_CSTATE;            /* '<S55>/Integrator' */
  real_T Filter_CSTATE;                /* '<S50>/Filter' */
  real_T TransferFcn1_CSTATE;          /* '<Root>/Transfer Fcn1' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<Root>/Transfer Fcn' */
  real_T Integrator_CSTATE;            /* '<S55>/Integrator' */
  real_T Filter_CSTATE;                /* '<S50>/Filter' */
  real_T TransferFcn1_CSTATE;          /* '<Root>/Transfer Fcn1' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn_CSTATE;        /* '<Root>/Transfer Fcn' */
  boolean_T Integrator_CSTATE;         /* '<S55>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S50>/Filter' */
  boolean_T TransferFcn1_CSTATE;       /* '<Root>/Transfer Fcn1' */
} XDis;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: P_gain_1.Table
   * Referenced by: '<S3>/P_gain_1'
   */
  real_T P_gain_1_tableData[121];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S3>/D_gain_1'
   *   '<S3>/I_gain_1'
   *   '<S3>/P_gain_1'
   */
  real_T pooled9[11];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S3>/D_gain_1'
   *   '<S3>/I_gain_1'
   *   '<S3>/P_gain_1'
   */
  real_T pooled10[11];

  /* Expression: D_gain_1.Table
   * Referenced by: '<S3>/D_gain_1'
   */
  real_T D_gain_1_tableData[121];

  /* Expression: I_gain_1.Table
   * Referenced by: '<S3>/I_gain_1'
   */
  real_T I_gain_1_tableData[121];

  /* Pooled Parameter (Expression: FISCHER_MotorCharacteristic(2:end,1))
   * Referenced by:
   *   '<Root>/E-Motor Mapping RL'
   *   '<Root>/E-Motor Mapping RR'
   */
  real_T pooled13[6];

  /* Pooled Parameter (Expression: FISCHER_MotorCharacteristic(2:end,2:end)')
   * Referenced by:
   *   '<Root>/E-Motor Mapping RL'
   *   '<Root>/E-Motor Mapping RR'
   */
  real_T pooled14[90];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S3>/D_gain_1'
   *   '<S3>/I_gain_1'
   *   '<S3>/P_gain_1'
   */
  uint32_T pooled15[2];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T gas;                          /* '<Root>/gas' */
  real_T whl_spd_rl;                   /* '<Root>/whl_spd_rl' */
  real_T whl_spd_rr;                   /* '<Root>/whl_spd_rr' */
  real_T v_x;                          /* '<Root>/v_x' */
  real_T a_y;                          /* '<Root>/a_y' */
  real_T steer_rl;                     /* '<Root>/steer_rl' */
  real_T steer_rr;                     /* '<Root>/steer_rr' */
  real_T yaw_rate;                     /* '<Root>/yaw_rate' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T trq_rl;                       /* '<Root>/trq_rl' */
  real_T trq_rr;                       /* '<Root>/trq_rr' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[4];
  real_T odeF[4][4];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Continuous states (default storage) */
extern X rtX;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void vehicle_control_initialize(void);
extern void vehicle_control_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S1>/Scope2' : Unused code path elimination
 * Block '<S13>/Scope' : Unused code path elimination
 * Block '<S13>/Scope1' : Unused code path elimination
 * Block '<S13>/Scope3' : Unused code path elimination
 * Block '<S13>/Scope5' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Propagation' : Unused code path elimination
 * Block '<S7>/Data Type Duplicate' : Unused code path elimination
 * Block '<S7>/Data Type Propagation' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<S8>/Scope' : Unused code path elimination
 * Block '<S9>/Scope' : Unused code path elimination
 * Block '<S10>/Scope' : Unused code path elimination
 * Block '<S11>/Scope' : Unused code path elimination
 * Block '<S12>/Scope' : Unused code path elimination
 * Block '<S12>/Scope4' : Unused code path elimination
 * Block '<Root>/Constant' : Unused code path elimination
 * Block '<Root>/Constant1' : Unused code path elimination
 * Block '<S13>/Add' : Unused code path elimination
 * Block '<S15>/Compare' : Unused code path elimination
 * Block '<S15>/Constant' : Unused code path elimination
 * Block '<S13>/Constant' : Unused code path elimination
 * Block '<S13>/Derivative' : Unused code path elimination
 * Block '<S13>/Fcn1' : Unused code path elimination
 * Block '<S13>/Fcn2' : Unused code path elimination
 * Block '<S13>/GreaterThan' : Unused code path elimination
 * Block '<S13>/Multiport Switch' : Unused code path elimination
 * Block '<S13>/Relational Operator' : Unused code path elimination
 * Block '<S13>/Relay' : Unused code path elimination
 * Block '<S16>/Logic' : Unused code path elimination
 * Block '<S16>/Memory' : Unused code path elimination
 * Block '<S13>/Saturation' : Unused code path elimination
 * Block '<S13>/Saturation1' : Unused code path elimination
 * Block '<S13>/Switch' : Unused code path elimination
 * Block '<S1>/Unit Delay' : Unused code path elimination
 * Block '<S14>/Add1' : Unused code path elimination
 * Block '<S14>/Constant' : Unused code path elimination
 * Block '<S14>/Divide' : Unused code path elimination
 * Block '<S14>/Gain' : Unused code path elimination
 * Block '<S14>/Gain1' : Unused code path elimination
 * Block '<S17>/Gain' : Unused code path elimination
 * Block '<S14>/Switch' : Unused code path elimination
 * Block '<S18>/Add' : Unused code path elimination
 * Block '<S20>/Compare' : Unused code path elimination
 * Block '<S20>/Constant' : Unused code path elimination
 * Block '<S18>/Constant' : Unused code path elimination
 * Block '<S18>/Derivative' : Unused code path elimination
 * Block '<S18>/Fcn1' : Unused code path elimination
 * Block '<S18>/Fcn2' : Unused code path elimination
 * Block '<S18>/GreaterThan' : Unused code path elimination
 * Block '<S18>/Multiport Switch' : Unused code path elimination
 * Block '<S18>/Relational Operator' : Unused code path elimination
 * Block '<S18>/Relay' : Unused code path elimination
 * Block '<S21>/Logic' : Unused code path elimination
 * Block '<S21>/Memory' : Unused code path elimination
 * Block '<S18>/Saturation' : Unused code path elimination
 * Block '<S18>/Saturation1' : Unused code path elimination
 * Block '<S18>/Switch' : Unused code path elimination
 * Block '<S2>/Unit Delay' : Unused code path elimination
 * Block '<S19>/Add1' : Unused code path elimination
 * Block '<S19>/Constant' : Unused code path elimination
 * Block '<S19>/Divide' : Unused code path elimination
 * Block '<S19>/Gain' : Unused code path elimination
 * Block '<S19>/Gain1' : Unused code path elimination
 * Block '<S22>/Gain' : Unused code path elimination
 * Block '<S19>/Switch' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'vehicle_control'
 * '<S1>'   : 'vehicle_control/Electronic Gas Pedal RL'
 * '<S2>'   : 'vehicle_control/Electronic Gas Pedal RR'
 * '<S3>'   : 'vehicle_control/PID'
 * '<S4>'   : 'vehicle_control/Rotation RL'
 * '<S5>'   : 'vehicle_control/Rotation RR'
 * '<S6>'   : 'vehicle_control/Saturation Dynamic'
 * '<S7>'   : 'vehicle_control/Saturation Dynamic1'
 * '<S8>'   : 'vehicle_control/Subsystem'
 * '<S9>'   : 'vehicle_control/Subsystem1'
 * '<S10>'  : 'vehicle_control/max regen_R'
 * '<S11>'  : 'vehicle_control/max_regen_L'
 * '<S12>'  : 'vehicle_control/yaw rate error'
 * '<S13>'  : 'vehicle_control/Electronic Gas Pedal RL/TC'
 * '<S14>'  : 'vehicle_control/Electronic Gas Pedal RL/slipratio'
 * '<S15>'  : 'vehicle_control/Electronic Gas Pedal RL/TC/Compare To Constant'
 * '<S16>'  : 'vehicle_control/Electronic Gas Pedal RL/TC/S-R Flip-Flop'
 * '<S17>'  : 'vehicle_control/Electronic Gas Pedal RL/slipratio/Radians to Degrees'
 * '<S18>'  : 'vehicle_control/Electronic Gas Pedal RR/Subsystem1'
 * '<S19>'  : 'vehicle_control/Electronic Gas Pedal RR/slipratio'
 * '<S20>'  : 'vehicle_control/Electronic Gas Pedal RR/Subsystem1/Compare To Constant'
 * '<S21>'  : 'vehicle_control/Electronic Gas Pedal RR/Subsystem1/S-R Flip-Flop'
 * '<S22>'  : 'vehicle_control/Electronic Gas Pedal RR/slipratio/Radians to Degrees'
 * '<S23>'  : 'vehicle_control/PID/60 kph controller1'
 * '<S24>'  : 'vehicle_control/PID/60 kph controller1/Anti-windup'
 * '<S25>'  : 'vehicle_control/PID/60 kph controller1/D Gain'
 * '<S26>'  : 'vehicle_control/PID/60 kph controller1/Filter'
 * '<S27>'  : 'vehicle_control/PID/60 kph controller1/Filter ICs'
 * '<S28>'  : 'vehicle_control/PID/60 kph controller1/I Gain'
 * '<S29>'  : 'vehicle_control/PID/60 kph controller1/Ideal P Gain'
 * '<S30>'  : 'vehicle_control/PID/60 kph controller1/Ideal P Gain Fdbk'
 * '<S31>'  : 'vehicle_control/PID/60 kph controller1/Integrator'
 * '<S32>'  : 'vehicle_control/PID/60 kph controller1/Integrator ICs'
 * '<S33>'  : 'vehicle_control/PID/60 kph controller1/N Copy'
 * '<S34>'  : 'vehicle_control/PID/60 kph controller1/N Gain'
 * '<S35>'  : 'vehicle_control/PID/60 kph controller1/P Copy'
 * '<S36>'  : 'vehicle_control/PID/60 kph controller1/Parallel P Gain'
 * '<S37>'  : 'vehicle_control/PID/60 kph controller1/Reset Signal'
 * '<S38>'  : 'vehicle_control/PID/60 kph controller1/Saturation'
 * '<S39>'  : 'vehicle_control/PID/60 kph controller1/Saturation Fdbk'
 * '<S40>'  : 'vehicle_control/PID/60 kph controller1/Sum'
 * '<S41>'  : 'vehicle_control/PID/60 kph controller1/Sum Fdbk'
 * '<S42>'  : 'vehicle_control/PID/60 kph controller1/Tracking Mode'
 * '<S43>'  : 'vehicle_control/PID/60 kph controller1/Tracking Mode Sum'
 * '<S44>'  : 'vehicle_control/PID/60 kph controller1/Tsamp - Integral'
 * '<S45>'  : 'vehicle_control/PID/60 kph controller1/Tsamp - Ngain'
 * '<S46>'  : 'vehicle_control/PID/60 kph controller1/postSat Signal'
 * '<S47>'  : 'vehicle_control/PID/60 kph controller1/preSat Signal'
 * '<S48>'  : 'vehicle_control/PID/60 kph controller1/Anti-windup/Passthrough'
 * '<S49>'  : 'vehicle_control/PID/60 kph controller1/D Gain/External Parameters'
 * '<S50>'  : 'vehicle_control/PID/60 kph controller1/Filter/Cont. Filter'
 * '<S51>'  : 'vehicle_control/PID/60 kph controller1/Filter ICs/Internal IC - Filter'
 * '<S52>'  : 'vehicle_control/PID/60 kph controller1/I Gain/External Parameters'
 * '<S53>'  : 'vehicle_control/PID/60 kph controller1/Ideal P Gain/Passthrough'
 * '<S54>'  : 'vehicle_control/PID/60 kph controller1/Ideal P Gain Fdbk/Disabled'
 * '<S55>'  : 'vehicle_control/PID/60 kph controller1/Integrator/Continuous'
 * '<S56>'  : 'vehicle_control/PID/60 kph controller1/Integrator ICs/Internal IC'
 * '<S57>'  : 'vehicle_control/PID/60 kph controller1/N Copy/Disabled'
 * '<S58>'  : 'vehicle_control/PID/60 kph controller1/N Gain/External Parameters'
 * '<S59>'  : 'vehicle_control/PID/60 kph controller1/P Copy/Disabled'
 * '<S60>'  : 'vehicle_control/PID/60 kph controller1/Parallel P Gain/External Parameters'
 * '<S61>'  : 'vehicle_control/PID/60 kph controller1/Reset Signal/Disabled'
 * '<S62>'  : 'vehicle_control/PID/60 kph controller1/Saturation/Passthrough'
 * '<S63>'  : 'vehicle_control/PID/60 kph controller1/Saturation Fdbk/Disabled'
 * '<S64>'  : 'vehicle_control/PID/60 kph controller1/Sum/Sum_PID'
 * '<S65>'  : 'vehicle_control/PID/60 kph controller1/Sum Fdbk/Disabled'
 * '<S66>'  : 'vehicle_control/PID/60 kph controller1/Tracking Mode/Disabled'
 * '<S67>'  : 'vehicle_control/PID/60 kph controller1/Tracking Mode Sum/Passthrough'
 * '<S68>'  : 'vehicle_control/PID/60 kph controller1/Tsamp - Integral/Passthrough'
 * '<S69>'  : 'vehicle_control/PID/60 kph controller1/Tsamp - Ngain/Passthrough'
 * '<S70>'  : 'vehicle_control/PID/60 kph controller1/postSat Signal/Forward_Path'
 * '<S71>'  : 'vehicle_control/PID/60 kph controller1/preSat Signal/Forward_Path'
 * '<S72>'  : 'vehicle_control/yaw rate error/reference yaw rate 1'
 */
#endif                                 /* RTW_HEADER_vehicle_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
