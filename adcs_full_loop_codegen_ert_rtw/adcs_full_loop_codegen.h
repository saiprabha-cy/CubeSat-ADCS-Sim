/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: adcs_full_loop_codegen.h
 *
 * Code generated for Simulink model 'adcs_full_loop_codegen'.
 *
 * Model version                  : 1.13
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Wed Sep  9 17:34:35 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef adcs_full_loop_codegen_h_
#define adcs_full_loop_codegen_h_
#ifndef adcs_full_loop_codegen_COMMON_INCLUDES_
#define adcs_full_loop_codegen_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                             /* adcs_full_loop_codegen_COMMON_INCLUDES_ */

#include "adcs_full_loop_codegen_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T QIntegrator_DSTATE[4];        /* '<Root>/Q Integrator' */
  real_T OmegaIntegrator_DSTATE[3];    /* '<Root>/Omega Integrator' */
  real_T H_wheelIntegrator_DSTATE[3];  /* '<Root>/H_wheel Integrator' */
} DW_adcs_full_loop_codegen_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T q_target_in[4];               /* '<Root>/q_target_in' */
} ExtU_adcs_full_loop_codegen_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T err_deg_out;                  /* '<Root>/err_deg_out' */
  real_T omega_out[3];                 /* '<Root>/omega_out' */
  real_T h_wheel_out[3];               /* '<Root>/h_wheel_out' */
} ExtY_adcs_full_loop_codegen_T;

/* Real-time Model Data Structure */
struct tag_RTM_adcs_full_loop_codege_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_adcs_full_loop_codegen_T adcs_full_loop_codegen_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_adcs_full_loop_codegen_T adcs_full_loop_codegen_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_adcs_full_loop_codegen_T adcs_full_loop_codegen_Y;

/* Model entry point functions */
extern void adcs_full_loop_codegen_initialize(void);
extern void adcs_full_loop_codegen_step(void);
extern void adcs_full_loop_codegen_terminate(void);

/* Real-time Model object */
extern RT_MODEL_adcs_full_loop_codeg_T *const adcs_full_loop_codegen_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<Root>/Display1' : Unused code path elimination
 * Block '<Root>/Display2' : Unused code path elimination
 * Block '<Root>/Error Scope' : Unused code path elimination
 * Block '<Root>/H_wheel Scope' : Unused code path elimination
 * Block '<Root>/Omega Scope' : Unused code path elimination
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
 * '<Root>' : 'adcs_full_loop_codegen'
 * '<S1>'   : 'adcs_full_loop_codegen/Attitude Error Angle'
 * '<S2>'   : 'adcs_full_loop_codegen/Kinematics'
 * '<S3>'   : 'adcs_full_loop_codegen/PD Control'
 * '<S4>'   : 'adcs_full_loop_codegen/Quaternion Error'
 * '<S5>'   : 'adcs_full_loop_codegen/Quaternion Normalize'
 * '<S6>'   : 'adcs_full_loop_codegen/Reaction Wheel'
 * '<S7>'   : 'adcs_full_loop_codegen/Rigid Body Dynamics'
 */
#endif                                 /* adcs_full_loop_codegen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
