/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: adcs_full_loop_codegen.c
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

#include "adcs_full_loop_codegen.h"
#include <math.h>
#include <emmintrin.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_adcs_full_loop_codegen_T adcs_full_loop_codegen_DW;

/* External inputs (root inport signals with default storage) */
ExtU_adcs_full_loop_codegen_T adcs_full_loop_codegen_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_adcs_full_loop_codegen_T adcs_full_loop_codegen_Y;

/* Real-time model */
static RT_MODEL_adcs_full_loop_codeg_T adcs_full_loop_codegen_M_;
RT_MODEL_adcs_full_loop_codeg_T *const adcs_full_loop_codegen_M =
  &adcs_full_loop_codegen_M_;

/* Model step function */
void adcs_full_loop_codegen_step(void)
{
  __m128d tmp_1;
  __m128d tmp_2;
  real_T tmp[16];
  real_T rtb_q_n[4];
  real_T rtb_qe_vec[3];
  real_T rtb_tau_body[3];
  real_T rv[3];
  real_T tmp_0[2];
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T i;
  int32_T tmp_3;
  static const real_T a[9] = { 0.04187, 0.0, 0.0, 0.0, 0.04187, 0.0, 0.0, 0.0,
    0.006667 };

  /* MATLAB Function: '<Root>/Quaternion Normalize' incorporates:
   *  DiscreteIntegrator: '<Root>/Q Integrator'
   */
  scale = 3.3121686421112381E-170;
  absxk = fabs(adcs_full_loop_codegen_DW.QIntegrator_DSTATE[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(adcs_full_loop_codegen_DW.QIntegrator_DSTATE[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(adcs_full_loop_codegen_DW.QIntegrator_DSTATE[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(adcs_full_loop_codegen_DW.QIntegrator_DSTATE[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  tmp_2 = _mm_set1_pd(scale * sqrt(y));

  /* DiscreteIntegrator: '<Root>/Q Integrator' */
  tmp_1 = _mm_div_pd(_mm_loadu_pd(&adcs_full_loop_codegen_DW.QIntegrator_DSTATE
    [0]), tmp_2);

  /* MATLAB Function: '<Root>/Quaternion Normalize' */
  _mm_storeu_pd(&rtb_q_n[0], tmp_1);

  /* DiscreteIntegrator: '<Root>/Q Integrator' */
  tmp_2 = _mm_div_pd(_mm_loadu_pd(&adcs_full_loop_codegen_DW.QIntegrator_DSTATE
    [2]), tmp_2);

  /* MATLAB Function: '<Root>/Quaternion Normalize' */
  _mm_storeu_pd(&rtb_q_n[2], tmp_2);

  /* Outport: '<Root>/omega_out' incorporates:
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   */
  adcs_full_loop_codegen_Y.omega_out[0] =
    adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0];

  /* MATLAB Function: '<Root>/Quaternion Error' incorporates:
   *  Inport: '<Root>/q_target_in'
   */
  rv[0] = -adcs_full_loop_codegen_U.q_target_in[1];

  /* Outport: '<Root>/omega_out' incorporates:
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   */
  adcs_full_loop_codegen_Y.omega_out[1] =
    adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1];

  /* MATLAB Function: '<Root>/Quaternion Error' incorporates:
   *  Inport: '<Root>/q_target_in'
   */
  rv[1] = -adcs_full_loop_codegen_U.q_target_in[2];

  /* Outport: '<Root>/omega_out' incorporates:
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   */
  adcs_full_loop_codegen_Y.omega_out[2] =
    adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2];

  /* MATLAB Function: '<Root>/Quaternion Error' incorporates:
   *  Inport: '<Root>/q_target_in'
   *  MATLAB Function: '<Root>/Attitude Error Angle'
   */
  rv[2] = -adcs_full_loop_codegen_U.q_target_in[3];
  tmp_2 = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd
    (adcs_full_loop_codegen_U.q_target_in[0]), _mm_loadu_pd(&rtb_q_n[1])),
    _mm_mul_pd(_mm_set1_pd(rtb_q_n[0]), _mm_set_pd
               (-adcs_full_loop_codegen_U.q_target_in[2],
                -adcs_full_loop_codegen_U.q_target_in[1]))), _mm_sub_pd
                     (_mm_mul_pd(_mm_set_pd(rtb_q_n[1],
    -adcs_full_loop_codegen_U.q_target_in[2]), _mm_set_pd
    (-adcs_full_loop_codegen_U.q_target_in[3], rtb_q_n[3])), _mm_mul_pd
                      (_mm_set_pd(-adcs_full_loop_codegen_U.q_target_in[1],
    rtb_q_n[2]), _mm_set_pd(rtb_q_n[3], -adcs_full_loop_codegen_U.q_target_in[3]))));
  _mm_storeu_pd(&rtb_qe_vec[0], tmp_2);
  rtb_qe_vec[2] = (adcs_full_loop_codegen_U.q_target_in[0] * rtb_q_n[3] +
                   rtb_q_n[0] * -adcs_full_loop_codegen_U.q_target_in[3]) +
    (-adcs_full_loop_codegen_U.q_target_in[1] * rtb_q_n[2] - rtb_q_n[1] *
     -adcs_full_loop_codegen_U.q_target_in[2]);
  scale = adcs_full_loop_codegen_U.q_target_in[0] * rtb_q_n[0];
  if (scale - ((-adcs_full_loop_codegen_U.q_target_in[1] * rtb_q_n[1] +
                -adcs_full_loop_codegen_U.q_target_in[2] * rtb_q_n[2]) +
               -adcs_full_loop_codegen_U.q_target_in[3] * rtb_q_n[3]) < 0.0) {
    rtb_qe_vec[0] = -rtb_qe_vec[0];
    rtb_qe_vec[1] = -rtb_qe_vec[1];
    rtb_qe_vec[2] = -rtb_qe_vec[2];
  }

  /* MATLAB Function: '<Root>/Attitude Error Angle' */
  absxk = 0.0;
  for (i = 0; i < 3; i++) {
    /* MATLAB Function: '<Root>/Reaction Wheel' incorporates:
     *  DiscreteIntegrator: '<Root>/Omega Integrator'
     *  MATLAB Function: '<Root>/PD Control'
     */
    t = fmax(fmin(-(-0.003 * rtb_qe_vec[i] - 0.012 *
                    adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[i]), 0.0006),
             -0.0006);
    rtb_tau_body[i] = t;

    /* DiscreteIntegrator: '<Root>/H_wheel Integrator' incorporates:
     *  MATLAB Function: '<Root>/Reaction Wheel'
     */
    y = adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[i];

    /* MATLAB Function: '<Root>/Reaction Wheel' incorporates:
     *  DiscreteIntegrator: '<Root>/H_wheel Integrator'
     */
    if ((y >= 0.01) && (t > 0.0)) {
      rtb_tau_body[i] = 0.0;
    } else if ((y <= -0.01) && (t < 0.0)) {
      rtb_tau_body[i] = 0.0;
    }

    /* MATLAB Function: '<Root>/Rigid Body Dynamics' incorporates:
     *  DiscreteIntegrator: '<Root>/H_wheel Integrator'
     *  DiscreteIntegrator: '<Root>/Omega Integrator'
     */
    rtb_qe_vec[i] = ((a[i + 3] *
                      adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1] + a[i]
                      * adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0]) +
                     a[i + 6] *
                     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2]) + y;

    /* Outport: '<Root>/h_wheel_out' incorporates:
     *  DiscreteIntegrator: '<Root>/H_wheel Integrator'
     */
    adcs_full_loop_codegen_Y.h_wheel_out[i] = y;

    /* MATLAB Function: '<Root>/Attitude Error Angle' */
    absxk += rtb_q_n[i + 1] * rv[i];
  }

  /* MATLAB Function: '<Root>/Rigid Body Dynamics' incorporates:
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   *  MATLAB Function: '<Root>/Reaction Wheel'
   */
  tmp_2 = _mm_sub_pd(_mm_set_pd(-(rtb_qe_vec[0] *
    adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2] -
    adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0] * rtb_qe_vec[2]),
    -(adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1] * rtb_qe_vec[2] -
      rtb_qe_vec[1] * adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2])),
                     _mm_loadu_pd(&rtb_tau_body[0]));
  _mm_storeu_pd(&tmp_0[0], tmp_2);

  /* MATLAB Function: '<Root>/Rigid Body Dynamics' incorporates:
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   *  MATLAB Function: '<Root>/Reaction Wheel'
   */
  t = -(adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0] * rtb_qe_vec[1] -
        rtb_qe_vec[0] * adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1]) -
    rtb_tau_body[2];
  y = tmp_0[0] * 0.0;
  rtb_qe_vec[1] = tmp_0[1] - y;
  rtb_qe_vec[2] = (t - y) - rtb_qe_vec[1] * 0.0;
  rtb_qe_vec[2] /= 0.006667;
  _mm_storeu_pd(&rtb_qe_vec[0], _mm_sub_pd(_mm_set_pd(rtb_qe_vec[1], tmp_0[0]),
    _mm_mul_pd(_mm_set_pd(rtb_qe_vec[2], 0.0), _mm_set_pd(0.0, rtb_qe_vec[2]))));
  rtb_qe_vec[1] /= 0.04187;
  rtb_qe_vec[0] -= 0.0 * rtb_qe_vec[1];
  rtb_qe_vec[0] /= 0.04187;

  /* MATLAB Function: '<Root>/Attitude Error Angle' */
  scale -= absxk;
  if (scale < 0.0) {
    scale = -scale;
  }

  /* Outport: '<Root>/err_deg_out' incorporates:
   *  MATLAB Function: '<Root>/Attitude Error Angle'
   */
  adcs_full_loop_codegen_Y.err_deg_out = 57.295779513082323 * acos(fmin(fmax
    (scale, -1.0), 1.0)) * 2.0;

  /* MATLAB Function: '<Root>/Kinematics' incorporates:
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   */
  tmp[0] = 0.0;
  tmp_2 = _mm_set1_pd(0.5);
  _mm_storeu_pd(&tmp_0[0], _mm_mul_pd(tmp_2, _mm_set_pd
    (-adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1],
     -adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0])));
  tmp[4] = tmp_0[0];
  tmp[8] = tmp_0[1];
  _mm_storeu_pd(&tmp_0[0], _mm_mul_pd(tmp_2, _mm_set_pd
    (adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0],
     -adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2])));
  tmp[12] = tmp_0[0];
  tmp[1] = tmp_0[1];
  tmp[5] = 0.0;
  _mm_storeu_pd(&tmp_0[0], _mm_mul_pd(tmp_2, _mm_set_pd
    (-adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2])));
  tmp[9] = tmp_0[0];
  tmp[13] = tmp_0[1];
  _mm_storeu_pd(&tmp_0[0], _mm_mul_pd(tmp_2, _mm_set_pd
    (-adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1])));
  tmp[2] = tmp_0[0];
  tmp[6] = tmp_0[1];
  tmp[10] = 0.0;
  _mm_storeu_pd(&tmp_0[0], _mm_mul_pd(tmp_2, _mm_set_pd
    (adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0])));
  tmp[14] = tmp_0[0];
  tmp[3] = tmp_0[1];
  _mm_storeu_pd(&tmp_0[0], _mm_mul_pd(tmp_2, _mm_set_pd
    (-adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1])));
  tmp[7] = tmp_0[0];
  tmp[11] = tmp_0[1];
  tmp[15] = 0.0;
  scale = 0.0;
  absxk = 0.0;
  t = 0.0;
  y = 0.0;
  for (i = 0; i < 4; i++) {
    tmp_3 = i << 2;
    tmp_2 = _mm_set1_pd(rtb_q_n[i]);
    tmp_1 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&tmp[tmp_3]), tmp_2), _mm_set_pd
                       (absxk, scale));
    _mm_storeu_pd(&tmp_0[0], tmp_1);
    scale = tmp_0[0];
    absxk = tmp_0[1];
    tmp_2 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&tmp[tmp_3 + 2]), tmp_2),
                       _mm_set_pd(y, t));
    _mm_storeu_pd(&tmp_0[0], tmp_2);
    t = tmp_0[0];
    y = tmp_0[1];
  }

  /* Update for DiscreteIntegrator: '<Root>/Q Integrator' */
  tmp_2 = _mm_set1_pd(0.01);

  /* MATLAB Function: '<Root>/Kinematics' incorporates:
   *  DiscreteIntegrator: '<Root>/Q Integrator'
   */
  tmp_1 = _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set_pd(absxk, scale)), _mm_loadu_pd
                     (&adcs_full_loop_codegen_DW.QIntegrator_DSTATE[0]));

  /* Update for DiscreteIntegrator: '<Root>/Q Integrator' */
  _mm_storeu_pd(&adcs_full_loop_codegen_DW.QIntegrator_DSTATE[0], tmp_1);

  /* MATLAB Function: '<Root>/Kinematics' incorporates:
   *  DiscreteIntegrator: '<Root>/Q Integrator'
   */
  tmp_1 = _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set_pd(y, t)), _mm_loadu_pd
                     (&adcs_full_loop_codegen_DW.QIntegrator_DSTATE[2]));

  /* Update for DiscreteIntegrator: '<Root>/Q Integrator' */
  _mm_storeu_pd(&adcs_full_loop_codegen_DW.QIntegrator_DSTATE[2], tmp_1);

  /* MATLAB Function: '<Root>/Reaction Wheel' incorporates:
   *  DiscreteIntegrator: '<Root>/H_wheel Integrator'
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   *  MATLAB Function: '<Root>/Rigid Body Dynamics'
   */
  _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set_pd(rtb_tau_body
    [0], rtb_qe_vec[0])), _mm_set_pd
    (adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[0],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0])));

  /* Update for DiscreteIntegrator: '<Root>/Omega Integrator' */
  adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0] = tmp_0[0];

  /* Update for DiscreteIntegrator: '<Root>/H_wheel Integrator' */
  adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[0] = tmp_0[1];

  /* MATLAB Function: '<Root>/Reaction Wheel' incorporates:
   *  DiscreteIntegrator: '<Root>/H_wheel Integrator'
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   *  MATLAB Function: '<Root>/Rigid Body Dynamics'
   */
  _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set_pd(rtb_tau_body
    [1], rtb_qe_vec[1])), _mm_set_pd
    (adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[1],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1])));

  /* Update for DiscreteIntegrator: '<Root>/Omega Integrator' */
  adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1] = tmp_0[0];

  /* Update for DiscreteIntegrator: '<Root>/H_wheel Integrator' */
  adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[1] = tmp_0[1];

  /* MATLAB Function: '<Root>/Reaction Wheel' incorporates:
   *  DiscreteIntegrator: '<Root>/H_wheel Integrator'
   *  DiscreteIntegrator: '<Root>/Omega Integrator'
   *  MATLAB Function: '<Root>/Rigid Body Dynamics'
   */
  _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_mul_pd(tmp_2, _mm_set_pd(rtb_tau_body
    [2], rtb_qe_vec[2])), _mm_set_pd
    (adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[2],
     adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2])));

  /* Update for DiscreteIntegrator: '<Root>/Omega Integrator' */
  adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2] = tmp_0[0];

  /* Update for DiscreteIntegrator: '<Root>/H_wheel Integrator' */
  adcs_full_loop_codegen_DW.H_wheelIntegrator_DSTATE[2] = tmp_0[1];
}

/* Model initialize function */
void adcs_full_loop_codegen_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<Root>/Q Integrator' */
  adcs_full_loop_codegen_DW.QIntegrator_DSTATE[0] = 1.0;
  adcs_full_loop_codegen_DW.QIntegrator_DSTATE[1] = 0.0;
  adcs_full_loop_codegen_DW.QIntegrator_DSTATE[2] = 0.0;
  adcs_full_loop_codegen_DW.QIntegrator_DSTATE[3] = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Omega Integrator' */
  adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[0] = 0.02;
  adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[1] = -0.015;
  adcs_full_loop_codegen_DW.OmegaIntegrator_DSTATE[2] = 0.025;
}

/* Model terminate function */
void adcs_full_loop_codegen_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
