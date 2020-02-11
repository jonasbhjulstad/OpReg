/*
 * Assignment_2.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Assignment_2".
 *
 * Model version              : 1.178
 * Simulink Coder version : 8.9 (R2015b) 13-Aug-2015
 * C source code generated on : Tue Feb 04 15:14:36 2020
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Assignment_2.h"
#include "Assignment_2_private.h"
#include "Assignment_2_dt.h"

/* Block signals (auto storage) */
B_Assignment_2_T Assignment_2_B;

/* Continuous states */
X_Assignment_2_T Assignment_2_X;

/* Block states (auto storage) */
DW_Assignment_2_T Assignment_2_DW;

/* Real-time model */
RT_MODEL_Assignment_2_T Assignment_2_M_;
RT_MODEL_Assignment_2_T *const Assignment_2_M = &Assignment_2_M_;

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  Assignment_2_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void Assignment_2_output(void)
{
  /* local block i/o variables */
  real_T rtb_HILReadEncoderTimebase_o1;
  real_T rtb_HILReadEncoderTimebase_o2;
  real_T rtb_HILReadEncoderTimebase_o3;
  real_T *lastU;
  real_T rtb_Backgain;
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
    /* set solver stop time */
    if (!(Assignment_2_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Assignment_2_M->solverInfo,
                            ((Assignment_2_M->Timing.clockTickH0 + 1) *
        Assignment_2_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Assignment_2_M->solverInfo,
                            ((Assignment_2_M->Timing.clockTick0 + 1) *
        Assignment_2_M->Timing.stepSize0 + Assignment_2_M->Timing.clockTickH0 *
        Assignment_2_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Assignment_2_M)) {
    Assignment_2_M->Timing.t[0] = rtsiGetT(&Assignment_2_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(Assignment_2_M)) {
    /* S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

    /* S-Function Block: Assignment_2/Helicopter_interface/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
    {
      t_error result;
      result = hil_task_read_encoder(Assignment_2_DW.HILReadEncoderTimebase_Task,
        1, &Assignment_2_DW.HILReadEncoderTimebase_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
      } else {
        rtb_HILReadEncoderTimebase_o1 =
          Assignment_2_DW.HILReadEncoderTimebase_Buffer[0];
        rtb_HILReadEncoderTimebase_o2 =
          Assignment_2_DW.HILReadEncoderTimebase_Buffer[1];
        rtb_HILReadEncoderTimebase_o3 =
          Assignment_2_DW.HILReadEncoderTimebase_Buffer[2];
      }
    }

    /* Gain: '<S4>/Travel: Count to rad' incorporates:
     *  Gain: '<S4>/Travel_gain'
     */
    Assignment_2_B.TravelCounttorad = Assignment_2_P.travel_gain *
      rtb_HILReadEncoderTimebase_o1 * Assignment_2_P.TravelCounttorad_Gain;

    /* Gain: '<S11>/Gain' */
    Assignment_2_B.Gain = Assignment_2_P.Gain_Gain *
      Assignment_2_B.TravelCounttorad;

    /* Sum: '<Root>/Sum3' incorporates:
     *  Constant: '<Root>/travel_offset [rad]'
     */
    Assignment_2_B.Sum3 = Assignment_2_P.travel_offsetrad_Value +
      Assignment_2_B.Gain;

    /* Gain: '<S4>/Pitch: Count to rad' */
    Assignment_2_B.PitchCounttorad = Assignment_2_P.PitchCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o2;

    /* Gain: '<S8>/Gain' */
    Assignment_2_B.Gain_i = Assignment_2_P.Gain_Gain_a *
      Assignment_2_B.PitchCounttorad;

    /* Sum: '<Root>/Sum4' incorporates:
     *  Constant: '<Root>/pitch_offset [rad]1'
     */
    Assignment_2_B.Sum4 = Assignment_2_P.pitch_offsetrad1_Value +
      Assignment_2_B.Gain_i;
  }

  /* Gain: '<S12>/Gain' incorporates:
   *  TransferFcn: '<S4>/Travel: Transfer Fcn'
   */
  Assignment_2_B.Gain_d = (Assignment_2_P.TravelTransferFcn_C *
    Assignment_2_X.TravelTransferFcn_CSTATE + Assignment_2_P.TravelTransferFcn_D
    * Assignment_2_B.TravelCounttorad) * Assignment_2_P.Gain_Gain_l;

  /* Gain: '<S9>/Gain' incorporates:
   *  TransferFcn: '<S4>/Pitch: Transfer Fcn'
   */
  Assignment_2_B.Gain_b = (Assignment_2_P.PitchTransferFcn_C *
    Assignment_2_X.PitchTransferFcn_CSTATE + Assignment_2_P.PitchTransferFcn_D *
    Assignment_2_B.PitchCounttorad) * Assignment_2_P.Gain_Gain_ae;
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
    /* Gain: '<S4>/Elevation: Count to rad' incorporates:
     *  Gain: '<S4>/Elevation_gain'
     */
    Assignment_2_B.ElevationCounttorad = Assignment_2_P.elevation_gain *
      rtb_HILReadEncoderTimebase_o3 * Assignment_2_P.ElevationCounttorad_Gain;

    /* Gain: '<S6>/Gain' */
    Assignment_2_B.Gain_e = Assignment_2_P.Gain_Gain_lv *
      Assignment_2_B.ElevationCounttorad;

    /* Sum: '<Root>/Sum' incorporates:
     *  Constant: '<Root>/elavation_offset [deg]'
     */
    Assignment_2_B.Sum = Assignment_2_B.Gain_e +
      Assignment_2_P.elavation_offsetdeg_Value;
  }

  /* Gain: '<S7>/Gain' incorporates:
   *  TransferFcn: '<S4>/Elevation: Transfer Fcn'
   */
  Assignment_2_B.Gain_dg = (Assignment_2_P.ElevationTransferFcn_C *
    Assignment_2_X.ElevationTransferFcn_CSTATE +
    Assignment_2_P.ElevationTransferFcn_D * Assignment_2_B.ElevationCounttorad) *
    Assignment_2_P.Gain_Gain_n;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/Vd_bias'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S5>/K_pd'
   *  Gain: '<S5>/K_pp'
   *  Sum: '<S5>/Sum2'
   *  Sum: '<S5>/Sum3'
   */
  Assignment_2_B.Sum1 = ((Assignment_2_P.Constant_Value -
    Assignment_2_P.Gain1_Gain * Assignment_2_B.Sum4) * Assignment_2_P.K_pp -
    Assignment_2_P.Gain1_Gain * Assignment_2_B.Gain_b * Assignment_2_P.K_pd) +
    Assignment_2_P.Vd_ff;
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
  }

  /* Integrator: '<S3>/Integrator' */
  /* Limited  Integrator  */
  if (Assignment_2_X.Integrator_CSTATE >= Assignment_2_P.Integrator_UpperSat) {
    Assignment_2_X.Integrator_CSTATE = Assignment_2_P.Integrator_UpperSat;
  } else {
    if (Assignment_2_X.Integrator_CSTATE <= Assignment_2_P.Integrator_LowerSat)
    {
      Assignment_2_X.Integrator_CSTATE = Assignment_2_P.Integrator_LowerSat;
    }
  }

  /* Sum: '<S3>/Sum' incorporates:
   *  Constant: '<Root>/elevation_ref'
   *  Gain: '<S2>/Gain1'
   */
  rtb_Backgain = Assignment_2_P.elevation_ref_Value - Assignment_2_P.Gain1_Gain *
    Assignment_2_B.Sum;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Constant: '<Root>/Vs_bias'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S3>/K_ed'
   *  Gain: '<S3>/K_ep'
   *  Integrator: '<S3>/Integrator'
   *  Sum: '<S3>/Sum1'
   */
  Assignment_2_B.Sum2 = ((Assignment_2_P.K_ep * rtb_Backgain +
    Assignment_2_X.Integrator_CSTATE) - Assignment_2_P.Gain1_Gain *
    Assignment_2_B.Gain_dg * Assignment_2_P.K_ed) + Assignment_2_P.Vs_ff;
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
  }

  /* Gain: '<S3>/K_ei' */
  Assignment_2_B.K_ei = Assignment_2_P.K_ei * rtb_Backgain;
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
  }

  /* Derivative: '<S4>/Derivative' */
  if ((Assignment_2_DW.TimeStampA >= Assignment_2_M->Timing.t[0]) &&
      (Assignment_2_DW.TimeStampB >= Assignment_2_M->Timing.t[0])) {
    rtb_Backgain = 0.0;
  } else {
    rtb_Backgain = Assignment_2_DW.TimeStampA;
    lastU = &Assignment_2_DW.LastUAtTimeA;
    if (Assignment_2_DW.TimeStampA < Assignment_2_DW.TimeStampB) {
      if (Assignment_2_DW.TimeStampB < Assignment_2_M->Timing.t[0]) {
        rtb_Backgain = Assignment_2_DW.TimeStampB;
        lastU = &Assignment_2_DW.LastUAtTimeB;
      }
    } else {
      if (Assignment_2_DW.TimeStampA >= Assignment_2_M->Timing.t[0]) {
        rtb_Backgain = Assignment_2_DW.TimeStampB;
        lastU = &Assignment_2_DW.LastUAtTimeB;
      }
    }

    rtb_Backgain = (Assignment_2_B.PitchCounttorad - *lastU) /
      (Assignment_2_M->Timing.t[0] - rtb_Backgain);
  }

  /* End of Derivative: '<S4>/Derivative' */

  /* Gain: '<S10>/Gain' */
  Assignment_2_B.Gain_l = Assignment_2_P.Gain_Gain_a1 * rtb_Backgain;
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
  }

  /* Gain: '<S1>/Back gain' incorporates:
   *  Sum: '<S1>/Subtract'
   */
  rtb_Backgain = (Assignment_2_B.Sum2 - Assignment_2_B.Sum1) *
    Assignment_2_P.Backgain_Gain;

  /* Saturate: '<S4>/Back motor: Saturation' */
  if (rtb_Backgain > Assignment_2_P.BackmotorSaturation_UpperSat) {
    Assignment_2_B.BackmotorSaturation =
      Assignment_2_P.BackmotorSaturation_UpperSat;
  } else if (rtb_Backgain < Assignment_2_P.BackmotorSaturation_LowerSat) {
    Assignment_2_B.BackmotorSaturation =
      Assignment_2_P.BackmotorSaturation_LowerSat;
  } else {
    Assignment_2_B.BackmotorSaturation = rtb_Backgain;
  }

  /* End of Saturate: '<S4>/Back motor: Saturation' */
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
  }

  /* Gain: '<S1>/Front gain' incorporates:
   *  Sum: '<S1>/Add'
   */
  rtb_Backgain = (Assignment_2_B.Sum1 + Assignment_2_B.Sum2) *
    Assignment_2_P.Frontgain_Gain;

  /* Saturate: '<S4>/Front motor: Saturation' */
  if (rtb_Backgain > Assignment_2_P.FrontmotorSaturation_UpperSat) {
    Assignment_2_B.FrontmotorSaturation =
      Assignment_2_P.FrontmotorSaturation_UpperSat;
  } else if (rtb_Backgain < Assignment_2_P.FrontmotorSaturation_LowerSat) {
    Assignment_2_B.FrontmotorSaturation =
      Assignment_2_P.FrontmotorSaturation_LowerSat;
  } else {
    Assignment_2_B.FrontmotorSaturation = rtb_Backgain;
  }

  /* End of Saturate: '<S4>/Front motor: Saturation' */
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
    /* S-Function (hil_write_analog_block): '<S4>/HIL Write Analog' */

    /* S-Function Block: Assignment_2/Helicopter_interface/HIL Write Analog (hil_write_analog_block) */
    {
      t_error result;
      Assignment_2_DW.HILWriteAnalog_Buffer[0] =
        Assignment_2_B.FrontmotorSaturation;
      Assignment_2_DW.HILWriteAnalog_Buffer[1] =
        Assignment_2_B.BackmotorSaturation;
      result = hil_write_analog(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILWriteAnalog_channels, 2,
        &Assignment_2_DW.HILWriteAnalog_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
      }
    }

    /* FromWorkspace: '<Root>/From Workspace' */
    {
      real_T *pDataValues = (real_T *)
        Assignment_2_DW.FromWorkspace_PWORK.DataPtr;
      real_T *pTimeValues = (real_T *)
        Assignment_2_DW.FromWorkspace_PWORK.TimePtr;
      int_T currTimeIndex = Assignment_2_DW.FromWorkspace_IWORK.PrevIndex;
      real_T t = Assignment_2_M->Timing.t[1];

      /* Get index */
      if (t <= pTimeValues[0]) {
        currTimeIndex = 0;
      } else if (t >= pTimeValues[140]) {
        currTimeIndex = 139;
      } else {
        if (t < pTimeValues[currTimeIndex]) {
          while (t < pTimeValues[currTimeIndex]) {
            currTimeIndex--;
          }
        } else {
          while (t >= pTimeValues[currTimeIndex + 1]) {
            currTimeIndex++;
          }
        }
      }

      Assignment_2_DW.FromWorkspace_IWORK.PrevIndex = currTimeIndex;

      /* Post output */
      {
        real_T t1 = pTimeValues[currTimeIndex];
        real_T t2 = pTimeValues[currTimeIndex + 1];
        if (t1 == t2) {
          if (t < t1) {
            Assignment_2_B.FromWorkspace = pDataValues[currTimeIndex];
          } else {
            Assignment_2_B.FromWorkspace = pDataValues[currTimeIndex + 1];
          }
        } else {
          real_T f1 = (t2 - t) / (t2 - t1);
          real_T f2 = 1.0 - f1;
          real_T d1;
          real_T d2;
          int_T TimeIndex= currTimeIndex;
          d1 = pDataValues[TimeIndex];
          d2 = pDataValues[TimeIndex + 1];
          Assignment_2_B.FromWorkspace = (real_T) rtInterpolate(d1, d2, f1, f2);
          pDataValues += 141;
        }
      }
    }
  }
}

/* Model update function */
void Assignment_2_update(void)
{
  real_T *lastU;

  /* Update for Derivative: '<S4>/Derivative' */
  if (Assignment_2_DW.TimeStampA == (rtInf)) {
    Assignment_2_DW.TimeStampA = Assignment_2_M->Timing.t[0];
    lastU = &Assignment_2_DW.LastUAtTimeA;
  } else if (Assignment_2_DW.TimeStampB == (rtInf)) {
    Assignment_2_DW.TimeStampB = Assignment_2_M->Timing.t[0];
    lastU = &Assignment_2_DW.LastUAtTimeB;
  } else if (Assignment_2_DW.TimeStampA < Assignment_2_DW.TimeStampB) {
    Assignment_2_DW.TimeStampA = Assignment_2_M->Timing.t[0];
    lastU = &Assignment_2_DW.LastUAtTimeA;
  } else {
    Assignment_2_DW.TimeStampB = Assignment_2_M->Timing.t[0];
    lastU = &Assignment_2_DW.LastUAtTimeB;
  }

  *lastU = Assignment_2_B.PitchCounttorad;

  /* End of Update for Derivative: '<S4>/Derivative' */
  if (rtmIsMajorTimeStep(Assignment_2_M)) {
    rt_ertODEUpdateContinuousStates(&Assignment_2_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Assignment_2_M->Timing.clockTick0)) {
    ++Assignment_2_M->Timing.clockTickH0;
  }

  Assignment_2_M->Timing.t[0] = rtsiGetSolverStopTime
    (&Assignment_2_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.002s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Assignment_2_M->Timing.clockTick1)) {
      ++Assignment_2_M->Timing.clockTickH1;
    }

    Assignment_2_M->Timing.t[1] = Assignment_2_M->Timing.clockTick1 *
      Assignment_2_M->Timing.stepSize1 + Assignment_2_M->Timing.clockTickH1 *
      Assignment_2_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void Assignment_2_derivatives(void)
{
  boolean_T lsat;
  boolean_T usat;
  XDot_Assignment_2_T *_rtXdot;
  _rtXdot = ((XDot_Assignment_2_T *) Assignment_2_M->ModelData.derivs);

  /* Derivatives for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  _rtXdot->TravelTransferFcn_CSTATE = 0.0;
  _rtXdot->TravelTransferFcn_CSTATE += Assignment_2_P.TravelTransferFcn_A *
    Assignment_2_X.TravelTransferFcn_CSTATE;
  _rtXdot->TravelTransferFcn_CSTATE += Assignment_2_B.TravelCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  _rtXdot->PitchTransferFcn_CSTATE = 0.0;
  _rtXdot->PitchTransferFcn_CSTATE += Assignment_2_P.PitchTransferFcn_A *
    Assignment_2_X.PitchTransferFcn_CSTATE;
  _rtXdot->PitchTransferFcn_CSTATE += Assignment_2_B.PitchCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  _rtXdot->ElevationTransferFcn_CSTATE = 0.0;
  _rtXdot->ElevationTransferFcn_CSTATE += Assignment_2_P.ElevationTransferFcn_A *
    Assignment_2_X.ElevationTransferFcn_CSTATE;
  _rtXdot->ElevationTransferFcn_CSTATE += Assignment_2_B.ElevationCounttorad;

  /* Derivatives for Integrator: '<S3>/Integrator' */
  lsat = (Assignment_2_X.Integrator_CSTATE <= Assignment_2_P.Integrator_LowerSat);
  usat = (Assignment_2_X.Integrator_CSTATE >= Assignment_2_P.Integrator_UpperSat);
  if (((!lsat) && (!usat)) || (lsat && (Assignment_2_B.K_ei > 0.0)) || (usat &&
       (Assignment_2_B.K_ei < 0.0))) {
    _rtXdot->Integrator_CSTATE = Assignment_2_B.K_ei;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S3>/Integrator' */
}

/* Model initialize function */
void Assignment_2_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: Assignment_2/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q8_usb", "0", &Assignment_2_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options(Assignment_2_DW.HILInitialize_Card,
      "update_rate=normal;decimation=1", 32);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(Assignment_2_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
      return;
    }

    if ((Assignment_2_P.HILInitialize_set_analog_input_ && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_analog_inpu_m && is_switching)) {
      {
        int_T i1;
        real_T *dw_AIMinimums = &Assignment_2_DW.HILInitialize_AIMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMinimums[i1] = Assignment_2_P.HILInitialize_analog_input_mini;
        }
      }

      {
        int_T i1;
        real_T *dw_AIMaximums = &Assignment_2_DW.HILInitialize_AIMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMaximums[i1] = Assignment_2_P.HILInitialize_analog_input_maxi;
        }
      }

      result = hil_set_analog_input_ranges(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_analog_input_chan, 8U,
        &Assignment_2_DW.HILInitialize_AIMinimums[0],
        &Assignment_2_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if ((Assignment_2_P.HILInitialize_set_analog_output && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_analog_outp_b && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOMinimums = &Assignment_2_DW.HILInitialize_AOMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMinimums[i1] = Assignment_2_P.HILInitialize_analog_output_min;
        }
      }

      {
        int_T i1;
        real_T *dw_AOMaximums = &Assignment_2_DW.HILInitialize_AOMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMaximums[i1] = Assignment_2_P.HILInitialize_analog_output_max;
        }
      }

      result = hil_set_analog_output_ranges(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_analog_output_cha, 8U,
        &Assignment_2_DW.HILInitialize_AOMinimums[0],
        &Assignment_2_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if ((Assignment_2_P.HILInitialize_set_analog_outp_e && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_analog_outp_j && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages = &Assignment_2_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = Assignment_2_P.HILInitialize_initial_analog_ou;
        }
      }

      result = hil_write_analog(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_analog_output_cha, 8U,
        &Assignment_2_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if (Assignment_2_P.HILInitialize_set_analog_outp_p) {
      {
        int_T i1;
        real_T *dw_AOVoltages = &Assignment_2_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = Assignment_2_P.HILInitialize_watchdog_analog_o;
        }
      }

      result = hil_watchdog_set_analog_expiration_state
        (Assignment_2_DW.HILInitialize_Card,
         Assignment_2_P.HILInitialize_analog_output_cha, 8U,
         &Assignment_2_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if ((Assignment_2_P.HILInitialize_set_encoder_param && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_encoder_par_m && is_switching)) {
      {
        int_T i1;
        int32_T *dw_QuadratureModes =
          &Assignment_2_DW.HILInitialize_QuadratureModes[0];
        for (i1=0; i1 < 8; i1++) {
          dw_QuadratureModes[i1] = Assignment_2_P.HILInitialize_quadrature;
        }
      }

      result = hil_set_encoder_quadrature_mode
        (Assignment_2_DW.HILInitialize_Card,
         Assignment_2_P.HILInitialize_encoder_channels, 8U,
         (t_encoder_quadrature_mode *)
         &Assignment_2_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if ((Assignment_2_P.HILInitialize_set_encoder_count && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_encoder_cou_k && is_switching)) {
      {
        int_T i1;
        int32_T *dw_InitialEICounts =
          &Assignment_2_DW.HILInitialize_InitialEICounts[0];
        for (i1=0; i1 < 8; i1++) {
          dw_InitialEICounts[i1] =
            Assignment_2_P.HILInitialize_initial_encoder_c;
        }
      }

      result = hil_set_encoder_counts(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_encoder_channels, 8U,
        &Assignment_2_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if ((Assignment_2_P.HILInitialize_set_pwm_params_at && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_pwm_params__f && is_switching)) {
      uint32_T num_duty_cycle_modes = 0;
      uint32_T num_frequency_modes = 0;

      {
        int_T i1;
        int32_T *dw_POModeValues = &Assignment_2_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = Assignment_2_P.HILInitialize_pwm_modes;
        }
      }

      result = hil_set_pwm_mode(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_pwm_channels, 8U, (t_pwm_mode *)
        &Assignment_2_DW.HILInitialize_POModeValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        const uint32_T *p_HILInitialize_pwm_channels =
          Assignment_2_P.HILInitialize_pwm_channels;
        int32_T *dw_POModeValues = &Assignment_2_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          if (dw_POModeValues[i1] == PWM_DUTY_CYCLE_MODE || dw_POModeValues[i1] ==
              PWM_ONE_SHOT_MODE || dw_POModeValues[i1] == PWM_TIME_MODE) {
            Assignment_2_DW.HILInitialize_POSortedChans[num_duty_cycle_modes] =
              p_HILInitialize_pwm_channels[i1];
            Assignment_2_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes] =
              Assignment_2_P.HILInitialize_pwm_frequency;
            num_duty_cycle_modes++;
          } else {
            Assignment_2_DW.HILInitialize_POSortedChans[7U - num_frequency_modes]
              = p_HILInitialize_pwm_channels[i1];
            Assignment_2_DW.HILInitialize_POSortedFreqs[7U - num_frequency_modes]
              = Assignment_2_P.HILInitialize_pwm_frequency;
            num_frequency_modes++;
          }
        }
      }

      if (num_duty_cycle_modes > 0) {
        result = hil_set_pwm_frequency(Assignment_2_DW.HILInitialize_Card,
          &Assignment_2_DW.HILInitialize_POSortedChans[0], num_duty_cycle_modes,
          &Assignment_2_DW.HILInitialize_POSortedFreqs[0]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
          return;
        }
      }

      if (num_frequency_modes > 0) {
        result = hil_set_pwm_duty_cycle(Assignment_2_DW.HILInitialize_Card,
          &Assignment_2_DW.HILInitialize_POSortedChans[num_duty_cycle_modes],
          num_frequency_modes,
          &Assignment_2_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
          return;
        }
      }

      {
        int_T i1;
        int32_T *dw_POModeValues = &Assignment_2_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = Assignment_2_P.HILInitialize_pwm_configuration;
        }
      }

      {
        int_T i1;
        int32_T *dw_POAlignValues =
          &Assignment_2_DW.HILInitialize_POAlignValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POAlignValues[i1] = Assignment_2_P.HILInitialize_pwm_alignment;
        }
      }

      {
        int_T i1;
        int32_T *dw_POPolarityVals =
          &Assignment_2_DW.HILInitialize_POPolarityVals[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POPolarityVals[i1] = Assignment_2_P.HILInitialize_pwm_polarity;
        }
      }

      result = hil_set_pwm_configuration(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_pwm_channels, 8U,
        (t_pwm_configuration *) &Assignment_2_DW.HILInitialize_POModeValues[0],
        (t_pwm_alignment *) &Assignment_2_DW.HILInitialize_POAlignValues[0],
        (t_pwm_polarity *) &Assignment_2_DW.HILInitialize_POPolarityVals[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        real_T *dw_POSortedFreqs = &Assignment_2_DW.HILInitialize_POSortedFreqs
          [0];
        for (i1=0; i1 < 8; i1++) {
          dw_POSortedFreqs[i1] = Assignment_2_P.HILInitialize_pwm_leading_deadb;
        }
      }

      {
        int_T i1;
        real_T *dw_POValues = &Assignment_2_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = Assignment_2_P.HILInitialize_pwm_trailing_dead;
        }
      }

      result = hil_set_pwm_deadband(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_pwm_channels, 8U,
        &Assignment_2_DW.HILInitialize_POSortedFreqs[0],
        &Assignment_2_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if ((Assignment_2_P.HILInitialize_set_pwm_outputs_a && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_pwm_outputs_g && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &Assignment_2_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = Assignment_2_P.HILInitialize_initial_pwm_outpu;
        }
      }

      result = hil_write_pwm(Assignment_2_DW.HILInitialize_Card,
        Assignment_2_P.HILInitialize_pwm_channels, 8U,
        &Assignment_2_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }

    if (Assignment_2_P.HILInitialize_set_pwm_outputs_o) {
      {
        int_T i1;
        real_T *dw_POValues = &Assignment_2_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = Assignment_2_P.HILInitialize_watchdog_pwm_outp;
        }
      }

      result = hil_watchdog_set_pwm_expiration_state
        (Assignment_2_DW.HILInitialize_Card,
         Assignment_2_P.HILInitialize_pwm_channels, 8U,
         &Assignment_2_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

  /* S-Function Block: Assignment_2/Helicopter_interface/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
  {
    t_error result;
    result = hil_task_create_encoder_reader(Assignment_2_DW.HILInitialize_Card,
      Assignment_2_P.HILReadEncoderTimebase_samples_,
      Assignment_2_P.HILReadEncoderTimebase_channels, 3,
      &Assignment_2_DW.HILReadEncoderTimebase_Task);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
    }
  }

  /* Start for FromWorkspace: '<Root>/From Workspace' */
  {
    static real_T pTimeValues0[] = { 0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75,
      2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0,
      5.25, 5.5, 5.75, 6.0, 6.25, 6.5, 6.75, 7.0, 7.25, 7.5, 7.75, 8.0, 8.25,
      8.5, 8.75, 9.0, 9.25, 9.5, 9.75, 10.0, 10.25, 10.5, 10.75, 11.0, 11.25,
      11.5, 11.75, 12.0, 12.25, 12.5, 12.75, 13.0, 13.25, 13.5, 13.75, 14.0,
      14.25, 14.5, 14.75, 15.0, 15.25, 15.5, 15.75, 16.0, 16.25, 16.5, 16.75,
      17.0, 17.25, 17.5, 17.75, 18.0, 18.25, 18.5, 18.75, 19.0, 19.25, 19.5,
      19.75, 20.0, 20.25, 20.5, 20.75, 21.0, 21.25, 21.5, 21.75, 22.0, 22.25,
      22.5, 22.75, 23.0, 23.25, 23.5, 23.75, 24.0, 24.25, 24.5, 24.75, 25.0,
      25.25, 25.5, 25.75, 26.0, 26.25, 26.5, 26.75, 27.0, 27.25, 27.5, 27.75,
      28.0, 28.25, 28.5, 28.75, 29.0, 29.25, 29.5, 29.75, 30.0, 30.25, 30.5,
      30.75, 31.0, 31.25, 31.5, 31.75, 32.0, 32.25, 32.5, 32.75, 33.0, 33.25,
      33.5, 33.75, 34.0, 34.25, 34.5, 34.75, 35.0 } ;

    static real_T pDataValues0[] = { -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.29017777602000533, -0.24761408004273791, -0.20824123731695326,
      -0.1719349683266089, -0.1385710762023282, -0.10802563414460499,
      -0.080175075700639878, -0.054896405655919821, -0.032067347877791488,
      -0.011566570073140627, 0.0067261351087891828, 0.022929621831802104,
      0.037161195282089644, 0.049536389339674129, 0.060168770321247553,
      0.069169728480122747, 0.07664829200967764, 0.082710938654570854,
      0.087461427826983434, 0.0910006370023566, 0.093426417415574461,
      0.09483345840635779, 0.09531317018346433, 0.094953577922920959,
      0.093839233454486748, 0.092051139723858547, 0.089666691920177052,
      0.086759631900865081, 0.083400018204102, 0.0796542092020102,
      0.075584860596169423, 0.07125093540486005, 0.066707726895375338,
      0.0620068930131138, 0.057196502237021683, 0.052321089709038753,
      0.047421723185767109, 0.04253607791121234, 0.037698519657452231,
      0.032940195286317746, 0.028289129804244327, 0.023770329564179349,
      0.019405890285622573, 0.015215109942029152, 0.011214604797614896,
      0.0074184291945295217, 0.0038381968150120165, 0.00048320481519915388,
      -0.0026394422836915313, -0.0055247063807253157, -0.0081694916572479809,
      -0.010572518210469617, -0.012734199838493223, -0.014656519065394219,
      -0.016342908832590188, -0.017798130017457513, -0.019028158883382724,
      -0.020040067680977994, -0.020841919672875527, -0.021442652791259474,
      -0.02185198416597153, -0.022080296042568129, -0.02213855208120635,
      -0.022038183732343244, -0.021791021253285796, -0.021409177363014047,
      -0.020904997113626016, -0.020290933698798447, -0.019579523275486958,
      -0.0187832449705678, -0.017914529863238292, -0.016985593044358465,
      -0.016008490833443328, -0.014994906466556095, -0.013956277133748788,
      -0.012903505392571572, -0.011847189143516599, -0.010797216492697118,
      -0.0097631494238989546, -0.0087536379827866471, -0.0077770353395432658,
      -0.00684053390150075, -0.0059511305448897217, -0.0051143365837166875,
      -0.004335674583150392, -0.0036187368767588686, -0.0029674908378131745,
      -0.0023833429104785828, -0.0018686747030887108, -0.0014223900451863689,
      -0.0010453259869708286, -0.00073348684739294292, -0.00048631210398857337,
      -0.0002963844289631132, -0.0001620514217746892, -7.1759058009542863E-5,
      -2.3309058912092679E-5, -4.1470177372478153E-10, -3.534108212271438E-10,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0 } ;

    Assignment_2_DW.FromWorkspace_PWORK.TimePtr = (void *) pTimeValues0;
    Assignment_2_DW.FromWorkspace_PWORK.DataPtr = (void *) pDataValues0;
    Assignment_2_DW.FromWorkspace_IWORK.PrevIndex = 0;
  }

  /* InitializeConditions for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  Assignment_2_X.TravelTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  Assignment_2_X.PitchTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  Assignment_2_X.ElevationTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S3>/Integrator' */
  Assignment_2_X.Integrator_CSTATE = Assignment_2_P.Integrator_IC;

  /* InitializeConditions for Derivative: '<S4>/Derivative' */
  Assignment_2_DW.TimeStampA = (rtInf);
  Assignment_2_DW.TimeStampB = (rtInf);
}

/* Model terminate function */
void Assignment_2_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: Assignment_2/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_pwm_outputs = 0;
    hil_task_stop_all(Assignment_2_DW.HILInitialize_Card);
    hil_monitor_stop_all(Assignment_2_DW.HILInitialize_Card);
    is_switching = false;
    if ((Assignment_2_P.HILInitialize_set_analog_out_ex && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_analog_outp_c && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages = &Assignment_2_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = Assignment_2_P.HILInitialize_final_analog_outp;
        }
      }

      num_final_analog_outputs = 8U;
    }

    if ((Assignment_2_P.HILInitialize_set_pwm_output_ap && !is_switching) ||
        (Assignment_2_P.HILInitialize_set_pwm_outputs_p && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &Assignment_2_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = Assignment_2_P.HILInitialize_final_pwm_outputs;
        }
      }

      num_final_pwm_outputs = 8U;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_pwm_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(Assignment_2_DW.HILInitialize_Card
                         , Assignment_2_P.HILInitialize_analog_output_cha,
                         num_final_analog_outputs
                         , Assignment_2_P.HILInitialize_pwm_channels,
                         num_final_pwm_outputs
                         , NULL, 0
                         , NULL, 0
                         , &Assignment_2_DW.HILInitialize_AOVoltages[0]
                         , &Assignment_2_DW.HILInitialize_POValues[0]
                         , (t_boolean *) NULL
                         , NULL
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog(Assignment_2_DW.HILInitialize_Card,
            Assignment_2_P.HILInitialize_analog_output_cha,
            num_final_analog_outputs, &Assignment_2_DW.HILInitialize_AOVoltages
            [0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_pwm_outputs > 0) {
          local_result = hil_write_pwm(Assignment_2_DW.HILInitialize_Card,
            Assignment_2_P.HILInitialize_pwm_channels, num_final_pwm_outputs,
            &Assignment_2_DW.HILInitialize_POValues[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(Assignment_2_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(Assignment_2_DW.HILInitialize_Card);
    hil_monitor_delete_all(Assignment_2_DW.HILInitialize_Card);
    hil_close(Assignment_2_DW.HILInitialize_Card);
    Assignment_2_DW.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  Assignment_2_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  Assignment_2_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  Assignment_2_initialize();
}

void MdlTerminate(void)
{
  Assignment_2_terminate();
}

/* Registration function */
RT_MODEL_Assignment_2_T *Assignment_2(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  Assignment_2_P.Integrator_UpperSat = rtInf;
  Assignment_2_P.Integrator_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)Assignment_2_M, 0,
                sizeof(RT_MODEL_Assignment_2_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Assignment_2_M->solverInfo,
                          &Assignment_2_M->Timing.simTimeStep);
    rtsiSetTPtr(&Assignment_2_M->solverInfo, &rtmGetTPtr(Assignment_2_M));
    rtsiSetStepSizePtr(&Assignment_2_M->solverInfo,
                       &Assignment_2_M->Timing.stepSize0);
    rtsiSetdXPtr(&Assignment_2_M->solverInfo, &Assignment_2_M->ModelData.derivs);
    rtsiSetContStatesPtr(&Assignment_2_M->solverInfo, (real_T **)
                         &Assignment_2_M->ModelData.contStates);
    rtsiSetNumContStatesPtr(&Assignment_2_M->solverInfo,
      &Assignment_2_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Assignment_2_M->solverInfo,
      &Assignment_2_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Assignment_2_M->solverInfo,
      &Assignment_2_M->ModelData.periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Assignment_2_M->solverInfo,
      &Assignment_2_M->ModelData.periodicContStateRanges);
    rtsiSetErrorStatusPtr(&Assignment_2_M->solverInfo, (&rtmGetErrorStatus
      (Assignment_2_M)));
    rtsiSetRTModelPtr(&Assignment_2_M->solverInfo, Assignment_2_M);
  }

  rtsiSetSimTimeStep(&Assignment_2_M->solverInfo, MAJOR_TIME_STEP);
  Assignment_2_M->ModelData.intgData.f[0] = Assignment_2_M->ModelData.odeF[0];
  Assignment_2_M->ModelData.contStates = ((real_T *) &Assignment_2_X);
  rtsiSetSolverData(&Assignment_2_M->solverInfo, (void *)
                    &Assignment_2_M->ModelData.intgData);
  rtsiSetSolverName(&Assignment_2_M->solverInfo,"ode1");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = Assignment_2_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    Assignment_2_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    Assignment_2_M->Timing.sampleTimes =
      (&Assignment_2_M->Timing.sampleTimesArray[0]);
    Assignment_2_M->Timing.offsetTimes =
      (&Assignment_2_M->Timing.offsetTimesArray[0]);

    /* task periods */
    Assignment_2_M->Timing.sampleTimes[0] = (0.0);
    Assignment_2_M->Timing.sampleTimes[1] = (0.002);

    /* task offsets */
    Assignment_2_M->Timing.offsetTimes[0] = (0.0);
    Assignment_2_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(Assignment_2_M, &Assignment_2_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = Assignment_2_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    Assignment_2_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(Assignment_2_M, -1);
  Assignment_2_M->Timing.stepSize0 = 0.002;
  Assignment_2_M->Timing.stepSize1 = 0.002;

  /* External mode info */
  Assignment_2_M->Sizes.checksums[0] = (3663187348U);
  Assignment_2_M->Sizes.checksums[1] = (2050958508U);
  Assignment_2_M->Sizes.checksums[2] = (3974979138U);
  Assignment_2_M->Sizes.checksums[3] = (1683037652U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    Assignment_2_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Assignment_2_M->extModeInfo,
      &Assignment_2_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Assignment_2_M->extModeInfo,
                        Assignment_2_M->Sizes.checksums);
    rteiSetTPtr(Assignment_2_M->extModeInfo, rtmGetTPtr(Assignment_2_M));
  }

  Assignment_2_M->solverInfoPtr = (&Assignment_2_M->solverInfo);
  Assignment_2_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&Assignment_2_M->solverInfo, 0.002);
  rtsiSetSolverMode(&Assignment_2_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  Assignment_2_M->ModelData.blockIO = ((void *) &Assignment_2_B);

  {
    Assignment_2_B.TravelCounttorad = 0.0;
    Assignment_2_B.Gain = 0.0;
    Assignment_2_B.Sum3 = 0.0;
    Assignment_2_B.Gain_d = 0.0;
    Assignment_2_B.PitchCounttorad = 0.0;
    Assignment_2_B.Gain_i = 0.0;
    Assignment_2_B.Sum4 = 0.0;
    Assignment_2_B.Gain_b = 0.0;
    Assignment_2_B.ElevationCounttorad = 0.0;
    Assignment_2_B.Gain_e = 0.0;
    Assignment_2_B.Sum = 0.0;
    Assignment_2_B.Gain_dg = 0.0;
    Assignment_2_B.Sum1 = 0.0;
    Assignment_2_B.Sum2 = 0.0;
    Assignment_2_B.K_ei = 0.0;
    Assignment_2_B.Gain_l = 0.0;
    Assignment_2_B.BackmotorSaturation = 0.0;
    Assignment_2_B.FrontmotorSaturation = 0.0;
    Assignment_2_B.FromWorkspace = 0.0;
  }

  /* parameters */
  Assignment_2_M->ModelData.defaultParam = ((real_T *)&Assignment_2_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &Assignment_2_X;
    Assignment_2_M->ModelData.contStates = (x);
    (void) memset((void *)&Assignment_2_X, 0,
                  sizeof(X_Assignment_2_T));
  }

  /* states (dwork) */
  Assignment_2_M->ModelData.dwork = ((void *) &Assignment_2_DW);
  (void) memset((void *)&Assignment_2_DW, 0,
                sizeof(DW_Assignment_2_T));

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_AIMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_AIMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_AOMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_AOMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_AOVoltages[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_FilterFrequency[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_POSortedFreqs[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Assignment_2_DW.HILInitialize_POValues[i] = 0.0;
    }
  }

  Assignment_2_DW.TimeStampA = 0.0;
  Assignment_2_DW.LastUAtTimeA = 0.0;
  Assignment_2_DW.TimeStampB = 0.0;
  Assignment_2_DW.LastUAtTimeB = 0.0;
  Assignment_2_DW.HILWriteAnalog_Buffer[0] = 0.0;
  Assignment_2_DW.HILWriteAnalog_Buffer[1] = 0.0;

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    Assignment_2_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 16;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* Initialize Sizes */
  Assignment_2_M->Sizes.numContStates = (4);/* Number of continuous states */
  Assignment_2_M->Sizes.numPeriodicContStates = (0);/* Number of periodic continuous states */
  Assignment_2_M->Sizes.numY = (0);    /* Number of model outputs */
  Assignment_2_M->Sizes.numU = (0);    /* Number of model inputs */
  Assignment_2_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  Assignment_2_M->Sizes.numSampTimes = (2);/* Number of sample times */
  Assignment_2_M->Sizes.numBlocks = (61);/* Number of blocks */
  Assignment_2_M->Sizes.numBlockIO = (19);/* Number of block outputs */
  Assignment_2_M->Sizes.numBlockPrms = (146);/* Sum of parameter "widths" */
  return Assignment_2_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
