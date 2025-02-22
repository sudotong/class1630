/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DroneRS_Compensator.c
 *
 * Code generated for Simulink model 'DroneRS_Compensator'.
 *
 * Model version                  : 1.2633
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Thu Nov 12 12:08:21 2015
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 *    3. ROM efficiency
 * Validation result: Not run
 */

#include "DroneRS_Compensator.h"
#include "DroneRS_Compensator_private.h"

/* Forward declaration for local functions */
static real_T DroneRS_Compensator_genpnorm(const real_T x[3]);

/* Output and update for atomic system: '<S1>/ControllerLQR1' */
void DroneRS_Compensa_ControllerLQR1(const real_T rtu_pos_refin[3], const real_T
  rtu_att_refin[3], boolean_T rtu_controlModePosVSAtt_flagin, const real_T
  rtu_states_estimin[2], const real_T rtu_states_estimin_c[2], real_T
  rtu_states_estimin_i, const real_T rtu_states_estimin_o[3], real_T
  rtu_states_estimin_j, const real_T rtu_states_estimin_a[3],
  B_ControllerLQR1_DroneRS_Comp_T *localB, P_ControllerLQR1_DroneRS_Comp_T
  *localP, P_DroneRS_Compensator_T *DroneRS_Compensator_P)
{
  /* local block i/o variables */
  real_T rtb_SaturationThrust;
  real_T y;
  real_T y_0;
  real_T rtb_TmpSignalConversionAtProduc[4];
  real_T rtu_states_estimin_0[12];
  real_T tmp[12];
  int32_T i;
  real_T tmp_0[4];
  int32_T i_0;
  real_T y_idx_0;
  real_T y_idx_1;
  real_T y_idx_2;
  real_T y_idx_3;

  /* Switch: '<S6>/PosVSAtt_Switch' incorporates:
   *  Constant: '<S6>/dz_ref'
   *  Constant: '<S6>/velocitiesPos_ref'
   *  Constant: '<S6>/velocitiesRot_ref'
   */
  if (rtu_controlModePosVSAtt_flagin) {
    localB->PosVSAtt_Switch[0] = rtu_pos_refin[0];
    localB->PosVSAtt_Switch[1] = rtu_pos_refin[1];
  } else {
    localB->PosVSAtt_Switch[0] = rtu_states_estimin[0];
    localB->PosVSAtt_Switch[1] = rtu_states_estimin[1];
  }

  localB->PosVSAtt_Switch[2] = rtu_pos_refin[2];
  localB->PosVSAtt_Switch[3] = rtu_att_refin[0];
  localB->PosVSAtt_Switch[4] = rtu_att_refin[1];
  localB->PosVSAtt_Switch[5] = rtu_att_refin[2];
  if (rtu_controlModePosVSAtt_flagin) {
    localB->PosVSAtt_Switch[6] = localP->velocitiesPos_ref_Value[0];
    localB->PosVSAtt_Switch[7] = localP->velocitiesPos_ref_Value[1];
    localB->PosVSAtt_Switch[8] = localP->velocitiesPos_ref_Value[2];
  } else {
    localB->PosVSAtt_Switch[6] = rtu_states_estimin_c[0];
    localB->PosVSAtt_Switch[7] = rtu_states_estimin_c[1];
    localB->PosVSAtt_Switch[8] = localP->dz_ref_Value;
  }

  localB->PosVSAtt_Switch[9] = localP->velocitiesRot_ref_Value[0];
  localB->PosVSAtt_Switch[10] = localP->velocitiesRot_ref_Value[1];
  localB->PosVSAtt_Switch[11] = localP->velocitiesRot_ref_Value[2];

  /* End of Switch: '<S6>/PosVSAtt_Switch' */

  /* Gain: '<S8>/W2ToThrust' */
  y = DroneRS_Compensator_P->quad.Ct * DroneRS_Compensator_P->quad.rho *
    DroneRS_Compensator_P->quad.A * 0.0010890000000000001;

  /* Sum: '<S2>/Add' incorporates:
   *  Gain: '<S4>/Gain'
   */
  rtu_states_estimin_0[0] = rtu_states_estimin[0];
  rtu_states_estimin_0[1] = rtu_states_estimin[1];
  rtu_states_estimin_0[2] = rtu_states_estimin_i;
  rtu_states_estimin_0[3] = rtu_states_estimin_o[0];
  rtu_states_estimin_0[4] = rtu_states_estimin_o[1];
  rtu_states_estimin_0[5] = rtu_states_estimin_o[2];
  rtu_states_estimin_0[6] = rtu_states_estimin_c[0];
  rtu_states_estimin_0[7] = rtu_states_estimin_c[1];
  rtu_states_estimin_0[8] = rtu_states_estimin_j;
  rtu_states_estimin_0[9] = rtu_states_estimin_a[0];
  rtu_states_estimin_0[10] = rtu_states_estimin_a[1];
  rtu_states_estimin_0[11] = rtu_states_estimin_a[2];
  for (i = 0; i < 12; i++) {
    tmp[i] = localB->PosVSAtt_Switch[i] - rtu_states_estimin_0[i];
  }

  /* End of Sum: '<S2>/Add' */

  /* Gain: '<S4>/Gain' incorporates:
   *  Gain: '<S8>/MotorsRotationDirection'
   */
  for (i = 0; i < 4; i++) {
    tmp_0[i] = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      tmp_0[i] += DroneRS_Compensator_P->K_lqr_toMotorcmd[(i_0 << 2) + i] *
        tmp[i_0];
    }
  }

  /* Gain: '<S8>/MotorsRotationDirection' incorporates:
   *  Gain: '<S8>/motorsRSToW2_Gain'
   */
  for (i = 0; i < 4; i++) {
    y_idx_0 = localP->MotorsRotationDirection_Gain[i + 12] * tmp_0[3] +
      (localP->MotorsRotationDirection_Gain[i + 8] * tmp_0[2] +
       (localP->MotorsRotationDirection_Gain[i + 4] * tmp_0[1] +
        localP->MotorsRotationDirection_Gain[i] * tmp_0[0]));
    rtb_TmpSignalConversionAtProduc[i] = y_idx_0;
  }

  /* Gain: '<S8>/W2ToThrust' incorporates:
   *  Gain: '<S8>/ThrustperMotor To TotalThrust and Torques'
   *  Gain: '<S8>/motorsRSToW2_Gain'
   */
  y_idx_0 = DroneRS_Compensator_P->quadEDT.motorsRSToW2_Gain *
    rtb_TmpSignalConversionAtProduc[0] * y;
  y_idx_1 = DroneRS_Compensator_P->quadEDT.motorsRSToW2_Gain *
    rtb_TmpSignalConversionAtProduc[1] * y;
  y_idx_2 = DroneRS_Compensator_P->quadEDT.motorsRSToW2_Gain *
    rtb_TmpSignalConversionAtProduc[2] * y;
  y_idx_3 = DroneRS_Compensator_P->quadEDT.motorsRSToW2_Gain *
    rtb_TmpSignalConversionAtProduc[3] * y;

  /* Gain: '<S8>/ThrustperMotor To TotalThrust and Torques' */
  for (i = 0; i < 4; i++) {
    y_0 = localP->ThrustperMotorToTotalThrustandT[i + 12] * y_idx_3 +
      (localP->ThrustperMotorToTotalThrustandT[i + 8] * y_idx_2 +
       (localP->ThrustperMotorToTotalThrustandT[i + 4] * y_idx_1 +
        localP->ThrustperMotorToTotalThrustandT[i] * y_idx_0));
    rtb_TmpSignalConversionAtProduc[i] = y_0;
  }

  /* Switch: '<S9>/TakeoffOrControl_Switch' incorporates:
   *  Constant: '<S11>/Constant'
   *  Constant: '<S9>/HoverThrustLinearizationPoint'
   *  Gain: '<S9>/takeoff_Gain'
   *  RelationalOperator: '<S11>/Compare'
   */
  if (rtu_pos_refin[2] > localP->Constant_Value) {
    rtb_SaturationThrust = -DroneRS_Compensator_P->quad.g *
      DroneRS_Compensator_P->quad.M * localP->takeoff_Gain_Gain;
  } else {
    rtb_SaturationThrust = rtb_TmpSignalConversionAtProduc[0];
  }

  /* End of Switch: '<S9>/TakeoffOrControl_Switch' */

  /* Sum: '<S9>/Add1' incorporates:
   *  Constant: '<S9>/HoverThrustLinearizationPoint'
   */
  rtb_SaturationThrust += -DroneRS_Compensator_P->quad.g *
    DroneRS_Compensator_P->quad.M;

  /* Saturate: '<S9>/SaturationThrust' */
  if (rtb_SaturationThrust > localP->SaturationThrust_UpperSat) {
    rtb_SaturationThrust = localP->SaturationThrust_UpperSat;
  } else {
    if (rtb_SaturationThrust < localP->SaturationThrust_LowerSat) {
      rtb_SaturationThrust = localP->SaturationThrust_LowerSat;
    }
  }

  /* End of Saturate: '<S9>/SaturationThrust' */

  /* SignalConversion: '<S7>/TmpSignal ConversionAtProductInport2' */
  y_idx_3 = rtb_SaturationThrust;

  /* Gain: '<S10>/ThrustToW2_Gain' */
  y = 1.0 / (DroneRS_Compensator_P->quad.Ct * DroneRS_Compensator_P->quad.rho *
             DroneRS_Compensator_P->quad.A * 0.0010890000000000001);

  /* Gain: '<S10>/W2ToMotorsCmd_Gain' */
  y_0 = 1.0 / DroneRS_Compensator_P->quadEDT.motorsRSToW2_Gain;

  /* Product: '<S7>/Product' incorporates:
   *  Constant: '<S7>/TorquetotalThrustToThrustperMotor'
   *  Saturate: '<S7>/Saturation2'
   *  SignalConversion: '<S7>/TmpSignal ConversionAtProductInport2'
   */
  for (i = 0; i < 4; i++) {
    y_idx_0 = localP->TorquetotalThrustToThrustperMot[i + 12] *
      rtb_TmpSignalConversionAtProduc[3] +
      (localP->TorquetotalThrustToThrustperMot[i + 8] *
       rtb_TmpSignalConversionAtProduc[2] +
       (localP->TorquetotalThrustToThrustperMot[i + 4] *
        rtb_TmpSignalConversionAtProduc[1] +
        localP->TorquetotalThrustToThrustperMot[i] * y_idx_3));
    tmp_0[i] = y_idx_0;
  }

  /* End of Product: '<S7>/Product' */

  /* Gain: '<S10>/W2ToMotorsCmd_Gain' incorporates:
   *  Gain: '<S10>/MotorsRotationDirection'
   *  Gain: '<S10>/ThrustToW2_Gain'
   *  Saturate: '<S7>/Saturation2'
   */
  if (tmp_0[0] > localP->Saturation2_UpperSat) {
    y_idx_0 = localP->Saturation2_UpperSat;
  } else if (tmp_0[0] < localP->Saturation2_LowerSat) {
    y_idx_0 = localP->Saturation2_LowerSat;
  } else {
    y_idx_0 = tmp_0[0];
  }

  localB->W2ToMotorsCmd_Gain[0] = y * y_idx_0 *
    localP->MotorsRotationDirection_Gain_f[0] * y_0;
  if (tmp_0[1] > localP->Saturation2_UpperSat) {
    y_idx_0 = localP->Saturation2_UpperSat;
  } else if (tmp_0[1] < localP->Saturation2_LowerSat) {
    y_idx_0 = localP->Saturation2_LowerSat;
  } else {
    y_idx_0 = tmp_0[1];
  }

  localB->W2ToMotorsCmd_Gain[1] = y * y_idx_0 *
    localP->MotorsRotationDirection_Gain_f[1] * y_0;
  if (tmp_0[2] > localP->Saturation2_UpperSat) {
    y_idx_0 = localP->Saturation2_UpperSat;
  } else if (tmp_0[2] < localP->Saturation2_LowerSat) {
    y_idx_0 = localP->Saturation2_LowerSat;
  } else {
    y_idx_0 = tmp_0[2];
  }

  localB->W2ToMotorsCmd_Gain[2] = y * y_idx_0 *
    localP->MotorsRotationDirection_Gain_f[2] * y_0;
  if (tmp_0[3] > localP->Saturation2_UpperSat) {
    y_idx_0 = localP->Saturation2_UpperSat;
  } else if (tmp_0[3] < localP->Saturation2_LowerSat) {
    y_idx_0 = localP->Saturation2_LowerSat;
  } else {
    y_idx_0 = tmp_0[3];
  }

  localB->W2ToMotorsCmd_Gain[3] = y * y_idx_0 *
    localP->MotorsRotationDirection_Gain_f[3] * y_0;
}

/*
 * Output and update for enable system:
 *    '<S94>/MeasurementUpdate'
 *    '<S154>/MeasurementUpdate'
 */
void DroneRS_Compe_MeasurementUpdate(boolean_T rtu_Enable, const real_T rtu_Lk[4],
  const real_T rtu_yk[2], const real_T rtu_yhatkk1[2],
  B_MeasurementUpdate_DroneRS_C_T *localB)
{
  real_T rtu_yk_idx_0;
  real_T rtu_yk_idx_1;

  /* Outputs for Enabled SubSystem: '<S94>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S119>/Enable'
   */
  if (rtu_Enable) {
    /* Sum: '<S119>/Sum' incorporates:
     *  Product: '<S119>/Product3'
     */
    rtu_yk_idx_0 = rtu_yk[0] - rtu_yhatkk1[0];
    rtu_yk_idx_1 = rtu_yk[1] - rtu_yhatkk1[1];

    /* Product: '<S119>/Product3' */
    localB->Product3[0] = 0.0;
    localB->Product3[0] += rtu_Lk[0] * rtu_yk_idx_0;
    localB->Product3[0] += rtu_Lk[2] * rtu_yk_idx_1;
    localB->Product3[1] = 0.0;
    localB->Product3[1] += rtu_Lk[1] * rtu_yk_idx_0;
    localB->Product3[1] += rtu_Lk[3] * rtu_yk_idx_1;
  }

  /* End of Outputs for SubSystem: '<S94>/MeasurementUpdate' */
}

/*
 * Output and update for atomic system:
 *    '<S73>/UseCurrentEstimator'
 *    '<S133>/UseCurrentEstimator'
 */
void DroneRS_Com_UseCurrentEstimator(boolean_T rtu_Enablek, const real_T rtu_Mk
  [4], const real_T rtu_uk[2], const real_T rtu_yk[2], const real_T rtu_Ck[4],
  const real_T rtu_Dk[4], const real_T rtu_xhatkk1[2],
  B_UseCurrentEstimator_DroneRS_T *localB)
{
  real_T rtu_yk_idx_0;
  real_T rtu_yk_idx_1;

  /* Outputs for Enabled SubSystem: '<S99>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S120>/Enable'
   */
  if (rtu_Enablek) {
    /* Sum: '<S120>/Add1' incorporates:
     *  Product: '<S120>/Product'
     *  Product: '<S120>/Product1'
     */
    rtu_yk_idx_0 = (rtu_yk[0] - (rtu_Ck[0] * rtu_xhatkk1[0] + rtu_Ck[2] *
      rtu_xhatkk1[1])) - (rtu_Dk[0] * rtu_uk[0] + rtu_Dk[2] * rtu_uk[1]);
    rtu_yk_idx_1 = (rtu_yk[1] - (rtu_Ck[1] * rtu_xhatkk1[0] + rtu_Ck[3] *
      rtu_xhatkk1[1])) - (rtu_Dk[1] * rtu_uk[0] + rtu_Dk[3] * rtu_uk[1]);

    /* Product: '<S120>/Product2' */
    localB->Product2[0] = 0.0;
    localB->Product2[0] += rtu_Mk[0] * rtu_yk_idx_0;
    localB->Product2[0] += rtu_Mk[2] * rtu_yk_idx_1;
    localB->Product2[1] = 0.0;
    localB->Product2[1] += rtu_Mk[1] * rtu_yk_idx_0;
    localB->Product2[1] += rtu_Mk[3] * rtu_yk_idx_1;
  }

  /* End of Outputs for SubSystem: '<S99>/Enabled Subsystem' */

  /* Sum: '<S99>/Add' */
  localB->Add[0] = localB->Product2[0] + rtu_xhatkk1[0];
  localB->Add[1] = localB->Product2[1] + rtu_xhatkk1[1];
}

/* Function for MATLAB Function: '<S3>/EstimatorAttitude' */
static real_T DroneRS_Compensator_genpnorm(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 2.2250738585072014E-308;
  absxk = fabs(x[0]);
  if (absxk > 2.2250738585072014E-308) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 2.2250738585072014E-308;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Initial conditions for atomic system: '<Root>/DroneRS_Compensator' */
void DroneR_DroneRS_Compensator_Init(DW_DroneRS_Compensator_DroneR_T *localDW,
  P_DroneRS_Compensator_DroneRS_T *localP)
{
  int32_T i;

  /* InitializeConditions for DiscreteFir: '<S15>/FIRaccelero' */
  localDW->FIRaccelero_circBuf = 0;
  for (i = 0; i < 15; i++) {
    localDW->FIRaccelero_states[i] = localP->FIRaccelero_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFir: '<S15>/FIRaccelero' */

  /* InitializeConditions for DiscreteFilter: '<S15>/IIRgyroz' */
  for (i = 0; i < 5; i++) {
    localDW->IIRgyroz_states[i] = localP->IIRgyroz_InitialStates;
  }

  /* End of InitializeConditions for DiscreteFilter: '<S15>/IIRgyroz' */

  /* InitializeConditions for MATLAB Function: '<S3>/EstimatorAttitude' */
  localDW->yaw_cur = 0.0;
  localDW->pitch_cur = 0.0;
  localDW->roll_cur = 0.0;

  /* InitializeConditions for Delay: '<S71>/Delay' */
  localDW->Delay_DSTATE[0] = localP->Delay_InitialCondition;
  localDW->Delay_DSTATE[1] = localP->Delay_InitialCondition;

  /* InitializeConditions for DiscreteFilter: '<S74>/IIRgyroz' */
  for (i = 0; i < 10; i++) {
    localDW->IIRgyroz_states_n[i] = localP->IIRgyroz_InitialStates_c;
  }

  /* End of InitializeConditions for DiscreteFilter: '<S74>/IIRgyroz' */

  /* InitializeConditions for UnitDelay: '<S121>/UD' */
  localDW->UD_DSTATE[0] = localP->DiscreteDerivative_ICPrevScaled;
  localDW->UD_DSTATE[1] = localP->DiscreteDerivative_ICPrevScaled;

  /* InitializeConditions for Delay: '<S12>/Delay2' */
  localDW->Delay2_DSTATE = localP->Delay2_InitialCondition;
  for (i = 0; i < 5; i++) {
    /* InitializeConditions for DiscreteFilter: '<S17>/IIRprs' */
    localDW->IIRprs_states[i] = localP->IIRprs_InitialStates;

    /* InitializeConditions for DiscreteFilter: '<S17>/IIRsonar' */
    localDW->IIRsonar_states[i] = localP->IIRsonar_InitialStates;
  }

  /* InitializeConditions for Delay: '<S16>/MemoryX' */
  localDW->icLoad = 1U;

  /* InitializeConditions for Delay: '<S70>/Delay' */
  localDW->Delay_DSTATE_l[0] = localP->Delay_InitialCondition_n;
  localDW->Delay_DSTATE_l[1] = localP->Delay_InitialCondition_n;

  /* InitializeConditions for Delay: '<S73>/MemoryX' */
  localDW->icLoad_c = 1U;

  /* InitializeConditions for Delay: '<S3>/Delay1' */
  localDW->Delay1_DSTATE[0] = localP->Delay1_InitialCondition;
  localDW->Delay1_DSTATE[1] = localP->Delay1_InitialCondition;

  /* InitializeConditions for Delay: '<S133>/MemoryX' */
  localDW->icLoad_e = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S71>/SimplyIntegrateVelocity' */
  localDW->SimplyIntegrateVelocity_DSTATE[0] =
    localP->SimplyIntegrateVelocity_IC;
  localDW->SimplyIntegrateVelocity_DSTATE[1] =
    localP->SimplyIntegrateVelocity_IC;
  localDW->SimplyIntegrateVelocity_PrevRes = 2;
}

/* Output and update for atomic system: '<Root>/DroneRS_Compensator' */
void DroneRS_Com_DroneRS_Compensator(boolean_T rtu_controlModePosVSAtt_flagin,
  const real_T rtu_pos_refin[3], const real_T rtu_attRS_refin[3], real_T
  rtu_sensordataRS_datin, real_T rtu_sensordataRS_datin_j, real_T
  rtu_sensordataRS_datin_n, real_T rtu_sensordataRS_datin_jr, real_T
  rtu_sensordataRS_datin_k, real_T rtu_sensordataRS_datin_c, real_T
  rtu_sensordataRS_datin_g, real_T rtu_sensordataRS_datin_gz, const real_T
  rtu_opticalFlowRS_datin[3], const real_T rtu_sensordatabiasRS_datin[7], const
  real_T rtu_posVIS_datin[4], real_T rtu_usePosVIS_flagin, const real_T
  rtu_batteryStatus_datin[2], B_DroneRS_Compensator_DroneRS_T *localB,
  DW_DroneRS_Compensator_DroneR_T *localDW, P_DroneRS_Compensator_DroneRS_T
  *localP, P_DroneRS_Compensator_T *DroneRS_Compensator_P)
{
  /* local block i/o variables */
  real_T rtb_invertzaxisGain;
  real_T rtb_altfrompress;
  real_T rtb_opticalFlowToVelocity_Gain[3];
  real_T rtb_w_euler[3];
  real_T rtb_Dk1uk1;
  real_T rtb_Ckxhatkk1_b;
  boolean_T rtb_LogicalOperator3_b;
  int32_T k;
  int32_T j;
  int32_T memOffset;
  real_T g;
  real_T scale;
  real_T t;
  real_T rtb_Sum[3];
  real_T rtb_TmpSignalConversionAtSFun_b[6];
  boolean_T rtb_LogicalOperator3;
  real_T rtb_vel_world[3];
  real_T rtb_r;
  real_T rtb_Dk1uk1_m[2];
  real_T rtb_Add1[2];
  boolean_T rtb_Compare_f;
  boolean_T rtb_Compare_lx;
  boolean_T rtb_Compare_ok;
  boolean_T rtb_Compare_k;
  boolean_T rtb_Compare_jr;
  real_T inversesIMU_Gain[6];
  real_T IIRsonar_tmp;
  real_T IIRgyroz_tmp;
  int32_T i;
  real_T tmp[9];
  real_T tmp_0[9];
  real_T tmp_1[9];
  real_T tmp_2[9];
  real_T tmp_3[9];
  real_T tmp_4[9];
  real_T rtb_TSamp_idx_0;
  real_T rtb_TSamp_idx_1;

  /* Inport: '<S1>/controlModePosVSAtt_flagin' */
  localB->controlModePosVSAtt_flagin = rtu_controlModePosVSAtt_flagin;

  /* Inport: '<S1>/posVIS_datin' */
  localB->posVIS_datin[0] = rtu_posVIS_datin[0];
  localB->posVIS_datin[1] = rtu_posVIS_datin[1];
  localB->posVIS_datin[2] = rtu_posVIS_datin[2];
  localB->posVIS_datin[3] = rtu_posVIS_datin[3];

  /* Inport: '<S1>/sensordatabiasRS_datin' */
  for (i = 0; i < 7; i++) {
    localB->sensordatabiasRS_datin[i] = rtu_sensordatabiasRS_datin[i];
  }

  /* End of Inport: '<S1>/sensordatabiasRS_datin' */

  /* Inport: '<S1>/sensordataRS_datin' */
  localB->sensordataRS_datin[0] = rtu_sensordataRS_datin;
  localB->sensordataRS_datin[1] = rtu_sensordataRS_datin_j;
  localB->sensordataRS_datin[2] = rtu_sensordataRS_datin_n;
  localB->sensordataRS_datin[3] = rtu_sensordataRS_datin_jr;
  localB->sensordataRS_datin[4] = rtu_sensordataRS_datin_k;
  localB->sensordataRS_datin[5] = rtu_sensordataRS_datin_c;
  localB->sensordataRS_datin[6] = rtu_sensordataRS_datin_g;
  localB->sensordataRS_datin[7] = rtu_sensordataRS_datin_gz;

  /* Inport: '<S1>/usePosVIS_flagin' */
  localB->usePosVIS_flagin = rtu_usePosVIS_flagin;

  /* Inport: '<S1>/opticalFlowRS_datin' */
  localB->opticalFlowRS_datin[0] = rtu_opticalFlowRS_datin[0];
  localB->opticalFlowRS_datin[1] = rtu_opticalFlowRS_datin[1];
  localB->opticalFlowRS_datin[2] = rtu_opticalFlowRS_datin[2];

  /* Gain: '<S15>/inversesIMU_Gain' incorporates:
   *  Sum: '<S15>/Sum1'
   */
  for (i = 0; i < 6; i++) {
    inversesIMU_Gain[i] = (localB->sensordataRS_datin[i] -
      localB->sensordatabiasRS_datin[i]) *
      DroneRS_Compensator_P->quadEDT.inversesIMU_Gain[i];
  }

  /* End of Gain: '<S15>/inversesIMU_Gain' */

  /* DiscreteFir: '<S15>/FIRaccelero' */
  scale = inversesIMU_Gain[0] * localP->FIRaccelero_Coefficients[0];
  i = 1;
  for (j = localDW->FIRaccelero_circBuf; j < 5; j++) {
    scale += localDW->FIRaccelero_states[j] * localP->FIRaccelero_Coefficients[i];
    i++;
  }

  for (j = 0; j < localDW->FIRaccelero_circBuf; j++) {
    scale += localDW->FIRaccelero_states[j] * localP->FIRaccelero_Coefficients[i];
    i++;
  }

  localB->FIRaccelero[0] = scale;
  scale = inversesIMU_Gain[1] * localP->FIRaccelero_Coefficients[0];
  i = 1;
  for (j = localDW->FIRaccelero_circBuf; j < 5; j++) {
    scale += localDW->FIRaccelero_states[5 + j] *
      localP->FIRaccelero_Coefficients[i];
    i++;
  }

  for (j = 0; j < localDW->FIRaccelero_circBuf; j++) {
    scale += localDW->FIRaccelero_states[5 + j] *
      localP->FIRaccelero_Coefficients[i];
    i++;
  }

  localB->FIRaccelero[1] = scale;
  scale = inversesIMU_Gain[2] * localP->FIRaccelero_Coefficients[0];
  i = 1;
  for (j = localDW->FIRaccelero_circBuf; j < 5; j++) {
    scale += localDW->FIRaccelero_states[10 + j] *
      localP->FIRaccelero_Coefficients[i];
    i++;
  }

  for (j = 0; j < localDW->FIRaccelero_circBuf; j++) {
    scale += localDW->FIRaccelero_states[10 + j] *
      localP->FIRaccelero_Coefficients[i];
    i++;
  }

  localB->FIRaccelero[2] = scale;

  /* End of DiscreteFir: '<S15>/FIRaccelero' */

  /* DiscreteFilter: '<S15>/IIRgyroz' */
  IIRgyroz_tmp = inversesIMU_Gain[5];
  i = 1;
  for (j = 0; j < 5; j++) {
    IIRgyroz_tmp -= DroneRS_Compensator_P->altEstim.filter_a_gyroz[i] *
      localDW->IIRgyroz_states[j];
    i++;
  }

  IIRgyroz_tmp /= DroneRS_Compensator_P->altEstim.filter_a_gyroz[0];
  rtb_r = DroneRS_Compensator_P->altEstim.filter_b_gyroz[0] * IIRgyroz_tmp;
  i = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += DroneRS_Compensator_P->altEstim.filter_b_gyroz[i] *
      localDW->IIRgyroz_states[j];
    i++;
  }

  /* End of DiscreteFilter: '<S15>/IIRgyroz' */

  /* SignalConversion: '<S13>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  MATLAB Function: '<S3>/EstimatorAttitude'
   */
  rtb_TmpSignalConversionAtSFun_b[0] = localB->FIRaccelero[0];
  rtb_TmpSignalConversionAtSFun_b[1] = localB->FIRaccelero[1];
  rtb_TmpSignalConversionAtSFun_b[2] = localB->FIRaccelero[2];
  rtb_TmpSignalConversionAtSFun_b[3] = inversesIMU_Gain[3];
  rtb_TmpSignalConversionAtSFun_b[4] = inversesIMU_Gain[4];
  rtb_TmpSignalConversionAtSFun_b[5] = rtb_r;

  /* MATLAB Function: '<S3>/EstimatorAttitude' incorporates:
   *  Constant: '<S186>/Constant'
   *  Constant: '<S3>/sampleTime'
   *  Logic: '<S15>/Logical Operator'
   *  RelationalOperator: '<S186>/Compare'
   *  SignalConversion: '<S13>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'DroneRS_Compensator/Estimator/EstimatorAttitude': '<S13>:1' */
  /* '<S13>:1:4' */
  /* '<S13>:1:5' */
  /* '<S13>:1:6' */
  /* '<S13>:1:8' */
  g = DroneRS_Compensator_genpnorm(*(real_T (*)[3])&
    localB->sensordatabiasRS_datin[0]);

  /* % Estimating attitude */
  /*  =============================== */
  /*  PURPOSE estimates the attitude fusing gyroscopic and accelerometer measurements with potentially available vision updates on yaw */
  /*  INSPIRED by http://www.pieter-jan.com/node/11  */
  /*  ADAPTED by Fabian Riether */
  /*  UPDATE DATE 2015/08/25 */
  /*  SPECIAL NOTES */
  /*  =============================== */
  /*  Change History */
  /*   2015/08/25 created */
  /*  ================================== */
  /* sensorupdates 1-3 acc[1-3], 4-6 gyr[1-3] */
  /* 65.536; */
  /* Rotation of angular velocity vector from Bodyframe to Worldframe, inverted Wronskian (body rates p-q-r to euler rates yaw pitch roll) */
  scale = cos(localDW->pitch_cur);
  tmp[0] = 0.0;
  tmp[3] = sin(localDW->roll_cur);
  tmp[6] = cos(localDW->roll_cur);
  tmp[1] = 0.0;
  tmp[4] = cos(localDW->roll_cur) * cos(localDW->pitch_cur);
  tmp[7] = -sin(localDW->roll_cur) * cos(localDW->pitch_cur);
  tmp[2] = cos(localDW->pitch_cur);
  tmp[5] = sin(localDW->roll_cur) * sin(localDW->pitch_cur);
  tmp[8] = cos(localDW->roll_cur) * sin(localDW->pitch_cur);
  for (i = 0; i < 3; i++) {
    tmp_0[3 * i] = tmp[3 * i] / scale;
    tmp_0[1 + 3 * i] = tmp[3 * i + 1] / scale;
    tmp_0[2 + 3 * i] = tmp[3 * i + 2] / scale;
  }

  for (i = 0; i < 3; i++) {
    rtb_w_euler[i] = 0.0;
    rtb_w_euler[i] += tmp_0[i] * rtb_TmpSignalConversionAtSFun_b[3];
    rtb_w_euler[i] += tmp_0[i + 3] * rtb_TmpSignalConversionAtSFun_b[4];
    rtb_w_euler[i] += tmp_0[i + 6] * rtb_r;
  }

  /* Integrate gyroscope data */
  rtb_r = rtb_w_euler[2] * DroneRS_Compensator_P->sampleTime_qcsim +
    localDW->roll_cur;

  /* Angle around pitch_cur X-axis */
  scale = rtb_w_euler[1] * DroneRS_Compensator_P->sampleTime_qcsim +
    localDW->pitch_cur;

  /* Angle around pitch_cur Y-axis */
  IIRsonar_tmp = rtb_w_euler[0] * DroneRS_Compensator_P->sampleTime_qcsim +
    localDW->yaw_cur;

  /* Angle around pitch_cur Z-axis   */
  /* Compensate for drift with accelerometer data if un-accelerated flight */
  t = DroneRS_Compensator_genpnorm(*(real_T (*)[3])&
    rtb_TmpSignalConversionAtSFun_b[0]);

  /* 0.03 */
  if ((t > 0.993 * g) && (t < 1.007 * g)) {
    rtb_r = (rt_atan2d_snf(localB->FIRaccelero[2], -localB->FIRaccelero[1]) +
             1.5707963267948966) * 0.001 + rtb_r * 0.999;

    /* pitchAcc = atan2(sensor_updates(1), sensor_updates(3)) ; */
    scale = (rt_atan2d_snf(localB->FIRaccelero[2], localB->FIRaccelero[0]) +
             1.5707963267948966) * cos(rtb_r) * 0.001 + scale * 0.999;
  }

  /* compensate yaw-bias/drift to world yaw if measurement form vision available */
  if ((localB->posVIS_datin[0] != localP->CompareToConstant_const) &&
      (localB->usePosVIS_flagin != 0.0)) {
    IIRsonar_tmp = 0.98 * IIRsonar_tmp + 0.02 * localB->posVIS_datin[3];
  }

  /* '<S13>:1:22' */
  /* '<S13>:1:23' */
  localDW->yaw_cur = IIRsonar_tmp;

  /* '<S13>:1:24' */
  localDW->pitch_cur = scale;

  /* '<S13>:1:25' */
  localDW->roll_cur = rtb_r;

  /* '<S13>:1:27' */
  /* '<S13>:1:28' */
  localB->att_estimout[0] = IIRsonar_tmp;
  localB->att_estimout[1] = scale;
  localB->att_estimout[2] = rtb_r;
  localB->datt_estimout[0] = inversesIMU_Gain[3];
  localB->datt_estimout[1] = inversesIMU_Gain[4];
  localB->datt_estimout[2] = rtb_TmpSignalConversionAtSFun_b[5];

  /* MATLAB Function: '<S134>/abs' incorporates:
   *  Delay: '<S71>/Delay'
   *  Sum: '<S134>/Add1'
   */
  /* MATLAB Function 'DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs': '<S181>:1' */
  /* '<S181>:1:2' */
  scale = 2.2250738585072014E-308;
  IIRsonar_tmp = fabs(localB->posVIS_datin[0] - localDW->Delay_DSTATE[0]);
  if (IIRsonar_tmp > 2.2250738585072014E-308) {
    rtb_r = 1.0;
    scale = IIRsonar_tmp;
  } else {
    t = IIRsonar_tmp / 2.2250738585072014E-308;
    rtb_r = t * t;
  }

  IIRsonar_tmp = fabs(localB->posVIS_datin[1] - localDW->Delay_DSTATE[1]);
  if (IIRsonar_tmp > scale) {
    t = scale / IIRsonar_tmp;
    rtb_r = rtb_r * t * t + 1.0;
    scale = IIRsonar_tmp;
  } else {
    t = IIRsonar_tmp / scale;
    rtb_r += t * t;
  }

  rtb_r = scale * sqrt(rtb_r);

  /* End of MATLAB Function: '<S134>/abs' */

  /* Logic: '<S134>/Logical Operator3' incorporates:
   *  Constant: '<S182>/Constant'
   *  Constant: '<S183>/Constant'
   *  Constant: '<S184>/Constant'
   *  Constant: '<S185>/Constant'
   *  RelationalOperator: '<S182>/Compare'
   *  RelationalOperator: '<S183>/Compare'
   *  RelationalOperator: '<S184>/Compare'
   *  RelationalOperator: '<S185>/Compare'
   */
  rtb_LogicalOperator3 = ((localB->posVIS_datin[0] !=
    localP->checkPosavailable_const) && (localB->att_estimout[1] <=
    DroneRS_Compensator_P->vishandle.att_UpperLimit) && (localB->att_estimout[2]
    <= DroneRS_Compensator_P->vishandle.att_UpperLimit) && (rtb_r <
    DroneRS_Compensator_P->vishandle.deltaXY));

  /* Abs: '<S74>/Abs2' */
  rtb_Dk1uk1 = fabs(localB->att_estimout[1]);

  /* RelationalOperator: '<S124>/Compare' incorporates:
   *  Constant: '<S124>/Constant'
   */
  rtb_Compare_jr = (rtb_Dk1uk1 <=
                    DroneRS_Compensator_P->ofhandle.pitchroll_UpperLimit);

  /* Abs: '<S74>/Abs3' */
  rtb_Dk1uk1 = fabs(localB->att_estimout[2]);

  /* RelationalOperator: '<S126>/Compare' incorporates:
   *  Constant: '<S126>/Constant'
   */
  rtb_Compare_k = (rtb_Dk1uk1 <=
                   DroneRS_Compensator_P->ofhandle.pitchroll_UpperLimit);

  /* Abs: '<S74>/Abs' */
  rtb_Dk1uk1 = fabs(inversesIMU_Gain[3]);

  /* RelationalOperator: '<S128>/Compare' incorporates:
   *  Constant: '<S128>/Constant'
   */
  rtb_Compare_ok = (rtb_Dk1uk1 <= DroneRS_Compensator_P->ofhandle.pq_UpperLimit);

  /* Abs: '<S74>/Abs1' */
  rtb_Dk1uk1 = fabs(inversesIMU_Gain[4]);

  /* RelationalOperator: '<S129>/Compare' incorporates:
   *  Constant: '<S129>/Constant'
   */
  rtb_Compare_lx = (rtb_Dk1uk1 <= DroneRS_Compensator_P->ofhandle.pq_UpperLimit);

  /* DiscreteFilter: '<S74>/IIRgyroz' */
  for (k = 0; k < 2; k++) {
    memOffset = k * 5;
    scale = inversesIMU_Gain[k + 3];
    i = 1;
    for (j = 0; j < 5; j++) {
      scale -= localDW->IIRgyroz_states_n[memOffset + j] *
        DroneRS_Compensator_P->altEstim.filter_a_gyroz[i];
      i++;
    }

    scale /= DroneRS_Compensator_P->altEstim.filter_a_gyroz[0];
    localDW->IIRgyroz_tmp_f[k] = scale;
    rtb_r = DroneRS_Compensator_P->altEstim.filter_b_gyroz[0] *
      localDW->IIRgyroz_tmp_f[k];
    i = 1;
    for (j = 0; j < 5; j++) {
      rtb_r += localDW->IIRgyroz_states_n[memOffset + j] *
        DroneRS_Compensator_P->altEstim.filter_b_gyroz[i];
      i++;
    }

    rtb_Dk1uk1_m[k] = rtb_r;
  }

  /* End of DiscreteFilter: '<S74>/IIRgyroz' */

  /* SampleTimeMath: '<S121>/TSamp'
   *
   * About '<S121>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_idx_0 = rtb_Dk1uk1_m[0] * localP->TSamp_WtEt;
  rtb_TSamp_idx_1 = rtb_Dk1uk1_m[1] * localP->TSamp_WtEt;

  /* Abs: '<S74>/Abs6' incorporates:
   *  Sum: '<S121>/Diff'
   *  UnitDelay: '<S121>/UD'
   */
  rtb_Dk1uk1 = fabs(rtb_TSamp_idx_0 - localDW->UD_DSTATE[0]);

  /* RelationalOperator: '<S122>/Compare' incorporates:
   *  Constant: '<S122>/Constant'
   */
  rtb_Compare_f = (rtb_Dk1uk1 <= DroneRS_Compensator_P->ofhandle.dpq_UpperLimit);

  /* Abs: '<S74>/Abs7' incorporates:
   *  Sum: '<S121>/Diff'
   *  UnitDelay: '<S121>/UD'
   */
  rtb_Dk1uk1 = fabs(rtb_TSamp_idx_1 - localDW->UD_DSTATE[1]);

  /* Logic: '<S74>/Logical Operator' incorporates:
   *  Constant: '<S123>/Constant'
   *  RelationalOperator: '<S123>/Compare'
   */
  rtb_Compare_lx = (rtb_Compare_jr && rtb_Compare_k && rtb_Compare_ok &&
                    rtb_Compare_lx && rtb_Compare_f && (rtb_Dk1uk1 <=
    DroneRS_Compensator_P->ofhandle.dpq_UpperLimit));

  /* Gain: '<S12>/invertzaxisGain' */
  rtb_invertzaxisGain = localP->invertzaxisGain_Gain *
    localB->sensordataRS_datin[6];

  /* Delay: '<S12>/Delay2' */
  rtb_Dk1uk1 = localDW->Delay2_DSTATE;

  /* Saturate: '<S17>/SaturationSonar' */
  if (rtb_invertzaxisGain > -DroneRS_Compensator_P->quadEDT.altSenor_LowerLimit)
  {
    g = -DroneRS_Compensator_P->quadEDT.altSenor_LowerLimit;
  } else if (rtb_invertzaxisGain < localP->SaturationSonar_LowerSat) {
    g = localP->SaturationSonar_LowerSat;
  } else {
    g = rtb_invertzaxisGain;
  }

  /* Sum: '<S17>/Add' incorporates:
   *  Saturate: '<S17>/SaturationSonar'
   */
  rtb_Ckxhatkk1_b = rtb_Dk1uk1 - g;

  /* Abs: '<S17>/Absestdiff' */
  rtb_Ckxhatkk1_b = fabs(rtb_Ckxhatkk1_b);

  /* RelationalOperator: '<S69>/Compare' incorporates:
   *  Constant: '<S69>/Constant'
   */
  rtb_Compare_jr = (rtb_Ckxhatkk1_b <=
                    DroneRS_Compensator_P->altEstim.outlierJump_UpperLimit);

  /* Gain: '<S12>/prsToAlt_Gain' incorporates:
   *  Sum: '<S15>/Sum2'
   */
  rtb_altfrompress = 1.0 / DroneRS_Compensator_P->quadEDT.altToPrs_Gain *
    (localB->sensordataRS_datin[7] - localB->sensordatabiasRS_datin[6]);

  /* DiscreteFilter: '<S17>/IIRprs' */
  scale = rtb_altfrompress;
  i = 1;
  for (j = 0; j < 5; j++) {
    scale -= DroneRS_Compensator_P->altEstim.filter_a_prs[i] *
      localDW->IIRprs_states[j];
    i++;
  }

  scale /= DroneRS_Compensator_P->altEstim.filter_a_prs[0];
  rtb_r = DroneRS_Compensator_P->altEstim.filter_b_prs[0] * scale;
  i = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += DroneRS_Compensator_P->altEstim.filter_b_prs[i] *
      localDW->IIRprs_states[j];
    i++;
  }

  rtb_Ckxhatkk1_b = rtb_r;

  /* End of DiscreteFilter: '<S17>/IIRprs' */

  /* Sum: '<S17>/Add1' */
  rtb_Ckxhatkk1_b -= rtb_Dk1uk1;

  /* Abs: '<S17>/Absestdiff1' */
  rtb_Ckxhatkk1_b = fabs(rtb_Ckxhatkk1_b);

  /* RelationalOperator: '<S66>/Compare' incorporates:
   *  Constant: '<S66>/Constant'
   */
  rtb_Compare_k = (rtb_Ckxhatkk1_b >=
                   DroneRS_Compensator_P->altEstim.stateDeviationPrs_Threshold);

  /* DiscreteFilter: '<S17>/IIRsonar' */
  IIRsonar_tmp = rtb_invertzaxisGain;
  i = 1;
  for (j = 0; j < 5; j++) {
    IIRsonar_tmp -= DroneRS_Compensator_P->altEstim.filter_a_prs[i] *
      localDW->IIRsonar_states[j];
    i++;
  }

  IIRsonar_tmp /= DroneRS_Compensator_P->altEstim.filter_a_prs[0];
  rtb_r = DroneRS_Compensator_P->altEstim.filter_b_prs[0] * IIRsonar_tmp;
  i = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += DroneRS_Compensator_P->altEstim.filter_b_prs[i] *
      localDW->IIRsonar_states[j];
    i++;
  }

  rtb_Ckxhatkk1_b = rtb_r;

  /* End of DiscreteFilter: '<S17>/IIRsonar' */

  /* Sum: '<S17>/Add2' */
  rtb_Ckxhatkk1_b -= rtb_Dk1uk1;

  /* Abs: '<S17>/Absestdiff2' */
  rtb_Ckxhatkk1_b = fabs(rtb_Ckxhatkk1_b);

  /* Logic: '<S17>/nicemeasurementor newupdateneeded' incorporates:
   *  Constant: '<S67>/Constant'
   *  Constant: '<S68>/Constant'
   *  Logic: '<S17>/findingoutliers'
   *  Logic: '<S17>/newupdateneeded'
   *  RelationalOperator: '<S67>/Compare'
   *  RelationalOperator: '<S68>/Compare'
   */
  rtb_Compare_ok = ((rtb_Compare_jr && (rtb_invertzaxisGain <
    -DroneRS_Compensator_P->quadEDT.altSenor_LowerLimit)) || (rtb_Compare_k &&
    (rtb_Ckxhatkk1_b >=
     DroneRS_Compensator_P->altEstim.stateDeviationSonflt_Threshold)));

  /* MATLAB Function: '<S12>/RStoWorldinacc' */
  /* MATLAB Function 'DroneRS_Compensator/Estimator/EstimatorAltitude/RStoWorldinacc': '<S18>:1' */
  /* '<S18>:1:2' */
  /* '<S18>:1:3' */
  /* '<S18>:1:4' */
  /* BBF > Inertial rotation matrix */
  /* '<S18>:1:13' */
  /* '<S18>:1:17' */
  tmp_1[0] = cos(localB->att_estimout[1]) * cos(localB->att_estimout[0]);
  tmp_1[3] = sin(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * cos
    (localB->att_estimout[0]) - cos(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_1[6] = cos(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * cos
    (localB->att_estimout[0]) + sin(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_1[1] = cos(localB->att_estimout[1]) * sin(localB->att_estimout[0]);
  tmp_1[4] = sin(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[0]) + cos(localB->att_estimout[2]) * cos
    (localB->att_estimout[0]);
  tmp_1[7] = cos(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[0]) - sin(localB->att_estimout[2]) * cos
    (localB->att_estimout[0]);
  tmp_1[2] = -sin(localB->att_estimout[1]);
  tmp_1[5] = sin(localB->att_estimout[2]) * cos(localB->att_estimout[1]);
  tmp_1[8] = cos(localB->att_estimout[2]) * cos(localB->att_estimout[1]);

  /* Sum: '<S12>/Sum' incorporates:
   *  Constant: '<S12>/gravity'
   *  MATLAB Function: '<S12>/RStoWorldinacc'
   */
  for (i = 0; i < 3; i++) {
    rtb_Sum[i] = ((tmp_1[i + 3] * localB->FIRaccelero[1] + tmp_1[i] *
                   localB->FIRaccelero[0]) + tmp_1[i + 6] * localB->FIRaccelero
                  [2]) + localP->gravity_Value[i];
  }

  /* End of Sum: '<S12>/Sum' */

  /* Delay: '<S16>/MemoryX' incorporates:
   *  Constant: '<S16>/X0'
   *  Constant: '<S20>/Constant'
   *  RelationalOperator: '<S20>/Compare'
   */
  if (rtb_Dk1uk1 > localP->outlierBelowFloor_const) {
    localDW->icLoad = 1U;
  }

  if (localDW->icLoad != 0) {
    localDW->MemoryX_DSTATE[0] = localP->X0_Value[0];
    localDW->MemoryX_DSTATE[1] = localP->X0_Value[1];
  }

  /* Outputs for Atomic SubSystem: '<S16>/UseCurrentEstimator' */
  /* Outputs for Enabled SubSystem: '<S44>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S65>/Enable'
   */
  if (rtb_Compare_ok) {
    /* Sum: '<S65>/Add1' incorporates:
     *  Constant: '<S16>/C'
     *  Constant: '<S16>/D'
     *  Delay: '<S16>/MemoryX'
     *  Product: '<S65>/Product'
     *  Product: '<S65>/Product1'
     */
    rtb_r = (rtb_invertzaxisGain - (localP->C_Value[0] * localDW->
              MemoryX_DSTATE[0] + localP->C_Value[1] * localDW->MemoryX_DSTATE[1]))
      - localP->D_Value * rtb_Sum[2];

    /* Product: '<S65>/Product2' incorporates:
     *  Constant: '<S21>/KalmanGainM'
     */
    localB->Product2[0] = localP->KalmanGainM_Value_h[0] * rtb_r;
    localB->Product2[1] = localP->KalmanGainM_Value_h[1] * rtb_r;
  }

  /* End of Outputs for SubSystem: '<S44>/Enabled Subsystem' */

  /* Reshape: '<S16>/Reshapexhat' incorporates:
   *  Delay: '<S16>/MemoryX'
   *  Sum: '<S44>/Add'
   */
  localB->Reshapexhat[0] = localB->Product2[0] + localDW->MemoryX_DSTATE[0];
  localB->Reshapexhat[1] = localB->Product2[1] + localDW->MemoryX_DSTATE[1];

  /* End of Outputs for SubSystem: '<S16>/UseCurrentEstimator' */

  /* Abs: '<S74>/Abs4' */
  rtb_Ckxhatkk1_b = fabs(inversesIMU_Gain[3]);

  /* RelationalOperator: '<S125>/Compare' incorporates:
   *  Constant: '<S125>/Constant'
   */
  rtb_Compare_jr = (rtb_Ckxhatkk1_b <=
                    DroneRS_Compensator_P->ofhandle.pq_UpperLimit_hov);

  /* Abs: '<S74>/Abs5' */
  rtb_Ckxhatkk1_b = fabs(inversesIMU_Gain[4]);

  /* Logic: '<S74>/Logical Operator1' incorporates:
   *  Constant: '<S127>/Constant'
   *  Constant: '<S132>/Constant'
   *  RelationalOperator: '<S127>/Compare'
   *  RelationalOperator: '<S132>/Compare'
   */
  rtb_Compare_jr = ((localB->Reshapexhat[0] <=
                     DroneRS_Compensator_P->ofhandle.Z_UpperLimit) &&
                    rtb_Compare_jr && (rtb_Ckxhatkk1_b <=
    DroneRS_Compensator_P->ofhandle.pq_UpperLimit_hov));

  /* Gain: '<S70>/opticalFlowToVelocity_Gain' */
  rtb_r = 1.0 / DroneRS_Compensator_P->quadEDT.VelocityToOpticalFlow_Gain;
  rtb_opticalFlowToVelocity_Gain[0] = rtb_r * localB->opticalFlowRS_datin[0];
  rtb_opticalFlowToVelocity_Gain[1] = rtb_r * localB->opticalFlowRS_datin[1];
  rtb_opticalFlowToVelocity_Gain[2] = rtb_r * localB->opticalFlowRS_datin[2];

  /* Abs: '<S74>/Abs8' incorporates:
   *  Delay: '<S70>/Delay'
   *  Sum: '<S74>/Add'
   */
  rtb_Ckxhatkk1_b = fabs(rtb_opticalFlowToVelocity_Gain[0] -
    localDW->Delay_DSTATE_l[0]);

  /* RelationalOperator: '<S130>/Compare' incorporates:
   *  Constant: '<S130>/Constant'
   */
  rtb_Compare_k = (rtb_Ckxhatkk1_b <=
                   DroneRS_Compensator_P->ofhandle.deltadxy_UpperLimit);

  /* Abs: '<S74>/Abs9' incorporates:
   *  Delay: '<S70>/Delay'
   *  Sum: '<S74>/Add'
   */
  rtb_Ckxhatkk1_b = fabs(rtb_opticalFlowToVelocity_Gain[1] -
    localDW->Delay_DSTATE_l[1]);

  /* Logic: '<S74>/Logical Operator3' incorporates:
   *  Constant: '<S131>/Constant'
   *  Logic: '<S74>/Logical Operator2'
   *  RelationalOperator: '<S131>/Compare'
   */
  rtb_LogicalOperator3_b = ((rtb_Compare_lx || rtb_Compare_jr) && rtb_Compare_k &&
    (rtb_Ckxhatkk1_b <= DroneRS_Compensator_P->ofhandle.deltadxy_UpperLimit));

  /* MATLAB Function: '<S72>/World2Body' */
  /* MATLAB Function 'DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld/World2Body': '<S75>:1' */
  /* '<S75>:1:3' */
  /* '<S75>:1:4' */
  /* '<S75>:1:5' */
  /*  rotz(yaw)*roty(pitch)*rotx(roll) */
  /*      R_RTBBodytoGlobal = [cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);   %BBF > Inertial rotation matrix */
  /*           cos(pitch)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); */
  /*           -sin(pitch)         sin(roll)*cos(pitch)                            cos(roll)*cos(pitch)]; */
  /* '<S75>:1:13' */
  /*  Rw_Global2RTBBody  = [0        sin(roll)          cos(roll);             %inverted Wronskian */
  /*            0        cos(roll)*cos(pitch) -sin(roll)*cos(pitch); */
  /*            cos(pitch) sin(roll)*sin(pitch) cos(roll)*sin(pitch)] / cos(pitch); */
  /* '<S75>:1:23' */
  /* y = -([cos(pi/4) sin(pi/4); -sin(pi/4) cos(pi/4)]*u); */
  tmp_2[0] = cos(localB->att_estimout[1]) * cos(localB->att_estimout[0]);
  tmp_2[3] = cos(localB->att_estimout[1]) * sin(localB->att_estimout[0]);
  tmp_2[6] = -sin(localB->att_estimout[1]);
  tmp_2[1] = cos(localB->att_estimout[0]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[2]) - cos(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_2[4] = sin(localB->att_estimout[1]) * sin(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]) + cos(localB->att_estimout[2]) * cos
    (localB->att_estimout[0]);
  tmp_2[7] = cos(localB->att_estimout[1]) * sin(localB->att_estimout[2]);
  tmp_2[2] = cos(localB->att_estimout[2]) * cos(localB->att_estimout[0]) * sin
    (localB->att_estimout[1]) + sin(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_2[5] = cos(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[0]) - cos(localB->att_estimout[0]) * sin
    (localB->att_estimout[2]);
  tmp_2[8] = cos(localB->att_estimout[1]) * cos(localB->att_estimout[2]);

  /* Sum: '<S72>/Add' incorporates:
   *  Constant: '<S72>/gravity'
   *  Gain: '<S72>/gainaccinput'
   *  MATLAB Function: '<S72>/World2Body'
   */
  for (i = 0; i < 3; i++) {
    rtb_vel_world[i] = localB->FIRaccelero[i] - ((tmp_2[i + 3] *
      localP->gravity_Value_g[1] + tmp_2[i] * localP->gravity_Value_g[0]) +
      tmp_2[i + 6] * localP->gravity_Value_g[2]);
  }

  /* End of Sum: '<S72>/Add' */

  /* Gain: '<S72>/gainaccinput' */
  rtb_Dk1uk1_m[0] = localP->gainaccinput_Gain * rtb_vel_world[0];
  rtb_Dk1uk1_m[1] = localP->gainaccinput_Gain * rtb_vel_world[1];

  /* Delay: '<S73>/MemoryX' incorporates:
   *  Constant: '<S73>/X0'
   */
  if (localDW->icLoad_c != 0) {
    localDW->MemoryX_DSTATE_f[0] = localP->X0_Value_k[0];
    localDW->MemoryX_DSTATE_f[1] = localP->X0_Value_k[1];
  }

  rtb_Add1[0] = localDW->MemoryX_DSTATE_f[0];
  rtb_Add1[1] = localDW->MemoryX_DSTATE_f[1];

  /* Outputs for Atomic SubSystem: '<S73>/UseCurrentEstimator' */

  /* Constant: '<S76>/KalmanGainM' incorporates:
   *  Constant: '<S73>/C'
   *  Constant: '<S73>/D'
   */
  DroneRS_Com_UseCurrentEstimator(rtb_LogicalOperator3_b,
    localP->KalmanGainM_Value_f, rtb_Dk1uk1_m, &rtb_opticalFlowToVelocity_Gain[0],
    localP->C_Value_d, localP->D_Value_f, rtb_Add1,
    &localB->UseCurrentEstimator_l);

  /* End of Outputs for SubSystem: '<S73>/UseCurrentEstimator' */

  /* Reshape: '<S73>/Reshapexhat' */
  localB->Reshapexhat_o[0] = localB->UseCurrentEstimator_l.Add[0];
  localB->Reshapexhat_o[1] = localB->UseCurrentEstimator_l.Add[1];

  /* SignalConversion: '<S19>/TmpSignal ConversionAt SFunction Inport2' incorporates:
   *  Delay: '<S3>/Delay1'
   *  MATLAB Function: '<S12>/WorldToRSinacc'
   */
  /* MATLAB Function 'DroneRS_Compensator/Estimator/EstimatorAltitude/WorldToRSinacc': '<S19>:1' */
  /* '<S19>:1:2' */
  /* '<S19>:1:3' */
  /* '<S19>:1:4' */
  /* '<S19>:1:7' */
  /* '<S19>:1:11' */
  g = localDW->Delay1_DSTATE[0];
  t = localDW->Delay1_DSTATE[1];

  /* MATLAB Function: '<S12>/WorldToRSinacc' incorporates:
   *  SignalConversion: '<S19>/TmpSignal ConversionAt SFunction Inport2'
   */
  tmp_3[0] = cos(localB->att_estimout[1]) * cos(localB->att_estimout[0]);
  tmp_3[3] = cos(localB->att_estimout[1]) * sin(localB->att_estimout[0]);
  tmp_3[6] = -sin(localB->att_estimout[1]);
  tmp_3[1] = cos(localB->att_estimout[0]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[2]) - cos(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_3[4] = sin(localB->att_estimout[1]) * sin(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]) + cos(localB->att_estimout[2]) * cos
    (localB->att_estimout[0]);
  tmp_3[7] = cos(localB->att_estimout[1]) * sin(localB->att_estimout[2]);
  tmp_3[2] = cos(localB->att_estimout[2]) * cos(localB->att_estimout[0]) * sin
    (localB->att_estimout[1]) + sin(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_3[5] = cos(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[0]) - cos(localB->att_estimout[0]) * sin
    (localB->att_estimout[2]);
  tmp_3[8] = cos(localB->att_estimout[1]) * cos(localB->att_estimout[2]);
  for (i = 0; i < 3; i++) {
    localB->acc_RS[i] = 0.0;
    localB->acc_RS[i] += tmp_3[i] * g;
    localB->acc_RS[i] += tmp_3[i + 3] * t;
    localB->acc_RS[i] += tmp_3[i + 6] * localB->Reshapexhat[1];
  }

  /* MATLAB Function: '<S71>/RStoWorld' incorporates:
   *  SignalConversion: '<S135>/TmpSignal ConversionAt SFunction Inport2'
   */
  /* MATLAB Function 'DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/RStoWorld': '<S135>:1' */
  /* '<S135>:1:2' */
  /* '<S135>:1:3' */
  /* '<S135>:1:4' */
  /* BBF > Inertial rotation matrix */
  /* '<S135>:1:13' */
  /* '<S135>:1:17' */
  tmp_4[0] = cos(localB->att_estimout[1]) * cos(localB->att_estimout[0]);
  tmp_4[3] = sin(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * cos
    (localB->att_estimout[0]) - cos(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_4[6] = cos(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * cos
    (localB->att_estimout[0]) + sin(localB->att_estimout[2]) * sin
    (localB->att_estimout[0]);
  tmp_4[1] = cos(localB->att_estimout[1]) * sin(localB->att_estimout[0]);
  tmp_4[4] = sin(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[0]) + cos(localB->att_estimout[2]) * cos
    (localB->att_estimout[0]);
  tmp_4[7] = cos(localB->att_estimout[2]) * sin(localB->att_estimout[1]) * sin
    (localB->att_estimout[0]) - sin(localB->att_estimout[2]) * cos
    (localB->att_estimout[0]);
  tmp_4[2] = -sin(localB->att_estimout[1]);
  tmp_4[5] = sin(localB->att_estimout[2]) * cos(localB->att_estimout[1]);
  tmp_4[8] = cos(localB->att_estimout[2]) * cos(localB->att_estimout[1]);
  for (i = 0; i < 3; i++) {
    rtb_vel_world[i] = tmp_4[i + 6] * localB->acc_RS[2] + (tmp_4[i + 3] *
      localB->Reshapexhat_o[1] + tmp_4[i] * localB->Reshapexhat_o[0]);
  }

  /* End of MATLAB Function: '<S71>/RStoWorld' */

  /* Delay: '<S133>/MemoryX' incorporates:
   *  Constant: '<S133>/X0'
   */
  if (localDW->icLoad_e != 0) {
    localDW->MemoryX_DSTATE_a[0] = localP->X0_Value_d[0];
    localDW->MemoryX_DSTATE_a[1] = localP->X0_Value_d[1];
  }

  rtb_Add1[0] = localDW->MemoryX_DSTATE_a[0];
  rtb_Add1[1] = localDW->MemoryX_DSTATE_a[1];

  /* Outputs for Atomic SubSystem: '<S133>/UseCurrentEstimator' */

  /* Constant: '<S136>/KalmanGainM' incorporates:
   *  Constant: '<S133>/C'
   *  Constant: '<S133>/D'
   */
  DroneRS_Com_UseCurrentEstimator(rtb_LogicalOperator3,
    localP->KalmanGainM_Value, &rtb_vel_world[0], &localB->posVIS_datin[0],
    localP->C_Value_f, localP->D_Value_f0, rtb_Add1,
    &localB->UseCurrentEstimator_f);

  /* End of Outputs for SubSystem: '<S133>/UseCurrentEstimator' */

  /* DiscreteIntegrator: '<S71>/SimplyIntegrateVelocity' */
  if (localB->controlModePosVSAtt_flagin &&
      (localDW->SimplyIntegrateVelocity_PrevRes <= 0)) {
    localDW->SimplyIntegrateVelocity_DSTATE[0] =
      localP->SimplyIntegrateVelocity_IC;
    localDW->SimplyIntegrateVelocity_DSTATE[1] =
      localP->SimplyIntegrateVelocity_IC;
  }

  if (localDW->SimplyIntegrateVelocity_DSTATE[0] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[0] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[0] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[0] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  if (localDW->SimplyIntegrateVelocity_DSTATE[1] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[1] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[1] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[1] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  /* Switch: '<S71>/UseIPPosSwitch' incorporates:
   *  DiscreteIntegrator: '<S71>/SimplyIntegrateVelocity'
   */
  if (localB->usePosVIS_flagin > localP->UseIPPosSwitch_Threshold) {
    localB->UseIPPosSwitch[0] = localB->UseCurrentEstimator_f.Add[0];
    localB->UseIPPosSwitch[1] = localB->UseCurrentEstimator_f.Add[1];
  } else {
    localB->UseIPPosSwitch[0] = localDW->SimplyIntegrateVelocity_DSTATE[0];
    localB->UseIPPosSwitch[1] = localDW->SimplyIntegrateVelocity_DSTATE[1];
  }

  /* End of Switch: '<S71>/UseIPPosSwitch' */

  /* Outputs for Atomic SubSystem: '<S1>/ControllerLQR1' */
  DroneRS_Compensa_ControllerLQR1(rtu_pos_refin, rtu_attRS_refin,
    localB->controlModePosVSAtt_flagin, localB->UseIPPosSwitch,
    localB->Reshapexhat_o, localB->Reshapexhat[0], localB->att_estimout,
    localB->acc_RS[2], localB->datt_estimout, &localB->ControllerLQR1,
    (P_ControllerLQR1_DroneRS_Comp_T *)&localP->ControllerLQR1,
    DroneRS_Compensator_P);

  /* End of Outputs for SubSystem: '<S1>/ControllerLQR1' */

  /* Bias: '<S12>/Bias' */
  rtb_Ckxhatkk1_b = localB->Reshapexhat[0] +
    DroneRS_Compensator_P->altEstim.outlierJump_UpperLimit;

  /* Bias: '<S12>/Bias1' */
  rtb_Dk1uk1 = localB->Reshapexhat[0] +
    -DroneRS_Compensator_P->altEstim.outlierJump_UpperLimit;

  /* Product: '<S39>/C[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S16>/C'
   *  Delay: '<S16>/MemoryX'
   */
  rtb_Ckxhatkk1_b = localP->C_Value[0] * localDW->MemoryX_DSTATE[0] +
    localP->C_Value[1] * localDW->MemoryX_DSTATE[1];

  /* Product: '<S39>/D[k-1]*u[k-1]' incorporates:
   *  Constant: '<S16>/D'
   */
  rtb_Dk1uk1 = localP->D_Value * rtb_Sum[2];

  /* Outputs for Enabled SubSystem: '<S39>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S64>/Enable'
   */
  if (rtb_Compare_ok) {
    /* Sum: '<S64>/Sum' incorporates:
     *  Sum: '<S39>/Add1'
     */
    rtb_r = rtb_invertzaxisGain - (rtb_Ckxhatkk1_b + rtb_Dk1uk1);

    /* Product: '<S64>/Product3' incorporates:
     *  Constant: '<S21>/KalmanGainL'
     */
    localB->Product3[0] = localP->KalmanGainL_Value[0] * rtb_r;
    localB->Product3[1] = localP->KalmanGainL_Value[1] * rtb_r;
  }

  /* End of Outputs for SubSystem: '<S39>/MeasurementUpdate' */

  /* Sum: '<S94>/Add1' incorporates:
   *  Constant: '<S73>/C'
   *  Constant: '<S73>/D'
   *  Delay: '<S73>/MemoryX'
   *  Product: '<S94>/C[k]*xhat[k|k-1]'
   *  Product: '<S94>/D[k-1]*u[k-1]'
   */
  rtb_Add1[0] = (localP->C_Value_d[0] * localDW->MemoryX_DSTATE_f[0] +
                 localP->C_Value_d[2] * localDW->MemoryX_DSTATE_f[1]) +
    (localP->D_Value_f[0] * rtb_Dk1uk1_m[0] + localP->D_Value_f[2] *
     rtb_Dk1uk1_m[1]);
  rtb_Add1[1] = (localP->C_Value_d[1] * localDW->MemoryX_DSTATE_f[0] +
                 localP->C_Value_d[3] * localDW->MemoryX_DSTATE_f[1]) +
    (localP->D_Value_f[1] * rtb_Dk1uk1_m[0] + localP->D_Value_f[3] *
     rtb_Dk1uk1_m[1]);

  /* Outputs for Enabled SubSystem: '<S94>/MeasurementUpdate' */

  /* Constant: '<S76>/KalmanGainL' */
  DroneRS_Compe_MeasurementUpdate(rtb_LogicalOperator3_b,
    localP->KalmanGainL_Value_f, &rtb_opticalFlowToVelocity_Gain[0], rtb_Add1,
    &localB->MeasurementUpdate_p);

  /* End of Outputs for SubSystem: '<S94>/MeasurementUpdate' */

  /* Product: '<S94>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S73>/A'
   *  Delay: '<S73>/MemoryX'
   *  Sum: '<S94>/Add'
   */
  g = localP->A_Value_d[1] * localDW->MemoryX_DSTATE_f[0] + localP->A_Value_d[3]
    * localDW->MemoryX_DSTATE_f[1];

  /* Update for Delay: '<S73>/MemoryX' incorporates:
   *  Constant: '<S73>/A'
   *  Constant: '<S73>/B'
   *  Delay: '<S73>/MemoryX'
   *  Product: '<S94>/A[k]*xhat[k|k-1]'
   *  Product: '<S94>/B[k]*u[k]'
   *  Sum: '<S94>/Add'
   */
  localDW->MemoryX_DSTATE_f[0] = ((localP->B_Value_c[0] * rtb_Dk1uk1_m[0] +
    localP->B_Value_c[2] * rtb_Dk1uk1_m[1]) + (localP->A_Value_d[0] *
    localDW->MemoryX_DSTATE_f[0] + localP->A_Value_d[2] *
    localDW->MemoryX_DSTATE_f[1])) + localB->MeasurementUpdate_p.Product3[0];
  localDW->MemoryX_DSTATE_f[1] = ((localP->B_Value_c[1] * rtb_Dk1uk1_m[0] +
    localP->B_Value_c[3] * rtb_Dk1uk1_m[1]) + g) +
    localB->MeasurementUpdate_p.Product3[1];

  /* Sum: '<S154>/Add1' incorporates:
   *  Constant: '<S133>/C'
   *  Constant: '<S133>/D'
   *  Delay: '<S133>/MemoryX'
   *  Product: '<S154>/C[k]*xhat[k|k-1]'
   *  Product: '<S154>/D[k-1]*u[k-1]'
   */
  rtb_Add1[0] = (localP->C_Value_f[0] * localDW->MemoryX_DSTATE_a[0] +
                 localP->C_Value_f[2] * localDW->MemoryX_DSTATE_a[1]) +
    (localP->D_Value_f0[0] * rtb_vel_world[0] + localP->D_Value_f0[2] *
     rtb_vel_world[1]);
  rtb_Add1[1] = (localP->C_Value_f[1] * localDW->MemoryX_DSTATE_a[0] +
                 localP->C_Value_f[3] * localDW->MemoryX_DSTATE_a[1]) +
    (localP->D_Value_f0[1] * rtb_vel_world[0] + localP->D_Value_f0[3] *
     rtb_vel_world[1]);

  /* Outputs for Enabled SubSystem: '<S154>/MeasurementUpdate' */

  /* Constant: '<S136>/KalmanGainL' */
  DroneRS_Compe_MeasurementUpdate(rtb_LogicalOperator3,
    localP->KalmanGainL_Value_j, &localB->posVIS_datin[0], rtb_Add1,
    &localB->MeasurementUpdate_f);

  /* End of Outputs for SubSystem: '<S154>/MeasurementUpdate' */

  /* Inport: '<S1>/batteryStatus_datin' */
  localB->batteryStatus_datin[0] = rtu_batteryStatus_datin[0];
  localB->batteryStatus_datin[1] = rtu_batteryStatus_datin[1];

  /* Update for DiscreteFir: '<S15>/FIRaccelero' */
  /* Update circular buffer index */
  localDW->FIRaccelero_circBuf--;
  if (localDW->FIRaccelero_circBuf < 0) {
    localDW->FIRaccelero_circBuf = 4;
  }

  /* Update circular buffer */
  localDW->FIRaccelero_states[localDW->FIRaccelero_circBuf] = inversesIMU_Gain[0];
  localDW->FIRaccelero_states[localDW->FIRaccelero_circBuf + 5] =
    inversesIMU_Gain[1];
  localDW->FIRaccelero_states[localDW->FIRaccelero_circBuf + 10] =
    inversesIMU_Gain[2];

  /* End of Update for DiscreteFir: '<S15>/FIRaccelero' */

  /* Update for DiscreteFilter: '<S15>/IIRgyroz' */
  localDW->IIRgyroz_states[4] = localDW->IIRgyroz_states[3];
  localDW->IIRgyroz_states[3] = localDW->IIRgyroz_states[2];
  localDW->IIRgyroz_states[2] = localDW->IIRgyroz_states[1];
  localDW->IIRgyroz_states[1] = localDW->IIRgyroz_states[0];
  localDW->IIRgyroz_states[0] = IIRgyroz_tmp;

  /* Update for Delay: '<S71>/Delay' */
  localDW->Delay_DSTATE[0] = localB->UseCurrentEstimator_f.Add[0];
  localDW->Delay_DSTATE[1] = localB->UseCurrentEstimator_f.Add[1];

  /* Update for DiscreteFilter: '<S74>/IIRgyroz' */
  for (k = 0; k < 2; k++) {
    memOffset = k * 5;
    localDW->IIRgyroz_states_n[memOffset + 4] = localDW->
      IIRgyroz_states_n[memOffset + 3];
    localDW->IIRgyroz_states_n[memOffset + 3] = localDW->
      IIRgyroz_states_n[memOffset + 2];
    localDW->IIRgyroz_states_n[memOffset + 2] = localDW->
      IIRgyroz_states_n[memOffset + 1];
    localDW->IIRgyroz_states_n[memOffset + 1] = localDW->
      IIRgyroz_states_n[memOffset];
    localDW->IIRgyroz_states_n[memOffset] = localDW->IIRgyroz_tmp_f[k];
  }

  /* End of Update for DiscreteFilter: '<S74>/IIRgyroz' */

  /* Update for UnitDelay: '<S121>/UD' */
  localDW->UD_DSTATE[0] = rtb_TSamp_idx_0;
  localDW->UD_DSTATE[1] = rtb_TSamp_idx_1;

  /* Update for Delay: '<S12>/Delay2' */
  localDW->Delay2_DSTATE = localB->Reshapexhat[0];

  /* Update for DiscreteFilter: '<S17>/IIRprs' */
  localDW->IIRprs_states[4] = localDW->IIRprs_states[3];
  localDW->IIRprs_states[3] = localDW->IIRprs_states[2];
  localDW->IIRprs_states[2] = localDW->IIRprs_states[1];
  localDW->IIRprs_states[1] = localDW->IIRprs_states[0];
  localDW->IIRprs_states[0] = scale;

  /* Update for DiscreteFilter: '<S17>/IIRsonar' */
  localDW->IIRsonar_states[4] = localDW->IIRsonar_states[3];
  localDW->IIRsonar_states[3] = localDW->IIRsonar_states[2];
  localDW->IIRsonar_states[2] = localDW->IIRsonar_states[1];
  localDW->IIRsonar_states[1] = localDW->IIRsonar_states[0];
  localDW->IIRsonar_states[0] = IIRsonar_tmp;

  /* Update for Delay: '<S16>/MemoryX' */
  localDW->icLoad = 0U;

  /* Product: '<S39>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S16>/A'
   *  Delay: '<S16>/MemoryX'
   *  Sum: '<S39>/Add'
   */
  g = localP->A_Value[1] * localDW->MemoryX_DSTATE[0] + localP->A_Value[3] *
    localDW->MemoryX_DSTATE[1];

  /* Update for Delay: '<S16>/MemoryX' incorporates:
   *  Constant: '<S16>/A'
   *  Constant: '<S16>/B'
   *  Delay: '<S16>/MemoryX'
   *  Product: '<S39>/A[k]*xhat[k|k-1]'
   *  Product: '<S39>/B[k]*u[k]'
   *  Sum: '<S39>/Add'
   */
  localDW->MemoryX_DSTATE[0] = ((localP->A_Value[0] * localDW->MemoryX_DSTATE[0]
    + localP->A_Value[2] * localDW->MemoryX_DSTATE[1]) + localP->B_Value[0] *
    rtb_Sum[2]) + localB->Product3[0];
  localDW->MemoryX_DSTATE[1] = (localP->B_Value[1] * rtb_Sum[2] + g) +
    localB->Product3[1];

  /* Update for Delay: '<S70>/Delay' */
  localDW->Delay_DSTATE_l[0] = localB->Reshapexhat_o[0];
  localDW->Delay_DSTATE_l[1] = localB->Reshapexhat_o[1];

  /* Update for Delay: '<S73>/MemoryX' */
  localDW->icLoad_c = 0U;

  /* Update for Delay: '<S3>/Delay1' */
  localDW->Delay1_DSTATE[0] = localB->Reshapexhat_o[0];
  localDW->Delay1_DSTATE[1] = localB->Reshapexhat_o[1];

  /* Update for Delay: '<S133>/MemoryX' */
  localDW->icLoad_e = 0U;

  /* Product: '<S154>/A[k]*xhat[k|k-1]' incorporates:
   *  Constant: '<S133>/A'
   *  Delay: '<S133>/MemoryX'
   *  Sum: '<S154>/Add'
   */
  g = localP->A_Value_dj[1] * localDW->MemoryX_DSTATE_a[0] + localP->A_Value_dj
    [3] * localDW->MemoryX_DSTATE_a[1];

  /* Update for Delay: '<S133>/MemoryX' incorporates:
   *  Constant: '<S133>/A'
   *  Constant: '<S133>/B'
   *  Delay: '<S133>/MemoryX'
   *  Product: '<S154>/A[k]*xhat[k|k-1]'
   *  Product: '<S154>/B[k]*u[k]'
   *  Sum: '<S154>/Add'
   */
  localDW->MemoryX_DSTATE_a[0] = ((localP->B_Value_k[0] * rtb_vel_world[0] +
    localP->B_Value_k[2] * rtb_vel_world[1]) + (localP->A_Value_dj[0] *
    localDW->MemoryX_DSTATE_a[0] + localP->A_Value_dj[2] *
    localDW->MemoryX_DSTATE_a[1])) + localB->MeasurementUpdate_f.Product3[0];
  localDW->MemoryX_DSTATE_a[1] = ((localP->B_Value_k[1] * rtb_vel_world[0] +
    localP->B_Value_k[3] * rtb_vel_world[1]) + g) +
    localB->MeasurementUpdate_f.Product3[1];

  /* Update for DiscreteIntegrator: '<S71>/SimplyIntegrateVelocity' */
  localDW->SimplyIntegrateVelocity_DSTATE[0] +=
    localP->SimplyIntegrateVelocity_gainval * rtb_vel_world[0];
  localDW->SimplyIntegrateVelocity_DSTATE[1] +=
    localP->SimplyIntegrateVelocity_gainval * rtb_vel_world[1];
  if (localDW->SimplyIntegrateVelocity_DSTATE[0] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[0] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[0] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[0] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  if (localDW->SimplyIntegrateVelocity_DSTATE[1] >=
      localP->SimplyIntegrateVelocity_UpperSa) {
    localDW->SimplyIntegrateVelocity_DSTATE[1] =
      localP->SimplyIntegrateVelocity_UpperSa;
  } else {
    if (localDW->SimplyIntegrateVelocity_DSTATE[1] <=
        localP->SimplyIntegrateVelocity_LowerSa) {
      localDW->SimplyIntegrateVelocity_DSTATE[1] =
        localP->SimplyIntegrateVelocity_LowerSa;
    }
  }

  localDW->SimplyIntegrateVelocity_PrevRes = (int8_T)
    localB->controlModePosVSAtt_flagin;

  /* End of Update for DiscreteIntegrator: '<S71>/SimplyIntegrateVelocity' */
}

/* Model step function */
void DroneRS_Compensator_step(RT_MODEL_DroneRS_Compensator_T *const
  DroneRS_Compensator_M, boolean_T
  DroneRS_Compensator_U_controlModePosVSAtt_flagin, real_T
  DroneRS_Compensator_U_pos_refin[3], real_T DroneRS_Compensator_U_attRS_refin[3],
  real_T DroneRS_Compensator_U_ddx, real_T DroneRS_Compensator_U_ddy, real_T
  DroneRS_Compensator_U_ddz, real_T DroneRS_Compensator_U_p, real_T
  DroneRS_Compensator_U_q, real_T DroneRS_Compensator_U_r, real_T
  DroneRS_Compensator_U_altitude_sonar, real_T DroneRS_Compensator_U_prs, real_T
  DroneRS_Compensator_U_opticalFlowRS_datin[3], real_T
  DroneRS_Compensator_U_sensordatabiasRS_datin[7], real_T
  DroneRS_Compensator_U_posVIS_datin[4], real_T
  DroneRS_Compensator_U_usePosVIS_flagin, real_T
  DroneRS_Compensator_U_batteryStatus_datin[2], real_T
  DroneRS_Compensator_Y_motorsRS_cmdout[4], real_T *DroneRS_Compensator_Y_X,
  real_T *DroneRS_Compensator_Y_Y, real_T *DroneRS_Compensator_Y_Z, real_T
  *DroneRS_Compensator_Y_yaw, real_T *DroneRS_Compensator_Y_pitch, real_T
  *DroneRS_Compensator_Y_roll, real_T *DroneRS_Compensator_Y_dx, real_T
  *DroneRS_Compensator_Y_dy, real_T *DroneRS_Compensator_Y_dz, real_T
  *DroneRS_Compensator_Y_pb, real_T *DroneRS_Compensator_Y_qb, real_T
  *DroneRS_Compensator_Y_rb, boolean_T
  *DroneRS_Compensator_Y_controlModePosVSAtt_flagout, real_T
  DroneRS_Compensator_Y_poseRS_refout[6], real_T *DroneRS_Compensator_Y_ddxb,
  real_T *DroneRS_Compensator_Y_ddyb, real_T *DroneRS_Compensator_Y_ddzb, real_T
  *DroneRS_Compensator_Y_pa, real_T *DroneRS_Compensator_Y_qa, real_T
  *DroneRS_Compensator_Y_ra, real_T *DroneRS_Compensator_Y_altitude_sonarb,
  real_T *DroneRS_Compensator_Y_prsb, real_T
  DroneRS_Compensator_Y_opticalFlowRS_datout[3], real_T
  DroneRS_Compensator_Y_sensordatabiasRS_datout[7], real_T
  DroneRS_Compensator_Y_posVIS_datout[4], real_T
  *DroneRS_Compensator_Y_usePosVIS_flagout, real_T
  DroneRS_Compensator_Y_batteryStatus_datout[2])
{
  P_DroneRS_Compensator_T *DroneRS_Compensator_P = ((P_DroneRS_Compensator_T *)
    DroneRS_Compensator_M->ModelData.defaultParam);
  B_DroneRS_Compensator_T *DroneRS_Compensator_B = ((B_DroneRS_Compensator_T *)
    DroneRS_Compensator_M->ModelData.blockIO);
  DW_DroneRS_Compensator_T *DroneRS_Compensator_DW = ((DW_DroneRS_Compensator_T *)
    DroneRS_Compensator_M->ModelData.dwork);
  int32_T i;

  /* Outputs for Atomic SubSystem: '<Root>/DroneRS_Compensator' */

  /* Inport: '<Root>/controlModePosVSAtt_flagin' incorporates:
   *  Inport: '<Root>/altitude_sonar'
   *  Inport: '<Root>/attRS_refin'
   *  Inport: '<Root>/batteryStatus_datin'
   *  Inport: '<Root>/ddx'
   *  Inport: '<Root>/ddy'
   *  Inport: '<Root>/ddz'
   *  Inport: '<Root>/opticalFlowRS_datin'
   *  Inport: '<Root>/p'
   *  Inport: '<Root>/posVIS_datin'
   *  Inport: '<Root>/pos_refin'
   *  Inport: '<Root>/prs'
   *  Inport: '<Root>/q'
   *  Inport: '<Root>/r'
   *  Inport: '<Root>/sensordatabiasRS_datin'
   *  Inport: '<Root>/usePosVIS_flagin'
   */
  DroneRS_Com_DroneRS_Compensator
    (DroneRS_Compensator_U_controlModePosVSAtt_flagin,
     DroneRS_Compensator_U_pos_refin, DroneRS_Compensator_U_attRS_refin,
     DroneRS_Compensator_U_ddx, DroneRS_Compensator_U_ddy,
     DroneRS_Compensator_U_ddz, DroneRS_Compensator_U_p, DroneRS_Compensator_U_q,
     DroneRS_Compensator_U_r, DroneRS_Compensator_U_altitude_sonar,
     DroneRS_Compensator_U_prs, DroneRS_Compensator_U_opticalFlowRS_datin,
     DroneRS_Compensator_U_sensordatabiasRS_datin,
     DroneRS_Compensator_U_posVIS_datin, DroneRS_Compensator_U_usePosVIS_flagin,
     DroneRS_Compensator_U_batteryStatus_datin,
     &DroneRS_Compensator_B->DroneRS_Compensator_d,
     &DroneRS_Compensator_DW->DroneRS_Compensator_d,
     (P_DroneRS_Compensator_DroneRS_T *)
     &DroneRS_Compensator_P->DroneRS_Compensator_d, DroneRS_Compensator_P);

  /* End of Outputs for SubSystem: '<Root>/DroneRS_Compensator' */

  /* Outport: '<Root>/motorsRS_cmdout' */
  DroneRS_Compensator_Y_motorsRS_cmdout[0] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.ControllerLQR1.W2ToMotorsCmd_Gain
    [0];
  DroneRS_Compensator_Y_motorsRS_cmdout[1] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.ControllerLQR1.W2ToMotorsCmd_Gain
    [1];
  DroneRS_Compensator_Y_motorsRS_cmdout[2] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.ControllerLQR1.W2ToMotorsCmd_Gain
    [2];
  DroneRS_Compensator_Y_motorsRS_cmdout[3] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.ControllerLQR1.W2ToMotorsCmd_Gain
    [3];

  /* Outport: '<Root>/X' */
  *DroneRS_Compensator_Y_X =
    DroneRS_Compensator_B->DroneRS_Compensator_d.UseIPPosSwitch[0];

  /* Outport: '<Root>/Y' */
  *DroneRS_Compensator_Y_Y =
    DroneRS_Compensator_B->DroneRS_Compensator_d.UseIPPosSwitch[1];

  /* Outport: '<Root>/Z' */
  *DroneRS_Compensator_Y_Z =
    DroneRS_Compensator_B->DroneRS_Compensator_d.Reshapexhat[0];

  /* Outport: '<Root>/yaw' */
  *DroneRS_Compensator_Y_yaw =
    DroneRS_Compensator_B->DroneRS_Compensator_d.att_estimout[0];

  /* Outport: '<Root>/pitch' */
  *DroneRS_Compensator_Y_pitch =
    DroneRS_Compensator_B->DroneRS_Compensator_d.att_estimout[1];

  /* Outport: '<Root>/roll' */
  *DroneRS_Compensator_Y_roll =
    DroneRS_Compensator_B->DroneRS_Compensator_d.att_estimout[2];

  /* Outport: '<Root>/dx' */
  *DroneRS_Compensator_Y_dx =
    DroneRS_Compensator_B->DroneRS_Compensator_d.Reshapexhat_o[0];

  /* Outport: '<Root>/dy' */
  *DroneRS_Compensator_Y_dy =
    DroneRS_Compensator_B->DroneRS_Compensator_d.Reshapexhat_o[1];

  /* Outport: '<Root>/dz' */
  *DroneRS_Compensator_Y_dz =
    DroneRS_Compensator_B->DroneRS_Compensator_d.acc_RS[2];

  /* Outport: '<Root>/pb' */
  *DroneRS_Compensator_Y_pb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.datt_estimout[0];

  /* Outport: '<Root>/qb' */
  *DroneRS_Compensator_Y_qb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.datt_estimout[1];

  /* Outport: '<Root>/rb' */
  *DroneRS_Compensator_Y_rb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.datt_estimout[2];

  /* Outport: '<Root>/controlModePosVSAtt_flagout' */
  *DroneRS_Compensator_Y_controlModePosVSAtt_flagout =
    DroneRS_Compensator_B->DroneRS_Compensator_d.controlModePosVSAtt_flagin;

  /* Outport: '<Root>/poseRS_refout' */
  for (i = 0; i < 6; i++) {
    DroneRS_Compensator_Y_poseRS_refout[i] =
      DroneRS_Compensator_B->DroneRS_Compensator_d.ControllerLQR1.PosVSAtt_Switch
      [i];
  }

  /* End of Outport: '<Root>/poseRS_refout' */

  /* Outport: '<Root>/ddxb' */
  *DroneRS_Compensator_Y_ddxb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[0];

  /* Outport: '<Root>/ddyb' */
  *DroneRS_Compensator_Y_ddyb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[1];

  /* Outport: '<Root>/ddzb' */
  *DroneRS_Compensator_Y_ddzb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[2];

  /* Outport: '<Root>/pa' */
  *DroneRS_Compensator_Y_pa =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[3];

  /* Outport: '<Root>/qa' */
  *DroneRS_Compensator_Y_qa =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[4];

  /* Outport: '<Root>/ra' */
  *DroneRS_Compensator_Y_ra =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[5];

  /* Outport: '<Root>/altitude_sonarb' */
  *DroneRS_Compensator_Y_altitude_sonarb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[6];

  /* Outport: '<Root>/prsb' */
  *DroneRS_Compensator_Y_prsb =
    DroneRS_Compensator_B->DroneRS_Compensator_d.sensordataRS_datin[7];

  /* Outport: '<Root>/opticalFlowRS_datout' */
  DroneRS_Compensator_Y_opticalFlowRS_datout[0] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.opticalFlowRS_datin[0];
  DroneRS_Compensator_Y_opticalFlowRS_datout[1] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.opticalFlowRS_datin[1];
  DroneRS_Compensator_Y_opticalFlowRS_datout[2] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.opticalFlowRS_datin[2];

  /* Outport: '<Root>/sensordatabiasRS_datout' */
  for (i = 0; i < 7; i++) {
    DroneRS_Compensator_Y_sensordatabiasRS_datout[i] =
      DroneRS_Compensator_B->DroneRS_Compensator_d.sensordatabiasRS_datin[i];
  }

  /* End of Outport: '<Root>/sensordatabiasRS_datout' */

  /* Outport: '<Root>/posVIS_datout' */
  DroneRS_Compensator_Y_posVIS_datout[0] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.posVIS_datin[0];
  DroneRS_Compensator_Y_posVIS_datout[1] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.posVIS_datin[1];
  DroneRS_Compensator_Y_posVIS_datout[2] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.posVIS_datin[2];
  DroneRS_Compensator_Y_posVIS_datout[3] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.posVIS_datin[3];

  /* Outport: '<Root>/usePosVIS_flagout' */
  *DroneRS_Compensator_Y_usePosVIS_flagout =
    DroneRS_Compensator_B->DroneRS_Compensator_d.usePosVIS_flagin;

  /* Outport: '<Root>/batteryStatus_datout' */
  DroneRS_Compensator_Y_batteryStatus_datout[0] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.batteryStatus_datin[0];
  DroneRS_Compensator_Y_batteryStatus_datout[1] =
    DroneRS_Compensator_B->DroneRS_Compensator_d.batteryStatus_datin[1];

  /* Matfile logging */
  rt_UpdateTXYLogVars(DroneRS_Compensator_M->rtwLogInfo,
                      (&DroneRS_Compensator_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.005s, 0.0s] */
    if ((rtmGetTFinal(DroneRS_Compensator_M)!=-1) &&
        !((rtmGetTFinal(DroneRS_Compensator_M)-
           DroneRS_Compensator_M->Timing.taskTime0) >
          DroneRS_Compensator_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(DroneRS_Compensator_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  DroneRS_Compensator_M->Timing.taskTime0 =
    (++DroneRS_Compensator_M->Timing.clockTick0) *
    DroneRS_Compensator_M->Timing.stepSize0;
}

/* Model initialize function */
void DroneRS_Compensator_initialize(RT_MODEL_DroneRS_Compensator_T *const
  DroneRS_Compensator_M, boolean_T
  *DroneRS_Compensator_U_controlModePosVSAtt_flagin, real_T
  DroneRS_Compensator_U_pos_refin[3], real_T DroneRS_Compensator_U_attRS_refin[3],
  real_T *DroneRS_Compensator_U_ddx, real_T *DroneRS_Compensator_U_ddy, real_T
  *DroneRS_Compensator_U_ddz, real_T *DroneRS_Compensator_U_p, real_T
  *DroneRS_Compensator_U_q, real_T *DroneRS_Compensator_U_r, real_T
  *DroneRS_Compensator_U_altitude_sonar, real_T *DroneRS_Compensator_U_prs,
  real_T DroneRS_Compensator_U_opticalFlowRS_datin[3], real_T
  DroneRS_Compensator_U_sensordatabiasRS_datin[7], real_T
  DroneRS_Compensator_U_posVIS_datin[4], real_T
  *DroneRS_Compensator_U_usePosVIS_flagin, real_T
  DroneRS_Compensator_U_batteryStatus_datin[2], real_T
  DroneRS_Compensator_Y_motorsRS_cmdout[4], real_T *DroneRS_Compensator_Y_X,
  real_T *DroneRS_Compensator_Y_Y, real_T *DroneRS_Compensator_Y_Z, real_T
  *DroneRS_Compensator_Y_yaw, real_T *DroneRS_Compensator_Y_pitch, real_T
  *DroneRS_Compensator_Y_roll, real_T *DroneRS_Compensator_Y_dx, real_T
  *DroneRS_Compensator_Y_dy, real_T *DroneRS_Compensator_Y_dz, real_T
  *DroneRS_Compensator_Y_pb, real_T *DroneRS_Compensator_Y_qb, real_T
  *DroneRS_Compensator_Y_rb, boolean_T
  *DroneRS_Compensator_Y_controlModePosVSAtt_flagout, real_T
  DroneRS_Compensator_Y_poseRS_refout[6], real_T *DroneRS_Compensator_Y_ddxb,
  real_T *DroneRS_Compensator_Y_ddyb, real_T *DroneRS_Compensator_Y_ddzb, real_T
  *DroneRS_Compensator_Y_pa, real_T *DroneRS_Compensator_Y_qa, real_T
  *DroneRS_Compensator_Y_ra, real_T *DroneRS_Compensator_Y_altitude_sonarb,
  real_T *DroneRS_Compensator_Y_prsb, real_T
  DroneRS_Compensator_Y_opticalFlowRS_datout[3], real_T
  DroneRS_Compensator_Y_sensordatabiasRS_datout[7], real_T
  DroneRS_Compensator_Y_posVIS_datout[4], real_T
  *DroneRS_Compensator_Y_usePosVIS_flagout, real_T
  DroneRS_Compensator_Y_batteryStatus_datout[2])
{
  P_DroneRS_Compensator_T *DroneRS_Compensator_P = ((P_DroneRS_Compensator_T *)
    DroneRS_Compensator_M->ModelData.defaultParam);
  DW_DroneRS_Compensator_T *DroneRS_Compensator_DW = ((DW_DroneRS_Compensator_T *)
    DroneRS_Compensator_M->ModelData.dwork);
  B_DroneRS_Compensator_T *DroneRS_Compensator_B = ((B_DroneRS_Compensator_T *)
    DroneRS_Compensator_M->ModelData.blockIO);

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  DroneRS_Compensator_P->DroneRS_Compensator_d.SaturationSonar_LowerSat =
    rtMinusInf;
  rtmSetTFinal(DroneRS_Compensator_M, 10.0);
  DroneRS_Compensator_M->Timing.stepSize0 = 0.005;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    DroneRS_Compensator_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(DroneRS_Compensator_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(DroneRS_Compensator_M->rtwLogInfo, (NULL));
    rtliSetLogT(DroneRS_Compensator_M->rtwLogInfo, "tout");
    rtliSetLogX(DroneRS_Compensator_M->rtwLogInfo, "");
    rtliSetLogXFinal(DroneRS_Compensator_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(DroneRS_Compensator_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(DroneRS_Compensator_M->rtwLogInfo, 2);
    rtliSetLogMaxRows(DroneRS_Compensator_M->rtwLogInfo, 0);
    rtliSetLogDecimation(DroneRS_Compensator_M->rtwLogInfo, 1);

    /*
     * Set pointers to the data and signal info for each output
     */
    {
      static void * rt_LoggedOutputSignalPtrs[28];
      rt_LoggedOutputSignalPtrs[0] = &DroneRS_Compensator_Y_motorsRS_cmdout[0];
      rt_LoggedOutputSignalPtrs[1] = &(*DroneRS_Compensator_Y_X);
      rt_LoggedOutputSignalPtrs[2] = &(*DroneRS_Compensator_Y_Y);
      rt_LoggedOutputSignalPtrs[3] = &(*DroneRS_Compensator_Y_Z);
      rt_LoggedOutputSignalPtrs[4] = &(*DroneRS_Compensator_Y_yaw);
      rt_LoggedOutputSignalPtrs[5] = &(*DroneRS_Compensator_Y_pitch);
      rt_LoggedOutputSignalPtrs[6] = &(*DroneRS_Compensator_Y_roll);
      rt_LoggedOutputSignalPtrs[7] = &(*DroneRS_Compensator_Y_dx);
      rt_LoggedOutputSignalPtrs[8] = &(*DroneRS_Compensator_Y_dy);
      rt_LoggedOutputSignalPtrs[9] = &(*DroneRS_Compensator_Y_dz);
      rt_LoggedOutputSignalPtrs[10] = &(*DroneRS_Compensator_Y_pb);
      rt_LoggedOutputSignalPtrs[11] = &(*DroneRS_Compensator_Y_qb);
      rt_LoggedOutputSignalPtrs[12] = &(*DroneRS_Compensator_Y_rb);
      rt_LoggedOutputSignalPtrs[13] =
        &(*DroneRS_Compensator_Y_controlModePosVSAtt_flagout);
      rt_LoggedOutputSignalPtrs[14] = &DroneRS_Compensator_Y_poseRS_refout[0];
      rt_LoggedOutputSignalPtrs[15] = &(*DroneRS_Compensator_Y_ddxb);
      rt_LoggedOutputSignalPtrs[16] = &(*DroneRS_Compensator_Y_ddyb);
      rt_LoggedOutputSignalPtrs[17] = &(*DroneRS_Compensator_Y_ddzb);
      rt_LoggedOutputSignalPtrs[18] = &(*DroneRS_Compensator_Y_pa);
      rt_LoggedOutputSignalPtrs[19] = &(*DroneRS_Compensator_Y_qa);
      rt_LoggedOutputSignalPtrs[20] = &(*DroneRS_Compensator_Y_ra);
      rt_LoggedOutputSignalPtrs[21] = &(*DroneRS_Compensator_Y_altitude_sonarb);
      rt_LoggedOutputSignalPtrs[22] = &(*DroneRS_Compensator_Y_prsb);
      rt_LoggedOutputSignalPtrs[23] =
        &DroneRS_Compensator_Y_opticalFlowRS_datout[0];
      rt_LoggedOutputSignalPtrs[24] =
        &DroneRS_Compensator_Y_sensordatabiasRS_datout[0];
      rt_LoggedOutputSignalPtrs[25] = &DroneRS_Compensator_Y_posVIS_datout[0];
      rt_LoggedOutputSignalPtrs[26] = &(*DroneRS_Compensator_Y_usePosVIS_flagout);
      rt_LoggedOutputSignalPtrs[27] =
        &DroneRS_Compensator_Y_batteryStatus_datout[0];
      rtliSetLogYSignalPtrs(DroneRS_Compensator_M->rtwLogInfo,
                            ((LogSignalPtrsType)rt_LoggedOutputSignalPtrs));
    }

    {
      static int_T rt_LoggedOutputWidths[] = {
        4,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        6,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        3,
        7,
        4,
        1,
        2
      };

      static int_T rt_LoggedOutputNumDimensions[] = {
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1
      };

      static int_T rt_LoggedOutputDimensions[] = {
        4,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        6,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        3,
        7,
        4,
        1,
        2
      };

      static boolean_T rt_LoggedOutputIsVarDims[] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      };

      static void* rt_LoggedCurrentSignalDimensions[] = {
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL),
        (NULL)
      };

      static int_T rt_LoggedCurrentSignalDimensionsSize[] = {
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4,
        4
      };

      static BuiltInDTypeId rt_LoggedOutputDataTypeIds[] = {
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_BOOLEAN,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE,
        SS_DOUBLE
      };

      static int_T rt_LoggedOutputComplexSignals[] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      };

      static const char_T *rt_LoggedOutputLabels[] = {
        "",
        "<X>",
        "<Y>",
        "<Z>",
        "<yaw>",
        "<pitch>",
        "<roll>",
        "<dx>",
        "<dy>",
        "<dz>",
        "<p>",
        "<q>",
        "<r>",
        "",
        "",
        "<ddx>",
        "<ddy>",
        "<ddz>",
        "<p>",
        "<q>",
        "<r>",
        "<altitude_sonar>",
        "<prs>",
        "",
        "",
        "",
        "",
        "" };

      static const char_T *rt_LoggedOutputBlockNames[] = {
        "DroneRS_Compensator/motorsRS_cmdout",
        "DroneRS_Compensator/X",
        "DroneRS_Compensator/Y",
        "DroneRS_Compensator/Z",
        "DroneRS_Compensator/yaw",
        "DroneRS_Compensator/pitch",
        "DroneRS_Compensator/roll",
        "DroneRS_Compensator/dx",
        "DroneRS_Compensator/dy",
        "DroneRS_Compensator/dz",
        "DroneRS_Compensator/pb",
        "DroneRS_Compensator/qb",
        "DroneRS_Compensator/rb",
        "DroneRS_Compensator/controlModePosVSAtt_flagout",
        "DroneRS_Compensator/poseRS_refout",
        "DroneRS_Compensator/ddxb",
        "DroneRS_Compensator/ddyb",
        "DroneRS_Compensator/ddzb",
        "DroneRS_Compensator/pa",
        "DroneRS_Compensator/qa",
        "DroneRS_Compensator/ra",
        "DroneRS_Compensator/altitude_sonarb",
        "DroneRS_Compensator/prsb",
        "DroneRS_Compensator/opticalFlowRS_datout",
        "DroneRS_Compensator/sensordatabiasRS_datout",
        "DroneRS_Compensator/posVIS_datout",
        "DroneRS_Compensator/usePosVIS_flagout",
        "DroneRS_Compensator/batteryStatus_datout" };

      static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert[] = {
        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_BOOLEAN, SS_BOOLEAN, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 }
      };

      static RTWLogSignalInfo rt_LoggedOutputSignalInfo[] = {
        {
          28,
          rt_LoggedOutputWidths,
          rt_LoggedOutputNumDimensions,
          rt_LoggedOutputDimensions,
          rt_LoggedOutputIsVarDims,
          rt_LoggedCurrentSignalDimensions,
          rt_LoggedCurrentSignalDimensionsSize,
          rt_LoggedOutputDataTypeIds,
          rt_LoggedOutputComplexSignals,
          (NULL),

          { rt_LoggedOutputLabels },
          (NULL),
          (NULL),
          (NULL),

          { rt_LoggedOutputBlockNames },

          { (NULL) },
          (NULL),
          rt_RTWLogDataTypeConvert
        }
      };

      rtliSetLogYSignalInfo(DroneRS_Compensator_M->rtwLogInfo,
                            rt_LoggedOutputSignalInfo);

      /* set currSigDims field */
      rt_LoggedCurrentSignalDimensions[0] = &rt_LoggedOutputWidths[0];
      rt_LoggedCurrentSignalDimensions[1] = &rt_LoggedOutputWidths[1];
      rt_LoggedCurrentSignalDimensions[2] = &rt_LoggedOutputWidths[2];
      rt_LoggedCurrentSignalDimensions[3] = &rt_LoggedOutputWidths[3];
      rt_LoggedCurrentSignalDimensions[4] = &rt_LoggedOutputWidths[4];
      rt_LoggedCurrentSignalDimensions[5] = &rt_LoggedOutputWidths[5];
      rt_LoggedCurrentSignalDimensions[6] = &rt_LoggedOutputWidths[6];
      rt_LoggedCurrentSignalDimensions[7] = &rt_LoggedOutputWidths[7];
      rt_LoggedCurrentSignalDimensions[8] = &rt_LoggedOutputWidths[8];
      rt_LoggedCurrentSignalDimensions[9] = &rt_LoggedOutputWidths[9];
      rt_LoggedCurrentSignalDimensions[10] = &rt_LoggedOutputWidths[10];
      rt_LoggedCurrentSignalDimensions[11] = &rt_LoggedOutputWidths[11];
      rt_LoggedCurrentSignalDimensions[12] = &rt_LoggedOutputWidths[12];
      rt_LoggedCurrentSignalDimensions[13] = &rt_LoggedOutputWidths[13];
      rt_LoggedCurrentSignalDimensions[14] = &rt_LoggedOutputWidths[14];
      rt_LoggedCurrentSignalDimensions[15] = &rt_LoggedOutputWidths[15];
      rt_LoggedCurrentSignalDimensions[16] = &rt_LoggedOutputWidths[16];
      rt_LoggedCurrentSignalDimensions[17] = &rt_LoggedOutputWidths[17];
      rt_LoggedCurrentSignalDimensions[18] = &rt_LoggedOutputWidths[18];
      rt_LoggedCurrentSignalDimensions[19] = &rt_LoggedOutputWidths[19];
      rt_LoggedCurrentSignalDimensions[20] = &rt_LoggedOutputWidths[20];
      rt_LoggedCurrentSignalDimensions[21] = &rt_LoggedOutputWidths[21];
      rt_LoggedCurrentSignalDimensions[22] = &rt_LoggedOutputWidths[22];
      rt_LoggedCurrentSignalDimensions[23] = &rt_LoggedOutputWidths[23];
      rt_LoggedCurrentSignalDimensions[24] = &rt_LoggedOutputWidths[24];
      rt_LoggedCurrentSignalDimensions[25] = &rt_LoggedOutputWidths[25];
      rt_LoggedCurrentSignalDimensions[26] = &rt_LoggedOutputWidths[26];
      rt_LoggedCurrentSignalDimensions[27] = &rt_LoggedOutputWidths[27];
    }

    rtliSetLogY(DroneRS_Compensator_M->rtwLogInfo, "yout");
  }

  /* block I/O */
  (void) memset(((void *) DroneRS_Compensator_B), 0,
                sizeof(B_DroneRS_Compensator_T));

  /* states (dwork) */
  (void) memset((void *)DroneRS_Compensator_DW, 0,
                sizeof(DW_DroneRS_Compensator_T));

  /* external inputs */
  (*DroneRS_Compensator_U_controlModePosVSAtt_flagin) = false;
  (void) memset(DroneRS_Compensator_U_pos_refin, 0,
                3U*sizeof(real_T));
  (void) memset(DroneRS_Compensator_U_attRS_refin, 0,
                3U*sizeof(real_T));
  (*DroneRS_Compensator_U_ddx) = 0.0;
  (*DroneRS_Compensator_U_ddy) = 0.0;
  (*DroneRS_Compensator_U_ddz) = 0.0;
  (*DroneRS_Compensator_U_p) = 0.0;
  (*DroneRS_Compensator_U_q) = 0.0;
  (*DroneRS_Compensator_U_r) = 0.0;
  (*DroneRS_Compensator_U_altitude_sonar) = 0.0;
  (*DroneRS_Compensator_U_prs) = 0.0;
  (void) memset(DroneRS_Compensator_U_opticalFlowRS_datin, 0,
                3U*sizeof(real_T));
  (void) memset(DroneRS_Compensator_U_sensordatabiasRS_datin, 0,
                7U*sizeof(real_T));
  (void) memset(DroneRS_Compensator_U_posVIS_datin, 0,
                4U*sizeof(real_T));
  (*DroneRS_Compensator_U_usePosVIS_flagin) = 0.0;
  (void) memset(DroneRS_Compensator_U_batteryStatus_datin, 0,
                2U*sizeof(real_T));

  /* external outputs */
  (void) memset(&DroneRS_Compensator_Y_motorsRS_cmdout[0], 0,
                4U*sizeof(real_T));
  (*DroneRS_Compensator_Y_X) = 0.0;
  (*DroneRS_Compensator_Y_Y) = 0.0;
  (*DroneRS_Compensator_Y_Z) = 0.0;
  (*DroneRS_Compensator_Y_yaw) = 0.0;
  (*DroneRS_Compensator_Y_pitch) = 0.0;
  (*DroneRS_Compensator_Y_roll) = 0.0;
  (*DroneRS_Compensator_Y_dx) = 0.0;
  (*DroneRS_Compensator_Y_dy) = 0.0;
  (*DroneRS_Compensator_Y_dz) = 0.0;
  (*DroneRS_Compensator_Y_pb) = 0.0;
  (*DroneRS_Compensator_Y_qb) = 0.0;
  (*DroneRS_Compensator_Y_rb) = 0.0;
  (*DroneRS_Compensator_Y_controlModePosVSAtt_flagout) = false;
  (void) memset(&DroneRS_Compensator_Y_poseRS_refout[0], 0,
                6U*sizeof(real_T));
  (*DroneRS_Compensator_Y_ddxb) = 0.0;
  (*DroneRS_Compensator_Y_ddyb) = 0.0;
  (*DroneRS_Compensator_Y_ddzb) = 0.0;
  (*DroneRS_Compensator_Y_pa) = 0.0;
  (*DroneRS_Compensator_Y_qa) = 0.0;
  (*DroneRS_Compensator_Y_ra) = 0.0;
  (*DroneRS_Compensator_Y_altitude_sonarb) = 0.0;
  (*DroneRS_Compensator_Y_prsb) = 0.0;
  (void) memset(&DroneRS_Compensator_Y_opticalFlowRS_datout[0], 0,
                3U*sizeof(real_T));
  (void) memset(&DroneRS_Compensator_Y_sensordatabiasRS_datout[0], 0,
                7U*sizeof(real_T));
  (void) memset(&DroneRS_Compensator_Y_posVIS_datout[0], 0,
                4U*sizeof(real_T));
  (*DroneRS_Compensator_Y_usePosVIS_flagout) = 0.0;
  (void) memset(&DroneRS_Compensator_Y_batteryStatus_datout[0], 0,
                2U*sizeof(real_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(DroneRS_Compensator_M->rtwLogInfo, 0.0,
    rtmGetTFinal(DroneRS_Compensator_M), DroneRS_Compensator_M->Timing.stepSize0,
    (&rtmGetErrorStatus(DroneRS_Compensator_M)));

  /* InitializeConditions for Atomic SubSystem: '<Root>/DroneRS_Compensator' */
  DroneR_DroneRS_Compensator_Init(&DroneRS_Compensator_DW->DroneRS_Compensator_d,
    (P_DroneRS_Compensator_DroneRS_T *)
    &DroneRS_Compensator_P->DroneRS_Compensator_d);

  /* End of InitializeConditions for SubSystem: '<Root>/DroneRS_Compensator' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
