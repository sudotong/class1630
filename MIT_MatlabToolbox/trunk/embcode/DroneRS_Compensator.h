/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DroneRS_Compensator.h
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

#ifndef RTW_HEADER_DroneRS_Compensator_h_
#define RTW_HEADER_DroneRS_Compensator_h_
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <string.h>
#ifndef DroneRS_Compensator_COMMON_INCLUDES_
# define DroneRS_Compensator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_logging.h"
#endif                                 /* DroneRS_Compensator_COMMON_INCLUDES_ */

#include "DroneRS_Compensator_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals for system '<S1>/ControllerLQR1' */
typedef struct {
  real_T PosVSAtt_Switch[12];          /* '<S6>/PosVSAtt_Switch' */
  real_T W2ToMotorsCmd_Gain[4];        /* '<S10>/W2ToMotorsCmd_Gain' */
} B_ControllerLQR1_DroneRS_Comp_T;

/* Block states (auto storage) for system '<S1>/ControllerLQR1' */
typedef struct {
  struct {
    void *LoggedData;
  } SCOPE_totalthrust_PWORK;           /* '<S9>/SCOPE_totalthrust ' */
} DW_ControllerLQR1_DroneRS_Com_T;

/* Block signals for system '<S94>/MeasurementUpdate' */
typedef struct {
  real_T Product3[2];                  /* '<S119>/Product3' */
} B_MeasurementUpdate_DroneRS_C_T;

/* Block signals for system '<S73>/UseCurrentEstimator' */
typedef struct {
  real_T Add[2];                       /* '<S99>/Add' */
  real_T Product2[2];                  /* '<S120>/Product2' */
} B_UseCurrentEstimator_DroneRS_T;

/* Block signals for system '<Root>/DroneRS_Compensator' */
typedef struct {
  real_T posVIS_datin[4];              /* '<S1>/posVIS_datin' */
  real_T sensordatabiasRS_datin[7];    /* '<S1>/sensordatabiasRS_datin' */
  real_T sensordataRS_datin[8];        /* '<S1>/sensordataRS_datin' */
  real_T usePosVIS_flagin;             /* '<S1>/usePosVIS_flagin' */
  real_T opticalFlowRS_datin[3];       /* '<S1>/opticalFlowRS_datin' */
  real_T FIRaccelero[3];               /* '<S15>/FIRaccelero' */
  real_T Reshapexhat[2];               /* '<S16>/Reshapexhat' */
  real_T Reshapexhat_o[2];             /* '<S73>/Reshapexhat' */
  real_T UseIPPosSwitch[2];            /* '<S71>/UseIPPosSwitch' */
  real_T batteryStatus_datin[2];       /* '<S1>/batteryStatus_datin' */
  real_T att_estimout[3];              /* '<S3>/EstimatorAttitude' */
  real_T datt_estimout[3];             /* '<S3>/EstimatorAttitude' */
  real_T acc_RS[3];                    /* '<S12>/WorldToRSinacc' */
  real_T Product2[2];                  /* '<S65>/Product2' */
  real_T Product3[2];                  /* '<S64>/Product3' */
  boolean_T controlModePosVSAtt_flagin;/* '<S1>/controlModePosVSAtt_flagin' */
  B_UseCurrentEstimator_DroneRS_T UseCurrentEstimator_f;/* '<S133>/UseCurrentEstimator' */
  B_MeasurementUpdate_DroneRS_C_T MeasurementUpdate_f;/* '<S154>/MeasurementUpdate' */
  B_UseCurrentEstimator_DroneRS_T UseCurrentEstimator_l;/* '<S73>/UseCurrentEstimator' */
  B_MeasurementUpdate_DroneRS_C_T MeasurementUpdate_p;/* '<S94>/MeasurementUpdate' */
  B_ControllerLQR1_DroneRS_Comp_T ControllerLQR1;/* '<S1>/ControllerLQR1' */
} B_DroneRS_Compensator_DroneRS_T;

/* Block states (auto storage) for system '<Root>/DroneRS_Compensator' */
typedef struct {
  real_T FIRaccelero_states[15];       /* '<S15>/FIRaccelero' */
  real_T IIRgyroz_states[5];           /* '<S15>/IIRgyroz' */
  real_T Delay_DSTATE[2];              /* '<S71>/Delay' */
  real_T IIRgyroz_states_n[10];        /* '<S74>/IIRgyroz' */
  real_T UD_DSTATE[2];                 /* '<S121>/UD' */
  real_T Delay2_DSTATE;                /* '<S12>/Delay2' */
  real_T IIRprs_states[5];             /* '<S17>/IIRprs' */
  real_T IIRsonar_states[5];           /* '<S17>/IIRsonar' */
  real_T MemoryX_DSTATE[2];            /* '<S16>/MemoryX' */
  real_T Delay_DSTATE_l[2];            /* '<S70>/Delay' */
  real_T MemoryX_DSTATE_f[2];          /* '<S73>/MemoryX' */
  real_T Delay1_DSTATE[2];             /* '<S3>/Delay1' */
  real_T MemoryX_DSTATE_a[2];          /* '<S133>/MemoryX' */
  real_T SimplyIntegrateVelocity_DSTATE[2];/* '<S71>/SimplyIntegrateVelocity' */
  real_T IIRgyroz_tmp_f[2];            /* '<S74>/IIRgyroz' */
  real_T yaw_cur;                      /* '<S3>/EstimatorAttitude' */
  real_T pitch_cur;                    /* '<S3>/EstimatorAttitude' */
  real_T roll_cur;                     /* '<S3>/EstimatorAttitude' */
  struct {
    void *LoggedData;
  } SCOPE_anglesRSestim_PWORK;         /* '<S3>/SCOPE_anglesRSestim' */

  struct {
    void *LoggedData;
  } eulerrates_PWORK;                  /* '<S3>/eulerrates' */

  struct {
    void *LoggedData;
  } SCOPE_altPrs_PWORK;                /* '<S12>/SCOPE_altPrs' */

  struct {
    void *LoggedData;
  } SCOPE_kalmanaltestim_PWORK;        /* '<S12>/SCOPE_kalmanaltestim' */

  struct {
    void *LoggedData;
  } SCOPEdxy_PWORK;                    /* '<S70>/SCOPEdxy' */

  struct {
    void *LoggedData;
  } SCOPEenableKFdxupdate_PWORK;       /* '<S70>/SCOPEenableKFdxupdate' */

  int32_T FIRaccelero_circBuf;         /* '<S15>/FIRaccelero' */
  int8_T SimplyIntegrateVelocity_PrevRes;/* '<S71>/SimplyIntegrateVelocity' */
  uint8_T icLoad;                      /* '<S16>/MemoryX' */
  uint8_T icLoad_c;                    /* '<S73>/MemoryX' */
  uint8_T icLoad_e;                    /* '<S133>/MemoryX' */
  DW_ControllerLQR1_DroneRS_Com_T ControllerLQR1;/* '<S1>/ControllerLQR1' */
} DW_DroneRS_Compensator_DroneR_T;

/* Block signals (auto storage) */
typedef struct {
  B_DroneRS_Compensator_DroneRS_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
} B_DroneRS_Compensator_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  DW_DroneRS_Compensator_DroneR_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
} DW_DroneRS_Compensator_T;

/* Parameters for system: '<S1>/ControllerLQR1' */
struct P_ControllerLQR1_DroneRS_Comp_T_ {
  real_T takeoff_Gain_Gain;            /* Expression: controlParams.takeoff_Gain
                                        * Referenced by: '<S9>/takeoff_Gain'
                                        */
  real_T dz_ref_Value;                 /* Expression: 0
                                        * Referenced by: '<S6>/dz_ref'
                                        */
  real_T velocitiesPos_ref_Value[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<S6>/velocitiesPos_ref'
                                        */
  real_T velocitiesRot_ref_Value[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<S6>/velocitiesRot_ref'
                                        */
  real_T TorquetotalThrustToThrustperMot[16];/* Expression: controlParams.Q2Ts
                                              * Referenced by: '<S7>/TorquetotalThrustToThrustperMotor'
                                              */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S11>/Constant'
                                        */
  real_T MotorsRotationDirection_Gain[16];/* Expression: diag([-1,1,-1,1])
                                           * Referenced by: '<S8>/MotorsRotationDirection'
                                           */
  real_T ThrustperMotorToTotalThrustandT[16];/* Expression: controlParams.Ts2Q
                                              * Referenced by: '<S8>/ThrustperMotor To TotalThrust and Torques'
                                              */
  real_T SaturationThrust_UpperSat;    /* Expression: controlParams.totalThrust_maxRelative*controlParams.motorsThrust_i_UpperLimit*4
                                        * Referenced by: '<S9>/SaturationThrust'
                                        */
  real_T SaturationThrust_LowerSat;    /* Expression: -(controlParams.totalThrust_maxRelative*controlParams.motorsThrust_i_UpperLimit*4)
                                        * Referenced by: '<S9>/SaturationThrust'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: -0.0118
                                        * Referenced by: '<S7>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: -0.3235
                                        * Referenced by: '<S7>/Saturation2'
                                        */
  real_T MotorsRotationDirection_Gain_f[4];/* Expression: [-1,1,-1,1]
                                            * Referenced by: '<S10>/MotorsRotationDirection'
                                            */
};

/* Parameters for system: '<Root>/DroneRS_Compensator' */
struct P_DroneRS_Compensator_DroneRS_T_ {
  real_T DiscreteDerivative_ICPrevScaled;/* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                          * Referenced by: '<S121>/UD'
                                          */
  real_T checkPosavailable_const;      /* Mask Parameter: checkPosavailable_const
                                        * Referenced by: '<S182>/Constant'
                                        */
  real_T CompareToConstant_const;      /* Mask Parameter: CompareToConstant_const
                                        * Referenced by: '<S186>/Constant'
                                        */
  real_T outlierBelowFloor_const;      /* Mask Parameter: outlierBelowFloor_const
                                        * Referenced by: '<S20>/Constant'
                                        */
  real_T FIRaccelero_InitialStates;    /* Expression: 0
                                        * Referenced by: '<S15>/FIRaccelero'
                                        */
  real_T FIRaccelero_Coefficients[6];  /* Expression: controlParams.filter_accelero.Coefficients
                                        * Referenced by: '<S15>/FIRaccelero'
                                        */
  real_T IIRgyroz_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S15>/IIRgyroz'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S71>/Delay'
                                        */
  real_T KalmanGainM_Value[4];         /* Expression: pInitialization.M
                                        * Referenced by: '<S136>/KalmanGainM'
                                        */
  real_T IIRgyroz_InitialStates_c;     /* Expression: 0
                                        * Referenced by: '<S74>/IIRgyroz'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S121>/TSamp'
                                        */
  real_T invertzaxisGain_Gain;         /* Expression: -1
                                        * Referenced by: '<S12>/invertzaxisGain'
                                        */
  real_T SaturationSonar_LowerSat;     /* Expression: -inf
                                        * Referenced by: '<S17>/SaturationSonar'
                                        */
  real_T Delay2_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S12>/Delay2'
                                        */
  real_T IIRprs_InitialStates;         /* Expression: 0
                                        * Referenced by: '<S17>/IIRprs'
                                        */
  real_T IIRsonar_InitialStates;       /* Expression: 0
                                        * Referenced by: '<S17>/IIRsonar'
                                        */
  real_T KalmanGainM_Value_h[2];       /* Expression: pInitialization.M
                                        * Referenced by: '<S21>/KalmanGainM'
                                        */
  real_T gravity_Value[3];             /* Expression: [0 0 quad.g]
                                        * Referenced by: '<S12>/gravity'
                                        */
  real_T C_Value[2];                   /* Expression: pInitialization.C
                                        * Referenced by: '<S16>/C'
                                        */
  real_T D_Value;                      /* Expression: pInitialization.D
                                        * Referenced by: '<S16>/D'
                                        */
  real_T X0_Value[2];                  /* Expression: pInitialization.X0
                                        * Referenced by: '<S16>/X0'
                                        */
  real_T Delay_InitialCondition_n;     /* Expression: 0
                                        * Referenced by: '<S70>/Delay'
                                        */
  real_T KalmanGainM_Value_f[4];       /* Expression: pInitialization.M
                                        * Referenced by: '<S76>/KalmanGainM'
                                        */
  real_T gravity_Value_g[3];           /* Expression: [0 0 -quad.g]
                                        * Referenced by: '<S72>/gravity'
                                        */
  real_T gainaccinput_Gain;            /* Expression: 0.2
                                        * Referenced by: '<S72>/gainaccinput'
                                        */
  real_T C_Value_d[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S73>/C'
                                        */
  real_T D_Value_f[4];                 /* Expression: pInitialization.D
                                        * Referenced by: '<S73>/D'
                                        */
  real_T X0_Value_k[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S73>/X0'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S3>/Delay1'
                                        */
  real_T C_Value_f[4];                 /* Expression: pInitialization.C
                                        * Referenced by: '<S133>/C'
                                        */
  real_T D_Value_f0[4];                /* Expression: pInitialization.D
                                        * Referenced by: '<S133>/D'
                                        */
  real_T X0_Value_d[2];                /* Expression: pInitialization.X0
                                        * Referenced by: '<S133>/X0'
                                        */
  real_T SimplyIntegrateVelocity_gainval;/* Computed Parameter: SimplyIntegrateVelocity_gainval
                                          * Referenced by: '<S71>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_IC;   /* Expression: 0
                                        * Referenced by: '<S71>/SimplyIntegrateVelocity'
                                        */
  real_T SimplyIntegrateVelocity_UpperSa;/* Expression: 2
                                          * Referenced by: '<S71>/SimplyIntegrateVelocity'
                                          */
  real_T SimplyIntegrateVelocity_LowerSa;/* Expression: -2
                                          * Referenced by: '<S71>/SimplyIntegrateVelocity'
                                          */
  real_T UseIPPosSwitch_Threshold;     /* Expression: 0
                                        * Referenced by: '<S71>/UseIPPosSwitch'
                                        */
  real_T A_Value[4];                   /* Expression: pInitialization.A
                                        * Referenced by: '<S16>/A'
                                        */
  real_T B_Value[2];                   /* Expression: pInitialization.B
                                        * Referenced by: '<S16>/B'
                                        */
  real_T KalmanGainL_Value[2];         /* Expression: pInitialization.L
                                        * Referenced by: '<S21>/KalmanGainL'
                                        */
  real_T A_Value_d[4];                 /* Expression: pInitialization.A
                                        * Referenced by: '<S73>/A'
                                        */
  real_T B_Value_c[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S73>/B'
                                        */
  real_T KalmanGainL_Value_f[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S76>/KalmanGainL'
                                        */
  real_T A_Value_dj[4];                /* Expression: pInitialization.A
                                        * Referenced by: '<S133>/A'
                                        */
  real_T B_Value_k[4];                 /* Expression: pInitialization.B
                                        * Referenced by: '<S133>/B'
                                        */
  real_T KalmanGainL_Value_j[4];       /* Expression: pInitialization.L
                                        * Referenced by: '<S136>/KalmanGainL'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S71>/Delay'
                                        */
  uint32_T Delay2_DelayLength;         /* Computed Parameter: Delay2_DelayLength
                                        * Referenced by: '<S12>/Delay2'
                                        */
  uint32_T MemoryX_DelayLength;        /* Computed Parameter: MemoryX_DelayLength
                                        * Referenced by: '<S16>/MemoryX'
                                        */
  uint32_T Delay_DelayLength_g;        /* Computed Parameter: Delay_DelayLength_g
                                        * Referenced by: '<S70>/Delay'
                                        */
  uint32_T MemoryX_DelayLength_g;      /* Computed Parameter: MemoryX_DelayLength_g
                                        * Referenced by: '<S73>/MemoryX'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S3>/Delay1'
                                        */
  uint32_T MemoryX_DelayLength_m;      /* Computed Parameter: MemoryX_DelayLength_m
                                        * Referenced by: '<S133>/MemoryX'
                                        */
  P_ControllerLQR1_DroneRS_Comp_T ControllerLQR1;/* '<S1>/ControllerLQR1' */
};

/* Parameters (auto storage) */
struct P_DroneRS_Compensator_T_ {
  struct_nVjCgugzLFJzCZr6yyeDeH quadEDT;/* Variable: quadEDT
                                         * Referenced by:
                                         *   '<S12>/prsToAlt_Gain'
                                         *   '<S15>/inversesIMU_Gain'
                                         *   '<S8>/motorsRSToW2_Gain'
                                         *   '<S10>/W2ToMotorsCmd_Gain'
                                         *   '<S17>/SaturationSonar'
                                         *   '<S70>/opticalFlowToVelocity_Gain'
                                         *   '<S68>/Constant'
                                         */
  struct_pP0yJPvqYhejK9gHgcbWI quad;   /* Variable: quad
                                        * Referenced by:
                                        *   '<S8>/W2ToThrust'
                                        *   '<S9>/HoverThrustLinearizationPoint'
                                        *   '<S10>/ThrustToW2_Gain'
                                        */
  struct_eTOByJ6BrrCe8gZfBpKFUD altEstim;/* Variable: altEstim
                                          * Referenced by:
                                          *   '<S12>/Bias'
                                          *   '<S12>/Bias1'
                                          *   '<S15>/IIRgyroz'
                                          *   '<S17>/IIRprs'
                                          *   '<S17>/IIRsonar'
                                          *   '<S66>/Constant'
                                          *   '<S67>/Constant'
                                          *   '<S69>/Constant'
                                          *   '<S74>/IIRgyroz'
                                          */
  struct_rM3FFntOU5Aaym8djgtmlC ofhandle;/* Variable: ofhandle
                                          * Referenced by:
                                          *   '<S122>/Constant'
                                          *   '<S123>/Constant'
                                          *   '<S124>/Constant'
                                          *   '<S125>/Constant'
                                          *   '<S126>/Constant'
                                          *   '<S127>/Constant'
                                          *   '<S128>/Constant'
                                          *   '<S129>/Constant'
                                          *   '<S130>/Constant'
                                          *   '<S131>/Constant'
                                          *   '<S132>/Constant'
                                          */
  struct_YkbJnRR8M5ye4XtO88GdQC vishandle;/* Variable: vishandle
                                           * Referenced by:
                                           *   '<S183>/Constant'
                                           *   '<S184>/Constant'
                                           *   '<S185>/Constant'
                                           */
  real_T K_lqr_toMotorcmd[48];         /* Variable: K_lqr_toMotorcmd
                                        * Referenced by: '<S4>/Gain'
                                        */
  real_T sampleTime_qcsim;             /* Variable: sampleTime_qcsim
                                        * Referenced by: '<S3>/sampleTime'
                                        */
  P_DroneRS_Compensator_DroneRS_T DroneRS_Compensator_d;/* '<Root>/DroneRS_Compensator' */
};

/* Real-time Model Data Structure */
struct tag_RTM_DroneRS_Compensator_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * ModelData:
   * The following substructure contains information regarding
   * the data used in the model.
   */
  struct {
    B_DroneRS_Compensator_T *blockIO;
    P_DroneRS_Compensator_T *defaultParam;
    DW_DroneRS_Compensator_T *dwork;
  } ModelData;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Model entry point functions */
extern void DroneRS_Compensator_initialize(RT_MODEL_DroneRS_Compensator_T *const
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
  DroneRS_Compensator_Y_batteryStatus_datout[2]);
extern void DroneRS_Compensator_step(RT_MODEL_DroneRS_Compensator_T *const
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
  DroneRS_Compensator_Y_batteryStatus_datout[2]);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('sim_quadrotor/DroneRS_Compensator')    - opens subsystem sim_quadrotor/DroneRS_Compensator
 * hilite_system('sim_quadrotor/DroneRS_Compensator/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'sim_quadrotor'
 * '<S1>'   : 'sim_quadrotor/DroneRS_Compensator'
 * '<S2>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1'
 * '<S3>'   : 'sim_quadrotor/DroneRS_Compensator/Estimator'
 * '<S4>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/FullstateController'
 * '<S5>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/SysteminputConverter'
 * '<S6>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/statesReferences'
 * '<S7>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/SysteminputConverter/ControlMixerRS'
 * '<S8>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/SysteminputConverter/Conversion Motorcommands To TotalThrust and Torques'
 * '<S9>'   : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/SysteminputConverter/Takeoffphase_Thrustadjustment'
 * '<S10>'  : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/SysteminputConverter/Thrust2Motorcmd'
 * '<S11>'  : 'sim_quadrotor/DroneRS_Compensator/ControllerLQR1/statesReferences/Compare To Zero'
 * '<S12>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude'
 * '<S13>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAttitude'
 * '<S14>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition'
 * '<S15>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/SensorPreprocessing'
 * '<S16>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude'
 * '<S17>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling'
 * '<S18>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/RStoWorldinacc'
 * '<S19>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/WorldToRSinacc'
 * '<S20>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/outlierBelowFloor'
 * '<S21>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL'
 * '<S22>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculateYhat'
 * '<S23>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionA'
 * '<S24>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionB'
 * '<S25>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionC'
 * '<S26>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionD'
 * '<S27>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionG'
 * '<S28>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionH'
 * '<S29>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionN'
 * '<S30>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP'
 * '<S31>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionP0'
 * '<S32>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionQ'
 * '<S33>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionR'
 * '<S34>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionReset'
 * '<S35>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX'
 * '<S36>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionX0'
 * '<S37>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/DataTypeConversionu'
 * '<S38>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/MemoryP'
 * '<S39>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer'
 * '<S40>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ReducedQRN'
 * '<S41>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionP0'
 * '<S42>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionQ'
 * '<S43>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/ScalarExpansionR'
 * '<S44>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator'
 * '<S45>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkA'
 * '<S46>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkB'
 * '<S47>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkC'
 * '<S48>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkD'
 * '<S49>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkEnable'
 * '<S50>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkG'
 * '<S51>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkH'
 * '<S52>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkN'
 * '<S53>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkP0'
 * '<S54>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkQ'
 * '<S55>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkR'
 * '<S56>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkReset'
 * '<S57>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checkX0'
 * '<S58>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checku'
 * '<S59>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/checky'
 * '<S60>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionL'
 * '<S61>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionM'
 * '<S62>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionP'
 * '<S63>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/CalculatePL/DataTypeConversionZ'
 * '<S64>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/Observer/MeasurementUpdate'
 * '<S65>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/KalmanFilter_altitude/UseCurrentEstimator/Enabled Subsystem'
 * '<S66>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffprs'
 * '<S67>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/currentStateVeryOffsonarflt'
 * '<S68>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierDist_min'
 * '<S69>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorAltitude/OutlierHandling/outlierJump'
 * '<S70>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity'
 * '<S71>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition'
 * '<S72>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld'
 * '<S73>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy'
 * '<S74>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling'
 * '<S75>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/AccelerationWorld/World2Body'
 * '<S76>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL'
 * '<S77>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculateYhat'
 * '<S78>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionA'
 * '<S79>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionB'
 * '<S80>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionC'
 * '<S81>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionD'
 * '<S82>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionG'
 * '<S83>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionH'
 * '<S84>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionN'
 * '<S85>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP'
 * '<S86>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionP0'
 * '<S87>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionQ'
 * '<S88>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionR'
 * '<S89>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionReset'
 * '<S90>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX'
 * '<S91>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionX0'
 * '<S92>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/DataTypeConversionu'
 * '<S93>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/MemoryP'
 * '<S94>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer'
 * '<S95>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ReducedQRN'
 * '<S96>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionP0'
 * '<S97>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionQ'
 * '<S98>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/ScalarExpansionR'
 * '<S99>'  : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator'
 * '<S100>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkA'
 * '<S101>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkB'
 * '<S102>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkC'
 * '<S103>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkD'
 * '<S104>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkEnable'
 * '<S105>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkG'
 * '<S106>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkH'
 * '<S107>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkN'
 * '<S108>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkP0'
 * '<S109>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkQ'
 * '<S110>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkR'
 * '<S111>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkReset'
 * '<S112>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checkX0'
 * '<S113>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checku'
 * '<S114>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/checky'
 * '<S115>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionL'
 * '<S116>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionM'
 * '<S117>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionP'
 * '<S118>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/CalculatePL/DataTypeConversionZ'
 * '<S119>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/Observer/MeasurementUpdate'
 * '<S120>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/KalmanFilter_dxdy/UseCurrentEstimator/Enabled Subsystem'
 * '<S121>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/DiscreteDerivative'
 * '<S122>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxdw1'
 * '<S123>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxdw2'
 * '<S124>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxp'
 * '<S125>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxp2'
 * '<S126>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxq'
 * '<S127>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxq2'
 * '<S128>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw1'
 * '<S129>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw2'
 * '<S130>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw3'
 * '<S131>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/maxw4'
 * '<S132>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorVelocity/OutlierHandling/minHeightforOF'
 * '<S133>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy'
 * '<S134>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling'
 * '<S135>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/RStoWorld'
 * '<S136>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL'
 * '<S137>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculateYhat'
 * '<S138>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionA'
 * '<S139>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionB'
 * '<S140>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionC'
 * '<S141>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionD'
 * '<S142>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionG'
 * '<S143>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionH'
 * '<S144>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionN'
 * '<S145>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP'
 * '<S146>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionP0'
 * '<S147>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionQ'
 * '<S148>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionR'
 * '<S149>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionReset'
 * '<S150>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX'
 * '<S151>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionX0'
 * '<S152>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/DataTypeConversionu'
 * '<S153>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/MemoryP'
 * '<S154>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer'
 * '<S155>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ReducedQRN'
 * '<S156>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionP0'
 * '<S157>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionQ'
 * '<S158>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/ScalarExpansionR'
 * '<S159>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator'
 * '<S160>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkA'
 * '<S161>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkB'
 * '<S162>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkC'
 * '<S163>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkD'
 * '<S164>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkEnable'
 * '<S165>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkG'
 * '<S166>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkH'
 * '<S167>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkN'
 * '<S168>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkP0'
 * '<S169>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkQ'
 * '<S170>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkR'
 * '<S171>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkReset'
 * '<S172>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checkX0'
 * '<S173>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checku'
 * '<S174>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/checky'
 * '<S175>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionL'
 * '<S176>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionM'
 * '<S177>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionP'
 * '<S178>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/CalculatePL/DataTypeConversionZ'
 * '<S179>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/Observer/MeasurementUpdate'
 * '<S180>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/KalmanFilter_posxy/UseCurrentEstimator/Enabled Subsystem'
 * '<S181>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/abs'
 * '<S182>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/checkPosavailable'
 * '<S183>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxp3'
 * '<S184>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/maxq3'
 * '<S185>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/EstimatorXYPosition/EstimatorXYPosition/OutlierHandling/planarjumpsVISPOS'
 * '<S186>' : 'sim_quadrotor/DroneRS_Compensator/Estimator/SensorPreprocessing/Compare To Constant'
 */
#endif                                 /* RTW_HEADER_DroneRS_Compensator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
