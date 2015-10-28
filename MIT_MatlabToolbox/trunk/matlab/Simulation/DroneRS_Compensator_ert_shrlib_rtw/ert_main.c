/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'DroneRS_Compensator'.
 *
 * Model version                  : 1.2620
 * Simulink Coder version         : 8.8 (R2015a) 09-Feb-2015
 * C/C++ source code generated on : Wed Oct 28 10:16:10 2015
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

#include <stddef.h>
#include <stdio.h>                     /* This ert_main.c example uses printf/fflush */
#include "DroneRS_Compensator.h"       /* Model's header file */
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "rt_logging.h"
#define QUOTE1(name)                   #name
#define QUOTE(name)                    QUOTE1(name)              /* need to expand name */
#ifndef SAVEFILE
# define MATFILE2(file)                #file ".mat"
# define MATFILE1(file)                MATFILE2(file)
# define MATFILE                       MATFILE1(MODEL)
#else
# define MATFILE                       QUOTE(SAVEFILE)
#endif

static RT_MODEL_DroneRS_Compensator_T DroneRS_Compensator_M_;
static RT_MODEL_DroneRS_Compensator_T *const DroneRS_Compensator_M =
  &DroneRS_Compensator_M_;             /* Real-time model */
static P_DroneRS_Compensator_T DroneRS_Compensator_P = {
  {
    13840.8,

    { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0165195073635001, 0.0152648883285633,
      0.0215786550496705, 0.000652733165165932, 0.000721701528439517,
      0.000690781425279554 },

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.05, 0.05, 1.0, 1.0, 1.0 },

    { 0.09, -0.06, 0.337, -0.0095, -0.0075, 0.0015, 101270.95 },

    { 1.00596, 1.00383, 0.99454 },

    { 0.99861, 1.00644 },
    0.99997,
    1.225,
    12.01725,
    0.05,

    { 0.99407531114557246, 0.99618461293246863, 1.0054899752649467,
      1.0013919347893572, 0.99360120821906917, 1.0000300009000269 },
    0.44,
    0.005,

    { -99.0, 0.0, 0.0, -9.0 },
    0.0,
    0.1,
    0.05,
    -30.0,
    20.0
  },                                   /* Variable: quadEDT
                                        * Referenced by:
                                        *   '<S11>/prsToAlt_Gain'
                                        *   '<S14>/inversesIMU_Gain'
                                        *   '<S9>/W2ToMotorsCmd_Gain'
                                        *   '<S16>/SaturationSonar'
                                        *   '<S69>/opticalFlowToVelocity_Gain'
                                        *   '<S67>/Constant'
                                        */

  {
    4.0,
    9.81,
    1.184,
    1.5e-5,
    0.068,

    { 8.0299999999999987e-5, 1.17e-5, 0.0, 1.1699999999999993e-5, 8.03e-5, 0.0,
      0.0, 0.0, 0.0001366 },
    -0.015875999999999998,
    0.0624,
    2.0,
    0.033,
    0.008,
    0.0,
    0.000375,
    0.0,
    0.0,
    1.0209375000000001e-7,
    0.0,
    6.0699375000000009e-5,
    2.0418750000000001e-7,
    0.0107,
    0.00078263752785053692,
    0.15433206602850458,
    0.11868238913561441,
    0.25481807079117214,
    -0.13613568165555773,
    0.15271630954950383,
    10000.0,
    5.5,
    0.0034211943997592849,
    0.605147136,
    4.7199903669109095e-8,
    1.1392838555498841e-10,
    0
  },                                   /* Variable: quad
                                        * Referenced by:
                                        *   '<S8>/HoverThrustLinearizationPoint'
                                        *   '<S9>/ThrustToW2_Gain'
                                        */

  {
    { 0.28212412246252078, 1.2725392917168599, 2.4208439774454433,
      2.4208439774454429, 1.272539291716859, 0.28212412246252039 },

    { 1.0, 2.2287149173647665, 2.5244618916938606, 1.5772531712757014,
      0.54102240682981717, 0.079562396085500836 },

    { 0.0075092575286030347, -0.022498139897706496, 0.014988905760749223,
      0.014988905760749157, -0.022498139897706482, 0.0075092575286030382 },

    { 1.0, -4.89810443312637, 9.5974881329446688, -9.4036722433820046,
      4.60730099744654, -0.90301240709954322 },
    0.3,
    0.8,
    0.4
  },                                   /* Variable: altEstim
                                        * Referenced by:
                                        *   '<S11>/Bias'
                                        *   '<S11>/Bias1'
                                        *   '<S14>/IIRgyroz'
                                        *   '<S16>/IIRprs'
                                        *   '<S16>/IIRsonar'
                                        *   '<S65>/Constant'
                                        *   '<S66>/Constant'
                                        *   '<S68>/Constant'
                                        *   '<S73>/IIRgyroz'
                                        */

  {
    0.6,
    7.0,
    0.5,
    80.0,
    -0.4,
    5.0
  },                                   /* Variable: ofhandle
                                        * Referenced by:
                                        *   '<S121>/Constant'
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
                                        */

  {
    0.18,
    0.5
  },                                   /* Variable: vishandle
                                        * Referenced by:
                                        *   '<S182>/Constant'
                                        *   '<S183>/Constant'
                                        *   '<S184>/Constant'
                                        */

  /*  Variable: K_poleplace
   * Referenced by: '<S4>/Gain'
   */
  { 0.0, 0.0, -0.0026773635963302741, 0.0, 0.0, 0.0, 0.0, 0.0090255657492354785,
    1.7339999999999958, 0.0, 0.0, 0.0, 0.0, 0.0012703799999999911, 0.0, 0.0, 0.0,
    0.0, 0.0086952146400000041, 0.0, 0.0, 0.0, 0.0, 0.023626519999999991, 0.0,
    0.0, -0.00070644154128440438, 0.0, 0.0, 0.0, 0.0, 0.0028569704383282365,
    0.68679999999999919, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0059063999999999992,
    0.0, 0.0, 0.0012594960000000004, 0.0, 0.0, 0.000833259999999997, 0.0, 0.0 },
  0.005,                               /* Variable: sampleTime_qcsim
                                        * Referenced by: '<S3>/sampleTime'
                                        */

  /* Start of '<Root>/DroneRS_Compensator' */
  {
    0.0,                               /* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                        * Referenced by: '<S120>/UD'
                                        */
    -99.0,                             /* Mask Parameter: checkPosavailable_const
                                        * Referenced by: '<S181>/Constant'
                                        */
    -99.0,                             /* Mask Parameter: CompareToConstant_const
                                        * Referenced by: '<S185>/Constant'
                                        */
    0.0,                               /* Mask Parameter: outlierBelowFloor_const
                                        * Referenced by: '<S19>/Constant'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S14>/FIRaccelero'
                                        */

    /*  Expression: controlParams.filter_accelero.Coefficients
     * Referenced by: '<S14>/FIRaccelero'
     */
    { 0.026407724923238066, 0.14053136276241623, 0.3330609123143457,
      0.3330609123143457, 0.14053136276241623, 0.026407724923238066 },
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S14>/IIRgyroz'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S70>/Delay'
                                        */

    /*  Expression: pInitialization.M
     * Referenced by: '<S135>/KalmanGainM'
     */
    { 0.0057568600814408791, 0.0, 0.0, 0.0057568600814408791 },
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S73>/IIRgyroz'
                                        */
    200.0,                             /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S120>/TSamp'
                                        */
    -1.0,                              /* Expression: -1
                                        * Referenced by: '<S11>/invertzaxisGain'
                                        */
    0.0,                               /* Expression: -inf
                                        * Referenced by: '<S16>/SaturationSonar'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S11>/Delay2'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S16>/IIRprs'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S16>/IIRsonar'
                                        */

    /*  Expression: pInitialization.M
     * Referenced by: '<S20>/KalmanGainM'
     */
    { 0.026241420641871412, 0.0697767360714959 },

    /*  Expression: [0 0 quad.g]
     * Referenced by: '<S11>/gravity'
     */
    { 0.0, 0.0, 9.81 },

    /*  Expression: pInitialization.C
     * Referenced by: '<S15>/C'
     */
    { 1.0, 0.0 },
    0.0,                               /* Expression: pInitialization.D
                                        * Referenced by: '<S15>/D'
                                        */

    /*  Expression: pInitialization.X0
     * Referenced by: '<S15>/X0'
     */
    { -0.046, 0.0 },
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S69>/Delay'
                                        */

    /*  Expression: pInitialization.M
     * Referenced by: '<S75>/KalmanGainM'
     */
    { 0.12546560898608936, 0.0, 0.0, 0.12546560898608936 },

    /*  Expression: [0 0 -quad.g]
     * Referenced by: '<S71>/gravity'
     */
    { 0.0, 0.0, -9.81 },
    0.2,                               /* Expression: 0.2
                                        * Referenced by: '<S71>/gainaccinput'
                                        */

    /*  Expression: pInitialization.C
     * Referenced by: '<S72>/C'
     */
    { 1.0, 0.0, 0.0, 1.0 },

    /*  Expression: pInitialization.D
     * Referenced by: '<S72>/D'
     */
    { 0.0, 0.0, 0.0, 0.0 },

    /*  Expression: pInitialization.X0
     * Referenced by: '<S72>/X0'
     */
    { 0.0, 0.0 },
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S3>/Delay1'
                                        */

    /*  Expression: pInitialization.C
     * Referenced by: '<S132>/C'
     */
    { 1.0, 0.0, 0.0, 1.0 },

    /*  Expression: pInitialization.D
     * Referenced by: '<S132>/D'
     */
    { 0.0, 0.0, 0.0, 0.0 },

    /*  Expression: pInitialization.X0
     * Referenced by: '<S132>/X0'
     */
    { 0.1, 0.0 },
    0.005,                             /* Computed Parameter: SimplyIntegrateVelocity_gainval
                                        * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                        */
    2.0,                               /* Expression: 2
                                        * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                        */
    -2.0,                              /* Expression: -2
                                        * Referenced by: '<S70>/SimplyIntegrateVelocity'
                                        */
    0.0,                               /* Expression: 0
                                        * Referenced by: '<S70>/UseIPPosSwitch'
                                        */

    /*  Expression: pInitialization.A
     * Referenced by: '<S15>/A'
     */
    { 1.0, 0.0, 0.005, 1.0 },

    /*  Expression: pInitialization.B
     * Referenced by: '<S15>/B'
     */
    { 0.0, 0.005 },

    /*  Expression: pInitialization.L
     * Referenced by: '<S20>/KalmanGainL'
     */
    { 0.026590304322228892, 0.0697767360714959 },

    /*  Expression: pInitialization.A
     * Referenced by: '<S72>/A'
     */
    { 1.0, 0.0, 0.0, 1.0 },

    /*  Expression: pInitialization.B
     * Referenced by: '<S72>/B'
     */
    { 0.005, 0.0, 0.0, 0.005 },

    /*  Expression: pInitialization.L
     * Referenced by: '<S75>/KalmanGainL'
     */
    { 0.12546560898608936, 0.0, 0.0, 0.12546560898608936 },

    /*  Expression: pInitialization.A
     * Referenced by: '<S132>/A'
     */
    { 1.0, 0.0, 0.0, 1.0 },

    /*  Expression: pInitialization.B
     * Referenced by: '<S132>/B'
     */
    { 0.005, 0.0, 0.0, 0.005 },

    /*  Expression: pInitialization.L
     * Referenced by: '<S135>/KalmanGainL'
     */
    { 0.0057568600814408791, 0.0, 0.0, 0.0057568600814408791 },
    1U,                                /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S70>/Delay'
                                        */
    1U,                                /* Computed Parameter: Delay2_DelayLength
                                        * Referenced by: '<S11>/Delay2'
                                        */
    1U,                                /* Computed Parameter: MemoryX_DelayLength
                                        * Referenced by: '<S15>/MemoryX'
                                        */
    1U,                                /* Computed Parameter: Delay_DelayLength_g
                                        * Referenced by: '<S69>/Delay'
                                        */
    1U,                                /* Computed Parameter: MemoryX_DelayLength_g
                                        * Referenced by: '<S72>/MemoryX'
                                        */
    1U,                                /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S3>/Delay1'
                                        */
    1U,                                /* Computed Parameter: MemoryX_DelayLength_m
                                        * Referenced by: '<S132>/MemoryX'
                                        */

    /* Start of '<S1>/ControllerFSFB' */
    {
      0.05,                            /* Expression: controlParams.takeoff_Gain
                                        * Referenced by: '<S8>/takeoff_Gain'
                                        */
      0.0,                             /* Expression: 0
                                        * Referenced by: '<S6>/dz_ref'
                                        */

      /*  Expression: [0;0;0]
       * Referenced by: '<S6>/velocitiesPos_ref'
       */
      { 0.0, 0.0, 0.0 },

      /*  Expression: [0;0;0]
       * Referenced by: '<S6>/velocitiesRot_ref'
       */
      { 0.0, 0.0, 0.0 },

      /*  Expression: controlParams.Q2Ts
       * Referenced by: '<S7>/TorquetotalThrustToThrustperMotor'
       */
      { 0.25, 0.25, 0.25, 0.25, 103.57362530676717, -103.57362530676717,
        103.57362530676717, -103.57362530676717, -5.6659197210460546,
        -5.6659197210460546, 5.6659197210460546, 5.6659197210460546,
        -5.6659197210460546, 5.6659197210460546, 5.6659197210460546,
        -5.6659197210460546 },
      0.0,                             /* Expression: 0
                                        * Referenced by: '<S10>/Constant'
                                        */
      1.2020433451342656,              /* Expression: controlParams.totalThrust_maxRelative*controlParams.motorsThrust_i_UpperLimit*4
                                        * Referenced by: '<S8>/SaturationThrust'
                                        */
      -1.2020433451342656,             /* Expression: -(controlParams.totalThrust_maxRelative*controlParams.motorsThrust_i_UpperLimit*4)
                                        * Referenced by: '<S8>/SaturationThrust'
                                        */
      -0.0118,                         /* Expression: -0.0118
                                        * Referenced by: '<S7>/Saturation2'
                                        */
      -0.3235,                         /* Expression: -0.3235
                                        * Referenced by: '<S7>/Saturation2'
                                        */

      /*  Expression: [-1,1,-1,1]
       * Referenced by: '<S9>/MotorsRotationDirection'
       */
      { -1.0, 1.0, -1.0, 1.0 }
    }
    /* End of '<S1>/ControllerFSFB' */
  }
  /* End of '<Root>/DroneRS_Compensator' */
};                                     /* Modifiable parameters */

static B_DroneRS_Compensator_T DroneRS_Compensator_B;/* Observable signals */
static DW_DroneRS_Compensator_T DroneRS_Compensator_DW;/* Observable states */

/* '<Root>/controlModePosVSAtt_flagin' */
static boolean_T DroneRS_Compensator_U_controlModePosVSAtt_flagin;

/* '<Root>/pos_refin' */
static real_T DroneRS_Compensator_U_pos_refin[3];

/* '<Root>/attRS_refin' */
static real_T DroneRS_Compensator_U_attRS_refin[3];

/* '<Root>/ddx' */
static real_T DroneRS_Compensator_U_ddx;

/* '<Root>/ddy' */
static real_T DroneRS_Compensator_U_ddy;

/* '<Root>/ddz' */
static real_T DroneRS_Compensator_U_ddz;

/* '<Root>/p' */
static real_T DroneRS_Compensator_U_p;

/* '<Root>/q' */
static real_T DroneRS_Compensator_U_q;

/* '<Root>/r' */
static real_T DroneRS_Compensator_U_r;

/* '<Root>/altitude_sonar' */
static real_T DroneRS_Compensator_U_altitude_sonar;

/* '<Root>/prs' */
static real_T DroneRS_Compensator_U_prs;

/* '<Root>/opticalFlowRS_datin' */
static real_T DroneRS_Compensator_U_opticalFlowRS_datin[3];

/* '<Root>/sensordatabiasRS_datin' */
static real_T DroneRS_Compensator_U_sensordatabiasRS_datin[7];

/* '<Root>/posVIS_datin' */
static real_T DroneRS_Compensator_U_posVIS_datin[4];

/* '<Root>/usePosVIS_flagin' */
static real_T DroneRS_Compensator_U_usePosVIS_flagin;

/* '<Root>/batteryStatus_datin' */
static real_T DroneRS_Compensator_U_batteryStatus_datin[2];

/* '<Root>/motorsRS_cmdout' */
static real_T DroneRS_Compensator_Y_motorsRS_cmdout[4];

/* '<Root>/X' */
static real_T DroneRS_Compensator_Y_X;

/* '<Root>/Y' */
static real_T DroneRS_Compensator_Y_Y;

/* '<Root>/Z' */
static real_T DroneRS_Compensator_Y_Z;

/* '<Root>/yaw' */
static real_T DroneRS_Compensator_Y_yaw;

/* '<Root>/pitch' */
static real_T DroneRS_Compensator_Y_pitch;

/* '<Root>/roll' */
static real_T DroneRS_Compensator_Y_roll;

/* '<Root>/dx' */
static real_T DroneRS_Compensator_Y_dx;

/* '<Root>/dy' */
static real_T DroneRS_Compensator_Y_dy;

/* '<Root>/dz' */
static real_T DroneRS_Compensator_Y_dz;

/* '<Root>/pb' */
static real_T DroneRS_Compensator_Y_pb;

/* '<Root>/qb' */
static real_T DroneRS_Compensator_Y_qb;

/* '<Root>/rb' */
static real_T DroneRS_Compensator_Y_rb;

/* '<Root>/controlModePosVSAtt_flagout' */
static boolean_T DroneRS_Compensator_Y_controlModePosVSAtt_flagout;

/* '<Root>/poseRS_refout' */
static real_T DroneRS_Compensator_Y_poseRS_refout[6];

/* '<Root>/ddxb' */
static real_T DroneRS_Compensator_Y_ddxb;

/* '<Root>/ddyb' */
static real_T DroneRS_Compensator_Y_ddyb;

/* '<Root>/ddzb' */
static real_T DroneRS_Compensator_Y_ddzb;

/* '<Root>/pa' */
static real_T DroneRS_Compensator_Y_pa;

/* '<Root>/qa' */
static real_T DroneRS_Compensator_Y_qa;

/* '<Root>/ra' */
static real_T DroneRS_Compensator_Y_ra;

/* '<Root>/altitude_sonarb' */
static real_T DroneRS_Compensator_Y_altitude_sonarb;

/* '<Root>/prsb' */
static real_T DroneRS_Compensator_Y_prsb;

/* '<Root>/opticalFlowRS_datout' */
static real_T DroneRS_Compensator_Y_opticalFlowRS_datout[3];

/* '<Root>/sensordatabiasRS_datout' */
static real_T DroneRS_Compensator_Y_sensordatabiasRS_datout[7];

/* '<Root>/posVIS_datout' */
static real_T DroneRS_Compensator_Y_posVIS_datout[4];

/* '<Root>/usePosVIS_flagout' */
static real_T DroneRS_Compensator_Y_usePosVIS_flagout;

/* '<Root>/batteryStatus_datout' */
static real_T DroneRS_Compensator_Y_batteryStatus_datout[2];

/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
void rt_OneStep(RT_MODEL_DroneRS_Compensator_T *const DroneRS_Compensator_M);
void rt_OneStep(RT_MODEL_DroneRS_Compensator_T *const DroneRS_Compensator_M)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(DroneRS_Compensator_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  DroneRS_Compensator_step(DroneRS_Compensator_M,
    DroneRS_Compensator_U_controlModePosVSAtt_flagin,
    DroneRS_Compensator_U_pos_refin, DroneRS_Compensator_U_attRS_refin,
    DroneRS_Compensator_U_ddx, DroneRS_Compensator_U_ddy,
    DroneRS_Compensator_U_ddz, DroneRS_Compensator_U_p, DroneRS_Compensator_U_q,
    DroneRS_Compensator_U_r, DroneRS_Compensator_U_altitude_sonar,
    DroneRS_Compensator_U_prs, DroneRS_Compensator_U_opticalFlowRS_datin,
    DroneRS_Compensator_U_sensordatabiasRS_datin,
    DroneRS_Compensator_U_posVIS_datin, DroneRS_Compensator_U_usePosVIS_flagin,
    DroneRS_Compensator_U_batteryStatus_datin,
    DroneRS_Compensator_Y_motorsRS_cmdout, &DroneRS_Compensator_Y_X,
    &DroneRS_Compensator_Y_Y, &DroneRS_Compensator_Y_Z,
    &DroneRS_Compensator_Y_yaw, &DroneRS_Compensator_Y_pitch,
    &DroneRS_Compensator_Y_roll, &DroneRS_Compensator_Y_dx,
    &DroneRS_Compensator_Y_dy, &DroneRS_Compensator_Y_dz,
    &DroneRS_Compensator_Y_pb, &DroneRS_Compensator_Y_qb,
    &DroneRS_Compensator_Y_rb,
    &DroneRS_Compensator_Y_controlModePosVSAtt_flagout,
    DroneRS_Compensator_Y_poseRS_refout, &DroneRS_Compensator_Y_ddxb,
    &DroneRS_Compensator_Y_ddyb, &DroneRS_Compensator_Y_ddzb,
    &DroneRS_Compensator_Y_pa, &DroneRS_Compensator_Y_qa,
    &DroneRS_Compensator_Y_ra, &DroneRS_Compensator_Y_altitude_sonarb,
    &DroneRS_Compensator_Y_prsb, DroneRS_Compensator_Y_opticalFlowRS_datout,
    DroneRS_Compensator_Y_sensordatabiasRS_datout,
    DroneRS_Compensator_Y_posVIS_datout,
    &DroneRS_Compensator_Y_usePosVIS_flagout,
    DroneRS_Compensator_Y_batteryStatus_datout);

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * The example "main" function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific.  This example
 * illustates how you do this relative to initializing the model.
 */
int_T main(int_T argc, const char *argv[])
{
  /* Unused arguments */
  (void)(argc);
  (void)(argv);

  /* Pack model data into RTM */
  DroneRS_Compensator_M->ModelData.defaultParam = &DroneRS_Compensator_P;
  DroneRS_Compensator_M->ModelData.blockIO = &DroneRS_Compensator_B;
  DroneRS_Compensator_M->ModelData.dwork = &DroneRS_Compensator_DW;

  /* Initialize model */
  DroneRS_Compensator_initialize(DroneRS_Compensator_M,
    &DroneRS_Compensator_U_controlModePosVSAtt_flagin,
    DroneRS_Compensator_U_pos_refin, DroneRS_Compensator_U_attRS_refin,
    &DroneRS_Compensator_U_ddx, &DroneRS_Compensator_U_ddy,
    &DroneRS_Compensator_U_ddz, &DroneRS_Compensator_U_p,
    &DroneRS_Compensator_U_q, &DroneRS_Compensator_U_r,
    &DroneRS_Compensator_U_altitude_sonar, &DroneRS_Compensator_U_prs,
    DroneRS_Compensator_U_opticalFlowRS_datin,
    DroneRS_Compensator_U_sensordatabiasRS_datin,
    DroneRS_Compensator_U_posVIS_datin, &DroneRS_Compensator_U_usePosVIS_flagin,
    DroneRS_Compensator_U_batteryStatus_datin,
    DroneRS_Compensator_Y_motorsRS_cmdout, &DroneRS_Compensator_Y_X,
    &DroneRS_Compensator_Y_Y, &DroneRS_Compensator_Y_Z,
    &DroneRS_Compensator_Y_yaw, &DroneRS_Compensator_Y_pitch,
    &DroneRS_Compensator_Y_roll, &DroneRS_Compensator_Y_dx,
    &DroneRS_Compensator_Y_dy, &DroneRS_Compensator_Y_dz,
    &DroneRS_Compensator_Y_pb, &DroneRS_Compensator_Y_qb,
    &DroneRS_Compensator_Y_rb,
    &DroneRS_Compensator_Y_controlModePosVSAtt_flagout,
    DroneRS_Compensator_Y_poseRS_refout, &DroneRS_Compensator_Y_ddxb,
    &DroneRS_Compensator_Y_ddyb, &DroneRS_Compensator_Y_ddzb,
    &DroneRS_Compensator_Y_pa, &DroneRS_Compensator_Y_qa,
    &DroneRS_Compensator_Y_ra, &DroneRS_Compensator_Y_altitude_sonarb,
    &DroneRS_Compensator_Y_prsb, DroneRS_Compensator_Y_opticalFlowRS_datout,
    DroneRS_Compensator_Y_sensordatabiasRS_datout,
    DroneRS_Compensator_Y_posVIS_datout,
    &DroneRS_Compensator_Y_usePosVIS_flagout,
    DroneRS_Compensator_Y_batteryStatus_datout);

  /* The MAT-file logging option selected; therefore, simulating
   * the model step behavior (in non real-time).  Running this
   * code produces results that can be loaded into MATLAB.
   */
  while (rtmGetErrorStatus(DroneRS_Compensator_M) == (NULL)) {
    rt_OneStep(DroneRS_Compensator_M);
  }

  /* Matfile logging */
  rt_StopDataLogging(MATFILE, DroneRS_Compensator_M->rtwLogInfo);

  /* Disable rt_OneStep() here */
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
