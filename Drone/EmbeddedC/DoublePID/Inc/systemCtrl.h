/*
 * Author: HOM Ponlok
 *
 * File: systemCtrl.h
 *
 * Model 'systemCtrl'.
 *
 * Date : Mon Jul 21 15:07:56 2025
 */

#ifndef systemCtrl_h_
#define systemCtrl_h_
#ifndef systemCtrl_COMMON_INCLUDES_
#define systemCtrl_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* systemCtrl_COMMON_INCLUDES_ */

#include "systemCtrl_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  double Integrator_DSTATE;            /* '<S99>/Integrator' */
  double FilterDifferentiatorTF_states;/* '<S92>/Filter Differentiator TF' */
  double Integrator_DSTATE_e;          /* '<S264>/Integrator' */
  double FilterDifferentiatorTF_states_e;/* '<S257>/Filter Differentiator TF' */
  double Integrator_DSTATE_m;          /* '<S45>/Integrator' */
  double FilterDifferentiatorTF_states_k;/* '<S38>/Filter Differentiator TF' */
  double Integrator_DSTATE_g;          /* '<S210>/Integrator' */
  double FilterDifferentiatorTF_state_kn;/* '<S203>/Filter Differentiator TF' */
  double Integrator_DSTATE_k;          /* '<S153>/Integrator' */
  double FilterDifferentiatorTF_states_c;/* '<S146>/Filter Differentiator TF' */
  double Integrator_DSTATE_gj;         /* '<S318>/Integrator' */
  double FilterDifferentiatorTF_states_h;/* '<S311>/Filter Differentiator TF' */
} DW_systemCtrl_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [1, -1, -1, -1; 1, -1, 1, 1; 1, 1, -1, 1; 1, 1, 1, -1]
   * Referenced by: '<S2>/Constant76'
   */
  double Constant76_Value[16];
} ConstP_systemCtrl_T;

/* Real-time Model Data Structure */
struct tag_RTM_systemCtrl_T {
  const char * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_systemCtrl_T systemCtrl_DW;

/* Constant parameters (default storage) */
extern const ConstP_systemCtrl_T systemCtrl_ConstP;

/* Model entry point functions */
extern void systemCtrl_initialize(void);

/* Customized model step function */
extern void systemCtrl_step(double setpointAngle[3], doublePIDInfo *doublePID,
  double freq, double Thrust, double Euler[3], double Omega[3], uint16_t
  motorVal[4]);

/* Real-time Model object */
extern RT_MODEL_systemCtrl_T *const systemCtrl_M;

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
 * '<Root>' : 'systemCtrl'
 * '<S1>'   : 'systemCtrl/SYS'
 * '<S2>'   : 'systemCtrl/SYS/Mixing'
 * '<S3>'   : 'systemCtrl/SYS/ctrlAngle'
 * '<S4>'   : 'systemCtrl/SYS/ctrlRate'
 * '<S5>'   : 'systemCtrl/SYS/Mixing/mapminmax2'
 * '<S6>'   : 'systemCtrl/SYS/ctrlAngle/ctrlPitch'
 * '<S7>'   : 'systemCtrl/SYS/ctrlAngle/ctrlRoll'
 * '<S8>'   : 'systemCtrl/SYS/ctrlAngle/ctrlYaw'
 * '<S9>'   : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Anti-windup'
 * '<S10>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/D Gain'
 * '<S11>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/External Derivative'
 * '<S12>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Filter'
 * '<S13>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Filter ICs'
 * '<S14>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/I Gain'
 * '<S15>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Ideal P Gain'
 * '<S16>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Ideal P Gain Fdbk'
 * '<S17>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Integrator'
 * '<S18>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Integrator ICs'
 * '<S19>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/N Copy'
 * '<S20>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/N Gain'
 * '<S21>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/P Copy'
 * '<S22>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Parallel P Gain'
 * '<S23>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Reset Signal'
 * '<S24>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Saturation'
 * '<S25>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Saturation Fdbk'
 * '<S26>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Sum'
 * '<S27>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Sum Fdbk'
 * '<S28>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tracking Mode'
 * '<S29>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tracking Mode Sum'
 * '<S30>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tsamp - Integral'
 * '<S31>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tsamp - Ngain'
 * '<S32>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/postSat Signal'
 * '<S33>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/preInt Signal'
 * '<S34>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/preSat Signal'
 * '<S35>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Anti-windup/Passthrough'
 * '<S36>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/D Gain/External Parameters'
 * '<S37>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/External Derivative/Error'
 * '<S38>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Filter/Disc. Trapezoidal Filter'
 * '<S39>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S40>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Filter/Disc. Trapezoidal Filter/Tsamp/External Ts'
 * '<S41>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Filter ICs/Internal IC - Filter'
 * '<S42>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/I Gain/External Parameters'
 * '<S43>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Ideal P Gain/Passthrough'
 * '<S44>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Ideal P Gain Fdbk/Disabled'
 * '<S45>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Integrator/Discrete'
 * '<S46>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Integrator ICs/Internal IC'
 * '<S47>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/N Copy/External Parameters'
 * '<S48>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/N Gain/External Parameters'
 * '<S49>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/P Copy/Disabled'
 * '<S50>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Parallel P Gain/External Parameters'
 * '<S51>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Reset Signal/Disabled'
 * '<S52>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Saturation/Passthrough'
 * '<S53>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Saturation Fdbk/Disabled'
 * '<S54>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Sum/Sum_PID'
 * '<S55>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Sum Fdbk/Disabled'
 * '<S56>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tracking Mode/Disabled'
 * '<S57>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tracking Mode Sum/Passthrough'
 * '<S58>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tsamp - Integral/External Ts'
 * '<S59>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/Tsamp - Ngain/Passthrough'
 * '<S60>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/postSat Signal/Forward_Path'
 * '<S61>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/preInt Signal/Internal PreInt'
 * '<S62>'  : 'systemCtrl/SYS/ctrlAngle/ctrlPitch/preSat Signal/Forward_Path'
 * '<S63>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Anti-windup'
 * '<S64>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/D Gain'
 * '<S65>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/External Derivative'
 * '<S66>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Filter'
 * '<S67>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Filter ICs'
 * '<S68>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/I Gain'
 * '<S69>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Ideal P Gain'
 * '<S70>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Ideal P Gain Fdbk'
 * '<S71>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Integrator'
 * '<S72>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Integrator ICs'
 * '<S73>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/N Copy'
 * '<S74>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/N Gain'
 * '<S75>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/P Copy'
 * '<S76>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Parallel P Gain'
 * '<S77>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Reset Signal'
 * '<S78>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Saturation'
 * '<S79>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Saturation Fdbk'
 * '<S80>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Sum'
 * '<S81>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Sum Fdbk'
 * '<S82>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tracking Mode'
 * '<S83>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tracking Mode Sum'
 * '<S84>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tsamp - Integral'
 * '<S85>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tsamp - Ngain'
 * '<S86>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/postSat Signal'
 * '<S87>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/preInt Signal'
 * '<S88>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/preSat Signal'
 * '<S89>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Anti-windup/Passthrough'
 * '<S90>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/D Gain/External Parameters'
 * '<S91>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/External Derivative/Error'
 * '<S92>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Filter/Disc. Trapezoidal Filter'
 * '<S93>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S94>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Filter/Disc. Trapezoidal Filter/Tsamp/External Ts'
 * '<S95>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Filter ICs/Internal IC - Filter'
 * '<S96>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/I Gain/External Parameters'
 * '<S97>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Ideal P Gain/Passthrough'
 * '<S98>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Ideal P Gain Fdbk/Disabled'
 * '<S99>'  : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Integrator/Discrete'
 * '<S100>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Integrator ICs/Internal IC'
 * '<S101>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/N Copy/External Parameters'
 * '<S102>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/N Gain/External Parameters'
 * '<S103>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/P Copy/Disabled'
 * '<S104>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Parallel P Gain/External Parameters'
 * '<S105>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Reset Signal/Disabled'
 * '<S106>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Saturation/Passthrough'
 * '<S107>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Saturation Fdbk/Disabled'
 * '<S108>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Sum/Sum_PID'
 * '<S109>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Sum Fdbk/Disabled'
 * '<S110>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tracking Mode/Disabled'
 * '<S111>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tracking Mode Sum/Passthrough'
 * '<S112>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tsamp - Integral/External Ts'
 * '<S113>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/Tsamp - Ngain/Passthrough'
 * '<S114>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/postSat Signal/Forward_Path'
 * '<S115>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/preInt Signal/Internal PreInt'
 * '<S116>' : 'systemCtrl/SYS/ctrlAngle/ctrlRoll/preSat Signal/Forward_Path'
 * '<S117>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Anti-windup'
 * '<S118>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/D Gain'
 * '<S119>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/External Derivative'
 * '<S120>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Filter'
 * '<S121>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Filter ICs'
 * '<S122>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/I Gain'
 * '<S123>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Ideal P Gain'
 * '<S124>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Ideal P Gain Fdbk'
 * '<S125>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Integrator'
 * '<S126>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Integrator ICs'
 * '<S127>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/N Copy'
 * '<S128>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/N Gain'
 * '<S129>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/P Copy'
 * '<S130>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Parallel P Gain'
 * '<S131>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Reset Signal'
 * '<S132>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Saturation'
 * '<S133>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Saturation Fdbk'
 * '<S134>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Sum'
 * '<S135>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Sum Fdbk'
 * '<S136>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tracking Mode'
 * '<S137>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tracking Mode Sum'
 * '<S138>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tsamp - Integral'
 * '<S139>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tsamp - Ngain'
 * '<S140>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/postSat Signal'
 * '<S141>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/preInt Signal'
 * '<S142>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/preSat Signal'
 * '<S143>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Anti-windup/Passthrough'
 * '<S144>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/D Gain/External Parameters'
 * '<S145>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/External Derivative/Error'
 * '<S146>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Filter/Disc. Trapezoidal Filter'
 * '<S147>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S148>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Filter/Disc. Trapezoidal Filter/Tsamp/External Ts'
 * '<S149>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Filter ICs/Internal IC - Filter'
 * '<S150>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/I Gain/External Parameters'
 * '<S151>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Ideal P Gain/Passthrough'
 * '<S152>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Ideal P Gain Fdbk/Disabled'
 * '<S153>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Integrator/Discrete'
 * '<S154>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Integrator ICs/Internal IC'
 * '<S155>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/N Copy/External Parameters'
 * '<S156>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/N Gain/External Parameters'
 * '<S157>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/P Copy/Disabled'
 * '<S158>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Parallel P Gain/External Parameters'
 * '<S159>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Reset Signal/Disabled'
 * '<S160>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Saturation/Passthrough'
 * '<S161>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Saturation Fdbk/Disabled'
 * '<S162>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Sum/Sum_PID'
 * '<S163>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Sum Fdbk/Disabled'
 * '<S164>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tracking Mode/Disabled'
 * '<S165>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tracking Mode Sum/Passthrough'
 * '<S166>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tsamp - Integral/External Ts'
 * '<S167>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/Tsamp - Ngain/Passthrough'
 * '<S168>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/postSat Signal/Forward_Path'
 * '<S169>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/preInt Signal/Internal PreInt'
 * '<S170>' : 'systemCtrl/SYS/ctrlAngle/ctrlYaw/preSat Signal/Forward_Path'
 * '<S171>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1'
 * '<S172>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1'
 * '<S173>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1'
 * '<S174>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Anti-windup'
 * '<S175>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/D Gain'
 * '<S176>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/External Derivative'
 * '<S177>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Filter'
 * '<S178>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Filter ICs'
 * '<S179>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/I Gain'
 * '<S180>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Ideal P Gain'
 * '<S181>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Ideal P Gain Fdbk'
 * '<S182>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Integrator'
 * '<S183>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Integrator ICs'
 * '<S184>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/N Copy'
 * '<S185>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/N Gain'
 * '<S186>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/P Copy'
 * '<S187>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Parallel P Gain'
 * '<S188>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Reset Signal'
 * '<S189>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Saturation'
 * '<S190>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Saturation Fdbk'
 * '<S191>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Sum'
 * '<S192>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Sum Fdbk'
 * '<S193>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tracking Mode'
 * '<S194>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tracking Mode Sum'
 * '<S195>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tsamp - Integral'
 * '<S196>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tsamp - Ngain'
 * '<S197>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/postSat Signal'
 * '<S198>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/preInt Signal'
 * '<S199>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/preSat Signal'
 * '<S200>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Anti-windup/Passthrough'
 * '<S201>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/D Gain/External Parameters'
 * '<S202>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/External Derivative/Error'
 * '<S203>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Filter/Disc. Trapezoidal Filter'
 * '<S204>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S205>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Filter/Disc. Trapezoidal Filter/Tsamp/External Ts'
 * '<S206>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Filter ICs/Internal IC - Filter'
 * '<S207>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/I Gain/External Parameters'
 * '<S208>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Ideal P Gain/Passthrough'
 * '<S209>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Ideal P Gain Fdbk/Disabled'
 * '<S210>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Integrator/Discrete'
 * '<S211>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Integrator ICs/Internal IC'
 * '<S212>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/N Copy/External Parameters'
 * '<S213>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/N Gain/External Parameters'
 * '<S214>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/P Copy/Disabled'
 * '<S215>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Parallel P Gain/External Parameters'
 * '<S216>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Reset Signal/Disabled'
 * '<S217>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Saturation/Passthrough'
 * '<S218>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Saturation Fdbk/Disabled'
 * '<S219>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Sum/Sum_PID'
 * '<S220>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Sum Fdbk/Disabled'
 * '<S221>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tracking Mode/Disabled'
 * '<S222>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tracking Mode Sum/Passthrough'
 * '<S223>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tsamp - Integral/External Ts'
 * '<S224>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/Tsamp - Ngain/Passthrough'
 * '<S225>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/postSat Signal/Forward_Path'
 * '<S226>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/preInt Signal/Internal PreInt'
 * '<S227>' : 'systemCtrl/SYS/ctrlRate/Pitch_rate_PID1/preSat Signal/Forward_Path'
 * '<S228>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Anti-windup'
 * '<S229>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/D Gain'
 * '<S230>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/External Derivative'
 * '<S231>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Filter'
 * '<S232>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Filter ICs'
 * '<S233>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/I Gain'
 * '<S234>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Ideal P Gain'
 * '<S235>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Ideal P Gain Fdbk'
 * '<S236>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Integrator'
 * '<S237>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Integrator ICs'
 * '<S238>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/N Copy'
 * '<S239>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/N Gain'
 * '<S240>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/P Copy'
 * '<S241>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Parallel P Gain'
 * '<S242>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Reset Signal'
 * '<S243>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Saturation'
 * '<S244>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Saturation Fdbk'
 * '<S245>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Sum'
 * '<S246>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Sum Fdbk'
 * '<S247>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tracking Mode'
 * '<S248>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tracking Mode Sum'
 * '<S249>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tsamp - Integral'
 * '<S250>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tsamp - Ngain'
 * '<S251>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/postSat Signal'
 * '<S252>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/preInt Signal'
 * '<S253>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/preSat Signal'
 * '<S254>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Anti-windup/Passthrough'
 * '<S255>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/D Gain/External Parameters'
 * '<S256>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/External Derivative/Error'
 * '<S257>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Filter/Disc. Trapezoidal Filter'
 * '<S258>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S259>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Filter/Disc. Trapezoidal Filter/Tsamp/External Ts'
 * '<S260>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Filter ICs/Internal IC - Filter'
 * '<S261>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/I Gain/External Parameters'
 * '<S262>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Ideal P Gain/Passthrough'
 * '<S263>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Ideal P Gain Fdbk/Disabled'
 * '<S264>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Integrator/Discrete'
 * '<S265>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Integrator ICs/Internal IC'
 * '<S266>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/N Copy/External Parameters'
 * '<S267>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/N Gain/External Parameters'
 * '<S268>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/P Copy/Disabled'
 * '<S269>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Parallel P Gain/External Parameters'
 * '<S270>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Reset Signal/Disabled'
 * '<S271>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Saturation/Passthrough'
 * '<S272>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Saturation Fdbk/Disabled'
 * '<S273>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Sum/Sum_PID'
 * '<S274>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Sum Fdbk/Disabled'
 * '<S275>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tracking Mode/Disabled'
 * '<S276>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tracking Mode Sum/Passthrough'
 * '<S277>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tsamp - Integral/External Ts'
 * '<S278>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/Tsamp - Ngain/Passthrough'
 * '<S279>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/postSat Signal/Forward_Path'
 * '<S280>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/preInt Signal/Internal PreInt'
 * '<S281>' : 'systemCtrl/SYS/ctrlRate/Roll_rate_PID1/preSat Signal/Forward_Path'
 * '<S282>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Anti-windup'
 * '<S283>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/D Gain'
 * '<S284>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/External Derivative'
 * '<S285>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Filter'
 * '<S286>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Filter ICs'
 * '<S287>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/I Gain'
 * '<S288>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Ideal P Gain'
 * '<S289>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Ideal P Gain Fdbk'
 * '<S290>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Integrator'
 * '<S291>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Integrator ICs'
 * '<S292>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/N Copy'
 * '<S293>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/N Gain'
 * '<S294>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/P Copy'
 * '<S295>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Parallel P Gain'
 * '<S296>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Reset Signal'
 * '<S297>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Saturation'
 * '<S298>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Saturation Fdbk'
 * '<S299>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Sum'
 * '<S300>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Sum Fdbk'
 * '<S301>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tracking Mode'
 * '<S302>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tracking Mode Sum'
 * '<S303>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tsamp - Integral'
 * '<S304>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tsamp - Ngain'
 * '<S305>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/postSat Signal'
 * '<S306>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/preInt Signal'
 * '<S307>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/preSat Signal'
 * '<S308>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Anti-windup/Passthrough'
 * '<S309>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/D Gain/External Parameters'
 * '<S310>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/External Derivative/Error'
 * '<S311>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Filter/Disc. Trapezoidal Filter'
 * '<S312>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S313>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Filter/Disc. Trapezoidal Filter/Tsamp/External Ts'
 * '<S314>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Filter ICs/Internal IC - Filter'
 * '<S315>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/I Gain/External Parameters'
 * '<S316>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Ideal P Gain/Passthrough'
 * '<S317>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Ideal P Gain Fdbk/Disabled'
 * '<S318>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Integrator/Discrete'
 * '<S319>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Integrator ICs/Internal IC'
 * '<S320>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/N Copy/External Parameters'
 * '<S321>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/N Gain/External Parameters'
 * '<S322>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/P Copy/Disabled'
 * '<S323>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Parallel P Gain/External Parameters'
 * '<S324>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Reset Signal/Disabled'
 * '<S325>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Saturation/Passthrough'
 * '<S326>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Saturation Fdbk/Disabled'
 * '<S327>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Sum/Sum_PID'
 * '<S328>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Sum Fdbk/Disabled'
 * '<S329>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tracking Mode/Disabled'
 * '<S330>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tracking Mode Sum/Passthrough'
 * '<S331>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tsamp - Integral/External Ts'
 * '<S332>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/Tsamp - Ngain/Passthrough'
 * '<S333>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/postSat Signal/Forward_Path'
 * '<S334>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/preInt Signal/Internal PreInt'
 * '<S335>' : 'systemCtrl/SYS/ctrlRate/Yaw_rate_PID1/preSat Signal/Forward_Path'
 */
#endif                                 /* systemCtrl_h_ */

/* [EOF] */
