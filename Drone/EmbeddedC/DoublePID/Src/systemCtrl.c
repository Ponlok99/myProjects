/*
 * Author: HOM Ponlok
 *
 * File: systemCtrl.c
 *
 * Model 'systemCtrl'.
 *
 * Date : Mon Jul 21 15:07:56 2025
 */

#include "systemCtrl.h"
#include "systemCtrl_types.h"
#include <stdint.h>
#include "systemCtrl_private.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Block states (default storage) */
DW_systemCtrl_T systemCtrl_DW;

/* Real-time model */
static RT_MODEL_systemCtrl_T systemCtrl_M_;
RT_MODEL_systemCtrl_T *const systemCtrl_M = &systemCtrl_M_;
double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void systemCtrl_step(double setpointAngle[3], doublePIDInfo *doublePID, double
                     freq, double Thrust, double Euler[3], double Omega[3],
                     uint16_t motorVal[4])
{
  double Integrator;
  double Integrator_c;
  double Integrator_f;
  double Integrator_i;
  double Integrator_k;
  double denAccum;
  double denAccum_0;
  double denAccum_1;
  double denAccum_2;
  double denAccum_3;
  double rtb_Add5;
  double rtb_NProdOut;
  double rtb_PProdOut;
  double rtb_Reciprocal;
  double rtb_Reciprocal_l;
  double rtb_Sum;
  double rtb_Sum_a;
  double rtb_UintegralTsProdOut;
  double rtb_UintegralTsProdOut_a;
  double rtb_UintegralTsProdOut_f;
  double rtb_UintegralTsProdOut_g;
  double rtb_UintegralTsProdOut_k;
  int32_t i;
  rtb_Reciprocal = setpointAngle[0] - Euler[2];
  rtb_NProdOut = 1.0 / freq;
  rtb_UintegralTsProdOut = rtb_Reciprocal * doublePID->angles.roll[1] *
    rtb_NProdOut;
  Integrator = systemCtrl_DW.Integrator_DSTATE + 0.5 * rtb_UintegralTsProdOut;
  rtb_Add5 = doublePID->angles.roll[3] * (0.5 * rtb_NProdOut);
  rtb_PProdOut = 1.0 / (rtb_Add5 + 1.0);
  denAccum = rtb_Reciprocal * doublePID->angles.roll[2] - rtb_PProdOut *
    (rtb_Add5 - 1.0) * systemCtrl_DW.FilterDifferentiatorTF_states;
  rtb_Reciprocal_l = ((rtb_Reciprocal * doublePID->angles.roll[0] + Integrator)
                      + (denAccum - systemCtrl_DW.FilterDifferentiatorTF_states)
                      * rtb_PProdOut * doublePID->angles.roll[3]) - Omega[2];
  rtb_UintegralTsProdOut_k = rtb_Reciprocal_l * doublePID->rates.rollRate[1] *
    rtb_NProdOut;
  Integrator_f = systemCtrl_DW.Integrator_DSTATE_e + 0.5 *
    rtb_UintegralTsProdOut_k;
  rtb_Add5 = doublePID->rates.rollRate[3] * (0.5 * rtb_NProdOut);
  rtb_Reciprocal = 1.0 / (rtb_Add5 + 1.0);
  denAccum_0 = rtb_Reciprocal_l * doublePID->rates.rollRate[2] - rtb_Reciprocal *
    (rtb_Add5 - 1.0) * systemCtrl_DW.FilterDifferentiatorTF_states_e;
  rtb_Sum = (rtb_Reciprocal_l * doublePID->rates.rollRate[0] + Integrator_f) +
    (denAccum_0 - systemCtrl_DW.FilterDifferentiatorTF_states_e) *
    rtb_Reciprocal * doublePID->rates.rollRate[3];
  rtb_PProdOut = setpointAngle[1] - Euler[1];
  rtb_UintegralTsProdOut_f = rtb_PProdOut * doublePID->angles.pitch[1] *
    rtb_NProdOut;
  Integrator_c = systemCtrl_DW.Integrator_DSTATE_m + 0.5 *
    rtb_UintegralTsProdOut_f;
  rtb_Reciprocal_l = doublePID->angles.pitch[3] * (0.5 * rtb_NProdOut);
  rtb_Reciprocal = 1.0 / (rtb_Reciprocal_l + 1.0);
  denAccum_1 = rtb_PProdOut * doublePID->angles.pitch[2] - rtb_Reciprocal *
    (rtb_Reciprocal_l - 1.0) * systemCtrl_DW.FilterDifferentiatorTF_states_k;
  rtb_Add5 = ((rtb_PProdOut * doublePID->angles.pitch[0] + Integrator_c) +
              (denAccum_1 - systemCtrl_DW.FilterDifferentiatorTF_states_k) *
              rtb_Reciprocal * doublePID->angles.pitch[3]) - Omega[1];
  rtb_UintegralTsProdOut_g = rtb_Add5 * doublePID->rates.pitchRate[1] *
    rtb_NProdOut;
  Integrator_i = systemCtrl_DW.Integrator_DSTATE_g + 0.5 *
    rtb_UintegralTsProdOut_g;
  rtb_Reciprocal_l = doublePID->rates.pitchRate[3] * (0.5 * rtb_NProdOut);
  rtb_Reciprocal = 1.0 / (rtb_Reciprocal_l + 1.0);
  denAccum_2 = rtb_Add5 * doublePID->rates.pitchRate[2] - rtb_Reciprocal *
    (rtb_Reciprocal_l - 1.0) * systemCtrl_DW.FilterDifferentiatorTF_state_kn;
  rtb_Sum_a = (rtb_Add5 * doublePID->rates.pitchRate[0] + Integrator_i) +
    (denAccum_2 - systemCtrl_DW.FilterDifferentiatorTF_state_kn) *
    rtb_Reciprocal * doublePID->rates.pitchRate[3];
  rtb_PProdOut = setpointAngle[2] - Euler[0];
  rtb_UintegralTsProdOut_a = rtb_PProdOut * doublePID->angles.yaw[1] *
    rtb_NProdOut;
  Integrator_k = systemCtrl_DW.Integrator_DSTATE_k + 0.5 *
    rtb_UintegralTsProdOut_a;
  rtb_Reciprocal_l = doublePID->angles.yaw[3] * (0.5 * rtb_NProdOut);
  rtb_Reciprocal = 1.0 / (rtb_Reciprocal_l + 1.0);
  denAccum_3 = rtb_PProdOut * doublePID->angles.yaw[2] - rtb_Reciprocal *
    (rtb_Reciprocal_l - 1.0) * systemCtrl_DW.FilterDifferentiatorTF_states_c;
  rtb_Add5 = ((rtb_PProdOut * doublePID->angles.yaw[0] + Integrator_k) +
              (denAccum_3 - systemCtrl_DW.FilterDifferentiatorTF_states_c) *
              rtb_Reciprocal * doublePID->angles.yaw[3]) - Omega[0];
  rtb_Reciprocal = rtb_Add5 * 0.0 * rtb_NProdOut;
  rtb_PProdOut = systemCtrl_DW.Integrator_DSTATE_gj + 0.5 * rtb_Reciprocal;
  rtb_NProdOut = 0.0 * (0.5 * rtb_NProdOut);
  rtb_Reciprocal_l = 1.0 / (rtb_NProdOut + 1.0);
  rtb_NProdOut = rtb_Add5 * 0.0 - rtb_Reciprocal_l * (rtb_NProdOut - 1.0) *
    systemCtrl_DW.FilterDifferentiatorTF_states_h;
  rtb_Add5 = (rtb_Add5 * 0.0 + rtb_PProdOut) + (rtb_NProdOut -
    systemCtrl_DW.FilterDifferentiatorTF_states_h) * rtb_Reciprocal_l * 0.0;
  for (i = 0; i < 4; i++) {
    rtb_Reciprocal_l = ((systemCtrl_ConstP.Constant76_Value[i] * Thrust +
                         systemCtrl_ConstP.Constant76_Value[i + 4] * rtb_Sum) +
                        systemCtrl_ConstP.Constant76_Value[i + 8] * rtb_Sum_a) +
      systemCtrl_ConstP.Constant76_Value[i + 12] * rtb_Add5;
    if (rtb_Reciprocal_l > 2000.0) {
      rtb_Reciprocal_l = 2000.0;
    } else if (rtb_Reciprocal_l < 1000.0) {
      rtb_Reciprocal_l = 1000.0;
    }

    rtb_Reciprocal_l = rt_roundd_snf(2.0 * (rtb_Reciprocal_l - 1000.0) + 47.0);
    if (rtIsNaN(rtb_Reciprocal_l)) {
      motorVal[i] = 0U;
    } else {
      motorVal[i] = (uint16_t)rtb_Reciprocal_l;
    }
  }

  systemCtrl_DW.Integrator_DSTATE = Integrator + 0.5 * rtb_UintegralTsProdOut;
  systemCtrl_DW.FilterDifferentiatorTF_states = denAccum;
  systemCtrl_DW.Integrator_DSTATE_e = Integrator_f + 0.5 *
    rtb_UintegralTsProdOut_k;
  systemCtrl_DW.FilterDifferentiatorTF_states_e = denAccum_0;
  systemCtrl_DW.Integrator_DSTATE_m = Integrator_c + 0.5 *
    rtb_UintegralTsProdOut_f;
  systemCtrl_DW.FilterDifferentiatorTF_states_k = denAccum_1;
  systemCtrl_DW.Integrator_DSTATE_g = Integrator_i + 0.5 *
    rtb_UintegralTsProdOut_g;
  systemCtrl_DW.FilterDifferentiatorTF_state_kn = denAccum_2;
  systemCtrl_DW.Integrator_DSTATE_k = Integrator_k + 0.5 *
    rtb_UintegralTsProdOut_a;
  systemCtrl_DW.FilterDifferentiatorTF_states_c = denAccum_3;
  systemCtrl_DW.Integrator_DSTATE_gj = rtb_PProdOut + 0.5 * rtb_Reciprocal;
  systemCtrl_DW.FilterDifferentiatorTF_states_h = rtb_NProdOut;
}

/* Model initialize function */
void systemCtrl_initialize(void)
{
  /* (no initialization code required) */
}

/* [EOF] */
