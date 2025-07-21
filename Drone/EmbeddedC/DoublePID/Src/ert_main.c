/*
 * Author: HOM Ponlok
 *
 * File: ert_main.c
 *
 * Model 'systemCtrl'.
 *
 * Date : Mon Jul 21 15:07:56 2025
 */

#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */
#include "systemCtrl.h"                /* Model header file */

/* Global variables used by function prototype control */

/* '<Root>/setpointAngle' */
static double setpointAngle[3] = { 0.0, 0.0, 0.0 };

/* '<Root>/doublePID' */
static doublePIDInfo doublePID = {
  {
    {
      0.0, 0.0, 0.0, 0.0 }
    ,                                  /* roll */

    {
      0.0, 0.0, 0.0, 0.0 }
    ,                                  /* pitch */

    {
      0.0, 0.0, 0.0, 0.0 }
    /* yaw */
  },                                   /* angles */

  {
    {
      0.0, 0.0, 0.0, 0.0 }
    ,                                  /* rollRate */

    {
      0.0, 0.0, 0.0, 0.0 }
    ,                                  /* pitchRate */

    {
      0.0, 0.0, 0.0, 0.0 }
    /* yawRate */
  }                                    /* rates */
};

/* '<Root>/freq' */
static double freq = 0.0;

/* '<Root>/Thrust' */
static double Thrust = 0.0;

/* '<Root>/Euler' */
static double Euler[3] = { 0.0, 0.0, 0.0 };

/* '<Root>/Omega' */
static double Omega[3] = { 0.0, 0.0, 0.0 };

/* '<Root>/motorVal' */
static uint16_t motorVal[4];

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
void rt_OneStep(void);
void rt_OneStep(void)
{
  static bool OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(systemCtrl_M, "Overrun");
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  systemCtrl_step(setpointAngle, &doublePID, freq, Thrust, Euler, Omega,
                  motorVal);

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * The example main function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific. This example
 * illustrates how you do this relative to initializing the model.
 */
int main(int argc, const char *argv[])
{
  /* Unused arguments */
  (void)(argc);
  (void)(argv);

  /* Initialize model */
  systemCtrl_initialize();

  /* Attach rt_OneStep to a timer or interrupt service routine with
   * period 0.2 seconds (base rate of the model) here.
   * The call syntax for rt_OneStep is
   *
   *  rt_OneStep();
   */
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(systemCtrl_M) == (NULL)) {
    /*  Perform application tasks here */
  }

  return 0;
}

/* [EOF] */
