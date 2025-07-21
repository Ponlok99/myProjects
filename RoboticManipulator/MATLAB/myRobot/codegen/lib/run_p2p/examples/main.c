/*
 * File: main.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "rt_nonfinite.h"
#include "run_p2p.h"
#include "run_p2p_initialize.h"
#include "run_p2p_terminate.h"
#include "run_p2p_types.h"

/* Variable Definitions */
static run_p2pStackData run_p2pStackDataGlobal;

/* Function Declarations */
static void argInit_3x2_real_T(double result[2][3]);

static p2pMode argInit_p2pMode(void);

static double argInit_real_T(void);

static trajMode argInit_trajMode(void);

/* Function Definitions */
/*
 * Arguments    : double result[2][3]
 * Return Type  : void
 */
static void argInit_3x2_real_T(double result[2][3])
{
  int idx0;
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      /* Set the value of the array element.
Change this value to the value that the application requires. */
      result[idx1][idx0] = argInit_real_T();
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : p2pMode
 */
static p2pMode argInit_p2pMode(void)
{
  return moveJ;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : trajMode
 */
static trajMode argInit_trajMode(void)
{
  return trapveltraj;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  static run_p2pPersistentData run_p2pPersistentDataGlobal;
  (void)argc;
  (void)argv;
  run_p2pStackDataGlobal.pd = &run_p2pPersistentDataGlobal;
  /* Initialize the application.
You do not need to do this more than one time. */
  run_p2p_initialize(&run_p2pStackDataGlobal);
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_run_p2p();
  /* Terminate the application.
You do not need to do this more than one time. */
  run_p2p_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_run_p2p(void)
{
  double jointTorq[101][6];
  double qInterp[101][6];
  double qdInterp[101][6];
  double pose[3][101];
  double wpts_tmp[2][3];
  /* Initialize function 'run_p2p' input arguments. */
  /* Initialize function input argument 'wpts'. */
  argInit_3x2_real_T(wpts_tmp);
  /* Initialize function input argument 'orientations'. */
  /* Initialize function input argument 'waypointVels'. */
  /* Initialize function input argument 'waypointAccels'. */
  /* Initialize function input argument 'waypointJerks'. */
  /* Call the entry-point 'run_p2p'. */
  run_p2p(&run_p2pStackDataGlobal, wpts_tmp, wpts_tmp, argInit_p2pMode(),
          argInit_trajMode(), wpts_tmp, wpts_tmp, wpts_tmp, pose, qInterp,
          qdInterp, jointTorq);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
