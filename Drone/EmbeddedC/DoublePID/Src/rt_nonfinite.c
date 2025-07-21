/*
 * Author: HOM Ponlok
 *
 * File: rt_nonfinite.c
 *
 * Model 'systemCtrl'.
 *
 * Date : Mon Jul 21 15:07:56 2025
 */

#include <stdbool.h>
#include "rt_nonfinite.h"
#include "math.h"

double rtNaN = -(double)NAN;
double rtInf = (double)INFINITY;
double rtMinusInf = -(double)INFINITY;
float rtNaNF = -(float)NAN;
float rtInfF = (float)INFINITY;
float rtMinusInfF = -(float)INFINITY;

/* Test if value is infinite */
bool rtIsInf(double value)
{
  return (bool)isinf(value);
}

/* Test if single-precision value is infinite */
bool rtIsInfF(float value)
{
  return (bool)isinf(value);
}

/* Test if value is not a number */
bool rtIsNaN(double value)
{
  return (bool)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
bool rtIsNaNF(float value)
{
  return (bool)(isnan(value) != 0);
}

/* [EOF] */
