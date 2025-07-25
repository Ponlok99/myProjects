/*
 * Author: HOM Ponlok
 * File: rt_nonfinite.c
 *
 * Date  : 21-Jul-2025 14:43:42
 */

/*
 * Abstract:
 *      MATLAB for code generation function to initialize non-finites,
 *      (Inf, NaN and -Inf).
 */
/* Include Files */
#include "rt_nonfinite.h"
#include <math.h>

#if defined(__ICL) && __ICL == 1700
#pragma warning(disable : 264)
#endif

real_T rtNaN = (real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = (real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

#if defined(__ICL) && __ICL == 1700
#pragma warning(default : 264)
#endif

/*
 * Function: rtIsInf ==================================================
 *  Abstract:
 *  Test if value is infinite
 */
boolean_T rtIsInf(real_T value)
{
  return (isinf(value) != 0U);
}

/*
 * Function: rtIsInfF =================================================
 *  Abstract:
 *  Test if single-precision value is infinite
 */
boolean_T rtIsInfF(real32_T value)
{
  return (isinf((real_T)value) != 0U);
}

/*
 * Function: rtIsNaN ==================================================
 *  Abstract:
 *  Test if value is not a number
 */
boolean_T rtIsNaN(real_T value)
{
  return (isnan(value) != 0U);
}

/*
 * Function: rtIsNaNF =================================================
 *  Abstract:
 *  Test if single-precision value is not a number
 */
boolean_T rtIsNaNF(real32_T value)
{
  return (isnan((real_T)value) != 0U);
}

/*
 * File trailer for rt_nonfinite.c
 *
 * [EOF]
 */
