/*
 * Author: HOM Ponlok
 *
 * File: rt_nonfinite.h
 *
 * Model 'systemCtrl'.
 *
 * Date : Mon Jul 21 15:07:56 2025
 */

#ifndef rt_nonfinite_h_
#define rt_nonfinite_h_
#include <stdbool.h>

extern double rtInf;
extern double rtMinusInf;
extern double rtNaN;
extern float rtInfF;
extern float rtMinusInfF;
extern float rtNaNF;
extern bool rtIsInf(double value);
extern bool rtIsInfF(float value);
extern bool rtIsNaN(double value);
extern bool rtIsNaNF(float value);

#endif                                 /* rt_nonfinite_h_ */

/* [EOF] */
