/*
 * Author: HOM Ponlok
 * File: ekf10Init.h
 *
 * Date  : 21-Jul-2025 14:43:42
 */

#ifndef EKF10INIT_H
#define EKF10INIT_H

/* Include Files */
#include "ekf10Init_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ekf10Init(double freq, const double initState[6], sysInit_t *sys);

extern void ekf10Init_initialize(void);

extern void ekf10Init_terminate(void);

extern void ekf10Step(sysInit_t *sys, const imudata_t *imu,
                      const double magNED[3], double alti);

extern void initStateEKF(const double meanAccel[3], const double meanMag[3],
                         double meanAlti, double initState[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ekf10Init.h
 *
 * [EOF]
 */
