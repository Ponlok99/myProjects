/*
 * Author: HOM Ponlok
 * File: ekf10Init_types.h
 *
 * Date  : 21-Jul-2025 14:43:42
 */

#ifndef EKF10INIT_TYPES_H
#define EKF10INIT_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_sysInit_t
#define typedef_sysInit_t
typedef struct {
  double euler[3];
  double dt;
  double Q[6][6];
  double R_acc[3][3];
  double R_mag[3][3];
  double R_baro;
  double x[6];
  double P[6][6];
} sysInit_t;
#endif /* typedef_sysInit_t */

#ifndef typedef_imudata_t
#define typedef_imudata_t
typedef struct {
  double gyro[3];
  double accel[3];
} imudata_t;
#endif /* typedef_imudata_t */

#endif
/*
 * File trailer for ekf10Init_types.h
 *
 * [EOF]
 */
