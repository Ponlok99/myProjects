/*
 * Author: HOM Ponlok
 *
 * File: systemCtrl_types.h
 *
 * Model 'systemCtrl'.
 *
 * Date : Mon Jul 21 15:07:56 2025
 */

#ifndef systemCtrl_types_h_
#define systemCtrl_types_h_
#ifndef DEFINED_TYPEDEF_FOR_angleBusInfo_
#define DEFINED_TYPEDEF_FOR_angleBusInfo_

typedef struct {
  double roll[4];
  double pitch[4];
  double yaw[4];
} angleBusInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_rateBusInfo_
#define DEFINED_TYPEDEF_FOR_rateBusInfo_

typedef struct {
  double rollRate[4];
  double pitchRate[4];
  double yawRate[4];
} rateBusInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_doublePIDInfo_
#define DEFINED_TYPEDEF_FOR_doublePIDInfo_

typedef struct {
  angleBusInfo angles;
  rateBusInfo rates;
} doublePIDInfo;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_systemCtrl_T RT_MODEL_systemCtrl_T;

#endif                                 /* systemCtrl_types_h_ */

/* [EOF] */
