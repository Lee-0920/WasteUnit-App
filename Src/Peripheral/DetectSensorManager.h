/*
 * DetectSensorManager.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_DETECTSENSOR_MANAGER_H_
#define SRC_DETECTSENSOR_MANAGER_H_

#include "Common/Types.h"

#define DETECTSENSORCONF_TOTAL		        7
#define DETECTSENSOR_MAX_MAP                 0x7F

void DetectSensorManager_Init(void);
Uint16 DetectSensorManager_GetTotalSensors(void);
Bool DetectSensorManager_GetSensor(Uint8 index);
Uint32 DetectSensorManager_GetSensorsMap(void);

#endif /* SRC_SOLENOIDVALVE_DetectSensorManager_H_ */
