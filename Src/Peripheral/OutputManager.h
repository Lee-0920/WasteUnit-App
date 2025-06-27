/*
 * ValveManager.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_OUTPUT_MANAGER_H_
#define SRC_OUTPUT_MANAGER_H_

#include "Common/Types.h"

#define OUTPUTCONF_TOTAL         3
#define OUTPUT_MAX_MAP           0xF

void OutputManager_Init(void);
Bool OutputManager_SetOutputsMap(Uint32 map);
Uint32 OutputManager_GetOutputsMap(void);

#endif /* SRC_OUTPUT_VALVEMANAGER_H_ */
