/*
 * LEDController.h
 *
 *  Created on: 2016年9月2日
 *      Author: LIANG
 */

#ifndef SRC_OPTICALCONTROL_LEDCONTROLLER_H_
#define SRC_OPTICALCONTROL_LEDCONTROLLER_H_

#include "Common/Types.h"

typedef struct
{
    float proportion;
    float integration;
    float differential;
}LEDControllerParam;

void LEDController_Init(void);
void LEDController_Restore(void);
void LEDController_TurnOnLed(Uint8 index);
Bool LEDController_Start(Uint8 index);
void LEDController_Stop(Uint8 index);
Uint32 LEDController_GetTarget(Uint8 index);
void LEDController_SetTarget(Uint8 index, Uint32 target);
LEDControllerParam LEDController_GetParam(Uint8 index);
void LEDController_SetParam(Uint8 index, LEDControllerParam param);
Bool LEDController_AdjustToValue(Uint8 index, Uint32 targetAD, Uint32 tolerance, Uint32 timeout);
void LEDController_AdjustStop(Uint8 index);
void LEDController_SendEventOpen(Uint8 index);
void LEDController_SendEventClose(Uint8 index);

#endif /* SRC_OPTICALCONTROL_LEDCONTROLLER_H_ */
