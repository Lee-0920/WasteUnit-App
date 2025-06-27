/*
 * MeterLEDDriver.h
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_METERLED_H_
#define SRC_DRIVER_METERLED_H_

#include "Common/Types.h"
#include "stm32f4xx.h"

void MeterLED_Init(void);
Bool MeterLED_TurnOn(uint8_t index);
Bool MeterLED_TurnOff(uint8_t index);

#endif /* SRC_DRIVER_METERLED_H_ */
