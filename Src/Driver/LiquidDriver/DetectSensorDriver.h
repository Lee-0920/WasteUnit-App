/*
 * ValveDriver.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_DETECTSENSORDRIVER_H_
#define SRC_DRIVER_DETECTSENSORDRIVER_H_

#include "Common/Types.h"
#include "stm32f4xx.h"

typedef struct
{
    GPIO_TypeDef *port;
    Uint16 pin;
    Uint32 rcc;
}Sensor;

typedef enum
{
   SENSOR_LOW,
   SENSOR_HIGH
}SensorStatus;

void DetectSensorDriver_Init(Sensor *sensor);
SensorStatus DetectSensorDriver_ReadStatus(Sensor *sensor);

#endif /* SRC_DRIVER_DetectSensorDriver_H_ */
