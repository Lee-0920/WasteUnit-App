/*
 * ValveDriver.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_OUTPUTDRIVER_H_
#define SRC_DRIVER_OUTPUTDRIVER_H_

#include "Common/Types.h"
#include "stm32f4xx.h"

typedef struct
{
    GPIO_TypeDef *port;
    Uint16 pin;
    Uint32 rcc;
}Output;

typedef enum
{
    OUTPUT_CLOSE,
    OUTPUT_OPEN
}OutputStatus;

void OutputDriver_Init(Output *valve);
void OutputDriver_Control(Output *valve, OutputStatus status);
OutputStatus OutputDriver_ReadStatus(Output *valve);

#endif /* SRC_DRIVER_OUTPUTDRIVER_H_ */
