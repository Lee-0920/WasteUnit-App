/*
 * ValveDriver.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/DetectSensorDriver.h>

void DetectSensorDriver_Init(Sensor *sensor)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(sensor->rcc, ENABLE);

    GPIO_InitStructure.GPIO_Pin = sensor->pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(sensor->port, &GPIO_InitStructure);
}

SensorStatus DetectSensorDriver_ReadStatus(Sensor *sensor)
{
    if(GPIO_ReadInputDataBit(sensor->port,sensor->pin))
    {
        return SENSOR_HIGH;
    }
    else
    {
        return SENSOR_LOW;
    }
}
