/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/DetectSensorMap.h>
#include "stm32f4xx.h"

#include "Peripheral/DetectSensorManager.h"

void DetectSensorMap_Init(Sensor *sensor)
{
    Uint8 i;

    sensor[0].pin = GPIO_Pin_10;
    sensor[0].port = GPIOE;
    sensor[0].rcc = RCC_AHB1Periph_GPIOE;
    DetectSensorDriver_Init(&sensor[0]);

    sensor[1].pin = GPIO_Pin_11;
    sensor[1].port = GPIOE;
    sensor[1].rcc = RCC_AHB1Periph_GPIOE;
    DetectSensorDriver_Init(&sensor[1]);

    sensor[2].pin = GPIO_Pin_12;
	sensor[2].port = GPIOE;
	sensor[2].rcc = RCC_AHB1Periph_GPIOE;
	DetectSensorDriver_Init(&sensor[2]);

	sensor[3].pin = GPIO_Pin_13;
	sensor[3].port = GPIOE;
	sensor[3].rcc = RCC_AHB1Periph_GPIOE;
	DetectSensorDriver_Init(&sensor[3]);

	sensor[4].pin = GPIO_Pin_14;
	sensor[4].port = GPIOE;
	sensor[4].rcc = RCC_AHB1Periph_GPIOE;
	DetectSensorDriver_Init(&sensor[4]);

	sensor[5].pin = GPIO_Pin_15;
	sensor[5].port = GPIOE;
	sensor[5].rcc = RCC_AHB1Periph_GPIOE;
	DetectSensorDriver_Init(&sensor[5]);

	sensor[6].pin = GPIO_Pin_5;
	sensor[6].port = GPIOC;
	sensor[6].rcc = RCC_AHB1Periph_GPIOC;
	DetectSensorDriver_Init(&sensor[6]);
}


