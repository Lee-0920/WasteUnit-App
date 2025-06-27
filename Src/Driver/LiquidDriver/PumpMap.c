/*
 * PumpMap.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */
#include "stm32f4xx.h"
#include "PumpDriver.h"
#include "PumpMap.h"
#include "HardwareType.h"

void PumpMap_Init(Pump *pump)
{
	pump[0].driver.pinClock = GPIO_Pin_1;
	pump[0].driver.portClock = GPIOD;
	pump[0].driver.rccClock = RCC_AHB1Periph_GPIOD;

	pump[0].driver.pinDir = GPIO_Pin_0;
	pump[0].driver.portDir = GPIOD;
	pump[0].driver.rccDir = RCC_AHB1Periph_GPIOD;

	pump[0].driver.pinReset = GPIO_Pin_2;
	pump[0].driver.portReset = GPIOD;
	pump[0].driver.rccReset = RCC_AHB1Periph_GPIOD;

	pump[0].driver.pinEnable = GPIO_Pin_3;
	pump[0].driver.portEnable = GPIOD;
	pump[0].driver.rccEnable = RCC_AHB1Periph_GPIOD;

	PumpDriver_Init(&pump[0].driver);
	PumpDriver_PullLow(&pump[0].driver);
	PumpDriver_Disable(&pump[0].driver);

	pump[1].driver.pinClock = GPIO_Pin_5;
	pump[1].driver.portClock = GPIOD;
	pump[1].driver.rccClock = RCC_AHB1Periph_GPIOD;

	pump[1].driver.pinDir = GPIO_Pin_4;
	pump[1].driver.portDir = GPIOD;
	pump[1].driver.rccDir = RCC_AHB1Periph_GPIOD;

	pump[1].driver.pinReset = GPIO_Pin_6;
	pump[1].driver.portReset = GPIOD;
	pump[1].driver.rccReset = RCC_AHB1Periph_GPIOD;

	pump[1].driver.pinEnable = GPIO_Pin_7;
	pump[1].driver.portEnable = GPIOD;
	pump[1].driver.rccEnable = RCC_AHB1Periph_GPIOD;

	PumpDriver_Init(&pump[1].driver);
	PumpDriver_PullLow(&pump[1].driver);
	PumpDriver_Disable(&pump[1].driver);
}

