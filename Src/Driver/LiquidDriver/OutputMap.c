/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/OutputMap.h>
#include "stm32f4xx.h"

#include "Peripheral/OutputManager.h"

void OutputMap_Init(Output *output)
{
    Uint8 i;

    output[0].pin = GPIO_Pin_6;
    output[0].port = GPIOA;
    output[0].rcc = RCC_AHB1Periph_GPIOA;
    OutputDriver_Init(&output[0]);

    output[1].pin = GPIO_Pin_7;
    output[1].port = GPIOA;
    output[1].rcc = RCC_AHB1Periph_GPIOA;
    OutputDriver_Init(&output[1]);

    output[2].pin = GPIO_Pin_4;
	output[2].port = GPIOC;
	output[2].rcc = RCC_AHB1Periph_GPIOC;
	OutputDriver_Init(&output[2]);

    for(i = 0; i < OUTPUTCONF_TOTAL; i++)
    {
    	OutputDriver_Control(&output[i], OUTPUT_CLOSE);
    }

}


