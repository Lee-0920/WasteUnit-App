/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/ValveMap.h>
#include "stm32f4xx.h"
#include "SolenoidValve/ValveManager.h"

//static Valve s_valveDir[2];

void ValveMap_Init(Valve *valve)
{
    Uint8 i;

    valve[0].pin = GPIO_Pin_8;
    valve[0].port = GPIOB;
    valve[0].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[0]);

    valve[1].pin = GPIO_Pin_9;
    valve[1].port = GPIOB;
    valve[1].rcc = RCC_AHB1Periph_GPIOB;
    ValveDriver_Init(&valve[1]);

    for(i = 0; i < SOLENOIDVALVECONF_TOTALVAlVES; i++)
    {
        ValveDriver_Control(&valve[i], VAlVE_CLOSE);
    }

}


