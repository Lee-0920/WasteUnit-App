/*
 * PumpDriver.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */
#include "FreeRTOS.h"
#include "SystemConfig.h"
#include "PumpDriver.h"

void PumpDriver_Init(PumpDriver *pumpDriver)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(
    		pumpDriver->rccClock | pumpDriver->rccDir | pumpDriver->rccEnable |  pumpDriver->rccReset,
            ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = pumpDriver->pinEnable;
    GPIO_Init(pumpDriver->portEnable, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = pumpDriver->pinClock;
    GPIO_Init(pumpDriver->portClock, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = pumpDriver->pinDir;
    GPIO_Init(pumpDriver->portDir, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = pumpDriver->pinReset;
    GPIO_Init(pumpDriver->portReset, &GPIO_InitStructure);

    pumpDriver->forwardLevel = Bit_RESET;
}

void PumpDriver_Enable(PumpDriver *pumpDriver)
{
	if(pumpDriver->portEnable != NULL)
	{
		 GPIO_SetBits(pumpDriver->portEnable, pumpDriver->pinEnable);
	}
	if(pumpDriver->portReset != NULL)
	{
		Printf("\nreset\n");
		GPIO_SetBits(pumpDriver->portReset, pumpDriver->pinReset);
	}
}

void PumpDriver_SetForwardLevel(PumpDriver *pumpDriver, BitAction bitVal)
{
	pumpDriver->forwardLevel = bitVal;
}

void PumpDriver_Disable(PumpDriver *pumpDriver)
{
	if(pumpDriver->portEnable != NULL)
	{
		GPIO_ResetBits(pumpDriver->portEnable, pumpDriver->pinEnable);
	}
	if(pumpDriver->portReset != NULL)
	{
		GPIO_ResetBits(pumpDriver->portReset, pumpDriver->pinReset);
	}
}

void PumpDriver_SetDirection(PumpDriver *pumpDriver, Direction dir)
{
    if (COUNTERCLOCKWISE == dir)
    {
        GPIO_SetBits(pumpDriver->portDir, pumpDriver->pinDir);
    }
    else
    {
        GPIO_ResetBits(pumpDriver->portDir, pumpDriver->pinDir);
    }
}

void PumpDriver_PullHigh(PumpDriver *pumpDriver)
{
    GPIO_SetBits(pumpDriver->portClock, pumpDriver->pinClock);
}

void PumpDriver_PullLow(PumpDriver *pumpDriver)
{
    GPIO_ResetBits(pumpDriver->portClock, pumpDriver->pinClock);
}

void PumpDriver_Reset(PumpDriver *pumpDriver)
{

}
