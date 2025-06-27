/*
 * ValveDriver.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/OutputDriver.h>

void OutputDriver_Init(Output *output)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(output->rcc, ENABLE);

    GPIO_InitStructure.GPIO_Pin = output->pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(output->port, &GPIO_InitStructure);
    GPIO_ResetBits(output->port, output->pin);
}

void OutputDriver_Control(Output *output, OutputStatus status)
{
    if (OUTPUT_OPEN == status)
    {
        GPIO_SetBits(output->port,output->pin);
    }
    else
    {
        GPIO_ResetBits(output->port,output->pin);
    }
}

OutputStatus OutputDriver_ReadStatus(Output *output)
{
    if(GPIO_ReadOutputDataBit(output->port,output->pin))
    {
        return OUTPUT_OPEN;
    }
    else
    {
        return OUTPUT_CLOSE;
    }
}
