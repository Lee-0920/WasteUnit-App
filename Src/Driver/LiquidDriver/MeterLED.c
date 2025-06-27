/*
 * MeterLedDriver.c
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */
#include <LiquidDriver/MeterLED.h>
#include <Driver/HardwareType.h>
#include "Tracer/Trace.h"



static GPIO_BaseStruct g_meterLED[2];

static void MeterLED_Config(void)
{

    g_meterLED[0].pin = GPIO_Pin_6;
	g_meterLED[0].port = GPIOA;
	g_meterLED[0].rcc = RCC_AHB1Periph_GPIOA;

    g_meterLED[1].pin = GPIO_Pin_7;
    g_meterLED[1].port = GPIOA;
    g_meterLED[1].rcc = RCC_AHB1Periph_GPIOA;
}

void MeterLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    MeterLED_Config();

    RCC_AHB1PeriphClockCmd(g_meterLED[0].rcc | g_meterLED[1].rcc, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = g_meterLED[0].pin;
    GPIO_Init(g_meterLED[0].port, &GPIO_InitStructure);
    GPIO_ResetBits(g_meterLED[0].port, g_meterLED[0].pin);

    GPIO_InitStructure.GPIO_Pin = g_meterLED[1].pin;
    GPIO_Init(g_meterLED[1].port, &GPIO_InitStructure);
    GPIO_ResetBits(g_meterLED[1].port, g_meterLED[1].pin);
}

/**
 * @brief 开启定量LED
 * @details NUM 1-2
 */
Bool MeterLED_TurnOn(uint8_t index)
{
    if(index >= 1 && index <= 2)
    {
    	GPIO_SetBits(g_meterLED[index-1].port, g_meterLED[index-1].pin);
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n No. %d LED.NUM: 1 - 2", index);
        return FALSE;
    }
}

/**
 * @brief 关闭定量LED
 * @details NUM 1-2
 */
Bool MeterLED_TurnOff(uint8_t index)
{
    if(index >= 1 && index <= 2)
    {
    	GPIO_ResetBits(g_meterLED[index-1].port, g_meterLED[index-1].pin);
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n No. %d LED.NUM: 1 - 2", index);
        return FALSE;
    }
}

