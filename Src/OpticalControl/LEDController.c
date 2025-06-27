/*
 * LEDController.c
 *
 *  Created on: 2016年9月2日
 *      Author: LIANG
 */

#include <OpticalDriver/OpticalLed.h>
#include "Tracer/Trace.h"
#include "FreeRTOS.h"
#include "DncpStack/DncpStack.h"
#include "task.h"
#include "System.h"
#include "OpticalControl.h"
#include "LEDController.h"
#include "SystemConfig.h"
#include "String.h"
#include "Driver/McuFlash.h"
#include "math.h"

#define LED_DAC_DEFAULT_VALVE  1
#define PID_TARGET_DEFAULT_VALVE 11000000

typedef enum
{
    LEDCTL_IDLE,
    LEDCTL_BUSY
} LEDCTLStatus;

typedef enum
{
    LED_CONTROL_AUTO,
    LED_CONTROL_ONCE,
}LEDControlMode;

typedef struct
{
    Uint8 index;

    LEDControlMode ledctlMode;
    LEDCTLStatus ledctlStatus;
    int ledctlTimeout;
    Uint32 ledctlTolerance;
    AdjustResult ledctlResult;
    Bool ledctlAdjustOver;
    Bool ledctlSendEvent;

    float LEDDACOut;
    Uint32 PIDTarget;
    float lastError;
    float lastDifferential;

    LEDControllerParam LEDControllerParam;

    TaskHandle_t LEDHandle;
}LEDController;

#define LED_CONTROL_NUM  2

static LEDController g_ledController[2];

const static LEDControllerParam kDefaultLEDControllerParamm =
{ .proportion = 0.003, .integration = 0.01, .differential = 0.006, };

static void LEDController_LEDHandle(void *argument);

void LEDController_Init(void)
{
    Uint8 buffer[LEDCONTROLLER_SIGN_FLASH_LEN] = { 0 };
    Uint32 flashFactorySign = 0;
    Uint8 index = 0;

    for(index = 0; index < LED_CONTROL_NUM; index++)
    {
        if(index > 0)  //扩展的LED控制器
        {
            McuFlash_Read(EXTLEDCONTROLLER_SIGN_FLASH_BASE_ADDR, LEDCONTROLLER_SIGN_FLASH_LEN, buffer); //读取出厂标志位
        }
        else
        {
            McuFlash_Read(LEDCONTROLLER_SIGN_FLASH_BASE_ADDR, LEDCONTROLLER_SIGN_FLASH_LEN, buffer); //读取出厂标志位
        }
        memcpy(&flashFactorySign, buffer, LEDCONTROLLER_SIGN_FLASH_LEN);

        if (FLASH_FACTORY_SIGN == flashFactorySign) //表示已经过出厂设置
        {
            g_ledController[index].PIDTarget = LEDController_GetTarget(index);
            g_ledController[index].LEDControllerParam = LEDController_GetParam(index);
        }
        else
        {
            LEDController_SetTarget(index, PID_TARGET_DEFAULT_VALVE);
            LEDController_SetParam(index, kDefaultLEDControllerParamm);

            flashFactorySign = FLASH_FACTORY_SIGN;
            memcpy(buffer, &flashFactorySign, LEDCONTROLLER_SIGN_FLASH_LEN);

            if(index > 0)  //扩展的LED控制器
            {
                McuFlash_Write(EXTLEDCONTROLLER_SIGN_FLASH_BASE_ADDR, LEDCONTROLLER_SIGN_FLASH_LEN, buffer);
            }
            else
            {
                McuFlash_Write(LEDCONTROLLER_SIGN_FLASH_BASE_ADDR, LEDCONTROLLER_SIGN_FLASH_LEN, buffer);
            }
        }

        g_ledController[index].index = index;
        g_ledController[index].ledctlMode = LED_CONTROL_AUTO;
        g_ledController[index].ledctlStatus = LEDCTL_IDLE;

        xTaskCreate(LEDController_LEDHandle, "MeaLEDPID", MEASURE_LED_PID_HANDLE_STK_SIZE, (void *)&g_ledController[index], MEASURE_LED_PID_HANDLE_TASK_PRIO,
                &g_ledController[index].LEDHandle);
    }

    OpticalLed_Init();
}

void LEDController_Restore(void)
{
    LEDController_Stop(0);
    LEDController_Stop(1);
}

/**
 * @brief 打开LED灯，其控制电压为默认值
 */
void LEDController_TurnOnLed(Uint8 index)
{
    OpticalLed_TurnOn(index);
    TRACE_INFO("\n LED %d turn on", index);
}

Uint32 LEDController_GetTarget(Uint8 index)
{
    Uint8 readData[LEDCONTROLLER_TARGET_FLASH_LEN] = { 0 };
    Uint32 target = 0;

    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return target;
    }

    if(index > 0)  //扩展的LED控制器
    {
        McuFlash_Read(EXTLEDCONTROLLER_TARGET_ADDRESS, LEDCONTROLLER_TARGET_FLASH_LEN, readData);
    }
    else
    {
        McuFlash_Read(LEDCONTROLLER_TARGET_ADDRESS, LEDCONTROLLER_TARGET_FLASH_LEN, readData);
    }
    memcpy(&target, readData, sizeof(Uint32));

    return target;
}

void LEDController_SetTarget(Uint8 index, Uint32 target)
{
    Uint8 writeData[LEDCONTROLLER_TARGET_FLASH_LEN] = { 0 };

    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return;
    }

    memcpy(writeData, &target, sizeof(Uint32));
    if(index > 0)  //扩展的LED控制器
    {
        McuFlash_Write(EXTLEDCONTROLLER_TARGET_ADDRESS, LEDCONTROLLER_TARGET_FLASH_LEN, writeData);
    }
    else
    {
        McuFlash_Write(LEDCONTROLLER_TARGET_ADDRESS, LEDCONTROLLER_TARGET_FLASH_LEN, writeData);
    }
    g_ledController[index].PIDTarget = target;
    ///s_PIDTarget = target;
    TRACE_INFO("\n LEDController %d Target %d:", index, g_ledController[index].PIDTarget);
}

LEDControllerParam LEDController_GetParam(Uint8 index)
{
    Uint8 readData[LEDCONTROLLER_PARAM_FLASH_LEN] = { 0 };
    LEDControllerParam ledControllerParam = {0};

    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return ledControllerParam;
    }

    if(index > 0)  //扩展的LED控制器
    {
        McuFlash_Read(EXTLEDCONTROLLER_PARAM_ADDRESS, LEDCONTROLLER_PARAM_FLASH_LEN, readData);
    }
    else
    {
        McuFlash_Read(LEDCONTROLLER_PARAM_ADDRESS, LEDCONTROLLER_PARAM_FLASH_LEN, readData);
    }
    memcpy(&ledControllerParam, readData, sizeof(LEDControllerParam));

    return ledControllerParam;
}

void LEDController_SetParam(Uint8 index, LEDControllerParam param)
{
    Uint8 writeData[LEDCONTROLLER_PARAM_FLASH_LEN] = { 0 };

    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return;
    }

    memcpy(writeData, &param, sizeof(LEDControllerParam));
    if(index > 0)  //扩展的LED控制器
    {
        McuFlash_Write(EXTLEDCONTROLLER_PARAM_ADDRESS, LEDCONTROLLER_PARAM_FLASH_LEN, writeData);
    }
    else
    {
        McuFlash_Write(LEDCONTROLLER_PARAM_ADDRESS, LEDCONTROLLER_PARAM_FLASH_LEN, writeData);
    }
    g_ledController[index].LEDControllerParam = param;

    TRACE_INFO("\n LEDController %d Param: { P = %f, I = %f, D = %f}", index, param.proportion, param.integration, param.differential);
}

Bool LEDController_Start(Uint8 index)
{
    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return FALSE;
    }

    if (LEDCTL_IDLE == g_ledController[index].ledctlStatus)
    {
        g_ledController[index].PIDTarget = LEDController_GetTarget(index);
        g_ledController[index].lastError = 0;
        g_ledController[index].lastDifferential = 0;
        g_ledController[index].LEDDACOut = LED_DAC_DEFAULT_VALVE;
        g_ledController[index].ledctlStatus = LEDCTL_BUSY;
        g_ledController[index].ledctlMode = LED_CONTROL_AUTO;
        OpticalLed_TurnOn(index);
        vTaskResume(g_ledController[index].LEDHandle);
        TRACE_INFO("\n LEDController  %d start", index);
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n LEDController %d failed to start because it is running.", index);
        return FALSE;
    }
}

Bool LEDController_AdjustToValue(Uint8 index, Uint32 targetAD, Uint32 tolerance, Uint32 timeout)
{
    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return FALSE;
    }

    if(targetAD <= 16777215)
    {
        if (LEDCTL_IDLE == g_ledController[index].ledctlStatus)
        {
            g_ledController[index].PIDTarget = targetAD;
            g_ledController[index].lastError = 0;
            g_ledController[index].lastDifferential = 0;
            g_ledController[index].LEDDACOut = LED_DAC_DEFAULT_VALVE;
            g_ledController[index].ledctlStatus = LEDCTL_BUSY;
            g_ledController[index].ledctlMode = LED_CONTROL_ONCE;
            g_ledController[index].ledctlResult = ADJUST_RESULT_FAILED;
            g_ledController[index].ledctlAdjustOver = FALSE;
            g_ledController[index].ledctlTimeout = timeout;
            g_ledController[index].ledctlTolerance = tolerance;
            OpticalLed_TurnOn(index);
            vTaskResume(g_ledController[index].LEDHandle);
            TRACE_INFO("\n LEDController %d adjust start: targetAD =  %d,  tolerance = %d, timeout = %d", index, targetAD, tolerance, timeout);
            return TRUE;
        }
        else
        {
            TRACE_ERROR("\n LEDController %d adjust failed to start because it is running.", index);
            return FALSE;
        }
    }
    else
    {
        TRACE_ERROR("\n LEDController adjust failed to start because target AD out of range");
        return FALSE;
    }
}

void LEDController_AdjustStop(Uint8 index)
{
    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return;
    }

    OpticalLed_TurnOff(index);
    TRACE_INFO("\n led %d close.", index);
    if (LEDCTL_BUSY == g_ledController[index].ledctlStatus)
    {
        g_ledController[index].ledctlStatus = LEDCTL_IDLE;
        TRACE_INFO("\n LEDController %d Adjust stop.", index);

        if(g_ledController[index].ledctlAdjustOver == FALSE)
        {
            g_ledController[index].ledctlResult = ADJUST_RESULT_STOPPED;
        }

        if(g_ledController[index].ledctlSendEvent == TRUE)
        {
            Uint8 data[2] = {0};
            data[0] = g_ledController[index].ledctlResult;
            data[1] = index;

            // 发送结果事件
            DncpStack_SendEvent(DSCP_EVENT_OAI_AD_ADJUST_RESULT, (void*)data, 2);
            DncpStack_BufferEvent(DSCP_EVENT_OAI_AD_ADJUST_RESULT, (void*)data, 2);
            TRACE_INFO("\n LEDController %d Adjust Stopped.  Result = %d", index, (int)g_ledController[index].ledctlResult);
            LEDController_SendEventClose(index);  //关闭事件发送
        }

        g_ledController[index].ledctlAdjustOver = FALSE;
        g_ledController[index].ledctlResult = ADJUST_RESULT_FAILED;
        g_ledController[index].ledctlTimeout = 10000;
        g_ledController[index].ledctlTolerance = 500000;
    }
    else
    {
        TRACE_INFO("\n LEDController %d Adjust is not running.", index);
    }
}


void LEDController_Stop(Uint8 index)
{
    if(index >= LED_CONTROL_NUM)
    {
        TRACE_ERROR("Error LED Controller index = %d", index);
        return;
    }

    OpticalLed_TurnOff(index);
    TRACE_INFO("\n led %d close.", index);
    if (LEDCTL_BUSY == g_ledController[index].ledctlStatus)
    {
        g_ledController[index].ledctlStatus = LEDCTL_IDLE;
        TRACE_INFO("\n LEDController %d stop.", index);
    }
    else
    {
        TRACE_INFO("\n LEDController %d is not running.", index);
    }
}

static float LEDController_IncPIDCalc(Uint8 index)
{
    Uint32 currentAD;
    float E,DE,Dout;
    int Err;
    OptacalSignalAD optacalSignalAD = OpticalControl_GetNowSignalAD();
    currentAD = optacalSignalAD.reference;

    if((g_ledController[index].ledctlMode == LED_CONTROL_ONCE) && ((Uint32)(currentAD - g_ledController[index].PIDTarget) < g_ledController[index].ledctlTolerance
            || (Uint32)(g_ledController[index].PIDTarget - currentAD) < g_ledController[index].ledctlTolerance))
    {
        g_ledController[index].ledctlResult = ADJUST_RESULT_FINISHED;
        Dout = 0;
        g_ledController[index].ledctlAdjustOver = TRUE;
        TRACE_CODE("\n LEDController Adjust reach to %d.", (Uint32)currentAD);
    }
    else
    {
        Err = g_ledController[index].PIDTarget - currentAD;
        E = (float)Err/1000000.0;
        TRACE_CODE("\n Err: %d, E: %f", Err, E);
        //E = ((float)(s_PIDTarget - currentAD) / 1000000);
        DE = E - g_ledController[index].lastError;

        Dout = g_ledController[index].LEDControllerParam.proportion * DE
                + g_ledController[index].LEDControllerParam.integration * E
                + g_ledController[index].LEDControllerParam.differential * (DE - g_ledController[index].lastDifferential);
        g_ledController[index].lastError = E;
        g_ledController[index].lastDifferential = DE;
        TRACE_CODE("\n objref: %d, curref: %d, curmea: %d, pidout: %f", g_ledController[index].PIDTarget,
                optacalSignalAD.reference, optacalSignalAD.measure, Dout);
    }

    return Dout;
}

void LEDController_LEDHandle(void *argument)
{
    LEDController* ledController = (LEDController*)argument;
    vTaskSuspend(NULL);
    while (1)
    {
        switch (ledController->ledctlStatus)
        {
        case LEDCTL_IDLE:
            OpticalLed_TurnOff(ledController->index);
            vTaskSuspend(NULL);
            break;
        case LEDCTL_BUSY:
            System_Delay(200);
            if(ledController->ledctlMode == LED_CONTROL_ONCE)
            {
                ledController->ledctlTimeout -= 200;
                if(ledController->ledctlTimeout <= 0)
                {
                    ledController->ledctlResult = ADJUST_RESULT_TIMEOUT;
                    ledController->ledctlAdjustOver = TRUE;
                    TRACE_INFO("\n LEDController Adjust timeout.");
                }
                ledController->LEDDACOut += LEDController_IncPIDCalc(ledController->index);
            }
            else
            {
                ledController->LEDDACOut += LEDController_IncPIDCalc(ledController->index);
            }

            if (ledController->LEDDACOut > DAC_MAX_LIMIT_VALUE)
            {
                ledController->LEDDACOut = DAC_MAX_LIMIT_VALUE;
            }
            else if (ledController->LEDDACOut < 0)
            {
                ledController->LEDDACOut = 0;
            }
            TRACE_MARK("LEDDACOut: %f", ledController->LEDDACOut);
            OpticalLed_ChangeDACOut(ledController->index, ledController->LEDDACOut);

            if((ledController->ledctlMode == LED_CONTROL_ONCE) && (ledController->ledctlAdjustOver == TRUE))
            {
                if(ledController->ledctlSendEvent == TRUE)
                {
                    Uint8 data[2] = {0};
                    data[0] = ledController->ledctlResult;
                    data[1] = ledController->index;

                    // 发送结果事件
                    DncpStack_SendEvent(DSCP_EVENT_OAI_AD_ADJUST_RESULT, (void*)data, 2);
                    DncpStack_BufferEvent(DSCP_EVENT_OAI_AD_ADJUST_RESULT, (void*)data, 2);
                    TRACE_INFO("\n LEDController %d Adjust Over.  Result = %d", ledController->index, (int)ledController->ledctlResult);
                    LEDController_SendEventClose(ledController->index);  //关闭事件发送
                }

                ledController->ledctlAdjustOver = FALSE;
                ledController->ledctlResult = ADJUST_RESULT_FAILED;
                ledController->ledctlTimeout = 10000;
                ledController->ledctlTolerance = 500000;
                ledController->ledctlStatus = LEDCTL_IDLE;  //停止调节
            }

            break;
        }
    }
}

/**
 * @brief 打开调光停止发送事件功能
 */
void LEDController_SendEventOpen(Uint8 index)
{
    g_ledController[index].ledctlSendEvent = TRUE;
}

/**
 * @brief 关闭调光停止发送事件功能
 */
void LEDController_SendEventClose(Uint8 index)
{
    g_ledController[index].ledctlSendEvent = FALSE;
}

