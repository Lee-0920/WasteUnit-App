/**
 #include <OpticalADCollect.h>
 * @file
 * @brief 光学采集控制接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#include <OpticalDriver/OpticalADCollect.h>
#include <OpticalDriver/OpticalChannel.h>
#include <OpticalDriver/OpticalLed.h>
#include "Common/Types.h"
#include "DncpStack/DncpStack.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "Tracer/Trace.h"
#include "System.h"
#include "DNCP/Lai/LaiRS485Handler.h"
#include "OpticalControl.h"
#include "SystemConfig.h"
#include <string.h>

typedef enum
{
    COLLECT_IDLE, COLLECT_BUSY
} CollectStatus;

static float s_acquireADTime = 0;
static float s_acquireADUseTime = 0;

static CollectStatus s_collectStatus = COLLECT_IDLE;
static Bool s_isRequestStop = FALSE;
static Bool s_isSendEvent = FALSE;
static Bool s_isCollectNewData = FALSE;
static volatile OptacalSignalAD s_signalAD =
{ 0 }; //当前AD更新任务用于保存最新的AD值。

static Bool s_isOpenTestMode = FALSE;
static Uint8 s_channel = 0; //信号采集通道选择

static void OpticalControl_ADHandleTask(void *argument);
static void OpticalControl_ADCollectTask(void *argument);
static void OpticalControl_SignalADPeriodHandle(TimerHandle_t argument);

static xTimerHandle s_signalADNotifyPeriodTimer;
static xTaskHandle s_opticalCollectHandle;

OptacalSignalAD OpticalControl_GetFilterSignalAD(float acquireTime)
{
    OptacalSignalAD resultAD =
    { 0 };
    uint64_t sumRefData = 0;
    uint64_t sumMeaData = 0;
    uint16_t cnt = 0;
    TickType_t mainTime;

    if (acquireTime != 0)
    {
        mainTime = (TickType_t) (acquireTime * 1000) + xTaskGetTickCount();
    }
    else
    {
        mainTime = xTaskGetTickCount();
    }
    // 采集
    while (1)
    {
        do
        {
            System_Delay(2);
        } while (FALSE == s_isCollectNewData);

        if ((sumRefData + s_signalAD.reference) >= 0xFFFFFFFFFFFFFFFF
                || (sumMeaData + s_signalAD.measure) >= 0xFFFFFFFFFFFFFFFF)
        {

            TRACE_ERROR(
                    "The intermediate variable of the measuring end or the reference end is out of range.");
            break;
        }

        sumRefData += s_signalAD.reference; //获取Ref端AD值
        TRACE_DEBUG("\n ref=%d; ", s_signalAD.reference);

        sumMeaData += s_signalAD.measure; //获取Mea端AD值
        TRACE_DEBUG("mea=%d; ", s_signalAD.measure);
        cnt++;
        if (TRUE == s_isRequestStop)
        {
            resultAD.reference = 0;
            resultAD.measure = 0;
            return resultAD;
        }
        if (mainTime <= xTaskGetTickCount() || (mainTime - xTaskGetTickCount()) < 500 )
        {
            break;
        }
    }

    resultAD.reference = sumRefData / cnt;
    resultAD.measure = sumMeaData / cnt;
    TRACE_MARK("\n cnt %d", cnt);
    return resultAD;
}

OptacalSignalAD OpticalControl_GetNowSignalAD(void)
{
    return s_signalAD;
}
/**
 * @brief 光学采集初始化
 * @details
 */
void OpticalControl_Init(void)
{

    OpticalADCollect_Init();

    xTaskCreate(OpticalControl_ADHandleTask, "OpticalADHandle",
            OPTICALCONTROL_ADHANDLE_STK_SIZE, NULL,
            OPTICALCONTROL_ADHANDLE_TASK_PRIO, &s_opticalCollectHandle);

    xTaskCreate(OpticalControl_ADCollectTask, "OpticalADCollect",
            OPTICALCONTROL_ADCOLLECT_STK_SIZE, NULL,
            OPTICALCONTROL_ADCOLLECT_TASK_PRIO, 0);

    s_signalADNotifyPeriodTimer = xTimerCreate("signalADPeriod",
            (uint32_t) (1000 / portTICK_RATE_MS), pdTRUE, (void *) OPTICAL_AD_NOTIFY_PERIOD_TIMER_REIO,
            OpticalControl_SignalADPeriodHandle);

}

/**
 * @brief 启动AD采集过程
 * @param adacquiretime
 */
Bool OpticalControl_StartAcquirer(float acquireADTime)
{
    Bool ret = TRUE;
    if (s_collectStatus == COLLECT_IDLE)             // 采集处于空闲
    {
        DncpStack_ClearBufferedEvent();

        s_acquireADUseTime = xTaskGetTickCount();
        s_collectStatus = COLLECT_BUSY;
        s_isRequestStop = FALSE;
        s_acquireADTime = acquireADTime;
        TRACE_INFO("\n StartAcquirer acquireADTime: ");
        System_PrintfFloat(TRACE_LEVEL_INFO, acquireADTime, 3);
        TRACE_INFO(" s");
        vTaskResume(s_opticalCollectHandle);
    }
    else                                    // 处于采集过程
    {
        TRACE_ERROR("\n collect is busy");
        ret = FALSE;
    }
    return ret;
}

/**
 * @brief 停止AD采集过程
 * @param
 */
Bool OpticalControl_StopAcquirer()
{
    if (s_collectStatus == COLLECT_BUSY)              // 处于采集过程
    {
        s_collectStatus = COLLECT_IDLE;
        s_isRequestStop = TRUE;
        TRACE_INFO("\n Request stop acquirer.");
        return TRUE;
    }
    else                                    // 采集处于空闲
    {
        TRACE_ERROR("\n collect is idle");
        return FALSE;
    }
}

void OpticalControl_SendEventOpen(void)
{
    s_isSendEvent = TRUE;
}

void OpticalControl_SendEventClose(void)
{
    s_isSendEvent = FALSE;
}

/**
 * @brief 光学采集恢复初始化
 * @details
 */
void OpticalControl_Restore(void)
{
    OpticalControl_SendEventClose();
    OpticalControl_StopAcquirer();
}

static void OpticalControl_ADHandleTask(void *argument)
{
    OptacalSignalAD signalad =
    { 0 };

    vTaskSuspend(NULL);
    while (1)
    {
        switch (s_collectStatus)
        {
        case COLLECT_IDLE:
            vTaskSuspend(NULL);
            break;
        case COLLECT_BUSY:
            signalad = OpticalControl_GetFilterSignalAD(s_acquireADTime);
            TRACE_INFO("\n result: ref %d, mea %d", signalad.reference,
                    signalad.measure);
            s_acquireADUseTime = xTaskGetTickCount() - s_acquireADUseTime;
            s_collectStatus = COLLECT_IDLE;
            TRACE_INFO("\n Stop acquirer acquireADUseTime: ");
            System_PrintfFloat(TRACE_LEVEL_INFO, s_acquireADUseTime / 1000, 3);
            TRACE_INFO(" s");
            if (TRUE == s_isSendEvent)
            {
                AcquiredResult result;
                if (FALSE == s_isRequestStop)
                {
                    result = ACQUIRED_RESULT_FINISHED;
                }
                else
                {
                    result = ACQUIRED_RESULT_STOPPED;
                }
                Uint8 data[9] = {0};
                memcpy(data, &signalad, sizeof(signalad));
                data[8] = result;
                DncpStack_SendEvent(DSCP_EVENT_OAI_AD_ACQUIRED, (void *)data, sizeof(data));
                DncpStack_BufferEvent(DSCP_EVENT_OAI_AD_ACQUIRED, (void *)data, sizeof(data));
            }
            s_isRequestStop = FALSE;
            break;
        }
    }
}

/**
 * @brief 此函数用于测试单独某个通道信号采集，测量端和参考端的采集的相关功能将无效
 * @param isOpenTestMode 是否打开测试功能，TRUE为打开
 * @param channel 需要单独测试的通道，1 - 2
 */
void OpticalControl_CollectTestFun(Bool isOpenTestMode, Uint8 channel)
{
    s_isOpenTestMode = isOpenTestMode;
    s_channel = channel;
}

static void OpticalControl_ADCollectTask(void *argument)
{
    while (1)
    {
        if (FALSE == s_isOpenTestMode)
        {
            s_isCollectNewData = FALSE;
            s_signalAD.reference = OpticalADCollect_GetAD(PD_REF_CHANNEL);
            s_signalAD.measure = OpticalADCollect_GetAD(PD_MEA_CHANNEL);
            s_isCollectNewData = TRUE;
            System_Delay(1);
        }
        else
        {
            Uint32 valve = OpticalADCollect_GetAD(s_channel);
            switch(s_channel)
            {
                case PD_REF_CHANNEL:
                    TRACE_INFO("\n ref ");
                    break;
                case PD_MEA_CHANNEL:
                    TRACE_INFO("\n mea ");
                    break;
            }
            TRACE_INFO(" channel %d , ad: %d,", s_channel, valve);
            if(valve >= 0x800000)
            {
                valve -= 0x800000;//去掉符号位
                System_PrintfFloat(TRACE_LEVEL_INFO, valve * 2.5 / 8388607 + 2.5, 3);
            }
            else
            {
                valve = 0x800000 - valve;
                System_PrintfFloat(TRACE_LEVEL_INFO, 2.5 - valve * 2.5 / 8388607, 3);
            }
            TRACE_INFO(" V");

            System_Delay(500);
        }
    }
}

void OpticalControl_SetSignalADNotifyPeriod(float period)
{
    TRACE_INFO("\n period:");
    System_PrintfFloat(TRACE_LEVEL_INFO, period, 3);
    TRACE_INFO(" s");
    if (period > 0)
    {
        xTimerChangePeriod(s_signalADNotifyPeriodTimer,
                (uint32_t)((period * 1000) / portTICK_RATE_MS), 0);
        xTimerStart(s_signalADNotifyPeriodTimer, 0);
    }
    else
    {
        xTimerStop(s_signalADNotifyPeriodTimer, 0);
    }
}

static void OpticalControl_SignalADPeriodHandle(TimerHandle_t argument)
{
    if (TRUE == LaiRS485_GetHostStatus())
    {
        DncpStack_SendEvent(DSCP_EVENT_OAI_SIGNAL_AD_NOTICE,
                (void*) &s_signalAD, sizeof(s_signalAD));

    }
}

void OpticalControl_PrintfInfo(void)
{
    Uint32 reference = s_signalAD.reference;
    Uint32 measure = s_signalAD.measure;
    Printf("\n reference: %d, ", reference);

    if(reference >= 0x800000)
    {
        reference -= 0x800000;//去掉符号位
        System_PrintfFloat(1, reference * 2.5 / 8388607 + 2.5, 3);
    }
    else
    {
        reference = 0x800000 - reference;
        System_PrintfFloat(1, 2.5 - reference * 2.5 / 8388607, 3);
    }

    Printf(" V,measure: %d, ", measure);

    if(measure >= 0x800000)
    {
        measure -= 0x800000;//去掉符号位
        System_PrintfFloat(1, measure * 2.5 / 8388607 + 2.5, 3);
    }
    else
    {
        measure = 0x800000 - measure;
        System_PrintfFloat(1, 2.5 - measure * 2.5 / 8388607, 3);
    }
    Printf(" V");
}

