/**
 #include <OpticalAcquirer.h>
 * @file
 * @brief 光学采集接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#include <OpticalDriver/OpticalLed.h>
#include <string.h>
#include "Common/Utils.h"
#include "DNCP/App/DscpSysDefine.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "DncpStack/DncpStack.h"
#include "LuipApi/OpticalAcquireInterface.h"
#include "OpticalControl/OpticalControl.h"
#include "OpticalControl/LEDController.h"
#include "OpticalControl/StaticADControl.h"
#include "ExtOpticalAcquirer.h"

/**
 * @brief 打开测量模块LED灯，控制LED的DAC电压为默认值。
 * @param index Uint8，测量模块LED索引。
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_TurnOnLed(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        StaticADControl_ResetMeaLedParam(index);    //重设对应LED的数字电位器参数
        LEDController_TurnOnLed(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        StaticADControl_ResetMeaLedParam(0);    //重设对应LED的数字电位器参数
        LEDController_TurnOnLed(0);
    }
    else
    {
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置信号AD上报周期。
 * @details 系统将根据设定的周期，定时向上发出信号AD上报事件。
 * @param period Float32，信号AD上报周期，单位为秒。0表示不需要上报，默认为0。
 * @see DSCP_EVENT_OAI_SIGNAL_AD_NOTICE
 * @note 所设置的上报周期将在下一次启动时丢失，默认为0，不上报。而且通信失败时不上报。
 */
void ExtOpticalAcquirer_SetSignalADNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float period;
    int size = 0;
    Uint16 ret = DSCP_OK;

    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
    }
    else
    {
        memcpy(&period, data, sizeof(float));
        OpticalControl_SetSignalADNotifyPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 启动采集过程。
 * @details 采集到的信号数据将以事件 @ref DSCP_EVENT_OAI_AD_ACQUIRED
 *  的形式上传给上位机。
 * @param acquireTime Float32，采样时间，单位为秒，0表示只采1个样本。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StartAcquirer(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    float adacquiretime;
    Uint16 size = 0;
    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }
    else
    {
        memcpy(&adacquiretime, data, sizeof(float));
        TRACE_DEBUG("\n Time %d ms", (Uint32 )(adacquiretime * 1000));
        OpticalControl_SendEventOpen();
        if (FALSE == OpticalControl_StartAcquirer(adacquiretime))
        {
            ret = DSCP_ERROR;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止采集过程。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StopAcquirer(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Bool result;
    result = OpticalControl_StopAcquirer();
    if (result == FALSE)
    {
        ret = DSCP_ERROR;
    }
    OpticalControl_SendEventOpen();
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 启动测量模块LED光强自适应控制器,并打开测量模块LED灯。
 * @param index Uint8，测量模块LED索引。
 * @details LED控制器启动后，系统将根据设定的LED控制器参数进行自动控制，尝试达到指定参考端AD值。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StartLEDController(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        StaticADControl_ResetMeaLedParam(index);    //重设对应LED的数字电位器参数
        LEDController_Start(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        StaticADControl_ResetMeaLedParam(0);    //重设对应LED的数字电位器参数
        LEDController_Start(0);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止测量模块LED光强自适应控制器和关闭测量模块LED。
 * @param index Uint8，测量模块LED索引。
 * @details 如果LED控制器没有打开则只关闭LED灯
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StopLEDController(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        LEDController_Stop(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        LEDController_Stop(0);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询测量模块LED光强自适应控制器设定的目标值。
 * @param index Uint8，测量模块LED索引。
 * @return target Uint32 ，目标值。
 * @see DSCP_CMD_OAI_SET_LEDCONTROLLER_TARGET
 */
void ExtOpticalAcquirer_GetLEDControllerTarget(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    Uint32 target = 0;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        target = LEDController_GetTarget(index);
        TRACE_DEBUG("\n LED %d controller target: %d", index, target);
    }
    else if(0 == len)  //兼容旧版命令
    {
        target = LEDController_GetTarget(0);
        TRACE_DEBUG("\n LED controller target: %d", target);
    }
    else
    {
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendResp(dscp, &target, sizeof(Uint32));
}

/**
 * @brief 设置测量模块LED光强自适应控制器设定的目标值。该参数永久保存在FLASH。
 * @param target Uint32 ，目标值。目标为参考端的AD值。
 * @param index Uint8，测量模块LED索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_SetLEDControllerTarget(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint32 target = 0;
    Uint8 index = 0;

    if (sizeof(index) + sizeof(target) == len)
    {
        memcpy(&target, data, sizeof(target));
        memcpy(&index, data + sizeof(target), sizeof(index));
        LEDController_SetTarget(index, target);
    }
    else if(sizeof(target) == len)  //兼容旧版命令
    {
        memcpy(&target, data, sizeof(Uint32));
        LEDController_SetTarget(0, target);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询测量模块LED光强自适应控制器参数。
 * @param index Uint8，测量模块LED索引。
 * @return LED控制器参数，格式如下：
 *  - proportion Float32，PID的比例系数。
 *  - integration Float32，PID的积分系数。
 *  - differential Float32，PID的微分系数。
 * @see DSCP_CMD_OAI_SET_LEDCONTROLLER_PARAM
 */
void ExtOpticalAcquirer_GetLEDControllerParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    LEDControllerParam ledControllerParam;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        ledControllerParam = LEDController_GetParam(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        ledControllerParam = LEDController_GetParam(0);
    }
    else
    {
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    TRACE_DEBUG("\n p = %f, i = %f, d = %f", ledControllerParam.proportion, ledControllerParam.integration, ledControllerParam.differential);

    DscpDevice_SendResp(dscp, &ledControllerParam, sizeof(LEDControllerParam));
}

/**
 * @brief 设置测量模块LED光强自适应控制器参数。
 * @details LED控制器将根据设置的参数进行PID调节。该参数永久保存在FLASH。
 * @param proportion Float32，PID的比例系数。
 * @param integration Float32，PID的积分系数。
 * @param differential Float32，PID的微分系数。
 * @param index Uint8，测量模块LED索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_SetLEDControllerParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    LEDControllerParam ledControllerParam;
    Uint16 ret = DSCP_OK;
    Uint8 index = 0;

    if (sizeof(ledControllerParam) + sizeof(index) == len)
    {
        memcpy(&ledControllerParam, data, sizeof(ledControllerParam));
        memcpy(&index, data+sizeof(ledControllerParam), sizeof(index));
        LEDController_SetParam(index, ledControllerParam);
    }
    else if(sizeof(ledControllerParam) == len)  //兼容旧版命令
    {
        memcpy(&ledControllerParam, data, sizeof(ledControllerParam));
        LEDController_SetParam(0, ledControllerParam);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 打开测量模块LED灯,启动AD单次定向调节至目标值操作。
 * @details LED控制器启动后，系统将根据设定的LED控制器参数进行控制，尝试达到指定参考端AD值。
 * @param targetAD Uint32，目标AD。
 * @param tolerance Uint32，容差AD。
 * @param timeout Uint32，超时时间，单位ms。
 * @param index Uint8，测量模块LED索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StartLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint32 targetAD;
    Uint32 tolerance;
    Uint32 timeout;
    Uint8 index = 0;

    if (sizeof(targetAD) + sizeof(tolerance) + sizeof(timeout)  + sizeof(index) == len)
    {
        memcpy(&targetAD, data, sizeof(targetAD));
        memcpy(&tolerance, data+sizeof(targetAD), sizeof(tolerance));
        memcpy(&timeout, data+sizeof(targetAD)+sizeof(tolerance), sizeof(timeout));
        memcpy(&index, data+sizeof(targetAD) + sizeof(tolerance) + sizeof(timeout), sizeof(index));

        LEDController_SendEventOpen(index);
        if (FALSE == LEDController_AdjustToValue(index, targetAD, tolerance, timeout))
        {
            LEDController_SendEventClose(index);
            ret = DSCP_ERROR;
        }
    }
    else if(sizeof(targetAD) + sizeof(tolerance) + sizeof(timeout) == len)  //兼容旧版命令
    {
        memcpy(&targetAD, data, sizeof(targetAD));
        memcpy(&tolerance, data+sizeof(targetAD), sizeof(tolerance));
        memcpy(&timeout, data+sizeof(targetAD)+sizeof(tolerance), sizeof(timeout));

        LEDController_SendEventOpen(0);
        if (FALSE == LEDController_AdjustToValue(0, targetAD, tolerance, timeout))
        {
            LEDController_SendEventClose(0);
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止测量模块LED单次定向调节和关闭LED。
 * @param index Uint8，测量模块LED索引。
 * @details 如果LED控制器没有打开则只关闭LED灯
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StopLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        LEDController_SendEventOpen(index);
        LEDController_AdjustStop(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        LEDController_SendEventOpen(0);
        LEDController_AdjustStop(0);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 启动采集PD电位器调节静态AD至目标值操作。
 * @details 调整信号采集PD电位器值，调整采集到的AD。
 * @param index Uint8，目标PD索引号。
 * @param targetAD Uint32，目标AD。
 * @param useExtLED Uint8，是否针对附加的LED进行设置(只针对测量模块)
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StartStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index;
    Uint32 targetAD;
    Uint8 useExtLED = FALSE;

    if (sizeof(index) + sizeof(targetAD) + sizeof(useExtLED) == len)
    {
        memcpy(&index, data, sizeof(index));
        memcpy(&targetAD, data+sizeof(index), sizeof(targetAD));
        memcpy(&useExtLED, data+sizeof(index)+sizeof(targetAD), sizeof(useExtLED));

        if (TRUE == StaticADControl_Start(index, targetAD, useExtLED))
        {
            StaticADControl_SendEventOpen();
        }
        else
        {
            ret = DSCP_ERROR;
        }
    }
    else if(sizeof(index) + sizeof(targetAD)== len)  //兼容旧版命令
    {
        memcpy(&index, data, sizeof(index));
        memcpy(&targetAD, data+sizeof(index), sizeof(targetAD));

        if (TRUE == StaticADControl_Start(index, targetAD, FALSE))
        {
            StaticADControl_SendEventOpen();
        }
        else
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止采集PD电位器调节静态AD并关闭对应LED。
 * @details 停止静态AD调节过程
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_StopStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    StaticADControl_SendEventOpen();
    StaticADControl_Stop();
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询采集PD电位器默认静态AD控制参数。
 * @param index Uint8，目标PD索引号。
 * @param useExtLED Uint8，是否针对附加的LED进行设置(只针对测量模块)
 * @return value Uint16 ，静态AD调节参数。
 * @see DSCP_CMD_OAI_SET_STATIC_AD_CONTROL_PARAM
 */
void ExtOpticalAcquirer_GetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 value = DSCP_ERROR;
    Uint8 index = 0;
    Uint8 useExtLED = FALSE;

    if (sizeof(index) + sizeof(useExtLED) == len)
    {
        memcpy(&index, data, sizeof(index));
        memcpy(&useExtLED, data + sizeof(index), sizeof(useExtLED));
        value = StaticADControl_GetDefaultValue(index, useExtLED);
    }
    else if(0 == len)  //兼容旧版命令
    {
        memcpy(&index, data, sizeof(index));
        value = StaticADControl_GetDefaultValue(index, FALSE);
    }
    else
    {
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }


    DscpDevice_SendResp(dscp, &value, sizeof(value));
}

/**
 * @brief 设置采集PD电位器默认静态AD控制参数。
 * @details 静态AD控制器将根据设置的参数设置器件，并将参数永久保存在FLASH。
 * @details 静态AD调节状态中禁用。
 * @param index Uint8，目标PD索引号。
 * @param value Uint16 ，静态AD调节参数。
 * @param useExtLED Uint8，是否针对附加的LED进行设置(只针对测量模块)
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_SetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint16 size = 0;
    Uint8 index;
    Uint16 value;
    Uint8 useExtLED = FALSE;

    if (sizeof(index) + sizeof(value) + sizeof(useExtLED) == len)
    {
        memcpy(&index, data, sizeof(index));
        memcpy(&value, data+sizeof(index), sizeof(value));
        memcpy(&useExtLED, data+sizeof(index)+sizeof(value), sizeof(useExtLED));

        StaticADControl_SetDefaultValue(index, value, useExtLED);
        if(StaticADControl_SetRealValue(index, value) == FALSE)
        {
            ret = DSCP_ERROR;
        }
    }
    else if(sizeof(index) + sizeof(value) == len)  //兼容旧版命令
    {
        memcpy(&index, data, sizeof(index));
        memcpy(&value, data+sizeof(index), sizeof(value));

        StaticADControl_SetDefaultValue(index, value, FALSE);
        if(StaticADControl_SetRealValue(index, value) == FALSE)
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\OAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询采集PD电位器静态AD控制功能是否有效
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  功能有效，操作成功；
 *  - @ref DSCP_ERROR 功能无效，操作失败；
 */
void ExtOpticalAcquire_IsADControlValid(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_ERROR;
    if(TRUE == StaticADControl_IsValid())
    {
        ret = DSCP_OK;
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询测量模块LED默认光强输出电压。
 * @param index Uint8，测量模块LED索引号。
 * @return Float32，电压V：
 */
void ExtOpticalAcquirer_GetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float value = 0;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));

        value = OpticalLed_GetDefaultValue(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        value = OpticalLed_GetDefaultValue(0);
    }
    else
    {
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    TRACE_DEBUG("\n LED DAC default value: %f", value);
    DscpDevice_SendResp(dscp, &value, sizeof(value));
}

/**
 * @brief 设置测量模块LED默认光强输出电压。
 * @param value Float32，电压V。
 * @param index Uint8，测量模块LED索引号。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtOpticalAcquirer_SetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    float value = 0;
    Uint8 index = 0;

    if (sizeof(value) + sizeof(index) == len)
    {
        memcpy(&value, data, sizeof(value));
        memcpy(&index, data + sizeof(value), sizeof(index));

        if(FALSE == OpticalLed_SetDefaultValue(index, value))
        {
            ret = DSCP_ERROR_PARAM;
        }
    }
    else if(sizeof(value)  == len)  //兼容旧版命令
    {
        memcpy(&value, data, sizeof(value));

        if(FALSE == OpticalLed_SetDefaultValue(0, value))
        {
            ret = DSCP_ERROR_PARAM;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\nOAI Dscp Len Error , cur len : %d", len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

