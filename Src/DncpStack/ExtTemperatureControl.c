/*
 * ExtTemperatureControl.c
 *
 *  Created on: 2020年4月2日
 *      Author: Administrator
 */


/**
 * @file
 * @brief 温度接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#include <string.h>
#include <TempDriver/BoxFanDriver.h>
#include "ExtTemperatureControl.h"
#include "Common/Utils.h"
#include "DNCP/App/DscpSysDefine.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "Driver/TempDriver/BoxFanDriver.h"
#include "TemperatureControl/TempCollecterManager.h"
#include "TemperatureControl/ThermostatManager.h"
#include "LuipApi/ExtTemperatureControlInterface.h"
#include "FreeRTOS.h"

/**
 * @brief 查询温度传感器的校准系数。
 * @return 负输入分压，Float32。
 * @return 参考电压 ，Float32。
 * @return 校准电压 Float32。
 * @param index Uint8，要查询的恒温器索引。
 * @see DSCP_CMD_TCI_SET_CALIBRATE_FACTOR
 */
void ExtTempControl_GetCalibrateFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    TempCalibrateParam calibrateFactor;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        calibrateFactor = ThermostatManager_GetCalibrateFactor(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        calibrateFactor = ThermostatManager_GetCalibrateFactor(0);
    }
    else
    {
        TRACE_ERROR("\nGetCalibrateFactor Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }

    DscpDevice_SendResp(dscp, &calibrateFactor, sizeof(calibrateFactor));
}

/**
 * @brief 设置温度传感器的校准系数。
 * @details 因为个体温度传感器有差异，出厂或使用时需要校准。该参数将永久保存。
 *   @param negativeInput Float32，负输入分压 。
 *   @param vref Float32，参考电压。
 *   @param vcal Float32，校准电压。
 *   @param index Uint8，要设置的恒温器索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtTempControl_SetCalibrateFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    TempCalibrateParam calibratefactor;
    Uint8 index = 0;
    int size = sizeof(TempCalibrateParam) + sizeof(index);

    //设置数据正确性判断
    size = sizeof(TempCalibrateParam) + sizeof(index);
    if(len == size)
    {
        memcpy(&calibratefactor, data, sizeof(TempCalibrateParam));
        memcpy(&index, data + sizeof(TempCalibrateParam), sizeof(index));
        if (FALSE == ThermostatManager_SetCalibrateFactor(index, calibratefactor))
        {
            ret = DSCP_ERROR;
        }
    }
    else if(len == sizeof(TempCalibrateParam))  //兼容旧版命令
    {
        memcpy(&calibratefactor, data, sizeof(TempCalibrateParam));
        if (FALSE == ThermostatManager_SetCalibrateFactor(0, calibratefactor))
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n SetCalibrateFactor Parame Len Error , right len : %d, cur len : %d", size, len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询当前恒温器温度。
 * @param index Uint8，要查询的恒温器索引。
 * @return 当前恒温室温度，Float32。
 */
void ExtTempControl_GetTemperature(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float temp = 0;
    Uint8 index = 0;
    OldTemperature temperature;
    memset(&temperature, 0, sizeof(OldTemperature));

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        temp = ThermostatManager_GetCurrentTemp(index);

        // 发送回应
        DscpDevice_SendResp(dscp, (void *) &temp, sizeof(temp));
    }
    else if(0 == len)  //兼容旧版命令
    {
        temperature.thermostatTemp = ThermostatManager_GetCurrentTemp(0);
        temperature.environmentTemp = TempCollecterManager_GetEnvironmentTemp();

        // 发送回应
        DscpDevice_SendResp(dscp, (void *) &temperature, sizeof(temperature));
    }
    else
    {
        TRACE_ERROR("\nGetTemperature Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }
}

/**
 * @brief 查询恒温控制参数。
 * @param index Uint8，要查询的恒温器索引。
 * @return 恒温控制参数，格式如下：
 *  - proportion Float32，PID的比例系数。
 *  - integration Float32，PID的积分系数。
 *  - differential Float32，PID的微分系数。
 * @see DSCP_CMD_TCI_SET_THERMOSTAT_PARAM
 */
void ExtTempControl_GetThermostatParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index = 0;
    ThermostatParam thermostatparam;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        thermostatparam = ThermostatManager_GetPIDParam(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        thermostatparam = ThermostatManager_GetPIDParam(0);
    }
    else
    {
        TRACE_ERROR("\n GetThermostatParam Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }
    // 发送回应
    DscpDevice_SendResp(dscp, (void *) &thermostatparam, sizeof(thermostatparam));
}

/**
 * @brief 设置恒温控制参数。
 * @details 该参数将永久保存。系统启动时同步到当前恒温控制参数上。
 * @param proportion Float32，PID的比例系数。
 * @param integration Float32，PID的积分系数。
 * @param differential Float32，PID的微分系数。
 * @param index Uint8，要设置的恒温器索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtTempControl_SetThermostatParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    ThermostatParam thermostatparam;
    Uint8 index = 0;
    int size = sizeof(thermostatparam) + sizeof(index);

    if(len == size)
    {
        memcpy(&thermostatparam, data, sizeof(ThermostatParam));
        memcpy(&index, data + sizeof(ThermostatParam), sizeof(index));
        if (FALSE == ThermostatManager_SetPIDParam(index, thermostatparam))
        {
            ret = DSCP_ERROR;
        }
    }
    else if(len == sizeof(thermostatparam))  //兼容旧版命令
    {
        memcpy(&thermostatparam, data, sizeof(ThermostatParam));
        if (FALSE == ThermostatManager_SetPIDParam(0, thermostatparam))
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n SetThermostatParam Parame Len Error , right len : %d, cur len : %d", size, len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询恒温器的工作状态。
 * @param index Uint8，要查询的恒温器索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_IDLE 空闲；
 *  - @ref DSCP_BUSY 忙碌，需要停止后才能做下一个动作；
 */
void ExtTempControl_GetThermostatStatus(DscpDevice* dscp, Byte* data, Uint16 len)
{
    StatusCode statusCode = DSCP_BUSY;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        if (THERMOSTAT_IDLE == ThermostatManager_GetStatus(index))
        {
            statusCode = DSCP_IDLE;
        }
        else
        {
            statusCode = DSCP_BUSY;
        }
    }
    else if(0 == len)  //兼容旧版命令
    {
        if (THERMOSTAT_IDLE == ThermostatManager_GetStatus(0))
        {
            statusCode = DSCP_IDLE;
        }
        else
        {
            statusCode = DSCP_BUSY;
        }
    }
    else
    {
        TRACE_ERROR("\n GetThermostatParam Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }
    DscpDevice_SendStatus(dscp, statusCode);
}

/**
 * @brief 开始恒温。
 * @details 恒温开始后，系统将根据设定的恒温控制参数进行自动温度控制，尝试达到指定温度。
 *  不管成功与否，操作结果都将以事件的形式上传给上位机，但恒温器将继续工作，
 *  直到接收到DSCP_CMD_TCI_STOP_THERMOSTAT才停止。关联的事件有：
 *   - @ref DSCP_EVENT_TCI_THERMOSTAT_RESULT
 * @param mode Uint8，恒温模式，支持的模式见： @ref ThermostatMode 。
 * @param targetTemp Float32，恒温目标温度。
 * @param toleranceTemp Float32，容差温度，与目标温度的差值在该参数范围内即认为到达目标温度。
 * @param timeout Float32，超时时间，单位为秒。超时仍未达到目标温度，也将返回结果事件。
 * @param index Uint8，要操作的恒温器索引。
 * @param isExpectEvent bool,是否等待事件。
 * @return 状态回应，Uint16，支持的状态有：
 *   - @ref DSCP_OK  操作成功；
 *   - @ref DSCP_BUSY 操作失败，如恒温已经工作，需要先停止；
 *   - @ref DSCP_ERROR_PARAM 恒温参数错误；
 *   - @ref DSCP_ERROR 温度传感器异常；
 * @note 该命令将立即返回，恒温完成将以事件的形式上报。
 */
void ExtTempControl_StartThermostat(DscpDevice* dscp, Byte* data, Uint16 len)
{
    ThermostatMode mode;         //恒温
    float targetTemp;           // 目标温度
    float toleranceTemp;        // 容差
    float timeout;              // 超时时间
    Uint8 index = 0;
    Bool isExpectEvent;

    int size = 0;
    unsigned short ret = DSCP_OK;
    //设置数据正确性判断
    size =  sizeof(ThermostatMode) + sizeof(float) * 3 + sizeof(index) + sizeof(Bool);

    if(len == size)
    {
        memcpy(&mode, data, sizeof(ThermostatMode));
        memcpy(&targetTemp, data + sizeof(ThermostatMode), sizeof(float));
        memcpy(&toleranceTemp, data + sizeof(ThermostatMode) + sizeof(float) , sizeof(float));
        memcpy(&timeout, data + sizeof(ThermostatMode) + 2*sizeof(float), sizeof(float));
        memcpy(&index, data + sizeof(ThermostatMode) + 3*sizeof(float), sizeof(index));
        memcpy(&isExpectEvent, data + sizeof(ThermostatMode) + 3*sizeof(float) + sizeof(index), sizeof(Bool));

        if (TRUE == isExpectEvent)
        {
            ThermostatManager_SendEventOpen(index);
        }
        else
        {
            ThermostatManager_SendEventClose(index);
        }
        ret = (Uint16)ThermostatManager_Start(index, mode, targetTemp, toleranceTemp, timeout);
    }
    else if(len == (sizeof(ThermostatMode) + sizeof(float) * 3))  //兼容旧版命令
    {
        memcpy(&mode, data, sizeof(ThermostatMode));
        memcpy(&targetTemp, data + sizeof(ThermostatMode), sizeof(float));
        memcpy(&toleranceTemp, data + sizeof(ThermostatMode) + sizeof(float) , sizeof(float));
        memcpy(&timeout, data + sizeof(ThermostatMode) + 2*sizeof(float), sizeof(float));

        ThermostatManager_SendEventOpen(0);
        ret = (Uint16)ThermostatManager_Start(index, mode, targetTemp, toleranceTemp, timeout);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n StartThermostat Parame Len Error , right len : %d, cur len : %d", size, len);
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止恒温控制。
 * @details 停止后，加热器和冷却器将不工作。
 * @param index Uint8，要操作的恒温器索引。
* @param isExpectEvent bool,是否等待事件。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 * @note 该命令将立即返回。
 */
void ExtTempControl_StopThermostat(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_ERROR;
    Uint8 index = 0;
    Bool isExpectEvent;

    int size = sizeof(index) + sizeof(Bool);
    if (size == len)
    {
        memcpy(&index, data, sizeof(index));
        memcpy(&isExpectEvent, data + sizeof(index), sizeof(Bool));

        if (TRUE == isExpectEvent)
        {
            ThermostatManager_SendEventOpen(index);
        }
        else
        {
            ThermostatManager_SendEventClose(index);
        }
        if(TRUE == ThermostatManager_RequestStop(index))
        {
            ret = DSCP_OK;
        }
    }
    else if(0 == len)  //兼容旧版命令
    {
        ThermostatManager_SendEventOpen(0);
        if(TRUE == ThermostatManager_RequestStop(0))
        {
            ret = DSCP_OK;
        }
    }
    else
    {
        TRACE_ERROR("\n StopThermostat Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置温度上报周期。
 * @details 系统将根据设定的周期，定时向上发出温度上报事件。
 * @param period Float32，温度上报周期，单位为秒。0表示不需要上报，默认为0。
 * @see DSCP_EVENT_TCI_TEMPERATURE_NOTICE
 * @note 所设置的上报周期将在下一次启动时丢失，默认为0，不上报。
 */
void ExtTempControl_SetTemperatureNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Float32 period;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(period);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n SetTemperatureNotifyPeriod Parame Len Error , right len : %d, cur len : %d", size, len);
    }
    //修改并保存
    if (DSCP_OK == ret)
    {
        //修改并保存
        memcpy(&period, data, sizeof(period));
        ThermostatManager_SetTempReportPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置机箱风扇。
 * @details 根据设定的占空比，调节风扇速率
 * @param level float ,风扇速率，对应高电平占空比。默认为0，机箱风扇关闭。
 */
void ExtTempControl_BoxFanSetOutput(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float level= 0.0;
    int size = 0;
    unsigned short ret = DSCP_OK;   // 默认系统正常

    size = sizeof(level);
    if(len == size)
    {
        memcpy(&level, data, sizeof(float));
        TRACE_INFO("\n BoxFanDriver_SetOutput:");
        System_PrintfFloat(TRACE_LEVEL_INFO, level * 100, 3);
        TRACE_INFO(" %%");
        BoxFanDriver_SetOutput(level);
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n BoxFanSetOutput Parame Len Error , right len : %d, cur len : %d", size, len);
    }

    DscpDevice_SendStatus(dscp, ret);// 发送状态回应
}

/**
 * @brief 设置恒温器风扇。
 * @details 根据设定的占空比，调节风扇速率
 * @param level float ,风扇速率，对应高电平占空比。默认为0，恒温器风扇关闭。
 * @param thermostatIndex Uint8，要设置的恒温器索引。
 * @param fanIndex Uint8，要设置的恒温器中的风扇索引。
 */
void ExtTempControl_ThermostatFanSetOutput(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    float level= 0.0;
    Uint8 thermostatIndex = 0;
    Uint8 fanIndex = 0;
    int size = sizeof(float) + sizeof(Uint8) * 2;

    if(len == size)
    {
        memcpy(&level, data, sizeof(float));
        memcpy(&thermostatIndex, data + sizeof(float), sizeof(thermostatIndex));
        memcpy(&fanIndex, data + sizeof(float) + sizeof(Uint8), sizeof(fanIndex));

        TRACE_INFO("\n %s Device %d SetOutput :", ThermostatManager_GetName(thermostatIndex), fanIndex);
        System_PrintfFloat(TRACE_LEVEL_INFO, level * 100, 5);
        TRACE_INFO(" %%");
        if (FALSE == ThermostatManager_SetSingleRefrigeratorOutput(thermostatIndex, fanIndex, level))
        {
            ret = DSCP_ERROR;
        }
    }
    else if(len == sizeof(float))   //兼容旧版命令
    {
        memcpy(&level, data, sizeof(float));
        if (FALSE == ThermostatManager_SetSingleRefrigeratorOutput(0, 0, level))
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n ThermostatFanSetOutput Parame Len Error , right len : %d, cur len : %d", size, len);
    }

    DscpDevice_SendStatus(dscp, ret);// 发送状态回应
}

/**
 * @brief 获取恒温器加热丝输出的最大占空比。
 * @param maxDutyCycle float ,加热丝输出的最大占空比
 * @param index Uint8，要获取的恒温器索引。
 * @see DSCP_CMD_TCI_SET_HEATER_MAX_DUTY_CYCLE
 * @note
 */
void ExtTempControl_GetHeaterMaxDutyCycle(DscpDevice* dscp, Byte* data,Uint16 len)
{
    Uint8 index = 0;
    float value;

    if (sizeof(index) == len)
    {
       memcpy(&index, data, sizeof(index));
       value = ThermostatManager_GetHeaterMaxDutyCycle(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        value = ThermostatManager_GetHeaterMaxDutyCycle(0);
    }
    else
    {
        TRACE_ERROR("\n GetHeaterMaxDutyCycle Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }
    DscpDevice_SendResp(dscp, (void *) &value, sizeof(value));
}

/**
 * @brief 设置恒温器加热丝输出的最大占空比。
 * @param maxDutyCycle float，加热丝输出的最大占空比
 * @param index Uint8，要设置的恒温器索引。
 * @see DSCP_CMD_TCI_GET_HEATER_MAX_DUTY_CYCLE
 */
void ExtTempControl_SetHeaterMaxDutyCycle(DscpDevice* dscp, Byte* data,Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint8 index = 0;
    float value;
    int size = 0;

    size = sizeof(value) + sizeof(index);
    if(len == size)
    {
        memcpy(&value, data, sizeof(float));
        memcpy(&index, data + sizeof(float), sizeof(index));

        if (FALSE == ThermostatManager_SetMaxDutyCycle(index, value))
        {
            ret = DSCP_ERROR;
        }
    }
    else if(len == sizeof(float))  //兼容旧版命令
    {
        memcpy(&value, data, sizeof(float));

        if (FALSE == ThermostatManager_SetMaxDutyCycle(0, value))
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("\n SetHeaterMaxDutyCycle Parame Len Error , right len : %d, cur len : %d", size, len);
    }

    DscpDevice_SendStatus(dscp, ret);// 发送状态回应
}

/**
 * @brief 查询恒温器当前恒温控制参数。
 * @param index Uint8，要查询的恒温器索引。
 * @return 恒温控制参数，格式如下：
 *  - proportion Float32，PID的比例系数。
 *  - integration Float32，PID的积分系数。
 *  - differential Float32，PID的微分系数。
 * @see DSCP_CMD_TCI_SET_CURRENT_THERMOSTAT_PARAM
 */
void ExtTempControl_GetCurrentThermostatParam(DscpDevice* dscp, Byte* data,Uint16 len)
{
    ThermostatParam thermostatparam;
    Uint8 index = 0;

    if (sizeof(index) == len)
    {
        memcpy(&index, data, sizeof(index));
        thermostatparam = ThermostatManager_GetCurrentPIDParam(index);
    }
    else if(0 == len)  //兼容旧版命令
    {
        thermostatparam = ThermostatManager_GetCurrentPIDParam(0);
    }
    else
    {
        TRACE_ERROR("\n GetCurrentThermostatParam Parame Len Error , right len : %d, cur len : %d", sizeof(index), len);
    }
    DscpDevice_SendResp(dscp, (void *) &thermostatparam, sizeof(thermostatparam));
}

/**
 * @brief 设置当前恒温控制参数。
 * @details 恒温系统将根据设置的参数进行温度调节。此参数上电时获取FLASH的PID。
 * @param proportion Float32，PID的比例系数。
 * @param integration Float32，PID的积分系数。
 * @param differential Float32，PID的微分系数。
 * @param index Uint8，要设置的恒温器索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void ExtTempControl_SetCurrentThermostatParam(DscpDevice* dscp, Byte* data,Uint16 len)
{
    Uint16 ret = DSCP_OK;
    ThermostatParam thermostatparam;
    Uint8 index = 0;
    int size = 0;

    size = sizeof(ThermostatParam) + sizeof(index);
    if(len == size)
    {
        memcpy(&thermostatparam, data, sizeof(ThermostatParam));
        memcpy(&index, data + sizeof(ThermostatParam), sizeof(index));
        ThermostatManager_SetCurrentPIDParam(index, thermostatparam);
    }
    else if(len == sizeof(ThermostatParam))  //兼容旧版命令
    {
        memcpy(&thermostatparam, data, sizeof(ThermostatParam));
        ThermostatManager_SetCurrentPIDParam(0, thermostatparam);
    }
    else
    {
        ret = DSCP_ERROR;
    }

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 开启继电器
 * @note 该命令已不支持
 */
void ExtTempControl_TurnOnRelay(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 关闭继电器
 * @note 该命令已不支持
 */
void ExtTempControl_TurnOffRelay(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询系统支持的恒温器数目。
 * @return 总数目， Uint16。
 */
void ExtTempControl_GetTotalThermostat(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 number = TOTAL_THERMOSTAT;
    TRACE_INFO("\n GetTotalThermostat %d", number);
    DscpDevice_SendResp(dscp, (void *) &number, sizeof(number));
}

/**
 * @brief 查询当前环境温度。
 * @return 当前环境温度，Float32。
 */
void ExtTempControl_GetEnvTemperature(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float temp = TempCollecterManager_GetEnvironmentTemp();
    TRACE_INFO("\n EnvironmentTemp : ");
    System_PrintfFloat(TRACE_LEVEL_INFO, temp, 1);
    DscpDevice_SendResp(dscp, (void *) &temp, sizeof(temp));
}

