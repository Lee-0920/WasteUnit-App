/**
 * @file
 * @brief 光学采集接口实现头文件
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#ifndef SRC_DNCPSTACK_EXTOPTICALACQUIRER_H_
#define SRC_DNCPSTACK_EXTOPTICALACQUIRER_H_

#include "LuipApi/ExtOpticalAcquireInterface.h"
#include "DNCP/App/DscpDevice.h"

#ifdef __cplusplus
extern "C" {
#endif

void ExtOpticalAcquirer_Init(void);
void ExtOpticalAcquirer_TurnOnLed(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_SetSignalADNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StartAcquirer(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StopAcquirer(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StartLEDController(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StopLEDController(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_GetLEDControllerTarget(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_SetLEDControllerTarget(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_GetLEDControllerParam(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_SetLEDControllerParam(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StartLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StopLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StartStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_StopStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_GetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_SetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquire_IsADControlValid(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_GetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtOpticalAcquirer_SetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len);

//命令入口，全局宏定义：每一条命令对应一个命令码和处理函数
#define CMD_TABLE_EXTOPTICALACQUIRE \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_TURN_ON_LED, ExtOpticalAcquirer_TurnOnLed), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_SET_SIGNAL_AD_NOTIFY_PERIOD, ExtOpticalAcquirer_SetSignalADNotifyPeriod), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_START_ACQUIRER, ExtOpticalAcquirer_StartAcquirer), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_STOP_ACQUIRER, ExtOpticalAcquirer_StopAcquirer), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_START_LEDCONTROLLER, ExtOpticalAcquirer_StartLEDController), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_STOP_LEDCONTROLLER, ExtOpticalAcquirer_StopLEDController), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_GET_LEDCONTROLLER_TARGET, ExtOpticalAcquirer_GetLEDControllerTarget), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_SET_LEDCONTROLLER_TARGET, ExtOpticalAcquirer_SetLEDControllerTarget), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_GET_LEDCONTROLLER_PARAM, ExtOpticalAcquirer_GetLEDControllerParam), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_SET_LEDCONTROLLER_PARAM, ExtOpticalAcquirer_SetLEDControllerParam),\
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_START_LEDADJUST, ExtOpticalAcquirer_StartLEDAdjust), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_STOP_LEDADJUST, ExtOpticalAcquirer_StopLEDAdjust), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_START_STATIC_AD_CONTROL, ExtOpticalAcquirer_StartStaticADControl), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_STOP_STATIC_AD_CONTROL, ExtOpticalAcquirer_StopStaticADControl), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_GET_STATIC_AD_CONTROL_PARAM, ExtOpticalAcquirer_GetStaticADControlParam), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_SET_STATIC_AD_CONTROL_PARAM, ExtOpticalAcquirer_SetStaticADControlParam), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_IS_STATIC_AD_CONTROL_VALID, ExtOpticalAcquire_IsADControlValid), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_GET_LED_DEFAULT_VALUE, ExtOpticalAcquirer_GetLEDDefaultValue), \
        DSCP_CMD_ENTRY(DSCP_CMD_EOAI_SET_LED_DEFAULT_VALUE, ExtOpticalAcquirer_SetLEDDefaultValue)

#ifdef __cplusplus
}
#endif

#endif /* SRC_DNCPSTACK_EXTOPTICALACQUIRER_H_ */
