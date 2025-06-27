/**
 * @file
 * @brief 温度接口实现头文件
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */


#ifndef SRC_DNCPSTACK_EXTTEMPERATURECONTROL_H_
#define SRC_DNCPSTACK_EXTTEMPERATURECONTROL_H_

#include "LuipApi/ExtTemperatureControlInterface.h"
#include "DNCP/App/DscpDevice.h"

#ifdef __cplusplus
extern "C" {
#endif

void ExtTemperatureControl_Init(void);
void ExtTempControl_GetCalibrateFactor(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_SetCalibrateFactor(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_GetTemperature(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_GetThermostatParam(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_SetThermostatParam(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_GetThermostatStatus(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_StartThermostat(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_StopThermostat(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_SetTemperatureNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_BoxFanSetOutput(DscpDevice* dscp, Byte* data,Uint16 len);
void ExtTempControl_ThermostatFanSetOutput(DscpDevice* dscp, Byte* data,Uint16 len);
void ExtTempControl_GetHeaterMaxDutyCycle(DscpDevice* dscp, Byte* data,Uint16 len);
void ExtTempControl_SetHeaterMaxDutyCycle(DscpDevice* dscp, Byte* data,Uint16 len);
void ExtTempControl_GetCurrentThermostatParam(DscpDevice* dscp, Byte* data,Uint16 len);
void ExtTempControl_SetCurrentThermostatParam(DscpDevice* dscp, Byte* data,Uint16 len);
void ExtTempControl_TurnOnRelay(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_TurnOffRelay(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_GetTotalThermostat(DscpDevice* dscp, Byte* data, Uint16 len);
void ExtTempControl_GetEnvTemperature(DscpDevice* dscp, Byte* data, Uint16 len);

#define CMD_TABLE_EXTTEMPERATURECONTROL \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_CALIBRATE_FACTOR, ExtTempControl_GetCalibrateFactor), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_SET_CALIBRATE_FACTOR, ExtTempControl_SetCalibrateFactor), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_TEMPERATURE, ExtTempControl_GetTemperature), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_THERMOSTAT_PARAM, ExtTempControl_GetThermostatParam), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_SET_THERMOSTAT_PARAM, ExtTempControl_SetThermostatParam),\
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_THERMOSTAT_STATUS, ExtTempControl_GetThermostatStatus), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_START_THERMOSTAT, ExtTempControl_StartThermostat), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_STOP_THERMOSTAT, ExtTempControl_StopThermostat), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_SET_TEMPERATURE_NOTIFY_PERIOD, ExtTempControl_SetTemperatureNotifyPeriod), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_TURN_BOXFAN, ExtTempControl_BoxFanSetOutput), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_TURN_THERMOSTAT_FAN, ExtTempControl_ThermostatFanSetOutput), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_HEATER_MAX_DUTY_CYCLE, ExtTempControl_GetHeaterMaxDutyCycle), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_SET_HEATER_MAX_DUTY_CYCLE, ExtTempControl_SetHeaterMaxDutyCycle), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_CURRENT_THERMOSTAT_PARAM, ExtTempControl_GetCurrentThermostatParam), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_SET_CURRENT_THERMOSTAT_PARAM, ExtTempControl_SetCurrentThermostatParam), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_TURN_ON_RELAY, ExtTempControl_TurnOnRelay), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_TURN_OFF_RELAY, ExtTempControl_TurnOffRelay), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_TOTAL_THERMOSTAT, ExtTempControl_GetTotalThermostat), \
    DSCP_CMD_ENTRY(DSCP_CMD_ETCI_GET_ENV_TEMPERATURE, ExtTempControl_GetEnvTemperature)

#ifdef __cplusplus
}
#endif

#endif /* SRC_DNCPSTACK_EXTTEMPERATURECONTROL_H_ */
